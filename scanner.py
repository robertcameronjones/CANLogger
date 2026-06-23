"""
scanner.py — Active vehicle interrogation for CAN Logger.

Session flow
────────────
Phase 1  PREAMBLE
    Simultaneously broadcast J1979 PID 00 on both 07DF (11-bit) and
    18DB33F1 (29-bit). Wait 600 ms. Whichever addressing mode gets a
    response is used for the rest of the session.

Phase 2  VIN  (always first — gives hints for everything else)
    Try in layer order from fetch_list.json:
      J1979  → Mode 09 PID 02  (broadcast, ISO-TP multi-frame)
      J1979-2 → DID F802       (broadcast, ISO-TP multi-frame)
      prop    → physical ECU   (e.g. Stellantis 18DA60F1 DID 22F1)
    ISO-TP flow control is sent immediately by the existing
    _ISOTPReassembler — no delay added here.

Phase 3  VIN LOOKUP
    Match decoded VIN against vehicle_profiles.json wildcard patterns.
    Load protocol hints and proprietary ECU addresses.

Phase 4  SIGNAL LOOP  (1 s between each request)
    Engine Run Time → RPM → HVB SoC → Odometer → repeat.
    Each signal tries its layers in order (J1979 → J1979-2 → prop)
    and locks onto the first layer that responds.  Subsequent loops
    use only the proven layer — no re-discovery overhead.

Proprietary signals (e.g. Stellantis DID 7010 for HVB SoC) are
registered into the shared _lib_by_canid dict at runtime so the
existing _CANListener decoder handles them automatically.

Dependencies are injected via init() to avoid circular imports.
"""

import asyncio
import fnmatch
import json
import time
from pathlib import Path

import can

# ── Paths ─────────────────────────────────────────────────────────────────────
_DIR             = Path(__file__).parent
PROFILES_PATH    = _DIR / "vehicle_profiles.json"
FETCH_LIST_PATH  = _DIR / "fetch_list.json"

# ── Injected server state (set once via init()) ───────────────────────────────
_state     = None   # server _state dict
_signals   = None   # server _signals dict
_sig_lock  = None   # server _signals_lock (threading.Lock)
_can_ids   = None   # server _can_ids dict
_cid_lock  = None   # server _can_ids_lock (threading.Lock)
_lib       = None   # server _lib_by_canid dict (modified in-place)
_broadcast = None   # server _broadcast(data: dict) function


def init(state, signals, sig_lock, can_ids, cid_lock,
         lib_by_canid, broadcast_fn):
    global _state, _signals, _sig_lock, _can_ids, _cid_lock, _lib, _broadcast
    _state     = state
    _signals   = signals
    _sig_lock  = sig_lock
    _can_ids   = can_ids
    _cid_lock  = cid_lock
    _lib       = lib_by_canid
    _broadcast = broadcast_fn


# ── Scanner state ─────────────────────────────────────────────────────────────
_scan: dict = {
    "running":    False,
    "phase":      "idle",   # preamble | vin | loop | idle
    "addressing": None,     # "11bit" | "29bit"
    "broadcast":  None,     # int: 0x07DF or 0x18DB33F1
    "is_ext":     False,
    "vin":        None,
    "profile":    None,     # matched vehicle_profiles.json entry
    "layers_ok":  {},       # signal key → layer index that first responded
    "layer_cursor": {},     # signal key → next layer index to try
    "log":        [],       # last 50 status lines
    "task":       None,     # asyncio.Task
}


def get_status() -> dict:
    return {k: v for k, v in _scan.items() if k != "task"}


# ── Logging ───────────────────────────────────────────────────────────────────
def _log(msg: str):
    ts   = time.strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    _scan["log"].append(line)
    if len(_scan["log"]) > 50:
        _scan["log"] = _scan["log"][-50:]
    if _broadcast:
        _broadcast({
            "type":       "scan_log",
            "msg":        line,
            "phase":      _scan["phase"],
            "vin":        _scan["vin"],
            "addressing": _scan["addressing"],
        })


# ── Config loaders ────────────────────────────────────────────────────────────
def _load_profiles() -> list:
    try:
        return json.loads(PROFILES_PATH.read_text())
    except Exception as e:
        _log(f"vehicle_profiles.json load error: {e}")
        return []


def _load_fetchlist() -> list:
    try:
        return json.loads(FETCH_LIST_PATH.read_text())
    except Exception as e:
        _log(f"fetch_list.json load error: {e}")
        return []


# ── VIN wildcard matching ─────────────────────────────────────────────────────
def _match(vin: str, pattern: str) -> bool:
    """
    Match VIN against a '|'-separated list of glob patterns.
    '*' = any chars, '%' = single char (SQL-style alias for '?').
    Trailing '*' is appended automatically for prefix-only patterns.
    """
    vin = vin.upper()
    for part in pattern.split("|"):
        p = part.strip().replace("%", "?").upper()
        if "*" not in p and "?" not in p:
            p += "*"
        if fnmatch.fnmatch(vin, p):
            return True
    return False


def _find_profile(vin: str, profiles: list) -> dict | None:
    for p in profiles:
        if _match(vin, p.get("vin_pattern", "")):
            return p
    return None


# ── CAN message builders ──────────────────────────────────────────────────────
def _pad8(data: list[int]) -> bytes:
    """Pad a list of ints to 8 bytes."""
    return bytes(data + [0x00] * (8 - len(data)))


def _j1979_req(service: str, pid: str, arb_id: int, is_ext: bool) -> can.Message:
    s = int(service, 16)
    p = int(pid, 16)
    return can.Message(
        arbitration_id=arb_id,
        data=_pad8([0x02, s, p]),
        is_extended_id=is_ext,
    )


def _uds_req(did: str, arb_id: int, is_ext: bool) -> can.Message:
    d = int(did, 16)
    return can.Message(
        arbitration_id=arb_id,
        data=_pad8([0x03, 0x22, (d >> 8) & 0xFF, d & 0xFF]),
        is_extended_id=is_ext,
    )


# ── Signal key helpers ────────────────────────────────────────────────────────
def _norm_id(hex_str: str) -> str:
    v = int(hex_str, 16)
    return f"{v:08X}" if v > 0x7FF else f"{v:04X}"


def _prop_sig_key(response_id: str, did: str) -> str:
    """
    Compute the _signals key that _CANListener generates for a
    proprietary UDS DID response, e.g.:
      response_id="18DAF144", did="7010" → "lib_18DAF144_62_7010"
    """
    d         = int(did, 16)
    pid_bytes = [(d >> 8) & 0xFF, d & 0xFF]
    norm      = _norm_id(response_id)
    pid_hex   = "".join(f"{b:02X}" for b in pid_bytes)
    return f"lib_{norm}_62_{pid_hex}"


# ── Proprietary signal registration ──────────────────────────────────────────
def _register_prop(layer: dict, signal_name: str):
    """
    Insert a proprietary UDS DID into _lib_by_canid so the existing
    _CANListener._decode_frame() decodes and broadcasts it automatically.
    Safe to call multiple times — won't double-register.
    """
    response_id = layer.get("response_id", "")
    did_str     = layer.get("did", "")
    if not response_id or not did_str:
        return

    norm = _norm_id(response_id)
    d    = int(did_str, 16)
    pid_bytes = [(d >> 8) & 0xFF, d & 0xFF]

    existing = _lib.get(norm, [])
    for e in existing:
        if e["pid_bytes"] == pid_bytes:
            return  # already registered

    entry = {
        "name":          layer.get("name", signal_name),
        "response_byte": 0x62,
        "pid_bytes":     pid_bytes,
        "nbytes":        layer.get("nbytes", 1),
        "mul":           float(layer.get("mul", 1.0)),
        "off":           float(layer.get("off", 0.0)),
        "unit":          layer.get("unit", ""),
        "is_string":     layer.get("is_string", False),
        "raw_hex":       layer.get("raw_hex", False),
        "byte_index":    layer.get("byte_index", None),
        "value_map":     layer.get("value_map", None),
        "vehicle_vin":   "",
        "make": "", "model": "", "year": "",
    }
    _lib.setdefault(norm, []).append(entry)
    _log(f"  Registered prop: {norm} DID {did_str} → {entry['name']}")


# ── TX helper — send and broadcast to UI log ─────────────────────────────────
def _send(bus, msg: can.Message, label: str = ""):
    """Send a CAN message and broadcast it to the UI frame log as a TX frame."""
    try:
        bus.send(msg)
    except can.CanOperationError as e:
        _log(f"  Bus error — stopping scan: {e}")
        _scan["running"] = False
        return
    except Exception as e:
        _log(f"  Send error: {e}")
        return
    arb_id   = msg.arbitration_id
    is_ext   = msg.is_extended_id
    id_str   = f"{arb_id:08X}" if is_ext else f"{arb_id:04X}"
    data_hex = " ".join(f"{b:02X}" for b in msg.data) if msg.data else ""
    t0       = (_state or {}).get("start_time") or 0.0
    elapsed  = round(time.time() - t0, 4) if t0 else 0.0

    # Write TX frame to the log file — use hardware-relative timestamp
    can_logger = (_state or {}).get("can_logger")
    if can_logger:
        try:
            t0_hw = (_state or {}).get("t0_hw") or 0.0
            start = (_state or {}).get("start_time") or time.time()
            msg.timestamp = t0_hw + (time.time() - start)
            can_logger(msg)
        except Exception:
            pass

    if _broadcast:
        _broadcast({
            "type":  "frame",
            "n":     "→",        # TX — not an RX message number
            "t":     elapsed,
            "id":    id_str,
            "xtd":   is_ext,
            "rtr":   False,
            "fd":    False,
            "err":   False,
            "dl":    msg.dlc,
            "data":  data_hex,
            "flags": f"TX {label}" if label else "TX",
        })


# ── Wait for a signal key ─────────────────────────────────────────────────────
async def _wait_keys(keys: list[str], timeout: float) -> str | None:
    """
    Poll _signals until one of 'keys' appears, or timeout expires.
    Keys should be cleared by the caller before sending the request
    so a stale value from a previous cycle doesn't produce a false hit.
    Returns the key found, or None on timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with _sig_lock:
            for k in keys:
                if k in _signals:
                    return k
        await asyncio.sleep(0.05)
    return None


# ── Phase 1 — Preamble (addressing discovery + VIN burst) ────────────────────
async def _preamble(bus) -> bool:
    """
    Single burst sent to both 11-bit and 29-bit simultaneously:
      1. PID 00  — determines addressing (whichever ECU range responds)
      2. Mode 09 PID 02 — J1979 VIN request
      3. DID F802       — J1979-2 VIN request (fallback)

    All three requests go out immediately, back-to-back.
    ISO-TP Flow Control for any VIN First Frame is sent by the
    reassembler the instant the FF arrives — no slot boundary.

    We then wait 1.5 s for all responses (long enough for 3-frame VIN).
    If VIN still hasn't arrived we continue anyway — the signal loop
    will keep requesting it as part of the normal cycle.
    """
    _scan["phase"] = "preamble"
    _log("Preamble burst: PID 00 + UDS DID F190 (functional) + Stellantis physical")

    with _cid_lock:
        ids_before = set(_can_ids.keys())
    with _sig_lock:
        _signals.pop("vin", None)

    # Register prop VIN decoder so the listener can decode it if it responds
    # (needed before any frames arrive)
    vin_prop_keys = []

    # ── Burst — PID 00 + UDS VIN functional + Stellantis physical ────────────
    burst = [
        # OBD-II PID 00 — discovers 11-bit / 29-bit OBD-II ECUs
        (_j1979_req("01", "00",  0x07DF,      False), "07DF  PID 00"),
        (_j1979_req("01", "00",  0x18DB33F1,  True),  "18DB33F1  PID 00"),
        # UDS DID F190 functional — works on Mercedes, BMW, VAG, any UDS ECU
        (_uds_req  ("F190",      0x07DF,      False), "07DF  UDS DID F190  (VIN)"),
        (_uds_req  ("F190",      0x18DB33F1,  True),  "18DB33F1  UDS DID F190  (VIN)"),
        # Stellantis physical — 29-bit slot 0x60
        (_uds_req  ("F1B0",      0x18DA60F1,  True),  "18DA60F1  DID F1B0  (VIN)"),
        (_uds_req  ("F190",      0x18DA60F1,  True),  "18DA60F1  DID F190  (VIN)"),
        # Mercedes physical — slots seen responding (0x58, 0x59) + common gateway (0x10, 0x00)
        (_uds_req  ("F190",      0x18DA58F1,  True),  "18DA58F1  DID F190  (VIN)"),
        (_uds_req  ("F190",      0x18DA59F1,  True),  "18DA59F1  DID F190  (VIN)"),
        (_uds_req  ("F190",      0x18DA10F1,  True),  "18DA10F1  DID F190  (VIN)"),
        (_uds_req  ("F190",      0x18DA00F1,  True),  "18DA00F1  DID F190  (VIN)"),
    ]

    for msg, label in burst:
        _send(bus, msg, label)
        _log(f"  → {label}")
        await asyncio.sleep(1.0)

    # Wait for addressing + VIN responses (1.5 s covers 3-frame VIN reassembly)
    await asyncio.sleep(1.5)

    # ── Determine addressing ───────────────────────────────────────────────────
    with _cid_lock:
        new_ids = set(_can_ids.keys()) - ids_before

    _log(f"  New ECUs: {', '.join(sorted(new_ids)) or 'none'}")

    has_29 = any(s.startswith("18DA") for s in new_ids)
    has_11 = any(s.startswith("07E")  for s in new_ids)

    if has_29:
        _scan.update(addressing="29bit", broadcast=0x18DB33F1, is_ext=True)
        _log("  → 29-bit addressing confirmed")
    elif has_11:
        _scan.update(addressing="11bit", broadcast=0x07DF, is_ext=False)
        _log("  → 11-bit addressing confirmed")
    else:
        _log("  → No response — check ignition and connection. Scan stopped.")
        return False

    # ── Check if VIN already arrived ──────────────────────────────────────────
    with _sig_lock:
        vin = _signals.get("vin", {}).get("value", "")
    if vin and len(vin) >= 5:
        _log(f"  VIN from burst: {vin}")
        _scan["vin"] = vin
    else:
        _log("  VIN not in burst — will retry in signal loop")

    return True


# ── Phase 2 — VIN fetch (retry if preamble burst missed it) ──────────────────
async def _fetch_vin(bus, fetchlist: list) -> str | None:
    """
    Called after preamble. If VIN was already captured in the burst,
    returns immediately. Otherwise tries proprietary physical addresses
    (e.g. Stellantis 18DA60F1 DID 22F1) that can't be broadcast.
    """
    # Already got it in the preamble burst?
    with _sig_lock:
        vin = _signals.get("vin", {}).get("value", "")
    if vin and len(vin) >= 5:
        return vin

    _scan["phase"] = "vin"
    _log("VIN not from burst — trying proprietary physical addresses...")

    vin_sig = next((s for s in fetchlist if s["key"] == "vin"), None)
    if not vin_sig:
        return None

    for layer in vin_sig.get("layers", []):
        if layer["protocol"] != "prop":
            continue   # broadcast layers already tried in preamble

        req_id    = layer.get("request_id", "")
        did       = layer.get("did", "")
        resp_id   = layer.get("response_id", "")
        layer_ext = layer.get("is_ext", True)

        _log(f"  Trying prop {req_id} DID {did}...")
        _register_prop(layer, "VIN")

        sig_key = _prop_sig_key(resp_id, did) if resp_id else None
        if sig_key:
            with _sig_lock:
                _signals.pop(sig_key, None)

        _send(bus, _uds_req(did, int(req_id, 16), layer_ext), f"VIN prop {did}")

        if sig_key:
            found = await _wait_keys([sig_key], timeout=1.5)
            if found:
                val = _signals.get(sig_key, {}).get("value", "")
                vin = str(val) if val else ""
                if len(vin) >= 5:
                    _log(f"  VIN (prop {did}): {vin}")
                    with _sig_lock:
                        entry = dict(_signals[sig_key])
                        entry["name"] = "VIN"
                        _signals["vin"] = entry
                    _broadcast({
                        "type": "signal", "pid": "vin", "name": "VIN",
                        "unit": "", "value": vin,
                        "min": None, "max": None,
                        "can_id": _signals.get(sig_key, {}).get("can_id", ""),
                    })
                    return vin

        await asyncio.sleep(1.0)

    _log("  VIN: no response from any layer")
    return None


# ── Phase 4 — Signal loop ─────────────────────────────────────────────────────
async def _signal_loop(bus, fetchlist: list):
    _scan["phase"] = "loop"
    bcast   = _scan["broadcast"]
    is_ext  = _scan["is_ext"]
    vin     = _scan["vin"]

    poll_sigs = [s for s in fetchlist
                 if not s.get("one_shot") and s.get("enabled", True)]

    _log(f"Signal loop: {[s['name'] for s in poll_sigs]}")

    while _scan["running"]:
        for sig in poll_sigs:
            if not _scan["running"]:
                break

            key        = sig["key"]
            name       = sig["name"]
            layers     = sig.get("layers", [])
            ok_idx = _scan["layers_ok"].get(key)

            # Pick exactly one layer to send this cycle.
            # If a working layer is known, use it. Otherwise use the next
            # candidate (tracked per-signal so we rotate on each miss).
            if ok_idx is not None:
                idx, layer = ok_idx, layers[ok_idx]
            else:
                next_idx = _scan["layer_cursor"].get(key, 0) % len(layers)
                idx, layer = next_idx, layers[next_idx]

            proto    = layer["protocol"]
            sig_keys = []
            sent     = False

            # ── J1979 ─────────────────────────────────────────────────────────
            if proto == "J1979":
                pid      = layer["pid"]
                sig_keys = [layer.get("signal_key", f"j1979_{pid.upper()}")]
                with _sig_lock:
                    for k in sig_keys: _signals.pop(k, None)
                _send(bus, _j1979_req("01", pid, bcast, is_ext), f"{name} PID {pid}")
                sent = True

            # ── J1979-2 ───────────────────────────────────────────────────────
            elif proto == "J1979-2":
                did      = layer["did"]
                sig_keys = [layer.get("signal_key", f"j19792_{did.upper()}")]
                with _sig_lock:
                    for k in sig_keys: _signals.pop(k, None)
                _send(bus, _uds_req(did, bcast, is_ext), f"{name} DID {did}")
                sent = True

            # ── Proprietary ───────────────────────────────────────────────────
            elif proto == "prop":
                vin_pat = layer.get("vin_pattern", "")
                if vin_pat and not (vin and _match(vin, vin_pat)):
                    continue  # pattern specified but VIN doesn't match — skip
                if True:
                    req_id    = layer.get("request_id", "")
                    did       = layer.get("did", "")
                    resp_id   = layer.get("response_id", "")
                    layer_ext = layer.get("is_ext", True)
                    _register_prop(layer, name)
                    sig_key  = _prop_sig_key(resp_id, did) if resp_id else None
                    sig_keys = [sig_key] if sig_key else []
                    if sig_keys:
                        with _sig_lock:
                            for k in sig_keys: _signals.pop(k, None)
                    _send(bus, _uds_req(did, int(req_id, 16), layer_ext),
                          f"{name} prop DID {did}")
                    sent = True

            if sent and sig_keys:
                found = await _wait_keys(sig_keys, timeout=0.85)
                if found:
                    if ok_idx is None:
                        _scan["layers_ok"][key] = idx
                        _log(f"  {name}: layer {idx} ({proto}) locked ✓")
                    _scan["layer_cursor"][key] = idx   # stay on this one
                else:
                    if ok_idx is not None:
                        _log(f"  {name}: layer {idx} no response — unlocking")
                        _scan["layers_ok"].pop(key, None)
                    # Advance cursor to try next layer next cycle
                    _scan["layer_cursor"][key] = (idx + 1) % len(layers)

            # 1 s between each signal request
            await asyncio.sleep(1.0)


# ── Main scan task ────────────────────────────────────────────────────────────
async def _scan_task(bus):
    try:
        profiles  = _load_profiles()
        fetchlist = _load_fetchlist()

        # Phase 1 — Preamble (retry until ECU responds or scan is stopped)
        while _scan["running"]:
            ok = await _preamble(bus)
            if ok:
                break
            _log("  Waiting for vehicle — retrying in 5s...")
            _scan["phase"] = "waiting"
            await asyncio.sleep(5.0)
        if not _scan["running"]:
            return

        # Phase 2 — VIN
        vin = await _fetch_vin(bus, fetchlist)
        _scan["vin"] = vin

        # Phase 3 — Profile lookup
        if vin:
            prof = _find_profile(vin, profiles)
            _scan["profile"] = prof
            _log(f"Profile: {prof['name']}" if prof else "Profile: no match — using standard PIDs only")

        # Phase 4 — Signal loop
        await _signal_loop(bus, fetchlist)

    except asyncio.CancelledError:
        _log("Scan stopped.")
    except Exception as e:
        _log(f"Scan error: {e}")
        raise
    finally:
        _scan.update(running=False, phase="idle", task=None)


# ── ECU address sweep ─────────────────────────────────────────────────────────
# Common ECU slot addresses to probe (29-bit 18DAxxF1 → 18DAF1xx)
_SWEEP_ADDRS = [
    0x00, 0x01, 0x03, 0x07, 0x10, 0x11, 0x13, 0x14, 0x15, 0x17,
    0x18, 0x1A, 0x1C, 0x20, 0x21, 0x23, 0x28, 0x2B, 0x33, 0x36,
    0x3B, 0x40, 0x42, 0x44, 0x46, 0x47, 0x4C, 0x51, 0x52, 0x54,
    0x55, 0x58, 0x60, 0x6B, 0x76, 0x7A, 0x7F,
]

async def _sweep_task(bus):
    """Probe each 18DAxxF1 address with UDS DID F190 (VIN).
    Logs every address that responds (positive or negative)."""
    _scan.update(running=True, phase="sweep", log=[])
    _log("ECU sweep — probing 18DAxxF1 addresses with DID F190")
    _log(f"  {len(_SWEEP_ADDRS)} addresses × 1s = ~{len(_SWEEP_ADDRS)}s")
    found = []
    try:
        for ecu in _SWEEP_ADDRS:
            if not _scan["running"]:
                break
            req_id  = 0x18DA0000 | (ecu << 8) | 0xF1
            resp_id = 0x18DAF100 | ecu
            req_id_str  = f"{req_id:08X}"
            resp_id_str = f"{resp_id:08X}"

            _log(f"  → {req_id_str}  DID F190 (ECU slot 0x{ecu:02X})")

            with _cid_lock:
                had_resp = resp_id_str in _can_ids

            msg = _uds_req("F190", req_id, True)
            _send(bus, msg, f"sweep ECU 0x{ecu:02X}")

            await asyncio.sleep(1.0)

            with _cid_lock:
                now_resp = resp_id_str in _can_ids
            if now_resp and not had_resp:
                _log(f"  ✓ RESPONSE from {resp_id_str} (ECU 0x{ecu:02X})")
                found.append(resp_id_str)

        if found:
            _log(f"Sweep complete. Responding ECUs: {', '.join(found)}")
        else:
            _log("Sweep complete. No ECU responses detected.")
    except asyncio.CancelledError:
        _log("Sweep stopped.")
    finally:
        _scan.update(running=False, phase="idle", task=None)


def start_sweep(bus) -> bool:
    """Start ECU address sweep. Returns False if already running."""
    if _scan["running"]:
        return False
    task = asyncio.ensure_future(_sweep_task(bus))
    _scan["task"] = task
    return True


# ── Public API ────────────────────────────────────────────────────────────────
def start(bus) -> bool:
    """Start a scan session. Returns False if one is already running."""
    if _scan["running"]:
        return False
    _scan.update(
        running=True, phase="preamble",
        vin=None, profile=None,
        addressing=None, broadcast=None, is_ext=False,
        layers_ok={}, layer_cursor={}, log=[],
    )
    task = asyncio.ensure_future(_scan_task(bus))
    _scan["task"] = task
    _log("Scan session started.")
    return True


def stop():
    """Cancel the running scan task."""
    task = _scan.get("task")
    if task and not task.done():
        task.cancel()
    _scan.update(running=False, phase="idle")

#!/usr/bin/env python3
"""
CAN Logger — Web UI backend.
Serves index.html and streams CAN frames, CAN ID stats, and decoded signals
to the browser via WebSocket.

Signal decoding layers (in priority order):
  1. Vehicle-specific entries from signal_library.json (CAN Log Splitter)
  2. J1979-2 (SAE) — Mode 22 / UDS ReadDataByIdentifier (0x62 response)
  3. J1979   (SAE) — Mode 01 / OBD-II generic PIDs (0x41 response)

Usage:
    ./start.sh        ← recommended
    open http://localhost:8000
"""

import asyncio
import contextlib
import ctypes
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import can
from can import Bus, BusState, Logger, Notifier
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

# ──────────────────────────────────────────────────────────────────────────────
# J1979 (OBD-II Mode 01) generic formula table — fallback when no library entry
# matches the incoming ECU CAN ID.
# ──────────────────────────────────────────────────────────────────────────────

J1979_PIDS: dict[str, dict] = {
    "04": {"name": "Engine Load",        "nbytes": 1, "mul": 100/255,  "off": 0,    "unit": "%"},
    "05": {"name": "Coolant Temp",       "nbytes": 1, "mul": 1,        "off": -40,  "unit": "°C"},
    "06": {"name": "Fuel Trim B1 ST",    "nbytes": 1, "mul": 100/128,  "off": -100, "unit": "%"},
    "07": {"name": "Fuel Trim B1 LT",    "nbytes": 1, "mul": 100/128,  "off": -100, "unit": "%"},
    "0B": {"name": "Manifold Pressure",  "nbytes": 1, "mul": 1,        "off": 0,    "unit": "kPa"},
    "0C": {"name": "Engine RPM",         "nbytes": 2, "mul": 0.25,     "off": 0,    "unit": "rpm"},
    "0D": {"name": "Vehicle Speed",      "nbytes": 1, "mul": 1,        "off": 0,    "unit": "km/h"},
    "0E": {"name": "Timing Advance",     "nbytes": 1, "mul": 0.5,      "off": -64,  "unit": "°"},
    "0F": {"name": "Intake Air Temp",    "nbytes": 1, "mul": 1,        "off": -40,  "unit": "°C"},
    "10": {"name": "MAF Air Flow",       "nbytes": 2, "mul": 0.01,     "off": 0,    "unit": "g/s"},
    "11": {"name": "Throttle Position",  "nbytes": 1, "mul": 100/255,  "off": 0,    "unit": "%"},
    "1F": {"name": "Engine Run Time",    "nbytes": 2, "mul": 1,        "off": 0,    "unit": "s"},
    "21": {"name": "Dist w/ MIL",        "nbytes": 2, "mul": 1,        "off": 0,    "unit": "km"},
    "2F": {"name": "Fuel Level %",       "nbytes": 1, "mul": 100/255,  "off": 0,    "unit": "%"},
    "31": {"name": "Dist Since Clear",   "nbytes": 2, "mul": 1,        "off": 0,    "unit": "km"},
    "33": {"name": "Baro Pressure",      "nbytes": 1, "mul": 1,        "off": 0,    "unit": "kPa"},
    "42": {"name": "Control Voltage",    "nbytes": 2, "mul": 0.001,    "off": 0,    "unit": "V"},
    "46": {"name": "Ambient Temp",       "nbytes": 1, "mul": 1,        "off": -40,  "unit": "°C"},
    "5B": {"name": "HVB State of Charge","nbytes": 1, "mul": 100/255,  "off": 0,    "unit": "%"},
    "5C": {"name": "Engine Oil Temp",    "nbytes": 1, "mul": 1,        "off": -40,  "unit": "°C"},
    "A6": {"name": "Odometer",           "nbytes": 4, "mul": 0.1,      "off": 0,    "unit": "km"},
}

# J1979-2 (SAE J1979-2) standardised Mode 22 DIDs — F4xx mirrors Mode 01 PIDs.
# These are decoded the same way as J1979 but matched on 2-byte DID in a
# 0x62 response frame.
J1979_2_DIDS: dict[str, dict] = {
    "F40C": {"name": "Engine RPM (J1979-2)",        "nbytes": 2, "mul": 0.25,    "off": 0,   "unit": "rpm"},
    "F40D": {"name": "Vehicle Speed (J1979-2)",     "nbytes": 1, "mul": 1,       "off": 0,   "unit": "km/h"},
    "F40F": {"name": "Intake Air Temp (J1979-2)",   "nbytes": 1, "mul": 1,       "off": -40, "unit": "°C"},
    "F411": {"name": "Throttle Pos (J1979-2)",      "nbytes": 1, "mul": 100/255, "off": 0,   "unit": "%"},
    "F41F": {"name": "Engine Run Time (J1979-2)",   "nbytes": 2, "mul": 1,       "off": 0,   "unit": "s"},
    "F42F": {"name": "Fuel Level % (J1979-2)",      "nbytes": 1, "mul": 100/255, "off": 0,   "unit": "%"},
    "F4A6": {"name": "Odometer (J1979-2)",          "nbytes": 4, "mul": 0.1,     "off": 0,   "unit": "km"},
    "F805": {"name": "Coolant Temp (J1979-2)",      "nbytes": 1, "mul": 1,       "off": -40, "unit": "°C"},
}

# ──────────────────────────────────────────────────────────────────────────────
# Signal library — loaded from CAN Log Splitter's signal_library.json
# ──────────────────────────────────────────────────────────────────────────────

import os as _os

SIGNAL_LIB_PATHS = [p for p in [
    Path(_os.environ["SIGNAL_LIB"]) if "SIGNAL_LIB" in _os.environ else None,
    Path("/Users/robertjones/Documents/CAN Log Splitter/signal_library.json"),
    Path("/Users/robertjones/Documents/CAN Log Splitter/data/signal_library.json"),
    Path("signal_library.json"),   # drop a copy here for portability
] if p is not None]

# After loading: { norm_can_id → [matcher, ...] }
# Each matcher: {name, response_byte, pid_bytes, nbytes, mul, off, unit,
#                is_string, vehicle_vin, make, model, year}
_lib_by_canid: dict[str, list[dict]] = {}
_lib_path_used: str = ""
_lib_entry_count: int = 0


def _norm_can_id(hex_str: str) -> str:
    """Normalise a hex CAN ID string to the same format the listener emits."""
    try:
        v = int(hex_str.strip(), 16)
        return f"{v:08X}" if v > 0x7FF else f"{v:04X}"
    except ValueError:
        return hex_str.strip().upper()


def _parse_pid_did(pid_did_str: str) -> tuple[int, list[int]] | None:
    """
    Parse 'PID A6' → (0x41, [0xA6])
    Parse 'DID F4A6' → (0x62, [0xF4, 0xA6])
    Parse 'DID 1002' → (0x62, [0x10, 0x02])
    Returns (response_byte, pid_bytes) or None on failure.
    """
    parts = pid_did_str.strip().split()
    if len(parts) != 2:
        return None
    kind, hexval = parts[0].upper(), parts[1].upper()
    try:
        if kind == "PID":
            pid = int(hexval, 16)
            return (0x41, [pid])
        elif kind == "DID":
            did = int(hexval, 16)
            return (0x62, [(did >> 8) & 0xFF, did & 0xFF])
    except ValueError:
        pass
    return None


def _parse_nbytes(payload_bytes_str: str) -> int:
    """Count bytes in a space-separated hex string like '00 00 47 2B'."""
    s = payload_bytes_str.strip()
    if not s or s in ("—", "–", "-"):
        return 0
    return len(s.split())


def load_signal_library():
    global _lib_by_canid, _lib_path_used, _lib_entry_count
    _lib_by_canid.clear()
    loaded = []

    for p in SIGNAL_LIB_PATHS:
        if p.exists():
            try:
                with open(p, "r") as f:
                    loaded = json.load(f)
                _lib_path_used = str(p)
                break
            except Exception:
                continue

    count = 0
    for entry in loaded:
        can_id_ret = entry.get("can_id_return", "").strip()
        pid_did    = entry.get("pid_did", "").strip()
        unit       = entry.get("unit", "").strip()
        is_string  = unit.lower() == "string"

        parsed = _parse_pid_did(pid_did)
        if parsed is None:
            continue

        response_byte, pid_bytes = parsed
        nbytes = _parse_nbytes(entry.get("payload_bytes", ""))

        try:
            mul = float(entry.get("multiplier", 1))
        except (ValueError, TypeError):
            mul = 1.0
        try:
            off = float(entry.get("offset", 0))
        except (ValueError, TypeError):
            off = 0.0

        norm_id = _norm_can_id(can_id_ret)
        _lib_by_canid.setdefault(norm_id, []).append({
            "name":          entry.get("signal_name", "Unknown"),
            "response_byte": response_byte,
            "pid_bytes":     pid_bytes,
            "nbytes":        nbytes,
            "mul":           mul,
            "off":           off,
            "unit":          unit if not is_string else "",
            "is_string":     is_string,
            "vehicle_vin":   entry.get("vehicle_vin", ""),
            "make":          entry.get("make", ""),
            "model":         entry.get("model", ""),
            "year":          entry.get("year", ""),
        })
        count += 1

    _lib_entry_count = count
    print(f"  Signal library: {count} entries from {_lib_path_used or '(not found)'}")


# ──────────────────────────────────────────────────────────────────────────────
# Byte decoding helpers
# ──────────────────────────────────────────────────────────────────────────────

def _bytes_to_int(data: list[int], nbytes: int) -> int | None:
    if len(data) < nbytes or nbytes == 0:
        return None
    val = 0
    for i in range(nbytes):
        val = (val << 8) | data[i]
    return val


def _decode_string(data: list[int]) -> str | None:
    try:
        chars = [chr(b) for b in data if 0x20 <= b <= 0x7E and chr(b).isalnum()]
        return "".join(chars) if len(chars) >= 4 else None
    except Exception:
        return None


# ──────────────────────────────────────────────────────────────────────────────
# Runtime state
# ──────────────────────────────────────────────────────────────────────────────

BITRATES = [5_000, 10_000, 20_000, 50_000, 100_000, 125_000, 250_000,
            500_000, 800_000, 1_000_000]
LOG_DIR  = Path("logs")
CHANNELS_ALL = [f"PCAN_USBBUS{i}" for i in range(1, 9)]

_state = {
    "running":    False,
    "bus":        None,
    "notifier":   None,
    "log_path":   None,
    "msg_count":  0,
    "start_time": None,
    "channel":    None,
    "bitrate":    None,
    "passive":    True,
    "fd":         False,
    "fmt":        ".trc",
}

# CAN ID tracker  {id_str: {count, last_ms, payload, last_ts}}
_can_ids: dict[str, dict] = {}
_can_ids_lock = threading.Lock()

# Signal tracker  {pid: {value, min, max, can_id, vin_partial}}
_signals: dict[str, dict] = {}
_signals_lock = threading.Lock()

_ws_clients: set[WebSocket] = set()
_loop: asyncio.AbstractEventLoop | None = None

# ──────────────────────────────────────────────────────────────────────────────
# FastAPI app
# ──────────────────────────────────────────────────────────────────────────────

@contextlib.asynccontextmanager
async def lifespan(app: FastAPI):
    global _loop
    _loop = asyncio.get_event_loop()
    load_signal_library()
    yield
    if _state["running"]:
        _state["notifier"].stop()
        _state["bus"].shutdown()

app = FastAPI(lifespan=lifespan)

# ──────────────────────────────────────────────────────────────────────────────
# Device scan
# ──────────────────────────────────────────────────────────────────────────────

def scan_devices() -> list[dict]:
    try:
        found = can.detect_available_configs(interfaces=["pcan"])
        detected = {cfg["channel"] for cfg in found}
    except Exception:
        detected = set()

    active_ch = _state.get("channel")
    results = []
    for ch in CHANNELS_ALL:
        if ch == active_ch:
            condition = "active"
        elif ch in detected:
            condition = "available"
        else:
            condition = "unavailable"
        results.append({"channel": ch, "condition": condition})
    return results

# ──────────────────────────────────────────────────────────────────────────────
# WebSocket broadcast
# ──────────────────────────────────────────────────────────────────────────────

def _broadcast(data: dict):
    if _loop is None or not _ws_clients:
        return
    payload = json.dumps(data)
    _loop.call_soon_threadsafe(_enqueue_broadcast, payload)

def _enqueue_broadcast(payload: str):
    asyncio.ensure_future(_send_all(payload))

async def _send_all(payload: str):
    dead = set()
    for ws in _ws_clients:
        try:
            await ws.send_text(payload)
        except Exception:
            dead.add(ws)
    _ws_clients.difference_update(dead)

# ──────────────────────────────────────────────────────────────────────────────
# CAN listener
# ──────────────────────────────────────────────────────────────────────────────

class _CANListener(can.Listener):
    _canid_throttle: dict[str, float] = {}
    _CANID_MIN_INTERVAL = 0.25   # max 4 CAN-ID table updates/s per ID

    def on_message_received(self, msg: can.Message):
        _state["msg_count"] += 1
        now_ts = time.time()

        id_str = (f"{msg.arbitration_id:08X}" if msg.is_extended_id
                  else f"{msg.arbitration_id:04X}")
        data_bytes = list(msg.data) if msg.data else []
        data_hex   = " ".join(f"{b:02X}" for b in data_bytes)

        flags = []
        if msg.is_extended_id:  flags.append("XTD")
        if msg.is_remote_frame: flags.append("RTR")
        if msg.is_fd:           flags.append("FD")
        if msg.is_error_frame:  flags.append("ERR")

        elapsed = (msg.timestamp - _state["start_time"]) if _state["start_time"] else 0.0

        # ── raw frame broadcast ──────────────────────────────────────────────
        _broadcast({
            "type":  "frame",
            "n":     _state["msg_count"],
            "t":     round(elapsed, 4),
            "id":    id_str,
            "xtd":   msg.is_extended_id,
            "rtr":   msg.is_remote_frame,
            "fd":    msg.is_fd,
            "err":   msg.is_error_frame,
            "dl":    msg.dlc,
            "data":  data_hex,
            "flags": " ".join(flags),
        })

        # ── CAN ID table update ──────────────────────────────────────────────
        with _can_ids_lock:
            entry = _can_ids.get(id_str)
            if entry is None:
                _can_ids[id_str] = {
                    "count": 1, "payload": data_hex,
                    "last_ts": now_ts, "last_ms": 0,
                }
                should_bc_id = True
            else:
                entry["last_ms"] = round((now_ts - entry["last_ts"]) * 1000)
                entry["last_ts"] = now_ts
                entry["payload"] = data_hex
                entry["count"]  += 1
                last_bc = self._canid_throttle.get(id_str, 0)
                should_bc_id = (now_ts - last_bc) >= self._CANID_MIN_INTERVAL

        if should_bc_id:
            self._canid_throttle[id_str] = now_ts
            with _can_ids_lock:
                e = _can_ids.get(id_str, {})
            _broadcast({
                "type":    "canid",
                "id":      id_str,
                "count":   e.get("count", 0),
                "payload": e.get("payload", ""),
                "last_ms": e.get("last_ms", 0),
            })

        if len(data_bytes) < 3 or msg.is_error_frame:
            return

        # ── Signal decoding — three layers ──────────────────────────────────
        self._decode_frame(id_str, data_bytes)

    def _decode_frame(self, id_str: str, data: list[int]):
        """
        Decode in standards-first order; vehicle library is the fallback:
          1. J1979   — Mode 01 / 0x41 response  (SAE standard PIDs)
          2. J1979-2 — Mode 22 / 0x62 response  (SAE F4xx DIDs)
          3. Vehicle signal_library.json         (vehicle-specific / proprietary)
        VIN (Mode 09 / 0x49) handled separately at any layer.
        """
        if len(data) < 2:
            return
        mode_byte = data[1]
        decoded_key: str | None = None   # set if a standard layer succeeds

        # Layer 1 — J1979 standard PIDs (Mode 01 / 0x41) ---------------------
        if mode_byte == 0x41 and len(data) >= 3:
            pid_str = f"{data[2]:02X}"
            sig1 = J1979_PIDS.get(pid_str)
            if sig1:
                raw = _bytes_to_int(data[3:], sig1["nbytes"])
                if raw is not None:
                    decoded_key = f"j1979_{pid_str}"
                    val = raw * sig1["mul"] + sig1["off"]
                    self._update_and_emit(decoded_key, sig1["name"],
                                         sig1["unit"], val, id_str)

        # Layer 2 — J1979-2 standard DIDs (Mode 22 / 0x62, F4xx range) -------
        elif mode_byte == 0x62 and len(data) >= 4:
            did_str = f"{data[2]:02X}{data[3]:02X}"
            sig2 = J1979_2_DIDS.get(did_str)
            if sig2:
                raw = _bytes_to_int(data[4:], sig2["nbytes"])
                if raw is not None:
                    decoded_key = f"j19792_{did_str}"
                    val = raw * sig2["mul"] + sig2["off"]
                    self._update_and_emit(decoded_key, sig2["name"],
                                         sig2["unit"], val, id_str)

        # VIN — Mode 09 PID 02 response (0x49) — handled at any layer --------
        elif mode_byte == 0x49 and len(data) >= 3 and data[2] == 0x02:
            s = _decode_string(data[3:])
            if s:
                with _signals_lock:
                    entry = _signals.setdefault("vin", {
                        "name": "VIN", "unit": "", "can_id": id_str,
                        "value": "", "min": None, "max": None,
                    })
                    existing = entry.get("value") or ""
                    for ch in s:
                        if ch not in existing:
                            existing += ch
                    entry["value"] = existing[:17]
                    entry["can_id"] = id_str
                _broadcast({"type": "signal", "pid": "vin",
                            "name": "VIN", "unit": "",
                            "value": entry["value"], "min": None, "max": None,
                            "can_id": id_str})
            decoded_key = "vin"

        # Layer 3 — vehicle signal_library.json (fallback) -------------------
        # Only reached when neither J1979 nor J1979-2 claimed this frame.
        if decoded_key is None:
            for sig in _lib_by_canid.get(id_str, []):
                if mode_byte != sig["response_byte"]:
                    continue
                pid_bytes = sig["pid_bytes"]
                n_pid = len(pid_bytes)
                if len(data) < 2 + n_pid:
                    continue
                if data[2:2 + n_pid] != pid_bytes:
                    continue

                value_data = data[2 + n_pid:]
                key = (f"lib_{id_str}_{sig['response_byte']:02X}_"
                       f"{''.join(f'{b:02X}' for b in pid_bytes)}")

                if sig["is_string"]:
                    s = _decode_string(value_data)
                    if s:
                        self._emit_signal(key, sig["name"], "", s,
                                          None, None, id_str)
                else:
                    raw = _bytes_to_int(value_data, sig["nbytes"])
                    if raw is not None:
                        val = raw * sig["mul"] + sig["off"]
                        self._update_and_emit(key, sig["name"],
                                              sig["unit"], val, id_str)

    def _update_and_emit(self, key: str, name: str, unit: str,
                         val: float, can_id: str):
        val = round(val, 2)
        with _signals_lock:
            entry = _signals.setdefault(key, {
                "name": name, "unit": unit, "can_id": can_id,
                "value": None, "min": None, "max": None,
            })
            entry["value"]  = val
            entry["can_id"] = can_id
            if entry["min"] is None or val < entry["min"]:
                entry["min"] = val
            if entry["max"] is None or val > entry["max"]:
                entry["max"] = val
            cur_min, cur_max = entry["min"], entry["max"]

        _broadcast({"type": "signal", "pid": key,
                    "name": name, "unit": unit,
                    "value": val, "min": cur_min, "max": cur_max,
                    "can_id": can_id})

    def _emit_signal(self, key: str, name: str, unit: str,
                     value, min_v, max_v, can_id: str):
        with _signals_lock:
            _signals[key] = {"name": name, "unit": unit, "can_id": can_id,
                             "value": value, "min": min_v, "max": max_v}
        _broadcast({"type": "signal", "pid": key,
                    "name": name, "unit": unit,
                    "value": value, "min": min_v, "max": max_v,
                    "can_id": can_id})

    def stop(self):
        pass

# ──────────────────────────────────────────────────────────────────────────────
# REST endpoints
# ──────────────────────────────────────────────────────────────────────────────

@app.get("/")
async def index():
    return FileResponse("index.html")

@app.get("/scan")
async def scan():
    devices = scan_devices()
    return JSONResponse({"devices": devices, "bitrates": BITRATES})

@app.post("/start")
async def start(config: dict):
    if _state["running"]:
        return JSONResponse({"ok": False, "error": "Already running"})

    channel = config.get("channel", "PCAN_USBBUS1")
    bitrate = int(config.get("bitrate", 500_000))
    passive = bool(config.get("passive", True))
    fd      = bool(config.get("fd", False))
    fmt     = config.get("fmt", ".trc")
    state   = BusState.PASSIVE if passive else BusState.ACTIVE

    try:
        bus = Bus(interface="pcan", channel=channel, bitrate=bitrate,
                  state=state, fd=fd)
    except Exception as exc:
        return JSONResponse({"ok": False, "error": str(exc)})

    LOG_DIR.mkdir(exist_ok=True)
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    note = "".join(c if c.isalnum() or c in "-_" else "_"
                   for c in config.get("note", "").strip())[:40]
    stem     = f"can_log_{ts}" + (f"_{note}" if note else "")
    log_path = LOG_DIR / f"{stem}{fmt}"
    ctr = 1
    while log_path.exists():
        log_path = log_path.with_stem(f"{stem}_{ctr}")
        ctr += 1

    # Reset trackers
    with _can_ids_lock:
        _can_ids.clear()
    with _signals_lock:
        _signals.clear()

    listeners = [_CANListener(), Logger(str(log_path))]
    notifier  = Notifier(bus, listeners)

    _state.update(running=True, bus=bus, notifier=notifier, log_path=log_path,
                  msg_count=0, start_time=time.time(),
                  channel=channel, bitrate=bitrate, passive=passive, fd=fd, fmt=fmt)

    _broadcast({"type": "started", "log": log_path.name,
                "channel": channel, "bitrate": bitrate,
                "passive": passive, "fd": fd})
    return JSONResponse({"ok": True, "log": str(log_path)})

@app.post("/stop")
async def stop():
    if not _state["running"]:
        return JSONResponse({"ok": False, "error": "Not running"})

    _state["notifier"].stop()
    _state["bus"].shutdown()
    log_path  = _state["log_path"]
    msg_count = _state["msg_count"]
    size_kb   = log_path.stat().st_size / 1024 if log_path and log_path.exists() else 0
    _state.update(running=False, bus=None, notifier=None, log_path=None)

    _broadcast({"type": "stopped", "log": log_path.name if log_path else "—",
                "msgs": msg_count, "size_kb": round(size_kb, 1)})
    return JSONResponse({"ok": True, "log": str(log_path),
                         "msgs": msg_count, "size_kb": round(size_kb, 1)})

@app.post("/kill")
async def kill():
    for key in ("notifier", "bus"):
        obj = _state.get(key)
        if obj:
            try: obj.stop() if key == "notifier" else obj.shutdown()
            except Exception: pass
    _state.update(running=False, bus=None, notifier=None, log_path=None,
                  msg_count=0, start_time=None)
    _broadcast({"type": "killed"})
    return JSONResponse({"ok": True})

@app.get("/status")
async def status():
    with _can_ids_lock:
        id_count = len(_can_ids)
    return JSONResponse({
        "running": _state["running"], "channel": _state["channel"],
        "bitrate": _state["bitrate"], "passive": _state["passive"],
        "msgs": _state["msg_count"], "log": _state["log_path"].name if _state["log_path"] else None,
        "unique_ids": id_count,
    })

@app.get("/library")
async def library():
    return JSONResponse({
        "entries":   _lib_entry_count,
        "path":      _lib_path_used or None,
        "can_ids":   list(_lib_by_canid.keys()),
        "j1979_pids":   list(J1979_PIDS.keys()),
        "j1979_2_dids": list(J1979_2_DIDS.keys()),
    })

# ──────────────────────────────────────────────────────────────────────────────
# WebSocket
# ──────────────────────────────────────────────────────────────────────────────

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    _ws_clients.add(ws)
    # Send current state on connect
    with _can_ids_lock:
        ids_snapshot = dict(_can_ids)
    with _signals_lock:
        sig_snapshot = dict(_signals)
    for id_str, e in ids_snapshot.items():
        await ws.send_text(json.dumps({
            "type": "canid", "id": id_str,
            "count": e["count"], "payload": e["payload"], "last_ms": e["last_ms"],
        }))
    for pid, e in sig_snapshot.items():
        await ws.send_text(json.dumps({
            "type": "signal", "pid": pid,
            "name": e["name"], "unit": e["unit"],
            "value": e["value"], "min": e["min"], "max": e["max"],
            "can_id": e["can_id"],
        }))
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        _ws_clients.discard(ws)

# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import webbrowser
    print("\n  CAN Logger Web UI")
    print("  Open: http://localhost:8000\n")
    threading.Timer(1.2, lambda: webbrowser.open("http://localhost:8000")).start()
    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=False)

#!/usr/bin/env python3
"""
CAN Logger — Web UI backend (GridConnect / SLCAN edition).

Standalone copy that talks to a GridConnect CANUSB COM FD (or any LAWICEL/
SLCAN serial adapter) instead of a PEAK PCAN-USB. Classic CAN only — no CAN FD.
The original PEAK version is unchanged in the parent folder.

Serves index.html and streams CAN frames, CAN ID stats, and decoded signals
to the browser via WebSocket.

Signal decoding layers (standards first, vehicle library as fallback):
  1. J1979   (SAE) — Mode 01 / OBD-II generic PIDs   (0x41 response)
  2. J1979-2 (SAE) — Mode 22 / UDS F4xx DIDs          (0x62 response)
  3. signal_library.json — vehicle-specific PIDs/DIDs  (fallback)

ISO-TP (ISO 15765-2) reassembly is handled inline before decoding:
  - Single frames passed through directly.
  - Multi-frame (First Frame + Consecutive Frames) reassembled per source ECU.
  - Flow Control (CTS) sent automatically when bus is in active mode.
  - Supports both 11-bit (standard OBD-II) and 29-bit (extended) addressing.

Usage:
    ./start.sh        ← recommended
    open http://localhost:8002
"""

import sys

class _StderrFilter:
    """Suppress noisy PCAN driver messages printed directly to stderr."""
    _SUPPRESS = ("Bus error:", "error counter")
    def write(self, s):
        if not any(p in s for p in self._SUPPRESS):
            sys.__stderr__.write(s)
    def flush(self): sys.__stderr__.flush()

sys.stderr = _StderrFilter()

import asyncio
import contextlib
import ctypes
import json
import re
import threading
import time
from datetime import datetime
from pathlib import Path

import can
from can import Bus, Logger, Notifier
from can.interfaces.slcan import slcanBus
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

import scanner

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
    "F42F": {"name": "Fuel Level (J1979-2)",        "nbytes": 1, "mul": 100/255, "off": 0,   "unit": "%"},
    "F4A6": {"name": "Odometer (J1979-2)",          "nbytes": 4, "mul": 0.098,   "off": 0,   "unit": "km"},
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
    """Convert up to nbytes of data to an integer.
    Uses however many bytes are actually available — ISO-TP length
    already bounds the payload so we trust what arrives."""
    n = min(nbytes, len(data)) if nbytes > 0 else len(data)
    if n == 0:
        return None
    val = 0
    for i in range(n):
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

# slcan-supported bitrates (LAWICEL S-commands). Note: 5k and 800k are NOT
# valid for slcan; 83.3k and 750k are.
BITRATES = [10_000, 20_000, 50_000, 83_300, 100_000, 125_000, 250_000,
            500_000, 750_000, 1_000_000]
LOG_DIR  = Path("logs")

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
    scanner.init(
        _state, _signals, _signals_lock,
        _can_ids, _can_ids_lock,
        _lib_by_canid, _broadcast,
    )
    yield
    # ── Ordered shutdown — release USB cleanly ────────────────────────────────
    scanner.stop()
    await asyncio.sleep(0.2)          # let scanner task cancel
    if _state["running"]:
        _teardown_bus()

app = FastAPI(lifespan=lifespan)

# ──────────────────────────────────────────────────────────────────────────────
# Device scan
# ──────────────────────────────────────────────────────────────────────────────

# Serial ports that are never CAN adapters — hide them from the picker.
_SERIAL_SKIP = ("bluetooth", "debug-console")


def scan_devices() -> list[dict]:
    """List serial ports for the GridConnect / LAWICEL (slcan) adapter."""
    try:
        from serial.tools import list_ports
        ports = list_ports.comports()
    except Exception:
        ports = []

    active_ch = _state.get("channel")
    results = []
    for p in sorted(ports, key=lambda x: x.device):
        dev = p.device
        if any(s in dev.lower() for s in _SERIAL_SKIP):
            continue
        results.append({
            "channel":   dev,
            "condition": "active" if dev == active_ch else "available",
            "label":     (p.description or "").strip() or dev,
        })
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
# slcan bus with adapter error-register polling
# ──────────────────────────────────────────────────────────────────────────────
#
# The CANable 1.0 "canable-fw" firmware does NOT implement the standard LAWICEL
# 'F' status-flags command, and it does not emit CAN error frames. It exposes a
# single nonstandard 'E' command that returns its internal *sticky* error
# register as the (unterminated) string:  "CANable Error Register: <hex>"
#
# Register bits (canable-fw error.h — sticky since power-on, no clear command):
#     bit0 ERR_PERIPHINIT          peripheral init failure
#     bit1 ERR_USBTX_BUSY          USB TX busy (host not draining fast enough)
#     bit2 ERR_CAN_TXFAIL          a CAN transmit failed (no ACK / bus fault)
#     bit3 ERR_CANRXFIFO_OVERFLOW  RX FIFO overflow — frames were dropped
#     bit4 ERR_FULLBUF_CANTX       internal CAN TX ring full
#     bit5 ERR_FULLBUF_USBRX       internal USB RX ring full
#
# The 'E' reply carries no terminator, so to avoid corrupting the frame stream
# we only issue 'E' when the read buffer is empty, and only treat a line that
# *starts* with the marker as a reply. Classic-CAN frames begin with t/T/r/R
# (non-hex), so any frame glued after the reply splits off cleanly.

_ERRREG_RE = re.compile(r"CANable Error Register:\s*([0-9A-Fa-f]{1,2})")

# (bit mask, key, human label, severe?)
_ERRREG_BITS = [
    (0x01, "periph_init",  "Peripheral init failure", False),
    (0x02, "usbtx_busy",   "USB TX busy",             False),
    (0x04, "can_tx_fail",  "CAN TX failed",           True),
    (0x08, "rx_overflow",  "RX FIFO overflow",        True),
    (0x10, "cantx_buffull","CAN TX buffer full",      False),
    (0x20, "usbrx_buffull","USB RX buffer full",      False),
]


class StatusSlcanBus(slcanBus):
    """slcanBus that also polls the canable-fw 'E' error register."""

    _POLL_INTERVAL = 2.0  # seconds between 'E' polls

    def __init__(self, *args, status_callback=None, **kwargs):
        # These must exist before super().__init__ (it calls self._write).
        self._write_lock  = threading.Lock()
        self._status_lock = threading.Lock()
        self._status_cb   = status_callback
        self._err_reg     = 0
        self._err_seen    = 0       # cumulative OR of everything read this session
        self._err_time    = 0.0
        self._supported   = None    # None=unknown, True/False once determined
        self._last_poll   = 0.0
        self._first_poll  = 0.0
        super().__init__(*args, **kwargs)

    # Serialize writes so an 'E' poll never interleaves bytes with a send().
    def _write(self, string: str) -> None:
        with self._write_lock:
            super()._write(string)

    def _maybe_poll(self) -> None:
        now = time.monotonic()
        if now - self._last_poll < self._POLL_INTERVAL:
            return
        # Only poll when no partial frame is buffered — keeps the reply clean.
        if self._buffer:
            return
        self._last_poll = now
        if not self._first_poll:
            self._first_poll = now
        try:
            self._write("E")
        except Exception:
            return
        # The 'E' reply ("CANable Error Register: <hex>") has NO terminator, so
        # the line-based reader would never return it on an idle bus. Read it
        # directly here. We're in the (single) recv thread, so this can't race.
        self._read_err_reply()

    def _read_err_reply(self) -> None:
        deadline = time.monotonic() + 0.12
        chunk = bytearray()
        ser = self.serialPortOrig
        while time.monotonic() < deadline:
            n = ser.in_waiting
            if n:
                chunk.extend(ser.read(n))
                m = _ERRREG_RE.search(chunk.decode(errors="ignore"))
                if m:
                    self._record(int(m.group(1), 16))
                    # Bytes after the marker may be real frame data — hand them
                    # back to the normal line reader.
                    tail = chunk.decode(errors="ignore")[m.end():]
                    if tail:
                        self._buffer.extend(tail.encode())
                    return
            else:
                time.sleep(0.004)
        # No parseable reply this round — preserve whatever we read for _read().
        if chunk:
            self._buffer.extend(chunk)

    def _recv_internal(self, timeout):
        self._maybe_poll()

        # If we've polled for a while with no parseable reply, mark unsupported.
        if (self._supported is None and self._first_poll
                and time.monotonic() - self._first_poll > 8.0):
            with self._status_lock:
                self._supported = False
            self._emit()

        if self._queue.qsize():
            string = self._queue.get_nowait()
        else:
            string = self._read(timeout)

        if string and string.startswith("CANable Error Register"):
            m = _ERRREG_RE.match(string)
            if m:
                self._record(int(m.group(1), 16))
                remainder = string[m.end():]
                if remainder:
                    self._queue.put_nowait(remainder)
                    return super()._recv_internal(timeout)
            return None, False

        if string is not None:
            self._queue.put_nowait(string)
        return super()._recv_internal(timeout)

    def _record(self, reg: int) -> None:
        with self._status_lock:
            self._supported = True
            self._err_reg   = reg
            self._err_seen |= reg
            self._err_time  = time.time()
        self._emit()

    def _emit(self) -> None:
        if self._status_cb:
            try:
                self._status_cb(self.error_snapshot())
            except Exception:
                pass

    def error_snapshot(self) -> dict:
        with self._status_lock:
            reg, seen, supported, ts = (self._err_reg, self._err_seen,
                                        self._supported, self._err_time)
        active = [{"key": k, "label": lbl, "severe": sev}
                  for bit, k, lbl, sev in _ERRREG_BITS if seen & bit]
        return {"supported": supported, "reg": reg, "seen": seen,
                "active": active, "ts": ts}


def _on_bus_status(snap: dict) -> None:
    _broadcast({"type": "errreg", **snap})

# ──────────────────────────────────────────────────────────────────────────────
# CAN listener
# ──────────────────────────────────────────────────────────────────────────────

# ──────────────────────────────────────────────────────────────────────────────
# ISO-TP reassembler (ISO 15765-2)
# ──────────────────────────────────────────────────────────────────────────────

class _ISOTPReassembler:
    """
    Lightweight ISO-TP reassembler.  No external library required.

    All frames for every source ECU pass through here.  Returns a complete
    reassembled application-layer payload when a sequence finishes, or None
    while still collecting.

    After reassembly the payload is protocol-layer bytes with the ISO-TP
    framing stripped, e.g.:
        Mode 01 response → [0x41, PID, A, B, ...]
        Mode 22 response → [0x62, DID_H, DID_L, A, B, ...]
        Mode 09 VIN      → [0x49, 0x02, 0x01, V, I, N, ...]

    Flow Control (CTS) frames are sent automatically when the bus reference
    is provided and the session is in active (non-passive) mode.
    """

    def __init__(self):
        # { src_id_str → {total, data, next_sn} }
        self._bufs: dict[str, dict] = {}

    def feed(self, src_id: str, src_int: int, is_ext: bool,
             raw: list[int], bus) -> list[int] | None:
        if not raw:
            return None

        nibble = (raw[0] >> 4) & 0x0F

        # ── Single Frame (SF) ────────────────────────────────────────────────
        if nibble == 0x0:
            length = raw[0] & 0x0F
            if length == 0 or len(raw) < length + 1:
                return None
            self._bufs.pop(src_id, None)
            return raw[1:1 + length]

        # ── First Frame (FF) ─────────────────────────────────────────────────
        if nibble == 0x1:
            if len(raw) < 2:
                return None
            total = ((raw[0] & 0x0F) << 8) | raw[1]
            self._bufs[src_id] = {
                "total": total,
                "data":  list(raw[2:]),
                "sn":    1,
            }
            self._send_fc(src_int, is_ext, bus)
            return None

        # ── Consecutive Frame (CF) ───────────────────────────────────────────
        if nibble == 0x2:
            buf = self._bufs.get(src_id)
            if buf is None:
                return None
            sn = raw[0] & 0x0F
            if sn != buf["sn"] % 16:
                self._bufs.pop(src_id, None)   # sequence error — discard
                return None
            buf["data"].extend(raw[1:])
            buf["sn"] += 1
            if len(buf["data"]) >= buf["total"]:
                payload = buf["data"][:buf["total"]]
                self._bufs.pop(src_id, None)
                return payload
            return None

        # FC and unknown frame types — not application data
        return None

    def _send_fc(self, src_int: int, is_ext: bool, bus):
        """Send a Flow Control (Continue To Send) frame back to the ECU."""
        if bus is None:
            return
        tx_id = self._fc_tx_id(src_int, is_ext)
        if tx_id is None:
            return
        fc = bytes([0x30, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        try:
            bus.send(can.Message(arbitration_id=tx_id, data=fc,
                                 is_extended_id=is_ext, is_fd=False))
        except Exception:
            pass

    @staticmethod
    def _fc_tx_id(rx_id: int, is_ext: bool) -> int | None:
        """
        Derive the tester TX address from the ECU's response address.

        11-bit (standard): ECU responds on 0x7E8–0x7EF → tester sends on
                           0x7E0–0x7E7 (rx_id − 8).

        29-bit (extended): ECU responds on 0x18DAF1{ECU} → tester sends on
                           0x18DA{ECU}F1 (swap last two address bytes).
        """
        if not is_ext:
            if 0x7E8 <= rx_id <= 0x7EF:
                return rx_id - 8
            return None
        # 29-bit: bytes are 0x18 0xDA {tester} {ecu}
        ecu_addr    = rx_id & 0xFF
        tester_addr = (rx_id >> 8) & 0xFF
        return (rx_id & 0xFFFF0000) | (ecu_addr << 8) | tester_addr

    def clear(self):
        self._bufs.clear()


_isotp = _ISOTPReassembler()

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

        if _state.get("t0_hw") is None:
            _state["t0_hw"]         = msg.timestamp
            _state["first_msg_wall"] = datetime.now()
        elapsed = msg.timestamp - _state["t0_hw"]

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

        if msg.is_error_frame or len(data_bytes) < 2:
            return

        # ── ISO-TP reassembly → signal decoding ──────────────────────────────
        # Pass the bus only when active (non-passive) so FC frames are sent.
        bus = _state["bus"] if not _state.get("passive") else None
        payload = _isotp.feed(id_str, msg.arbitration_id,
                              msg.is_extended_id, data_bytes, bus)
        if payload is not None:
            self._decode_frame(id_str, payload)

    def _decode_frame(self, id_str: str, data: list[int]):
        """
        Decode a fully reassembled ISO-TP payload.

        After ISO-TP stripping, data[0] is the service/mode byte:
          Mode 01 response → [0x41, PID, A, B, ...]
          Mode 22 response → [0x62, DID_H, DID_L, A, B, ...]
          Mode 09 VIN      → [0x49, 0x02, 0x01, V, I, N, ...]

        Standards first; vehicle library is the fallback.
        """
        if len(data) < 2:
            return
        mode_byte = data[0]          # ← reassembled payload, no length prefix
        decoded_key: str | None = None

        # Layer 1 — J1979 standard PIDs (Mode 01 / 0x41) ---------------------
        if mode_byte == 0x41 and len(data) >= 2:
            pid_str = f"{data[1]:02X}"
            sig1 = J1979_PIDS.get(pid_str)
            if sig1:
                raw = _bytes_to_int(data[2:], sig1["nbytes"])
                if raw is not None:
                    decoded_key = f"j1979_{pid_str}"
                    val = raw * sig1["mul"] + sig1["off"]
                    self._update_and_emit(decoded_key, sig1["name"],
                                         sig1["unit"], val, id_str)

        # Layer 2 — J1979-2 standard DIDs (Mode 22 / 0x62, F4xx range) -------
        elif mode_byte == 0x62 and len(data) >= 3:
            did_str = f"{data[1]:02X}{data[2]:02X}"
            sig2 = J1979_2_DIDS.get(did_str)
            if sig2:
                raw = _bytes_to_int(data[3:], sig2["nbytes"])
                if raw is not None:
                    decoded_key = f"j19792_{did_str}"
                    val = raw * sig2["mul"] + sig2["off"]
                    self._update_and_emit(decoded_key, sig2["name"],
                                         sig2["unit"], val, id_str)

        # VIN — Mode 09 PID 02 response (0x49) --------------------------------
        # Reassembled payload: [0x49, 0x02, count, VIN_char×17]
        elif mode_byte == 0x49 and len(data) >= 3 and data[1] == 0x02:
            # data[2] = number of data items (always 0x01); VIN starts at [3]
            vin_bytes = data[3:] if len(data) > 3 else data[2:]
            vin = "".join(chr(b) for b in vin_bytes
                          if 0x20 <= b <= 0x7E and chr(b).isalnum())[:17]
            if len(vin) >= 5:
                with _signals_lock:
                    _signals["vin"] = {"name": "VIN", "unit": "",
                                       "can_id": id_str, "value": vin,
                                       "min": None, "max": None}
                _broadcast({"type": "signal", "pid": "vin",
                            "name": "VIN", "unit": "", "value": vin,
                            "min": None, "max": None, "can_id": id_str})
            decoded_key = "vin"

        # Layer 3 — vehicle signal_library.json (fallback) -------------------
        # Consulted only when J1979 / J1979-2 did not claim this payload.
        # After reassembly: mode=data[0], pid/did bytes start at data[1].
        if decoded_key is None:
            for sig in _lib_by_canid.get(id_str, []):
                if mode_byte != sig["response_byte"]:
                    continue
                pid_bytes = sig["pid_bytes"]
                n_pid = len(pid_bytes)
                if len(data) < 1 + n_pid:
                    continue
                if data[1:1 + n_pid] != pid_bytes:
                    continue

                value_data = data[1 + n_pid:]
                key = (f"lib_{id_str}_{sig['response_byte']:02X}_"
                       f"{''.join(f'{b:02X}' for b in pid_bytes)}")

                if sig.get("value_map") is not None:
                    bi  = sig.get("byte_index") or 0
                    bval = value_data[bi] if bi < len(value_data) else None
                    if bval is not None:
                        s = sig["value_map"].get(f"{bval:02X}".upper(),
                                                  f"0x{bval:02X}")
                        self._emit_signal(key, sig["name"], "", s,
                                          None, None, id_str, track_states=True)
                elif sig.get("raw_hex"):
                    s = " ".join(f"{b:02X}" for b in value_data) if value_data else ""
                    if s:
                        self._emit_signal(key, sig["name"], "", s,
                                          None, None, id_str)
                elif sig["is_string"]:
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
                     value, min_v, max_v, can_id: str, track_states: bool = False):
        with _signals_lock:
            prev = _signals.get(key, {})
            if track_states and isinstance(value, str):
                # min = first seen, max = previous value before last change
                min_v = prev.get("min") if prev.get("min") is not None else value
                max_v = prev.get("value") if prev.get("value") != value else prev.get("max")
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

    channel = config.get("channel", "")
    bitrate = int(config.get("bitrate", 500_000))
    passive = bool(config.get("passive", False))
    fd      = False   # GridConnect/slcan path: classic CAN only
    fmt     = config.get("fmt", ".trc")

    if not channel:
        return JSONResponse({"ok": False, "error": "No serial port selected"})

    try:
        # GridConnect CANUSB COM FD speaks the LAWICEL/SLCAN protocol.
        # listen_only mirrors the PEAK PASSIVE state: silent monitor, no TX.
        # StatusSlcanBus additionally polls the canable-fw 'E' error register.
        bus = StatusSlcanBus(channel=channel, bitrate=bitrate,
                             listen_only=passive,
                             status_callback=_on_bus_status)
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
    _isotp.clear()

    can_logger = Logger(str(log_path))
    listeners  = [_CANListener(), can_logger]
    notifier   = Notifier(bus, listeners)

    _state.update(running=True, bus=bus, notifier=notifier, log_path=log_path,
                  can_logger=can_logger,
                  msg_count=0, start_time=time.time(), t0_hw=None, first_msg_wall=None,
                  channel=channel, bitrate=bitrate, passive=passive, fd=fd, fmt=fmt)

    _broadcast({"type": "started", "log": log_path.name,
                "channel": channel, "bitrate": bitrate,
                "passive": passive, "fd": fd})
    return JSONResponse({"ok": True, "log": str(log_path)})

def _teardown_bus():
    """Stop notifier, reset CAN error state, shut down bus, clear state."""
    try:
        if _state.get("notifier"):
            _state["notifier"].stop()
        if _state.get("bus"):
            try: _state["bus"].reset()   # clear error-warning before USB release
            except Exception: pass
            _state["bus"].shutdown()
    except Exception:
        pass
    log_path       = _state.get("log_path")
    msg_count      = _state.get("msg_count", 0)
    start_wall     = _state.get("start_time")
    first_msg_wall = _state.get("first_msg_wall")
    _state.update(running=False, bus=None, notifier=None, log_path=None,
                  can_logger=None, msg_count=0, start_time=None, t0_hw=None,
                  first_msg_wall=None)

    # Prepend wall-clock timing comments at the top of the log file
    if log_path and log_path.exists():
        try:
            start_str = (datetime.fromtimestamp(start_wall).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                         if start_wall else "unknown")
            first_str = (first_msg_wall.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                         if first_msg_wall else "(no frames received)")
            header = (f"; Logging started (wall clock): {start_str}\n"
                      f"; First CAN frame (wall clock):  {first_str}\n"
                      f";\n")
            with open(log_path, "r") as f:
                original = f.read()
            with open(log_path, "w") as f:
                f.write(header + original)
        except Exception:
            pass

    return log_path, msg_count


@app.post("/stop")
async def stop():
    if not _state["running"]:
        return JSONResponse({"ok": False, "error": "Not running"})
    scanner.stop()
    await asyncio.sleep(0.2)
    log_path, msg_count = _teardown_bus()
    size_kb = log_path.stat().st_size / 1024 if log_path and log_path.exists() else 0
    _broadcast({"type": "stopped", "log": log_path.name if log_path else "—",
                "msgs": msg_count, "size_kb": round(size_kb, 1)})
    return JSONResponse({"ok": True, "log": str(log_path),
                         "msgs": msg_count, "size_kb": round(size_kb, 1)})


@app.post("/kill")
async def kill():
    scanner.stop()
    await asyncio.sleep(0.1)
    _teardown_bus()
    _broadcast({"type": "killed"})
    return JSONResponse({"ok": True})

@app.get("/status")
async def status():
    with _can_ids_lock:
        id_count = len(_can_ids)
    bus = _state.get("bus")
    errreg = bus.error_snapshot() if hasattr(bus, "error_snapshot") else None
    return JSONResponse({
        "running": _state["running"], "channel": _state["channel"],
        "bitrate": _state["bitrate"], "passive": _state["passive"],
        "msgs": _state["msg_count"], "log": _state["log_path"].name if _state["log_path"] else None,
        "unique_ids": id_count,
        "errreg": errreg,
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

# ── Active scan endpoints ─────────────────────────────────────────────────────

@app.post("/scan/start")
async def scan_start():
    if not _state["running"]:
        return JSONResponse({"ok": False,
                             "error": "CAN bus not started — start logging first"})
    if _state.get("passive"):
        return JSONResponse({"ok": False,
                             "error": "Bus is in passive mode — restart without listen-only"})
    ok = scanner.start(_state["bus"])
    if not ok:
        return JSONResponse({"ok": False, "error": "Scan already running"})
    return JSONResponse({"ok": True})

@app.post("/scan/stop")
async def scan_stop():
    scanner.stop()
    return JSONResponse({"ok": True})

@app.post("/scan/sweep")
async def scan_sweep():
    if not _state["running"]:
        return JSONResponse({"ok": False, "error": "Start logging first"})
    ok = scanner.start_sweep(_state["bus"])
    if not ok:
        return JSONResponse({"ok": False, "error": "Scan already running"})
    return JSONResponse({"ok": True})

@app.get("/scan/status")
async def scan_status():
    return JSONResponse(scanner.get_status())

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
    bus = _state.get("bus")
    if bus is not None and hasattr(bus, "error_snapshot"):
        await ws.send_text(json.dumps({"type": "errreg", **bus.error_snapshot()}))
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        _ws_clients.discard(ws)

# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import webbrowser
    print("\n  CAN Logger Web UI — GridConnect / SLCAN edition")
    print("  Open: http://localhost:8002\n")
    threading.Timer(1.2, lambda: webbrowser.open("http://localhost:8002")).start()
    uvicorn.run("server:app", host="0.0.0.0", port=8002, reload=False)

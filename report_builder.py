#!/usr/bin/env python3
"""
CAN Log Report Builder
======================
Reads a .trc log file, identifies OBD-II / UDS request-response pairs,
and generates a self-contained HTML report with a zoomable timeline.

Usage:
    python report_builder.py [path/to/file.trc]

If no file is given it auto-selects the most recent .trc in ./logs/.
"""

import re
import sys
import json
import math
from pathlib import Path
from datetime import datetime, timedelta
from collections import defaultdict

# ── OBD-II PID names ─────────────────────────────────────────────────────────
PID_NAMES = {
    "00": "Supported PIDs [01-20]",  "01": "Monitor Status",
    "02": "Freeze DTC",              "03": "Fuel System Status",
    "04": "Engine Load",             "05": "Coolant Temp",
    "06": "STFT Bank1",              "07": "LTFT Bank1",
    "08": "STFT Bank2",              "09": "LTFT Bank2",
    "0A": "Fuel Pressure",           "0B": "MAP Pressure",
    "0C": "Engine RPM",              "0D": "Vehicle Speed",
    "0E": "Timing Advance",          "0F": "Intake Air Temp",
    "10": "MAF Rate",                "11": "Throttle Position",
    "12": "Secondary Air Status",    "13": "O2 Sensors Present",
    "1C": "OBD Standard",            "1F": "Run Time Since Start",
    "20": "Supported PIDs [21-40]",  "21": "Distance w/ MIL",
    "22": "Fuel Rail Pressure",      "23": "Fuel Rail Gauge",
    "2F": "Fuel Tank Level",         "30": "Warmups Since Clear",
    "31": "Distance Since Clear",    "33": "Barometric Pressure",
    "40": "Supported PIDs [41-60]",  "41": "Monitor Status Drive",
    "42": "Control Module Voltage",  "43": "Absolute Load",
    "44": "Air-Fuel Equiv Ratio",    "45": "Relative Throttle",
    "46": "Ambient Air Temp",        "47": "Abs Throttle B",
    "4D": "Time w/ MIL On",          "4E": "Time Since Clear",
    "60": "Supported PIDs [61-80]",  "61": "Demand Engine Torque",
    "62": "Actual Engine Torque",    "63": "Engine Torque Reference",
    "80": "Supported PIDs [81-A0]",  "A0": "Supported PIDs [A1-C0]",
    "A6": "Odometer",
    "C0": "Supported PIDs [C1-E0]",
}
DID_NAMES = {
    "F190": "VIN (F190)",  "F1B0": "VIN (F1B0)",
    "F40C": "RPM (J1979-2)",         "F40D": "Speed (J1979-2)",
    "F40F": "IAT (J1979-2)",         "F411": "Throttle (J1979-2)",
    "F41F": "Run Time (J1979-2)",    "F42F": "Fuel Level (J1979-2)",
    "F4A6": "Odometer (J1979-2)",    "F805": "Coolant Temp (J1979-2)",
    "7010": "HVB SoC",               "2000": "Odometer (prop)",
    "5802": "Vehicle Ready",
}
MODE09_NAMES = {"02": "VIN", "04": "CAL ID", "06": "CVN", "0A": "ECU Name"}

# ── TRC parser ────────────────────────────────────────────────────────────────
LINE_RE = re.compile(
    r"^\s*(\d+)\s+([\d.]+)\s+DT\s+(\d+)\s+([0-9A-Fa-f]+)\s+Rx\s+-\s+(\d+)\s+(.+)$"
)

def parse_trc(path: Path):
    """Return (meta, frames).
    meta = dict of header info.
    frames = list of dicts {n, t_ms, bus, can_id, dlc, data: list[int]}.
    """
    meta = {
        "file": path.name,
        "fileversion": None,
        "starttime_raw": None,
        "starttime_wall": None,       # python-can header
        "log_start_wall": None,       # our appended comment
        "first_frame_wall": None,     # our appended comment
        "filename_wall": None,        # derived from the filename timestamp
    }

    # Filename timestamp is the most reliable start reference:
    #   can_log_YYYYMMDD_HHMMSS_mmm_<name>.trc
    if fm := re.search(r"can_log_(\d{8})_(\d{6})_(\d{3})", path.name):
        try:
            meta["filename_wall"] = datetime.strptime(
                fm.group(1) + fm.group(2) + fm.group(3), "%Y%m%d%H%M%S%f"
            )
        except ValueError:
            pass

    frames = []

    with open(path, "r", errors="replace") as fh:
        for line in fh:
            line = line.rstrip()

            # Header comments
            if line.startswith(";"):
                if m := re.match(r";\$FILEVERSION=(.*)", line):
                    meta["fileversion"] = m.group(1).strip()
                elif m := re.match(r";\$STARTTIME=(.*)", line):
                    meta["starttime_raw"] = m.group(1).strip()
                elif m := re.search(r"Start time:\s*(.+)", line):
                    meta["starttime_wall"] = m.group(1).strip()
                elif m := re.search(r"Logging started \(wall clock\):\s*(.+)", line):
                    meta["log_start_wall"] = m.group(1).strip()
                elif m := re.search(r"First CAN frame \(wall clock\):\s*(.+)", line):
                    meta["first_frame_wall"] = m.group(1).strip()
                continue

            if m := LINE_RE.match(line):
                n      = int(m.group(1))
                t_ms   = float(m.group(2))
                bus    = int(m.group(3))
                can_id = m.group(4).upper().lstrip("0") or "0"
                dlc    = int(m.group(5))
                data   = [int(x, 16) for x in m.group(6).split()]
                frames.append({"n": n, "t_ms": t_ms, "bus": bus,
                               "can_id": can_id, "dlc": dlc, "data": data})

    return meta, frames

# ── Frame decoder ─────────────────────────────────────────────────────────────
REQUEST_IDS  = {"7DF", "18DB33F1"}
RESPONSE_IDS_11 = {f"7E{i}" for i in "89ABCDEF"}
RESPONSE_IDS_29 = None   # any 18DAF1xx

def is_response_id(can_id: str) -> bool:
    if can_id in RESPONSE_IDS_11:
        return True
    if len(can_id) == 8 and can_id.startswith("18DAF1"):
        return True
    return False

def is_request_id(can_id: str) -> bool:
    if can_id in REQUEST_IDS:
        return True
    if len(can_id) == 8 and can_id.startswith("18DA") and can_id.endswith("F1"):
        return True
    return False

def decode_frame(f):
    """Return a decoded description dict or None."""
    d = f["data"]
    if not d:
        return None
    can_id = f["can_id"]

    # ── Request ───────────────────────────────────────────────────────────────
    if is_request_id(can_id):
        if len(d) >= 3 and d[0] == 0x02 and d[1] == 0x01:
            pid = f"{d[2]:02X}"
            return {"kind": "req", "proto": "OBD-II", "service": "01",
                    "pid": pid, "label": f"PID {pid} – {PID_NAMES.get(pid, '?')}"}
        if len(d) >= 3 and d[0] == 0x02 and d[1] == 0x09:
            pid = f"{d[2]:02X}"
            return {"kind": "req", "proto": "OBD-II", "service": "09",
                    "pid": pid, "label": f"Mode 09 PID {pid} – {MODE09_NAMES.get(pid, '?')}"}
        if len(d) >= 4 and d[0] == 0x03 and d[1] == 0x22:
            did = f"{d[2]:02X}{d[3]:02X}"
            return {"kind": "req", "proto": "UDS", "service": "22",
                    "pid": did, "label": f"DID {did} – {DID_NAMES.get(did, '?')}"}
        return None

    # ── Response ──────────────────────────────────────────────────────────────
    if is_response_id(can_id):
        # ISO-TP First Frame
        if (d[0] & 0xF0) == 0x10:
            payload = d[2:]
            if len(payload) >= 3 and payload[0] == 0x49 and payload[1] == 0x02:
                return {"kind": "resp", "proto": "OBD-II", "service": "09",
                        "pid": "02", "label": "VIN First Frame",
                        "multiframe": True, "raw": payload[2:]}
            if len(payload) >= 3 and payload[0] == 0x62:
                did = f"{payload[1]:02X}{payload[2]:02X}"
                return {"kind": "resp", "proto": "UDS", "service": "22",
                        "pid": did, "label": f"DID {did} – {DID_NAMES.get(did,'?')} (FF)",
                        "multiframe": True, "raw": payload[3:]}
            return {"kind": "resp", "proto": "?", "service": "?",
                    "pid": "?", "label": "Multiframe First Frame", "multiframe": True}

        # ISO-TP Consecutive Frame — skip for analysis
        if (d[0] & 0xF0) == 0x20:
            return None

        # Single frame OBD-II
        if len(d) >= 2 and d[1] == 0x41:
            pid = f"{d[2]:02X}" if len(d) > 2 else "?"
            raw = d[3:]
            return {"kind": "resp", "proto": "OBD-II", "service": "01",
                    "pid": pid, "label": f"PID {pid} – {PID_NAMES.get(pid,'?')}",
                    "raw": raw, "multiframe": False}
        if len(d) >= 2 and d[1] == 0x49:
            pid = f"{d[2]:02X}" if len(d) > 2 else "?"
            return {"kind": "resp", "proto": "OBD-II", "service": "09",
                    "pid": pid, "label": f"Mode 09 PID {pid}",
                    "raw": d[3:], "multiframe": False}
        if len(d) >= 4 and d[1] == 0x62:
            did = f"{d[2]:02X}{d[3]:02X}"
            return {"kind": "resp", "proto": "UDS", "service": "22",
                    "pid": did, "label": f"DID {did} – {DID_NAMES.get(did,'?')}",
                    "raw": d[4:], "multiframe": False}
        if len(d) >= 2 and d[1] == 0x7F:
            return {"kind": "resp", "proto": "UDS", "service": "NRC",
                    "pid": "NRC", "label": f"Negative Response (NRC {d[2]:02X})",
                    "raw": d[2:], "multiframe": False}
    return None

# ── Pair matcher ──────────────────────────────────────────────────────────────
def match_pairs(frames):
    """
    Build per-PID stats and a list of (req_frame, resp_frame|None) pairs.
    A response is matched if it arrives within 500 ms of a request for the same PID.
    """
    pending = {}   # pid → (req_frame, decoded_req)
    pairs   = []
    stats   = defaultdict(lambda: {"req": 0, "resp": 0, "no_resp": 0,
                                   "min_rt_ms": None, "max_rt_ms": None,
                                   "total_rt_ms": 0, "proto": "?", "label": "?"})

    for f in frames:
        dec = decode_frame(f)
        if dec is None:
            continue

        pid = dec["pid"]
        key = (dec["proto"], pid)

        if dec["kind"] == "req":
            # If there was a previous unmatched request for same PID, mark it
            if key in pending:
                pf, pd = pending[key]
                pairs.append((pf, None, pd))
                stats[key]["req"] += 1
                stats[key]["no_resp"] += 1
            pending[key] = (f, dec)
            stats[key]["proto"] = dec["proto"]
            stats[key]["label"] = dec["label"]

        elif dec["kind"] == "resp":
            if key in pending:
                pf, pd = pending.pop(key)
                rt = f["t_ms"] - pf["t_ms"]
                if rt <= 500:
                    pairs.append((pf, f, pd))
                    s = stats[key]
                    s["req"]  += 1
                    s["resp"] += 1
                    s["total_rt_ms"] += rt
                    if s["min_rt_ms"] is None or rt < s["min_rt_ms"]:
                        s["min_rt_ms"] = rt
                    if s["max_rt_ms"] is None or rt > s["max_rt_ms"]:
                        s["max_rt_ms"] = rt
                    s["proto"] = pd["proto"]
                    s["label"] = pd["label"]
                else:
                    pairs.append((pf, None, pd))
                    stats[key]["req"] += 1
                    stats[key]["no_resp"] += 1
                    pending[key] = (f, dec)   # treat response as new outstanding

    # Flush remaining
    for key, (pf, pd) in pending.items():
        pairs.append((pf, None, pd))
        stats[key]["req"]     += 1
        stats[key]["no_resp"] += 1

    return pairs, dict(stats)

# ── HTML renderer ─────────────────────────────────────────────────────────────
COLORS = [
    "#4FC3F7","#81C784","#FFB74D","#E57373","#CE93D8",
    "#80DEEA","#F48FB1","#A5D6A7","#FFCC80","#B39DDB",
    "#80CBC4","#EF9A9A","#FFF176","#90CAF9","#FFAB91",
]

def build_test_config_html(tm):
    """Render the test configuration section from a .meta.json dict."""
    if not tm:
        return ""

    # Hardware info
    hw = tm.get("hardware", {})
    hw_rows = "".join(
        f'<tr><td class="k">{k.replace("_"," ").title()}</td><td>{v}</td></tr>'
        for k, v in hw.items()
    )

    # Volt & CAN steps combined into a timeline table
    tl_rows = ""
    for ev in tm.get("timeline", []):
        v_str = f'{ev["rail_v"]:.3f} V  {ev["rail_a"]:.3f} A' if "rail_v" in ev else "—"
        tl_rows += (f'<tr><td class="c">{ev["min"]:.2f}</td>'
                    f'<td>{ev["event"]}</td><td class="c">{v_str}</td></tr>')

    # Volt step badges
    vsteps = "  →  ".join(
        f'<span class="pill pill-blue">{s["min"]}m → {s["voltage_v"]}V</span>'
        for s in tm.get("volt_steps", [])
    )
    csteps = "  →  ".join(
        f'<span class="pill {"pill-green" if s["state"]=="ON" else "pill-red"}">'
        f'{s["min"]}m → CAN {s["state"]}</span>'
        for s in tm.get("can_steps", [])
    )

    return f"""
  <h2>Test Configuration</h2>
  <div class="cards">
    <div class="card"><div class="k">Test Name</div><div class="v" style="font-size:.85em">{tm.get('test_name','—')}</div></div>
    <div class="card"><div class="k">Start Mode</div><div class="v">{tm.get('start_mode','—')}</div></div>
    <div class="card"><div class="k">Power On</div><div class="v">{tm.get('power_on_min',0)} min @ {tm.get('power_voltage_v','?')} V</div></div>
    <div class="card"><div class="k">WiTech On</div><div class="v">{tm.get('witech_on','—')}</div></div>
    <div class="card"><div class="k">WiTech Off</div><div class="v">{tm.get('witech_off','—')}</div></div>
    <div class="card"><div class="k">Total Time</div><div class="v">{tm.get('total_time_min','—')} min</div></div>
    <div class="card"><div class="k">At Test End</div><div class="v" style="font-size:.85em">{tm.get('at_test_end','—')}</div></div>
  </div>

  <div style="margin-bottom:10px"><strong style="color:#778;font-size:.78em;text-transform:uppercase;letter-spacing:.05em">Voltage Steps</strong><br><div style="margin-top:5px">{vsteps}</div></div>
  <div style="margin-bottom:18px"><strong style="color:#778;font-size:.78em;text-transform:uppercase;letter-spacing:.05em">CAN Steps</strong><br><div style="margin-top:5px">{csteps}</div></div>

  <table style="margin-bottom:22px">
    <thead><tr><th class="r" style="width:80px">Time (min)</th><th>Event</th><th class="r">Rail Measurement</th></tr></thead>
    <tbody>{tl_rows}</tbody>
  </table>

  {"<table style='margin-bottom:22px'><thead><tr><th>Hardware</th><th>Value</th></tr></thead><tbody>" + hw_rows + "</tbody></table>" if hw_rows else ""}
"""


def build_html(meta, frames, pairs, stats, test_meta=None):
    total_ms = frames[-1]["t_ms"] if frames else 0
    first_ms = frames[0]["t_ms"] if frames else 0
    n_frames = len(frames)

    # ── Resolve wall-clock start / first-frame, with graceful fallbacks ─────────
    def _parse_wall(s):
        if not s:
            return None
        for f in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
            try:
                return datetime.strptime(s, f)
            except Exception:
                continue
        return None

    tm = test_meta or {}
    test_start_dt = _parse_wall(tm.get("test_start_wall"))
    test_end_dt   = _parse_wall(tm.get("test_end_wall"))
    anchor        = tm.get("anchor") or {}
    anchor_dt     = _parse_wall(anchor.get("wall"))

    # ── Wall-clock anchoring ───────────────────────────────────────────────────
    # The .trc only timestamps relative to its first frame. To place frames on a
    # real wall-clock / test-time axis we need one known (frame ↔ wall) anchor.
    # Priority: explicit observed anchor in meta  →  log header comment  →  filename.
    first_frame_wall = None
    if anchor_dt is not None:
        if anchor.get("frame", "first") == "last":
            first_frame_wall = anchor_dt - timedelta(milliseconds=total_ms)
        else:
            first_frame_wall = anchor_dt - timedelta(milliseconds=first_ms)
    if first_frame_wall is None:
        first_frame_wall = _parse_wall(meta.get("first_frame_wall"))
    log_start_dt = _parse_wall(meta.get("log_start_wall")) or meta.get("filename_wall")
    if first_frame_wall is None and log_start_dt is not None:
        first_frame_wall = log_start_dt + timedelta(milliseconds=first_ms)

    # Axis origin: test start if known, else the first frame.
    axis_origin = test_start_dt or first_frame_wall
    # frame_offset_ms shifts log-relative t (from first frame) onto the axis origin.
    frame_offset_ms = 0.0
    if axis_origin is not None and first_frame_wall is not None:
        frame_offset_ms = (first_frame_wall - axis_origin).total_seconds() * 1000

    axis_origin_epoch = (axis_origin.timestamp() * 1000) if axis_origin else 0
    wall_axis = axis_origin is not None

    fmt = "%Y-%m-%d %H:%M:%S.%f"
    log_start_str = log_start_dt.strftime(fmt)[:-3]     if log_start_dt     else "—"
    first_str     = first_frame_wall.strftime(fmt)[:-3] if first_frame_wall else "—"

    # Wall-clock gap (log start → first frame)
    gap_str = ""
    if log_start_dt and first_frame_wall:
        gap_str = f"{(first_frame_wall - log_start_dt).total_seconds():.1f}s"

    # Assign a color and index to each (proto, pid) key
    pid_keys = sorted(stats.keys(), key=lambda k: (k[0], k[1]))
    pid_color = {k: COLORS[i % len(COLORS)] for i, k in enumerate(pid_keys)}
    pid_index = {k: i for i, k in enumerate(pid_keys)}

    # Timeline data for JS  [{pid_idx, t_req, t_resp|null, label}]
    # All frame times are shifted by frame_offset_ms onto the axis origin.
    tl_data = []
    for req_f, resp_f, dec in pairs:
        key = (dec["proto"], dec["pid"])
        tl_data.append({
            "i":   pid_index.get(key, 0),
            "t":   req_f["t_ms"] + frame_offset_ms,
            "r":   (resp_f["t_ms"] + frame_offset_ms) if resp_f else None,
            "lbl": dec["label"],
            "c":   pid_color.get(key, "#888"),
        })

    # Stats table rows
    stats_rows = ""
    for key in pid_keys:
        s = stats[key]
        avg = (s["total_rt_ms"] / s["resp"]) if s["resp"] else None
        avg_str  = f"{avg:.1f}" if avg is not None else "—"
        min_str  = f"{s['min_rt_ms']:.1f}" if s["min_rt_ms"] is not None else "—"
        max_str  = f"{s['max_rt_ms']:.1f}" if s["max_rt_ms"] is not None else "—"
        rate     = f"{s['resp']/s['req']*100:.0f}%" if s["req"] else "—"
        dot      = f'<span style="display:inline-block;width:12px;height:12px;border-radius:50%;background:{pid_color[key]};margin-right:6px"></span>'
        stats_rows += f"""
        <tr>
          <td>{dot}{s['label']}</td>
          <td class="c">{s['proto']}</td>
          <td class="c">{s['req']}</td>
          <td class="c">{s['resp']}</td>
          <td class="c">{s['no_resp']}</td>
          <td class="c">{rate}</td>
          <td class="c">{min_str}</td>
          <td class="c">{avg_str}</td>
          <td class="c">{max_str}</td>
        </tr>"""

    # Overlay events (vertical markers) from the test meta.
    # Events are positioned in axis time: "min" = minutes from axis origin
    # (test start); legacy "t_ms" = ms from the first CAN frame.
    overlay_events = (test_meta or {}).get("overlay_events", [])
    def _ev_pos(e):
        if "min" in e:
            return e["min"] * 60000.0
        return e.get("t_ms", 0) + frame_offset_ms
    ev_data = [{"t": _ev_pos(e),
                "lbl": e.get("label", ""),
                "c":   e.get("color", "#ffd54f")}
               for e in overlay_events]
    # Auto marker delineating where real CAN data ends (logging continues after).
    last_frame_pos = total_ms + frame_offset_ms
    ev_data.append({"t": last_frame_pos, "lbl": "Last CAN frame", "c": "#8895aa"})

    # Event legend (numbered, with wall-clock time) — keeps the canvas uncluttered.
    def _ev_wall(pos):
        if axis_origin is not None:
            return (axis_origin + timedelta(milliseconds=pos)).strftime("%H:%M:%S")
        return f"+{pos/60000:.1f}m"
    legend_items = "".join(
        f'<span class="ev-item">'
        f'<span class="ev-num" style="background:{e["c"]}">{i}</span>'
        f'<span class="ev-t">{_ev_wall(e["t"])}</span>{e["lbl"]}</span>'
        for i, e in enumerate(ev_data, 1)
    )
    events_legend_html = (f'<div class="ev-legend">{legend_items}</div>'
                          if ev_data else "")

    # Timeline domain: the test runs long after CAN stops, so the x-axis must
    # span the whole test window (not just the last CAN frame).
    event_max = max((e["t"] for e in ev_data), default=0)
    if test_start_dt and test_end_dt:
        test_total_ms = (test_end_dt - test_start_dt).total_seconds() * 1000
    else:
        test_total_ms = (test_meta or {}).get("total_time_min", 0) * 60000
    domain_ms = max(last_frame_pos, event_max, test_total_ms)

    # Shaded "no CAN" region (last frame → end of test window)
    nocan_start_ms = last_frame_pos

    # Elapsed-from-origin (test start) for the summary card, before any view shift
    last_frame_elapsed_min = last_frame_pos / 60000

    # ── Optional view clip window (wall clock) ──────────────────────────────────
    # "view_start_wall"/"view_end_wall" in the meta clip the visible x-axis to a
    # specific wall-clock window. All positions are shifted so the window starts
    # at 0 and AXIS_ORIGIN is moved to the window start (keeps fmtWall correct).
    view_start_dt = _parse_wall(tm.get("view_start_wall"))
    view_end_dt   = _parse_wall(tm.get("view_end_wall"))
    view0_ms = ((view_start_dt - axis_origin).total_seconds() * 1000
                if (axis_origin is not None and view_start_dt is not None) else 0.0)
    if axis_origin is not None and view_end_dt is not None:
        view_end_ms = (view_end_dt - axis_origin).total_seconds() * 1000
    else:
        view_end_ms = domain_ms + view0_ms

    if view0_ms != 0.0:
        for d in tl_data:
            d["t"] -= view0_ms
            if d["r"] is not None:
                d["r"] -= view0_ms
        for e in ev_data:
            e["t"] -= view0_ms
        last_frame_pos -= view0_ms
        nocan_start_ms -= view0_ms
        axis_origin_epoch += view0_ms
    domain_ms = view_end_ms - view0_ms

    # (Re)serialize after any view shift
    tl_json = json.dumps(tl_data)
    ev_json = json.dumps(ev_data)

    n_lanes  = len(pid_keys)
    lane_labels = json.dumps([stats[k]["label"] for k in pid_keys])
    lane_colors = json.dumps([pid_color[k] for k in pid_keys])

    duration_s = total_ms / 1000
    h = int(duration_s // 3600)
    m = int((duration_s % 3600) // 60)
    s = duration_s % 60
    dur_str = f"{h}h {m}m {s:.1f}s" if h else f"{m}m {s:.1f}s"

    # Wall-clock strings for the summary cards
    last_frame_wall = (first_frame_wall + timedelta(milliseconds=total_ms)) if first_frame_wall else None
    last_frame_card = (f"{last_frame_wall.strftime('%H:%M:%S')} · +{last_frame_elapsed_min:.1f}m"
                       if last_frame_wall else dur_str)
    if test_start_dt and test_end_dt:
        test_window_card = f"{test_start_dt.strftime('%H:%M')} → {test_end_dt.strftime('%H:%M')}"
    else:
        test_window_card = f"{int(domain_ms//60000)}m {domain_ms%60000/1000:.0f}s"

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>CAN Log Report — {meta['file']}</title>
<style>
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{ font-family: 'Segoe UI', system-ui, sans-serif; background: #0f1117; color: #dde; font-size: 13px; }}
  h1   {{ font-size: 1.3em; font-weight: 600; color: #7ecfff; }}
  h2   {{ font-size: 1em; font-weight: 600; color: #aac8e8; margin: 1.4em 0 .5em; text-transform: uppercase; letter-spacing: .06em; }}
  .page  {{ max-width: 1400px; margin: 0 auto; padding: 24px 20px 60px; }}
  .header {{ display: flex; align-items: baseline; gap: 18px; margin-bottom: 20px; border-bottom: 1px solid #2a3040; padding-bottom: 14px; }}
  .fname  {{ color: #888; font-size: .85em; }}

  /* Session cards */
  .cards {{ display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 22px; }}
  .card  {{ background: #1a1f2e; border: 1px solid #2a3344; border-radius: 8px;
             padding: 10px 16px; min-width: 160px; }}
  .card .k {{ color: #778; font-size: .78em; text-transform: uppercase; letter-spacing: .05em; margin-bottom: 3px; }}
  .card .v {{ color: #e8eeff; font-size: 1.05em; font-weight: 600; }}

  /* Stats table */
  table {{ width: 100%; border-collapse: collapse; }}
  th    {{ background: #1a2030; color: #8aaccc; font-weight: 600; padding: 7px 10px;
            text-align: left; border-bottom: 1px solid #2a3344; font-size: .82em;
            text-transform: uppercase; letter-spacing: .04em; }}
  td    {{ padding: 6px 10px; border-bottom: 1px solid #1e2435; color: #ccd; }}
  td.c  {{ text-align: right; color: #aab8cc; }}
  tr:hover td {{ background: #1e2535; }}

  /* Timeline */
  .tl-wrap {{ background: #12151f; border: 1px solid #2a3344; border-radius: 8px;
               overflow: hidden; margin-bottom: 8px; }}
  .tl-toolbar {{ display: flex; gap: 8px; align-items: center; padding: 8px 12px;
                  background: #1a1f2e; border-bottom: 1px solid #2a3344; }}
  .tl-toolbar button {{ background: #2a3450; border: 1px solid #3a4560; color: #aac;
                         padding: 3px 10px; border-radius: 4px; cursor: pointer; font-size: .82em; }}
  .tl-toolbar button:hover {{ background: #3a4566; }}
  .tl-toolbar span {{ color: #668; font-size: .8em; }}
  #tl-canvas {{ display: block; cursor: grab; }}
  #tl-canvas:active {{ cursor: grabbing; }}
  .tl-tooltip {{ position: fixed; background: #1e2840; border: 1px solid #4a5a80;
                  border-radius: 6px; padding: 6px 10px; font-size: .8em; color: #cde;
                  pointer-events: none; display: none; z-index: 999; max-width: 280px; line-height: 1.5; }}

  /* Event legend */
  .ev-legend {{ display: flex; flex-wrap: wrap; gap: 8px 18px; margin: 0 0 14px;
                padding: 10px 14px; background: #12151f; border: 1px solid #2a3344;
                border-radius: 8px; }}
  .ev-item   {{ display: inline-flex; align-items: center; gap: 7px; font-size: .85em; color: #cdd; }}
  .ev-num    {{ display: inline-flex; align-items: center; justify-content: center;
                width: 18px; height: 18px; border-radius: 50%; color: #0b0e16;
                font-weight: 700; font-size: .8em; flex: none; }}
  .ev-t      {{ color: #9fb0c8; font-family: monospace; font-size: .95em; }}
</style>
</head>
<body>
<div class="page">

  <div class="header">
    <h1>CAN Log Analysis Report</h1>
    <span class="fname">{meta['file']}</span>
  </div>

  {build_test_config_html(test_meta)}

  <!-- Session summary cards -->
  <div class="cards">
    <div class="card"><div class="k">Log Start</div>
      <div class="v" style="font-size:.85em">{log_start_str}</div></div>
    <div class="card"><div class="k">First Frame</div>
      <div class="v" style="font-size:.85em">{first_str}</div></div>
    <div class="card"><div class="k">Start → First Frame</div>
      <div class="v">{gap_str or '—'}</div></div>
    <div class="card"><div class="k">Last CAN Frame</div>
      <div class="v" style="font-size:.9em">{last_frame_card}</div></div>
    <div class="card"><div class="k">Test Window</div>
      <div class="v">{test_window_card}</div></div>
    <div class="card"><div class="k">Total Frames</div>
      <div class="v">{n_frames:,}</div></div>
    <div class="card"><div class="k">Unique PIDs / DIDs</div>
      <div class="v">{n_lanes}</div></div>
    <div class="card"><div class="k">Req/Resp Pairs</div>
      <div class="v">{sum(1 for _,r,_ in pairs if r)}</div></div>
  </div>

  <!-- Timeline -->
  <h2>Request / Response Timeline</h2>
  {events_legend_html}
  <div class="tl-wrap">
    <div class="tl-toolbar">
      <button onclick="zoom(1.5)">＋ Zoom In</button>
      <button onclick="zoom(1/1.5)">－ Zoom Out</button>
      <button onclick="resetView()">Reset</button>
      <span>Axis = wall clock · scroll/drag to pan · green = response · red = no response · shaded = no CAN</span>
    </div>
    <canvas id="tl-canvas"></canvas>
  </div>
  <div class="tl-tooltip" id="tooltip"></div>

  <!-- Stats table -->
  <h2>PID / DID Summary</h2>
  <table>
    <thead><tr>
      <th>Signal</th><th>Protocol</th>
      <th style="text-align:right">Requests</th>
      <th style="text-align:right">Responses</th>
      <th style="text-align:right">No Resp</th>
      <th style="text-align:right">Rate</th>
      <th style="text-align:right">Min RT (ms)</th>
      <th style="text-align:right">Avg RT (ms)</th>
      <th style="text-align:right">Max RT (ms)</th>
    </tr></thead>
    <tbody>{stats_rows}</tbody>
  </table>

</div>

<div class="tl-tooltip" id="tooltip2"></div>

<script>
const TL   = {tl_json};
const EVENTS = {ev_json};
const LBLS = {lane_labels};
const CLRS = {lane_colors};
const TOTAL_MS   = {domain_ms:.1f};   // full test window (x-axis domain, ms from origin)
const DATA_END   = {last_frame_pos:.1f};     // last actual CAN frame (axis ms)
const NOCAN_FROM = {nocan_start_ms:.1f};
const AXIS_ORIGIN = {axis_origin_epoch:.0f}; // epoch ms at axis position 0
const WALL_AXIS   = {str(wall_axis).lower()};
const N_LANES  = {n_lanes};

const LANE_H  = 28;
const LABEL_W = 220;
const EV_BAND = EVENTS.length ? 22 : 0;   // top band for numbered event badges
const PAD_TOP = 30 + EV_BAND;             // axis header height (+ event band)
const MARK_R  = 4;

const canvas = document.getElementById('tl-canvas');
const tip    = document.getElementById('tooltip');
let   scale  = 1;   // px per ms
let   panX   = 0;   // scroll offset in px

function initScale() {{
  const W = canvas.parentElement.clientWidth - LABEL_W - 2;
  scale = W / TOTAL_MS;
}}

function resize() {{
  canvas.width  = canvas.parentElement.clientWidth;
  canvas.height = PAD_TOP + N_LANES * LANE_H + 10;
  draw();
}}

function draw() {{
  const ctx = canvas.getContext('2d');
  const W = canvas.width;
  const H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  // Background
  ctx.fillStyle = '#12151f';
  ctx.fillRect(0, 0, W, H);

  // Label area background
  ctx.fillStyle = '#171c2a';
  ctx.fillRect(0, 0, LABEL_W, H);

  // Grid lines & lane backgrounds
  for (let i = 0; i < N_LANES; i++) {{
    const y = PAD_TOP + i * LANE_H;
    ctx.fillStyle = i % 2 === 0 ? '#131825' : '#12151f';
    ctx.fillRect(LABEL_W, y, W - LABEL_W, LANE_H);

    // Label
    ctx.fillStyle = CLRS[i];
    ctx.font = '10px monospace';
    ctx.fillText(LBLS[i], 6, y + LANE_H/2 + 4);

    // Separator
    ctx.strokeStyle = '#1e2535';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, y + LANE_H); ctx.lineTo(W, y + LANE_H); ctx.stroke();
  }}

  // Event band (top) + time axis
  if (EV_BAND) {{
    ctx.fillStyle = '#161b29';
    ctx.fillRect(LABEL_W, 0, W - LABEL_W, EV_BAND);
    ctx.strokeStyle = '#2a3344'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(LABEL_W, EV_BAND); ctx.lineTo(W, EV_BAND); ctx.stroke();
  }}
  ctx.fillStyle = '#1a1f2e';
  ctx.fillRect(LABEL_W, EV_BAND, W - LABEL_W, PAD_TOP - EV_BAND);
  ctx.strokeStyle = '#2a3344'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(LABEL_W, PAD_TOP); ctx.lineTo(W, PAD_TOP); ctx.stroke();

  // Time ticks
  const viewW = W - LABEL_W;
  const visMs = viewW / scale;
  const tick  = niceInterval(visMs / 8);
  const t0vis = -panX / scale;
  const t1vis = t0vis + visMs;
  ctx.fillStyle = '#556';
  ctx.font = '9px sans-serif';
  ctx.textAlign = 'center';
  for (let t = Math.ceil(t0vis / tick) * tick; t <= t1vis; t += tick) {{
    const x = LABEL_W + t * scale + panX;
    ctx.strokeStyle = '#1e2840'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, PAD_TOP); ctx.lineTo(x, H); ctx.stroke();
    ctx.fillStyle = '#778';
    ctx.fillText(WALL_AXIS ? fmtWall(t) : fmtT(t), x, PAD_TOP - 4);
  }}
  ctx.textAlign = 'left';

  // "No CAN traffic" region (after the last frame, test still running)
  if (TOTAL_MS > DATA_END + 1) {{
    const xs = LABEL_W + NOCAN_FROM * scale + panX;
    const xe = LABEL_W + TOTAL_MS  * scale + panX;
    const x0 = Math.max(xs, LABEL_W);
    const x1 = Math.min(xe, W);
    if (x1 > x0) {{
      ctx.fillStyle = 'rgba(224,80,80,0.06)';
      ctx.fillRect(x0, PAD_TOP, x1 - x0, H - PAD_TOP);
      // diagonal hatch
      ctx.strokeStyle = 'rgba(224,80,80,0.10)'; ctx.lineWidth = 1;
      for (let x = x0; x < x1; x += 10) {{
        ctx.beginPath(); ctx.moveTo(x, PAD_TOP); ctx.lineTo(Math.min(x + (H-PAD_TOP), x1), H); ctx.stroke();
      }}
      if (x1 - x0 > 90) {{
        ctx.fillStyle = 'rgba(230,120,120,0.7)';
        ctx.font = 'italic 11px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('no CAN traffic — logging/test still running',
                     (x0 + x1) / 2, PAD_TOP + (H - PAD_TOP) / 2);
        ctx.textAlign = 'left';
      }}
    }}
  }}

  // Marks
  for (const ev of TL) {{
    const x = LABEL_W + ev.t * scale + panX;
    if (x < LABEL_W - 10 || x > W + 10) continue;
    const y = PAD_TOP + ev.i * LANE_H + LANE_H / 2;

    // Response line
    if (ev.r !== null) {{
      const xr = LABEL_W + ev.r * scale + panX;
      ctx.strokeStyle = '#2a6640'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(xr, y); ctx.stroke();
      // Response dot
      ctx.fillStyle = '#4caf80';
      ctx.beginPath(); ctx.arc(xr, y, MARK_R - 1, 0, Math.PI*2); ctx.fill();
    }}

    // Request dot
    ctx.fillStyle = ev.r !== null ? ev.c : '#e05050';
    ctx.beginPath(); ctx.arc(x, y, MARK_R, 0, Math.PI*2); ctx.fill();
  }}

  // Event overlays: vertical dashed lines + small numbered badges (text in HTML legend)
  // Vertical lines first
  for (let k = 0; k < EVENTS.length; k++) {{
    const ev = EVENTS[k];
    const x  = LABEL_W + ev.t * scale + panX;
    if (x < LABEL_W - 0.5 || x > W + 0.5) continue;
    ctx.strokeStyle = ev.c; ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.moveTo(x, EV_BAND - 2); ctx.lineTo(x, H); ctx.stroke();
    ctx.setLineDash([]);
  }}
  // Numbered badges, nudged sideways when several share the same x
  ctx.font = 'bold 10px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  const placed = [];
  for (let k = 0; k < EVENTS.length; k++) {{
    const ev = EVENTS[k];
    const x  = LABEL_W + ev.t * scale + panX;
    if (x < LABEL_W - 0.5 || x > W + 0.5) continue;
    let level = 0;
    for (const p of placed) {{ if (Math.abs(p - (x)) < 18) level++; }}
    placed.push(x);
    let bx = x + level * 16;
    if (bx + 9 > W) bx = x - 9 - level * 16;   // flip near right edge
    const by = EV_BAND / 2;
    ctx.fillStyle = ev.c;
    ctx.beginPath(); ctx.arc(bx, by, 8, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#0b0e16';
    ctx.fillText(String(k + 1), bx, by + 0.5);
  }}
  ctx.textAlign = 'left';
  ctx.textBaseline = 'alphabetic';

  // Label column border
  ctx.strokeStyle = '#2a3560'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(LABEL_W, 0); ctx.lineTo(LABEL_W, H); ctx.stroke();
}}

function niceInterval(approx) {{
  const p = Math.pow(10, Math.floor(Math.log10(approx)));
  for (const f of [1, 2, 5, 10]) {{ if (f * p >= approx) return f * p; }}
  return 10 * p;
}}

function fmtT(ms) {{
  if (ms >= 60000) return (ms/60000).toFixed(1) + 'm';
  if (ms >= 1000)  return (ms/1000).toFixed(1) + 's';
  return ms.toFixed(0) + 'ms';
}}

function pad2(n) {{ return n < 10 ? '0' + n : '' + n; }}

// Wall-clock label for an axis position (ms from origin) → "HH:MM:SS"
function fmtWall(ms) {{
  const d = new Date(AXIS_ORIGIN + ms);
  return pad2(d.getHours()) + ':' + pad2(d.getMinutes()) + ':' + pad2(d.getSeconds());
}}

function zoom(factor) {{
  const cx = (canvas.width - LABEL_W) / 2;
  const tAtCx = (cx - panX) / scale;
  scale *= factor;
  panX = cx - tAtCx * scale;
  clampPan(); draw();
}}

function resetView() {{
  initScale(); panX = 0; draw();
}}

function clampPan() {{
  const viewW = canvas.width - LABEL_W;
  const maxPan = 0;
  const minPan = -(TOTAL_MS * scale - viewW);
  panX = Math.max(Math.min(panX, maxPan), Math.min(minPan, 0));
}}

// Drag / scroll
let dragX = null;
canvas.addEventListener('mousedown', e => {{ dragX = e.clientX; }});
window.addEventListener('mousemove', e => {{
  if (dragX !== null) {{
    panX += e.clientX - dragX; dragX = e.clientX;
    clampPan(); draw();
  }}
  // Tooltip
  const rect = canvas.getBoundingClientRect();
  const mx = e.clientX - rect.left;
  const my = e.clientY - rect.top;
  if (mx > LABEL_W && my > PAD_TOP) {{
    const lane = Math.floor((my - PAD_TOP) / LANE_H);
    const tCur = (mx - LABEL_W - panX) / scale;
    let best = null, bestD = 8 / scale;
    for (const ev of TL) {{
      if (ev.i !== lane) continue;
      const d = Math.abs(ev.t - tCur);
      if (d < bestD) {{ bestD = d; best = ev; }}
    }}
    if (best) {{
      const rt = best.r !== null ? (best.r - best.t).toFixed(1) + ' ms' : 'no response';
      const tstr = WALL_AXIS ? fmtWall(best.t) : fmtT(best.t);
      tip.innerHTML = '<b>' + best.lbl + '</b><br>t = ' + tstr + '<br>RT = ' + rt;
      tip.style.display = 'block';
      tip.style.left = (e.clientX + 12) + 'px';
      tip.style.top  = (e.clientY - 10) + 'px';
    }} else {{ tip.style.display = 'none'; }}
  }} else {{ tip.style.display = 'none'; }}
}});
window.addEventListener('mouseup', () => {{ dragX = null; }});
canvas.addEventListener('wheel', e => {{
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const cx = e.clientX - rect.left - LABEL_W;
  const tAtCx = (cx - panX) / scale;
  scale *= e.deltaY < 0 ? 1.15 : 1/1.15;
  panX = cx - tAtCx * scale;
  clampPan(); draw();
}}, {{ passive: false }});

window.addEventListener('resize', () => {{ initScale(); panX = 0; resize(); }});
initScale(); resize();
</script>
</body>
</html>
"""

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    if len(sys.argv) > 1:
        trc_path = Path(sys.argv[1])
    else:
        log_dir = Path(__file__).parent / "logs"
        trcs = sorted(log_dir.glob("*.trc"), key=lambda p: p.stat().st_mtime, reverse=True)
        if not trcs:
            sys.exit("No .trc files found in ./logs/")
        trc_path = trcs[0]
        print(f"Auto-selected: {trc_path.name}")

    # Load optional test metadata sidecar (.meta.json)
    test_meta = None
    meta_path = trc_path.with_suffix(".meta.json")
    if meta_path.exists():
        import json as _json
        test_meta = _json.loads(meta_path.read_text())
        print(f"  Test config: {meta_path.name}")

    print(f"Parsing {trc_path} …")
    meta, frames = parse_trc(trc_path)
    print(f"  {len(frames):,} frames")

    pairs, stats = match_pairs(frames)
    print(f"  {len(pairs):,} req/resp pairs across {len(stats)} PIDs/DIDs")

    out_path = trc_path.with_suffix(".html")
    html = build_html(meta, frames, pairs, stats, test_meta=test_meta)
    out_path.write_text(html, encoding="utf-8")
    print(f"  → {out_path}")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
CAN Logger for macOS — PCAN-USB via PCBUSB driver.

Setup:
  1. Install PCBUSB library:  https://mac-can.github.io/drivers/libPCBUSB.html
  2. pip install -r requirements.txt
  3. python can_logger.py           ← interactive setup wizard
     python can_logger.py --help    ← all CLI flags
     python can_logger.py --scan    ← detect devices only
"""

import argparse
import ctypes
import signal
import sys
import threading
import time
from collections import deque
from datetime import datetime, timedelta
from pathlib import Path

import can
from can import Bus, BusState, Logger, Notifier

from rich import box
from rich.align import Align
from rich.columns import Columns
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

console = Console()

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

BITRATES = [5_000, 10_000, 20_000, 50_000, 100_000, 125_000, 250_000,
            500_000, 800_000, 1_000_000]

BITRATE_LABELS = {
    5_000:       "  5 kbit/s",
    10_000:      " 10 kbit/s",
    20_000:      " 20 kbit/s",
    50_000:      " 50 kbit/s",
    100_000:     "100 kbit/s",
    125_000:     "125 kbit/s",
    250_000:     "250 kbit/s",
    500_000:     "500 kbit/s",
    800_000:     "800 kbit/s",
    1_000_000:   "  1 Mbit/s",
}

LOG_FORMATS = {
    ".trc": "TRC  — PEAK PCAN-View / Explorer format",
    ".asc": "ASC  — Vector / CANalyzer format",
    ".csv": "CSV  — comma-separated, opens in Excel",
    ".log": "LOG  — python-can plain text",
    ".blf": "BLF  — Vector binary format",
}

CHANNELS = [f"PCAN_USBBUS{i}" for i in range(1, 9)]

PCAN_CHANNEL_AVAILABLE = 0x01
PCAN_CHANNEL_OCCUPIED  = 0x02
PCAN_CHANNEL_CONDITION = 0x08

MAX_DISPLAY_ROWS = 200   # messages kept in the rolling buffer


# ──────────────────────────────────────────────────────────────────────────────
# Device detection
# ──────────────────────────────────────────────────────────────────────────────

def _load_pcbusb():
    for path in ("/usr/local/lib/libPCBUSB.dylib", "libPCBUSB.dylib"):
        try:
            return ctypes.cdll.LoadLibrary(path)
        except OSError:
            pass
    return None


_PCAN_HANDLES = {
    "PCAN_USBBUS1": 0x51, "PCAN_USBBUS2": 0x52,
    "PCAN_USBBUS3": 0x53, "PCAN_USBBUS4": 0x54,
    "PCAN_USBBUS5": 0x55, "PCAN_USBBUS6": 0x56,
    "PCAN_USBBUS7": 0x57, "PCAN_USBBUS8": 0x58,
}


def scan_devices() -> list[dict]:
    lib = _load_pcbusb()
    results = []
    for ch_name, handle in _PCAN_HANDLES.items():
        entry = {"channel": ch_name, "condition": "unavailable"}
        if lib is not None:
            buf = ctypes.c_uint32(0)
            ret = lib.CAN_GetValue(
                ctypes.c_uint16(handle),
                ctypes.c_uint8(PCAN_CHANNEL_CONDITION),
                ctypes.byref(buf),
                ctypes.c_uint32(4),
            )
            if ret == 0:
                cond = buf.value & 0x03
                if cond & PCAN_CHANNEL_AVAILABLE:
                    entry["condition"] = "available"
                elif cond & PCAN_CHANNEL_OCCUPIED:
                    entry["condition"] = "occupied"
        results.append(entry)
    return results


def print_scan_results(devices: list[dict]) -> list[dict]:
    console.print()
    table = Table(box=box.SIMPLE, show_header=True, header_style="bold")
    table.add_column("Channel", style="cyan")
    table.add_column("Status")
    found = []
    for d in devices:
        cond = d["condition"]
        if cond == "available":
            table.add_row(d["channel"], "[green]✓  Available[/green]")
            found.append(d)
        elif cond == "occupied":
            table.add_row(d["channel"], "[yellow]⚠  In use by another process[/yellow]")
            found.append(d)
    if not found:
        console.print("  [red]No PCAN-USB devices detected.[/red]")
        console.print("  • Is the adapter plugged in?")
        console.print("  • Is libPCBUSB.dylib at /usr/local/lib/?")
    else:
        console.print(table)
    console.print()
    return found


# ──────────────────────────────────────────────────────────────────────────────
# Interactive wizard
# ──────────────────────────────────────────────────────────────────────────────

def _pick(prompt: str, options: list, labels: list[str], default: int = 0) -> int:
    console.print(f"\n[bold]{prompt}[/bold]")
    for i, label in enumerate(labels):
        marker = "  [dim](default)[/dim]" if i == default else ""
        console.print(f"  [cyan]{i+1:2}.[/cyan]  {label}{marker}")
    while True:
        raw = input(f"\n  Choice [1-{len(options)}] (Enter = {default+1}): ").strip()
        if raw == "":
            return default
        if raw.isdigit() and 1 <= int(raw) <= len(options):
            return int(raw) - 1
        console.print("  [red]Please enter a number in range.[/red]")


def _confirm(prompt: str, default: bool = True) -> bool:
    hint = "Y/n" if default else "y/N"
    raw = input(f"  {prompt} [{hint}]: ").strip().lower()
    return default if raw == "" else raw.startswith("y")


def wizard() -> argparse.Namespace:
    console.rule("[bold cyan]CAN Logger — Setup Wizard[/bold cyan]")

    # device
    console.print("\n  [bold]Scanning for PCAN-USB devices...[/bold]")
    devices = scan_devices()
    found = print_scan_results(devices)
    if not found:
        sys.exit(1)

    if len(found) == 1:
        channel = found[0]["channel"]
        console.print(f"  Using: [cyan]{channel}[/cyan]\n")
    else:
        idx = _pick("Select channel:",
                    [d["channel"] for d in found],
                    [d["channel"] for d in found])
        channel = found[idx]["channel"]

    # bitrate
    default_br = BITRATES.index(500_000)
    br_idx = _pick("Bit-rate:", BITRATES,
                   [BITRATE_LABELS[b] for b in BITRATES], default=default_br)
    bitrate = BITRATES[br_idx]

    # mode
    console.print()
    passive = _confirm(
        "Listen-only / passive mode? (recommended — no ACK sent to bus)",
        default=True,
    )

    # CAN FD
    fd = _confirm("CAN FD mode? (requires PCAN-USB FD hardware)", default=False)

    # log format
    fmt_keys = list(LOG_FORMATS.keys())
    fmt_idx = _pick("Log file format:", fmt_keys,
                    [f"[bold]{k}[/bold]  {v}" for k, v in LOG_FORMATS.items()],
                    default=0)
    fmt = fmt_keys[fmt_idx]

    # output
    console.print()
    raw = input("  Output filename (Enter = auto timestamp): ").strip()
    output = raw if raw else None

    # duration
    raw = input("  Auto-stop after N seconds (Enter = run until Ctrl+C): ").strip()
    duration = float(raw) if raw.replace(".", "", 1).isdigit() else None

    console.print()
    return argparse.Namespace(
        channel=channel, bitrate=bitrate, passive=passive, fd=fd,
        fmt=fmt, output=output, no_log=False, duration=duration,
    )


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="CAN Logger for macOS — PCAN-USB via PCBUSB driver",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Run without arguments for the interactive setup wizard.

Examples:
  python can_logger.py                       # wizard
  python can_logger.py --scan                # list attached devices and exit
  python can_logger.py -b 250000 --passive   # 250 kbit/s listen-only
  python can_logger.py -c PCAN_USBBUS2 -f .asc
  python can_logger.py --no-log              # screen only, no file
""",
    )
    p.add_argument("--scan", action="store_true",
                   help="Scan for PCAN-USB devices and exit")
    p.add_argument("-c", "--channel", default=None, choices=CHANNELS,
                   metavar="CHANNEL", help="PCAN USB channel (default: wizard)")
    p.add_argument("-b", "--bitrate", type=int, default=500_000,
                   choices=BITRATES, metavar="BITRATE",
                   help="CAN bit-rate in bit/s (default: 500000)")
    p.add_argument("--passive", action="store_true",
                   help="Listen-only (no ACK sent) — default when using wizard")
    p.add_argument("--active", action="store_true",
                   help="Active mode — sends ACK (overrides --passive)")
    p.add_argument("--fd", action="store_true",
                   help="CAN FD mode (PCAN-USB FD required)")
    p.add_argument("-f", "--format", dest="fmt", default=".trc",
                   choices=list(LOG_FORMATS.keys()), metavar="FORMAT",
                   help="Log format: .trc .asc .csv .log .blf (default: .trc)")
    p.add_argument("-o", "--output", default=None, metavar="FILE",
                   help="Output filename base (timestamp appended if omitted)")
    p.add_argument("--no-log", action="store_true",
                   help="No log file; screen display only")
    p.add_argument("--duration", type=float, default=None, metavar="SECONDS",
                   help="Stop automatically after N seconds")
    return p.parse_args()


# ──────────────────────────────────────────────────────────────────────────────
# Live display
# ──────────────────────────────────────────────────────────────────────────────

class CANDisplay:
    """Thread-safe live display. Call add_message() from any thread."""

    def __init__(self, channel: str, bitrate: int, passive: bool,
                 fd: bool, out_path: Path | None):
        self.channel  = channel
        self.bitrate  = bitrate
        self.passive  = passive
        self.fd       = fd
        self.out_path = out_path

        self._lock     = threading.Lock()
        self._msgs     = deque(maxlen=MAX_DISPLAY_ROWS)
        self._count    = 0
        self._start    = time.monotonic()
        self._rate     = 0.0
        self._rate_buf = 0
        self._rate_ts  = time.monotonic()
        self._bus_ok   = True

    def add_message(self, msg: can.Message):
        with self._lock:
            self._msgs.append(msg)
            self._count += 1
            self._rate_buf += 1
            now = time.monotonic()
            dt = now - self._rate_ts
            if dt >= 0.5:
                self._rate = self._rate_buf / dt
                self._rate_buf = 0
                self._rate_ts = now

    def set_bus_error(self, ok: bool):
        self._bus_ok = ok

    # ── renderable ──────────────────────────────────────────────────────────

    def _header(self) -> Panel:
        mode_txt  = ("[yellow]PASSIVE — listen only[/yellow]" if self.passive
                     else "[green]ACTIVE — sending ACK[/green]")
        fd_txt    = "  [blue]CAN FD[/blue]" if self.fd else ""
        br_txt    = BITRATE_LABELS.get(self.bitrate, f"{self.bitrate} bit/s")
        status    = "[green]● ONLINE[/green]" if self._bus_ok else "[red]● ERROR[/red]"

        left  = Text.assemble(
            ("  ", ""),
            (self.channel, "bold cyan"),
            ("   │   ", "dim"),
            (br_txt.strip(), "bold white"),
            (fd_txt, ""),
            ("   │   ", "dim"),
            (mode_txt, ""),
        )
        right = Text(f"{status}  ", justify="right")
        return Panel(
            Columns([left, Align(right, align="right")]),
            title="[bold]CAN Logger[/bold]",
            border_style="cyan",
        )

    def _table(self) -> Table:
        t = Table(
            box=box.SIMPLE_HEAD,
            show_header=True,
            header_style="bold dim",
            expand=True,
            show_edge=False,
            padding=(0, 1),
        )
        t.add_column("  Time (s)",   style="dim",        width=14, no_wrap=True)
        t.add_column("ID",           style="cyan",        width=10, no_wrap=True)
        t.add_column("DL",           justify="right",     width=3,  no_wrap=True)
        t.add_column("Data",         style="white",       ratio=1,  no_wrap=True)
        t.add_column("Flags",        style="dim yellow",  width=12, no_wrap=True)

        with self._lock:
            msgs = list(self._msgs)

        for msg in msgs:
            # ID
            if msg.is_error_frame:
                id_str = Text(f"[ERR]", style="bold red")
            elif msg.is_extended_id:
                id_str = Text(f"{msg.arbitration_id:08X}", style="bold cyan")
            else:
                id_str = Text(f"     {msg.arbitration_id:04X}", style="cyan")

            # Data
            if msg.is_remote_frame:
                data_str = Text("(remote frame)", style="dim yellow")
            elif msg.data:
                raw = " ".join(f"{b:02X}" for b in msg.data)
                data_str = Text(raw)
            else:
                data_str = Text("", style="dim")

            # Flags
            flags = []
            if msg.is_extended_id:   flags.append("XTD")
            if msg.is_remote_frame:  flags.append("RTR")
            if msg.is_fd:            flags.append("FD")
            if msg.is_error_frame:   flags.append("ERR")
            flags_str = " ".join(flags)

            # Row style
            row_style = ""
            if msg.is_error_frame:
                row_style = "red"
            elif msg.is_fd:
                row_style = "blue"

            t.add_row(
                f"{msg.timestamp:12.4f}",
                id_str,
                str(msg.dlc),
                data_str,
                flags_str,
                style=row_style,
            )
        return t

    def _footer(self) -> Panel:
        with self._lock:
            count = self._count
            rate  = self._rate

        elapsed = time.monotonic() - self._start
        elapsed_str = str(timedelta(seconds=int(elapsed)))

        log_txt = (f"[dim]{self.out_path.name}[/dim]" if self.out_path
                   else "[dim]no file[/dim]")

        txt = Text.assemble(
            ("  Msgs: ", "dim"), (f"{count:,}", "bold white"),
            ("   │   Rate: ", "dim"), (f"{rate:6.1f}/s", "bold white"),
            ("   │   Elapsed: ", "dim"), (elapsed_str, "bold white"),
            ("   │   Log: ", "dim"), (log_txt, ""),
            ("   │   ", "dim"),
            ("[dim]Ctrl+C to stop[/dim]", ""),
        )
        return Panel(txt, border_style="dim cyan", height=3)

    def __rich__(self):
        layout = Layout()
        layout.split_column(
            Layout(self._header(), name="header", size=3),
            Layout(self._table(), name="table"),
            Layout(self._footer(), name="footer", size=3),
        )
        return layout


# ──────────────────────────────────────────────────────────────────────────────
# Run
# ──────────────────────────────────────────────────────────────────────────────

def build_output_path(fmt: str, base: str | None) -> Path:
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = base if base else f"can_log_{ts}"
    ext  = fmt if fmt.startswith(".") else f".{fmt}"
    p    = Path(name)
    return p if p.suffix.lower() == ext else Path(name + ext)


def run(args: argparse.Namespace) -> int:
    passive  = args.passive and not getattr(args, "active", False)
    state    = BusState.PASSIVE if passive else BusState.ACTIVE
    out_path = None if args.no_log else build_output_path(args.fmt, args.output)

    display = CANDisplay(
        channel=args.channel,
        bitrate=args.bitrate,
        passive=passive,
        fd=args.fd,
        out_path=out_path,
    )

    # ── open bus ─────────────────────────────────────────────────────────────
    try:
        bus = Bus(
            interface="pcan",
            channel=args.channel,
            bitrate=args.bitrate,
            state=state,
            fd=args.fd,
        )
    except Exception as exc:
        console.print(f"\n[red bold]ERROR:[/red bold] Could not open {args.channel}: {exc}")
        console.print("\nTroubleshooting:")
        console.print("  • Run [cyan]python can_logger.py --scan[/cyan] to list detected devices")
        console.print("  • Is the PCAN-USB adapter plugged in?")
        console.print("  • Does the bit-rate match the network?")
        console.print("  • Is [dim]libPCBUSB.dylib[/dim] at [dim]/usr/local/lib/[/dim]?\n")
        return 1

    # ── listeners ────────────────────────────────────────────────────────────
    class DisplayListener(can.Listener):
        def on_message_received(self, msg):
            display.add_message(msg)
        def stop(self):
            pass

    listeners: list[can.Listener] = [DisplayListener()]
    if out_path is not None:
        listeners.append(Logger(str(out_path)))

    notifier = Notifier(bus, listeners)

    # ── signal handling ──────────────────────────────────────────────────────
    stop_flag = False

    def _stop(sig, frame):
        nonlocal stop_flag
        stop_flag = True

    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    # ── live display loop ────────────────────────────────────────────────────
    t_start = time.monotonic()
    with Live(display, console=console, refresh_per_second=10,
              screen=True, vertical_overflow="visible"):
        while not stop_flag:
            elapsed = time.monotonic() - t_start
            if args.duration and elapsed >= args.duration:
                break
            time.sleep(0.05)

    # ── shutdown ─────────────────────────────────────────────────────────────
    notifier.stop()
    bus.shutdown()

    elapsed = time.monotonic() - t_start
    console.rule()
    console.print(f"  Stopped after [bold]{elapsed:.1f}s[/bold]  —  "
                  f"[bold]{display._count:,}[/bold] messages received")
    if out_path and out_path.exists():
        size_kb = out_path.stat().st_size / 1024
        console.print(f"  Log saved: [cyan]{out_path.resolve()}[/cyan]  "
                      f"([dim]{size_kb:.1f} KB[/dim])")
    console.print()
    return 0


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main() -> int:
    args = parse_args()

    if args.scan:
        console.print()
        console.rule("[bold cyan]PCAN Device Scan[/bold cyan]")
        devices = scan_devices()
        print_scan_results(devices)
        return 0

    # No channel specified → run wizard
    if args.channel is None:
        args = wizard()

    return run(args)


if __name__ == "__main__":
    sys.exit(main())

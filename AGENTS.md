# AGENTS.md — Operating guide for AI assistants

This file tells an AI agent how to run and reason about this project. Read it
before starting servers or touching the CAN hardware.

## TL;DR — "start it up"

When the user asks to **start the logger**, they almost always mean the
**GridConnect / SLCAN edition** (used with the CANable / Joinrich RH-02 adapter):

```bash
cd "/Users/robertjones/Documents/CAN Logger/gridconnect" && ./start.sh
# → http://localhost:8002
```

The original **PEAK PCAN-USB edition** lives at the repo root:

```bash
cd "/Users/robertjones/Documents/CAN Logger" && ./start.sh
# → http://localhost:8001
```

Both reuse the same virtualenv at `./venv` (repo root).

## ⚠️ CRITICAL: the agent usually cannot start the server itself

The server must open a USB serial device (e.g. `/dev/cu.usbmodem1201`). Commands
an agent runs normally go through Cursor's **sandbox**, which blocks USB access.
A sandboxed server starts fine but every capture fails with:

```
could not open port /dev/cu.usbmodem1201: [Errno 1] Operation not permitted
```

Therefore, to start the server, do **one** of:

1. **Preferred:** ask the user to run `./start.sh` in their **own terminal**.
   This runs outside the sandbox, has real device access, and persists with
   their session.
2. If launching it yourself, you **must** run outside the sandbox (full
   permissions). Do not background a sandboxed server — it will look healthy but
   be unable to open the adapter.

Do **not** kill the user's running server to "restart" it; you will likely
replace a working (unsandboxed) instance with a broken (sandboxed) one.

## Hardware / device facts

- Active adapter: **CANable 1.0** running `normaldotcom/canable-fw` slcan
  firmware (this is the Joinrich RH-02, reflashed from gs_usb to slcan).
- Enumerates as a serial port, typically `/dev/cu.usbmodem1201`
  (USB ID `0xAD50:0x60C4`, label `CANable 9fddea4 …`).
- Classic CAN only (no CAN FD). Common bitrate: **500000**.
- python-can interface: `slcan`. Passive = `listen_only` (silent monitor).
- **One owner per port:** only a single process can hold the serial device at a
  time. The logger and any transmitter (e.g. a drive-cycle player) cannot share
  one adapter — the second to open it will fail/hang.

## Check / stop the server (safe, read-only-ish)

```bash
# Is it up + logging?
curl -s http://localhost:8002/status

# What's listening / holding things
lsof -nP -iTCP:8002 -sTCP:LISTEN
lsof /dev/cu.usbmodem1201        # who holds the adapter

# Rescan from the UI is also exposed as:
curl -s http://localhost:8002/scan
```

To stop: `Ctrl-C` in the terminal running it, or kill the PID listening on the
port. Prefer letting the user do this.

## GridConnect edition specifics (`gridconnect/`)

- Standalone copy of the app, slcan-only, served on **8002** (parent is 8001).
- `server.py` opens the bus via `StatusSlcanBus` (a `slcanBus` subclass) which
  also polls the firmware's nonstandard `E` command to read the adapter's
  **error register**, surfaced in the UI's "Errors" stat.
  - This firmware does **not** emit true CAN error frames and does not support
    the standard LAWICEL `F` flags. The `E` register is sticky-since-power-on and
    reports: CAN TX failed, RX FIFO overflow, and internal buffer-full / USB-busy
    conditions — not bus-off / error-passive / arbitration-lost.
- Logs: `gridconnect/logs/can_log_<timestamp>[_<note>].trc` (gitignored).

## Repo / env notes

- Git remote: `origin` → `github.com/robertcameronjones/CANLogger.git`, branch `main`.
- `venv/`, `logs/`, `*.trc`, `__pycache__/`, `.DS_Store` are gitignored.
- `PCBUSB/` is the third-party PEAK macOS driver (only needed for the PEAK
  edition) and is not part of this project's source.
- Decoding signal library is optional; see `README.md`.

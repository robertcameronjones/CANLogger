# AGENTS.md — Operating guide for AI assistants

This file tells an AI agent how to run and reason about this project. Read it
before starting servers or touching the CAN hardware.

## TL;DR — "start it up"

Two editions because the **hardware interfaces differ** — PEAK uses the `pcan`
driver; the bench adapter uses **slcan over USB serial**. Same UI, different
backends. You pick the device in the dropdown; nothing is locked to one adapter.

| | **Vehicle** (PEAK PCAN) | **Bench** (slcan serial) |
|---|---|---|
| **Hardware** | PEAK PCAN-USB | CANable / Joinrich RH-02 (slcan firmware) |
| **Start** | `./start.sh` (repo root) | `cd slcan && ./start.sh` |
| **URL** | http://localhost:8001 | http://localhost:8003 |
| **Interface** | `pcan` | `slcan` |

Both can run at the same time (different ports, different USB devices). Root
`start.sh` only frees port **8001**; it does not stop the bench logger on 8003.

When the user asks to **start the logger** without specifying which edition,
context matters:
- **At the desk / bench** → `slcan/` edition (CANable)
- **In the vehicle** → repo root (PEAK PCAN)

```bash
# Vehicle — PEAK PCAN-USB
cd "/Users/robertjones/Documents/CAN Logger" && ./start.sh
# → http://localhost:8001

# Bench — CANable / Joinrich (slcan over USB serial)
cd "/Users/robertjones/Documents/CAN Logger/slcan" && ./start.sh
# → http://localhost:8003
```

Both reuse the same virtualenv at `./venv` (repo root).

**Naming note:** an old misnamed folder `gridconnect/` held the bench slcan copy.
That name was wrong — the vehicle logger is PEAK PCAN at the repo root, not
GridConnect. The bench edition is now in `slcan/`.

## ⚠️ CRITICAL: the agent usually cannot start the server itself

The slcan edition must open a USB serial device (e.g. `/dev/cu.usbmodem1301`).
Commands an agent runs normally go through Cursor's **sandbox**, which blocks USB
access. A sandboxed server starts fine but every capture fails with:

```
could not open port /dev/cu.usbmodem1301: [Errno 1] Operation not permitted
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

### Vehicle (repo root)
- PEAK PCAN-USB (or PCAN-USB FD) via mac-can **PCBUSB** driver.
- python-can interface: `pcan`. Channels: `PCAN_USBBUS1` … `PCAN_USBBUS8`.
- Supports CAN FD. Common bitrate: **500000**.

### Bench (`slcan/`)
- **CANable 1.0** running `normaldotcom/canable-fw` slcan firmware (Joinrich
  RH-02 reflashed from gs_usb to slcan).
- Enumerates as a serial port, typically `/dev/cu.usbmodem1301`
  (USB ID `0xAD50:0x60C4`).
- Classic CAN only (no CAN FD). python-can interface: `slcan`.
- Passive = `listen_only` (silent monitor).
- **One owner per port:** only a single process can hold the serial device at a
  time.

## Check / stop the server (safe, read-only-ish)

```bash
# Vehicle
curl -s http://localhost:8001/status
lsof -nP -iTCP:8001 -sTCP:LISTEN

# Bench
curl -s http://localhost:8003/status
lsof -nP -iTCP:8003 -sTCP:LISTEN
lsof /dev/cu.usbmodem1301        # who holds the adapter
curl -s http://localhost:8003/scan
```

To stop: `Ctrl-C` in the terminal running it, or kill the PID listening on the
port. Prefer letting the user do this.

## PEAK / vehicle edition (repo root)

- `server.py` — `interface="pcan"`, served on **8001**.
- Logs: `logs/can_log_<timestamp>[_<note>].trc` (gitignored).
- Requires PCBUSB driver (see `README.md`).

## SLCAN / bench edition (`slcan/`)

- `server.py` — plain `slcanBus`, CANable-focused device picker, served on **8003**.
- Logs: `slcan/logs/can_log_<timestamp>[_<note>].trc` (gitignored).

## Repo / env notes

- Git remote: `origin` → `github.com/robertcameronjones/CANLogger.git`, branch `main`.
- `venv/`, `logs/`, `*.trc`, `__pycache__/`, `.DS_Store` are gitignored.
- `PCBUSB/` is the third-party PEAK macOS driver (vehicle edition only).
- Decoding signal library is optional; see `README.md`.

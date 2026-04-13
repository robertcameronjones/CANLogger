# CAN Logger

macOS web-based CAN bus logger for PEAK-System PCAN-USB interfaces, built on
[mac-can / PCBUSB](https://github.com/mac-can/PCBUSB-Library) and
[python-can](https://python-can.readthedocs.io/).

## Features

- **4-pane live UI** — settings sidebar, raw message log, CAN ID tracker, decoded signal pane
- **CAN ID pane** — unique IDs sorted by address, live payload, message count, ΔmS between frames
- **Signal pane** — decoded values with min/max tracking; three decoding layers:
  1. **J1979** (SAE) — standard OBD-II Mode 01 PIDs (`0x41` response)
  2. **J1979-2** (SAE) — Mode 22 UDS DIDs in the F4xx range (`0x62` response)
  3. **Vehicle signal library** — vehicle-specific PIDs/DIDs from `signal_library.json` (fallback)
- Log to `.trc`, `.asc`, `.csv`, `.log`, or `.blf` — never overwrites prior files
- Listen-only (passive) or active mode, CAN FD support
- Force-kill stuck connections, optional per-session log note

## Hardware

PEAK-System PCAN-USB or PCAN-USB FD adapter + the
[PCBUSB library](https://github.com/mac-can/PCBUSB-Library/releases) (Universal
Binary, v0.13+, Apple Silicon compatible).

## Setup

```bash
# 1 — Install PCBUSB (ARM64 universal binary)
#     Download the .tar.gz from https://github.com/mac-can/PCBUSB-Library/releases
#     then run its install.sh

# 2 — Create venv and install dependencies
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# 3 — (Optional) point to your signal library
export SIGNAL_LIB="/path/to/signal_library.json"
#     or drop signal_library.json in this folder

# 4 — Launch
./start.sh
# Opens http://localhost:8000
```

## Signal Library

The decoder looks for `signal_library.json` in this order:

1. Path in the `SIGNAL_LIB` environment variable
2. `~/Documents/CAN Log Splitter/signal_library.json` (default install)
3. `signal_library.json` in the project folder

The library format is defined by the
[CAN Log Splitter](https://github.com/your-org/CAN-Log-Splitter) project.
Each entry carries `can_id_return`, `pid_did`, `multiplier`, `offset`, and
`unit` fields that describe how to decode a specific signal on a specific ECU.

## Log Files

Logs are written to the `logs/` subdirectory with millisecond-precision
timestamps. An optional session note can be appended to the filename.
The note field is cleared automatically when logging stops.

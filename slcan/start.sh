#!/bin/zsh
cd "$(dirname "$0")"

# Bench / SLCAN edition (CANable, Joinrich, etc.). Serves http://localhost:8003
# Independent of the PEAK vehicle logger (8001) — does not kill or share that port.
export CAN_LOGGER_PORT=8003
exec ../venv/bin/python3 server.py

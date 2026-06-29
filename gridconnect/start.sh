#!/bin/zsh
cd "$(dirname "$0")"

# GridConnect / SLCAN edition — reuses the parent project's venv
# (which has pyserial installed). Serves on http://localhost:8002
exec ../venv/bin/python3 server.py

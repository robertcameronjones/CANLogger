#!/bin/zsh
cd "$(dirname "$0")"

# Use the venv Python directly — no need to activate first
exec ./venv/bin/python3 server.py

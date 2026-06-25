#!/bin/zsh
cd "$(dirname "$0")"

PORT=8001

# Free the port if a previous instance is still bound to it
PIDS=$(lsof -ti tcp:$PORT 2>/dev/null)
if [[ -n "$PIDS" ]]; then
  echo "Port $PORT busy (pids: $PIDS) — killing old instance..."
  kill -9 $PIDS 2>/dev/null
  sleep 1
fi

# Use the venv Python directly — no need to activate first
echo "Starting CAN Logger on http://localhost:$PORT ..."
exec ./venv/bin/python3 server.py

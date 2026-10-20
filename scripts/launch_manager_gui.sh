#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Desktop applications should run in the logged-in user's session. If the
# launcher was invoked through sudo, drop back to that user before starting
# Tk so the display session remains consistent.
if [[ "$EUID" -eq 0 && -n "${SUDO_USER:-}" && "$SUDO_USER" != "root" ]]; then
  USER_ID="$(id -u "$SUDO_USER")"
  exec sudo -u "$SUDO_USER" -H -- env \
    DISPLAY="${DISPLAY:-}" \
    XAUTHORITY="${XAUTHORITY:-/home/$SUDO_USER/.Xauthority}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$USER_ID}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/$USER_ID/bus}" \
    python3 "$SCRIPT_DIR/launch_manager_gui.py"
fi

exec python3 "$SCRIPT_DIR/launch_manager_gui.py"

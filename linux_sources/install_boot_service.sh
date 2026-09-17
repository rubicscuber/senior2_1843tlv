#!/bin/sh
# One-time installer: registers startpi_boot.sh as a systemd service so
# console_only_Pi starts automatically after every successful boot of the
# Raspberry Pi. Run this once on the Pi, from linux_sources/:
#   ./install_boot_service.sh
#
# Afterwards:
#   check status:   systemctl status console_only_pi
#   follow output:  journalctl -u console_only_pi -f     (also in logs/)
#   stop once:      sudo systemctl stop console_only_pi
#   disable:        sudo systemctl disable console_only_pi

SERVICE_NAME="console_only_pi"
UNIT_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RUN_USER=$(id -un)

if [ ! -x "$SCRIPT_DIR/console_only_Pi" ]; then
    echo "Error: console_only_Pi is not built yet. Run 'make' in $SCRIPT_DIR first."
    exit 1
fi
chmod +x "$SCRIPT_DIR/startpi_boot.sh"

echo "Installing $UNIT_FILE (runs as user $RUN_USER from $SCRIPT_DIR)..."
sudo tee "$UNIT_FILE" > /dev/null <<EOF
[Unit]
Description=Traffic Monitoring target report (console_only_Pi)
# start once the system is fully up; the script itself waits for the ports
After=multi-user.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=$SCRIPT_DIR/startpi_boot.sh
# relaunch whenever it exits (ports missing, program crash, EVM unplugged);
# a manual 'systemctl stop' is not restarted
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
echo "Installed and enabled. The report will start on the next boot."
echo "Start it now without rebooting with: sudo systemctl start $SERVICE_NAME"

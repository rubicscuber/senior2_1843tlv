#!/bin/sh
# One-time installer: registers startpi_boot.sh as a systemd service so
# console_only_Pi starts automatically after every successful boot of the
# Raspberry Pi. Run this once on the Pi, from linux_sources/:
#   ./install_boot_service.sh
#
# It also prepares GPIO access for the safety monitor (gpio group, udev rule
# on plain Debian, SupplementaryGroups= for the service).
#
# Afterwards:
#   check status:   systemctl status console_only_pi
#   follow output:  journalctl -u console_only_pi -f     (also in logs/)
#   stop once:      sudo systemctl stop console_only_pi
#   disable:        sudo systemctl disable console_only_pi

SERVICE_NAME="console_only_pi"
UNIT_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
UDEV_RULE="/etc/udev/rules.d/99-gpio-chardev.rules"

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RUN_USER=$(id -un)

if [ ! -x "$SCRIPT_DIR/console_only_Pi" ]; then
    echo "Error: console_only_Pi is not built yet. Run 'make' in $SCRIPT_DIR first."
    exit 1
fi
chmod +x "$SCRIPT_DIR/startpi_boot.sh"

ask() { # ask "question" -> returns 0 on y/Y (non-interactive stdin answers no)
    printf '%s [y/N] ' "$1"
    read -r answer
    case "$answer" in
        y|Y) return 0 ;;
        *) return 1 ;;
    esac
}

# ---- GPIO access for the safety monitor ----
SUPP_GROUPS=""
if [ -e /dev/gpiochip0 ]; then
    if ! getent group gpio >/dev/null 2>&1; then
        echo "No 'gpio' group exists (plain Debian rather than Raspberry Pi OS), so"
        echo "/dev/gpiochip* is root-only and the safety monitor could not open its pins."
        if ask "Create the gpio group and a udev rule granting it access?"; then
            sudo groupadd gpio
            printf 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"\n' \
                | sudo tee "$UDEV_RULE" > /dev/null
            sudo udevadm control --reload-rules
            sudo udevadm trigger --subsystem-match=gpio
            echo "Wrote $UDEV_RULE."
        fi
    fi
    if getent group gpio >/dev/null 2>&1; then
        # the service gets the group directly; the user needs it only to run
        # console_only_Pi with GPIO pins by hand
        SUPP_GROUPS="SupplementaryGroups=gpio"
        if ! id -nG "$RUN_USER" | grep -qw gpio; then
            echo "User $RUN_USER is not in the gpio group (needed to use the GPIO pins from"
            echo "your own shell; the service itself is granted the group in its unit file)."
            if ask "Add $RUN_USER to the gpio group now?"; then
                sudo usermod -aG gpio "$RUN_USER"
                echo "Log out and back in for the new group to take effect in your shell."
            fi
        fi
    fi
else
    echo "Note: /dev/gpiochip0 not found; skipping GPIO permission setup."
fi

echo "Installing $UNIT_FILE (runs as user $RUN_USER from $SCRIPT_DIR)..."
sudo tee "$UNIT_FILE" > /dev/null <<EOF
[Unit]
Description=Traffic Monitoring target report (console_only_Pi)
# start once the system is fully up; the script itself waits for the ports
After=multi-user.target

[Service]
Type=simple
User=$RUN_USER
$SUPP_GROUPS
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
echo "Safety-monitor settings go in $SCRIPT_DIR/startpi_boot.env (see startpi_boot.env.example)."
if [ -e /dev/gpiochip0 ]; then
    echo "GPIO devices:"
    ls -l /dev/gpiochip* 2>/dev/null
fi

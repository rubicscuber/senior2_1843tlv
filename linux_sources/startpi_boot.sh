#!/bin/sh
# Boot-time launcher for console_only_Pi on a Raspberry Pi.
#
# Runs the same command as startpi_console.sh, but hardened for unattended
# start at boot: it works from any current directory, waits for the EVM's
# USB serial ports to enumerate, and logs all output to linux_sources/logs/.
#
# To run it automatically after every successful boot, install it as a
# systemd service (once, on the Pi):
#   ./install_boot_service.sh
#
# It can also be run by hand: ./startpi_boot.sh

# same settings as startpi_console.sh
CFG_FILE="../chirp_configs/18xx_traffic_monitoring_70m_MIMO_3D.cfg"
CLI_PORT="/dev/ttyACM0"
DATA_PORT="/dev/ttyACM1"
WAIT_SECS="${WAIT_SECS:-60}" # how long to wait for the serial ports at boot

# run from this script's directory so the relative cfg path resolves
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR" || exit 1

LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/console_pi_$(date +%m%d%y_%H%M%S).log"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') $1" | tee -a "$LOG_FILE"
}

if [ ! -x "./console_only_Pi" ]; then
    log "console_only_Pi not found or not executable in $SCRIPT_DIR. Run 'make' first."
    exit 1
fi

# the EVM's USB serial ports can enumerate a few seconds after boot
waited=0
while [ ! -e "$CLI_PORT" ] || [ ! -e "$DATA_PORT" ]; do
    if [ "$waited" -ge "$WAIT_SECS" ]; then
        log "Serial ports $CLI_PORT / $DATA_PORT not present after ${WAIT_SECS}s. Giving up."
        exit 1
    fi
    log "Waiting for $CLI_PORT and $DATA_PORT..."
    sleep 2
    waited=$((waited + 2))
done

log "Starting console_only_Pi (cfg: $CFG_FILE, cli: $CLI_PORT, data: $DATA_PORT)"
./console_only_Pi -g "$CFG_FILE" -c "$CLI_PORT" -d "$DATA_PORT" 2>&1 | tee -a "$LOG_FILE"

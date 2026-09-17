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
WAIT_SECS="${WAIT_SECS:-60}"     # how long to wait for the serial ports at boot
LOG_MAX_KB="${LOG_MAX_KB:-10240}" # rotate the current log once it passes this size
LOG_KEEP="${LOG_KEEP:-10}"        # number of log files from earlier runs to keep

# run from this script's directory so the relative cfg path resolves
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR" || exit 1

LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/console_pi_$(date +%m%d%y_%H%M%S).log"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') $1" | tee -a "$LOG_FILE"
}

# keep the SD card from filling up across many boots: drop all but the
# newest LOG_KEEP log files from earlier runs
ls -1t "$LOG_DIR"/console_pi_*.log "$LOG_DIR"/console_pi_*.log.old 2>/dev/null \
    | tail -n +"$((LOG_KEEP + 1))" \
    | while IFS= read -r old; do rm -f "$old"; done

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

# Echo every line to stdout (the systemd journal) and append it to the log
# file, rotating the file once it reaches LOG_MAX_KB so a long run cannot
# fill the disk: at most two files (current + .old) exist for this run.
./console_only_Pi -g "$CFG_FILE" -c "$CLI_PORT" -d "$DATA_PORT" 2>&1 | {
    lines=0
    while IFS= read -r line; do
        printf '%s\n' "$line"
        printf '%s\n' "$line" >> "$LOG_FILE"
        lines=$((lines + 1))
        if [ $((lines % 500)) -eq 0 ]; then
            size_kb=$(( $(wc -c < "$LOG_FILE") / 1024 ))
            if [ "$size_kb" -ge "$LOG_MAX_KB" ]; then
                mv -f "$LOG_FILE" "$LOG_FILE.old"
                log "Log rotated at ${size_kb} KB; previous part saved as $(basename "$LOG_FILE").old"
            fi
        fi
    done
}
log "console_only_Pi exited."

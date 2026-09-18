#!/bin/sh
# Boot-time launcher for console_only_Pi on a Raspberry Pi.
#
# Runs the same command as startpi_console.sh, but hardened for unattended
# start at boot: it works from any current directory, waits for the EVM's
# USB serial ports to enumerate, and logs all output to linux_sources/logs/.
#
# Optional safety monitor (wheel-speed input, two-second rule, LED alerts):
# set APPROACH_THRESHOLD (closing speed in m/s) to enable it. Every setting
# below can be overridden from the environment or, more conveniently for the
# boot service, in linux_sources/startpi_boot.env (see startpi_boot.env.example).
#
# To run it automatically after every successful boot, install it as a
# systemd service (once, on the Pi):
#   ./install_boot_service.sh
#
# It can also be run by hand: ./startpi_boot.sh

# run from this script's directory so the relative cfg path resolves
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR" || exit 1

# optional settings file (plain VAR=value lines, see startpi_boot.env.example)
if [ -r "$SCRIPT_DIR/startpi_boot.env" ]; then
    # shellcheck disable=SC1091
    . "$SCRIPT_DIR/startpi_boot.env"
fi

# same settings as startpi_console.sh
CFG_FILE="${CFG_FILE:-../chirp_configs/18xx_traffic_monitoring_70m_MIMO_3D.cfg}"
CLI_PORT="${CLI_PORT:-/dev/ttyACM0}"
DATA_PORT="${DATA_PORT:-/dev/ttyACM1}"
WAIT_SECS="${WAIT_SECS:-60}"     # how long to wait for the serial ports at boot
LOG_MAX_KB="${LOG_MAX_KB:-10240}" # rotate the current log once it passes this size
LOG_KEEP="${LOG_KEEP:-10}"        # number of log files from earlier runs to keep

# safety monitor ("${VAR-default}": an explicitly empty value disables the item)
APPROACH_THRESHOLD="${APPROACH_THRESHOLD-}"   # m/s closing speed for the approach alert; empty = monitor off
GAP_SECONDS="${GAP_SECONDS-2}"                # two-second rule threshold, s
WHEEL_DIAMETER="${WHEEL_DIAMETER-0.7}"        # wheel diameter, m
PULSES_PER_REV="${PULSES_PER_REV-1}"          # hall-effect pulses per wheel revolution
PULSE_GPIO="${PULSE_GPIO-17}"                 # BCM pin of the wheel sensor
LED_GAP_GPIO="${LED_GAP_GPIO-22}"             # BCM pin of the two-second-rule LED; empty = none
LED_SPEED_GPIO="${LED_SPEED_GPIO-23}"         # BCM pin of the approach-alert LED; empty = none
CORRIDOR="${CORRIDOR-}"                       # lateral half-width (m) for the gap rule; empty = off
EXTRA_ARGS="${EXTRA_ARGS-}"                   # further console_only_Pi options, e.g. "--led-active-low"

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

# assemble the command line (POSIX sh has no arrays: use the positional parameters)
set -- -g "$CFG_FILE" -c "$CLI_PORT" -d "$DATA_PORT"
if [ -n "$APPROACH_THRESHOLD" ]; then
    set -- "$@" --approach-threshold "$APPROACH_THRESHOLD" --gap-seconds "$GAP_SECONDS" \
        --pulse-gpio "$PULSE_GPIO" --wheel-diameter "$WHEEL_DIAMETER" --pulses-per-rev "$PULSES_PER_REV"
    if [ -n "$LED_GAP_GPIO" ]; then
        set -- "$@" --led-gap-gpio "$LED_GAP_GPIO"
    fi
    if [ -n "$LED_SPEED_GPIO" ]; then
        set -- "$@" --led-speed-gpio "$LED_SPEED_GPIO"
    fi
    if [ -n "$CORRIDOR" ]; then
        set -- "$@" --corridor "$CORRIDOR"
    fi
fi
# EXTRA_ARGS is word-split on purpose
# shellcheck disable=SC2086
set -- "$@" $EXTRA_ARGS
log "Command: console_only_Pi $*"

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
./console_only_Pi "$@" 2>&1 | {
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

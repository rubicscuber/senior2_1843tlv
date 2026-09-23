# Traffic Monitoring Terminal Visualizer (Linux)

A C++17 port of the MATLAB Traffic Monitoring visualizer (`tm_visualizer.m`)
that runs entirely in a Linux terminal. The MATLAB GUI (the `setup_tm` app,
plot figure, frame slider and popup menus) is replaced by command-line
options, an ANSI/ASCII plot and keyboard controls. No external libraries are
required — only a C++17 compiler and POSIX (termios) serial support.

## Building

```sh
cd linux_sources
make
```

This produces four executables: `tm_visualizer` (the visualizer),
`console_only` (a raw packet dumper), `console_only_Pi` (a target
detection reporter and rider safety monitor for headless/Raspberry Pi use)
and `wheel_test` (a bench test for the wheel-speed sensor input).
`make clean` removes them and the `build/` directory; `make selftest` runs the
built-in logic tests of the safety monitor and the frame parser. On a Raspberry Pi see
*Requirements on Debian 13 (trixie) 64-bit* below.

## Running

A chirp configuration file is always required (`-g`). Use one of the files in
`../chirp_configs/`, matching the binary flashed on the EVM.

### Playback mode (replay a recorded stream)

```sh
./tm_visualizer -g ../chirp_configs/18xx_traffic_monitoring_70m_MIMO_3D.cfg \
                -f capture.dat --height 2 --el -20
```

Accepts the space-separated hex format written by the MATLAB visualizer's
logger (and by `-r` below), or a raw binary capture — the format is detected
automatically.

### Real-time mode (live EVM over UART)

```sh
./tm_visualizer -g ../chirp_configs/18xx_traffic_monitoring_70m_MIMO_3D.cfg \
                -c /dev/ttyACM0 -d /dev/ttyACM1 \
                --height 2 --el -20 -r capture.dat
```

`-c` is the CLI/config port (115200 baud) and `-d` the data port (921600
baud); on Linux the XDS110 usually enumerates as `/dev/ttyACM0` and
`/dev/ttyACM1` (or `/dev/ttyUSB0`/`ttyUSB1`). Add your user to the `dialout`
group if you get permission errors. If the device is already configured and
streaming (e.g. after a previous run without NRST), pass `--no-load` and omit
`-c`.

`-r <file>` records the raw data stream as hex, replayable both here (`-f`)
and in the original MATLAB visualizer.

### Options

| Option | Meaning |
|---|---|
| `-g, --cfg <file>` | chirp configuration (.cfg) file — required |
| `-f, --file <dat>` | playback mode: recorded stream to replay |
| `-d, --data <dev>` | real-time mode: data UART device |
| `-c, --cli <dev>` | real-time mode: CLI/config UART device |
| `-r, --record <file>` | real-time mode: log the raw stream as hex |
| `--height <m>` | sensor mounting height (default 0) |
| `--az <deg>` | azimuth tilt (default 0) |
| `--el <deg>` | elevation tilt, down is negative (default 0) |
| `--lanes N,x,y,w,h` | lane counting: N adjacent lanes starting at (x, y), each w wide and h deep, in meters (e.g. `--lanes 2,-6,10,6,20`) |
| `--view xy\|yz\|xz` | initial plot projection (default `xy`) |
| `--fps <n>` | real-time mode: maximum display refresh rate (default 20). The newest frame is always shown and the backlog is discarded, so lowering this on a slow console (Pi HDMI/serial) costs nothing but smoothness |
| `--no-load` | do not send the cfg to the device |
| `--no-plot` | print one stats line per frame instead of drawing the plot |
| `--paused` | start playback paused |

### Keys while running

| Key | Action |
|---|---|
| `1` / `2` / `3` | switch view: X-Y / Y-Z / X-Z (the GUI's view popup) |
| `space` | play / pause (playback; the GUI's play control) |
| `n` / `b` | step forward / back one frame (playback; the GUI's slider) |
| `q` or Ctrl-C | quit |

## Display

- gray dots — sensor boresight and approximate FOV guide lines
- blue `*` — detected point cloud (TLV 1000), transformed by the azimuth and
  elevation mounting rotations plus the height offset
- yellow digits — tracked objects (TLV 1010), labelled with the last digit of
  their track id; full ids and positions are listed under the plot
- red rectangles — lanes (X-Y view only), with per-lane target counts in the
  status line
- the status line's "Num Frames in Buffer" is how many frames arrived since
  the last redraw (normally 1–2), and "Bad frames" counts frames discarded
  for a wrong packet length. A steadily rising bad-frame count means bytes
  are being lost on the serial link (loose cable, or the console cannot keep
  up — try a lower `--fps`)

## console_only: raw TLV packet dump

`console_only` prints every frame's header fields and each TLV (type, name,
length, payload hex) straight to stdout with no screen control — useful for
debugging the stream format or piping to a file:

```sh
./console_only capture.dat                 # dump a recorded stream
./console_only -d /dev/ttyACM1             # dump a live data UART (Ctrl-C stops)
./console_only capture.dat > dump.txt      # save the labelled dump

# send a chirp cfg to an idle device first, then dump (-c and -g go together):
./console_only -d /dev/ttyACM1 -c /dev/ttyACM0 \
               -g ../chirp_configs/18xx_traffic_monitoring_70m_MIMO_3D.cfg
```

Without `-c`/`-g` the device must already be configured and streaming. As in
the visualizer, if the data port already has bytes waiting, the cfg is not
re-sent (press NRST on the EVM to load a new one).

## console_only_Pi: target detection report (Raspberry Pi)

`console_only_Pi` takes the same arguments as `console_only` but, instead of
hex dumps, prints one plain-text report per frame saying whether a target is
detected, with each target's track id, position, range, velocity, speed and
acceleration, plus a summary on exit. The output is line-oriented with no
screen control, so it works over SSH or a serial console on a Raspberry Pi.
Build it on the Pi itself with `make` (see *Requirements on Debian 13* below), then:

```sh
./console_only_Pi capture.dat              # report from a recorded stream
./console_only_Pi -d /dev/ttyACM1          # live report (Ctrl-C stops)
./console_only_Pi -d /dev/ttyACM1 -c /dev/ttyACM0 \
                  -g ../chirp_configs/18xx_traffic_monitoring_70m_MIMO_3D.cfg
```

Example output:

```
Frame 2      | TARGET DETECTED: 1 target(s), 12 point-cloud detections
  TID 7    pos (   1.00,   12.00,    0.00) m  range  12.04 m
           vel (   0.50,    8.00,    0.00) m/s  speed   8.02 m/s  accel (0.10, 0.20, 0.00) m/s^2
Frame 3      | no target detected (9 point-cloud detections)
```

### Safety monitor: wheel speed, two-second rule and LED alerts

With `--approach-threshold <m/s>` the reporter becomes a rider warning device
for a **rear-facing** radar: it derives the vehicle's own ground speed from a
hall-effect wheel sensor on a GPIO pin (or a fixed `--self-speed`), works out
each target's closing speed and whether it keeps a two-second gap, lights a
LED on a gap violation and, on a fast approach, plays an audio file out of the
Pi's 3.5 mm jack (`--alert-sound`) and/or flashes a second LED. Without the
flag the output is identical to the plain report.

Wiring (BCM numbering; the defaults in `startpi_boot.sh` are 17 / 22 / 23):

- hall-effect switch: output -> `--pulse-gpio` (the internal pull-up is
  enabled and the switch pulls the pin low when the magnet passes), GND
  common, VCC to 3.3 V. One magnet on the wheel = 1 pulse per revolution.
- LEDs: GPIO -> 330 ohm -> LED anode, cathode -> GND (lit when the pin is
  high). Use `--led-active-low` for the opposite wiring.
- The kernel's GPIO character device (`/dev/gpiochip0`) is used directly, so
  no library is needed; the user must be in the `gpio` group (see
  *Requirements* below).

What is computed per target (sensor frame: +Y points behind you along the
road, X is lateral; the tracker's velocities are relative to the moving
radar):

| Quantity | Formula |
|---|---|
| range `r` | `hypot(x, y)` (horizontal road gap) |
| closing speed `c` | `-(x*vx + y*vy) / r`, positive when approaching |
| target ground speed `v_t` | `max(0, selfSpeed - vy)`: 0 for stationary roadside objects and for traffic moving away |
| time gap | `r / v_t` (only when `v_t` > 0.5 m/s, i.e. the target is following) |
| **gap violation** (steady LED) | time gap < `--gap-seconds` (default 2.0), and `abs(x)` <= `--corridor` when one is set |
| **approach alert** (audio file and/or flashing LED) | `c` > `--approach-threshold` |
| self speed | pulse periods averaged over one wheel revolution; `pi * diameter / pulsesPerRev` per pulse |

An alert stays active for `--alert-hold` (1 s) after the last frame that
triggered it, so a track dropping out for a frame does not flicker the LED or
stutter the audio (and always at least until the next frame, so
`--alert-hold 0` means "only while the alert is present"). All alerts clear
when no radar frame arrives for `--frame-timeout` (1 s) or when the program
exits. A frame is processed as soon as its own bytes have arrived, so an
alert reacts within the frame period rather than one frame later.

**Approach alert audio.** `--alert-sound <file>` plays the file (WAV is the
safe choice) through the Pi's 3.5 mm jack for as long as the approach alert
is active: the player is started when the alert begins, restarted each time
the file ends while the alert lasts (`--sound-once` plays it a single time
per alert instead) and stopped the moment the alert ends. The player is a
separate process (`aplay -q` from alsa-utils by default), so audio never
delays the radar loop. One-time setup on the Pi:

```sh
sudo apt install alsa-utils          # aplay (already present on Raspberry Pi OS)
aplay -l                             # the jack shows up as "bcm2835 Headphones"
sudo raspi-config                    # System Options > Audio > Headphones
amixer -c Headphones sset Headphones 90%
aplay alert.wav                      # must be audible from the jack
./console_only_Pi capture.dat --self-speed 10 --approach-threshold 2 \
                  --alert-sound alert.wav --sound-check --playback-fps 20
```

`--sound-check` plays the file once at start and refuses to run if that
fails. If `aplay` picks the wrong output (HDMI, a USB sound card), point it
at the jack explicitly: `--sound-player "aplay -q -D plughw:Headphones"`.
Other players work the same way as long as they take the file as their last
argument and exit when the file ends (`mpg123 -q` for MP3, `paplay` on a
desktop image with PipeWire). The user running the program must be in the
`audio` group (the default Raspberry Pi OS user is). In unpaced file replay
the audio is only printed as `[SOUND] start/stop` lines, because the clock
is synthetic there; use `--playback-fps` to hear it.

If the wheel sensor goes silent for 30 s while targets are being tracked, a
warning is printed: with a self speed of 0 every target keeping pace counts
as "not following", so the two-second rule cannot fire until pulses resume
(the approach alert, which only needs the closing speed, keeps working).

If the data port fails (EVM unplugged or reset), the program turns the LEDs
off, prints the summary and exits with status 1 instead of waiting forever,
so the boot service below restarts it and waits for the ports to reappear.

Options (all speeds in m/s):

| Option | Default | Meaning |
|---|---|---|
| `--approach-threshold <m/s>` | — | enables the monitor; closing speed above which the approach alert fires |
| `--gap-seconds <s>` | 2.0 | two-second rule threshold |
| `--corridor <m>` | off | only targets within +/- this lateral distance count for the gap rule |
| `--self-speed <m/s>` | — | fixed self speed instead of the wheel sensor (required for recorded files) |
| `--pulse-gpio <bcm>` | — | wheel sensor input pin (live mode); needs the two options below |
| `--wheel-diameter <m>`, `--pulses-per-rev <n>` | — | wheel geometry |
| `--pulse-edge rising\|falling` | falling | which edge counts as a pulse |
| `--pulse-bias pull-up\|pull-down\|none` | pull-up | input bias |
| `--pulse-debounce-us <n>` | 500 | kernel debounce, 0 = off (with many pulses per revolution use 0 for the most precise timestamps) |
| `--max-speed <m/s>`, `--stop-timeout <s>` | 40, 2 | glitch filter (faster pulses are ignored) / speed reads 0 after this long without a pulse |
| `--led-gap-gpio <bcm>`, `--led-speed-gpio <bcm>` | — | LED output pins (steady gap LED / flashing approach LED) |
| `--alert-sound <file>` | — | audio file played out of the 3.5 mm jack while the approach alert is active |
| `--sound-player <cmd>` | `aplay -q` | player command; the file is appended as the last argument |
| `--sound-once` | off | play the file once per alert instead of repeating it while the alert lasts |
| `--sound-check` | off | play the file once at start; exit with an error if that fails |
| `--led-active-low` | off | LEDs light when the pin is driven low |
| `--gpiochip <path>` | auto | use this chip with offset = BCM number instead of looking the pin up by name |
| `--sim-gpio` | off | print `[LED] gap ON  t=...` transitions instead of driving pins |
| `--alert-hold <s>`, `--flash-hz <n>`, `--frame-timeout <s>` | 1.0, 8, 1.0 | LED timing (hold 0 = only until the next frame) |
| `--playback-fps <n>` | 0 | recorded files: replay in real time at this rate (with or without the monitor; LED bench test); 0 = as fast as possible |
| `--pulse-verbose` | off | print every wheel pulse with its interval and speed |
| `--self-test` | | run the built-in logic tests (`make selftest`) and exit |

Examples:

```sh
./console_only_Pi --self-test                              # logic tests, no hardware

# recorded stream, fixed 10 m/s self speed, LED transitions printed:
./console_only_Pi capture.dat --self-speed 10 --approach-threshold 2 --corridor 1.75 --sim-gpio

# live: wheel sensor on GPIO17, gap LED on GPIO22, approach alert as audio
./console_only_Pi -d /dev/ttyACM1 -c /dev/ttyACM0 -g ../chirp_configs/IWR_1843BOOST_bike.cfg \
                  --approach-threshold 2 --pulse-gpio 17 --wheel-diameter 0.7 --pulses-per-rev 1 \
                  --led-gap-gpio 22 --alert-sound alert.wav

# the same with a flashing LED on GPIO23 as well
./console_only_Pi ... --led-gap-gpio 22 --led-speed-gpio 23 --alert-sound alert.wav
```

Each frame line gains the self speed and each target a third line:

```
Frame 150    | TARGET DETECTED: 2 target(s), 4 point-cloud detections | self 10.00 m/s (36.0 km/h)
  TID 5    pos (   0.30,   18.83,    0.50) m  range  18.83 m
           vel (   0.00,   -4.00,    0.00) m/s  speed   4.00 m/s  accel (0.00, 0.00, 0.00) m/s^2
           closing  +4.00 m/s  ground  14.00 m/s  gap  1.34 s  GAP VIOLATION  APPROACH ALERT
  TID 9    pos (   4.00,   89.50,    0.50) m  range  89.59 m
           vel (   0.00,   10.00,    0.00) m/s  speed  10.00 m/s  accel (0.00, 0.00, 0.00) m/s^2
           closing  -9.99 m/s  ground   0.00 m/s  gap    --  (not following)  (outside corridor)
```

The summary then reports how many frames had a violation or an alert, the
minimum time gap, the maximum closing speed and the wheel pulse counts. The
program prints the resolved GPIO lines and the pulse input's idle level at
start (a low idle level with the default wiring means the magnet is parked at
the sensor or the wiring is wrong).

### wheel_test: bench-testing the wheel sensor

`wheel_test` runs the self-speed part of the safety monitor on its own, with
no radar attached: the same GPIO edge-event input, dropped-event accounting
and `WheelSpeedEstimator` that `console_only_Pi` uses, taking the same wheel
options so a working command line copies over 1:1. It prints every pulse
(interval, instantaneous and averaged speed), a status line every 0.5 s with
the speed, pulse counters, time since the last pulse and the input's level,
and a summary with the mean reading against the expected speed.

```sh
# real sensor on GPIO17: spin the wheel or pass the magnet by hand
./wheel_test --pulse-gpio 17 --wheel-diameter 0.7 --pulses-per-rev 1

# no wheel: drive GPIO27 as a pulse generator equivalent to 10 m/s and jumper
# it to GPIO17; the reading must settle at 10.00 m/s with 0 rejected pulses
./wheel_test --pulse-gpio 17 --loopback-gpio 27 --loopback-speed 10 --duration 20

# no hardware at all (any Linux box): synthetic pulses through the estimator
./wheel_test --sim 10 --duration 5 --quiet-pulses

# generator stops after 4 s: the reading must decay to 0 within --stop-timeout
./wheel_test --sim 10 --duration 8 --gen-stop-after 4 --quiet-pulses
```

What to look for on the bench:

- the idle level printed at start is 1 with the default wiring (0 means the
  magnet is parked at the sensor or the wiring/pull-up is wrong);
- one accepted pulse per magnet pass, none rejected. Rejected pulses at a
  steady speed mean contact bounce or a double edge per pass: raise
  `--pulse-debounce-us`, or lower `--max-speed` to widen the glitch filter;
- the speed decays to 0 within `--stop-timeout` (2 s) after the wheel stops;
- in loopback mode the mean reading is within a few percent of the generated
  speed (the summary prints the error). A large error with the right pulse
  count points at a wrong `--wheel-diameter` or `--pulses-per-rev`.

Add `--interval`, `--duration` and `--quiet-pulses` to taste; `--help` lists
everything. `wheel_test` and `console_only_Pi` cannot use the same pin at the
same time (the kernel reports the line as busy), so stop the boot service
first: `sudo systemctl stop console_only_pi`.

### Running console_only_Pi automatically at boot

`startpi_boot.sh` runs the same command as `startpi_console.sh`, hardened
for unattended start: it works from any directory, waits (up to 60 s) for
the EVM's serial ports to enumerate, and logs everything to `logs/`. Logs
are bounded so long runs cannot fill the SD card: the current log rotates
at 10 MB (`LOG_MAX_KB`, keeping one `.old` part) and only the 10 newest
files from earlier runs are kept (`LOG_KEEP`); both can be overridden as
environment variables.

Settings for the boot run, including the safety monitor, live in
`startpi_boot.env` next to the script (copy `startpi_boot.env.example`; plain
`VAR=value` lines): set `APPROACH_THRESHOLD` to enable the monitor and adjust
`WHEEL_DIAMETER`, `PULSES_PER_REV`, `PULSE_GPIO`, `LED_GAP_GPIO`,
`LED_SPEED_GPIO`, `CORRIDOR`, `ALERT_SOUND`, `SOUND_PLAYER` or `EXTRA_ARGS`
as needed (an empty `LED_*_GPIO` means no LED on that pin; `ALERT_SOUND` is
the audio file for the approach alert, given with an absolute path). The script logs the assembled command
line at start, so `journalctl` shows exactly what ran.
To have the Pi launch it after every successful boot, install it once as a
systemd service:

```sh
cd linux_sources
make                        # build console_only_Pi first
./install_boot_service.sh   # writes and enables console_only_pi.service
```

The service starts after boot, restarts automatically whenever the program
exits (ports not up yet, EVM unplugged, crash), and runs as the installing
user. Useful commands afterwards:

```sh
systemctl status console_only_pi        # is it running?
journalctl -u console_only_pi -f        # follow live output

sudo systemctl stop console_only_pi     # stop currently running instance (acts like <ctrl-c>)
sudo systemctl start console_only_pi    # start the service immediately if it was stopped
sudo systemctl enable console_only_pi   # start the service on next boot if below command was used
sudo systemctl disable console_only_pi  # remove from boot (wont disable currently running instance)

#another way to access the live output
cd linux_sources && tail -f "$(ls -t logs/console_pi_*.log | head -1)"
```

### Requirements on Debian 13 (trixie) 64-bit

Needed for every mode (plain report, recorded files, visualizer):

- `sudo apt install build-essential` to build: any GCC with C++17 support
  (trixie ships GCC 14) plus `make`; the kernel GPIO header `<linux/gpio.h>`
  comes with it (linux-libc-dev). No other library is used.
- Serial port access: the user running the program must be in the `dialout`
  group (`sudo usermod -aG dialout $USER`, then log out and in). The default
  Raspberry Pi OS user already is. The boot service runs as the installing
  user and keeps that user's groups.

Needed only for the safety monitor with real GPIO pins (`--pulse-gpio`,
`--led-*-gpio`; not for `--self-speed` or `--sim-gpio` runs):

- A kernel with GPIO character device uAPI v2, i.e. 5.10 or newer (the build
  refuses older headers). Any current Raspberry Pi OS or Debian kernel
  qualifies.
- GPIO permissions: Raspberry Pi OS ships a `gpio` group and a udev rule that
  makes `/dev/gpiochip*` `root:gpio 0660`. Plain Debian does not; run
  `./install_boot_service.sh`, which offers to create the group and rule,
  grants the service the group via `SupplementaryGroups=gpio`, and offers to
  add your user to the group (log out and in afterwards).
- Pins are looked up by their kernel line name (`GPIO17`, `GPIO22`, `GPIO23`
  etc.); the program prints the resolved chip and line at start. A few header
  pins carry other names in the mainline device tree (for example GPIO4 is
  `GPIO_GCLK`, 14/15 are `TXD1`/`RXD1`); for those the program falls back to
  `/dev/gpiochip0` with offset = BCM number, which is correct on a Pi 3/4, and
  says so. `--gpiochip /dev/gpiochip0` forces that mapping for all pins.

Needed only for the approach alert audio (`--alert-sound`):

- `alsa-utils` for `aplay` (installed by default on Raspberry Pi OS), and
  membership in the `audio` group for the user running the program (the
  default Raspberry Pi OS user has it). The 3.5 mm jack needs `dtparam=audio=on`
  in `/boot/firmware/config.txt`, which is the default.

Optional: `sudo apt install gpiod` for the command-line tools used in the
bench checks below (libgpiod 2.x syntax as shipped by trixie; the 1.x syntax
found in older examples does not work). The program itself does not use
libgpiod.

```sh
gpioinfo | grep -E 'GPIO(17|22|23)\b'                       # lines exist and are unused
gpioget --bias=pull-up --numeric -c gpiochip0 17            # 1 idle, 0 with the magnet at the sensor
gpiomon --bias=pull-up --edges=falling -c gpiochip0 17      # one event per wheel pass
gpioset -c gpiochip0 --toggle 500ms 22=1                    # blink the gap LED (Ctrl-C stops)
```

### Tracker configuration on a moving platform

The chirp configurations in `chirp_configs/` were written for a stationary
roadside radar. Two settings matter for the safety monitor:

- `boundaryBox -50 50 10 80 -0.5 3`: targets closer than 10 m are not
  tracked, so a follower is lost as it closes in and the two-second rule
  cannot be assessed below 5 m/s self speed (10 m / 2 s). The program prints
  this note at start. Lowering the third value (and `staticBoundaryBox`'s)
  extends tracking closer to the radar.
- `clutterRemoval -1 1` and `allocationParam`'s 1 m/s velocity threshold: a
  vehicle keeping exactly your speed has almost no Doppler and may not be
  allocated a track; the approach alert (which needs closing speed) is
  unaffected.

A changed cfg only loads after an NRST on the EVM. Stationary roadside
objects do appear as tracks on a moving platform; the monitor reports them as
"(not following)" because their ground speed works out to 0.

## Source layout

| File | Ports |
|---|---|
| `src/main.cpp` | `tm_visualizer.m` — setup, mode selection, main loop, transforms |
| `src/cfg_parser.*` | `readCfgFile.m`, `defineCLICommands.m`, `parseCLICommands2Struct.m`, `calculateChirpParams.m` |
| `src/frame_parser.*` | `parseBytes_TM.m`, `getTLV.m`, `getGtrackFrameHeader.m`, `getGtrackPtCloud.m`, `getGtrackTargetList.m`, `getGtrackPtType.m`, `readDATFile2Buffer.m` |
| `src/serial_port.*` | `initCfgPort.m`, `initDataPort.m`, `loadCfg.m`, `readUARTtoBuffer.m` |
| `src/visualizer.*` | figure/plot code: `init3DPlot_TM.m`, `drawFOVLines.m`, `initLanes.m`, stats annotation |
| `src/tm_types.h` | shared frame/point-cloud/target structures |
| `src/console_only.cpp` | standalone raw TLV packet dumper (`console_only` executable) |
| `src/console_only_Pi.cpp` | target detection reporter and safety monitor for Raspberry Pi (`console_only_Pi` executable) |
| `src/gpio_line.*` | Linux GPIO character device (uAPI v2): edge-event input and output lines, pin lookup by name |
| `src/wheel_speed.*` | self ground speed from wheel pulses (`WheelSpeedEstimator`) |
| `src/safety_monitor.*` | closing speed, ground speed, two-second rule and approach alert per target |
| `src/alert_leds.*` | alert hold/flash/fail-safe state machine with GPIO, console and null LED backends |
| `src/alert_sound.*` | approach alert audio: runs a player (`aplay`) as a child process while the alert is active |
| `src/pi_selftest.*` | built-in logic tests (`console_only_Pi --self-test`) |
| `src/wheel_test.cpp` | wheel-speed sensor bench test (`wheel_test` executable) |
| `src/mono_clock.h` | CLOCK_MONOTONIC helper shared by the modules above |

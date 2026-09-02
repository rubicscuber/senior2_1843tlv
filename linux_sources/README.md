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

This produces the `tm_visualizer` executable. `make clean` removes it and the
`build/` directory.

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

## Source layout

| File | Ports |
|---|---|
| `src/main.cpp` | `tm_visualizer.m` — setup, mode selection, main loop, transforms |
| `src/cfg_parser.*` | `readCfgFile.m`, `defineCLICommands.m`, `parseCLICommands2Struct.m`, `calculateChirpParams.m` |
| `src/frame_parser.*` | `parseBytes_TM.m`, `getTLV.m`, `getGtrackFrameHeader.m`, `getGtrackPtCloud.m`, `getGtrackTargetList.m`, `getGtrackPtType.m`, `readDATFile2Buffer.m` |
| `src/serial_port.*` | `initCfgPort.m`, `initDataPort.m`, `loadCfg.m`, `readUARTtoBuffer.m` |
| `src/visualizer.*` | figure/plot code: `init3DPlot_TM.m`, `drawFOVLines.m`, `initLanes.m`, stats annotation |
| `src/tm_types.h` | shared frame/point-cloud/target structures |

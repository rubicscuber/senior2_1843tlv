// Traffic Monitoring terminal visualizer for Linux.
// C++ port of tm_visualizer.m with the MATLAB GUI (setup_tm app, figure,
// slider, popup menus) replaced by command-line options, an ANSI terminal
// plot and keyboard controls.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>

#include "cfg_parser.h"
#include "frame_parser.h"
#include "serial_port.h"
#include "tm_types.h"
#include "visualizer.h"

namespace {

constexpr double DEG2RAD = M_PI / 180.0;

std::atomic<bool> g_run{true}; // RUN_VIZ equivalent; cleared by SIGINT / 'q'

void signalHandler(int)
{
    g_run = false;
}

// ---- keyboard: raw, non-blocking stdin (replaces GUI controls) ----
struct TerminalGuard {
    termios saved{};
    bool active = false;

    void enableRaw()
    {
        if (!isatty(STDIN_FILENO))
            return;
        if (tcgetattr(STDIN_FILENO, &saved) != 0)
            return;
        termios raw = saved;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0)
            active = true;
    }

    ~TerminalGuard()
    {
        if (active)
            tcsetattr(STDIN_FILENO, TCSANOW, &saved);
        // show cursor again and drop to a fresh line
        if (isatty(STDOUT_FILENO)) {
            const char* restore = "\033[?25h\033[0m\n";
            (void)!::write(STDOUT_FILENO, restore, std::strlen(restore));
        }
    }
};

int readKey()
{
    unsigned char c;
    if (::read(STDIN_FILENO, &c, 1) == 1)
        return c;
    return -1;
}

// ---- transformations (tm_visualizer.m "Pre-compute transformation matrix") ----
struct Transform {
    double raz[3][3]; // azimuth rotation
    double rel[3][3]; // elevation rotation
    double height;

    Transform(const Offset& offset)
    {
        const double ca = std::cos(offset.az * DEG2RAD), sa = std::sin(offset.az * DEG2RAD);
        const double ce = std::cos(offset.el * DEG2RAD), se = std::sin(offset.el * DEG2RAD);
        const double az[3][3] = {{ca, -sa, 0}, {sa, ca, 0}, {0, 0, 1}};
        const double el[3][3] = {{1, 0, 0}, {0, ce, -se}, {0, se, ce}};
        std::memcpy(raz, az, sizeof(az));
        std::memcpy(rel, el, sizeof(el));
        height = offset.height;
    }

    // full transform (rotMat_az * rotMat_el), applied to the point cloud
    void applyFull(double& x, double& y, double& z) const
    {
        applyElevation(x, y, z);
        const double ex = x, ey = y;
        x = raz[0][0] * ex + raz[0][1] * ey;
        y = raz[1][0] * ex + raz[1][1] * ey;
    }

    // elevation-only transform; tracker output is already azimuth-rotated
    // on the device, so targets only get rotMat_el (see tm_visualizer.m)
    void applyElevation(double& x, double& y, double& z) const
    {
        const double ey = y, ez = z;
        (void)x;
        y = rel[1][1] * ey + rel[1][2] * ez;
        z = rel[2][1] * ey + rel[2][2] * ez;
    }
};

// Build render data from a parsed frame: transform coordinates, add the
// height offset and count targets per lane (the SHOW_LANES block).
RenderData buildRenderData(const Frame& frame, int numFramesAvailable,
                           const Transform& tf, const Lanes& lanes)
{
    RenderData d;
    d.frameNumber = frame.header.frameNumber;
    d.numFramesAvailable = numFramesAvailable;

    const bool havePtCloud = frame.havePointCloud && frame.detObj.numDetectedObj != 0;
    const bool haveTargets = frame.haveTargetList && !frame.targets.empty();

    d.numPoints = frame.havePointCloud ? static_cast<int>(frame.detObj.numDetectedObj) : -1;
    d.numTargets = frame.haveTargetList ? static_cast<int>(frame.targets.size()) : -1;

    if (havePtCloud) {
        for (uint32_t i = 0; i < frame.detObj.numDetectedObj; i++) {
            double x = frame.detObj.x[i], y = frame.detObj.y[i], z = frame.detObj.z[i];
            tf.applyFull(x, y, z);
            d.ptX.push_back(x);
            d.ptY.push_back(y);
            d.ptZ.push_back(z + tf.height);
        }
    }
    if (haveTargets) {
        for (size_t i = 0; i < frame.targets.size(); i++) {
            double x = frame.targets.posX[i], y = frame.targets.posY[i],
                   z = frame.targets.posZ[i];
            tf.applyElevation(x, y, z);
            d.tgtX.push_back(x);
            d.tgtY.push_back(y);
            d.tgtZ.push_back(z + tf.height);
            d.tgtId.push_back(frame.targets.tid[i]);
        }
    }

    if (lanes.enable) {
        d.laneCounts.assign(lanes.numLanes, 0);
        for (size_t i = 0; i < d.tgtX.size(); i++) {
            for (int l = 0; l < lanes.numLanes; l++) {
                const double xMin = lanes.x + lanes.w * l;
                const double xMax = lanes.x + lanes.w * (l + 1);
                if (d.tgtX[i] >= xMin && d.tgtX[i] < xMax
                    && d.tgtY[i] >= lanes.y && d.tgtY[i] < lanes.y + lanes.h)
                    d.laneCounts[l]++;
            }
        }
    }
    return d;
}

void printUsage(const char* prog)
{
    std::printf(
        "Traffic Monitoring terminal visualizer (Linux port of tm_visualizer.m)\n"
        "\n"
        "Usage:\n"
        "  %s -g <cfg-file> -f <dat-file> [options]              playback mode\n"
        "  %s -g <cfg-file> -d <data-dev> -c <cli-dev> [options]  real-time mode\n"
        "\n"
        "Modes (choose one):\n"
        "  -f, --file <dat>      play back a recorded UART stream (hex .dat or raw binary)\n"
        "  -d, --data <dev>      read live data from a serial device, e.g. /dev/ttyUSB1\n"
        "\n"
        "Options:\n"
        "  -g, --cfg <file>      chirp configuration (.cfg) file (required)\n"
        "  -c, --cli <dev>       CLI/config serial device, e.g. /dev/ttyUSB0 (real-time)\n"
        "  -r, --record <file>   record the raw stream as hex to <file> (real-time)\n"
        "      --height <m>      sensor mounting height in meters (default 0)\n"
        "      --az <deg>        azimuth tilt in degrees (default 0)\n"
        "      --el <deg>        elevation tilt in degrees, down is negative (default 0)\n"
        "      --lanes N,x,y,w,h lane counting: N lanes starting at (x,y), each w wide,\n"
        "                        h deep (meters)\n"
        "      --view xy|yz|xz   initial plot view (default xy)\n"
        "      --no-load         do not send the cfg to the device (already running)\n"
        "      --no-plot         print one stats line per frame instead of the plot\n"
        "      --paused          start playback paused\n"
        "  -h, --help            show this help\n"
        "\n"
        "Keys while running: 1/2/3 switch view, q quit;\n"
        "playback only: space play/pause, n next frame, b previous frame.\n",
        prog, prog);
}

struct Options {
    std::string cfgFile;
    std::string datFile;
    std::string dataDev;
    std::string cliDev;
    std::string recordFile;
    Offset offset;
    Lanes lanes;
    View view = View::XY;
    bool loadCfgToDevice = true;
    bool plot = true;
    bool startPaused = false;
    bool realTime = false;
};

bool parseLanes(const std::string& arg, Lanes& lanes)
{
    double n = 0;
    if (std::sscanf(arg.c_str(), "%lf,%lf,%lf,%lf,%lf",
                    &n, &lanes.x, &lanes.y, &lanes.w, &lanes.h) != 5)
        return false;
    lanes.numLanes = static_cast<int>(n);
    lanes.enable = lanes.numLanes > 0;
    return true;
}

bool parseArgs(int argc, char** argv, Options& opt)
{
    auto need = [&](int& i) -> const char* {
        if (i + 1 >= argc) {
            std::fprintf(stderr, "Error: %s requires an argument\n", argv[i]);
            return nullptr;
        }
        return argv[++i];
    };

    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        const char* v = nullptr;
        if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            return false;
        } else if (a == "-g" || a == "--cfg") {
            if (!(v = need(i))) return false;
            opt.cfgFile = v;
        } else if (a == "-f" || a == "--file") {
            if (!(v = need(i))) return false;
            opt.datFile = v;
        } else if (a == "-d" || a == "--data") {
            if (!(v = need(i))) return false;
            opt.dataDev = v;
        } else if (a == "-c" || a == "--cli") {
            if (!(v = need(i))) return false;
            opt.cliDev = v;
        } else if (a == "-r" || a == "--record") {
            if (!(v = need(i))) return false;
            opt.recordFile = v;
        } else if (a == "--height") {
            if (!(v = need(i))) return false;
            opt.offset.height = std::stod(v);
        } else if (a == "--az") {
            if (!(v = need(i))) return false;
            opt.offset.az = std::stod(v);
        } else if (a == "--el") {
            if (!(v = need(i))) return false;
            opt.offset.el = std::stod(v);
        } else if (a == "--lanes") {
            if (!(v = need(i))) return false;
            if (!parseLanes(v, opt.lanes)) {
                std::fprintf(stderr, "Error: --lanes expects N,x,y,w,h\n");
                return false;
            }
        } else if (a == "--view") {
            if (!(v = need(i))) return false;
            const std::string s = v;
            if (s == "xy") opt.view = View::XY;
            else if (s == "yz") opt.view = View::YZ;
            else if (s == "xz") opt.view = View::XZ;
            else {
                std::fprintf(stderr, "Error: unknown view '%s'\n", v);
                return false;
            }
        } else if (a == "--no-load") {
            opt.loadCfgToDevice = false;
        } else if (a == "--no-plot") {
            opt.plot = false;
        } else if (a == "--paused") {
            opt.startPaused = true;
        } else {
            std::fprintf(stderr, "Error: unknown option '%s'\n", a.c_str());
            printUsage(argv[0]);
            return false;
        }
    }

    if (opt.cfgFile.empty()) {
        std::fprintf(stderr, "Error: a cfg file (-g) is required.\n");
        printUsage(argv[0]);
        return false;
    }
    opt.realTime = !opt.dataDev.empty();
    if (!opt.realTime && opt.datFile.empty()) {
        std::fprintf(stderr, "Error: choose a mode: -f <dat-file> or -d <data-dev>.\n");
        printUsage(argv[0]);
        return false;
    }
    if (opt.realTime && opt.cliDev.empty() && opt.loadCfgToDevice) {
        std::fprintf(stderr,
                     "Error: real-time mode needs -c <cli-dev> (or --no-load if the "
                     "device is already configured and running).\n");
        return false;
    }
    if (!opt.realTime && !opt.recordFile.empty()) {
        std::printf("Note: recording is only available in real-time mode; ignoring -r.\n");
        opt.recordFile.clear();
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt))
        return 1;

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // flip azimuth since the transformations assume CCW is + direction
    opt.offset.az *= -1;

    // ---- read and parse the cfg file ----
    std::vector<std::string> cfgLines;
    if (!readCfgFile(opt.cfgFile, cfgLines)) {
        std::fprintf(stderr, "Error: Could not open CFG file. Quitting.\n");
        return 1;
    }
    const CliConfig cliCfg = parseCliCommands(cfgLines);
    const std::vector<ChirpParams> calcP = calculateChirpParams(cliCfg);
    if (calcP.empty()) {
        std::fprintf(stderr, "Error: no profileCfg found in CFG file. Quitting.\n");
        return 1;
    }
    printChirpParams(calcP);

    double maxRange = 0, framePeriodMs = 50;
    for (const auto& p : calcP) {
        maxRange = std::max(maxRange, p.rangeMax_m);
        framePeriodMs = p.frameTime_msec > 0 ? p.frameTime_msec : framePeriodMs;
    }

    // ---- data sources ----
    SerialPort dataPort, cfgPort;
    std::vector<uint8_t> bytesBuffer;
    std::vector<Frame> allFrames; // playback mode
    FILE* recordFid = nullptr;

    if (opt.realTime) {
        if (!dataPort.open(opt.dataDev, 921600))
            return 1;
        if (!opt.cliDev.empty() && !cfgPort.open(opt.cliDev, 115200))
            return 1;

        // port status check, as in tm_visualizer.m
        bool loadConfig = opt.loadCfgToDevice;
        if (dataPort.bytesAvailable() > 0) {
            std::printf("Device appears to already be running. Will not load a new "
                        "configuration. To load a new config, press NRST on the EVM "
                        "and try again.\n");
            loadConfig = false;
        } else if (loadConfig) {
            cfgPort.writeLine("version");
            usleep(500000); // let the response arrive
            std::string response;
            for (int i = 0; i < 10; i++) // version reports back ~10 lines
                response += cfgPort.readLine(200);
            if (!response.empty()) {
                std::printf("Test successful: CFG Port Opened & Data Received\n");
            } else {
                std::fprintf(stderr, "Port opened but no response received. Check "
                                     "port # and SOP mode on EVM\n");
                return 1;
            }
        }

        if (loadConfig && !loadCfg(cfgPort, cfgLines))
            return 1;

        if (!opt.recordFile.empty()) {
            recordFid = std::fopen(opt.recordFile.c_str(), "w");
            if (recordFid)
                std::printf("Opening %s. Ready to log data.\n", opt.recordFile.c_str());
            else
                std::printf("Error with log file name or path. No logging.\n");
        }
        bytesBuffer.reserve(BYTES_BUFFER_MAX_SIZE);
    } else {
        if (!readDatFileToBuffer(opt.datFile, bytesBuffer)) {
            std::fprintf(stderr, "Error: could not read data file %s. Quitting.\n",
                         opt.datFile.c_str());
            return 1;
        }
        int numFramesAvailable = 0;
        allFrames = parseBytesTM(bytesBuffer, ReadMode::ALL, numFramesAvailable);
        if (allFrames.empty()) {
            std::fprintf(stderr, "Error: no frames found in %s. Quitting.\n",
                         opt.datFile.c_str());
            return 1;
        }
        std::printf("Loaded %zu frames from %s.\n", allFrames.size(),
                    opt.datFile.c_str());
    }

    // ---- visualizer setup ----
    // ISK/BOOST antenna FOV (ANTENNA_TYPE 1 in tm_visualizer.m)
    const double azFOV = 120, elFOV = 40;
    TerminalViz viz(maxRange, azFOV, elFOV, opt.offset, opt.lanes);
    viz.setView(opt.view);
    const Transform tf(opt.offset);

    TerminalGuard term;
    if (opt.plot)
        term.enableRaw();

    // ---- main loop: parse UART / step frames and update the display ----
    bool paused = opt.startPaused;
    size_t frameIndex = 0;
    const int totalFrames = static_cast<int>(allFrames.size());

    while (g_run) {
        // keyboard (replaces the view popup, play control and frame slider)
        for (int key; (key = readKey()) != -1;) {
            switch (key) {
            case 'q': case 'Q': g_run = false; break;
            case '1': viz.setView(View::XY); break;
            case '2': viz.setView(View::YZ); break;
            case '3': viz.setView(View::XZ); break;
            case ' ': if (!opt.realTime) paused = !paused; break;
            case 'n': case 'N':
                if (!opt.realTime && frameIndex + 1 < allFrames.size())
                    frameIndex++;
                break;
            case 'b': case 'B':
                if (!opt.realTime && frameIndex > 0)
                    frameIndex--;
                break;
            default: break;
            }
        }
        if (!g_run)
            break;

        if (opt.realTime) {
            // read whatever is on the UART into the byte buffer
            uint8_t chunk[4096];
            while (bytesBuffer.size() < BYTES_BUFFER_MAX_SIZE) {
                const size_t room =
                    std::min(sizeof(chunk), BYTES_BUFFER_MAX_SIZE - bytesBuffer.size());
                const int n = dataPort.readBytes(chunk, room);
                if (n <= 0)
                    break;
                bytesBuffer.insert(bytesBuffer.end(), chunk, chunk + n);
                if (recordFid) {
                    for (int i = 0; i < n; i++)
                        std::fprintf(recordFid, "%02x ", chunk[i]);
                }
            }

            int numFramesAvailable = 0;
            std::vector<Frame> frames =
                parseBytesTM(bytesBuffer, ReadMode::FIFO, numFramesAvailable);
            if (!frames.empty() && frames.front().valid) {
                RenderData d =
                    buildRenderData(frames.front(), numFramesAvailable, tf, opt.lanes);
                if (opt.plot)
                    viz.render(d);
                else
                    TerminalViz::printStatsLine(d);
            }
            usleep(5000);
        } else {
            const Frame& frame = allFrames[frameIndex];
            RenderData d = buildRenderData(
                frame, totalFrames - static_cast<int>(frameIndex), tf, opt.lanes);
            d.playback = true;
            d.paused = paused;
            d.frameIndex = static_cast<int>(frameIndex) + 1;
            d.totalFrames = totalFrames;
            if (opt.plot) {
                viz.render(d);
            } else {
                TerminalViz::printStatsLine(d);
                if (frameIndex + 1 >= allFrames.size())
                    break; // no interactivity without the plot: stop at the end
            }
            if (!paused && frameIndex + 1 < allFrames.size())
                frameIndex++;
            usleep(static_cast<useconds_t>(framePeriodMs * 1000));
        }
    }

    // ---- close ports and files ----
    if (recordFid) {
        if (std::fclose(recordFid) == 0)
            std::printf("\nLog file closed w/o error.");
        else
            std::printf("\nError closing log file.");
    }
    dataPort.close();
    cfgPort.close();
    std::printf("\nVisualizer terminated.\n");
    return 0;
}

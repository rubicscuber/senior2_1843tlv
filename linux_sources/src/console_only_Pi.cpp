// console_only_Pi: target detection report for a Raspberry Pi console session.
//
// Plain line-by-line console output (no screen control, safe over SSH or a
// serial console): for every frame it states whether any tracked target is
// detected and, if so, prints each target's id, position, range, velocity,
// speed and acceleration as parsed from the TARGET_LIST TLV (type 1010).
// A summary is printed on exit.
//
// Takes the same input arguments as console_only:
//   console_only_Pi <capture.dat>                                (recorded stream)
//   console_only_Pi -d /dev/ttyACM1                              (live, Ctrl-C stops)
//   console_only_Pi -d /dev/ttyACM1 -c /dev/ttyACM0 -g <cfg>     (configure first)
//
// Builds as its own executable; see the Makefile's `console_only_Pi` target.

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include <signal.h>
#include <unistd.h>

#include "cfg_parser.h"
#include "frame_parser.h"
#include "serial_port.h"
#include "tm_types.h"

namespace {

std::atomic<bool> g_run{true};

void signalHandler(int)
{
    g_run = false;
}

// running totals for the exit summary
struct Stats {
    unsigned long framesSeen = 0;
    unsigned long framesWithTargets = 0;
    std::set<uint32_t> uniqueTids;
};

// Print one frame's detection state and target details.
void reportFrame(const Frame& frame, Stats& stats)
{
    stats.framesSeen++;
    const size_t numTargets = frame.haveTargetList ? frame.targets.size() : 0;
    const int numPoints =
        frame.havePointCloud ? static_cast<int>(frame.detObj.numDetectedObj) : 0;

    if (numTargets == 0) {
        std::printf("Frame %-6u | no target detected (%d point-cloud detections)\n",
                    frame.header.frameNumber, numPoints);
        return;
    }

    stats.framesWithTargets++;
    std::printf("Frame %-6u | TARGET DETECTED: %zu target(s), %d point-cloud detections\n",
                frame.header.frameNumber, numTargets, numPoints);

    const TargetList& t = frame.targets;
    for (size_t i = 0; i < numTargets; i++) {
        stats.uniqueTids.insert(t.tid[i]);
        const double range = std::sqrt(t.posX[i] * t.posX[i] + t.posY[i] * t.posY[i]
                                       + t.posZ[i] * t.posZ[i]);
        const double speed = std::sqrt(t.velX[i] * t.velX[i] + t.velY[i] * t.velY[i]
                                       + t.velZ[i] * t.velZ[i]);
        std::printf("  TID %-4u pos (%7.2f, %7.2f, %7.2f) m  range %6.2f m\n",
                    t.tid[i], t.posX[i], t.posY[i], t.posZ[i], range);
        std::printf("           vel (%7.2f, %7.2f, %7.2f) m/s  speed %6.2f m/s"
                    "  accel (%.2f, %.2f, %.2f) m/s^2\n",
                    t.velX[i], t.velY[i], t.velZ[i], speed,
                    t.accX[i], t.accY[i], t.accZ[i]);
    }
}

void printSummary(const Stats& stats)
{
    std::printf("\nSummary: %lu frame(s) processed, %lu with targets, "
                "%zu unique track id(s).\n",
                stats.framesSeen, stats.framesWithTargets, stats.uniqueTids.size());
}

int runFile(const std::string& path)
{
    std::vector<uint8_t> buf;
    if (!readDatFileToBuffer(path, buf)) {
        std::fprintf(stderr, "Error: could not read data file %s.\n", path.c_str());
        return 1;
    }
    std::printf("Read %zu bytes from %s.\n", buf.size(), path.c_str());

    int numFramesAvailable = 0;
    const std::vector<Frame> frames = parseBytesTM(buf, ReadMode::ALL, numFramesAvailable);
    if (frames.empty()) {
        std::fprintf(stderr, "Error: no frames found in %s.\n", path.c_str());
        return 1;
    }

    Stats stats;
    for (const Frame& frame : frames) {
        if (!g_run)
            break;
        if (frame.valid)
            reportFrame(frame, stats);
    }
    printSummary(stats);
    return 0;
}

int runSerial(const std::string& device, const std::string& cliDevice,
              const std::string& cfgFile)
{
    SerialPort port;
    if (!port.open(device, 921600))
        return 1;

    // optionally send the chirp configuration over the CLI port first,
    // following the same port-status logic as tm_visualizer
    if (!cliDevice.empty()) {
        std::vector<std::string> cfgLines;
        if (!readCfgFile(cfgFile, cfgLines)) {
            std::fprintf(stderr, "Error: Could not open CFG file. Quitting.\n");
            return 1;
        }
        if (port.bytesAvailable() > 0) {
            std::printf("Device appears to already be running. Will not load a "
                        "new configuration. To load a new config, press NRST on "
                        "the EVM and try again.\n");
        } else {
            SerialPort cliPort;
            if (!cliPort.open(cliDevice, 115200))
                return 1;
            if (!loadCfg(cliPort, cfgLines))
                return 1;
        }
    }

    std::printf("Watching for targets on %s. Press Ctrl-C to stop.\n", device.c_str());

    Stats stats;
    std::vector<uint8_t> buf;
    buf.reserve(BYTES_BUFFER_MAX_SIZE);
    uint8_t chunk[4096];
    while (g_run) {
        const int n = port.readBytes(chunk, sizeof(chunk));
        if (n > 0)
            buf.insert(buf.end(), chunk, chunk + n);

        int numFramesAvailable = 0;
        const std::vector<Frame> frames =
            parseBytesTM(buf, ReadMode::FIFO, numFramesAvailable);
        if (!frames.empty()) {
            if (frames.front().valid) {
                reportFrame(frames.front(), stats);
                std::fflush(stdout);
            }
        } else if (n <= 0) {
            usleep(5000); // nothing read and no frame pending: idle briefly
        }
    }
    printSummary(stats);
    return 0;
}

void printUsage(const char* prog)
{
    std::printf("TM target detection report (plain console output, Raspberry Pi friendly)\n"
                "\n"
                "Usage:\n"
                "  %s <capture.dat>       report from a recorded stream (hex .dat or binary)\n"
                "  %s -d <device>         report from a live data UART, e.g. -d /dev/ttyACM1\n"
                "\n"
                "Options (live mode):\n"
                "  -c <device>   CLI/config UART, e.g. /dev/ttyACM0; requires -g\n"
                "  -g <file>     chirp cfg file to send to the device before reporting\n",
                prog, prog);
}

} // namespace

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::string datFile, dataDev, cliDev, cfgFile;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        if ((a == "-d" || a == "-c" || a == "-g") && i + 1 >= argc) {
            std::fprintf(stderr, "Error: %s requires an argument\n", a.c_str());
            return 1;
        }
        if (a == "-d")
            dataDev = argv[++i];
        else if (a == "-c")
            cliDev = argv[++i];
        else if (a == "-g")
            cfgFile = argv[++i];
        else if (a[0] != '-' && datFile.empty())
            datFile = a;
        else {
            std::fprintf(stderr, "Error: unknown option '%s'\n", a.c_str());
            printUsage(argv[0]);
            return 1;
        }
    }

    if (!dataDev.empty()) {
        if (cliDev.empty() != cfgFile.empty()) {
            std::fprintf(stderr,
                         "Error: -c and -g must be used together (a CLI port to "
                         "send on and a cfg file to send).\n");
            return 1;
        }
        return runSerial(dataDev, cliDev, cfgFile);
    }
    if (!datFile.empty()) {
        if (!cliDev.empty() || !cfgFile.empty())
            std::printf("Note: -c/-g only apply to live mode (-d); ignoring.\n");
        return runFile(datFile);
    }

    printUsage(argv[0]);
    return 1;
}

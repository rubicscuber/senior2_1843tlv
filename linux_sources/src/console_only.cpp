// console_only: dump raw Traffic Monitoring UART packets to stdout.
//
// Prints every frame's header fields and each TLV (type, length, payload)
// as a labelled hex dump — no plot, no screen control, just plain console
// output suitable for piping to a file. Reads either a recorded .dat stream
// (hex text or raw binary, auto-detected) or a live data UART.
//
// Usage:
//   console_only <capture.dat>
//   console_only -d /dev/ttyACM1                                (Ctrl-C to stop)
//   console_only -d /dev/ttyACM1 -c /dev/ttyACM0 -g <cfg-file>  (configure first)
//
// Builds as its own executable; see the Makefile's `console_only` target.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <signal.h>
#include <unistd.h>

#include "cfg_parser.h"
#include "frame_parser.h"
#include "serial_port.h"

namespace {

const uint8_t MAGIC_WORD[8] = {2, 1, 4, 3, 6, 5, 8, 7};
constexpr size_t HEADER_LEN = 8 + 8 * 4; // magic word + 8 uint32 fields

std::atomic<bool> g_run{true};

void signalHandler(int)
{
    g_run = false;
}

uint32_t readU32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

const char* tlvTypeName(uint32_t type)
{
    switch (type) {
    case 1000: return "TRACKERPROC_DETECTED_POINTS";
    case 1010: return "TRACKERPROC_TARGET_LIST";
    case 1011: return "TRACKERPROC_TARGET_INDEX";
    case 9:    return "MMW_SIDE_INFO";
    default:   return "UNKNOWN";
    }
}

// Offset of the next magic word at or after `from`; npos if none.
size_t findMagic(const std::vector<uint8_t>& buf, size_t from)
{
    if (buf.size() < sizeof(MAGIC_WORD))
        return std::string::npos;
    for (size_t i = from; i + sizeof(MAGIC_WORD) <= buf.size(); i++) {
        if (std::memcmp(buf.data() + i, MAGIC_WORD, sizeof(MAGIC_WORD)) == 0)
            return i;
    }
    return std::string::npos;
}

void hexDump(const uint8_t* data, size_t len, const char* indent)
{
    for (size_t i = 0; i < len; i += 16) {
        std::printf("%s%04zx:", indent, i);
        for (size_t j = i; j < i + 16 && j < len; j++)
            std::printf(" %02x", data[j]);
        std::printf("\n");
    }
}

// Print one complete frame packet as a labelled dump.
void dumpFrame(const uint8_t* pkt, size_t pktLen, size_t streamOffset)
{
    std::printf("\n=== Frame packet at stream offset 0x%zx (%zu bytes) ===\n",
                streamOffset, pktLen);

    std::printf("Header:\n");
    std::printf("  magicWord:     ");
    for (int i = 0; i < 8; i++)
        std::printf("%02x ", pkt[i]);
    std::printf("\n");
    const uint32_t packetLength = readU32(pkt + 12);
    const uint32_t numTLVs = readU32(pkt + 32);
    std::printf("  version:        0x%08x\n", readU32(pkt + 8));
    std::printf("  packetLength:   %u\n", packetLength);
    std::printf("  platform:       0x%x\n", readU32(pkt + 16));
    std::printf("  frameNumber:    %u\n", readU32(pkt + 20));
    std::printf("  timestamp:      %u\n", readU32(pkt + 24));
    std::printf("  numDetectedObj: %u\n", readU32(pkt + 28));
    std::printf("  numTLVs:        %u\n", numTLVs);
    std::printf("  subFrameNumber: %u\n", readU32(pkt + 36));
    if (packetLength != pktLen)
        std::printf("  WARNING: packetLength does not match actual packet size %zu\n",
                    pktLen);

    size_t idx = HEADER_LEN;
    for (uint32_t t = 0; t < numTLVs; t++) {
        if (idx + 8 > pktLen) {
            std::printf("TLV %u: TRUNCATED (no room for TLV header)\n", t + 1);
            break;
        }
        const uint32_t type = readU32(pkt + idx);
        const uint32_t length = readU32(pkt + idx + 4);
        idx += 8;
        std::printf("TLV %u: type=%u (%s), length=%u", t + 1, type,
                    tlvTypeName(type), length);
        switch (type) {
        case 1000: std::printf("  [%u points x 16 bytes]", length / 16); break;
        case 1010: std::printf("  [%u targets x 112 bytes]", length / 112); break;
        case 1011: std::printf("  [%u point indices]", length); break;
        default: break;
        }
        std::printf("\n");

        const size_t avail = std::min(static_cast<size_t>(length), pktLen - idx);
        hexDump(pkt + idx, avail, "    ");
        if (avail < length)
            std::printf("    TRUNCATED: %zu of %u payload bytes present\n",
                        avail, length);
        idx += avail;
    }

    if (idx < pktLen) {
        std::printf("Trailing bytes after TLVs (%zu):\n", pktLen - idx);
        hexDump(pkt + idx, pktLen - idx, "    ");
    }
}

// Dump every complete frame in `buf`, delimited by magic words. Returns the
// offset of the last (still incomplete) frame so live mode can keep it.
// `streamOffset` is the absolute position of buf[0] in the overall stream.
// When `flush` is set, the trailing frame is dumped too (end of a file).
size_t dumpBuffer(const std::vector<uint8_t>& buf, size_t streamOffset, bool flush)
{
    size_t pos = findMagic(buf, 0);
    if (pos == std::string::npos)
        return buf.size(); // no frame start: nothing to keep
    if (pos > 0)
        std::printf("Skipping %zu bytes before first magic word.\n", pos);

    while (g_run) {
        const size_t next = findMagic(buf, pos + sizeof(MAGIC_WORD));
        if (next != std::string::npos) {
            dumpFrame(buf.data() + pos, next - pos, streamOffset + pos);
            pos = next;
        } else {
            if (flush)
                dumpFrame(buf.data() + pos, buf.size() - pos, streamOffset + pos);
            break;
        }
    }
    return pos;
}

int runFile(const std::string& path)
{
    std::vector<uint8_t> buf;
    if (!readDatFileToBuffer(path, buf)) {
        std::fprintf(stderr, "Error: could not read data file %s.\n", path.c_str());
        return 1;
    }
    std::printf("Read %zu bytes from %s.\n", buf.size(), path.c_str());
    dumpBuffer(buf, 0, true);
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

    std::printf("Dumping raw TLV packets from %s. Press Ctrl-C to stop.\n",
                device.c_str());

    std::vector<uint8_t> buf;
    size_t streamOffset = 0;
    uint8_t chunk[4096];
    while (g_run) {
        const int n = port.readBytes(chunk, sizeof(chunk));
        if (n > 0) {
            buf.insert(buf.end(), chunk, chunk + n);
            const size_t keepFrom = dumpBuffer(buf, streamOffset, false);
            if (keepFrom > 0) {
                buf.erase(buf.begin(), buf.begin() + keepFrom);
                streamOffset += keepFrom;
            }
            std::fflush(stdout);
        } else {
            usleep(5000);
        }
    }
    // dump whatever is left of the frame in progress
    if (!buf.empty())
        dumpBuffer(buf, streamOffset, true);
    return 0;
}

void printUsage(const char* prog)
{
    std::printf("Raw TM TLV packet dump (labelled hex, console output only)\n"
                "\n"
                "Usage:\n"
                "  %s <capture.dat>       dump a recorded stream (hex .dat or raw binary)\n"
                "  %s -d <device>         dump a live data UART, e.g. -d /dev/ttyACM1\n"
                "\n"
                "Options (live mode):\n"
                "  -c <device>   CLI/config UART, e.g. /dev/ttyACM0; requires -g\n"
                "  -g <file>     chirp cfg file to send to the device before dumping\n",
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

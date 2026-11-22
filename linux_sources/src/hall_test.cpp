// hall_test: bench test for the Littelfuse 55100-3M hall effect switch on a
// Raspberry Pi.
//
// The 55100-3M has an open-collector (sinking) output: it pulls OUT low while
// a magnet (south pole facing the sensor) is in range and releases it
// otherwise, so the line needs a pull-up and reads
//     1 = no magnet, 0 = magnet present.
//
// Wiring (Pi 3 header):
//     red   (VDD) -> pin 1  (3.3V)   [pin 2 (5V) also OK, see README]
//     black (GND) -> pin 6  (GND)
//     blue  (OUT) -> pin 11 (GPIO17), plus 4.7k-10k pull-up to 3.3V
//                    (the internal ~50k pull-up is enabled by default and is
//                    enough on the bench)
//
// Uses the Linux GPIO character device (v2 uAPI, kernel 5.10+) directly, so no
// external libraries are needed. Every edge is timestamped by the kernel, so
// pulse intervals / RPM are accurate even if this process is briefly delayed.

#include <cerrno>
#include <cinttypes>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <linux/gpio.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

volatile sig_atomic_t g_run = 1;

void signalHandler(int)
{
    g_run = 0;
}

struct Options {
    std::string chip = "/dev/gpiochip0";
    unsigned line = 17;
    bool pullUp = true;
    double pulsesPerRev = 1.0;
    unsigned debounceUs = 0;
};

void printUsage(const char* prog)
{
    std::printf("Hall effect switch test (Littelfuse 55100-3M, active-low output)\n"
                "\n"
                "Usage: %s [options]\n"
                "\n"
                "Options:\n"
                "  -l, --line <n>        BCM GPIO number of the OUT wire (default 17 = pin 11)\n"
                "  -c, --chip <dev>      GPIO chip device (default /dev/gpiochip0)\n"
                "  -p, --ppr <n>         magnets/pulses per revolution, for RPM (default 1)\n"
                "      --no-pullup       disable the internal pull-up (use with an external one)\n"
                "      --debounce <us>   ignore edges shorter than this (default 0 = off)\n"
                "  -h, --help            show this help\n"
                "\n"
                "Prints the pin state at start, one line per magnet arrival/departure,\n"
                "a heartbeat every 5 s while idle, and a summary on Ctrl-C.\n",
                prog);
}

bool parseArgs(int argc, char** argv, Options& opt)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: %s requires an argument\n", a.c_str());
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (a == "-l" || a == "--line") {
            const char* v = next();
            if (!v)
                return false;
            opt.line = static_cast<unsigned>(std::strtoul(v, nullptr, 10));
        } else if (a == "-c" || a == "--chip") {
            const char* v = next();
            if (!v)
                return false;
            opt.chip = v;
        } else if (a == "-p" || a == "--ppr") {
            const char* v = next();
            if (!v)
                return false;
            opt.pulsesPerRev = std::strtod(v, nullptr);
            if (opt.pulsesPerRev <= 0) {
                std::fprintf(stderr, "Error: --ppr must be > 0\n");
                return false;
            }
        } else if (a == "--no-pullup") {
            opt.pullUp = false;
        } else if (a == "--debounce") {
            const char* v = next();
            if (!v)
                return false;
            opt.debounceUs = static_cast<unsigned>(std::strtoul(v, nullptr, 10));
        } else {
            std::fprintf(stderr, "Error: unknown option '%s'\n\n", a.c_str());
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

const char* stateName(int value)
{
    return value ? "no magnet (1)" : "MAGNET    (0)";
}

double nsToMs(uint64_t ns)
{
    return static_cast<double>(ns) / 1e6;
}

} // namespace

int main(int argc, char** argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt))
        return 1;

    setvbuf(stdout, nullptr, _IOLBF, 0); // line-buffered even when piped to a file

    struct sigaction sa{};
    sa.sa_handler = signalHandler; // no SA_RESTART: let poll() return on Ctrl-C
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    const int chipFd = ::open(opt.chip.c_str(), O_RDONLY | O_CLOEXEC);
    if (chipFd < 0) {
        std::fprintf(stderr, "Error: cannot open %s: %s\n", opt.chip.c_str(), std::strerror(errno));
        if (errno == EACCES)
            std::fprintf(stderr, "Add your user to the 'gpio' group or run with sudo.\n");
        return 1;
    }

    gpio_v2_line_request req{};
    req.offsets[0] = opt.line;
    req.num_lines = 1;
    std::strncpy(req.consumer, "hall_test", sizeof(req.consumer) - 1);
    req.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING |
                       GPIO_V2_LINE_FLAG_EDGE_FALLING |
                       (opt.pullUp ? GPIO_V2_LINE_FLAG_BIAS_PULL_UP : GPIO_V2_LINE_FLAG_BIAS_DISABLED);
    if (opt.debounceUs > 0) {
        req.config.num_attrs = 1;
        req.config.attrs[0].mask = 1; // applies to line index 0
        req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
        req.config.attrs[0].attr.debounce_period_us = opt.debounceUs;
    }

    if (::ioctl(chipFd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
        std::fprintf(stderr, "Error: cannot request GPIO%u on %s: %s\n", opt.line,
                     opt.chip.c_str(), std::strerror(errno));
        if (errno == EBUSY)
            std::fprintf(stderr, "The line is in use by another program or driver.\n");
        ::close(chipFd);
        return 1;
    }
    ::close(chipFd); // the line fd stays valid on its own
    const int lineFd = req.fd;

    gpio_v2_line_values vals{};
    vals.mask = 1;
    int state = -1;
    if (::ioctl(lineFd, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals) == 0)
        state = static_cast<int>(vals.bits & 1);

    std::printf("Hall test on %s line %u (BCM GPIO%u), pull-up %s%s\n", opt.chip.c_str(),
                opt.line, opt.line, opt.pullUp ? "on" : "off",
                opt.debounceUs ? (", debounce " + std::to_string(opt.debounceUs) + " us").c_str() : "");
    std::printf("Initial state: %s\n", state < 0 ? "unknown" : stateName(state));
    std::printf("Pass the magnet (south pole) over the sensor. Ctrl-C to stop.\n\n");

    uint64_t pulses = 0, releases = 0, missed = 0;
    uint64_t lastFallNs = 0, firstFallNs = 0;
    uint64_t minPeriodNs = UINT64_MAX, maxPeriodNs = 0;
    uint32_t lastSeqno = 0;
    int idleTicks = 0;

    while (g_run) {
        pollfd pfd{lineFd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, 1000);
        if (pr < 0) {
            if (errno == EINTR)
                continue;
            std::perror("poll");
            break;
        }
        if (pr == 0) {
            if (++idleTicks >= 5) {
                idleTicks = 0;
                if (::ioctl(lineFd, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals) == 0)
                    std::printf("  ... waiting, pin = %s, pulses so far %" PRIu64 "\n",
                                stateName(static_cast<int>(vals.bits & 1)), pulses);
            }
            continue;
        }
        idleTicks = 0;

        gpio_v2_line_event events[16];
        const ssize_t n = ::read(lineFd, events, sizeof(events));
        if (n < 0) {
            if (errno == EINTR)
                continue;
            std::perror("read");
            break;
        }

        for (size_t i = 0; i < static_cast<size_t>(n) / sizeof(events[0]); ++i) {
            const gpio_v2_line_event& ev = events[i];
            if (lastSeqno && ev.line_seqno != lastSeqno + 1)
                missed += ev.line_seqno - lastSeqno - 1; // kernel event buffer overflowed
            lastSeqno = ev.line_seqno;

            if (ev.id == GPIO_V2_LINE_EVENT_FALLING_EDGE) {
                ++pulses;
                std::printf("[%10.3f s] MAGNET ON   pulse #%" PRIu64, ev.timestamp_ns / 1e9, pulses);
                if (lastFallNs) {
                    const uint64_t period = ev.timestamp_ns - lastFallNs;
                    if (period < minPeriodNs)
                        minPeriodNs = period;
                    if (period > maxPeriodNs)
                        maxPeriodNs = period;
                    const double hz = 1e9 / static_cast<double>(period);
                    std::printf("  interval %9.2f ms  %8.2f Hz  %9.1f RPM", nsToMs(period), hz,
                                hz * 60.0 / opt.pulsesPerRev);
                } else {
                    firstFallNs = ev.timestamp_ns;
                }
                std::printf("\n");
                lastFallNs = ev.timestamp_ns;
            } else {
                ++releases;
                std::printf("[%10.3f s] MAGNET OFF", ev.timestamp_ns / 1e9);
                if (lastFallNs)
                    std::printf("  held %9.2f ms", nsToMs(ev.timestamp_ns - lastFallNs));
                std::printf("\n");
            }
        }
    }

    ::close(lineFd);

    std::printf("\n---- summary ----\n");
    std::printf("Magnet arrivals (falling edges): %" PRIu64 "\n", pulses);
    std::printf("Magnet departures (rising edges): %" PRIu64 "\n", releases);
    if (pulses >= 2) {
        const double avgNs = static_cast<double>(lastFallNs - firstFallNs) / static_cast<double>(pulses - 1);
        std::printf("Interval min / avg / max: %.2f / %.2f / %.2f ms (avg %.1f RPM at %g ppr)\n",
                    nsToMs(minPeriodNs), avgNs / 1e6, nsToMs(maxPeriodNs),
                    60e9 / avgNs / opt.pulsesPerRev, opt.pulsesPerRev);
    }
    if (missed)
        std::printf("WARNING: %" PRIu64 " edge events were dropped by the kernel buffer\n", missed);
    if (pulses == 0)
        std::printf("No pulses seen: check wiring/pull-up, try the other magnet pole, move closer.\n");
    return 0;
}

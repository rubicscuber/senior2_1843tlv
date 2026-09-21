// wheel_test: bench test for the wheel-speed input of console_only_Pi.
//
// Exercises exactly the code path the safety monitor uses for the self
// speed: the hall-effect sensor's GPIO edge events (gpio_line.cpp), the
// dropped-event accounting through the kernel's line sequence number, and
// the WheelSpeedEstimator (wheel_speed.cpp). No radar needed.
//
//   wheel_test --pulse-gpio 17                        spin the wheel / pass the magnet by hand
//   wheel_test --pulse-gpio 17 --loopback-gpio 27 --loopback-speed 10
//                                                     no wheel: drive GPIO27 as a pulse
//                                                     generator, jumper it to GPIO17, and
//                                                     the estimate must read 10 m/s
//   wheel_test --sim 10                               no hardware: synthetic pulses only
//
// Prints each pulse (interval, instantaneous and averaged speed) and a status
// line every --interval seconds with the speed, the pulse counters, the time
// since the last pulse and the input's current level, then a summary.
// Takes the same wheel options as console_only_Pi so a working command line
// can be copied over 1:1.

#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <signal.h>
#include <unistd.h>

#include "gpio_line.h"
#include "mono_clock.h"
#include "wheel_speed.h"

namespace {

std::atomic<bool> g_run{true};

void signalHandler(int)
{
    g_run = false;
}

constexpr uint64_t NS_PER_S = 1000000000ull;

struct Options {
    int pulseGpio = -1;
    WheelParams wheel;
    GpioInput::Config pulseCfg;
    std::string gpiochip;
    double statusInterval = 0.5; // s between status lines
    double duration = 0.0;       // s; 0 = until Ctrl-C
    bool quietPulses = false;
    int loopbackGpio = -1;       // output pin driven as a pulse generator
    double loopbackSpeed = -1.0; // m/s the generator represents
    double simSpeed = -1.0;      // m/s of synthetic pulses (no hardware)
    double genStopAfter = 0.0;   // s; stop the generator then (shows the decay to 0); 0 = never
};

void printUsage(const char* prog)
{
    std::printf(
        "Wheel-speed bench test for console_only_Pi's safety monitor\n"
        "\n"
        "Usage:\n"
        "  %s --pulse-gpio <bcm> [wheel options]                 read the real sensor\n"
        "  %s --pulse-gpio <bcm> --loopback-gpio <bcm> --loopback-speed <m/s>\n"
        "                                                      generate pulses on an output pin\n"
        "                                                      (jumper it to the input pin)\n"
        "  %s --sim <m/s>                                       no hardware: synthetic pulses\n"
        "\n"
        "Wheel options (same meaning and defaults as console_only_Pi):\n"
        "  --wheel-diameter <m>        wheel diameter (default 0.7)\n"
        "  --pulses-per-rev <n>        pulses per wheel revolution (default 1)\n"
        "  --pulse-edge rising|falling  counted edge (default falling)\n"
        "  --pulse-bias pull-up|pull-down|none   input bias (default pull-up)\n"
        "  --pulse-debounce-us <n>     kernel debounce, 0 = off (default 500)\n"
        "  --max-speed <m/s>           glitch filter: faster pulses are ignored (default 40)\n"
        "  --stop-timeout <s>          speed reads 0 after this long without a pulse (default 2)\n"
        "  --gpiochip <path>           use this chip with offset = BCM number (default: auto)\n"
        "\n"
        "Test options:\n"
        "  --interval <s>              seconds between status lines (default 0.5)\n"
        "  --duration <s>              stop after this long (default: until Ctrl-C)\n"
        "  --quiet-pulses              do not print every pulse\n"
        "  --gen-stop-after <s>        loopback/sim: stop generating pulses after this long,\n"
        "                              to watch the estimate decay to 0 (wheel stopped)\n"
        "\n"
        "Wiring: hall switch output -> pulse GPIO (internal pull-up), GND common, VCC 3.3 V.\n"
        "Loopback: a jumper from the loopback GPIO straight to the pulse GPIO, nothing else.\n"
        "The idle level should read 1 with the default wiring; 0 means the magnet is parked\n"
        "at the sensor or the wiring/pull-up is wrong.\n",
        prog, prog, prog);
}

bool parseDouble(const char* flag, const char* text, double& out)
{
    char* end = nullptr;
    errno = 0;
    out = std::strtod(text, &end);
    if (end == text || *end != '\0' || errno != 0 || !std::isfinite(out)) {
        std::fprintf(stderr, "Error: %s expects a number, got '%s'\n", flag, text);
        return false;
    }
    return true;
}

bool parseInt(const char* flag, const char* text, int& out)
{
    char* end = nullptr;
    errno = 0;
    const long v = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || errno != 0 || v < -1000000 || v > 1000000) {
        std::fprintf(stderr, "Error: %s expects an integer, got '%s'\n", flag, text);
        return false;
    }
    out = static_cast<int>(v);
    return true;
}

bool parseArgs(int argc, char** argv, Options& opt, int& exitCode)
{
    exitCode = 1;
    opt.wheel.diameterM = 0.7;
    opt.wheel.pulsesPerRev = 1;

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
        double d = 0;
        int n = 0;
        if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            exitCode = 0;
            return false;
        } else if (a == "--pulse-gpio") {
            if (!(v = need(i)) || !parseInt("--pulse-gpio", v, n)) return false;
            opt.pulseGpio = n;
        } else if (a == "--wheel-diameter") {
            if (!(v = need(i)) || !parseDouble("--wheel-diameter", v, d)) return false;
            opt.wheel.diameterM = d;
        } else if (a == "--pulses-per-rev") {
            if (!(v = need(i)) || !parseInt("--pulses-per-rev", v, n)) return false;
            opt.wheel.pulsesPerRev = n;
        } else if (a == "--pulse-edge") {
            if (!(v = need(i))) return false;
            const std::string s = v;
            if (s == "rising") opt.pulseCfg.edge = GpioInput::Edge::Rising;
            else if (s == "falling") opt.pulseCfg.edge = GpioInput::Edge::Falling;
            else { std::fprintf(stderr, "Error: --pulse-edge expects rising or falling\n"); return false; }
        } else if (a == "--pulse-bias") {
            if (!(v = need(i))) return false;
            const std::string s = v;
            if (s == "pull-up") opt.pulseCfg.bias = GpioInput::Bias::PullUp;
            else if (s == "pull-down") opt.pulseCfg.bias = GpioInput::Bias::PullDown;
            else if (s == "none") opt.pulseCfg.bias = GpioInput::Bias::None;
            else { std::fprintf(stderr, "Error: --pulse-bias expects pull-up, pull-down or none\n"); return false; }
        } else if (a == "--pulse-debounce-us") {
            if (!(v = need(i)) || !parseInt("--pulse-debounce-us", v, n)) return false;
            if (n < 0) { std::fprintf(stderr, "Error: --pulse-debounce-us must be >= 0\n"); return false; }
            opt.pulseCfg.debounceUs = static_cast<uint32_t>(n);
        } else if (a == "--max-speed") {
            if (!(v = need(i)) || !parseDouble("--max-speed", v, d)) return false;
            opt.wheel.maxSpeedMps = d;
        } else if (a == "--stop-timeout") {
            if (!(v = need(i)) || !parseDouble("--stop-timeout", v, d)) return false;
            opt.wheel.stopTimeoutS = d;
        } else if (a == "--gpiochip") {
            if (!(v = need(i))) return false;
            opt.gpiochip = v;
        } else if (a == "--interval") {
            if (!(v = need(i)) || !parseDouble("--interval", v, d)) return false;
            if (!(d > 0)) { std::fprintf(stderr, "Error: --interval must be > 0\n"); return false; }
            opt.statusInterval = d;
        } else if (a == "--duration") {
            if (!(v = need(i)) || !parseDouble("--duration", v, d)) return false;
            if (d < 0) { std::fprintf(stderr, "Error: --duration must be >= 0\n"); return false; }
            opt.duration = d;
        } else if (a == "--quiet-pulses") {
            opt.quietPulses = true;
        } else if (a == "--gen-stop-after") {
            if (!(v = need(i)) || !parseDouble("--gen-stop-after", v, d)) return false;
            if (d < 0) { std::fprintf(stderr, "Error: --gen-stop-after must be >= 0\n"); return false; }
            opt.genStopAfter = d;
        } else if (a == "--loopback-gpio") {
            if (!(v = need(i)) || !parseInt("--loopback-gpio", v, n)) return false;
            opt.loopbackGpio = n;
        } else if (a == "--loopback-speed") {
            if (!(v = need(i)) || !parseDouble("--loopback-speed", v, d)) return false;
            if (!(d > 0)) { std::fprintf(stderr, "Error: --loopback-speed must be > 0\n"); return false; }
            opt.loopbackSpeed = d;
        } else if (a == "--sim") {
            if (!(v = need(i)) || !parseDouble("--sim", v, d)) return false;
            if (!(d > 0)) { std::fprintf(stderr, "Error: --sim must be > 0\n"); return false; }
            opt.simSpeed = d;
        } else {
            std::fprintf(stderr, "Error: unknown option '%s'\n", a.c_str());
            printUsage(argv[0]);
            return false;
        }
    }

    std::string err;
    if (!opt.wheel.validate(err)) {
        std::fprintf(stderr, "Error: %s\n", err.c_str());
        return false;
    }
    const bool sim = opt.simSpeed > 0;
    const bool live = opt.pulseGpio >= 0;
    if (sim == live) {
        std::fprintf(stderr, sim ? "Error: --sim cannot be combined with --pulse-gpio.\n"
                                 : "Error: give --pulse-gpio <bcm> (real sensor) or --sim <m/s>.\n");
        printUsage(argv[0]);
        return false;
    }
    if ((opt.loopbackGpio >= 0) != (opt.loopbackSpeed > 0)) {
        std::fprintf(stderr, "Error: --loopback-gpio and --loopback-speed go together.\n");
        return false;
    }
    if (opt.loopbackGpio >= 0 && sim) {
        std::fprintf(stderr, "Error: --loopback-gpio needs --pulse-gpio, not --sim.\n");
        return false;
    }
    if (opt.loopbackGpio >= 0 && opt.loopbackGpio == opt.pulseGpio) {
        std::fprintf(stderr, "Error: the loopback pin must differ from the pulse pin.\n");
        return false;
    }
    const double genSpeed = sim ? opt.simSpeed : opt.loopbackSpeed;
    if (genSpeed > 0 && genSpeed > opt.wheel.maxSpeedMps) {
        std::fprintf(stderr, "Error: the generated speed exceeds --max-speed (%.1f m/s): every pulse "
                             "would be rejected as a glitch.\n", opt.wheel.maxSpeedMps);
        return false;
    }
    if (opt.genStopAfter > 0 && genSpeed <= 0)
        std::printf("Note: --gen-stop-after only applies to --loopback-speed / --sim; ignoring.\n");
    return true;
}

// Feeds pulses into the estimator and prints them; shared by the real input
// and the simulation.
struct PulseSink {
    WheelSpeedEstimator& est;
    uint64_t epoch;
    bool quiet;
    uint32_t lastSeqno = 0;
    uint64_t lastPulseNs = 0;

    void feed(uint64_t tNs, uint32_t count, const char* source)
    {
        const bool accepted = est.addPulse(tNs, count);
        if (accepted)
            lastPulseNs = tNs;
        if (quiet)
            return;
        const double t = (tNs >= epoch) ? (tNs - epoch) * 1e-9 : 0.0;
        std::printf("[PULSE] t=%8.3f s  %-8s x%-2u  interval %7.1f ms  instant %6.2f m/s  avg %6.2f m/s  %s\n",
                    t, accepted ? "accepted" : "REJECTED", count,
                    est.lastIntervalSeconds() * 1e3, est.lastIntervalSpeed(), est.speedAt(tNs), source);
        if (!accepted)
            std::printf("        (rejected: faster than --max-speed, or a timestamp not after the previous pulse)\n");
    }

    void feedEvents(const std::vector<GpioInput::Event>& events)
    {
        for (const auto& e : events) {
            // as in console_only_Pi: a gap in the kernel's line_seqno is a
            // known number of events dropped from the queue
            uint32_t count = 1;
            if (lastSeqno != 0 && e.lineSeqno > lastSeqno)
                count = e.lineSeqno - lastSeqno;
            lastSeqno = e.lineSeqno;
            feed(e.timestampNs, count, e.rising ? "rising" : "falling");
        }
    }
};

} // namespace

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    Options opt;
    int exitCode = 1;
    if (!parseArgs(argc, argv, opt, exitCode))
        return exitCode;

    const bool sim = opt.simSpeed > 0;
    WheelSpeedEstimator est(opt.wheel);
    std::printf("Wheel: %.3f m diameter x %d pulse(s)/rev = %.4f m per pulse; window %zu pulse(s); "
                "glitch filter rejects intervals under %.1f ms (%.1f m/s); stop timeout %.1f s\n",
                opt.wheel.diameterM, opt.wheel.pulsesPerRev, est.distancePerPulse(), est.window(),
                est.distancePerPulse() / opt.wheel.maxSpeedMps * 1e3, opt.wheel.maxSpeedMps,
                opt.wheel.stopTimeoutS);

    // ---- inputs / outputs ----
    std::unique_ptr<GpioInput> in;
    std::unique_ptr<GpioOutput> gen;
    double expectedSpeed = -1.0;
    if (!sim) {
        GpioLineId id;
        if (!resolveBcmGpio(opt.pulseGpio, opt.gpiochip, id))
            return 1;
        in = std::make_unique<GpioInput>();
        opt.pulseCfg.consumer = "wheel_test";
        if (!in->open(id, opt.pulseCfg))
            return 1;
        const int level = in->readValue();
        std::printf("Pulse input: %s -> %s line %u (%s edge, %s, debounce %s), idle level %d\n",
                    id.name.c_str(), id.chipPath.c_str(), id.offset, gpioEdgeName(opt.pulseCfg.edge),
                    gpioBiasName(opt.pulseCfg.bias),
                    in->debounceApplied() ? (std::to_string(opt.pulseCfg.debounceUs) + " us").c_str() : "off",
                    level);
        if (level == 0 && opt.pulseCfg.bias == GpioInput::Bias::PullUp
            && opt.pulseCfg.edge == GpioInput::Edge::Falling && opt.loopbackGpio < 0) {
            std::printf("Warning: input reads low while idle: magnet parked at the sensor, or the "
                        "wiring/pull-up is wrong.\n");
        }
        if (opt.loopbackGpio >= 0) {
            GpioLineId gid;
            if (!resolveBcmGpio(opt.loopbackGpio, opt.gpiochip, gid))
                return 1;
            gen = std::make_unique<GpioOutput>();
            if (!gen->open(gid, false, "wheel_test"))
                return 1;
            expectedSpeed = opt.loopbackSpeed;
            std::printf("Loopback generator: %s -> %s line %u toggling at %.1f m/s = one pulse every "
                        "%.1f ms. Jumper it to %s.\n",
                        gid.name.c_str(), gid.chipPath.c_str(), gid.offset, expectedSpeed,
                        est.distancePerPulse() / expectedSpeed * 1e3, id.name.c_str());
        }
    } else {
        expectedSpeed = opt.simSpeed;
        std::printf("Simulation: synthetic pulses at %.2f m/s = one every %.1f ms, no hardware.\n",
                    expectedSpeed, est.distancePerPulse() / expectedSpeed * 1e3);
    }
    if (opt.duration > 0)
        std::printf("Running for %.1f s. Ctrl-C stops early.\n", opt.duration);
    else
        std::printf("Running until Ctrl-C.\n");
    std::fflush(stdout);

    // ---- main loop ----
    const uint64_t epoch = monotonicNowNs();
    const uint64_t statusNs = static_cast<uint64_t>(opt.statusInterval * 1e9);
    const uint64_t durationNs = static_cast<uint64_t>(opt.duration * 1e9);
    // generator: one pulse per period; the output toggles every half period
    const uint64_t periodNs = expectedSpeed > 0
        ? static_cast<uint64_t>(est.distancePerPulse() / expectedSpeed * 1e9) : 0;
    const uint64_t genStopNs = static_cast<uint64_t>(opt.genStopAfter * 1e9);
    bool genRunning = periodNs > 0;
    uint64_t nextToggle = epoch + periodNs / 2;
    bool genLevel = true; // idle high like a pulled-up open-collector sensor
    uint64_t nextStatus = epoch + statusNs;
    PulseSink sink{est, epoch, opt.quietPulses};
    std::vector<GpioInput::Event> events;
    double sumSpeed = 0.0;      // status readings taken once the estimate is live
    unsigned long numStatus = 0; // (two accepted pulses), for the accuracy figure
    exitCode = 0;

    while (g_run) {
        const uint64_t now = monotonicNowNs();
        if (durationNs > 0 && now - epoch >= durationNs)
            break;

        // pulse generator (loopback output or simulated pulses)
        if (genRunning && genStopNs > 0 && now - epoch >= genStopNs) {
            genRunning = false;
            if (gen)
                gen->set(true); // park the line at its idle (high) level
            std::printf("Generator stopped at t=%.1f s (wheel stopped): the speed must read 0 within "
                        "the %.1f s stop timeout.\n", (now - epoch) * 1e-9, opt.wheel.stopTimeoutS);
        }
        if (genRunning) {
            while (now >= nextToggle) {
                genLevel = !genLevel;
                if (gen) {
                    gen->set(genLevel);
                } else {
                    const bool counted = (opt.pulseCfg.edge == GpioInput::Edge::Falling) ? !genLevel : genLevel;
                    if (counted)
                        sink.feed(nextToggle, 1, "simulated");
                }
                nextToggle += periodNs / 2;
            }
        }

        // real edge events, timestamped by the kernel
        if (in) {
            events.clear();
            if (in->readEvents(events) < 0) {
                std::fprintf(stderr, "Error: reading the pulse input failed: %s\n", std::strerror(errno));
                exitCode = 1;
                break;
            }
            sink.feedEvents(events);
        }

        if (now >= nextStatus) {
            const double v = est.speedAt(now);
            if (est.acceptedPulses() >= 2) {
                sumSpeed += v;
                numStatus++;
            }
            std::printf("t=%7.1f s  speed %6.2f m/s (%5.1f km/h)  pulses %" PRIu64 " ok / %" PRIu64 " rejected",
                        (now - epoch) * 1e-9, v, v * 3.6, est.acceptedPulses(), est.rejectedPulses());
            if (sink.lastPulseNs > 0)
                std::printf("  last pulse %5.2f s ago", (now > sink.lastPulseNs ? now - sink.lastPulseNs : 0) * 1e-9);
            else
                std::printf("  no pulse yet");
            if (in)
                std::printf("  level %d", in->readValue());
            if (expectedSpeed > 0)
                std::printf("  expected %.2f", expectedSpeed);
            std::printf("\n");
            std::fflush(stdout);
            nextStatus += statusNs;
        }
        usleep(1000);
    }

    // ---- summary ----
    const uint64_t end = monotonicNowNs();
    if (gen)
        gen->set(false);
    std::printf("\nSummary: %.1f s, %" PRIu64 " pulse(s) accepted, %" PRIu64 " rejected, final speed %.2f m/s",
                (end - epoch) * 1e-9, est.acceptedPulses(), est.rejectedPulses(), est.speedAt(end));
    if (numStatus > 0)
        std::printf(", mean of the %lu status reading(s) after the estimate came live %.2f m/s",
                    numStatus, sumSpeed / numStatus);
    std::printf(".\n");
    if (expectedSpeed > 0 && numStatus > 0 && genStopNs == 0) {
        const double mean = sumSpeed / numStatus;
        const double errPct = (mean - expectedSpeed) / expectedSpeed * 100.0;
        std::printf("Expected %.2f m/s: mean reading is %+.1f%% off", expectedSpeed, errPct);
        if (std::fabs(errPct) > 5.0)
            std::printf(" (more than 5%%: check the pulses-per-rev / diameter settings, "
                        "the sensor for double edges, and the rejected count)");
        std::printf(".\n");
    }
    if (est.acceptedPulses() == 0 && !sim)
        std::printf("No pulse was seen: check the wiring, the idle level printed above, and that the "
                    "magnet passes the sensor.\n");
    return exitCode;
}

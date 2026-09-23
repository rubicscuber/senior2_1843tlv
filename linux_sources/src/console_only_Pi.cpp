// console_only_Pi: target detection report for a Raspberry Pi console session,
// with an optional rider safety monitor (wheel-speed input, two-second rule,
// approach alert and GPIO alert LEDs).
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
// Safety monitor (rear-facing radar; enabled by --approach-threshold):
//   self ground speed from a hall-effect wheel sensor on a GPIO input (or a
//   fixed --self-speed), per-target closing speed and two-second-rule check,
//   a steady LED on a gap violation and, on a fast approach, an audio file
//   played out of the Pi's 3.5 mm jack (--alert-sound) and/or a flashing LED.
//   See printUsage() / README for the options. Without --approach-threshold
//   the program behaves exactly as before.
//
// Builds as its own executable; see the Makefile's `console_only_Pi` target.

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <signal.h>
#include <unistd.h>

#include "alert_leds.h"
#include "alert_sound.h"
#include "cfg_parser.h"
#include "frame_parser.h"
#include "gpio_line.h"
#include "mono_clock.h"
#include "pi_selftest.h"
#include "safety_monitor.h"
#include "serial_port.h"
#include "tm_types.h"
#include "wheel_speed.h"

namespace {

std::atomic<bool> g_run{true};

void signalHandler(int)
{
    g_run = false;
}

constexpr uint64_t NS_PER_S = 1000000000ull;
constexpr uint64_t SYNTHETIC_FRAME_NS = 50000000ull; // 50 ms per frame in unpaced file mode
// no wheel pulse for this long while targets are tracked: warn that the
// two-second rule is inactive (self speed reads 0 without pulses)
constexpr uint64_t WHEEL_WARN_NS = 30 * NS_PER_S;

// ---- options ----

struct PiOptions {
    std::string datFile, dataDev, cliDev, cfgFile;
    bool selfTest = false;

    // safety monitor (enabled by --approach-threshold)
    bool monitor = false;
    SafetyParams safety;
    double selfSpeed = -1.0;            // fixed self speed, m/s; < 0 = not set
    int pulseGpio = -1;
    WheelParams wheel;
    bool wheelDiameterSet = false;
    bool pulsesPerRevSet = false;
    GpioInput::Config pulseCfg;
    std::string gpiochip;
    int ledGapGpio = -1;
    int ledSpeedGpio = -1;
    bool ledActiveLow = false;
    bool simGpio = false;
    bool pulseVerbose = false;
    AlertLedParams leds;
    AlertSoundParams sound;   // approach alert audio; enabled by --alert-sound
    bool soundCheck = false;  // play the file once at start
    double playbackFps = 0.0;
};

// running totals for the exit summary
struct Stats {
    unsigned long framesSeen = 0;
    unsigned long framesWithTargets = 0;
    std::set<uint32_t> uniqueTids;
    unsigned long framesGapViolation = 0;
    unsigned long framesApproachAlert = 0;
    double minTimeGap = INFINITY;
    double maxClosingSpeed = -INFINITY;
};

// everything the safety monitor needs at run time
struct SafetyContext {
    SafetyMonitor monitor;
    std::unique_ptr<WheelSpeedEstimator> estimator;
    std::unique_ptr<GpioInput> pulseIn;
    uint32_t lastSeqno = 0;
    double fixedSelfSpeed = -1.0;
    bool pulseVerbose = false;
    AlertLeds leds;
    std::unique_ptr<AlertSound> sound; // approach alert audio; null = none
    uint64_t epochNs;
    uint64_t lastPulseNs;      // last accepted wheel pulse (start time until the first one)
    bool wheelWarned = false;  // "no pulses" warning printed for the current silent stretch

    SafetyContext(const SafetyParams& sp, std::unique_ptr<LedBackend> backend,
                  const AlertLedParams& lp, uint64_t epoch)
        : monitor(sp), leds(std::move(backend), lp), epochNs(epoch), lastPulseNs(epoch)
    {
    }

    // drain queued wheel pulses into the estimator
    void pollPulses(uint64_t now)
    {
        if (!pulseIn || !estimator)
            return;
        std::vector<GpioInput::Event> events;
        if (pulseIn->readEvents(events) < 0) {
            std::fprintf(stderr, "Warning: reading pulse input failed: %s\n", std::strerror(errno));
            return;
        }
        for (const auto& e : events) {
            // the kernel keeps counting line_seqno even when it drops queued
            // events, so a gap in the sequence is a known number of lost pulses
            uint32_t count = 1;
            if (lastSeqno != 0 && e.lineSeqno > lastSeqno)
                count = e.lineSeqno - lastSeqno;
            lastSeqno = e.lineSeqno;
            const bool accepted = estimator->addPulse(e.timestampNs, count);
            if (accepted) {
                lastPulseNs = std::max(lastPulseNs, e.timestampNs);
                if (wheelWarned) {
                    std::printf("Wheel pulses resumed; the two-second rule is active again.\n");
                    wheelWarned = false;
                }
            }
            if (pulseVerbose) {
                const double t = (e.timestampNs >= epochNs) ? (e.timestampNs - epochNs) * 1e-9 : 0.0;
                std::printf("[PULSE] t=%.3f s  %s  x%u  interval %.1f ms  instant %.2f m/s  avg %.2f m/s\n",
                            t, accepted ? "accepted" : "rejected", count,
                            estimator->lastIntervalSeconds() * 1e3, estimator->lastIntervalSpeed(),
                            estimator->speedAt(now));
            }
        }
    }

    double selfSpeed(uint64_t now) const
    {
        if (fixedSelfSpeed >= 0.0)
            return fixedSelfSpeed;
        return estimator ? estimator->speedAt(now) : 0.0;
    }

    // keeps the audio player child in step with the alert (call every loop)
    void pollSound(uint64_t now)
    {
        if (sound)
            sound->poll(now);
    }

    // A silent wheel sensor makes the self speed read 0, which turns every
    // target keeping pace into "not following": the two-second rule is then
    // silently off while the approach alert still works. Say so, once per
    // silent stretch, when targets are being tracked meanwhile.
    void checkWheel(uint64_t now, size_t numTargets)
    {
        if (!estimator || wheelWarned || numTargets == 0)
            return;
        if (now > lastPulseNs && now - lastPulseNs > WHEEL_WARN_NS) {
            std::printf("Warning: no wheel pulse for %.0f s while targets are tracked. If the bike is "
                        "moving, check the wheel sensor: with self speed 0 the two-second rule "
                        "cannot fire (the approach alert still can).\n",
                        (now - lastPulseNs) * 1e-9);
            wheelWarned = true;
        }
    }
};

// ---- reporting ----

void printSelfSpeed(const FrameAssessment* a)
{
    if (a)
        std::printf(" | self %.2f m/s (%.1f km/h)", a->selfSpeed, a->selfSpeed * 3.6);
}

void printAssessment(const TargetAssessment& t)
{
    if (!t.valid) {
        std::printf("           (skipped: invalid position)\n");
        return;
    }
    std::printf("           closing %+6.2f m/s  ground %6.2f m/s  ", t.closingSpeed, t.groundSpeed);
    if (t.isFollower)
        std::printf("gap %5.2f s", t.timeGap);
    else
        std::printf("gap    --  (not following)");
    if (!t.inCorridor)
        std::printf("  (outside corridor)");
    if (t.gapViolation)
        std::printf("  GAP VIOLATION");
    if (t.approachAlert)
        std::printf("  APPROACH ALERT");
    std::printf("\n");
}

// Print one frame's detection state and target details. With `a` == nullptr
// the output is exactly the plain report; with an assessment the self speed
// and a per-target safety line are added.
void reportFrame(const Frame& frame, Stats& stats, const FrameAssessment* a)
{
    stats.framesSeen++;
    const size_t numTargets = frame.haveTargetList ? frame.targets.size() : 0;
    const int numPoints =
        frame.havePointCloud ? static_cast<int>(frame.detObj.numDetectedObj) : 0;

    if (a) {
        if (a->anyGapViolation)
            stats.framesGapViolation++;
        if (a->anyApproachAlert)
            stats.framesApproachAlert++;
        stats.minTimeGap = std::min(stats.minTimeGap, a->minTimeGap);
        stats.maxClosingSpeed = std::max(stats.maxClosingSpeed, a->maxClosingSpeed);
    }

    if (numTargets == 0) {
        std::printf("Frame %-6u | no target detected (%d point-cloud detections)",
                    frame.header.frameNumber, numPoints);
        printSelfSpeed(a);
        std::printf("\n");
        return;
    }

    stats.framesWithTargets++;
    std::printf("Frame %-6u | TARGET DETECTED: %zu target(s), %d point-cloud detections",
                frame.header.frameNumber, numTargets, numPoints);
    printSelfSpeed(a);
    std::printf("\n");

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
        if (a && i < a->targets.size())
            printAssessment(a->targets[i]);
    }
}

void printSummary(const Stats& stats, const SafetyContext* ctx)
{
    std::printf("\nSummary: %lu frame(s) processed, %lu with targets, "
                "%zu unique track id(s).\n",
                stats.framesSeen, stats.framesWithTargets, stats.uniqueTids.size());
    if (!ctx)
        return;
    std::printf("Safety: %lu frame(s) with a two-second-rule violation, %lu with an approach alert; ",
                stats.framesGapViolation, stats.framesApproachAlert);
    if (std::isinf(stats.minTimeGap))
        std::printf("min time gap n/a, ");
    else
        std::printf("min time gap %.2f s, ", stats.minTimeGap);
    if (std::isinf(stats.maxClosingSpeed))
        std::printf("max closing speed n/a.\n");
    else
        std::printf("max closing speed %.2f m/s.\n", stats.maxClosingSpeed);
    if (ctx->estimator) {
        std::printf("Wheel: %" PRIu64 " pulse(s) accepted, %" PRIu64 " rejected as glitches, "
                    "last self speed %.2f m/s.\n",
                    ctx->estimator->acceptedPulses(), ctx->estimator->rejectedPulses(),
                    ctx->selfSpeed(monotonicNowNs()));
    }
    if (ctx->sound)
        std::printf("Sound: the approach alert started the audio %lu time(s).\n", ctx->sound->starts());
}

// ---- safety monitor setup ----

bool openLed(const PiOptions& opt, int bcm, const char* label, std::unique_ptr<GpioOutput>& out)
{
    if (bcm < 0)
        return true;
    GpioLineId id;
    if (!resolveBcmGpio(bcm, opt.gpiochip, id))
        return false;
    out = std::make_unique<GpioOutput>();
    if (!out->open(id, opt.ledActiveLow))
        return false;
    std::printf("%s LED: %s -> %s line %u (%s)\n", label, id.name.c_str(), id.chipPath.c_str(),
                id.offset, opt.ledActiveLow ? "active-low" : "active-high");
    return true;
}

// Builds the safety context (GPIO lines are opened here, before any serial
// port, so permission problems fail fast). ctx stays null when the monitor is
// off. simulateSound: print the audio transitions instead of playing (unpaced
// file replay, where the clock is synthetic).
bool setupSafety(const PiOptions& opt, uint64_t epochNs, bool simulateSound,
                 std::unique_ptr<SafetyContext>& ctx)
{
    if (!opt.monitor)
        return true;

    std::unique_ptr<LedBackend> backend;
    std::string ledMode;
    if (opt.simGpio) {
        backend = std::make_unique<ConsoleLedBackend>(epochNs);
        ledMode = "simulated (printed to console)";
    } else if (opt.ledGapGpio >= 0 || opt.ledSpeedGpio >= 0) {
        std::unique_ptr<GpioOutput> gap, speed;
        if (!openLed(opt, opt.ledGapGpio, "Gap", gap) || !openLed(opt, opt.ledSpeedGpio, "Speed", speed))
            return false;
        backend = std::make_unique<GpioLedBackend>(std::move(gap), std::move(speed));
        ledMode = "GPIO";
    } else {
        backend = std::make_unique<NullLedBackend>();
        ledMode = "none (report only)";
    }

    ctx = std::make_unique<SafetyContext>(opt.safety, std::move(backend), opt.leds, epochNs);
    ctx->pulseVerbose = opt.pulseVerbose;

    // approach alert audio (checked before the serial ports, like the GPIO lines)
    std::string soundMode = "none";
    if (!opt.sound.file.empty()) {
        AlertSoundParams sp = opt.sound;
        sp.simulate = simulateSound;
        ctx->sound = std::make_unique<AlertSound>(sp, epochNs);
        std::string err;
        if (!ctx->sound->check(err)) {
            std::fprintf(stderr, "Error: --alert-sound: %s\n", err.c_str());
            return false;
        }
        if (opt.soundCheck) {
            std::printf("Sound check: playing %s once...\n", sp.file.c_str());
            std::fflush(stdout);
            if (!ctx->sound->playOnce(30.0)) {
                std::fprintf(stderr, "Error: the sound check failed: '%s' could not play %s. "
                                     "Check the audio device (aplay -l) and --sound-player.\n",
                             sp.player.c_str(), sp.file.c_str());
                return false;
            }
        }
        ctx->leds.setApproachListener(ctx->sound.get());
        soundMode = sp.file + (sp.simulate ? " (simulated: printed to console)" : " via '" + sp.player + "'")
                    + (sp.repeat ? ", repeated while active" : ", once per alert");
    }

    std::string speedSource;
    if (opt.selfSpeed >= 0.0) {
        ctx->fixedSelfSpeed = opt.selfSpeed;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "fixed %.2f m/s", opt.selfSpeed);
        speedSource = buf;
    } else {
        ctx->estimator = std::make_unique<WheelSpeedEstimator>(opt.wheel);
        GpioLineId id;
        if (!resolveBcmGpio(opt.pulseGpio, opt.gpiochip, id))
            return false;
        ctx->pulseIn = std::make_unique<GpioInput>();
        if (!ctx->pulseIn->open(id, opt.pulseCfg))
            return false;
        const int level = ctx->pulseIn->readValue();
        std::printf("Pulse input: %s -> %s line %u (%s edge, %s, debounce %s), idle level %d\n",
                    id.name.c_str(), id.chipPath.c_str(), id.offset, gpioEdgeName(opt.pulseCfg.edge),
                    gpioBiasName(opt.pulseCfg.bias),
                    ctx->pulseIn->debounceApplied()
                        ? (std::to_string(opt.pulseCfg.debounceUs) + " us").c_str() : "off",
                    level);
        if (level == 0 && opt.pulseCfg.bias == GpioInput::Bias::PullUp
            && opt.pulseCfg.edge == GpioInput::Edge::Falling) {
            std::printf("Warning: pulse input reads low while idle: the magnet may be parked at the "
                        "sensor, or the wiring/pull-up is wrong.\n");
        }
        char buf[96];
        std::snprintf(buf, sizeof(buf), "wheel %.3f m x %d pulse(s)/rev = %.4f m per pulse",
                      opt.wheel.diameterM, opt.wheel.pulsesPerRev, ctx->estimator->distancePerPulse());
        speedSource = buf;
    }

    const SafetyParams& sp = opt.safety;
    std::printf("Safety monitor: rear-facing radar; two-second rule %.1f s; approach alert above "
                "%.2f m/s; corridor %s; alert hold %.1f s; flash %.1f Hz; LEDs: %s; approach sound: %s; "
                "self speed: %s\n",
                sp.gapSeconds, sp.approachThresholdMps,
                sp.corridorHalfWidthM > 0
                    ? ("+/-" + std::to_string(sp.corridorHalfWidthM).substr(0, 5) + " m").c_str()
                    : "off",
                opt.leds.holdSeconds, opt.leds.flashHz, ledMode.c_str(), soundMode.c_str(),
                speedSource.c_str());
    return true;
}

// The stock TM cfgs only track targets from 10 m out; tell the user what that
// means for the two-second rule.
void printBoundaryBoxNote(const std::vector<std::string>& cfgLines, double gapSeconds)
{
    for (const auto& line : cfgLines) {
        std::istringstream iss(line);
        std::string cmd;
        double minX, maxX, minY;
        if ((iss >> cmd) && cmd == "boundaryBox" && (iss >> minX >> maxX >> minY) && minY > 0.0) {
            std::printf("Note: boundaryBox starts at %.0f m: targets closer than that are not tracked, "
                        "so the two-second rule cannot be assessed below %.1f m/s self speed.\n",
                        minY, minY / gapSeconds);
            return;
        }
    }
}

// Process one valid frame through the monitor (if any), the report and the LEDs.
void handleFrame(const Frame& frame, Stats& stats, SafetyContext* ctx, uint64_t now)
{
    if (!ctx) {
        reportFrame(frame, stats, nullptr);
        return;
    }
    const FrameAssessment a = ctx->monitor.evaluate(frame.targets, ctx->selfSpeed(now));
    reportFrame(frame, stats, &a);
    ctx->leds.onFrame(now, a.anyGapViolation, a.anyApproachAlert);
    ctx->leds.update(now);
    ctx->checkWheel(now, frame.haveTargetList ? frame.targets.size() : 0);
}

// ---- modes ----

int runFile(const PiOptions& opt)
{
    const uint64_t epoch = monotonicNowNs();
    // unpaced: a synthetic 50 ms clock keeps alert timing deterministic (and
    // the audio is only printed); paced (--playback-fps): real time, so LEDs
    // and sound behave as on a live run
    const bool paced = opt.playbackFps > 0.0;
    std::unique_ptr<SafetyContext> ctx;
    if (!setupSafety(opt, epoch, !paced, ctx))
        return 1;

    std::vector<uint8_t> buf;
    if (!readDatFileToBuffer(opt.datFile, buf)) {
        std::fprintf(stderr, "Error: could not read data file %s.\n", opt.datFile.c_str());
        return 1;
    }
    std::printf("Read %zu bytes from %s.\n", buf.size(), opt.datFile.c_str());

    int numFramesAvailable = 0;
    const std::vector<Frame> frames = parseBytesTM(buf, ReadMode::ALL, numFramesAvailable);
    if (frames.empty()) {
        std::fprintf(stderr, "Error: no frames found in %s.\n", opt.datFile.c_str());
        return 1;
    }

    const uint64_t frameNs = paced ? static_cast<uint64_t>(NS_PER_S / opt.playbackFps)
                                   : SYNTHETIC_FRAME_NS;
    uint64_t nextNs = epoch;
    uint64_t now = epoch;

    Stats stats;
    for (size_t i = 0; i < frames.size() && g_run; i++) {
        now = paced ? monotonicNowNs() : epoch + i * frameNs;
        if (frames[i].valid)
            handleFrame(frames[i], stats, ctx.get(), now);
        if (paced) {
            nextNs += frameNs;
            while (g_run && monotonicNowNs() < nextNs) {
                if (ctx) {
                    const uint64_t t = monotonicNowNs();
                    ctx->leds.update(t);
                    ctx->pollSound(t);
                }
                usleep(5000);
            }
            if (ctx)
                std::fflush(stdout);
        }
    }
    if (ctx)
        ctx->leds.allOff(paced ? monotonicNowNs() : now + frameNs);
    printSummary(stats, ctx.get());
    return 0;
}

int runSerial(const PiOptions& opt)
{
    const uint64_t epoch = monotonicNowNs();
    std::unique_ptr<SafetyContext> ctx;
    if (!setupSafety(opt, epoch, false, ctx))
        return 1;

    SerialPort port;
    if (!port.open(opt.dataDev, 921600))
        return 1;

    // optionally send the chirp configuration over the CLI port first,
    // following the same port-status logic as tm_visualizer
    if (!opt.cliDev.empty()) {
        std::vector<std::string> cfgLines;
        if (!readCfgFile(opt.cfgFile, cfgLines)) {
            std::fprintf(stderr, "Error: Could not open CFG file. Quitting.\n");
            return 1;
        }
        // wait longer than one frame period so a streaming device is detected
        if (port.bytesAvailableWithin(500) > 0) {
            std::printf("Device appears to already be running. Will not load a "
                        "new configuration. To load a new config, press NRST on "
                        "the EVM and try again.\n");
        } else {
            SerialPort cliPort;
            if (!cliPort.open(opt.cliDev, 115200))
                return 1;
            if (!loadCfg(cliPort, cfgLines, &g_run))
                return 1;
        }
        if (ctx)
            printBoundaryBoxNote(cfgLines, opt.safety.gapSeconds);
    }

    std::printf("Watching for targets on %s. Press Ctrl-C to stop.\n", opt.dataDev.c_str());
    std::fflush(stdout);

    int exitCode = 0;
    Stats stats;
    std::vector<uint8_t> buf;
    buf.reserve(BYTES_BUFFER_MAX_SIZE);
    uint8_t chunk[4096];
    while (g_run) {
        const uint64_t now = monotonicNowNs();
        if (ctx) {
            ctx->pollPulses(now);
            ctx->leds.update(now); // hold timers, fail-safe and flashing run between frames
            ctx->pollSound(now);   // restart/stop the audio player as the alert changes
        }

        const int n = port.readBytes(chunk, sizeof(chunk));
        if (n < 0) {
            // the port is gone (EVM unplugged or reset): stop rather than idle
            // forever on a dead descriptor, so a supervisor can restart us
            std::fprintf(stderr, "Error: reading %s failed (%s). Device disconnected? Exiting.\n",
                         opt.dataDev.c_str(), std::strerror(errno));
            exitCode = 1;
            break;
        }
        if (n > 0)
            buf.insert(buf.end(), chunk, chunk + n);

        int numFramesAvailable = 0;
        const std::vector<Frame> frames =
            parseBytesTM(buf, ReadMode::FIFO, numFramesAvailable);
        if (!frames.empty()) {
            if (frames.front().valid) {
                handleFrame(frames.front(), stats, ctx.get(), now);
                std::fflush(stdout);
            }
        } else if (n <= 0) {
            usleep(5000); // nothing read and no frame pending: idle briefly
        }
    }
    if (ctx)
        ctx->leds.allOff(monotonicNowNs());
    printSummary(stats, ctx.get());
    return exitCode;
}

// ---- command line ----

void printUsage(const char* prog)
{
    std::printf(
        "TM target detection report (plain console output, Raspberry Pi friendly)\n"
        "\n"
        "Usage:\n"
        "  %s <capture.dat>       report from a recorded stream (hex .dat or binary)\n"
        "  %s -d <device>         report from a live data UART, e.g. -d /dev/ttyACM1\n"
        "\n"
        "Options (live mode):\n"
        "  -c <device>   CLI/config UART, e.g. /dev/ttyACM0; requires -g\n"
        "  -g <file>     chirp cfg file to send to the device before reporting\n"
        "\n"
        "Safety monitor (optional; rear-facing radar, all speeds in m/s):\n"
        "  --approach-threshold <m/s>  enable the monitor: closing speed above which the\n"
        "                              approach alert fires (flashing LED)\n"
        "  --gap-seconds <s>           two-second rule threshold (default 2.0; steady LED)\n"
        "  --corridor <m>              only targets within +/- this lateral distance count\n"
        "                              for the gap rule (default: off)\n"
        "  --self-speed <m/s>          fixed self speed instead of a wheel sensor\n"
        "                              (required in file mode)\n"
        "  --pulse-gpio <bcm>          wheel sensor input pin (live mode); needs\n"
        "  --wheel-diameter <m>        ... the wheel diameter and\n"
        "  --pulses-per-rev <n>        ... the pulses per wheel revolution\n"
        "  --pulse-edge rising|falling  counted edge (default falling)\n"
        "  --pulse-bias pull-up|pull-down|none   input bias (default pull-up)\n"
        "  --pulse-debounce-us <n>     kernel debounce, 0 = off (default 500)\n"
        "  --max-speed <m/s>           glitch filter: faster pulses are ignored (default 40)\n"
        "  --stop-timeout <s>          speed reads 0 after this long without a pulse (default 2)\n"
        "  --led-gap-gpio <bcm>        steady LED output for two-second-rule violations\n"
        "  --alert-sound <file>        approach alert: play this audio file (e.g. WAV) out of\n"
        "                              the audio jack while the alert is active\n"
        "  --sound-player <cmd>        player command, the file is appended (default \"aplay -q\";\n"
        "                              e.g. \"aplay -q -D plughw:Headphones\")\n"
        "  --sound-once                play the file once per alert instead of repeating it\n"
        "  --sound-check               play the file once at start to verify the audio path\n"
        "  --led-speed-gpio <bcm>      flashing LED output for approach alerts (can be used\n"
        "                              together with --alert-sound)\n"
        "  --led-active-low            LEDs light when the pin is driven low\n"
        "  --gpiochip <path>           use this chip with offset = BCM number (default: auto)\n"
        "  --sim-gpio                  print LED transitions instead of driving pins\n"
        "  --alert-hold <s>            keep an alert LED on this long after the last\n"
        "                              triggering frame (default 1.0; 0 = only until\n"
        "                              the next frame)\n"
        "  --flash-hz <n>              approach LED flash rate (default 8)\n"
        "  --frame-timeout <s>         LEDs off when no radar frame arrives for this long\n"
        "                              (default 1.0)\n"
        "  --playback-fps <n>          file mode: replay in real time at this frame rate\n"
        "                              (default: as fast as possible)\n"
        "  --pulse-verbose             print every wheel pulse\n"
        "  --self-test                 run the built-in logic tests and exit\n"
        "\n"
        "Wiring: hall switch output -> pulse GPIO (internal pull-up), GND common;\n"
        "LED anode -> 330 ohm -> LED GPIO, cathode -> GND (active-high).\n"
        "Audio: route the Pi's output to the 3.5 mm jack (raspi-config > Audio, or\n"
        "--sound-player with -D <device> from 'aplay -l'); test with --sound-check.\n",
        prog, prog);
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

// Returns false (after printing the reason) when the program should exit.
// `exitCode` is 0 for --help.
bool parseArgs(int argc, char** argv, PiOptions& opt, int& exitCode)
{
    exitCode = 1;
    std::string monitorFlagSeen; // first monitor-only flag, for the error message

    auto need = [&](int& i) -> const char* {
        if (i + 1 >= argc) {
            std::fprintf(stderr, "Error: %s requires an argument\n", argv[i]);
            return nullptr;
        }
        return argv[++i];
    };
    auto monitorFlag = [&](const std::string& a) {
        if (monitorFlagSeen.empty())
            monitorFlagSeen = a;
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
        } else if (a == "--self-test") {
            opt.selfTest = true;
        } else if (a == "-d") {
            if (!(v = need(i))) return false;
            opt.dataDev = v;
        } else if (a == "-c") {
            if (!(v = need(i))) return false;
            opt.cliDev = v;
        } else if (a == "-g") {
            if (!(v = need(i))) return false;
            opt.cfgFile = v;
        } else if (a == "--approach-threshold") {
            if (!(v = need(i)) || !parseDouble("--approach-threshold", v, d)) return false;
            if (d < 0) { std::fprintf(stderr, "Error: --approach-threshold must be >= 0\n"); return false; }
            opt.safety.approachThresholdMps = d;
            opt.monitor = true;
        } else if (a == "--gap-seconds") {
            if (!(v = need(i)) || !parseDouble("--gap-seconds", v, d)) return false;
            if (!(d > 0)) { std::fprintf(stderr, "Error: --gap-seconds must be > 0\n"); return false; }
            opt.safety.gapSeconds = d;
            monitorFlag(a);
        } else if (a == "--corridor") {
            if (!(v = need(i)) || !parseDouble("--corridor", v, d)) return false;
            if (d < 0) { std::fprintf(stderr, "Error: --corridor must be >= 0\n"); return false; }
            opt.safety.corridorHalfWidthM = d;
            monitorFlag(a);
        } else if (a == "--self-speed") {
            if (!(v = need(i)) || !parseDouble("--self-speed", v, d)) return false;
            if (d < 0) { std::fprintf(stderr, "Error: --self-speed must be >= 0\n"); return false; }
            opt.selfSpeed = d;
            monitorFlag(a);
        } else if (a == "--pulse-gpio") {
            if (!(v = need(i)) || !parseInt("--pulse-gpio", v, n)) return false;
            opt.pulseGpio = n;
            monitorFlag(a);
        } else if (a == "--wheel-diameter") {
            if (!(v = need(i)) || !parseDouble("--wheel-diameter", v, d)) return false;
            opt.wheel.diameterM = d;
            opt.wheelDiameterSet = true;
            monitorFlag(a);
        } else if (a == "--pulses-per-rev") {
            if (!(v = need(i)) || !parseInt("--pulses-per-rev", v, n)) return false;
            opt.wheel.pulsesPerRev = n;
            opt.pulsesPerRevSet = true;
            monitorFlag(a);
        } else if (a == "--pulse-edge") {
            if (!(v = need(i))) return false;
            const std::string s = v;
            if (s == "rising") opt.pulseCfg.edge = GpioInput::Edge::Rising;
            else if (s == "falling") opt.pulseCfg.edge = GpioInput::Edge::Falling;
            else { std::fprintf(stderr, "Error: --pulse-edge expects rising or falling\n"); return false; }
            monitorFlag(a);
        } else if (a == "--pulse-bias") {
            if (!(v = need(i))) return false;
            const std::string s = v;
            if (s == "pull-up") opt.pulseCfg.bias = GpioInput::Bias::PullUp;
            else if (s == "pull-down") opt.pulseCfg.bias = GpioInput::Bias::PullDown;
            else if (s == "none") opt.pulseCfg.bias = GpioInput::Bias::None;
            else { std::fprintf(stderr, "Error: --pulse-bias expects pull-up, pull-down or none\n"); return false; }
            monitorFlag(a);
        } else if (a == "--pulse-debounce-us") {
            if (!(v = need(i)) || !parseInt("--pulse-debounce-us", v, n)) return false;
            if (n < 0) { std::fprintf(stderr, "Error: --pulse-debounce-us must be >= 0\n"); return false; }
            opt.pulseCfg.debounceUs = static_cast<uint32_t>(n);
            monitorFlag(a);
        } else if (a == "--max-speed") {
            if (!(v = need(i)) || !parseDouble("--max-speed", v, d)) return false;
            opt.wheel.maxSpeedMps = d;
            monitorFlag(a);
        } else if (a == "--stop-timeout") {
            if (!(v = need(i)) || !parseDouble("--stop-timeout", v, d)) return false;
            opt.wheel.stopTimeoutS = d;
            monitorFlag(a);
        } else if (a == "--led-gap-gpio") {
            if (!(v = need(i)) || !parseInt("--led-gap-gpio", v, n)) return false;
            opt.ledGapGpio = n;
            monitorFlag(a);
        } else if (a == "--led-speed-gpio") {
            if (!(v = need(i)) || !parseInt("--led-speed-gpio", v, n)) return false;
            opt.ledSpeedGpio = n;
            monitorFlag(a);
        } else if (a == "--alert-sound") {
            if (!(v = need(i))) return false;
            opt.sound.file = v;
            monitorFlag(a);
        } else if (a == "--sound-player") {
            if (!(v = need(i))) return false;
            opt.sound.player = v;
            monitorFlag(a);
        } else if (a == "--sound-once") {
            opt.sound.repeat = false;
            monitorFlag(a);
        } else if (a == "--sound-check") {
            opt.soundCheck = true;
            monitorFlag(a);
        } else if (a == "--led-active-low") {
            opt.ledActiveLow = true;
            monitorFlag(a);
        } else if (a == "--gpiochip") {
            if (!(v = need(i))) return false;
            opt.gpiochip = v;
            monitorFlag(a);
        } else if (a == "--sim-gpio") {
            opt.simGpio = true;
            monitorFlag(a);
        } else if (a == "--alert-hold") {
            if (!(v = need(i)) || !parseDouble("--alert-hold", v, d)) return false;
            if (d < 0) { std::fprintf(stderr, "Error: --alert-hold must be >= 0\n"); return false; }
            opt.leds.holdSeconds = d;
            monitorFlag(a);
        } else if (a == "--flash-hz") {
            if (!(v = need(i)) || !parseDouble("--flash-hz", v, d)) return false;
            if (!(d > 0) || d > 50) { std::fprintf(stderr, "Error: --flash-hz must be between 0 and 50\n"); return false; }
            opt.leds.flashHz = d;
            monitorFlag(a);
        } else if (a == "--frame-timeout") {
            if (!(v = need(i)) || !parseDouble("--frame-timeout", v, d)) return false;
            if (!(d > 0)) { std::fprintf(stderr, "Error: --frame-timeout must be > 0\n"); return false; }
            opt.leds.frameTimeoutSeconds = d;
            monitorFlag(a);
        } else if (a == "--playback-fps") {
            if (!(v = need(i)) || !parseDouble("--playback-fps", v, d)) return false;
            if (d < 0 || d > 1000) { std::fprintf(stderr, "Error: --playback-fps must be between 0 and 1000\n"); return false; }
            opt.playbackFps = d;
        } else if (a == "--pulse-verbose") {
            opt.pulseVerbose = true;
            monitorFlag(a);
        } else if (!a.empty() && a[0] != '-' && opt.datFile.empty()) {
            opt.datFile = a;
        } else {
            std::fprintf(stderr, "Error: unknown option '%s'\n", a.c_str());
            printUsage(argv[0]);
            return false;
        }
    }

    if (opt.selfTest)
        return true;

    // ---- mode selection (unchanged behaviour) ----
    const bool live = !opt.dataDev.empty();
    if (!live && opt.datFile.empty()) {
        printUsage(argv[0]);
        return false;
    }
    if (live && (opt.cliDev.empty() != opt.cfgFile.empty())) {
        std::fprintf(stderr,
                     "Error: -c and -g must be used together (a CLI port to "
                     "send on and a cfg file to send).\n");
        return false;
    }
    if (!live && (!opt.cliDev.empty() || !opt.cfgFile.empty()))
        std::printf("Note: -c/-g only apply to live mode (-d); ignoring.\n");

    // ---- safety monitor validation ----
    if (!opt.monitor) {
        if (!monitorFlagSeen.empty()) {
            std::fprintf(stderr, "Error: %s requires --approach-threshold <m/s> (it enables the "
                                 "safety monitor).\n", monitorFlagSeen.c_str());
            return false;
        }
        // --playback-fps paces the file replay with or without the monitor
        return true;
    }

    const bool haveFixed = opt.selfSpeed >= 0.0;
    const bool havePulse = opt.pulseGpio >= 0;
    if (haveFixed && havePulse) {
        std::fprintf(stderr, "Error: use either --self-speed or --pulse-gpio, not both.\n");
        return false;
    }
    if (!haveFixed && !havePulse) {
        std::fprintf(stderr, live
            ? "Error: the safety monitor needs a self speed: --pulse-gpio <bcm> (with "
              "--wheel-diameter and --pulses-per-rev) or --self-speed <m/s>.\n"
            : "Error: file mode needs --self-speed <m/s> for the safety monitor.\n");
        return false;
    }
    if (havePulse) {
        if (!live) {
            std::fprintf(stderr, "Error: --pulse-gpio needs a live data port (-d); use --self-speed "
                                 "for recorded files.\n");
            return false;
        }
        if (!opt.wheelDiameterSet || !opt.pulsesPerRevSet) {
            std::fprintf(stderr, "Error: --pulse-gpio requires --wheel-diameter <m> and "
                                 "--pulses-per-rev <n>.\n");
            return false;
        }
        std::string err;
        if (!opt.wheel.validate(err)) {
            std::fprintf(stderr, "Error: %s\n", err.c_str());
            return false;
        }
    } else if (opt.wheelDiameterSet || opt.pulsesPerRevSet || opt.pulseVerbose) {
        std::printf("Note: wheel options have no effect with --self-speed.\n");
    }
    if ((opt.pulseGpio >= 0 && (opt.pulseGpio == opt.ledGapGpio || opt.pulseGpio == opt.ledSpeedGpio))
        || (opt.ledGapGpio >= 0 && opt.ledGapGpio == opt.ledSpeedGpio)) {
        std::fprintf(stderr, "Error: --pulse-gpio, --led-gap-gpio and --led-speed-gpio must be "
                             "different pins.\n");
        return false;
    }
    if (opt.ledActiveLow && opt.ledGapGpio < 0 && opt.ledSpeedGpio < 0)
        std::printf("Note: --led-active-low has no effect without LED pins.\n");
    if (opt.sound.file.empty() && (opt.soundCheck || !opt.sound.repeat || opt.sound.player != "aplay -q"))
        std::printf("Note: --sound-player/--sound-once/--sound-check have no effect without --alert-sound.\n");
    if (!opt.sound.file.empty() && opt.sound.player.find_first_not_of(" \t") == std::string::npos) {
        std::fprintf(stderr, "Error: --sound-player must name a command.\n");
        return false;
    }
    if (opt.simGpio && (opt.ledGapGpio >= 0 || opt.ledSpeedGpio >= 0))
        std::printf("Note: --sim-gpio prints LED transitions instead of driving the LED pins.\n");
    if (live && opt.playbackFps > 0)
        std::printf("Note: --playback-fps only applies to file mode; ignoring.\n");
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    PiOptions opt;
    int exitCode = 1;
    if (!parseArgs(argc, argv, opt, exitCode))
        return exitCode;

    if (opt.selfTest) {
        const int failures = runPiSelfTests();
        return failures > 255 ? 255 : failures;
    }
    if (!opt.dataDev.empty())
        return runSerial(opt);
    return runFile(opt);
}

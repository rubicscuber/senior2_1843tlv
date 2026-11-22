// led_sound_test: bench test for the prototype's alert outputs (the single
// red alert LED and the speaker on the 3.5 mm jack), no radar needed.
//
// It drives the same code console_only_Pi uses (AlertLeds in single-LED mode,
// GpioLedBackend, AlertSound), so a pass here means the real alerts will
// light and sound the same way. Each round runs:
//   1. LED steady              = what a two-second-rule violation looks like
//   2. speaker only            = the alert file played once (sound check)
//   3. LED flashing + speaker  = what an approach alert looks/sounds like
//   4. everything off
//
// Usage:
//   led_sound_test                          LED on GPIO22, sounds/alert.wav via the jack
//   led_sound_test --led-gpio 27 --loop     another pin, repeat until Ctrl-C
//   led_sound_test --no-sound               LED only
//   led_sound_test --no-led                 speaker only

#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <unistd.h>

#include "alert_leds.h"
#include "alert_sound.h"
#include "gpio_line.h"
#include "mono_clock.h"

namespace {

volatile sig_atomic_t g_run = 1;

void signalHandler(int)
{
    g_run = 0;
}

struct Options {
    int ledGpio = 22;
    std::string gpiochip;
    bool ledActiveLow = false;
    bool useLed = true;
    std::string soundFile = "sounds/alert.wav";
    std::string player = "aplay -q -D plughw:Headphones";
    bool useSound = true;
    double stepSeconds = 3.0;
    double flashHz = 8.0;
    bool loop = false;
};

void printUsage(const char* prog)
{
    std::printf("Alert LED + speaker test (no radar needed)\n"
                "\n"
                "Usage: %s [options]\n"
                "\n"
                "Options:\n"
                "  --led-gpio <bcm>      alert LED pin (default 22 = header pin 15)\n"
                "  --led-active-low      LED lights when the pin is driven low\n"
                "  --gpiochip <path>     use this chip with offset = BCM number (default: auto)\n"
                "  --no-led              skip the LED steps\n"
                "  --sound <file>        audio file (default sounds/alert.wav)\n"
                "  --player <cmd>        player command, file appended\n"
                "                        (default \"aplay -q -D plughw:Headphones\" = 3.5 mm jack)\n"
                "  --no-sound            skip the speaker steps\n"
                "  --step <s>            length of each LED step (default 3)\n"
                "  --flash-hz <n>        flash rate (default 8, as console_only_Pi)\n"
                "  --loop                repeat the sequence until Ctrl-C\n"
                "  -h, --help            show this help\n"
                "\n"
                "Wiring: GPIO -> 330 ohm -> LED anode, cathode -> GND. Speaker/amp on the 3.5 mm jack.\n",
                prog);
}

bool parseNumber(const char* flag, const char* text, double& out)
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

bool parseArgs(int argc, char** argv, Options& opt)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: %s requires an argument\n", a.c_str());
                return nullptr;
            }
            return argv[++i];
        };
        const char* v = nullptr;
        double d = 0.0;
        if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (a == "--led-gpio") {
            if (!(v = need()) || !parseNumber(a.c_str(), v, d)) return false;
            if (d < 0 || d != std::floor(d)) {
                std::fprintf(stderr, "Error: --led-gpio expects a GPIO number\n");
                return false;
            }
            opt.ledGpio = static_cast<int>(d);
        } else if (a == "--led-active-low") {
            opt.ledActiveLow = true;
        } else if (a == "--gpiochip") {
            if (!(v = need())) return false;
            opt.gpiochip = v;
        } else if (a == "--no-led") {
            opt.useLed = false;
        } else if (a == "--sound") {
            if (!(v = need())) return false;
            opt.soundFile = v;
        } else if (a == "--player") {
            if (!(v = need())) return false;
            opt.player = v;
        } else if (a == "--no-sound") {
            opt.useSound = false;
        } else if (a == "--step") {
            if (!(v = need()) || !parseNumber(a.c_str(), v, d)) return false;
            if (d <= 0) {
                std::fprintf(stderr, "Error: --step must be > 0\n");
                return false;
            }
            opt.stepSeconds = d;
        } else if (a == "--flash-hz") {
            if (!(v = need()) || !parseNumber(a.c_str(), v, d)) return false;
            opt.flashHz = d;
        } else if (a == "--loop") {
            opt.loop = true;
        } else {
            std::fprintf(stderr, "Error: unknown option '%s'\n\n", a.c_str());
            printUsage(argv[0]);
            return false;
        }
    }
    if (!opt.useLed && !opt.useSound) {
        std::fprintf(stderr, "Error: --no-led and --no-sound together leave nothing to test.\n");
        return false;
    }
    return true;
}

// Feed the LED state machine synthetic "radar frames" carrying the given alert
// flags (20 Hz, like the radar) for `seconds`, keeping the flashing and the
// sound player running in between.
void runPhase(AlertLeds& leds, AlertSound* sound, bool gap, bool approach, double seconds)
{
    constexpr uint64_t FRAME_NS = 50000000ull; // 50 ms
    const uint64_t start = monotonicNowNs();
    const uint64_t end = start + static_cast<uint64_t>(seconds * 1e9);
    uint64_t nextFrame = start;
    while (g_run) {
        const uint64_t now = monotonicNowNs();
        if (now >= end)
            break;
        if (now >= nextFrame) {
            leds.onFrame(now, gap, approach);
            nextFrame += FRAME_NS;
        }
        leds.update(now);
        if (sound)
            sound->poll(now);
        usleep(5000);
    }
}

} // namespace

int main(int argc, char** argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt))
        return 1;
    setvbuf(stdout, nullptr, _IOLBF, 0);

    struct sigaction sa{};
    sa.sa_handler = signalHandler;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // ---- LED ----
    std::unique_ptr<LedBackend> backend;
    if (opt.useLed) {
        GpioLineId id;
        if (!resolveBcmGpio(opt.ledGpio, opt.gpiochip, id))
            return 1;
        auto out = std::make_unique<GpioOutput>();
        if (!out->open(id, opt.ledActiveLow, "led_sound_test"))
            return 1;
        std::printf("LED: %s -> %s line %u (%s)\n", id.name.c_str(), id.chipPath.c_str(), id.offset,
                    opt.ledActiveLow ? "active-low" : "active-high");
        backend = std::make_unique<GpioLedBackend>(std::move(out), nullptr);
    } else {
        backend = std::make_unique<NullLedBackend>();
        std::printf("LED: skipped (--no-led)\n");
    }

    AlertLedParams lp;
    lp.singleLed = true;      // the prototype's one red LED
    lp.holdSeconds = 0.0;     // switch cleanly between the test steps
    lp.flashHz = opt.flashHz;
    lp.frameTimeoutSeconds = 1.0;
    AlertLeds leds(std::move(backend), lp);

    // ---- speaker ----
    std::unique_ptr<AlertSound> sound;
    if (opt.useSound) {
        AlertSoundParams sp;
        sp.file = opt.soundFile;
        sp.player = opt.player;
        sp.repeat = true;
        sound = std::make_unique<AlertSound>(sp, monotonicNowNs());
        std::string err;
        if (!sound->check(err)) {
            std::fprintf(stderr, "Error: sound: %s\n(use --sound/--player, or --no-sound to test the LED only)\n",
                         err.c_str());
            return 1;
        }
        std::printf("Speaker: %s via '%s'\n", opt.soundFile.c_str(), opt.player.c_str());
        leds.setApproachListener(sound.get()); // the approach alert starts/stops the sound
    } else {
        std::printf("Speaker: skipped (--no-sound)\n");
    }
    std::printf("Ctrl-C stops at any time.\n");

    bool soundFailed = false;
    int round = 0;
    do {
        ++round;
        std::printf("\n==== round %d ====\n", round);

        if (opt.useLed && g_run) {
            std::printf("[1] LED STEADY ON for %.0f s  (two-second rule broken)\n", opt.stepSeconds);
            runPhase(leds, sound.get(), true, false, opt.stepSeconds);
            runPhase(leds, sound.get(), false, false, 1.0);
        }

        if (opt.useSound && g_run) {
            std::printf("[2] SPEAKER: playing %s once...\n", opt.soundFile.c_str());
            if (sound->playOnce(30.0)) {
                std::printf("    played OK (did you hear it?)\n");
            } else {
                soundFailed = true;
                std::printf("    FAILED: the player could not play the file. Check 'aplay -l' lists\n"
                            "    \"bcm2835 Headphones\" and try: aplay -D plughw:Headphones %s\n",
                            opt.soundFile.c_str());
            }
            runPhase(leds, sound.get(), false, false, 1.0);
        }

        if (g_run) {
            std::printf("[3] APPROACH ALERT for %.0f s: %s%s%s\n", opt.stepSeconds,
                        opt.useLed ? "LED FLASHING" : "", opt.useLed && opt.useSound ? " + " : "",
                        opt.useSound ? "SPEAKER" : "");
            runPhase(leds, sound.get(), false, true, opt.stepSeconds);
        }

        if (g_run) {
            std::printf("[4] all OFF\n");
            runPhase(leds, sound.get(), false, false, 1.5);
        }
    } while (opt.loop && g_run);

    const uint64_t now = monotonicNowNs();
    leds.allOff(now);
    if (sound)
        sound->stop(now);

    std::printf("\nDone (%d round(s)%s). Expected: steady LED, one sound, flashing LED with sound, off.\n",
                round, g_run ? "" : ", stopped by Ctrl-C");
    if (soundFailed)
        std::printf("The speaker step FAILED; see the message above.\n");
    return soundFailed ? 2 : 0;
}

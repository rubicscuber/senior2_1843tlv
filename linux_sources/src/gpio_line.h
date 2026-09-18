// Minimal RAII wrappers over the Linux GPIO character device uAPI v2
// (/dev/gpiochipN, <linux/gpio.h>, kernel >= 5.10). No library dependency:
// works on Debian 13 / Raspberry Pi OS with the kernel's built-in interface.
//
// GpioInput requests a line with edge detection; the kernel timestamps every
// edge (CLOCK_MONOTONIC) and queues it, so pulses are not lost between polls.
// GpioOutput requests a line as output and turns it off on release.
#ifndef GPIO_LINE_H
#define GPIO_LINE_H

#include <cstdint>
#include <string>
#include <vector>

struct GpioLineId {
    std::string chipPath; // e.g. "/dev/gpiochip0"
    uint32_t offset = 0;  // line offset on that chip
    std::string name;     // e.g. "GPIO17"
};

// Resolve a Raspberry Pi BCM GPIO number to a chip + line offset.
// chipOverride non-empty: use that chip with offset == bcm (no lookup).
// Otherwise scan /dev/gpiochip* for a line named "GPIO<bcm>" (the name used by
// both the Raspberry Pi and mainline device trees for the header pins) and fall
// back to /dev/gpiochip0 offset == bcm with a note. Prints a clear message and
// returns false on permission or missing-device errors.
bool resolveBcmGpio(int bcm, const std::string& chipOverride, GpioLineId& out);

class GpioInput {
public:
    enum class Edge { Rising, Falling };
    enum class Bias { PullUp, PullDown, None };

    struct Config {
        Edge edge = Edge::Falling;        // open-collector hall switch pulls low when the magnet passes
        Bias bias = Bias::PullUp;
        uint32_t debounceUs = 500;        // kernel debounce; 0 = none (falls back automatically if refused)
        uint32_t eventBufferSize = 512;   // kernel event queue depth for this line
        std::string consumer = "console_only_Pi";
    };

    struct Event {
        uint64_t timestampNs; // CLOCK_MONOTONIC
        uint32_t lineSeqno;   // kernel sequence number for this line (gaps = dropped events)
        bool rising;
    };

    GpioInput() = default;
    ~GpioInput();
    GpioInput(const GpioInput&) = delete;
    GpioInput& operator=(const GpioInput&) = delete;

    bool open(const GpioLineId& id, const Config& cfg);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // Non-blocking: appends all queued edge events to `out`, returns the number
    // appended, or -1 on error.
    int readEvents(std::vector<Event>& out);

    // Current logical level (0/1), -1 on error. Diagnostic only.
    int readValue() const;

    bool debounceApplied() const { return debounce_; }

private:
    int fd_ = -1;
    bool debounce_ = false;
    bool usePoll_ = false;
};

class GpioOutput {
public:
    GpioOutput() = default;
    ~GpioOutput(); // turns the line off and releases it
    GpioOutput(const GpioOutput&) = delete;
    GpioOutput& operator=(const GpioOutput&) = delete;

    // activeLow: the LED is lit when the pin is driven low. The line starts off.
    bool open(const GpioLineId& id, bool activeLow, const std::string& consumer = "console_only_Pi");
    bool set(bool on); // logical: true = LED lit, in either wiring
    void close();
    bool isOpen() const { return fd_ >= 0; }

private:
    int fd_ = -1;
};

const char* gpioEdgeName(GpioInput::Edge edge);
const char* gpioBiasName(GpioInput::Bias bias);

#endif // GPIO_LINE_H

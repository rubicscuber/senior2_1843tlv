// Alert LED state machine: a steady LED for two-second-rule violations and a
// rapidly flashing LED for approach alerts, with hold timers so a track that
// drops out for a frame does not flicker the LEDs, and a fail-safe that turns
// everything off when the radar stops delivering frames.
//
// The physical output is abstracted behind LedBackend so the same logic drives
// real GPIO lines, prints transitions to the console (simulation), or nothing.
#ifndef ALERT_LEDS_H
#define ALERT_LEDS_H

#include <cstdint>
#include <memory>

class GpioOutput;

class LedBackend {
public:
    virtual ~LedBackend() = default;
    virtual void setGapLed(bool on, uint64_t nowNs) = 0;
    virtual void setSpeedLed(bool on, uint64_t nowNs) = 0;
};

// report-only: no outputs at all
class NullLedBackend : public LedBackend {
public:
    void setGapLed(bool, uint64_t) override {}
    void setSpeedLed(bool, uint64_t) override {}
};

// prints "[LED] gap ON   t=12.345 s" for every transition (--sim-gpio)
class ConsoleLedBackend : public LedBackend {
public:
    explicit ConsoleLedBackend(uint64_t epochNs) : epochNs_(epochNs) {}
    void setGapLed(bool on, uint64_t nowNs) override;
    void setSpeedLed(bool on, uint64_t nowNs) override;

private:
    void print(const char* which, bool on, uint64_t nowNs) const;
    uint64_t epochNs_;
};

// Receives the approach alert's active/inactive transitions (after hold time
// and fail-safe are applied), for outputs other than the LED, e.g. audio.
class ApproachAlertListener {
public:
    virtual ~ApproachAlertListener() = default;
    virtual void onApproachAlert(bool active, uint64_t nowNs) = 0;
};

// drives real GPIO output lines; a missing line is a no-op
class GpioLedBackend : public LedBackend {
public:
    GpioLedBackend(std::unique_ptr<GpioOutput> gap, std::unique_ptr<GpioOutput> speed);
    ~GpioLedBackend() override;
    void setGapLed(bool on, uint64_t) override;
    void setSpeedLed(bool on, uint64_t) override;

private:
    std::unique_ptr<GpioOutput> gap_;
    std::unique_ptr<GpioOutput> speed_;
};

struct AlertLedParams {
    double holdSeconds = 1.0;         // keep an alert LED active this long after the last triggering frame
                                      // (an alert is always kept at least until the next frame; 0 = only that)
    double flashHz = 8.0;             // speed LED flash rate (<= 0: steady)
    double frameTimeoutSeconds = 1.0; // no valid radar frame for this long -> all LEDs off
};

class AlertLeds {
public:
    AlertLeds(std::unique_ptr<LedBackend> backend, const AlertLedParams& p);

    // Call for every VALID radar frame (including frames without targets):
    // it feeds the alert flags and keeps the fail-safe alive.
    void onFrame(uint64_t nowNs, bool gapViolation, bool approachAlert);

    // Call on every main-loop iteration so hold timers, the fail-safe and the
    // flashing keep running between frames. Touches the backend only on change.
    void update(uint64_t nowNs);

    // Force both LEDs off (exit path) and clear all pending alerts.
    void allOff(uint64_t nowNs);

    // Optional extra output notified when the approach alert starts/ends
    // (the caller keeps ownership; nullptr = none).
    void setApproachListener(ApproachAlertListener* l) { listener_ = l; }

    bool gapActive() const { return gapActive_; }
    bool approachActive() const { return approachActive_; }
    bool gapLedOn() const { return gapLed_; }
    bool speedLedOn() const { return speedLed_; }

private:
    void apply(uint64_t nowNs, bool gapOn, bool speedOn);

    std::unique_ptr<LedBackend> backend_;
    ApproachAlertListener* listener_ = nullptr;
    uint64_t holdNs_;
    uint64_t halfPeriodNs_;
    uint64_t frameTimeoutNs_;
    uint64_t gapUntil_ = 0;
    uint64_t approachUntil_ = 0;
    uint64_t lastFrame_ = 0;
    uint64_t flashEpoch_ = 0;
    bool haveFrame_ = false;
    bool gapInFrame_ = false;      // flags of the latest frame, valid until the next one
    bool approachInFrame_ = false;
    bool gapActive_ = false;
    bool approachActive_ = false;
    bool gapLed_ = false;
    bool speedLed_ = false;
};

#endif // ALERT_LEDS_H

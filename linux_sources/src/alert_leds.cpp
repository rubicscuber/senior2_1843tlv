#include "alert_leds.h"

#include <algorithm>
#include <cstdio>

#include "gpio_line.h"

// ---- ConsoleLedBackend ----

void ConsoleLedBackend::setGapLed(bool on, uint64_t nowNs)
{
    print("gap", on, nowNs);
}

void ConsoleLedBackend::setSpeedLed(bool on, uint64_t nowNs)
{
    print("speed", on, nowNs);
}

void ConsoleLedBackend::print(const char* which, bool on, uint64_t nowNs) const
{
    const double t = (nowNs >= epochNs_) ? (nowNs - epochNs_) * 1e-9 : 0.0;
    std::printf("[LED] %-5s %-3s  t=%.3f s\n", which, on ? "ON" : "OFF", t);
}

// ---- GpioLedBackend ----

GpioLedBackend::GpioLedBackend(std::unique_ptr<GpioOutput> gap, std::unique_ptr<GpioOutput> speed)
    : gap_(std::move(gap)), speed_(std::move(speed))
{
}

GpioLedBackend::~GpioLedBackend() = default; // GpioOutput destructors turn the lines off

void GpioLedBackend::setGapLed(bool on, uint64_t)
{
    if (gap_)
        gap_->set(on);
}

void GpioLedBackend::setSpeedLed(bool on, uint64_t)
{
    if (speed_)
        speed_->set(on);
}

// ---- AlertLeds ----

AlertLeds::AlertLeds(std::unique_ptr<LedBackend> backend, const AlertLedParams& p)
    : backend_(std::move(backend)),
      holdNs_(static_cast<uint64_t>(std::max(0.0, p.holdSeconds) * 1e9)),
      halfPeriodNs_(p.flashHz > 0.0 ? static_cast<uint64_t>(0.5e9 / p.flashHz) : 0),
      frameTimeoutNs_(static_cast<uint64_t>(std::max(0.0, p.frameTimeoutSeconds) * 1e9))
{
}

void AlertLeds::onFrame(uint64_t nowNs, bool gapViolation, bool approachAlert)
{
    lastFrame_ = nowNs;
    haveFrame_ = true;
    // the latest frame's own flags keep an alert active until the next frame
    // replaces them, whatever the hold time (so a hold shorter than the frame
    // period, including 0, cannot blank or flicker the LEDs)
    gapInFrame_ = gapViolation;
    approachInFrame_ = approachAlert;
    if (gapViolation)
        gapUntil_ = std::max(gapUntil_, nowNs + holdNs_);
    if (approachAlert)
        approachUntil_ = std::max(approachUntil_, nowNs + holdNs_);
}

void AlertLeds::update(uint64_t nowNs)
{
    // fail-safe: without recent radar frames no alert can be trusted
    const bool alive = haveFrame_ && nowNs >= lastFrame_ && (nowNs - lastFrame_) <= frameTimeoutNs_;
    const bool gap = alive && (gapInFrame_ || nowNs < gapUntil_);
    const bool approach = alive && (approachInFrame_ || nowNs < approachUntil_);

    if (approach && !approachActive_)
        flashEpoch_ = nowNs; // flashing always starts with the LED on
    gapActive_ = gap;
    approachActive_ = approach;

    bool speedOn = false;
    if (approach) {
        if (halfPeriodNs_ == 0)
            speedOn = true;
        else
            speedOn = (((nowNs - flashEpoch_) / halfPeriodNs_) % 2) == 0;
    }
    apply(nowNs, gap, speedOn);
}

void AlertLeds::allOff(uint64_t nowNs)
{
    gapUntil_ = 0;
    approachUntil_ = 0;
    gapInFrame_ = false;
    approachInFrame_ = false;
    gapActive_ = false;
    approachActive_ = false;
    apply(nowNs, false, false);
}

void AlertLeds::apply(uint64_t nowNs, bool gapOn, bool speedOn)
{
    if (gapOn != gapLed_) {
        gapLed_ = gapOn;
        backend_->setGapLed(gapOn, nowNs);
    }
    if (speedOn != speedLed_) {
        speedLed_ = speedOn;
        backend_->setSpeedLed(speedOn, nowNs);
    }
}

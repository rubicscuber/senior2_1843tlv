// Self ground-speed estimation from a wheel pulse train (hall-effect switch).
// Pure logic: takes pulse timestamps in nanoseconds (CLOCK_MONOTONIC, as
// delivered by the GPIO character device) and has no hardware dependency, so
// it is unit-testable and reusable.
#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

struct WheelParams {
    double diameterM = 0.0;     // wheel diameter, m (required > 0)
    int    pulsesPerRev = 0;    // pulses per wheel revolution (required >= 1)
    double maxSpeedMps = 40.0;  // glitch filter: pulses closer than distPerPulse/maxSpeed are rejected
    double stopTimeoutS = 2.0;  // no pulse for this long -> speed reported as 0
    int    window = 0;          // pulses averaged; 0 = auto = clamp(pulsesPerRev, 2, 32)

    bool validate(std::string& err) const;
};

class WheelSpeedEstimator {
public:
    explicit WheelSpeedEstimator(const WheelParams& p);

    // Register a pulse edge at time tNs. `count` is the number of pulses this
    // event represents (> 1 when the caller detected dropped events through
    // the kernel's line sequence number). Returns false if rejected as a glitch.
    bool addPulse(uint64_t tNs, uint32_t count = 1);

    // Speed estimate at time nowNs (m/s). Averages the pulse periods in the
    // window; between pulses the estimate is capped by the time elapsed since
    // the last pulse, so a stopping wheel decays to 0 instead of freezing.
    double speedAt(uint64_t nowNs) const;

    double distancePerPulse() const { return distPerPulse_; }
    size_t window() const { return window_; }
    uint64_t acceptedPulses() const { return accepted_; }
    uint64_t rejectedPulses() const { return rejected_; }
    double lastIntervalSeconds() const { return lastIntervalNs_ * 1e-9; }
    double lastIntervalSpeed() const { return lastIntervalSpeed_; } // from the last accepted interval alone

private:
    struct Sample {
        uint64_t tNs;
        uint32_t count;
    };
    std::deque<Sample> samples_; // at most window_ + 1 entries
    size_t   window_;
    double   distPerPulse_;
    uint64_t minIntervalNs_;
    uint64_t stopTimeoutNs_;
    bool     haveLast_ = false;
    uint64_t lastNs_ = 0;
    uint64_t lastIntervalNs_ = 0;
    double   lastIntervalSpeed_ = 0.0;
    uint64_t accepted_ = 0;
    uint64_t rejected_ = 0;
};

#endif // WHEEL_SPEED_H

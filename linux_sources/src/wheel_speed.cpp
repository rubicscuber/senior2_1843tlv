#include "wheel_speed.h"

#include <algorithm>
#include <cmath>

bool WheelParams::validate(std::string& err) const
{
    if (!(diameterM > 0.0)) {
        err = "wheel diameter must be greater than 0";
        return false;
    }
    if (pulsesPerRev < 1) {
        err = "pulses per revolution must be at least 1";
        return false;
    }
    if (!(maxSpeedMps > 0.0)) {
        err = "max speed must be greater than 0";
        return false;
    }
    if (!(stopTimeoutS > 0.0)) {
        err = "stop timeout must be greater than 0";
        return false;
    }
    if (window < 0) {
        err = "window must be 0 (auto) or positive";
        return false;
    }
    return true;
}

WheelSpeedEstimator::WheelSpeedEstimator(const WheelParams& p)
    : window_(p.window > 0 ? static_cast<size_t>(p.window)
                           : static_cast<size_t>(std::clamp(p.pulsesPerRev, 2, 32))),
      distPerPulse_(M_PI * p.diameterM / p.pulsesPerRev),
      minIntervalNs_(static_cast<uint64_t>(distPerPulse_ / p.maxSpeedMps * 1e9)),
      stopTimeoutNs_(static_cast<uint64_t>(p.stopTimeoutS * 1e9))
{
}

bool WheelSpeedEstimator::addPulse(uint64_t tNs, uint32_t count)
{
    if (count == 0)
        count = 1;
    if (haveLast_) {
        if (tNs <= lastNs_) { // duplicate or out-of-order timestamp
            rejected_++;
            return false;
        }
        const uint64_t dt = tNs - lastNs_;
        // `count` pulses in dt: the mean interval must still be physically plausible
        if (dt < minIntervalNs_ * count) {
            rejected_++;
            return false;
        }
        if (dt > stopTimeoutNs_) {
            // the wheel stopped in between: never average across a stop
            samples_.clear();
            lastIntervalNs_ = 0;
            lastIntervalSpeed_ = 0.0;
        } else {
            lastIntervalNs_ = dt / count;
            lastIntervalSpeed_ = distPerPulse_ * count / (dt * 1e-9);
        }
    }
    samples_.push_back({tNs, count});
    while (samples_.size() > window_ + 1)
        samples_.pop_front();
    haveLast_ = true;
    lastNs_ = tNs;
    accepted_ += count;
    return true;
}

double WheelSpeedEstimator::speedAt(uint64_t nowNs) const
{
    if (samples_.size() < 2)
        return 0.0;
    if (nowNs > lastNs_ && nowNs - lastNs_ > stopTimeoutNs_)
        return 0.0;

    const uint64_t span = samples_.back().tNs - samples_.front().tNs;
    if (span == 0)
        return 0.0;
    uint64_t pulses = 0;
    for (size_t i = 1; i < samples_.size(); i++) // the first sample only marks the window start
        pulses += samples_[i].count;
    double v = static_cast<double>(pulses) * distPerPulse_ / (span * 1e-9);

    // no pulse yet since lastNs_: the true speed cannot exceed one pulse
    // distance per elapsed time, so cap (and thereby decay) the estimate
    if (nowNs > lastNs_) {
        const double vCap = distPerPulse_ / ((nowNs - lastNs_) * 1e-9);
        if (vCap < v)
            v = vCap;
    }
    return v;
}

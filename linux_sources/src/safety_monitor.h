// Two-second-rule and approach-speed assessment of radar targets.
//
// Geometry (rear-facing radar, sensor frame): +Y is the radar boresight,
// pointing BEHIND the vehicle along the road; X is lateral; Z is up. The
// tracker reports target positions and velocities RELATIVE to the moving
// sensor, so a target approaching from behind has a negative Y velocity and a
// stationary roadside object recedes at exactly the self speed.
//
// Per target:
//   range r        = hypot(x, y)                  (horizontal road gap)
//   closing speed  = -(x*vx + y*vy) / r           (> 0 when approaching)
//   ground speed   = max(0, selfSpeed - vy)       (target speed along our
//                    direction of travel; 0 for stationary objects and for
//                    traffic moving the other way)
//   time gap       = r / groundSpeed              (only when it is a follower)
//   gap violation  = follower && inCorridor && timeGap < gapSeconds
//   approach alert = closing speed > approachThreshold
// Pure logic, no hardware dependency.
#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <cmath>
#include <cstdint>
#include <vector>

#include "tm_types.h"

struct SafetyParams {
    double gapSeconds = 2.0;            // the "two seconds"
    double approachThresholdMps = 0.0;  // closing speed above which an approach alert fires
    double corridorHalfWidthM = 0.0;    // |x| limit for the gap rule; 0 = no lateral filter
    double minFollowerSpeedMps = 0.5;   // below this ground speed a target is "not following"
    double minRangeM = 0.1;             // degenerate tracks closer than this are skipped
    bool   useHorizontal = true;        // range/closing from X,Y only (sensor is mounted above the road)
};

struct TargetAssessment {
    uint32_t tid = 0;
    bool   valid = false;               // false: skipped (non-finite input or below minRange)
    double range = 0.0;
    double closingSpeed = 0.0;
    double groundSpeed = 0.0;
    double timeGap = INFINITY;          // +inf when not a follower
    bool   inCorridor = true;
    bool   isFollower = false;
    bool   gapViolation = false;
    bool   approachAlert = false;
};

struct FrameAssessment {
    double selfSpeed = 0.0;
    std::vector<TargetAssessment> targets; // same order as the TargetList
    bool   anyGapViolation = false;
    bool   anyApproachAlert = false;
    double minTimeGap = INFINITY;          // over followers inside the corridor
    double maxClosingSpeed = -INFINITY;
};

class SafetyMonitor {
public:
    explicit SafetyMonitor(const SafetyParams& p) : p_(p) {}

    FrameAssessment evaluate(const TargetList& targets, double selfSpeedMps) const;
    const SafetyParams& params() const { return p_; }

private:
    SafetyParams p_;
};

#endif // SAFETY_MONITOR_H

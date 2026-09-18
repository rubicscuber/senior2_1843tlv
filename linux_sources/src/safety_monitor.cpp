#include "safety_monitor.h"

#include <algorithm>

FrameAssessment SafetyMonitor::evaluate(const TargetList& t, double selfSpeedMps) const
{
    FrameAssessment fa;
    fa.selfSpeed = selfSpeedMps;

    for (size_t i = 0; i < t.size(); i++) {
        TargetAssessment a;
        a.tid = t.tid[i];
        const double x = t.posX[i], y = t.posY[i], z = t.posZ[i];
        const double vx = t.velX[i], vy = t.velY[i], vz = t.velZ[i];

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)
            || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
            fa.targets.push_back(a);
            continue;
        }

        double r, dot;
        if (p_.useHorizontal) {
            r = std::hypot(x, y);
            dot = x * vx + y * vy;
        } else {
            r = std::sqrt(x * x + y * y + z * z);
            dot = x * vx + y * vy + z * vz;
        }
        if (r < p_.minRangeM) {
            fa.targets.push_back(a);
            continue;
        }

        a.valid = true;
        a.range = r;
        a.closingSpeed = -dot / r;
        // speed along our direction of travel: we move toward -Y, so a target
        // keeping pace has vy == -selfSpeed... in the sensor frame a target at
        // rest relative to us has vy == 0, and a stationary object vy == +selfSpeed
        a.groundSpeed = std::max(0.0, selfSpeedMps - vy);
        a.isFollower = a.groundSpeed > p_.minFollowerSpeedMps;
        a.timeGap = a.isFollower ? r / a.groundSpeed : INFINITY;
        a.inCorridor = (p_.corridorHalfWidthM <= 0.0) || (std::fabs(x) <= p_.corridorHalfWidthM);
        a.gapViolation = a.isFollower && a.inCorridor && a.timeGap < p_.gapSeconds;
        a.approachAlert = a.closingSpeed > p_.approachThresholdMps;

        fa.anyGapViolation = fa.anyGapViolation || a.gapViolation;
        fa.anyApproachAlert = fa.anyApproachAlert || a.approachAlert;
        if (a.isFollower && a.inCorridor)
            fa.minTimeGap = std::min(fa.minTimeGap, a.timeGap);
        fa.maxClosingSpeed = std::max(fa.maxClosingSpeed, a.closingSpeed);
        fa.targets.push_back(a);
    }
    return fa;
}

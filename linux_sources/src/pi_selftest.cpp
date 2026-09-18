#include "pi_selftest.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "alert_leds.h"
#include "safety_monitor.h"
#include "tm_types.h"
#include "wheel_speed.h"

namespace {

int g_failures = 0;

void check(bool ok, const char* label)
{
    std::printf("%s  %s\n", ok ? "ok  " : "FAIL", label);
    if (!ok)
        g_failures++;
}

void checkNear(double got, double expected, double tol, const char* label)
{
    const bool ok = std::isfinite(got) && std::fabs(got - expected) <= tol;
    if (ok)
        std::printf("ok    %s\n", label);
    else
        std::printf("FAIL  %s (got %.4f, expected %.4f)\n", label, got, expected);
    if (!ok)
        g_failures++;
}

constexpr uint64_t S = 1000000000ull;
constexpr uint64_t MS = 1000000ull;

void addTarget(TargetList& t, uint32_t tid, float x, float y, float z, float vx, float vy, float vz)
{
    t.tid.push_back(tid);
    t.posX.push_back(x);
    t.posY.push_back(y);
    t.posZ.push_back(z);
    t.velX.push_back(vx);
    t.velY.push_back(vy);
    t.velZ.push_back(vz);
    t.accX.push_back(0);
    t.accY.push_back(0);
    t.accZ.push_back(0);
}

// records every backend transition for inspection
struct RecordingBackend : LedBackend {
    struct Ev {
        uint64_t t;
        char which;
        bool on;
    };
    std::vector<Ev> events;
    void setGapLed(bool on, uint64_t t) override { events.push_back({t, 'g', on}); }
    void setSpeedLed(bool on, uint64_t t) override { events.push_back({t, 's', on}); }
};

void testWheel()
{
    std::printf("-- WheelSpeedEstimator\n");
    WheelParams p;
    p.diameterM = 0.7;
    p.pulsesPerRev = 1;
    std::string err;
    check(p.validate(err), "wheel: valid params accepted");

    WheelSpeedEstimator w(p);
    checkNear(w.distancePerPulse(), 2.19911, 1e-4, "wheel: distance per pulse = pi*D/ppr");
    checkNear(w.speedAt(0), 0.0, 1e-12, "wheel: no pulses -> 0 m/s");
    w.addPulse(1 * S);
    checkNear(w.speedAt(1 * S), 0.0, 1e-12, "wheel: one pulse -> still 0 m/s");
    w.addPulse(1 * S + 100 * MS);
    checkNear(w.speedAt(1 * S + 100 * MS), 21.9911, 0.01, "wheel: 100 ms period -> 21.99 m/s");
    check(!w.addPulse(1 * S + 101 * MS), "wheel: 1 ms glitch rejected");
    check(w.rejectedPulses() == 1 && w.acceptedPulses() == 2, "wheel: accepted/rejected counters");
    checkNear(w.speedAt(1 * S + 101 * MS), 21.9911, 0.01, "wheel: glitch leaves the estimate unchanged");
    checkNear(w.speedAt(1 * S + 600 * MS), 4.3982, 0.01, "wheel: overdue pulse caps speed (0.5 s -> 4.40 m/s)");
    checkNear(w.speedAt(1 * S + 2600 * MS), 0.0, 1e-12, "wheel: stop timeout -> 0 m/s");

    WheelSpeedEstimator w2(p);
    w2.addPulse(1 * S);
    w2.addPulse(1 * S + 300 * MS, 3);
    checkNear(w2.speedAt(1 * S + 300 * MS), 21.9911, 0.01, "wheel: coalesced count=3 over 300 ms -> 21.99 m/s");

    WheelParams p4 = p;
    p4.pulsesPerRev = 4;
    WheelSpeedEstimator w4(p4);
    check(w4.window() == 4, "wheel: auto window = pulses per rev");
    uint64_t t = 1 * S;
    w4.addPulse(t);
    t += 40 * MS; w4.addPulse(t);
    t += 60 * MS; w4.addPulse(t);
    t += 40 * MS; w4.addPulse(t);
    t += 60 * MS; w4.addPulse(t);
    checkNear(w4.speedAt(t), 10.9956, 0.02, "wheel: 4 ppr uneven spacing averages to 11.0 m/s");
    t += 5 * S; // wheel stopped, then restarts
    w4.addPulse(t);
    checkNear(w4.speedAt(t), 0.0, 1e-12, "wheel: restart after a stop needs new pulses");
    t += 100 * MS;
    w4.addPulse(t);
    checkNear(w4.speedAt(t), 5.4978, 0.01, "wheel: restart uses only post-stop pulses");

    WheelParams bad = p;
    bad.diameterM = 0;
    check(!bad.validate(err), "wheel: diameter 0 rejected");
    bad = p;
    bad.pulsesPerRev = 0;
    check(!bad.validate(err), "wheel: 0 pulses/rev rejected");
}

void testSafety()
{
    std::printf("-- SafetyMonitor\n");
    SafetyParams sp;
    sp.approachThresholdMps = 2.0;
    SafetyMonitor m(sp);

    TargetList follower;
    addTarget(follower, 5, 0, 20, 0, 0, -3, 0);
    FrameAssessment a = m.evaluate(follower, 10.0);
    check(a.targets.size() == 1 && a.targets[0].valid, "safety: follower assessed");
    checkNear(a.targets[0].closingSpeed, 3.0, 1e-6, "safety: closing speed 3 m/s");
    checkNear(a.targets[0].groundSpeed, 13.0, 1e-6, "safety: ground speed = self + closing = 13 m/s");
    checkNear(a.targets[0].timeGap, 20.0 / 13.0, 1e-6, "safety: time gap 1.538 s");
    check(a.targets[0].gapViolation && a.anyGapViolation, "safety: two-second rule violated");
    check(a.targets[0].approachAlert && a.anyApproachAlert, "safety: approach alert above 2 m/s");
    checkNear(a.minTimeGap, 20.0 / 13.0, 1e-6, "safety: frame min time gap");
    checkNear(a.maxClosingSpeed, 3.0, 1e-6, "safety: frame max closing speed");

    TargetList stationary;
    addTarget(stationary, 9, 4, 15, 0, 0, 10, 0); // recedes at exactly self speed
    a = m.evaluate(stationary, 10.0);
    checkNear(a.targets[0].groundSpeed, 0.0, 1e-6, "safety: stationary object -> ground speed 0");
    check(!a.targets[0].isFollower && !a.targets[0].gapViolation, "safety: stationary object is not a follower");
    check(!a.targets[0].approachAlert && !a.anyApproachAlert, "safety: receding object never alerts");
    check(std::isinf(a.targets[0].timeGap) && std::isinf(a.minTimeGap), "safety: no follower -> infinite gap");

    SafetyParams sc = sp;
    sc.corridorHalfWidthM = 1.75;
    SafetyMonitor mc(sc);
    TargetList adjacent;
    addTarget(adjacent, 11, 3, 12, 0, 0, -3, 0);
    a = mc.evaluate(adjacent, 10.0);
    check(!a.targets[0].inCorridor && !a.targets[0].gapViolation, "safety: outside corridor -> no gap violation");
    check(a.targets[0].approachAlert, "safety: approach alert is lane-independent");
    a = m.evaluate(adjacent, 10.0);
    check(a.targets[0].gapViolation, "safety: same target violates without a corridor");

    TargetList slower;
    addTarget(slower, 7, 0, 20, 0, 0, 1, 0); // slower than us but too close
    a = m.evaluate(slower, 15.0);
    checkNear(a.targets[0].closingSpeed, -1.0, 1e-6, "safety: slower follower has negative closing speed");
    checkNear(a.targets[0].timeGap, 20.0 / 14.0, 1e-6, "safety: slower follower gap 1.43 s");
    check(a.targets[0].gapViolation && !a.targets[0].approachAlert, "safety: gap violation without approach alert");

    TargetList opposite;
    addTarget(opposite, 8, 0, 6, 0, 0, 25, 0); // traffic moving the other way, close
    a = m.evaluate(opposite, 10.0);
    check(!a.targets[0].isFollower && !a.targets[0].gapViolation, "safety: traffic moving away is not a follower");

    TargetList oblique;
    addTarget(oblique, 3, 3, 4, 0, -3, -4, 0);
    a = m.evaluate(oblique, 0.0);
    checkNear(a.targets[0].closingSpeed, 5.0, 1e-6, "safety: oblique closing speed 5 m/s");

    TargetList degenerate;
    addTarget(degenerate, 1, 0.01f, 0.02f, 0, 0, 0, 0);
    a = m.evaluate(degenerate, 10.0);
    check(!a.targets[0].valid && !a.anyGapViolation, "safety: track below min range skipped");

    a = m.evaluate(TargetList{}, 10.0);
    check(a.targets.empty() && !a.anyGapViolation && !a.anyApproachAlert && std::isinf(a.minTimeGap),
          "safety: empty target list -> no alerts");
}

void testLeds()
{
    std::printf("-- AlertLeds\n");
    AlertLedParams lp; // hold 1 s, 8 Hz, frame timeout 1 s
    auto recOwner = std::make_unique<RecordingBackend>();
    RecordingBackend* rec = recOwner.get();
    AlertLeds leds(std::move(recOwner), lp);

    leds.update(0);
    check(rec->events.empty(), "leds: nothing happens before the first frame");
    leds.onFrame(0, true, true);
    leds.update(0);
    check(leds.gapLedOn() && leds.speedLedOn(), "leds: both LEDs on after an alerting frame");
    check(rec->events.size() == 2, "leds: backend called once per LED");
    leds.update(70 * MS);
    check(leds.gapLedOn() && !leds.speedLedOn(), "leds: speed LED off after half a flash period");
    leds.update(130 * MS);
    check(leds.speedLedOn(), "leds: speed LED on again in the next half period");
    leds.onFrame(900 * MS, false, false);
    leds.update(900 * MS);
    check(leds.gapLedOn() && leds.approachActive(), "leds: hold keeps alerts active 0.9 s later");
    leds.onFrame(1100 * MS, false, false);
    leds.update(1100 * MS);
    check(!leds.gapLedOn() && !leds.speedLedOn(), "leds: alerts expire after the 1 s hold");
    const size_t before = rec->events.size();
    leds.update(1200 * MS);
    check(rec->events.size() == before, "leds: no backend calls without a change");

    AlertLedParams longHold = lp;
    longHold.holdSeconds = 5.0;
    AlertLeds failsafe(std::make_unique<RecordingBackend>(), longHold);
    failsafe.onFrame(0, true, false);
    failsafe.update(0);
    check(failsafe.gapLedOn(), "leds: fail-safe test starts with the gap LED on");
    failsafe.update(1500 * MS);
    check(!failsafe.gapLedOn(), "leds: no radar frames for > 1 s -> LEDs off despite the hold");

    auto rec3Owner = std::make_unique<RecordingBackend>();
    RecordingBackend* rec3 = rec3Owner.get();
    AlertLeds off(std::move(rec3Owner), lp);
    off.onFrame(0, true, false);
    off.update(0);
    off.allOff(10 * MS);
    check(rec3->events.size() == 2 && rec3->events[1].which == 'g' && !rec3->events[1].on,
          "leds: allOff emits a transition only for the lit LED");
    off.update(20 * MS);
    check(!off.gapLedOn() && rec3->events.size() == 2, "leds: allOff clears pending alerts");
}

} // namespace

int runPiSelfTests()
{
    g_failures = 0;
    testWheel();
    testSafety();
    testLeds();
    if (g_failures == 0)
        std::printf("All self-tests passed.\n");
    else
        std::printf("%d self-test(s) FAILED.\n", g_failures);
    return g_failures;
}

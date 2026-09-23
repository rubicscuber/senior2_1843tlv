#include "pi_selftest.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "alert_leds.h"
#include "alert_sound.h"
#include "frame_parser.h"
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

    // a hold shorter than the frame period (here 0) must neither blank nor
    // flicker an LED: the alert stays on until the next frame replaces it
    AlertLedParams noHold = lp;
    noHold.holdSeconds = 0.0;
    AlertLeds hold0(std::make_unique<RecordingBackend>(), noHold);
    hold0.onFrame(0, true, true);
    hold0.update(0);
    check(hold0.gapLedOn() && hold0.speedLedOn(), "leds: hold 0 still lights the LEDs on the alerting frame");
    hold0.update(30 * MS);
    check(hold0.gapLedOn() && hold0.approachActive(), "leds: hold 0 keeps the alert until the next frame");
    hold0.onFrame(50 * MS, false, false);
    hold0.update(50 * MS);
    check(!hold0.gapLedOn() && !hold0.speedLedOn(), "leds: hold 0 clears on the next frame without alerts");
}

// records the approach-alert transitions an audio output would receive
struct RecordingListener : ApproachAlertListener {
    std::vector<std::pair<bool, uint64_t>> events;
    void onApproachAlert(bool active, uint64_t t) override { events.push_back({active, t}); }
};

void testSound()
{
    std::printf("-- approach alert -> audio\n");
    AlertLedParams lp; // hold 1 s, frame timeout 1 s
    RecordingListener rec;
    AlertLeds leds(std::make_unique<RecordingBackend>(), lp);
    leds.setApproachListener(&rec);

    leds.onFrame(0, false, false);
    leds.update(0);
    check(rec.events.empty(), "sound: no transition without an approach alert");
    leds.onFrame(50 * MS, false, true);
    leds.update(50 * MS);
    check(rec.events.size() == 1 && rec.events[0].first && rec.events[0].second == 50 * MS,
          "sound: starts on the first alerting frame");
    leds.onFrame(100 * MS, false, true);
    leds.update(100 * MS);
    leds.onFrame(150 * MS, false, false); // one frame without the target: held
    leds.update(150 * MS);
    leds.onFrame(200 * MS, false, true);
    leds.update(200 * MS);
    check(rec.events.size() == 1, "sound: stays started through the hold, no retrigger");
    leds.onFrame(250 * MS, false, false);
    leds.update(250 * MS);
    leds.update(1200 * MS); // 1 s hold after the last alerting frame (200 ms) is over
    check(rec.events.size() == 2 && !rec.events[1].first && rec.events[1].second == 1200 * MS,
          "sound: stops once the hold has expired");
    leds.onFrame(1300 * MS, false, true);
    leds.update(1300 * MS);
    leds.allOff(1400 * MS);
    check(rec.events.size() == 4 && rec.events[2].first && !rec.events[3].first
              && rec.events[3].second == 1400 * MS,
          "sound: restarts on a new alert and allOff stops it");
    leds.allOff(1500 * MS);
    check(rec.events.size() == 4, "sound: allOff while inactive sends nothing");

    // the sound module itself, in simulation (no player process)
    AlertSoundParams sp;
    sp.file = "alert.wav";
    sp.simulate = true;
    AlertSound snd(sp, 0);
    snd.onApproachAlert(true, 10 * MS);
    snd.onApproachAlert(true, 20 * MS);
    check(snd.active() && snd.starts() == 1, "sound: repeated activation counts once");
    snd.poll(30 * MS);
    snd.onApproachAlert(false, 40 * MS);
    check(!snd.active() && snd.starts() == 1, "sound: deactivation clears the active state");
    snd.onApproachAlert(true, 50 * MS);
    check(snd.starts() == 2, "sound: a new activation starts again");
}

// ---- frame parser: frames complete on the next magic word or on packetLength ----

void putU32(std::vector<uint8_t>& b, uint32_t v)
{
    for (int i = 0; i < 4; i++)
        b.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

// a frame with no TLVs: 40-byte header only, packetLength = 40 + padding
std::vector<uint8_t> makeFrame(uint32_t frameNumber, uint32_t padding = 0)
{
    std::vector<uint8_t> b = {2, 1, 4, 3, 6, 5, 8, 7};
    putU32(b, 0x03030000);         // version
    putU32(b, 40 + padding);       // packetLength
    putU32(b, 0xA1843);            // platform
    putU32(b, frameNumber);
    putU32(b, 0);                  // timestamp
    putU32(b, 0);                  // numDetectedObj
    putU32(b, 0);                  // numTLVs
    putU32(b, 0);                  // subFrameNumber
    b.insert(b.end(), padding, 0);
    return b;
}

void testParser()
{
    std::printf("-- frame parser\n");
    int avail = 0;

    std::vector<uint8_t> buf = makeFrame(1);
    std::vector<Frame> frames = parseBytesTM(buf, ReadMode::FIFO, avail);
    check(frames.size() == 1 && frames[0].valid && frames[0].header.frameNumber == 1 && buf.empty(),
          "parser: a lone frame completes on its packetLength (no next magic word needed)");

    buf = makeFrame(2, 16);
    buf.pop_back(); // one byte short
    frames = parseBytesTM(buf, ReadMode::FIFO, avail);
    check(frames.empty() && avail == 0 && buf.size() == 55,
          "parser: a frame missing its last byte waits");
    buf.push_back(0);
    frames = parseBytesTM(buf, ReadMode::FIFO, avail);
    check(frames.size() == 1 && frames[0].valid && frames[0].header.frameNumber == 2 && buf.empty(),
          "parser: ...and completes once the byte arrives");

    // two frames plus the first half of a third: FIFO yields one at a time
    buf = makeFrame(3);
    const std::vector<uint8_t> f4 = makeFrame(4, 8);
    buf.insert(buf.end(), f4.begin(), f4.end());
    buf.insert(buf.end(), {2, 1, 4, 3});
    frames = parseBytesTM(buf, ReadMode::FIFO, avail);
    check(avail == 2 && frames.size() == 1 && frames[0].header.frameNumber == 3,
          "parser: FIFO returns the oldest of two complete frames");
    frames = parseBytesTM(buf, ReadMode::FIFO, avail);
    check(avail == 1 && frames.size() == 1 && frames[0].header.frameNumber == 4 && buf.size() == 4,
          "parser: the partial trailing frame is kept in the buffer");

    // a header claiming more bytes than exist stays pending until the next
    // magic word bounds it (then it is reported as a bad frame, as before)
    buf = makeFrame(5);
    buf[12] = 0xFF; // packetLength becomes 255: more than the 40 bytes present
    const std::vector<uint8_t> f6 = makeFrame(6);
    frames = parseBytesTM(buf, ReadMode::FIFO, avail);
    check(frames.empty() && buf.size() == 40, "parser: implausible packetLength waits for the next frame");
    buf.insert(buf.end(), f6.begin(), f6.end());
    setFrameParserQuiet(true);
    frames = parseBytesTM(buf, ReadMode::ALL, avail);
    setFrameParserQuiet(false);
    check(avail == 2 && frames.size() == 2 && !frames[0].valid && frames[1].valid
              && frames[1].header.frameNumber == 6 && buf.empty(),
          "parser: bad length frame is invalid, the following one parses");
}

} // namespace

int runPiSelfTests()
{
    g_failures = 0;
    testWheel();
    testSafety();
    testLeds();
    testSound();
    testParser();
    if (g_failures == 0)
        std::printf("All self-tests passed.\n");
    else
        std::printf("%d self-test(s) FAILED.\n", g_failures);
    return g_failures;
}

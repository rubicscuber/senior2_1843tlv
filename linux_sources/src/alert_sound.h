// Audio output for the approach alert: plays an audio file through an
// external player (aplay by default, i.e. ALSA -> the Pi's 3.5 mm jack) while
// the alert is active, in place of or next to the flashing LED.
//
// The player runs as a child process so the safety loop is never blocked by
// audio; it is (re)started while the alert stays active and terminated as
// soon as the alert ends. AlertSound listens to the AlertLeds state machine
// through ApproachAlertListener, so hold time and fail-safe apply unchanged.
#ifndef ALERT_SOUND_H
#define ALERT_SOUND_H

#include <cstdint>
#include <string>
#include <vector>

#include <sys/types.h>

#include "alert_leds.h"

struct AlertSoundParams {
    std::string file;                // audio file to play; empty = disabled
    std::string player = "aplay -q"; // player command; the file is appended as the last argument
    bool repeat = true;              // restart the file while the alert stays active
    bool simulate = false;           // print "[SOUND] ..." transitions instead of playing
};

class AlertSound : public ApproachAlertListener {
public:
    AlertSound(const AlertSoundParams& p, uint64_t epochNs);
    ~AlertSound() override; // stops a running player

    // Verifies the file is readable and the player executable can be found.
    bool check(std::string& err) const;

    // Sound check: plays the file once and waits for the player to finish
    // (at most timeoutS). Returns false if it could not be started or failed.
    bool playOnce(double timeoutS);

    // ApproachAlertListener: called by AlertLeds on alert transitions.
    void onApproachAlert(bool active, uint64_t nowNs) override;

    // Call regularly (every main-loop iteration): reaps a finished player,
    // restarts it if the alert is still active, escalates a stop to SIGKILL.
    void poll(uint64_t nowNs);

    // Terminate a running player (alert over or program exit).
    void stop(uint64_t nowNs);

    bool active() const { return active_; }
    bool playing() const { return pid_ > 0 && !stopping_; }
    unsigned long starts() const { return starts_; }
    const std::string& playerCommand() const { return p_.player; }

private:
    bool spawn();
    bool reap(bool block); // true when no child remains
    void print(const char* what, uint64_t nowNs) const;

    AlertSoundParams p_;
    std::vector<std::string> argv_;
    uint64_t epochNs_;
    bool active_ = false;
    bool wantStart_ = false; // activation seen while the previous player was still stopping
    pid_t pid_ = -1;
    bool stopping_ = false;
    uint64_t killAtNs_ = 0;
    unsigned long starts_ = 0;
};

#endif // ALERT_SOUND_H

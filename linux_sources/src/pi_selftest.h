// Unit-style self-tests for the hardware-independent safety-monitor logic
// (wheel speed estimator, safety monitor, alert LED state machine).
// Run with `console_only_Pi --self-test`; returns the number of failed checks.
#ifndef PI_SELFTEST_H
#define PI_SELFTEST_H

int runPiSelfTests();

#endif // PI_SELFTEST_H

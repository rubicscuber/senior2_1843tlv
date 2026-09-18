// Monotonic clock helper.
// CLOCK_MONOTONIC is the clock the kernel stamps GPIO line events with, and it
// never jumps (a Raspberry Pi has no RTC, so CLOCK_REALTIME moves when NTP
// syncs after boot). Every timestamp in the safety monitor uses this clock.
#ifndef MONO_CLOCK_H
#define MONO_CLOCK_H

#include <cstdint>
#include <time.h>

inline uint64_t monotonicNowNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

#endif // MONO_CLOCK_H

#!/usr/bin/env python3
"""Generate alert.wav, the approach-alert sound for console_only_Pi.

Standard library only (wave, struct, math): run it anywhere, e.g. on the Pi:
    python3 make_alert_wav.py              # writes alert.wav next to this script
    python3 make_alert_wav.py my.wav       # or to the given path

Format: 16-bit PCM, stereo, 44.1 kHz, the native format of the Pi's 3.5 mm
headphone output, so `aplay` plays it through hw: and plughw: alike.

Sound: a 1 s "warble" (four 100 ms + 100 ms alternations of 1000 Hz and
1500 Hz, then 200 ms of silence). console_only_Pi restarts the file while the
approach alert stays active, so the silent tail turns the repeats into a
steady, recognisable rhythm instead of a continuous tone. Each tone has a
5 ms fade in/out so there are no clicks at the segment boundaries.
Tweak the constants below and rerun to change it.
"""
import math
import os
import struct
import sys
import wave

SAMPLE_RATE = 44100
CHANNELS = 2
TONE_A_HZ = 1000.0
TONE_B_HZ = 1500.0
SEGMENT_S = 0.100       # length of each tone
CYCLES = 4              # A+B pairs per file
TAIL_SILENCE_S = 0.200  # pause at the end (gap between repeats)
FADE_S = 0.005          # fade in/out per tone, against clicks
AMPLITUDE = 0.8         # of full scale (0.0 .. 1.0)


def tone(freq_hz, seconds):
    n = int(SAMPLE_RATE * seconds)
    fade = max(1, int(SAMPLE_RATE * FADE_S))
    out = []
    for i in range(n):
        env = 1.0
        if i < fade:
            env = i / fade
        elif i >= n - fade:
            env = (n - 1 - i) / fade
        out.append(AMPLITUDE * env * math.sin(2.0 * math.pi * freq_hz * i / SAMPLE_RATE))
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "alert.wav")
    samples = []
    for _ in range(CYCLES):
        samples += tone(TONE_A_HZ, SEGMENT_S)
        samples += tone(TONE_B_HZ, SEGMENT_S)
    samples += [0.0] * int(SAMPLE_RATE * TAIL_SILENCE_S)

    frames = bytearray()
    for s in samples:
        v = int(max(-1.0, min(1.0, s)) * 32767)
        frames += struct.pack("<h", v) * CHANNELS

    with wave.open(path, "wb") as w:
        w.setnchannels(CHANNELS)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(bytes(frames))
    print(f"wrote {path}: {len(samples) / SAMPLE_RATE:.2f} s, {CHANNELS} ch, 16-bit, {SAMPLE_RATE} Hz, "
          f"{os.path.getsize(path)} bytes")


if __name__ == "__main__":
    main()

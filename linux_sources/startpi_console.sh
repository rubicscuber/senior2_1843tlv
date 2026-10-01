#!/bin/bash
amixer -c Headphones sset PCM 90%
./console_only_Pi -d /dev/ttyACM1 -c /dev/ttyACM0 -g ../chirp_configs/18xx_traffic_monitoring_70m_MIMO_3D.cfg --self-speed 10 --approach-threshold 2 --alert-sound sounds/alert.wav --sound-player "aplay -q -D plughw:Headphones" --sound-check

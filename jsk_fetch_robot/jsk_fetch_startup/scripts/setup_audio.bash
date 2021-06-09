#!/usr/bin/env bash

# If fetch does not speak, please check speaker device name by the following command:
# $ pactl list | grep Name | grep alsa_output
# Then, make sure that the speaker volume is not zero
# $ alsamixer
# For fetch15's speaker
pactl set-default-sink $AUDIO_DEVICE || true && pactl set-sink-volume $AUDIO_DEVICE 100% || true

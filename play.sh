#!/bin/bash
#simple audio play from usb microphone to pc speaker
#i used a local fm station on 105Mhz to test audio streams this is hardcoded! in FM mode
#pasuspender -- sh -c 'arecord -D plughw:1,0 -c 1 -f S32_LE -r 48000 | aplay -D plughw:0,0'

#!/bin/bash
# Simple stereo audio play from USB microphone to PC speakers for SoftRock testing
# Bypasses PulseAudio to minimize latency and prevent audio resamplers from breaking the IQ phase.

# Ensure the script exits cleanly if interrupted (Ctrl+C)
trap 'echo "Stopping audio loop..."; exit' INT TERM

# Run arecord and aplay using ALSA direct hardware access
# -c 2: Forces 2 channels (Stereo) to capture both the I and Q signals
# -f S16_LE: Standard 16-bit format compatible with almost all USB sound cards
# -r 48000: Hardcoded 48kHz sample rate for testing
pasuspender -- sh -c 'arecord -D plughw:1,0 -c 2 -f S16_LE -r 48000 | aplay -D plughw:0,0'


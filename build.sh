#!/bin/bash
# Build (and optionally flash) the fan remote firmware.
# Usage:
#   ./build.sh              — generate config, compile, and flash
#   ./build.sh --no-upload  — generate config and compile only
#   PORT=/dev/ttyUSB1 ./build.sh  — override serial port

set -e
cd "$(dirname "$0")"

echo "=== gen_config ==="
python3 gen_config.py

echo "=== compile ==="
~/bin/arduino-cli compile --fqbn esp8266:esp8266:generic fan_remote

if [ "${1}" = "--no-upload" ]; then
    echo "=== skipping upload (--no-upload) ==="
    exit 0
fi

echo "=== upload ==="
~/bin/arduino-cli upload \
    --fqbn esp8266:esp8266:generic \
    --port "${PORT:-/dev/ttyUSB0}" \
    fan_remote

echo "=== done ==="

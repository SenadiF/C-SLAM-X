#!/usr/bin/env bash
# Rebuilds src/esp32/liburos_clock_fix.a from extras/uros_clock_fix.cpp with
# the ESP32 toolchain that the Arduino IDE installed.
set -e
here="$(cd "$(dirname "$0")" && pwd)"
bin="$(dirname "$(find ~/.arduino15/packages/esp32/tools -name xtensa-esp32-elf-g++ | head -1)")"
obj="$(mktemp --suffix=.o)"
"$bin/xtensa-esp32-elf-g++" -c -Os -mlongcalls -ffunction-sections -fdata-sections \
  -o "$obj" "$here/uros_clock_fix.cpp"
rm -f "$here/../src/esp32/liburos_clock_fix.a"
"$bin/xtensa-esp32-elf-ar" rcs "$here/../src/esp32/liburos_clock_fix.a" "$obj"
rm -f "$obj"
echo "built $here/../src/esp32/liburos_clock_fix.a"

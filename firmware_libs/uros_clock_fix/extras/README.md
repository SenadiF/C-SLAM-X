# uros_clock_fix

## Problem

`micro_ros_arduino` 2.0.8-jazzy ships `libmicroros.a` precompiled for a
32-bit `time_t`. Its `uxr_millis()` / `uxr_nanos()` call
`clock_gettime(CLOCK_REALTIME, &ts)` and read `tv_nsec` at byte offset 4.

ESP32 Arduino core 3.x (ESP-IDF 5) uses a 64-bit `time_t`: offset 4 is the
upper half of `tv_sec` (always 0) and `tv_nsec` is at offset 8. So
micro-ROS's clock only advances in whole seconds, and every timed wait in
the session (executor spin, ping, publish confirmation) lasts until the next
second boundary. On the robots this held `loop()`, and all sensor topics, at
~1 Hz.

Seen in the disassembly of `uxr_millis` in a linked sketch:

    call clock_gettime(1, sp)
    l32i a11, a1, 0    ; tv_sec  (low word)
    l32i a8,  a1, 4    ; "tv_nsec" - really the high word of tv_sec

## Fix

`library.properties` adds `-Wl,--wrap=uxr_millis -Wl,--wrap=uxr_nanos`, so
every call micro-ROS makes goes to `__wrap_uxr_millis` / `__wrap_uxr_nanos`
in `src/esp32/liburos_clock_fix.a`, which use `esp_timer_get_time()`.

The Arduino build only applies `ldflags` for a library that has a
precompiled archive, so the code lives in the `.a` (source in this folder,
rebuild with `build.sh`), not in `src/`.

## Use

1. Make the library visible to the Arduino IDE:
   `ln -s ~/Desktop/C-SLAM-X/firmware_libs/uros_clock_fix ~/Arduino/libraries/uros_clock_fix`
2. Add `#include <uros_clock_fix.h>` to the sketch.
3. Check: the linked `.elf` contains `__wrap_uxr_millis`, e.g.
   `xtensa-esp32-elf-nm sketch.ino.elf | grep __wrap_uxr`.

// Include this in a micro-ROS ESP32 sketch so the Arduino build links
// src/esp32/liburos_clock_fix.a with -Wl,--wrap=uxr_millis/uxr_nanos.
// Nothing to call - see extras/README.md.
#pragma once

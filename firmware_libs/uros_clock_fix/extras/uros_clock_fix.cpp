// Source of src/esp32/liburos_clock_fix.a (rebuild with build.sh).
// Kept out of src/ so the Arduino build uses the archive, which is what
// makes it apply library.properties' ldflags (--wrap).
#include <stdint.h>

extern "C" int64_t esp_timer_get_time(void);  // microseconds since boot

extern "C" int64_t __wrap_uxr_nanos(void)
{
  return esp_timer_get_time() * 1000LL;
}

extern "C" int64_t __wrap_uxr_millis(void)
{
  return esp_timer_get_time() / 1000LL;
}

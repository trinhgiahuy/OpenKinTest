#include "helpers.hh"

uint32_t millis() {
  long ms;
  time_t s;
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  s = spec.tv_sec;
  ms = round(spec.tv_nsec / 1.0e6);
  return (uint32_t) (s*1000 + ms);
}

void delay(uint32_t t) {
  usleep(t*1000);
  return;
}

#include "helpers.hh"

uint32_t millis() {
  long ms;
  long s;
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  s = (long)spec.tv_sec;
  ms = (long)spec.tv_nsec;
  ms = (ms / 1000000L);
  return (uint32_t) (s*1000 + ms);
}

void delay(uint32_t t) {
  usleep(t*1000);
  return;
}

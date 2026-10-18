// hw_types.h - types shared by the sketch (kept in a header so Arduino's auto-prototypes can see them).
#pragma once
#include <stdint.h>

struct Ultrasonic {
  uint8_t trig, echo;
  volatile uint32_t riseUs = 0, widthUs = 0;
  volatile bool rose = false, done = false;
};

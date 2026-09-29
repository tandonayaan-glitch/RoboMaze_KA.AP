// hw_types.h - types shared by the sketch (kept in a header so Arduino's auto-prototypes can see them).
#pragma once
#include <stdint.h>

struct Encoder {
  uint8_t pin;
  volatile int32_t ticks = 0;
  volatile int8_t dir = 1;          // sign of the last commanded wheel direction (single-channel encoder)
  volatile uint32_t lastUs = 0;
};

struct Ultrasonic {
  uint8_t trig, echo;
  volatile uint32_t riseUs = 0, widthUs = 0;
  volatile bool rose = false, done = false;
};

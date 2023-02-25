#include "ultrasonic.h"

long duration;

int get_distance() {
  duration = pulseIn(ultrasonic_echo_pin, HIGH);
  return duration * SOUND_SPEED/2;
}
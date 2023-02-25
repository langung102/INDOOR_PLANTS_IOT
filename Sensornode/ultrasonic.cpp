#include "ultrasonic.h"

long duration;

int get_distance_cm() {
  duration = pulseIn(ultrasonic_echo_pin, HIGH);
  return duration * SOUND_SPEED/2;
}

int get_distance_inch() {
  duration = pulseIn(ultrasonic_echo_pin, HIGH);
  return duration * SOUND_SPEED/2 * CM_TO_INCH;
}
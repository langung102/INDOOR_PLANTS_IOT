#include "ultrasonic.h"

long duration;

int get_distance() {
  digitalWrite(ultrasonic_trig_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(ultrasonic_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonic_trig_pin, LOW);

  duration = pulseIn(ultrasonic_echo_pin, HIGH);
  return duration * SOUND_SPEED/2;
}
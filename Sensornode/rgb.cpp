#include "rgb.h"

// void light_red() {
//   digitalWrite(rgb_R_pin, 255);
//   digitalWrite(rgb_G_pin, 0);
//   digitalWrite(rgb_B_pin, 0);
// }

// void light_blue() {
//   digitalWrite(rgb_R_pin, 0);
//   digitalWrite(rgb_G_pin, 0);
//   digitalWrite(rgb_B_pin, 255);
// }

void turn_on() {
  digitalWrite(rgb_R_pin, 255);
  digitalWrite(rgb_G_pin, 0);
  digitalWrite(rgb_B_pin, 0);
}

void turn_off() {
  digitalWrite(rgb_R_pin, 0);
  digitalWrite(rgb_G_pin, 0);
  digitalWrite(rgb_B_pin, 0);
}

void set_led(bool value) {
  if (value) {
    turn_on();
  } else {
    turn_off();
  }
}
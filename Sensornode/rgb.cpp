#include "rgb.h"

void light_red() {
  digitalWrite(rgb_R_pin, 255);
  digitalWrite(rgb_G_pin, 0);
  digitalWrite(rgb_B_pin, 0);
}

void light_blue() {
  digitalWrite(rgb_R_pin, 0);
  digitalWrite(rgb_G_pin, 0);
  digitalWrite(rgb_B_pin, 255);
}
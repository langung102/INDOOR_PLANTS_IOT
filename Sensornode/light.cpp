#include "light.h"

int get_light() {
  return analogRead(light_pin);
}
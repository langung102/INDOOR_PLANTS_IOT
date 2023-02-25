#include "soil.h"

int get_moisture_soil() {
  return analogRead(soil_pin);
}
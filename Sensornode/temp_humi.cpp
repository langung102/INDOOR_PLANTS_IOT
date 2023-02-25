#include "temp_humi.h"

DHT dht(temp_humi_pin, DHT11);

void begin_temp_humi() {
  dht.begin();
}

float get_temp() {
  return dht.readTemperature();
}

float get_humi() {
  return dht.readHumidity();
}
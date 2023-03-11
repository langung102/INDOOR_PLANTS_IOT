/*  Flow of the code

  1 - Put WiFi in STA Mode
  2 - Intialize ESPNOW
  3 - Add peer device
  4 - Define Send Callback Function
  5 - Define Receive Callback Function

*/

#include <esp_now.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "temp_humi.h"
#include "soil.h"
#include "light.h"
#include "global.h"
#include "ultrasonic.h"
#include "lcd.h"
#include "rgb.h"

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x35, 0xF3, 0x68};
uint8_t pump = 0;
unsigned long previousMillis = 0UL;
unsigned long interval = 700UL;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    uint16_t temp;
    uint16_t humi;
    uint16_t soil;
    uint16_t light;
    uint16_t distance;
    bool led;
    bool pump;
} struct_message;

// Create a struct_message called DHTReadings to hold sensor readings
struct_message outgoingReadings;

struct_message incomingReadings;

// Variable to store if sending data was successful
String success;
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.println("Data: ");Serial.println(incomingReadings.pump);
}

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println("Code start");

  pinMode(2, OUTPUT);
  pinMode(rgb_R_pin, OUTPUT);
  pinMode(rgb_G_pin, OUTPUT);
  pinMode(rgb_B_pin, OUTPUT);
  pinMode(ultrasonic_trig_pin, OUTPUT);
  pinMode(ultrasonic_echo_pin, INPUT);

  begin_temp_humi();

  setup_lcd();
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  for (int ii = 0; ii < 6; ++ii )
  {
    peerInfo.peer_addr[ii] = (uint8_t) broadcastAddress[ii];
  }
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

}

void loop()
{
  // temp_value = get_temp();
  // humi_value = get_humi();
  // soil_value = get_moisture_soil();
  // light_value = get_light();
  // distance_value = get_distance();

  temp_value = 38;
  humi_value = 81;
  soil_value = 29;
  light_value = 95;
  distance_value = 100;

  print_lcd();

  // sprintf(outgoingReadings.temp, "%d", temp_value);
  // sprintf(outgoingReadings.humi, "%d", humi_value);
  // sprintf(outgoingReadings.soil, "%d", soil_value);
  // sprintf(outgoingReadings.light, "%d", light_value);
  // sprintf(outgoingReadings.distance, "%d", distance_value);
  // sprintf(outgoingReadings.pump, "%d", digitalRead(pump_pin));

  outgoingReadings.temp = temp_value;
  outgoingReadings.humi = humi_value;
  outgoingReadings.soil = soil_value;
  outgoingReadings.light = light_value;
  outgoingReadings.distance = distance_value;
  outgoingReadings.pump = digitalRead(pump_pin);

  Serial.print("Temp: ");Serial.println(outgoingReadings.temp);
  Serial.print("Humi: ");Serial.println(outgoingReadings.humi);
  Serial.print("Soil: ");Serial.println(outgoingReadings.soil);
  Serial.print("Light: ");Serial.println(outgoingReadings.light );
  Serial.print("Distance: ");Serial.println(outgoingReadings.distance);
  Serial.print("Pump: ");Serial.println(outgoingReadings.pump);

  digitalWrite(2, (incomingReadings.pump));

  set_led(incomingReadings.led);

  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
    previousMillis = millis();
  }
}
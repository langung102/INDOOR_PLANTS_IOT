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
    char temp[20];
    char humi[20];
    char soil[20];
    char light[20];
    char distance[20];
    char pump[1];
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
  Serial.println("Data: ");Serial.println(*incomingReadings.pump - 48);
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
  
  temp_value = 100;
  humi_value = 100;
  soil_value = 100;
  light_value = 100;
  distance_value = 100;

  Serial.print("Temp: ");Serial.println(temp_value);
  Serial.print("Humi: ");Serial.println(humi_value);
  Serial.print("Soil: ");Serial.println(soil_value);
  Serial.print("Light: ");Serial.println(light_value);
  Serial.print("Distance: ");Serial.println(distance_value);
  Serial.print("Pump: ");Serial.println(digitalRead(pump_pin));

  print_lcd();

  sprintf(outgoingReadings.temp, "%d", temp_value);
  sprintf(outgoingReadings.humi, "%d", humi_value);
  sprintf(outgoingReadings.soil, "%d", soil_value);
  sprintf(outgoingReadings.light, "%d", light_value);
  sprintf(outgoingReadings.distance, "%d", distance_value);
  sprintf(outgoingReadings.pump, "%d", digitalRead(pump_pin));

  digitalWrite(2, (*incomingReadings.pump - 48));

  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
    previousMillis = millis();
  }
}
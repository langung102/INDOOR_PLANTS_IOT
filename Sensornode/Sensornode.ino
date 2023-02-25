/*  Flow of the code

  1 - Put WiFi in STA Mode
  2 - Intialize ESPNOW
  3 - Add peer device
  4 - Define Send Callback Function
  5 - Define Receive Callback Function

*/

#include <esp_now.h>
#include <Wire.h>
#include <WiFi.h>
#include "soil.h"

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x33, 0x58, 0x10};

uint8_t pump = 0;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    char msg[50];
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
  Serial.print("Data: ");Serial.println(*incomingReadings.msg - 48);
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

  // Set up I2C
  Wire.begin();

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
  // strcpy(outgoingReadings.msg, get_moisture_soil());
  sprintf(outgoingReadings.msg, "%d", get_moisture_soil());
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));

  digitalWrite(2, (*incomingReadings.msg - 48));
  
  Serial.println(outgoingReadings.msg);

  delay(1000);  
}

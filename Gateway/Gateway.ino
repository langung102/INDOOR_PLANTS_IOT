/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "config.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp32-hal-timer.h>
/************************ Example Starts Here *******************************/

// this int will hold the current count for our sketch
int count = 1;
int last = 0;
bool flag_pump = 0, flag_timer = 0, flag_pump_changed = 0;
unsigned long previousMillis_1 = 0UL;
unsigned long interval_1 = 1000UL;
unsigned long previousMillis_2 = 0UL;
unsigned long interval_2 = 10000UL;
hw_timer_t *My_timer = NULL;
// Track time of last published messages and limit feed->save events to once
// every IO_LOOP_DELAY milliseconds.
//
// Because this sketch is publishing AND subscribing, we can't use a long
// delay() function call in the main loop since that would prevent io.run()
// from being called often enough to receive all incoming messages.
//
// Instead, we can use the millis() function to get the current time in
// milliseconds and avoid publishing until IO_LOOP_DELAY milliseconds have
// passed.
#define IO_LOOP_DELAY 5000
unsigned long lastUpdate = 0;

// set up the feed
AdafruitIO_Feed *pump = io.feed("pump");
AdafruitIO_Feed *temp = io.feed("temp");
AdafruitIO_Feed *humi = io.feed("humi");
AdafruitIO_Feed *soil = io.feed("soil");
AdafruitIO_Feed *water = io.feed("water");
AdafruitIO_Feed *light = io.feed("light");
////////////////////////////////////////////

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x33, 0x58, 0x10};


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
  Serial.print("Data: ");Serial.println(*incomingReadings.temp - 48);
  Serial.print("Data: ");Serial.println(*incomingReadings.humi - 48);
  Serial.print("Data: ");Serial.println(*incomingReadings.soil - 48);
  Serial.print("Data: ");Serial.println(*incomingReadings.light - 48);
  Serial.print("Data: ");Serial.println(*incomingReadings.distance - 48);
  Serial.print("Data: ");Serial.println(*incomingReadings.pump - 48);
}

////////////////////////////////////////////

void connect_adafruit() {
  Serial.print("Connecting to Adafruit IO");
  // connect to io.adafruit.com
  io.connect();

  // set up a message handler for the count feed.
  // the handleMessage function (defined below)
  // will be called whenever a message is
  // received from adafruit io.
  pump->onMessage(handleMessage);

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  pump->get();

}

void connect_ESPNOW() {
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

void IRAM_ATTR onTimer(){
  flag_timer = 1;
}

void setup() {

  // start the serial connection
  Serial.begin(115200);

  Serial.println("Code start");

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

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 5000000, true);
  timerAlarmEnable(My_timer); //Just Enable

  // connect_adafruit();
}

void loop() {

  // if (flag_timer) {
  //   io.wifi_disconnect();
  //   WiFi.disconnect(true);
  //   delay(200);

  //   // get the status of Trasnmitted packet
  //   esp_now_register_send_cb(OnDataSent);
    
  //   // Register for a callback function that will be called when data is received
  //   esp_now_register_recv_cb(OnDataRecv);

  //   Serial.println("We in scope 1");
  //   Serial.print("sending -> ");
  //   Serial.println(count);
  //   sprintf(outgoingReadings.pump, "%d", count);

  //   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
  //   Serial.println(success);

  //   flag_pump = 0;
  //   flag_timer = 0;  
  // connect_adafruit();    
  // }
  // io.run();
  
  // if (millis() > (lastUpdate + IO_LOOP_DELAY)) {
  //   Serial.println("We in scope 2");
  //   // temp->save(*incomingReadings.temp - 48);
  //   // humi->save(*incomingReadings.humi - 48);
  //   // soil->save(*incomingReadings.soil - 48);
  //   // light->save(*incomingReadings.light - 48);
  //   // water->save(*incomingReadings.distance - 48);
  //   // pump->save(*incomingReadings.pump - 48);
  //   lastUpdate = millis();
  // }
  
  if (flag_pump && flag_timer) {
    connect_ESPNOW();
    Serial.println("We in scope 1");
    Serial.print("sending -> ");
    Serial.println(count);
    sprintf(outgoingReadings.pump, "%d", count);
    do {
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
      // Serial.println(success);
      delay(200);
    }  while (success != "Delivery Success :)");
    flag_pump = 0;
    flag_timer = 0;
  }

  unsigned long currentMillis_2 = millis();
  if(currentMillis_2 - previousMillis_2 > interval_2)
  {
    Serial.println(flag_pump);
    Serial.println(count);
    Serial.println("We in scope 2");

    connect_adafruit();

    unsigned long currentMillis = millis();
    unsigned long previousMillis = millis();

    while (currentMillis - previousMillis <= 7000) {
      io.run();
      currentMillis = millis();
    }
  
    // temp->save(*incomingReadings.temp - 48);
    // humi->save(*incomingReadings.humi - 48);
    // soil->save(*incomingReadings.soil - 48);
    // light->save(*incomingReadings.light - 48);
    // water->save(*incomingReadings.distance - 48);
    pump->save(*incomingReadings.pump - 48);
    

    // if (count != *incomingReadings.pump - 48 && !flag_pump_changed) {
    //   count = *incomingReadings.pump - 48;
    //   // pump->save(*incomingReadings.pump - 48);      
    //   flag_pump_changed = 0;
    // }

    io.wifi_disconnect();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    connect_ESPNOW();
    delay(200);
    
    Serial.println("Adafruit IO disconnected");
    previousMillis_2 = millis();
  }
} 

// this function is called whenever feed message
// is received from Adafruit IO. it was attached to
// the feed in the setup() function above.
void handleMessage(AdafruitIO_Data *data) {

  Serial.print("received <- ");
  Serial.println(data->value());
  if (count != *data->value() - 48) {
    Serial.println("Oops change!");
    count = *data->value() - 48;
    flag_pump = 1;
  }

}
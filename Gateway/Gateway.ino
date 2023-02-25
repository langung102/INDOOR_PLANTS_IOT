/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "config.h"
#include <esp_now.h>
#include <WiFi.h>
/************************ Example Starts Here *******************************/

// this int will hold the current count for our sketch
int count = 0;
int last = 0;

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
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x35, 0xF3, 0x68};

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
  Serial.print("Data: ");Serial.println(incomingReadings.msg);
}

////////////////////////////////////////////

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

  ////////////////////////////////////////////
  // wait for serial monitor to open
  while(! Serial);

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

void loop() {

  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  if (millis() > (lastUpdate + IO_LOOP_DELAY)) {
    // save count to the feed on Adafruit IO
    Serial.print("sending -> ");
    Serial.println(count);
    soil->save(count);

    // after publishing, store the current time
    lastUpdate = millis();
  }
}

// this function is called whenever feed message
// is received from Adafruit IO. it was attached to
// the feed in the setup() function above.
void handleMessage(AdafruitIO_Data *data) {

  Serial.print("received <- ");
  Serial.println(data->value());
  count = *data->value() - 48;
  strcpy(outgoingReadings.msg, data->value());
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));

}

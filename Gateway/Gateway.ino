/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "config.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp32-hal-timer.h>
#include <string>

#define DELAY_PUSH  60000000

using namespace std;
/************************ Variables *******************************/
bool last_pump = 0, last_led = 0;
bool flag_pump = 0, flag_led = 0, flag_sensor = 0, flag_auto = 0;
bool volatile flag_timer = 0;
hw_timer_t *timer_espnow = NULL;
unsigned long lastUpdate = 0;

// set up the feed
AdafruitIO_Feed *pump = io.feed("pump");
AdafruitIO_Feed *temp = io.feed("temp");
AdafruitIO_Feed *humi = io.feed("humi");
AdafruitIO_Feed *soil = io.feed("soil");
AdafruitIO_Feed *water = io.feed("water");
AdafruitIO_Feed *light = io.feed("light");
AdafruitIO_Feed *led = io.feed("led");
AdafruitIO_Feed *auto_param = io.feed("auto");
////////////////////////////////////////////

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x33, 0x58, 0x10};


//Structure example to send data
//Must match the receiver structure

typedef struct auto_struct {
  uint16_t temp[2];
  uint16_t humi[2];
  uint16_t soil[2];
  bool is_auto;
} auto_struct;

typedef struct struct_message {
  uint8_t id;
  uint16_t temp;
  uint16_t humi;
  uint16_t soil;
  uint16_t light;
  uint16_t distance;
  auto_struct condition;
  bool led;
  bool pump;
} struct_message;

auto_struct last_condition;

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
  Serial.print("Id: ");Serial.println(incomingReadings.id);
  Serial.print("Temp: ");Serial.println(incomingReadings.temp);
  Serial.print("Humi: ");Serial.println(incomingReadings.humi);
  Serial.print("Soil: ");Serial.println(incomingReadings.soil);
  Serial.print("Light: ");Serial.println(incomingReadings.light);
  Serial.print("Distance: ");Serial.println(incomingReadings.distance);
  Serial.print("Pump: ");Serial.println(incomingReadings.pump);
  Serial.print("Led: ");Serial.println(incomingReadings.led);
}

////////////////////////////////////////////

void connect_adafruit() {
  Serial.print("Connecting to Adafruit IO");
  // connect to io.adafruit.com
  io.connect();

  // set up a message handler for the last_pumpfeed.
  // the handleMessage function (defined below)
  // will be called whenever a message is
  // received from adafruit io.
  pump->onMessage(handle_pump);
  led->onMessage(handle_led);

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  pump->get();
  led->get();
  auto_param->get();

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

void IRAM_ATTR onTimer1() {
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
  for (int ii = 0; ii < 6; ++ii) {
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

  timer_espnow = timerBegin(0, 80, true);
  timerAttachInterrupt(timer_espnow, &onTimer1, true);
  timerAlarmWrite(timer_espnow, DELAY_PUSH, true); //Send data every 1 minutes
  timerAlarmEnable(timer_espnow); //Just Enable

  flag_timer = 1;

  last_condition.temp[0] = 0;
  last_condition.temp[1] = 0;
  last_condition.humi[0] = 0;
  last_condition.humi[1] = 0;
  last_condition.soil[0] = 0;
  last_condition.soil[1] = 0;
}

void loop() {
  if (flag_timer || flag_pump || flag_led || flag_auto) {
    Serial.println("We're in ESPNOW scope");

    timerAlarmDisable(timer_espnow);
    io.wifi_disconnect();
    WiFi.disconnect(true);
    connect_ESPNOW();
    delay(200);

    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);

    Serial.print("sending pump-> ");
    Serial.println(last_pump);
    outgoingReadings.pump = last_pump;

    Serial.print("sending led-> ");
    Serial.println(last_led);
    outgoingReadings.led = last_led;

    Serial.print("sending condition-> ");
    outgoingReadings.condition = last_condition;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
    Serial.println(success);

    if (!flag_pump && !flag_led) {
      unsigned long currentMillis = millis();
      unsigned long previousMillis = millis();

      while (currentMillis - previousMillis <= 1500) {
        currentMillis = millis();
      }

      if (last_pump != incomingReadings.pump) {
        last_pump = incomingReadings.pump;
      }

      if (last_led != incomingReadings.led) {
        last_led = incomingReadings.led;
      }
      
      flag_sensor = 1;
    }
    
    connect_adafruit();    
    timerAlarmWrite(timer_espnow, DELAY_PUSH, true);  
    timerAlarmEnable(timer_espnow); //Just Enable

    flag_pump = 0;
    flag_led = 0;
    flag_timer = 0;  
  }

  io.run();
  
  if (flag_sensor) {
    Serial.println("We're in IO Adafruit scope");

    Serial.println(last_pump);
    Serial.println(flag_pump || flag_led);
    Serial.println(flag_sensor);
    Serial.println(flag_timer);

    temp->save(incomingReadings.temp);
    humi->save(incomingReadings.humi);
    soil->save(incomingReadings.soil);
    light->save(incomingReadings.light);
    water->save(incomingReadings.distance);
    pump->save(incomingReadings.pump);
    led->save(incomingReadings.led);

    flag_sensor = 0;
  }
} 

// this function is called whenever feed message
// is received from Adafruit IO. it was attached to
// the feed in the setup() function above.

void handle_pump(AdafruitIO_Data *data) {
  Serial.print("received pump <- ");
  Serial.println(data->value());
  if (last_pump != (bool) (*data->value() - 48)) {
    Serial.println("Oops pump change!");
    last_pump= (bool) (*data->value() - 48);
    flag_pump = 1;
  }
}

void handle_led(AdafruitIO_Data *data) {
  Serial.print("received led <- ");
  Serial.println(data->value());
  if (last_led != (bool) (*data->value() - 48)) {
    Serial.println("Oops led change!");
    last_led = (bool) (*data->value() - 48);
    flag_led = 1;
  }
}

void handle_auto_param(AdafruitIO_Data *data) {
  auto_struct tmp;
  char temp[8], humi[8], soil[8];
  int t[2], h[2], s[2];

  sprintf(temp, "%s", strtok(data->value(), ";"));
  sprintf(humi, "%s", strtok(NULL, ";"));
  sprintf(soil, "%s", strtok(NULL, ";"));
   
  t[0] = atoi(strtok(temp, "-"));
  t[1] = atoi(strtok(NULL, "-"));
    
  h[0] = atoi(strtok(humi, "-"));
  h[1] = atoi(strtok(NULL, "-"));
    
  s[0] = atoi(strtok(soil, "-"));
  s[1] = atoi(strtok(NULL, "-"));

  if (   t[0] != last_condition.temp[0] || t[1] != last_condition.temp[1]
      || h[0] != last_condition.humi[0] || h[1] != last_condition.humi[1] 
      || s[0] != last_condition.soil[0] || s[1] != last_condition.soil[1]   ) {
    Serial.println("Oops condition change!");
    last_condition.temp[0] = t[0];
    last_condition.temp[1] = t[1];

    last_condition.humi[0] = h[0];
    last_condition.humi[1] = h[1];

    last_condition.soil[0] = s[0];
    last_condition.soil[1] = s[1];

    flag_auto = 1;
  }
  
  Serial.print("received param <- ");
  Serial.println(data->value());
}
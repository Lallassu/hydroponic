/*
 *
 * Hydroponic automation
 *
 * http://github.com/lallassu/hydroponic
 */
#include <WiFi.h>
#include <WiFiClientSecure.h>

// Required for camera OV2640
#include "OV2640Streamer.h"
#include "OV2640.h"

// Requires for temperature sensor DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>



///////////////////////////////////////////////////////////////
// Definitions of PINs etc.
///////////////////////////////////////////////////////////////
// ESP32 Pins
#define RELAY_FAN_PIN 27
#define RELAY_CAM_PIN 28
#define RELAY_PUMP_PIN 29
#define TEMPERATURE_1_PIN 4
#define TEMPERATURE_2_PIN 5

// WiFi details
#define SSID ""
#define PASSWORD ""

///////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////
OV2640 cam;
//WebServer server(80);
//WiFiServer rtspServer(8554);
WiFiClient client;
OneWire oneWire1(TEMPERATURE_1_PIN);
OneWire oneWire2(TEMPERATURE_2_PIN);
DallasTemperature sensors1(&oneWire1);
DallasTemperature sensors2(&oneWire2);

void(* resetFunc) (void) = 0; //declare reset function @ address 0

///////////////////////////////////////////////////////////////
// Setup 
///////////////////////////////////////////////////////////////
void setup() {

  // initialize digital pin as an output.
  Serial.begin(115200);
  while (!Serial)
  {
      ;
  }
  Serial.print(F("Setup..."));

  // Start the DS18B20 sensors
  sensors1.begin();
  sensors2.begin();

  Serial.printf("Camera init: %d\n", cam.init(esp32cam_aithinker_config));

  StartWIFI();

  pinMode(RELAY_FAN_PIN, OUTPUT);
  pinMode(RELAY_CAM_PIN, OUTPUT);
  pinMode(RELAY_PUMP_PIN, OUTPUT);
  Serial.println(F("Complete."));
}

///////////////////////////////////////////////////////////////
// main loop
///////////////////////////////////////////////////////////////
void loop() {
    // TBD
}

///////////////////////////////////////////////////////////////
// RelayOn to turn on a relay with given ID
///////////////////////////////////////////////////////////////
void RelayOn(int r) {
    Serial.printf("ON relay: %d\n", r);
    digitalWrite(r, HIGH);
}

///////////////////////////////////////////////////////////////
// RelayOnff to turn off a relay with given ID
///////////////////////////////////////////////////////////////
void RelayOff(int r) {
    Serial.printf("OFF relay: %d\n", r);
    digitalWrite(r, LOW);
}

///////////////////////////////////////////////////////////////
// StartWIFI starts up the wifi connection
///////////////////////////////////////////////////////////////
bool StartWIFI()
{
  Serial.println("Connecting to WIFI");
  Serial.println("");

  WiFi.disconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  unsigned long startMills = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    unsigned long cMills = millis();

    delay(500);

    if (millis() - startMills > 60000 * 3) {
      Serial.println("Could not connect to WIFI. Aborting");
      Serial.println("rebooting");
      resetFunc();
      return false;
    }
  }

  delay(1500);

  IPAddress ip;
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println(ip);

  return true;
}

///////////////////////////////////////////////////////////////
// Return the value of the given sensor
///////////////////////////////////////////////////////////////
float ReadTemperature(int sensor) {
  if (sensor == 1) {
      sensors1.requestTemperatures(); 
      return sensors1.getTempCByIndex(0);
  } else if (sensor == 2) {
      sensors2.requestTemperatures(); 
      return sensors1.getTempCByIndex(0);
  }

  return 0;
}

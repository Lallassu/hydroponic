/*
 *
 * Hydroponic automation
 *
 * http://github.com/lallassu/hydroponic
 */
#include <WiFi.h>
#include <WiFiClientSecure.h>

// Webserver
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Requires for temperature sensor DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// MQTT
#include <PubSubClient.h>

///////////////////////////////////////////////////////////////
// Definitions of PINs etc.
///////////////////////////////////////////////////////////////
// ESP32 Pins
#define RELAY_FAN_PIN 25
#define RELAY_PUMP_PIN 26
#define RELAY_CAM_PIN 27
#define TEMPERATURE_1_PIN 4
#define TEMPERATURE_2_PIN 5

// WiFi details
#define SSID "__test__"
#define PASSWORD "__test__"

// MQTT
#define MQTT_HOST "192.168.1.4"
#define MQTT_PORT 188
#define MQTT_USER "__test__"
#define MQTT_PASS "__test__"

// Temperature update on MQTT
#define TEMPERATURE_UPDATE_DELAY_MS 30000

///////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////
WiFiClient client;
OneWire oneWire1(TEMPERATURE_1_PIN);
OneWire oneWire2(TEMPERATURE_2_PIN);
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);
AsyncWebServer server(80);
PubSubClient mqttClient(client);

void(* resetFunc) (void) = 0; //declare reset function @ address 0

///////////////////////////////////////////////////////////////
// Setup 
///////////////////////////////////////////////////////////////
void setup() {

  // initialize digital pin as an output.
  Serial.begin(115200);
  while (!Serial) { ; }

  StartWIFI();
  Serial.print("Using IP address:");
  Serial.println(WiFi.localIP());

  // Start the DS18B20 sensors
  sensor1.begin();
  sensor2.begin();

  Serial.print(F("Setup..."));

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  pinMode(RELAY_FAN_PIN, OUTPUT);
  pinMode(RELAY_CAM_PIN, OUTPUT);
  pinMode(RELAY_PUMP_PIN, OUTPUT);

  // The relay uses high = off (in my case)
  digitalWrite(RELAY_PUMP_PIN, HIGH);
  digitalWrite(RELAY_CAM_PIN, HIGH);
  digitalWrite(RELAY_FAN_PIN, HIGH);

  // Create our async handlers.
  // https://www.home-assistant.io/integrations/switch.rest/
  server.on("/fan/on", HTTP_GET, [](AsyncWebServerRequest *req) {
    RelayOn(RELAY_FAN_PIN);
    req->send(200);
  });

  server.on("/fan/off", HTTP_GET, [](AsyncWebServerRequest *req) {
    RelayOff(RELAY_FAN_PIN);
    req->send(200);
  });

  server.on("/cam/on", HTTP_GET, [](AsyncWebServerRequest *req) {
    RelayOn(RELAY_CAM_PIN);
    req->send(200);
  });

  server.on("/cam/off", HTTP_GET, [](AsyncWebServerRequest *req) {
    RelayOff(RELAY_CAM_PIN);
    req->send(200);
  });

  server.on("/pump/on", HTTP_GET, [](AsyncWebServerRequest *req) {
    RelayOn(RELAY_PUMP_PIN);
    req->send(200);
  });

  server.on("/pump/off", HTTP_GET, [](AsyncWebServerRequest *req) {
    RelayOff(RELAY_PUMP_PIN);
    req->send(200);
  });

  Serial.println(F("Complete, starting webserver."));
  // Start the webserver
  server.begin();
}

///////////////////////////////////////////////////////////////
// main loop
///////////////////////////////////////////////////////////////
void loop() {
    if (!mqttClient.connected()) {
        StartMQTT();
    }
    mqttClient.loop();

    float temp = 0;
    char tempStr[8];
    temp = ReadTemperature(sensor1);
    dtostrf(temp, 1,2, tempStr);
    mqttClient.publish("hydroponic/temperature1", tempStr);

    temp = ReadTemperature(sensor2);
    dtostrf(temp, 1,2, tempStr);
    mqttClient.publish("hydroponic/temperature1", tempStr);

    delay(TEMPERATURE_UPDATE_DELAY_MS);
}

///////////////////////////////////////////////////////////////
// RelayOn to turn on a relay with given ID
///////////////////////////////////////////////////////////////
void RelayOn(int r) {
    Serial.printf("ON relay: %d\n", r);
    digitalWrite(r, LOW);
}

///////////////////////////////////////////////////////////////
// RelayOnff to turn off a relay with given ID
///////////////////////////////////////////////////////////////
void RelayOff(int r) {
    Serial.printf("OFF relay: %d\n", r);
    digitalWrite(r, HIGH);
}

///////////////////////////////////////////////////////////////
// StartMQTT starts up the mqtt connection
///////////////////////////////////////////////////////////////
void StartMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqttClient.connect("hydroponic", MQTT_USER, MQTT_PASS)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, state:");
            Serial.print(mqttClient.state());
            Serial.println(" retry in 5 seconds...");
            delay(5000);
        }
    }
}

///////////////////////////////////////////////////////////////
// StartWIFI starts up the wifi connection
///////////////////////////////////////////////////////////////
void StartWIFI() {
  Serial.println("Connecting to WIFI");
  Serial.println("");

  WiFi.disconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  unsigned long startMills = millis();

  while (WiFi.status() != WL_CONNECTED) {
    unsigned long cMills = millis();

    delay(500);

    if (millis() - startMills > 60000 * 3) {
      Serial.println("Could not connect to WIFI. Aborting");
      Serial.println("rebooting");
      resetFunc();
      return;
    }
  }

  delay(1500);

  IPAddress ip;
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println(ip);
}

///////////////////////////////////////////////////////////////
// Return the value of the given sensor
///////////////////////////////////////////////////////////////
float ReadTemperature(DallasTemperature sensor) {
    sensor.requestTemperatures(); 
    return sensor.getTempCByIndex(0);
}


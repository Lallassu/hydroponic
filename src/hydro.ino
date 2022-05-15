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
#include <ArduinoJson.h>
#include <AsyncJson.h>

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
#define SSID "__ssid__"
#define PASSWORD "__ssid_pass__"

// MQTT
#define MQTT_HOST "192.168.1.4"
#define MQTT_PORT 1883
#define MQTT_USER "__mqtt_user__"
#define MQTT_PASS "__mqtt_pass__"

// Temperature update interval on MQTT
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
StaticJsonDocument<250> jsonDocument;
char buffer[250];
void(* resetFunc) (void) = 0;

///////////////////////////////////////////////////////////////
// Setup 
///////////////////////////////////////////////////////////////
void setup() {
    // initialize digital pin as an output.
    Serial.begin(115200);
    while (!Serial) { ; }

    StartWIFI();

    // Start the DS18B20 sensors
    sensor1.begin();
    sensor2.begin();

    mqttClient.setServer(MQTT_HOST, MQTT_PORT);

    pinMode(RELAY_FAN_PIN, OUTPUT);
    pinMode(RELAY_CAM_PIN, OUTPUT);
    pinMode(RELAY_PUMP_PIN, OUTPUT);

    // The relay uses high = off (in my case)
    digitalWrite(RELAY_PUMP_PIN, HIGH);
    digitalWrite(RELAY_CAM_PIN, HIGH);
    digitalWrite(RELAY_FAN_PIN, HIGH);

    // https://www.home-assistant.io/integrations/switch.rest/

    // FAN
    server.addHandler(new AsyncCallbackJsonWebHandler("/fan", [](AsyncWebServerRequest *request, JsonVariant &json) {
            HandleRequest(json, request, RELAY_FAN_PIN);
            request->send(200, "application/json", buffer);
    }));

    server.on("/fan", HTTP_GET, [](AsyncWebServerRequest *req) {
            PinStateToJSON(RELAY_FAN_PIN);
            req->send(200, "application/json", buffer);
    });

    // CAM
    server.addHandler(new AsyncCallbackJsonWebHandler("/cam", [](AsyncWebServerRequest *request, JsonVariant &json) {
            HandleRequest(json, request, RELAY_CAM_PIN);
            request->send(200, "application/json", buffer);
    }));

    server.on("/cam", HTTP_GET, [](AsyncWebServerRequest *req) {
            PinStateToJSON(RELAY_CAM_PIN);
            req->send(200, "application/json", buffer);
    });

    // PUMP
    server.addHandler(new AsyncCallbackJsonWebHandler("/pump", [](AsyncWebServerRequest *request, JsonVariant &json) {
            HandleRequest(json, request, RELAY_PUMP_PIN);
            request->send(200, "application/json", buffer);
    }));

    server.on("/pump", HTTP_GET, [](AsyncWebServerRequest *req) {
            PinStateToJSON(RELAY_PUMP_PIN);
            req->send(200, "application/json", buffer);
    });

    Serial.print("Starting webserver on: ");
    Serial.println(WiFi.localIP());

    server.begin();
}

///////////////////////////////////////////////////////////////
// read state of a pin and write json response
///////////////////////////////////////////////////////////////
void PinStateToJSON(int pin) {
    jsonDocument.clear();  

    // Read the output register to get the last state of the pin
    gpio_num_t p = (gpio_num_t)(pin & 0x1F);
    if ((GPIO_REG_READ(GPIO_OUT_REG) >> p) & 1U) {
        jsonDocument["is_active"] = "false";
    } else {
        jsonDocument["is_active"] = "true";
    }
    serializeJson(jsonDocument, buffer);
}

///////////////////////////////////////////////////////////////
// Handle a REST request for on/off of relays
///////////////////////////////////////////////////////////////
void HandleRequest(JsonVariant &json, AsyncWebServerRequest *req, int type) {  
    jsonDocument.clear();  
    JsonObject jsonObj = json.as<JsonObject>();
    void (*fptr)(int);

    if (jsonObj["active"] == "true") {
        fptr = &RelayOn;
        jsonDocument["is_active"] = "true";
    } else if (jsonObj["active"] == "false") {
        fptr = &RelayOff;
        jsonDocument["is_active"] = "false";
    }

    (*fptr)(type);

    serializeJson(jsonDocument, buffer);
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
    mqttClient.publish("hydroponic/temperature2", tempStr);

    delay(TEMPERATURE_UPDATE_DELAY_MS);
}

///////////////////////////////////////////////////////////////
// RelayOn to turn on a relay with given ID
///////////////////////////////////////////////////////////////
void RelayOn(int r) {
    digitalWrite(r, LOW);
}

///////////////////////////////////////////////////////////////
// RelayOnff to turn off a relay with given ID
///////////////////////////////////////////////////////////////
void RelayOff(int r) {
    digitalWrite(r, HIGH);
}

///////////////////////////////////////////////////////////////
// StartMQTT starts up the mqtt connection
///////////////////////////////////////////////////////////////
void StartMQTT() {
    while (!mqttClient.connected()) {
        // Attempt to connect
        if (!mqttClient.connect("hydroponic", MQTT_USER, MQTT_PASS)) {
            Serial.print("mqtt failed, state:");
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
}

///////////////////////////////////////////////////////////////
// Return the value of the given sensor
///////////////////////////////////////////////////////////////
float ReadTemperature(DallasTemperature sensor) {
    sensor.requestTemperatures(); 
    return sensor.getTempCByIndex(0);
}


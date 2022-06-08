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

// BH1750 light sensor
#include <Wire.h>
#include <BH1750.h>

// DHT11 temp/humidity sensor
#include "DHT.h"

// MQTT
#include <PubSubClient.h>

// Watchdog
#include <esp_task_wdt.h>

///////////////////////////////////////////////////////////////
// Definitions of PINs etc.
///////////////////////////////////////////////////////////////
// ESP32 Pins
#define RELAY_FAN_PIN 25
#define RELAY_PUMP_PIN 26
#define RELAY_CAM_PIN 27
#define TEMPERATURE_1_PIN 4
#define TEMPERATURE_2_PIN 5

// Humid/Temp sensor DHT11
#define DHTTYPE DHT11
#define DHTPIN 15

// WiFi details
#define SSID "__ssid__"
#define PASSWORD "__ssid_pass__"

// MQTT
#define MQTT_HOST "192.168.1.4"
#define MQTT_PORT 1883
#define MQTT_USER "__mqtt_user__"
#define MQTT_PASS "__mqtt_pass__"

// Sensor update interval on MQTT
#define SENSOR_UPDATE_DELAY_MS 30000
#define SAMPLES 3
#define SENSOR_DELAY 100

// Watchdog timeout
#define WDT_TIMEOUT 20

///////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////
WiFiClient client;
BH1750 lightMeter;
OneWire oneWire1(TEMPERATURE_1_PIN);
OneWire oneWire2(TEMPERATURE_2_PIN);
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);
AsyncWebServer server(80);
PubSubClient mqttClient(client);
StaticJsonDocument<250> jsonDocument;
DHT dht(DHTPIN, DHTTYPE);

char buffer[250];
void(* resetFunc) (void) = 0;

///////////////////////////////////////////////////////////////
// Setup 
///////////////////////////////////////////////////////////////
void setup() {
    // initialize digital pin as an output.
    Serial.begin(115200);
    while (!Serial) { ; }
    Serial.println("Booting...");

    StartWIFI();

    // Start the DS18B20 sensors
    sensor1.begin();
    sensor2.begin();

    // I2C bus init
    Wire.begin();

    // DHT11 temp/humid sensor
    dht.begin();

    // Light sensor
    lightMeter.begin();

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

    // Enable watchdog and panic if not reset within timeout
    esp_task_wdt_init(WDT_TIMEOUT, true); 
    // Add this thread to wdt
    esp_task_wdt_add(NULL); 
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
char tempStr[8];
float temp1 = 0;
float temp2 = 0;
float lux = 0;
float heatIndex = 0;
float temp3 = 0;
float humid = 0;
float tmp = 0;
float sTemp1 = 0;
float sTemp2 = 0;
float sTemp3 = 0;
float sLux = 0;
float sHumid = 0;
int last = millis();

void loop() {
    delay(1000);
    esp_task_wdt_reset();
    
    // Make sure we are connected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to wifi(not connected)...");
        StartWIFI();
    }

    if (!mqttClient.connected()) {
        Serial.println("Connecting to mqtt...");
        StartMQTT();
    }
    mqttClient.loop();

    // Sample and send updates every X ms
    if (millis() - last >= SENSOR_UPDATE_DELAY_MS) {
        Serial.println("Sending sensor updates...");
        last = millis();
        temp1 = 0;
        temp2 = 0;
        lux = 0;
        heatIndex = 0;
        temp3 = 0;
        humid = 0;
        tmp = 0;
        sTemp1 = 0;
        sTemp2 = 0;
        sTemp3 = 0;
        sLux = 0;
        sHumid = 0;

        for (int i = 0; i < SAMPLES; i++) {
            delay(SENSOR_DELAY);
            tmp = ReadTemperature(sensor1);
            if (tmp >= -55 && tmp <= 125) {
                temp1 += tmp;
                sTemp1++;
            }

            delay(SENSOR_DELAY);
            tmp = ReadTemperature(sensor2);
            if (tmp >= -55 && tmp <= 125) {
                temp2 += tmp;
                sTemp2++;
            }

            delay(SENSOR_DELAY);
            tmp = lightMeter.readLightLevel();
            if (tmp >= 0 && tmp <= 65535) {
                lux += tmp;
                sLux++;
            }

            delay(SENSOR_DELAY);
            tmp = dht.readHumidity();
            // DHT11 actually only have 20 - 90
            if (tmp >= 0 && tmp <= 100) {
                humid += tmp;
                sHumid++;
            }

            delay(SENSOR_DELAY);
            tmp = dht.readTemperature();
            if (tmp >= 0 && tmp <= 50) {
                temp3 += tmp;
                sTemp3++;
            }
        }

        // Avg of the samples
        if (sTemp1 > 0) {
            temp1 /= sTemp1;
            dtostrf(temp1, 1,2, tempStr);
            mqttClient.publish("hydroponic/temperature1", tempStr);
        }

        if (sTemp2 > 0) {
            temp2 /= sTemp2;
            dtostrf(temp2, 1,2, tempStr);
            mqttClient.publish("hydroponic/temperature2", tempStr);
        }

        if (sLux > 0) {
            lux /= sLux;
            dtostrf(lux, 1,2, tempStr);
            mqttClient.publish("hydroponic/light1", tempStr);
        }

        if (sTemp3 > 0) {
            temp3 /= sTemp3;
            dtostrf(temp3, 1,2, tempStr);
            mqttClient.publish("hydroponic/temperature3", tempStr);
        }
        if (sHumid > 0){
            humid /= sHumid;
            dtostrf(humid, 1,2, tempStr);
            mqttClient.publish("hydroponic/humidity1", tempStr);
        }

        // Heat index (temp combiend with humidity = heat index)
        if (sHumid > 0 && sTemp3 > 0) {
            heatIndex = dht.computeHeatIndex(temp3, humid, false);
            dtostrf(heatIndex, 1,2, tempStr);
            mqttClient.publish("hydroponic/heatindex1", tempStr);
        }
        Serial.println("Done sending sensor updates...");
    }
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
    unsigned long startMills = millis();
    while (!mqttClient.connected()) {
        Serial.println("Connecting to mqtt...");
        // Attempt to connect
        if (!mqttClient.connect("hydroponic", MQTT_USER, MQTT_PASS)) {
            Serial.print("mqtt failed, state:");
            Serial.print(mqttClient.state());
            Serial.println(" retry in 3 seconds...");
            delay(2000);
        }
        if (millis() - startMills > 10000) {
            Serial.println("Could not connect to mqtt. Aborting");
            Serial.println("rebooting");
            resetFunc();
            return;
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

        if (millis() - startMills > 10000) {
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


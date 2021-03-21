#include "config.h"
#include "ColorLed.h"
#include "NeoPixelRing.h"
#include <pt.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>

#define NEO_PIXEL_PIN 12
#define THREAD1_WAIT 225
#define THREAD2_WAIT 225

//==============================================================================
// Globals

/**
 * Threads
 */
static struct pt thread1;
static struct pt thread2;

/**
 * Ranbow status
 */
static bool rainbowRunning = false;

/**
 *  Adafruit NeoPixel object
 *
 *  Argument 1 = Number of pixels in NeoPixel strip
 *  Argument 2 = Arduino pin number (most are valid)
 *  Argument 3 = Pixel type flags, add together as needed:
 *      NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
 *      NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
 *      NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
 *      NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
 *      NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
 */
Adafruit_NeoPixel neoPixel(12, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

/**
 *  My implementation of the neopixel ring in object form.
 */
NeoPixelRing ring(&neoPixel);

/**
 *  ESP8266 WiFiClient class to connect to the MQTT server
 */
WiFiClient client;

/**
 *  MQTT CLient connecting to my home mqtt broker
 */
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT);

/**
 *  MQTT Client subscrtiptions 
 */
Adafruit_MQTT_Subscribe setColor       = Adafruit_MQTT_Subscribe(&mqtt, SUB_SET_COLOR);
Adafruit_MQTT_Subscribe getColorStatus = Adafruit_MQTT_Subscribe(&mqtt, SUB_GET_COLOR_STATUS);
Adafruit_MQTT_Subscribe rainbow        = Adafruit_MQTT_Subscribe(&mqtt, SUB_RAINBOW);

//==============================================================================
// Functions

static int processMqtt(struct pt *pt)
{
    static unsigned long now = 0;
    static unsigned long lastReconnectAttempt = 0;
    static unsigned long lastProcessPacketsAttempt = 0;
    
    PT_BEGIN(pt);
    while(1) {
        int8_t connectionResponse;
        lastProcessPacketsAttempt = millis();
        PT_WAIT_UNTIL(pt, (millis() - lastProcessPacketsAttempt) > THREAD1_WAIT);
        
        if (mqtt.connected()) {
            mqtt.processPackets(2000);
        } else {
            now = millis();
            if ((now - lastReconnectAttempt) > 2500) {
                lastReconnectAttempt = now;
                if ((connectionResponse = mqtt.connect()) != 0) {
                    mqtt.disconnect();  

                    #ifdef DEBUG
                        Serial.print(F("Error: "));
                        Serial.println(mqtt.connectErrorString(connectionResponse));
                    #endif
                } else {
                    lastReconnectAttempt = 0;
                }
            }
        }

        lastProcessPacketsAttempt = millis();
        PT_WAIT_UNTIL(pt, (millis() - lastProcessPacketsAttempt) > THREAD1_WAIT);

        if(!mqtt.ping()) {
            mqtt.disconnect();
        }
    }
    PT_END(pt);
}

static int processRanbow(struct pt *pt)
{
    static unsigned long lastProcessRainbowAttempt = 0;
    
    PT_BEGIN(pt);
    while (rainbowRunning) {
        Serial.println("processRanbow loop()");

        lastProcessRainbowAttempt = millis();
        PT_WAIT_UNTIL(pt, rainbowRunning && (millis() - lastProcessRainbowAttempt) > THREAD2_WAIT);
        ring.fadeColor(255, 0, 0, 550);   // red
        lastProcessRainbowAttempt = millis();
        PT_WAIT_UNTIL(pt, rainbowRunning && (millis() - lastProcessRainbowAttempt) > THREAD2_WAIT);
        ring.fadeColor(255, 255, 0, 550); // yellow
        lastProcessRainbowAttempt = millis();
        PT_WAIT_UNTIL(pt, rainbowRunning && (millis() - lastProcessRainbowAttempt) > THREAD2_WAIT);
        ring.fadeColor(0, 255, 0, 550);   // green
        lastProcessRainbowAttempt = millis();
        PT_WAIT_UNTIL(pt, rainbowRunning && (millis() - lastProcessRainbowAttempt) > THREAD2_WAIT);
        ring.fadeColor(0, 255, 255, 550); // cyan
        lastProcessRainbowAttempt = millis();
        PT_WAIT_UNTIL(pt, rainbowRunning && (millis() - lastProcessRainbowAttempt) > THREAD2_WAIT);
        ring.fadeColor(0, 0, 255, 550);   // blue
        lastProcessRainbowAttempt = millis();
        PT_WAIT_UNTIL(pt, rainbowRunning && (millis() - lastProcessRainbowAttempt) > THREAD2_WAIT);
        ring.fadeColor(255, 0, 255, 550); // magenta
    }
    PT_END(pt);
}

void setColorCallback(char *data, uint16_t len)
{
    if (rainbowRunning) {
        #ifdef DEBUG
            Serial.println(F("Rainbow is currently running."));
        #endif

        return;
    }

    if (SUBSCRIPTIONDATALEN < len) {
        #ifdef DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #ifdef DEBUG
        Serial.print(F("Data: "));
        Serial.println(data);
        Serial.print(F("Length: "));
        Serial.println(len);
    #endif
    
    RGB color = {0,0,0};
    const int capacity = JSON_OBJECT_SIZE(4);
    StaticJsonDocument<capacity> doc;
    DeserializationError err = deserializeJson(doc, data);
    
    if (err) {
        #ifdef DEBUG
            Serial.print(F("deserializeJson() failed with code "));
            Serial.println(err.c_str());
        #endif

        return;
    }

    color.r = doc["r"].as<uint8_t>();
    color.g = doc["g"].as<uint8_t>();
    color.b = doc["b"].as<uint8_t>();
    int time = doc["time"];

    #ifdef DEBUG
        Serial.print(F("r: "));
        Serial.println(color.r);
        Serial.print(F("g: "));
        Serial.println(color.g);
        Serial.print(F("b: "));
        Serial.println(color.b);
        Serial.print(F("time: "));
        Serial.println(time);
    #endif

    if (time) {
        ring.fadeColor(&color, time);
    } else {
        ring.setColor(&color);
    }
}

void getColorCallback(char *data, uint16_t len)
{
    if (rainbowRunning) {
        #ifdef DEBUG
            Serial.println(F("Rainbow is currently running."));
        #endif

        return;
    }

    if (SUBSCRIPTIONDATALEN < len) {
        #ifdef DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #ifdef DEBUG
        Serial.print(F("Data: "));
        Serial.println(data);
        Serial.print(F("Length: "));
        Serial.println(len);
    #endif
    
    char output[SUBSCRIPTIONDATALEN];
    RGB color = ring.getColor();
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;

    doc["r"] = color.r;
    doc["g"] = color.g;
    doc["b"] = color.b;

    serializeJson(doc, output, sizeof(output));

    mqtt.publish(PUB_GET_COLOR_STATUS, output);
}

void rainbowCallback(char *data, uint16_t len) 
{
    Serial.println("rainbowCallback");

    if (strcmp(data, "ON") == 0 || strcmp(data, "on") == 0) {
        rainbowRunning = true;
    } else {
        rainbowRunning = false;
    }

    Serial.print("rainbowRunning: ");
    if (rainbowRunning) {
        Serial.println("true");
    } else {
        Serial.println("false");
    }
}

//==============================================================================
// Main

void setup() 
{
    Serial.begin(115200);
    delay(100);

    // init pins
    pinMode(NEO_PIXEL_PIN, OUTPUT);
    digitalWrite(NEO_PIXEL_PIN, LOW);

    // Connect to wifi
    #ifdef DEBUG
        Serial.println("");
        Serial.print(F("Connecting to "));
        Serial.print(WLAN_SSID);
    #endif

    WiFi.begin(WLAN_SSID, WLAN_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        delay(750);
        #ifdef DEBUG
            Serial.print(".");
        #endif
    }

    #ifdef DEBUG
        Serial.println(F(" Success!"));  
        Serial.print(F("IP address: "));
        Serial.println(WiFi.localIP());
    #endif

    // Setup Mqtt
    mqtt.unsubscribe(&setColor);
    mqtt.unsubscribe(&getColorStatus);

    setColor.setCallback(setColorCallback);
    mqtt.subscribe(&setColor);
    getColorStatus.setCallback(getColorCallback);
    mqtt.subscribe(&getColorStatus);
    rainbow.setCallback(rainbowCallback);
    mqtt.subscribe(&rainbow);

    // initialize neopixel
    neoPixel.begin(); // Also initializes the pinmode and state
    ring.off();       // Turn OFF all pixels  

    PT_INIT(&thread1);
    PT_INIT(&thread2);
}

void loop() 
{
    processMqtt(&thread1);
    processRanbow(&thread2);
}

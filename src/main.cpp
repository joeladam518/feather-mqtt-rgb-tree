#include "config.h"
// Library Headers
#include <WiFi.h> // ESP32 Wifi client library
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
// Custom headers
#include "helpers.h"
#include "ColorLed.h"
#include "NeoPixelRing.h"

#define NEO_PIXEL_PIN 14
#define NEO_PIXEL_COUNT 12

//==============================================================================
// Globals

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
Adafruit_NeoPixel neoPixel(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

/**
 *  My implementation of the neopixel ring in object form.
 */
NeoPixelRing ring(&neoPixel);

/**
 * The Wifi client;
 */
WiFiClient client;

/**
 *  MQTT CLient connecting to my home mqtt broker
 */
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT);

/**
 *  MQTT Client subscrtiptions
 */
Adafruit_MQTT_Subscribe setColor = Adafruit_MQTT_Subscribe(&mqtt, SUB_SET_COLOR);
Adafruit_MQTT_Subscribe getColor = Adafruit_MQTT_Subscribe(&mqtt, SUB_GET_COLOR);
//Adafruit_MQTT_Subscribe rainbow  = Adafruit_MQTT_Subscribe(&mqtt, SUB_RAINBOW);

/**
 * Task Handles
 */
static TaskHandle_t mqttTaskHandle = NULL;
static TaskHandle_t processCallbacksTaskHandle = NULL;

/**
 * Queues
 */
static QueueHandle_t mqttCallbackQueue;

/**
 * Mutexes
 */
static SemaphoreHandle_t ringMutex;

//==============================================================================
// Mqtt Call Backs

// void rainbowCallback(struct pt *pt)
// {
//     while (true) {
//         Serial.println("processRanbow loop()");
//         ring.fadeColor(255, 0, 0, 550);   // red
//         ring.fadeColor(255, 255, 0, 550); // yellow
//         ring.fadeColor(0, 255, 0, 550);   // green
//         ring.fadeColor(0, 255, 255, 550); // cyan
//         ring.fadeColor(0, 0, 255, 550);   // blue
//         ring.fadeColor(255, 0, 255, 550); // magenta
//     }
// }

void getColorCallback(char *data, uint16_t len)
{
    if (SUBSCRIPTIONDATALEN < len) {
        #ifdef DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #ifdef DEBUG
        printCallbackData(data, len);
    #endif

    char output[SUBSCRIPTIONDATALEN];
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;

    if (xSemaphoreTake(ringMutex, 0) == pdTRUE) {
        RGB color = ring.getColor();

        doc["r"] = color.r;
        doc["g"] = color.g;
        doc["b"] = color.b;

        xSemaphoreGive(ringMutex);
    }

    serializeJson(doc, output, sizeof(output));

    mqtt.publish(PUB_GET_COLOR, output);
}

void setColorCallback(char *data, uint16_t len)
{
    if (SUBSCRIPTIONDATALEN < len) {
        #ifdef DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #ifdef DEBUG
        printCallbackData(data, len);
    #endif

    RGB color = {0,0,0};
    const int capacity = JSON_OBJECT_SIZE(4);
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, data);

    if (error) {
        #ifdef DEBUG
            printDeserializeError(&error);
        #endif

        return;
    }

    color.r = doc["r"].as<uint8_t>();
    color.g = doc["g"].as<uint8_t>();
    color.b = doc["b"].as<uint8_t>();
    int time = doc["time"];

    #ifdef DEBUG
        printColorAndTime(&color, time);
    #endif

    if (xSemaphoreTake(ringMutex, 0) == pdTRUE) {
        if (time) {
            ring.fadeColor(&color, time);
        } else {
            ring.setColor(&color);
        }

        xSemaphoreGive(ringMutex);
    }
}

void mqttConnect()
{
    if (mqtt.connected()) {
        return;
    }

    vTaskSuspend(processCallbacksTaskHandle);
    Serial.print(F("Connecting to MQTT... "));

    int8_t ret;
    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println(F("Retrying MQTT connection in 5 seconds..."));

        mqtt.disconnect();
        vTaskDelay(2500 / portTICK_PERIOD_MS);  // wait 5 seconds

        retries--;
        if (retries == 0) {
            Serial.println(F("Could not connetect to the mqtt broker. Ran out of retries..."));
            // basically die and wait for WDT to reset me
            while (1);
        }
    }

    Serial.println("MQTT Connected!");
    vTaskResume(processCallbacksTaskHandle);
}

//==============================================================================
// Tasks

void mqttTask(void *parameter)
{
    while (1) {
        mqttConnect();

        // Should this only push the call back to a process queue?
        Adafruit_MQTT_Subscribe *subscription;
        while ((subscription = mqtt.readSubscription(5000))) {
            if (subscription == &getColor || subscription == &setColor) {
                char *newData = copyCallbackData((char *)subscription->lastread);

                if (newData == NULL) {
                    continue;
                }

                MqttCallbackStruct callbackStruct = {
                    subscription->callback_buffer,
                    newData,
                    subscription->datalen
                };

                xQueueSend(mqttCallbackQueue, (void *)&callbackStruct, 10);
            }
        }
    }
}

void processCallbacksTask(void *parameter)
{
    MqttCallbackStruct callbackStruct;

    while (1) {
        if (xQueueReceive(mqttCallbackQueue, (void *)&callbackStruct, 5) == pdFALSE) {
            continue;
        }

        callbackStruct.callback(callbackStruct.data, callbackStruct.length);

        free(callbackStruct.data);
        callbackStruct.data = NULL;
    }
}

//==============================================================================
// Main

// NOTE: Set up is excuted on core #1
void setup()
{
    Serial.begin(115200);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Connect to wifi
    #ifdef DEBUG
        Serial.println("");
        Serial.print(F("Connecting to "));
        Serial.print(WLAN_SSID);
    #endif

    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500/ portTICK_PERIOD_MS);
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
    mqtt.unsubscribe(&getColor);
    mqtt.unsubscribe(&setColor);

    getColor.setCallback(getColorCallback);
    mqtt.subscribe(&getColor);
    setColor.setCallback(setColorCallback);
    mqtt.subscribe(&setColor);
    //rainbow.setCallback(rainbowCallback);
    //mqtt.subscribe(&rainbow);

    // set up mutexes
    ringMutex = xSemaphoreCreateMutex();

    // set up queues
    mqttCallbackQueue = xQueueCreate(5, sizeof(MqttCallbackStruct));

    // Set up Tasks
    xTaskCreatePinnedToCore(
        mqttTask,        // Function to be called
        "Mqtt Task",     // Name of task
        2048,            // Stack size (bytes in ESP32, words in FreeRTOS)
        NULL,            // Parameter to pass to function
        1,               // Task priority (0 to configMAX_PRIORITIES - 1)
        &mqttTaskHandle, // Ask handle
        PRO_CPU_NUM      // Run on one core
    );
    vTaskSuspend(mqttTaskHandle);

    xTaskCreatePinnedToCore(
        processCallbacksTask,        // Function to be called
        "Process Callbacks Task",    // Name of task
        2048,                        // Stack size (bytes in ESP32, words in FreeRTOS)
        NULL,                        // Parameter to pass to function
        1,                           // Task priority (0 to configMAX_PRIORITIES - 1)
        &processCallbacksTaskHandle, // Ask handle
        APP_CPU_NUM                  // Run on one core
    );

    // initialize neopixel
    if (xSemaphoreTake(ringMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
        ring.begin(); // Also initializes the pinmode and state
        Serial.println("Line 272");
        ring.off();   // Turn OFF all pixels

        xSemaphoreGive(ringMutex);
    }

    vTaskResume(mqttTaskHandle);
    vTaskDelete(NULL);
}

// NOTE: Loop is excuted on core #1
void loop() {}

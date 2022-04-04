#include "config.h"
// Library Headers
#include <WiFi.h> // ESP32 Wifi client library
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
// Custom Headers
#include "main.h"
#include "led.h"
#include "NeoPixelRing.h"

#define NEO_PIXEL_PIN 14
#define NEO_PIXEL_COUNT 16
#define READ_SUBSCRIPTION_TIMEOUT 2000

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

// work with the neo pixels in object form
NeoPixelRing ring(&neoPixel);

// Wifi client
WiFiClient client;

// Mqtt client
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT);

// Mqtt client subscriptons
Adafruit_MQTT_Subscribe setColorSub = Adafruit_MQTT_Subscribe(&mqtt, SUB_SET_COLOR);
Adafruit_MQTT_Subscribe getColorSub = Adafruit_MQTT_Subscribe(&mqtt, SUB_GET_COLOR);

// Task handles
static TaskHandle_t processMqttTaskHandle = NULL;
static TaskHandle_t processActionsTaskHandle = NULL;
// Queues
static QueueHandle_t actionsQueue;
// Mutexes
static SemaphoreHandle_t ringMutex;

//==============================================================================
// Main

// NOTE: Set up is excuted on core #1
void setup()
{
    Serial.begin(115200);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Connect to Wifi
    Serial.println("");
    Serial.print(F("Connecting to "));
    Serial.print(WLAN_SSID);

    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print(".");
    }

    Serial.println(F("Success!"));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());

    // Setup Mqtt
    mqtt.unsubscribe(&getColorSub);
    mqtt.unsubscribe(&setColorSub);

    getColorSub.setCallback(getColor);
    mqtt.subscribe(&getColorSub);
    setColorSub.setCallback(setColor);
    mqtt.subscribe(&setColorSub);

    // Configure RTOS
    actionsQueue = xQueueCreate(5, sizeof(SubscriptionAction_t));
    ringMutex = xSemaphoreCreateMutex();

    // Initialize neopixel ring
    if (xSemaphoreTake(ringMutex, 0) == pdTRUE) {
        ring.begin();
        xSemaphoreGive(ringMutex);
    }

    xTaskCreatePinnedToCore(
        processMqttTask,           // Function to be called
        "Process Mqtt",            // Name of task
        2560,                      // Stack size (bytes in ESP32, words in FreeRTOS)
        NULL,                      // Parameter to pass to function
        1,                         // Task priority (0 to configMAX_PRIORITIES - 1)
        &processMqttTaskHandle,    // Task handle
        PRO_CPU_NUM                // Run on core
    );
    vTaskSuspend(processMqttTaskHandle);

    xTaskCreatePinnedToCore(
        processActionsTask,        // Function to be called
        "Process Handlers",        // Name of task
        2560,                      // Stack size (bytes in ESP32, words in FreeRTOS)
        NULL,                      // Parameter to pass to function
        1,                         // Task priority (0 to configMAX_PRIORITIES - 1)
        &processActionsTaskHandle, // Task handle
        APP_CPU_NUM                // Run on core
    );
    vTaskSuspend(processActionsTaskHandle);

    // Start the mqtt task
    vTaskResume(processMqttTaskHandle);
    // Remove the setup() and loop() task
    vTaskDelete(NULL);
}

// NOTE: Loop is excuted on core #1
void loop() {}

//==============================================================================
// Tasks

void processActionsTask(void *parameter)
{
    SubscriptionAction_t action;

    while (1) {
        if (xQueueReceive(actionsQueue, (void *)&action, 0) == pdTRUE) {
            Serial.println(F("processActionsTask() executing callback..."));
            action.callback(action.data, action.length);
            clearAction(&action);
        }
    }
}

void processMqttTask(void *parameter)
{
    Adafruit_MQTT_Subscribe *subscription;
    SubscriptionAction action;
    uint32_t startTime = 0;
    uint32_t endTime = 0;
    uint32_t elapsed = 0;

    while (1) {
        mqttConnect();

        startTime = millis();
        endTime = 0;
        elapsed = 0;

        while (elapsed < READ_SUBSCRIPTION_TIMEOUT) {
            if ((subscription = mqtt.readSubscription(READ_SUBSCRIPTION_TIMEOUT - elapsed))) {
                setAction(&action, subscription);

                if (action.callback != NULL) {
                    Serial.println(F("Sending to queue..."));
                    xQueueSend(actionsQueue, (void *)&action, 0);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                    break;
                } else {
                    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
                        Serial.println(F(""));
                        Serial.println(F("!! Action couldn't be sent to the queue !!"));
                        Serial.println(F(""));
                    #endif
                }
            }

            endTime = millis();
            if (endTime < startTime) startTime = endTime;
            elapsed += (endTime - startTime);
        }
    }
}

//==============================================================================
// Mqtt Callbacks

void getColor(char *data, uint16_t len)
{
    if (SUBSCRIPTIONDATALEN < len) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println("getColor(): ");
        printSubscriptionCallbackData(data, len);
    #endif

    char output[SUBSCRIPTIONDATALEN];
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;

    RGB_t color = ring.getColor();

    doc["r"] = color.r;
    doc["g"] = color.g;
    doc["b"] = color.b;

    serializeJson(doc, output, sizeof(output));
    mqtt.publish(PUB_GET_COLOR, output);

    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println(F("Done!"));
    #endif
}

void setColor(char *data, uint16_t len)
{
    if (SUBSCRIPTIONDATALEN < len) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println("setColor(): ");
        printSubscriptionCallbackData(data, len);
    #endif

    const int capacity = JSON_OBJECT_SIZE(4);
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, data);

    if (error) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            printDeserializeError(&error);
        #endif

        return;
    }

    uint8_t r = doc["r"].as<uint8_t>();
    uint8_t g = doc["g"].as<uint8_t>();
    uint8_t b = doc["b"].as<uint8_t>();
    int time = doc["time"].as<int>();

    if (xSemaphoreTake(ringMutex, 0) == pdTRUE) {
        if (time) {
            ring.fadeColor(r, g, b, time);
        } else {
            ring.setColor(r, g, b);
        }
        xSemaphoreGive(ringMutex);
    }

    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println(F("Done!"));
    #endif
}

//==============================================================================
// Methods

void mqttConnect()
{
    if (mqtt.connected()) {
        return;
    }

    Serial.print(F("Connecting to MQTT..."));
    vTaskSuspend(processActionsTaskHandle);

    uint8_t ret;
    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
        Serial.println(F(""));
        Serial.print(F("Mqtt connection error: "));
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println(F("Retrying MQTT connection in 5 seconds..."));

        mqtt.disconnect();
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // wait 5 seconds

        retries--;
        if (retries == 0) {
            Serial.println(F("Could not connetect to the mqtt broker. Ran out of retries..."));
            while (1); // die and wait for reset
        }
    }

    Serial.println(F("Success!"));
    vTaskResume(processActionsTaskHandle);
    vTaskDelay(250 / portTICK_PERIOD_MS);
}

void clearAction(SubscriptionAction_t *action)
{
    action->callback = NULL;
    memset(action->data, '\0', SUBSCRIPTIONDATALEN);
    action->length = 0;
}

void setAction(SubscriptionAction_t *action, Adafruit_MQTT_Subscribe *subscription)
{
    if (subscription->callback_buffer) {
        action->callback = subscription->callback_buffer;
        memset(action->data, '\0', SUBSCRIPTIONDATALEN);
        strncpy(action->data, (char *)subscription->lastread, (SUBSCRIPTIONDATALEN - 1));
        action->length = subscription->datalen;
    } else {
        clearAction(action);
    }
}

//==============================================================================
// Debug Helpers

#if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
    void printSubscriptionCallbackData(char *data, uint16_t len)
    {
        Serial.print(F("Data: "));
        Serial.println(data);
        Serial.print(F("Length: "));
        Serial.println(len);
    }

    void printDeserializeError(DeserializationError *error)
    {
        Serial.print(F("Arduino Json eserialization error: "));
        Serial.println(error->c_str());
    }
#endif

//==============================================================================

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

// work with the neo pixels in object form
NeoPixelRing ring(&neoPixel);

// Wifi client
WiFiClient client;

// Mqtt client
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT);

// Mqtt client subscriptons
Adafruit_MQTT_Subscribe setColor = Adafruit_MQTT_Subscribe(&mqtt, SUB_SET_COLOR);
Adafruit_MQTT_Subscribe getColor = Adafruit_MQTT_Subscribe(&mqtt, SUB_GET_COLOR);
//Adafruit_MQTT_Subscribe rainbow  = Adafruit_MQTT_Subscribe(&mqtt, SUB_RAINBOW);

// Task handles
static TaskHandle_t mqttTaskHandle = NULL;
static TaskHandle_t processCallbacksTaskHandle = NULL;

// Queues
static QueueHandle_t mqttCallbackQueue;

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
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
    }

    Serial.println(F(" Success!"));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());

    // Setup Mqtt
    mqtt.unsubscribe(&getColor);
    mqtt.unsubscribe(&setColor);

    getColor.setCallback(getColorCallback);
    mqtt.subscribe(&getColor);
    setColor.setCallback(setColorCallback);
    mqtt.subscribe(&setColor);
    //rainbow.setCallback(rainbowCallback);
    //mqtt.subscribe(&rainbow);

    // Configure RTOS
    ringMutex = xSemaphoreCreateMutex();
    mqttCallbackQueue = xQueueCreate(3, sizeof(CallbackBufferObject));

    xTaskCreatePinnedToCore(
        mqttTask,        // Function to be called
        "Mqtt Task",     // Name of task
        2048,            // Stack size (bytes in ESP32, words in FreeRTOS)
        NULL,            // Parameter to pass to function
        1,               // Task priority (0 to configMAX_PRIORITIES - 1)
        &mqttTaskHandle, // Task handle
        PRO_CPU_NUM      // Run on core
    );
    vTaskSuspend(mqttTaskHandle);

    xTaskCreatePinnedToCore(
        processCallbacksTask,        // Function to be called
        "Process Callbacks Task",    // Name of task
        2048,                        // Stack size (bytes in ESP32, words in FreeRTOS)
        NULL,                        // Parameter to pass to function
        1,                           // Task priority (0 to configMAX_PRIORITIES - 1)
        &processCallbacksTaskHandle, // Task handle
        APP_CPU_NUM                  // Run on core
    );

    // Initialize neopixel ring
    if (xSemaphoreTake(ringMutex, 0) == pdTRUE) {
        ring.begin();

        xSemaphoreGive(ringMutex);
    }

    vTaskResume(mqttTaskHandle);
    vTaskDelete(NULL); // Removes the setup() and loop() task
}

// NOTE: Loop is excuted on core #1
void loop() {}

//==============================================================================
// Tasks

void mqttTask(void *parameter)
{
    Adafruit_MQTT_Subscribe *subscription;

    while (1) {
        mqttConnect();

        while ((subscription = mqtt.readSubscription(2500))) {
            if (subscription == &getColor || subscription == &setColor) {
                CallbackBufferObject callbackObj;
                setCallbackObject(&callbackObj, subscription);

                if (callbackObj.callback != NULL) {
                    Serial.println(F("Sending to queue..."));
                    xQueueSend(mqttCallbackQueue, (void *)&callbackObj, 0);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                    break;
                } else {
                    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
                        Serial.println(F(""));
                        Serial.println(F("!! Callback couldn't be set to the queue... !!"));
                        Serial.println(F(""));
                    #endif
                }
            }
        }
    }
}

void processCallbacksTask(void *parameter)
{
    while (1) {
        CallbackBufferObject callbackObj;
        if (xQueueReceive(mqttCallbackQueue, (void *)&callbackObj, 0) == pdTRUE) {
            Serial.println(F("ProcessCallbacksTask: Calling callback..."));
            callbackObj.callback(callbackObj.data, callbackObj.length);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

//==============================================================================
// Mqtt Callbacks

void getColorCallback(char *data, uint16_t len)
{
    if (SUBSCRIPTIONDATALEN < len) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        printCallbackData(data, len);
    #endif

    char output[SUBSCRIPTIONDATALEN];
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;

    RGB color = ring.getColor();

    doc["r"] = color.r;
    doc["g"] = color.g;
    doc["b"] = color.b;

    serializeJson(doc, output, sizeof(output));

    mqtt.publish(PUB_GET_COLOR, output);

    Serial.println(F("Done!"));
}

void setColorCallback(char *data, uint16_t len)
{
    if (SUBSCRIPTIONDATALEN < len) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        printCallbackData(data, len);
    #endif

    RGB color = {0,0,0};
    const int capacity = JSON_OBJECT_SIZE(4);
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, data);

    if (error) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            printDeserializeError(&error);
        #endif

        return;
    }

    color.r = doc["r"].as<uint8_t>();
    color.g = doc["g"].as<uint8_t>();
    color.b = doc["b"].as<uint8_t>();
    int time = doc["time"];

    #if false && defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        printColor(&color);
        printTime(time);
    #endif

    if (xSemaphoreTake(ringMutex, 0) == pdTRUE) {
        if (time) {
            ring.fadeColor(&color, time);
        } else {
            ring.setColor(&color);
        }

        xSemaphoreGive(ringMutex);
    }

    Serial.println(F("Done!"));
}

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

//==============================================================================
// Methods

void clearCallbackData(CallbackBufferObject *callbackObj)
{
    memset(callbackObj->data, '\0', SUBSCRIPTIONDATALEN);
}

void copyCallbackData(CallbackBufferObject *callbackObj, char *data)
{
    clearCallbackData(callbackObj);
    strncpy(callbackObj->data, data, (SUBSCRIPTIONDATALEN - 1));
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
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // wait 5 seconds

        retries--;
        if (retries == 0) {
            Serial.println(F("Could not connetect to the mqtt broker. Ran out of retries..."));
            // basically die and wait for WDT to reset me
            while (1);
        }
    }

    Serial.println(F("MQTT Connected!"));
    vTaskResume(processCallbacksTaskHandle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void setCallbackObject(CallbackBufferObject *callbackObj, Adafruit_MQTT_Subscribe *subscription)
{
    if (subscription->callback_buffer) {
        callbackObj->callback = subscription->callback_buffer;
        copyCallbackData(callbackObj, (char *)subscription->lastread);
        callbackObj->length = subscription->datalen;
    } else {
        callbackObj->callback = NULL;
        clearCallbackData(callbackObj);
        callbackObj->length = 0;
    }
}

//==============================================================================
// Debug Helpers

#if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
    void printCallbackData(char *data, uint16_t len)
    {
        Serial.print(F("Data: "));
        Serial.println(data);
        Serial.print(F("Length: "));
        Serial.println(len);
    }

    void printColor(RGB *color)
    {
        Serial.print(F("r: "));
        Serial.println(color->r);
        Serial.print(F("g: "));
        Serial.println(color->g);
        Serial.print(F("b: "));
        Serial.println(color->b);
    }

    void printTime(int time)
    {
        Serial.print(F("time: "));
        Serial.println(time);
    }

    void printDeserializeError(DeserializationError *error)
    {
        Serial.print(F("Deserialization error: "));
        Serial.println(error->c_str());
    }
#endif

//==============================================================================

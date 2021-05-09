#ifndef __RGB_TREE_MAIN_H__
#define __RGB_TREE_MAIN_H__

#include "config.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include "led.h"

typedef struct CallbackBufferObject {
    SubscribeCallbackBufferType callback;
    char data[SUBSCRIPTIONDATALEN];
    uint16_t length;
} CallbackBufferObject;

// Debug Helpers
#if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
    void printCallbackData(char *data, uint16_t len);
    void printColor(RGB *color);
    void printDeserializeError(DeserializationError *error);
    void printTime(int time);
#endif

// Methods
void clearCallbackData(CallbackBufferObject *callbackObj);
void copyCallbackData(CallbackBufferObject *callbackObj, char *data);
void mqttConnect();
void setCallbackObject(CallbackBufferObject *callbackObj, Adafruit_MQTT_Subscribe *subscription);

// Mqtt Subscrtiption Callbacks
void getColorCallback(char *data, uint16_t len);
void setColorCallback(char *data, uint16_t len);
//void rainbowCallback(char *data, uint16_t len);

// Tasks
void mqttTask(void *parameter);
void processCallbacksTask(void *parameter);

#endif

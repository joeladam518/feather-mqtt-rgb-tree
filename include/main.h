#ifndef __RGB_TREE_MAIN_H__
#define __RGB_TREE_MAIN_H__

#include "config.h"
#include <Adafruit_MQTT.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include "led.h"

#define TW1_PIN 33
#define TW2_PIN 15
#define TW3_PIN 32
#define NEO_PIXEL_PIN 14
#define NEO_PIXEL_COUNT 16
#define READ_SUBSCRIPTION_TIMEOUT 2000

typedef struct SubscriptionAction {
    SubscribeCallbackBufferType callback;
    char data[SUBSCRIPTIONDATALEN];
    uint16_t length;
} SubscriptionAction_t;

// Mqtt Subscrtiption Callbacks
void getColor(char *data, uint16_t len);
void setColor(char *data, uint16_t len);
void getTwinkleLights(char *data, uint16_t len);
void setTwinkleLights(char *data, uint16_t len);

// Tasks
void processActionsTask(void *parameter);
void processInputTask(void *parameter);

// Methods
void mqttConnect();
void clearAction(SubscriptionAction_t *action);
void setAction(SubscriptionAction_t *action, Adafruit_MQTT_Subscribe *subscription);
void publishRgbStatus(void);
void publishTwinkleLightStatus(void);

// Debug Helpers
#if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
    void printSubscriptionCallbackData(char *data, uint16_t len);
    void printDeserializeError(DeserializationError *error);
#endif

#endif

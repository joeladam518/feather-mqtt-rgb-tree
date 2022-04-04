#ifndef __RGB_TREE_MAIN_H__
#define __RGB_TREE_MAIN_H__

#include "config.h"
#include <Adafruit_MQTT.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include "led.h"

typedef struct SubscriptionAction {
    SubscribeCallbackBufferType callback;
    char data[SUBSCRIPTIONDATALEN];
    uint16_t length;
} SubscriptionAction_t;

// Mqtt Subscrtiption Callbacks
void getColor(char *data, uint16_t len);
void setColor(char *data, uint16_t len);
//void rainbow(char *data, uint16_t len);

// Methods
void mqttConnect();
void clearAction(SubscriptionAction_t *action);
void setAction(SubscriptionAction_t *action, Adafruit_MQTT_Subscribe *subscription);

// Tasks
void processActionsTask(void *parameter);
void processMqttTask(void *parameter);

// Debug Helpers
#if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
    void printSubscriptionCallbackData(char *data, uint16_t len);
    void printDeserializeError(DeserializationError *error);
#endif

#endif

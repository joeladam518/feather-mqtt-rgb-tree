#ifndef __NEO_PIXEL_RING_HELPERS_H__
#define __NEO_PIXEL_RING_HELPERS_H__

#include <Arduino.h>
#include <ArduinoJson.h>
#include "ColorLed.h"

char *copyCallbackData(char *data);
void printCallbackData(char *data, uint16_t len);
void printColor(RGB *color);
void printColorAndTime(RGB *color, int time);
void printDeserializeError(DeserializationError *error);

typedef struct MqttCallbackStruct {
    SubscribeCallbackBufferType callback;
    char *data;
    uint16_t length;
} MqttCallbackStruct;

#endif

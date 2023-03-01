#ifndef GLOBALS_H
#define GLOBALS_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include <WiFi.h>
#include <mqtt_client.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "NeoPixelRing.h"

// Wifi client
extern WiFiClient wifiClient;

// Mqtt client
extern esp_mqtt_client_handle_t mqttClient;

// Adafruit NeoPixels
extern Adafruit_NeoPixel neoPixel;
extern NeoPixelRing ring;

// freertos variables
extern TaskHandle_t processShortActionsTaskHandle;
extern TaskHandle_t processLongActionsTaskHandle;
extern TaskHandle_t processInputTaskHandle;
extern QueueHandle_t shortActionsQueue;
extern QueueHandle_t longActionsQueue;
extern SemaphoreHandle_t ringMutex;

#endif

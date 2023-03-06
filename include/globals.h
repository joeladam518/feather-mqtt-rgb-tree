#ifndef __RGB_TREE_GLOBALS_H__
#define __RGB_TREE_GLOBALS_H__

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
extern TaskHandle_t mqttMTaskHandle;
extern TaskHandle_t processShortTaskHandle;
extern TaskHandle_t processLongTaskHandle;
extern QueueHandle_t shortActionQueue;
extern QueueHandle_t longActionQueue;
extern SemaphoreHandle_t ringMutex;

#endif

#ifndef __RGB_TREE_MQTT_H__
#define __RGB_TREE_MQTT_H__

#include "config.h"
#include <esp_log.h>
#include <mqtt_client.h>

#define SUBSCRIPTIONDATALEN 100
#define READ_SUBSCRIPTION_TIMEOUT 2000

static const char *MQTT_TAG = "MQTT_RGB_TREE";

typedef void (*SubscribeCallbackBufferType)(char *str, uint16_t len);

typedef enum RGB_TREE_CALLBACK_TYPE {
    UNKNOWN = 0,
    GET_COLOR = 1,
    SET_COLOR = 2,
    GET_TW = 3,
    SET_TW = 4
} rgb_tree_callback_type_t;

// Subscribe callbacks
void getColor(char *data, uint16_t len);
void setColor(char *data, uint16_t len);
void getTwinkleLights(char *data, uint16_t len);
void setTwinkleLights(char *data, uint16_t len);

typedef enum MQTT_QOS {
    QOS_AT_MOST_ONCE = 0,
    QOS_AT_LEAST_ONCE = 1,
    QOS_EXACTLY_ONCE = 2,
} mqtt_qos_t;

typedef struct SubscriptionAction {
    rgb_tree_callback_type_t callbackType;
    esp_mqtt_client_handle_t client;
    char data[SUBSCRIPTIONDATALEN];
    uint16_t dataLength;
} SubscriptionAction_t;

// Task functions
void processShortTask(void *parameter);
void processLongTask(void *parameter);

// Publish functions
void publishRgbStatus(void);
void publishTwinkleLightStatus(void);

// Other Stuff
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

#endif

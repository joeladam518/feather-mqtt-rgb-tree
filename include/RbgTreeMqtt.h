#ifndef __RGB_TREE_MQTT_H__
#define __RGB_TREE_MQTT_H__

#include <mqtt_client.h>

#define SUBSCRIPTIONDATALEN 100

static const char *TAG = "MQTT_RGB_TREE";

enum MQTT_QOS {
    QOS_AT_MOST_ONCE = 0,
    QOS_AT_LEAST_ONCE = 1,
    QOS_EXACTLY_ONCE = 2,
} mqtt_qos_t;

// Mqtt callbacks
void getColor(char *data, uint16_t len);
void setColor(char *data, uint16_t len);
void getTwinkleLights(char *data, uint16_t len);
void setTwinkleLights(char *data, uint16_t len);

// Other Stuff
esp_mqtt_client_handle_t mqtt_start(esp_mqtt_client_config_t *mqtt_config);

#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_log.h>
#include <mqtt_client.h>

#include "config.h"
#include "globals.h"
#include "led.h"
#include "NeoPixelRing.h"
#include "RbgTreeMqtt.h"

//==============================================================================
// Mqtt publish functions

void publishRgbStatus(void)
{
    char output[SUBSCRIPTIONDATALEN];
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;

    RGB_t color = {0, 0, 0};
    ring.getColor(&color);

    doc["r"] = color.r;
    doc["g"] = color.g;
    doc["b"] = color.b;

    serializeJson(doc, output, sizeof(output));
    esp_mqtt_client_publish(mqttClient, PUB_GET_COLOR, output, sizeof(output), 0, 0);
}

void publishTwinkleLightStatus(void)
{
    char output[SUBSCRIPTIONDATALEN];
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;

    doc["tw1"] = digitalRead(TW1_PIN);
    doc["tw2"] = digitalRead(TW2_PIN);
    doc["tw3"] = digitalRead(TW3_PIN);

    serializeJson(doc, output, sizeof(output));
    esp_mqtt_client_publish(mqttClient, PUB_GET_TW_LIGHTS, output, sizeof(output), 0, 0);
}

//==============================================================================
// Mqtt subscribe callbacks

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

    publishRgbStatus();

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
    uint16_t time = doc["time"].as<uint16_t>();

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

    publishRgbStatus();
}

void getTwinkleLights(char *data, uint16_t len)
{
    if (SUBSCRIPTIONDATALEN < len) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println("getTwinkleLights(): ");
        printSubscriptionCallbackData(data, len);
    #endif

    publishTwinkleLightStatus();

    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println(F("Done!"));
    #endif
}

void setTwinkleLights(char *data, uint16_t len)
{
    if (SUBSCRIPTIONDATALEN < len) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println("setTwinkleLights(): ");
        printSubscriptionCallbackData(data, len);
    #endif

    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, data);

    if (error) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            printDeserializeError(&error);
        #endif

        return;
    }


    if (doc.containsKey("tw1")) {
        uint8_t tw1 = doc["tw1"].as<uint8_t>();

        if (tw1 > 0) {
            digitalWrite(TW1_PIN, HIGH);
        } else {
            digitalWrite(TW1_PIN, LOW);
        }
    }

    if (doc.containsKey("tw2")) {
        uint8_t tw2 = doc["tw2"].as<uint8_t>();

        if (tw2 > 0) {
            digitalWrite(TW2_PIN, HIGH);
        } else {
            digitalWrite(TW2_PIN, LOW);
        }
    }

    if (doc.containsKey("tw3")) {
        uint8_t tw3 = doc["tw3"].as<uint8_t>();

        if (tw3 > 0) {
            digitalWrite(TW3_PIN, HIGH);
        } else {
            digitalWrite(TW3_PIN, LOW);
        }
    }

    publishTwinkleLightStatus();
}

//==============================================================================
// Mqtt functions

static void mqtt_unsubscribe_all(esp_mqtt_client_handle_t client)
{
    esp_mqtt_client_unsubscribe(client, SUB_GET_COLOR);
    esp_mqtt_client_unsubscribe(client, SUB_SET_COLOR);
    esp_mqtt_client_unsubscribe(client, SUB_GET_TW_LIGHTS);
    esp_mqtt_client_unsubscribe(client, SUB_SET_TW_LIGHTS);
}

static void mqtt_subsribe_all(esp_mqtt_client_handle_t client)
{
    mqtt_unsubscribe_all(client);
    esp_mqtt_client_subscribe(client, SUB_GET_COLOR, QOS_AT_MOST_ONCE);
    esp_mqtt_client_subscribe(client, SUB_SET_COLOR, QOS_AT_MOST_ONCE);
    esp_mqtt_client_subscribe(client, SUB_GET_TW_LIGHTS, QOS_AT_MOST_ONCE);
    esp_mqtt_client_subscribe(client, SUB_SET_TW_LIGHTS, QOS_AT_MOST_ONCE);
}

static void mqtt_handle_data_event(esp_mqtt_event_handle_t event)
{
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);

    if (strncmp(SUB_GET_COLOR, event->topic, event->topic_len) == 0) {
        getColor(event->data, event->data_len);
        return;
    }

    if (strncmp(SUB_SET_COLOR, event->topic, event->topic_len) == 0) {
        setColor(event->data, event->data_len);
        return;
    }

    if (strncmp(SUB_GET_TW_LIGHTS, event->topic, event->topic_len) == 0) {
        getTwinkleLights(event->data, event->data_len);
        return;
    }

    if (strncmp(SUB_GET_TW_LIGHTS, event->topic, event->topic_len) == 0) {
        getTwinkleLights(event->data, event->data_len);
        return;
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_subsribe_all(client);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            mqtt_handle_data_event(event);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                // log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                // log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                // log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

esp_mqtt_client_handle_t mqtt_start(esp_mqtt_client_config_t *mqtt_cfg)
{
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(mqtt_cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    return client;
}

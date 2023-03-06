#include "config.h"
#include "globals.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <esp_log.h>
#include <mqtt_client.h>

#include "led.h"
#include "MqttEventProcessing.h"
#include "NeoPixelRing.h"

#if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
#include "debug.h"
#endif

//==============================================================================
// Helpers

// Shorter topics must come before longer topics
static rgb_tree_callback_type_t getCallbackType(esp_mqtt_event_handle_t event)
{
    if (strncmp(SUB_SET_COLOR, event->topic, event->topic_len) == 0) {
        return SET_COLOR;
    }

    if (strncmp(SUB_GET_COLOR, event->topic, event->topic_len) == 0) {
        return GET_COLOR;
    }

    if (strncmp(SUB_SET_TW_LIGHTS, event->topic, event->topic_len) == 0) {
        return SET_TW;
    }

    if (strncmp(SUB_GET_TW_LIGHTS, event->topic, event->topic_len) == 0) {
        return GET_TW;
    }

    return UNKNOWN;
}

static bool isShortTask(rgb_tree_callback_type_t type) {
    if (type == GET_COLOR || type == GET_TW || type == SET_TW) {
        return true;
    }

    return false;
}

static bool isLongTask(rgb_tree_callback_type_t type)
{
    if (type == SET_COLOR) {
        return true;
    }

    return false;
}

static void clearAction(SubscriptionAction_t *action)
{
    action->callbackType = UNKNOWN;
    action->client = NULL;
    memset(action->data, '\0', SUBSCRIPTIONDATALEN);
    action->dataLength = 0;
}

static void setAction(
    SubscriptionAction_t *action,
    rgb_tree_callback_type_t type,
    esp_mqtt_event_handle_t event
) {
    clearAction(action);

    if (SUBSCRIPTIONDATALEN < event->data_len) {
        Serial.println("Data length was too long");
        return;
    }

    action->callbackType = type;
    action->client = event->client;
    strncpy(action->data, (char *)event->data, event->data_len);
    action->dataLength = event->data_len;
}

//==============================================================================
// Mqtt publish functions

void publishRgbStatus(void)
{
    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println(F("publishRgbStatus()"));
    #endif

    char output[SUBSCRIPTIONDATALEN];
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;

    if (xSemaphoreTake(ringMutex, 0) == pdTRUE) {
        RGB_t color = {0, 0, 0};
        ring.getColor(&color);

        doc["r"] = color.r;
        doc["g"] = color.g;
        doc["b"] = color.b;

        xSemaphoreGive(ringMutex);
    } else {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("The ring is already taken."));
        #endif

        return;
    }

    serializeJson(doc, output, sizeof(output));
    esp_mqtt_client_publish(mqttClient, PUB_GET_COLOR, output, 0, 0, 0);
}

void publishTwinkleLightStatus(void)
{
    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println(F("publishTwinkleLightStatus()"));
    #endif

    char output[SUBSCRIPTIONDATALEN];
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;

    doc["tw1"] = digitalRead(TW1_PIN);
    doc["tw2"] = digitalRead(TW2_PIN);
    doc["tw3"] = digitalRead(TW3_PIN);

    serializeJson(doc, output, sizeof(output));
    esp_mqtt_client_publish(mqttClient, PUB_GET_TW_LIGHTS, output, 0, 0, 0);
}

//==============================================================================
// Mqtt subscribe callback functions

void getColor(SubscriptionAction_t *action)
{
    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println("getColor()");
    #endif

    if (SUBSCRIPTIONDATALEN < action->dataLength) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    publishRgbStatus();
}

void setColor(SubscriptionAction_t *action)
{
    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println("setColor()");
    #endif

    if (SUBSCRIPTIONDATALEN < action->dataLength) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    const int capacity = JSON_OBJECT_SIZE(4);
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, action->data);

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
    } else {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("The ring is already taken. setColor()"));
        #endif

        return;
    }

    publishRgbStatus();
}

void getTwinkleLights(SubscriptionAction_t *action)
{
    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println("getTwinkleLights()");
    #endif

    if (SUBSCRIPTIONDATALEN < action->dataLength) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    publishTwinkleLightStatus();
}

void setTwinkleLights(SubscriptionAction_t *action)
{
    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println("setTwinkleLights()");
    #endif

    if (SUBSCRIPTIONDATALEN < action->dataLength) {
        #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
            Serial.println(F("data is larger than max legnth. Can not parse."));
        #endif

        return;
    }

    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, action->data);

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
// Process Tasks

void processShortTask(void *parameter)
{
    SubscriptionAction_t action;

    while (1) {
        if (xQueueReceive(shortActionQueue, &action, 0) == pdTRUE) {
            #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
                Serial.println(F("processShortTask()"));
                printSubscriptionAction(action);
            #endif

            switch(action.callbackType) {
                case GET_COLOR:
                    getColor(&action);
                    break;
                case GET_TW:
                    getTwinkleLights(&action);
                    break;
                case SET_TW:
                    setTwinkleLights(&action);
                    break;
                default:
                    Serial.println(F("processShortTask() did not call any callback"));
            }

            clearAction(&action);

            #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
                Serial.println(F(""));
            #endif
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void processLongTask(void *parameter)
{
    SubscriptionAction_t action;

    while (1) {
        if (xQueueReceive(longActionQueue, &action, 0) == pdTRUE) {
            #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
                Serial.println(F("processLongTask()"));
                printSubscriptionAction(action);
            #endif

            switch(action.callbackType) {
                case SET_COLOR:
                    setColor(&action);
                    break;
                default:
                    Serial.println(F("processLongTask() did not call any callback"));
            }

            clearAction(&action);

            #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
                Serial.println(F(""));
            #endif
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
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
    #if defined(RGB_TREE_DEBUG) && RGB_TREE_DEBUG
        Serial.println(F("mqtt_handle_data_event()"));
        printMqttMessageData(event->topic, event->topic_len, event->data, event->data_len);
        Serial.println("");
    #endif

    SubscriptionAction_t action;
    rgb_tree_callback_type_t callbackType = getCallbackType(event);

    if (isShortTask(callbackType)) {
        setAction(&action, callbackType, event);
        if (action.callbackType != UNKNOWN && action.client != NULL) {
            xQueueSend(shortActionQueue, &action, portMAX_DELAY);
        }
    }

    if (isLongTask(callbackType)) {
        setAction(&action, callbackType, event);
        if (action.callbackType != UNKNOWN && action.client != NULL) {
            xQueueSend(longActionQueue, &action, portMAX_DELAY);
        }
    }

    ESP_LOGI(MQTT_TAG, "Topic was unhandled");
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_subsribe_all(client);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            mqtt_handle_data_event(event);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGI(MQTT_TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(MQTT_TAG, "Other event id:%d", event->event_id);
            break;
    }
}

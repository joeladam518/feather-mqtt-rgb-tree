#include "config.h"
#include "debug.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include "MqttEventProcessing.h"

void printDeserializeError(DeserializationError *error)
{
    Serial.print(F("Arduino Json eserialization error: "));
    Serial.println(error->c_str());
}

void printMqttMessageData(char *topic, uint16_t topic_length, char *data, uint16_t data_length)
{
    Serial.print(F("Topic: "));
    Serial.printf("%.*s\n", topic_length, topic);
    Serial.print(F("Topic Length: "));
    Serial.println(topic_length);
    Serial.print(F("Data: "));
    Serial.printf("%.*s\n", data_length, data);
    Serial.print(F("Data Length: "));
    Serial.println(data_length);
}

void printSubscriptionAction(SubscriptionAction_t *action)
{
    Serial.print(F("Callback Type: "));
    Serial.println(action->callbackType);
    Serial.print(F("Data: "));
    Serial.println(action->data);
    Serial.print(F("Length: "));
    Serial.println(action->dataLength);
}

void printSubscriptionCallbackData(char *data, uint16_t length)
{
    Serial.print(F("Data: "));
    Serial.println(data);
    Serial.print(F("Length: "));
    Serial.println(length);
}

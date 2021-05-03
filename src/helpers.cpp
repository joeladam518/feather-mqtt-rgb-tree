#include "config.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <Adafruit_MQTT.h>
#include "ColorLed.h"

#ifdef DEBUG
    void printCallbackData(char *data, uint16_t len)
    {
        Serial.print(F("Data: "));
        Serial.println(data);
        Serial.print(F("Length: "));
        Serial.println(len);
    }

    void printColor(RGB *color)
    {
        Serial.print(F("r: "));
        Serial.println(color->r);
        Serial.print(F("g: "));
        Serial.println(color->g);
        Serial.print(F("b: "));
        Serial.println(color->b);
    }

    void printColorAndTime(RGB *color, int time)
    {
        Serial.print(F("r: "));
        Serial.println(color->r);
        Serial.print(F("g: "));
        Serial.println(color->g);
        Serial.print(F("b: "));
        Serial.println(color->b);

        Serial.print(F("time: "));
        Serial.println(time);
    }

    void printDeserializeError(DeserializationError *error)
    {
        Serial.print(F("Deserialization error: "));
        Serial.println(error->c_str());
    }
#endif

char *copyCallbackData(char *data)
{
    char *newData = (char *)malloc(SUBSCRIPTIONDATALEN);
    memset(newData, '\0', SUBSCRIPTIONDATALEN);
    strncpy(newData, data, (SUBSCRIPTIONDATALEN - 1));

    return newData;
}

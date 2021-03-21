#ifndef __NEO_PIXEL_RING__
#define __NEO_PIXEL_RING__

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "ColorLed.h"

class NeoPixelRing
{
private:
    Adafruit_NeoPixel *ring;

public:
    // Consturctors
    NeoPixelRing(Adafruit_NeoPixel *neoPixel);
    // Destructor
    //~NeoPixelRing();
    // Methods
    
    void fadeColor(uint8_t r, uint8_t g, uint8_t b, int fadeTime);
    void fadeColor(RGB *endColor, int fadeTime);
    RGB getColor(void);
    void off(void);
    void rainbow(uint8_t wait);
    void rainbowCycle(uint8_t wait);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setColor(RGB *color);
    RGB unpackColor(uint32_t color);
    uint32_t wheel(uint8_t wheelPos);
    void wipeColor(uint8_t r, uint8_t g, uint8_t b);
    void wipeColor(RGB *color);
};

#endif

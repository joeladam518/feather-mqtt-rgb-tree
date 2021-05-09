#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "led.h"
#include "NeoPixelRing.h"

//-------------------------------
// Constructor
//-------------------------------
NeoPixelRing::NeoPixelRing(Adafruit_NeoPixel *neoPixel):
    neoPixel(neoPixel)
{}

//-------------------------------
// Destructor
//-------------------------------
// TODO: if you ever allocate memory for anything in this class free it with this destructor
// NeoPixelRing::~NeoPixelRing()
// {}

//-------------------------------
// methods
//-------------------------------
void NeoPixelRing::begin()
{
    neoPixel->begin();
    off();
}

void NeoPixelRing::fadeColor(uint8_t r, uint8_t g, uint8_t b, int fadeTime)
{
    RGB color = {0, 0, 0};
    RGB startColor = this->getColor();

    for (int i = 0; i < fadeTime; i++) {
        color.r = map(i, 0, fadeTime, startColor.r, r);
        color.g = map(i, 0, fadeTime, startColor.g, g);
        color.b = map(i, 0, fadeTime, startColor.b, b);

        setColor(&color);
        vTaskDelay(5);
    }

    setColor(r, g, b);
    vTaskDelay(5);
}

void NeoPixelRing::fadeColor(RGB *endColor, int fadeTime)
{
    fadeColor(endColor->r, endColor->g, endColor->b, fadeTime);
}

RGB NeoPixelRing::getColor(void)
{
    // TODO: Whatever color the greatest number of pixel is, return that.
    return unpackColor(neoPixel->getPixelColor(0));
}

void NeoPixelRing::off(void)
{
    setColor(0,0,0);
}

void NeoPixelRing::setColor(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t i;
    for (i = 0; i < neoPixel->numPixels(); i++) {
        neoPixel->setPixelColor(i, r, g, b);
    }

    neoPixel->show();
}

void NeoPixelRing::setColor(RGB *color)
{
    uint16_t i;
    for (i = 0; i < neoPixel->numPixels(); i++) {
        neoPixel->setPixelColor(i, color->r, color->g, color->b);
    }

    neoPixel->show();
}

void NeoPixelRing::rainbow(uint8_t wait)
{
    uint16_t i, j;
    for (j = 0; j < 256; j++) {
        for(i = 0; i < neoPixel->numPixels(); i++) {
            neoPixel->setPixelColor(i, wheel((i + j) & 255));
        }

        neoPixel->show();
        vTaskDelay(wait);
    }
}

void NeoPixelRing::rainbowCycle(uint8_t wait)
{
    uint16_t i, j;
    uint16_t cycles = (256 * 5); // 5 cycles of all colors on wheel

    for (j = 0; j < cycles; j++) {
        for (i = 0; i < neoPixel->numPixels(); i++) {
            neoPixel->setPixelColor(i, wheel(((i * 256 / neoPixel->numPixels()) + j) & 255));
        }

        neoPixel->show();
        vTaskDelay(wait);
    }
}

// TODO make sure this uses dynamic memeory
RGB NeoPixelRing::unpackColor(uint32_t color)
{
    RGB unpackedColor;
    unpackedColor.r = (uint8_t)(color >> 16);
    unpackedColor.g = (uint8_t)(color >> 8);
    unpackedColor.b = (uint8_t)(color);

    return unpackedColor;
}

uint32_t NeoPixelRing::wheel(uint8_t wheelPos)
{
    wheelPos = 255 - wheelPos;

    if (wheelPos < 85) {
        return neoPixel->Color(255 - wheelPos * 3, 0, wheelPos * 3);
    }

    if (wheelPos < 170) {
        wheelPos -= 85;
        return neoPixel->Color(0, wheelPos * 3, 255 - wheelPos * 3);
    }

    wheelPos -= 170;
    return neoPixel->Color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

void NeoPixelRing::wipeColor(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t i;
    for (i = 0; i < neoPixel->numPixels(); i++) {
        neoPixel->setPixelColor(i, r, g, b);
        neoPixel->show();
        vTaskDelay(5);
    }
}

void NeoPixelRing::wipeColor(RGB *color)
{
    uint16_t i;
    for (i = 0; i < neoPixel->numPixels(); i++) {
        neoPixel->setPixelColor(i, color->r, color->g, color->b);
        neoPixel->show();
        vTaskDelay(5);
    }
}

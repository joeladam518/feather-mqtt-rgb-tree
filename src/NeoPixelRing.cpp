#include "NeoPixelRing.h"
#include <Adafruit_NeoPixel.h>
#include "ColorLed.h"

//-------------------------------
// Constructor
//-------------------------------
NeoPixelRing::NeoPixelRing(Adafruit_NeoPixel *neoPixel)
{
    ring = neoPixel;
}

//-------------------------------
// Destructor
//-------------------------------
// TODO: if you ever allocate memory for anything in this class free it with this destructor
// NeoPixelRing::~NeoPixelRing()
// {}

//-------------------------------
// methods
//-------------------------------
void NeoPixelRing::fadeColor(uint8_t r, uint8_t g, uint8_t b, int fadeTime)
{
    RGB color = {0, 0, 0};
    RGB startColor = this->getColor();

    for (int t = 0; t < fadeTime; ++t) {
        color.r = map(t, 0, fadeTime, startColor.r, r);
        color.g = map(t, 0, fadeTime, startColor.g, g);
        color.b = map(t, 0, fadeTime, startColor.b, b);

        setColor(&color);
        delay(1);
    }

    setColor(r,g,b);
    delay(5);
}
void NeoPixelRing::fadeColor(RGB *endColor, int fadeTime)
{
    fadeColor(endColor->r, endColor->g, endColor->b, fadeTime);
}
RGB NeoPixelRing::getColor(void)
{
    // TODO: Whatever color the greatest number of pixel is, return that.
    return unpackColor(ring->getPixelColor(0));
}
void NeoPixelRing::off(void)
{
    setColor(0,0,0);
}
void NeoPixelRing::setColor(uint8_t r, uint8_t g, uint8_t b)
{
    for(uint16_t i = 0; i < ring->numPixels(); ++i) {
        ring->setPixelColor(i, r, g, b);
    }
    ring->show();
    delay(5);
}
void NeoPixelRing::setColor(RGB *color)
{
    for(uint16_t i = 0; i < ring->numPixels(); ++i) {
        ring->setPixelColor(i, color->r, color->g, color->b);
    }
    ring->show();
    delay(5);
}
void NeoPixelRing::rainbow(uint8_t wait)
{
    uint16_t i, j;
    for(j = 0; j < 256; j++) {
        for(i = 0; i < ring->numPixels(); ++i) {
            ring->setPixelColor(i, wheel((i+j) & 255));
        }
        ring->show();
        delay(wait);
    }
}
void NeoPixelRing::rainbowCycle(uint8_t wait)
{
    uint16_t i, j;
    uint16_t cycles = (256 * 5); // 5 cycles of all colors on wheel
    
    for(j = 0; j < cycles; ++j) { 
        for(i = 0; i < ring->numPixels(); ++i) {
            ring->setPixelColor(i, wheel(((i * 256 / ring->numPixels()) + j) & 255));
        }
        ring->show();
        delay(wait);
    }
}
RGB NeoPixelRing::unpackColor(uint32_t color)
{
    RGB unpackedColor;
    unpackedColor.r = (uint8_t)(color >> 16);
    unpackedColor.g = (uint8_t)(color >>  8);
    unpackedColor.b = (uint8_t)color; 

    return unpackedColor;
}
uint32_t NeoPixelRing::wheel(uint8_t wheelPos)
{
    wheelPos = 255 - wheelPos;
    
    if (wheelPos < 85) {
        return ring->Color(255 - wheelPos * 3, 0, wheelPos * 3);
    }

    if (wheelPos < 170) {
        wheelPos -= 85;
        return ring->Color(0, wheelPos * 3, 255 - wheelPos * 3);
    }

    wheelPos -= 170;
    return ring->Color(wheelPos * 3, 255 - wheelPos * 3, 0);
}
void NeoPixelRing::wipeColor(uint8_t r, uint8_t g, uint8_t b)
{
    for(uint16_t i = 0; i < ring->numPixels(); ++i) {
        ring->setPixelColor(i, r, g, b);
        ring->show();
        delay(50);
    }
}
void NeoPixelRing::wipeColor(RGB *color)
{
    for(uint16_t i = 0; i < ring->numPixels(); ++i) {
        ring->setPixelColor(i, color->r, color->g, color->b);
        ring->show();
        delay(50);
    }
}

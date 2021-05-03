#ifndef __NEO_PIXEL_RING_COLOR_LED_H__
#define __NEO_PIXEL_RING_COLOR_LED_H__

#include <Arduino.h>

typedef struct RGB
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB;

typedef enum ColorRGB
{
    Red,
    Green,
    Blue
} ColorRGB;

#endif

#ifndef __LED_H__
#define __LED_H__

#include <Arduino.h>

typedef struct RGB
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB;

typedef struct RGBA
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
} RGBA;

typedef struct RGBW
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t w;
} RGBW;

typedef struct RGBAW
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
    uint8_t w;
} RGBAW;

#endif

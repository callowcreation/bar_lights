#pragma once

#ifndef Lights_h
#define Lights_h
#include "Arduino.h"

#include <Color.h>
#include <ColorStateIndex.h>

#include <Adafruit_NeoPixel.h>

#define PIN 6
#define LED_NUM 12

#define POTENTIOMETER_PIN A1
#define STATE_CHANGE_PIN 8
#define MODE_CHANGE_PIN 9

#define MAX_LIGHTS_COLOR_MODE 2
#define MAX_HUE 327680

#define SIZE_LIST 2
static const unsigned char MAX_LIGHTS_COLOR_STATE[SIZE_LIST] = {7, 2};

static const Color SOLID_COLORS_LIST[7] = {
    Color::CreateColor(255, 255, 255), // white
    Color::CreateColor(255, 0, 0),     // red
    Color::CreateColor(0, 255, 0),     // green
    Color::CreateColor(0, 0, 255),     // blue
    Color::CreateColor(255, 0, 255),   // purple
    Color::CreateColor(255, 255, 0),   // yellow
    Color::CreateColor(0, 255, 255),   // cyan
};


class Lights
{

public:
    Lights(unsigned int maxDistanceCM);

    void Setup(uint8_t brightness);
    void Loop(bool detectedMotion, unsigned int distance, double brightness);

    void ClearStrip();

private:
    Adafruit_NeoPixel mStrip;

    unsigned int mMaxDistanceCM;

    ColorStateIndex mCurrentState;

    // Generally, you should use "unsigned long" for variables that hold time
    // The value will quickly become too large for an int to store
    unsigned long mPreviousMillis;        // will store last time LED was updated
    unsigned long mPreviousRainbowMillis; // will store last time LED was updated

    uint16_t mCurrentPixel;
    bool mClearPixels;

    unsigned int mDistance;

    bool mLightsColorStatePressed = false;

    unsigned int mLightsColorMode;
    bool mLightsColorModePressed = false;

    unsigned long mFirstPixelHue;

    void InitPins();
    
    void WipeColor(uint32_t c, unsigned long interval);

    void ProcessSolidMode(double brightness);
    void ProcessAnimatedMode(unsigned int distance, double brightness);

    void SolidLights(Color c, double brightness);
    void Rainbow(unsigned int interval, double brightness);
    void RedGoldGreenLights(unsigned int distance, double brightness);

    void SetBrightness(uint8_t brightness);
    void SetStripColor(uint32_t color);

    Adafruit_NeoPixel GetStrip();
};
#endif
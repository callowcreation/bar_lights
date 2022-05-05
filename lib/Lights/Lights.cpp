#include "Arduino.h"
#include "Lights.h"

#include <Adafruit_NeoPixel.h>

Lights::Lights(unsigned int maxDistanceCM)
{
    // Parameter 1 = number of pixels in strip
    // Parameter 2 = Arduino pin number (most are valid)
    // Parameter 3 = pixel type flags, add together as needed:
    //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
    //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
    //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
    //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
    //   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
    mStrip = Adafruit_NeoPixel(LED_NUM, PIN, NEO_GRB + NEO_KHZ800);

    mMaxDistanceCM = maxDistanceCM;
}

void Lights::Setup(uint8_t brightness)
{
    mStrip.begin();
    SetBrightness(brightness);
    InitPins();
}

void Lights::Loop(bool detectedMotion, unsigned int distance, double brightness)
{
    if (digitalRead(STATE_CHANGE_PIN) == LOW && mLightsColorStatePressed == false)
    {
        mLightsColorStatePressed = true;
        mCurrentState.Increment(mLightsColorMode);
        if (mCurrentState.GetValue(mLightsColorMode) == MAX_LIGHTS_COLOR_STATE[mLightsColorMode])
        {
            mCurrentState.SetValue(mLightsColorMode, 0);
        }
        mPreviousRainbowMillis = 0;
        mPreviousMillis = 0;
    }
    else if (digitalRead(STATE_CHANGE_PIN) == HIGH)
    {
        mLightsColorStatePressed = false;
    }

    if (digitalRead(MODE_CHANGE_PIN) == LOW && mLightsColorModePressed == false)
    {
        mLightsColorModePressed = true;
        mLightsColorMode++;
        if (mLightsColorMode == MAX_LIGHTS_COLOR_MODE)
        {
            mLightsColorMode = 0;
        }
    }
    else if (digitalRead(MODE_CHANGE_PIN) == HIGH)
    {
        mLightsColorModePressed = false;
    }

    if (mClearPixels)
    {
        ClearStrip();
        mClearPixels = false;
    }
    else
    {
        if (!detectedMotion) // detected motion
        {
            switch (mLightsColorMode)
            {
            case 0:
                ProcessSolidMode(brightness);
                break;
            case 1:
                ProcessAnimatedMode(distance, brightness);
                break;
            default:
                break;
            }
        }
        else
        {
            mCurrentPixel = 0;
            mClearPixels = true;
        }
    }
}

void Lights::InitPins()
{
    pinMode(STATE_CHANGE_PIN, INPUT_PULLUP);
    pinMode(MODE_CHANGE_PIN, INPUT_PULLUP);
}

void Lights::ClearStrip()
{
    uint32_t color = mStrip.Color(0, 0, 0);
    SetStripColor(color);
}

void Lights::WipeColor(uint32_t c, unsigned long interval)
{
    // here is where you'd put code that needs to be running all the time.

    // check to see if it's time to blink the LED; that is, if the difference
    // between the current time and last time you blinked the LED is bigger than
    // the interval at which you want to blink the LED.
    unsigned long currentMillis = millis();

    if (currentMillis - mPreviousMillis >= interval) // test whether the period has elapsed
    {
        mCurrentPixel++;
        if (mCurrentPixel > mStrip.numPixels())
        {
            mCurrentPixel = 0;
        }
        else
        {
            mStrip.setPixelColor(mCurrentPixel - 1, c);
            mStrip.show();
        }
        mPreviousMillis = currentMillis; // IMPORTANT to save the start time of the current LED state.
    }
}

void Lights::ProcessSolidMode(double brightness)
{
    Color c = SOLID_COLORS_LIST[mCurrentState.Solid];
    SolidLights(c, brightness);
}

void Lights::ProcessAnimatedMode(unsigned int distance, double brightness)
{
    switch (mCurrentState.Animated)
    {
    case 0:
        RedGoldGreenLights(distance, brightness);
        break;
    case 1:
        Rainbow(distance * 0.2, brightness);
        break;
    default:
        break;
    }
}

void Lights::SolidLights(Color c, double brightness)
{
    uint32_t color = mStrip.Color(c.r * brightness, c.g * brightness, c.b * brightness);
    SetStripColor(color);
}

void Lights::Rainbow(unsigned int interval, double brightness)
{
    unsigned long currentMillis = millis();

    if (currentMillis - mPreviousRainbowMillis >= interval) // test whether the period has elapsed
    {
        mFirstPixelHue = (mFirstPixelHue + 256) % MAX_HUE;

        for (uint16_t i = 0; i < mStrip.numPixels(); i++)
        {
            unsigned long pixelHue = mFirstPixelHue + (i * 65536L / mStrip.numPixels());
            uint32_t c = mStrip.ColorHSV(pixelHue, 255, 255 * brightness);
            mStrip.setPixelColor(i, mStrip.gamma32(c));
        }
        mStrip.show();

        mPreviousRainbowMillis = currentMillis; // IMPORTANT to save the start time of the current LED state.
    }
}

void Lights::RedGoldGreenLights(unsigned int distance, double brightness)
{
    unsigned int clampedDist = distance;
    if (clampedDist > mMaxDistanceCM)
        clampedDist = mMaxDistanceCM;

    unsigned char result = (int)((float)clampedDist / (float)mMaxDistanceCM * 255.0f);

    Color c = {(unsigned char)(255 - result), result, (unsigned char)0};

    uint32_t color = mStrip.Color(c.r * brightness, c.g * brightness, c.b * brightness);
    WipeColor(color, 0);
}

void Lights::SetBrightness(uint8_t brightness)
{
    mStrip.setBrightness(brightness);
    mStrip.show(); // Initialize all pixels to 'off'
}

void Lights::SetStripColor(uint32_t color)
{
    for (uint16_t i = 0; i < mStrip.numPixels(); i++)
    {
        mStrip.setPixelColor(i, color);
    }
    mStrip.show();
}

Adafruit_NeoPixel Lights::GetStrip()
{
    return mStrip;
}
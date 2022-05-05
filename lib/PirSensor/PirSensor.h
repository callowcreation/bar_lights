#pragma once

#ifndef PirSensor_h
#define PirSensor_h
#include "Arduino.h"

// the amount of milliseconds the sensor has to be low
// before we assume all motion has stopped
#define MOTION_STOP_RESET 15000

class PirSensor
{

public:
    PirSensor(unsigned int calibrationTime);
    void Setup(unsigned int pirPin, unsigned int ledPin);
    void Loop();

    bool GetLockLow();

private:
    // the time we give the sensor to calibrate
    // (10-60 secs according to the datasheet)
    unsigned int mCalibrationTimeSeconds;

    // the time when the sensor outputs a low impulse
    unsigned long mLowIn;

    // stores if the transition to LOW was made
    // before any further output is made
    bool mLockLow;
    bool mLastLockLow;
    bool mTakeLowTime;

    // the digital pin connected to the PIR sensor's output
    unsigned int mPirPin;
    // the digital pin connected to the LED 
    // showing pir output activation
    unsigned int mLedPin;

    void InitPins();
    void CalibrationSensor();
};
#endif
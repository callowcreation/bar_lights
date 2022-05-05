#include "Arduino.h"
#include "PirSensor.h"

PirSensor::PirSensor(unsigned int calibrationTime)
{
    mCalibrationTimeSeconds = calibrationTime;
}

void PirSensor::Setup(unsigned int pirPin, unsigned int ledPin)
{
    mPirPin = pirPin;
    mLedPin = ledPin;

    InitPins();
    CalibrationSensor();
}

void PirSensor::Loop()
{
    if (digitalRead(mPirPin) == HIGH)
    {
        digitalWrite(mLedPin, HIGH); // the led visualizes the sensors output pin state

        if (mLockLow)
        { // makes sure we wait for a transition to LOW before any further output is made:

            mLockLow = false;
            Serial.println("---");
            Serial.print("motion detected at ");
            Serial.print(millis() / 1000);
            Serial.println(" sec");
        }
        mTakeLowTime = true;
    }
    if (digitalRead(mPirPin) == LOW)
    {
        digitalWrite(mLedPin, LOW); // the led visualizes the sensors output pin state
        if (mTakeLowTime)
        {
            mLowIn = millis();    // save the time of the transition from high to LOW
            mTakeLowTime = false; // make sure this is only done at the start of a LOW phase
        }
        // if the sensor is low for more than the given pause,
        // we assume that no more motion is going to happen
        if (!mLockLow && millis() - mLowIn > MOTION_STOP_RESET)
        {
            // makes sure this block of code is only executed again after
            // a new motion sequence has been detected
            mLockLow = true;
            /*Serial.print("motion ended at "); // output
            Serial.print((millis() - pause) / 1000);
            Serial.println(" sec");*/
        }
    }
    if (mLastLockLow != mLockLow)
    {
        mLastLockLow = mLockLow;
    }
}

void PirSensor::InitPins()
{
    pinMode(mPirPin, INPUT);
    pinMode(mLedPin, OUTPUT);
    digitalWrite(mPirPin, LOW); // give the sensor some time to calibrate
}

void PirSensor::CalibrationSensor()
{
    Serial.print("calibrating sensor ");
    for (int i = 0; i < mCalibrationTimeSeconds; i++)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println(" done");
    Serial.println("SENSOR ACTIVE");
    delay(50);
}

bool PirSensor::GetLockLow()
{
    return mLockLow;
}
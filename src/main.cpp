#include <math.h>
#include <Arduino.h>

#include <Ultrasonic.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include <PirSensor.h>
#include <Lights.h>

#define CALIBRATION_TIME_SECONDS 30
#define MAX_DISTANCE_CM 88

Ultrasonic ultrasonic1(12, 13); // An ultrasonic sensor HC-04
PirSensor pirSensor = PirSensor(CALIBRATION_TIME_SECONDS);
Lights lights = Lights(MAX_DISTANCE_CM);

void setup()
{
	Serial.begin(9600);

	pirSensor.Setup(3, 7);

	lights.Setup(100);
	lights.ClearStrip();
}

void loop()
{
	pirSensor.Loop();

	unsigned int currentDistance = ultrasonic1.read();

	int potentiometerValue = analogRead(POTENTIOMETER_PIN);
	double brightness = potentiometerValue / 4; // 1 to 255
	brightness /= 255;							// 0.0 to 1.0

	/*Serial.print("lightsColorMode: ");
	Serial.print(lightsColorMode);
	Serial.print("  lightsColorState: ");
	Serial.println(lightsColorState[lightsColorMode]);*/

	lights.Loop(pirSensor.GetLockLow(), currentDistance, brightness);
}

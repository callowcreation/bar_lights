#include <math.h>
#include <Arduino.h>

#include <Ultrasonic.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include "color.h"

// --- Ultrasonic -----------------------------------------------
Ultrasonic ultrasonic1(12, 13); // An ultrasonic sensor HC-04

unsigned int maxDistanceCM = 88;

// --- Ultrasonic -----------------------------------------------

// ---- PIR Sensor -----------------------------------------------
// the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 30;
// the time when the sensor outputs a low impulse
long unsigned int lowIn;
// the amount of milliseconds the sensor has to be low
// before we assume all motion has stopped
long unsigned int pause = 15000;
boolean lockLow = true;
boolean takeLowTime;
int pirPin = 3; // the digital pin connected to the PIR sensor's output
int ledPin = 8;

bool lastLockLow = lockLow;

bool turnLightsOn = false;

void pirSetup()
{
	pinMode(pirPin, INPUT);
	pinMode(ledPin, OUTPUT);
	digitalWrite(pirPin, LOW); // give the sensor some time to calibrate
	Serial.print("calibrating sensor ");
	for (int i = 0; i < calibrationTime; i++)
	{
		Serial.print(".");
		delay(1000);
	}
	Serial.println(" done");
	Serial.println("SENSOR ACTIVE");
	delay(50);
}

void pirLoop()
{
	if (digitalRead(pirPin) == HIGH)
	{
		digitalWrite(ledPin, HIGH); // the led visualizes the sensors output pin state

		if (lockLow)
		{ // makes sure we wait for a transition to LOW before any further output is made:

			lockLow = false;
			Serial.println("---");
			Serial.print("motion detected at ");
			Serial.print(millis() / 1000);
			Serial.println(" sec");
		}
		takeLowTime = true;
	}
	if (digitalRead(pirPin) == LOW)
	{
		digitalWrite(ledPin, LOW); // the led visualizes the sensors output pin state
		if (takeLowTime)
		{
			lowIn = millis();	 // save the time of the transition from high to LOW
			takeLowTime = false; // make sure this is only done at the start of a LOW phase
		}
		// if the sensor is low for more than the given pause,
		// we assume that no more motion is going to happen
		if (!lockLow && millis() - lowIn > pause)
		{
			// makes sure this block of code is only executed again after
			// a new motion sequence has been detected
			lockLow = true;
			Serial.print("motion ended at "); // output
			Serial.print((millis() - pause) / 1000);
			Serial.println(" sec");
		}
	}
	if (lastLockLow != lockLow)
	{
		lastLockLow = lockLow;
	}
}

// ---- PIR Sensor -----------------------------------------------

// ---- Lights -----------------------------------------------
#define PIN 6
#define LED_NUM 12

#define POTENTIOMETER_PIN A1
#define STATE_CHANGE_PIN 7

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_NUM, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0; // will store last time LED was updated

// constants won't change:
const long interval = 0; // interval at which to blink (milliseconds)

uint16_t currentPixel = 0;
bool clearPixels;

unsigned int distance = 0;
unsigned char lightsColorState = 0;
boolean lightsColorStatePressed = false;
const char MAX_LIGHTS_COLOR_STATE = 2;

void setBrightness(uint8_t brightness)
{
	strip.setBrightness(brightness);
	strip.show(); // Initialize all pixels to 'off'
}

void lightsSetup()
{
	strip.begin();
	setBrightness(100);
	pinMode(STATE_CHANGE_PIN, INPUT_PULLUP);
}

void setStripColorStrip(uint32_t color)
{
	for (uint16_t i = 0; i < strip.numPixels(); i++)
	{
		strip.setPixelColor(i, color);
		strip.show();
	}
}

void clearStrip()
{
	uint32_t color = strip.Color(0, 0, 0);
	setStripColorStrip(color);
}

// color wipe with mills not delay
void wipeColor(uint32_t c, long interval)
{
	// here is where you'd put code that needs to be running all the time.

	// check to see if it's time to blink the LED; that is, if the difference
	// between the current time and last time you blinked the LED is bigger than
	// the interval at which you want to blink the LED.
	unsigned long currentMillis = millis();

	if (currentMillis - previousMillis >= interval) // test whether the period has elapsed
	{
		currentPixel++;
		if (currentPixel > strip.numPixels())
		{
			currentPixel = 0;
		}
		else
		{
			strip.setPixelColor(currentPixel - 1, c);
			strip.show();
		}
		previousMillis = currentMillis; // IMPORTANT to save the start time of the current LED state.
	}
}

// color wipe with mills not delay
void transitionColor(uint32_t c, long interval)
{
	// here is where you'd put code that needs to be running all the time.

	// check to see if it's time to blink the LED; that is, if the difference
	// between the current time and last time you blinked the LED is bigger than
	// the interval at which you want to blink the LED.
	unsigned long currentMillis = millis();

	if (currentMillis - previousMillis >= interval) // test whether the period has elapsed
	{
		setStripColorStrip(c);
		previousMillis = currentMillis; // IMPORTANT to save the start time of the current LED state.
	}
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
	WheelPos = 255 - WheelPos;
	if (WheelPos < 85)
	{
		return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	if (WheelPos < 170)
	{
		WheelPos -= 85;
		return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	WheelPos -= 170;
	return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void redGoldGreenLights(unsigned int distance, double brightness)
{
	unsigned int clampedDist = distance;
	if (clampedDist > maxDistanceCM)
		clampedDist = maxDistanceCM;

	unsigned char result = (unsigned char)((float)clampedDist / (float)maxDistanceCM * 255.0f);

	Color c = {(unsigned char)255 - result, result, 0};

	uint32_t color = strip.Color(c.r * brightness, c.g * brightness, c.b * brightness);
	wipeColor(color, 0);
}

void whiteLights(double brightness)
{
	Color c = {(unsigned char)255, (unsigned char)255, (unsigned char)255};
	uint32_t color = strip.Color(c.r * brightness, c.g * brightness, c.b * brightness);
	setStripColorStrip(color);
}

void lightsLoop(unsigned int distance, double brightness)
{
	if (clearPixels)
	{
		clearStrip();
		clearPixels = false;
	}
	else
	{
		if (!lockLow) // detected motion
		{
			if (digitalRead(STATE_CHANGE_PIN) == LOW && lightsColorStatePressed == false)
			{
				lightsColorStatePressed = true;
				lightsColorState++;
				if (lightsColorState == MAX_LIGHTS_COLOR_STATE)
				{
					lightsColorState = 0;
				}
			}
			else if (digitalRead(STATE_CHANGE_PIN) == HIGH)
			{
				lightsColorStatePressed = false;
			}
			
			switch (lightsColorState)
			{
			case 0:
				whiteLights(brightness);
				break;
			case 1:
				redGoldGreenLights(distance, brightness);
				break;
			default:
				break;
			}
		}
		else
		{
			currentPixel = 0;
			clearPixels = true;
		}
	}
}

// ---- Lights -----------------------------------------------

void setup()
{
	Serial.begin(9600);
	pirSetup();
	lightsSetup();
	clearStrip();
}

void loop()
{
	pirLoop();

	distance = ultrasonic1.read();

	int potentiometerValue = analogRead(POTENTIOMETER_PIN);
	double brightness = potentiometerValue / 4; // 1 to 255
	brightness /= 255;							// 0.0 to 1.0

	Serial.print("  distance: ");
	Serial.println(distance);

	lightsLoop(distance, brightness);
}

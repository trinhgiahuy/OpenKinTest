/*
Based on Adafruit Arduino - Lesson 16. 
4 Wire stepper interface for STH-39D1126(1.8deg/step)
*/

#include <Stepper.h>

int in1Pin = 2;
int in2Pin = 4;
int in3Pin = 10;
int in4Pin = 11;

Stepper motor(200, in1Pin, in2Pin, in3Pin, in4Pin);  

void setup()
{
		pinMode(in1Pin, OUTPUT);
		pinMode(in2Pin, OUTPUT);
		pinMode(in3Pin, OUTPUT);
		pinMode(in4Pin, OUTPUT);
		motor.setSpeed(20);
}

void loop()
{
		motor.step(5);
		delay(2000);
}

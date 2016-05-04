/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */

void setup() {                
  // initialize the digital pin as an output.
  // Pin 15 has an LED connected on Beaglebone Black:
  digitalWrite(15, 1);
}

void loop() {
  digitalWrite(15, 1);   // set the LED on
  delay(1000);              // wait for a second
  digitalWrite(15, 0);    // set the LED off
  delay(1000);              // wait for a second
}


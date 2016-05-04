//
//  BaconDemo
//
//  Copyright (c) 2013 Parav Nagarsheth
//  Push Button Connected to P8_19, Digital Pin 7

int pushButton = 7;
int sliderPot  = 20;
void setup() {                
  // initialise digital pins as inputs
  pinMode(pushButton, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  int buttonState = digitalRead(pushButton);
  int sliderState = analogRead(sliderPot);

  // print out the state of the button:
  printf("Button State:%d Slider State:%d\n", buttonState, sliderState);
  delay(10);        // delay in between reads for stability
}


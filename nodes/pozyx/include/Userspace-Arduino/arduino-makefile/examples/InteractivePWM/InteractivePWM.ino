/*
        interactivePWM
 
        Interactively controls the Bacon Cape RGB LED intensity with
        the slider while the color displayed can be cycled through
        red, green, and blue with the push button.
*/
 
int color[3] = {3, 5, 6};
 
void setup() {
        pinMode(7, INPUT);
        // pinMode(20, INPUT);
        // pinMode(3, OUTPUT);
 
}
 
int i = 0;
 
void loop() {
        int buttonState = digitalRead(7);
        // Read slider pot and normalize small values to 0
        int sliderState = analogRead(20) / 100;
 
        // If button pressed, switch our color
        if (!buttonState) {
                analogWrite(color[i], 0);
                if (!(++i%3))
                        i = 0;
                delay(250);
        }
 
        // Scale output value and write to PWM
        analogWrite(color[i], sliderState * 2000);
 
        // Delay a bit
        delay(100);
}


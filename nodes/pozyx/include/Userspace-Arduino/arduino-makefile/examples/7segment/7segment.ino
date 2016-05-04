	

    /*
    7segment
     
    Displays digits on 7 segment display on Bacon Board
    */
     
    #define LATCH 10
    #define CLOCK 13
    #define DATA 11
    #define pushButton 7
    #define sliderPot 20
     
    /*
     clk=/sys/class/gpio/gpio2
     serial=/sys/class/gpio/gpio4
     latch=/sys/class/gpio/gpio5
    */
    const byte ledCharSet[11] = {
     
      B00111111,B00000110,B01011011,B01001111,B01100110,B01101101,B01111101,B00000111,B01111111,B01101111,B10000000
    };
     
    void setup(){
      // initialise digital pins as inputs
      pinMode(pushButton, INPUT);
      pinMode(LATCH, 1);
      pinMode(CLOCK, 1);
      pinMode(DATA,1);
     
    }
     
    int i = 0;
     
     
    void loop(){
            int buttonState = digitalRead(pushButton);
	    int sliderState = analogRead(sliderPot);
            /* Serial.println("Seven segment display test"); */
            /* print out the state of the button: */
            /* delay in between reads for stability */
            if (!buttonState) {
                    digitalWrite(LATCH,0);
                    shiftOut(DATA,CLOCK,MSBFIRST,~(ledCharSet[i]));
                    digitalWrite(LATCH,1);
                    i++;
                    if(i>=10){
                        i = 0;
                    }
            delay(250);
            }
            else {

		    i=abs(sliderState/100);
                    digitalWrite(LATCH,0);
                    shiftOut(DATA,CLOCK,MSBFIRST,~(ledCharSet[i]));
                    digitalWrite(LATCH,1);

	    }
            delay(10);
    }


/*
  PWM
  Turns on RGB LED 
*/		
void setup() {

analogWrite(3,0);
analogWrite(5,0);
analogWrite(6,0);

}

void loop() {

int i=0;


for (i = 0 ; i < 33 ; i++ ) {
		analogWrite(3,(i*8));
		delay(50);
}

for (i = 32 ; i > -1 ; i-- ) {
		analogWrite(3,(i*8));
		delay(50);
}

		delay(100);

for (i = 0 ; i < 33 ; i++ ) {
		analogWrite(5,(i*8));
		delay(50);
}

for (i = 32 ; i > -1 ; i-- ) {
		analogWrite(5,(i*8));
		delay(50);
}

		delay(100);

for (i = 0 ; i < 33 ; i++ ) {
		analogWrite(6,(i*8));
		delay(50);
}

for (i = 32 ; i > -1 ; i-- ) {
		analogWrite(6,(i*8));
		delay(50);
}

		delay(100);

}

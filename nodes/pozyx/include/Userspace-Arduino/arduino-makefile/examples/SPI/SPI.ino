/*
  AT93C46 SPI EEPROM sample code

  created 19 Aug 2013
  by Anuj Deshpande
*/

#include<SPI.h>
#define SS 4

void setup()
{
  pinMode(SS,OUTPUT);
  SPI.begin();
  SPI.setClockDivider(200);
  SPI.setDataMode(0);
  SPI.setBitOrder(MSBFIRST);
  digitalWrite(SS,LOW);

}
void loop()
{
  byte b ; 
  digitalWrite(SS,HIGH);
  b = SPI.transfer(0b11000000);
  printf("%d \n",b);
  digitalWrite(SS,LOW);
  delay(200);
  }

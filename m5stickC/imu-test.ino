#include <M5StickCPlus.h>

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

/* After M5StickC Plus is started or reset
  the program in the setUp () function will be run, and this part will only be run once.

*/
void setup(){
  M5.begin();             //Init M5StickC plus
  M5.IMU.Init();
  M5.Lcd.setRotation(3);
  M5.Lcd.setCursor(80, 15);
  M5.Lcd.println("IMU data test");
  M5.Lcd.setCursor(30, 30);
  M5.Lcd.println("  X       Y       Z");
  M5.Lcd.setCursor(30,70);
  M5.Lcd.println("  Pitch   Roll    Yaw");
}

void loop(){
  static float tmp = 0;
  
  //Get IMU data
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  M5.IMU.getTempData(&tmp);
  M5.Lcd.setCursor(30, 40);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
  M5.Lcd.setCursor(170, 40);
  M5.Lcd.print("o/s");
  M5.Lcd.setCursor(30, 50);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
  M5.Lcd.setCursor(170, 50);
  M5.Lcd.print("G");
  M5.Lcd.setCursor(30, 80);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);

  M5.Lcd.setCursor(30, 95);
  M5.Lcd.printf("Temperature : %.2f C", tmp);
  delay(1000);         //Delay 1s
  
}

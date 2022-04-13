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
  M5.Lcd.setCusor(80, 15);
  M5.Lcd.println("IMU data test");
  M5.Lcd.setCusor(30, 30);
  M5.Lcd.println(" X       Y       Z");
  M5.Lcd.setCusor(" Pitch   Roll    Yaw");
}

void loop(){
  static float tmp = 0;
  
  //Get IMU data
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  M5.IMU.getTempData(&temp);
  
  delay(1000);         //Delay 1s
  
}

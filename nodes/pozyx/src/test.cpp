#include <stdint.h>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>


int main()
{  

  std::cout << "In setup!" << std::endl;

  int fd = wiringPiI2CSetup(0x4b);
  if (fd != -1) {
    std::cout << "wiringPi loaded" << std::endl;

    uint8_t status = (uint8_t)wiringPiI2CReadReg8(fd, 0x0);
    std::cout << "Status: " << std::hex << (int)status << std::endl;
    uint8_t data = (uint8_t)wiringPiI2CReadReg8(fd, 0x1);
    std::cout << "FW: " << std::hex << (int)data << std::endl;
    data = (uint8_t)wiringPiI2CReadReg8(fd, 0x2);
    std::cout << "HW: " << std::hex << (int)data << std::endl;
    data = (uint8_t)wiringPiI2CReadReg8(fd, 0x3);
    std::cout << "ST: " << std::hex << (int)data << std::endl;
    data = (uint8_t)wiringPiI2CReadReg8(fd, 0x4);
    std::cout << "Error: " << std::hex << (int)data << std::endl;

  } else {
    std::cerr << "wiringPi error: " << fd << std::endl;
  }

}

void loop() {
}

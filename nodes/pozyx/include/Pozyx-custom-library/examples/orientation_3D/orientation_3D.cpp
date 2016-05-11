/**
  The pozyx ranging demo (c) Pozyx Labs
  please check out https://www.pozyx.io/Documentation/Tutorials/getting_started

  This demo requires one (or two) pozyx shields and one Arduino. It demonstrates the 3D orientation and the functionality
  to remotely read register data from a pozyx device. Place one of the pozyx shields on the Arduino and upload this sketch.

  This demo reads the following sensor data:
  - pressure
  - acceleration
  - magnetic field strength
  - angular velocity
  - the heading, roll and pitch
  - the quaternion rotation describing the 3D orientation of the device. This can be used to transform from the body coordinate system to the world coordinate system.
  - the linear acceleration (the acceleration excluding gravity)
  - the gravitational vector

  The data can be viewed in the Processing sketch orientation_3D.pde
*/

#include "helpers.hh"
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <iostream>
#include <fstream>

using std::ofstream;

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

bool bRemote = false;                  // boolean to indicate if we want to read sensor data from the attached pozyx shield (value 0) or from a remote pozyx device (value 1)
uint16_t destination_id = 0x6606;     // the network id of the other pozyx device: fill in the network id of the other device
uint32_t last_millis;                 // used to compute the measurement interval in milliseconds

ofstream outfile;

////////////////////////////////////////////////

void loop();

int main()
{
  if(Pozyx.begin(true, MODE_POLLING, 0) == POZYX_FAILURE){
    std::cerr << "ERROR: Unable to connect to POZYX shield" << std::endl;
    std::cerr << "Reset required" << std::endl;
    delay(100);
    return -1;
  }

  outfile.open("/home/openkin/data/test.txt", std::ios::out);

  last_millis = millis();
  delay(10);

  std::cout << std::dec;

  while (true) {
    loop();
  }
  return 0;
}

void loop(){

  int16_t sensor_data[24];
  uint8_t calib_status = 0;
  int i, dt;

  if(bRemote == true)
  {
    // remotely read the sensor data
    int status = Pozyx.remoteRegRead(destination_id, POZYX_PRESSURE, (uint8_t*)&sensor_data, 24*sizeof(int16_t));
    if(status != POZYX_SUCCESS){
      return;
    }

  }else
  {
    // wait until this device gives an interrupt
    if (true || Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 1))
    {
      // we received an interrupt from pozyx telling us new IMU data is ready, now let's read it!
      Pozyx.regRead(POZYX_PRESSURE, (uint8_t*)&sensor_data, 24*sizeof(int16_t));

      // also read out the calibration status
      Pozyx.regRead(POZYX_CALIB_STATUS, &calib_status, 1);
    }else{
      // we didn't receive an interrupt
      uint8_t interrupt_status = 0;
      Pozyx.regRead(POZYX_INT_STATUS, &interrupt_status, 1);

      return;
    }
  }

  // print out the results

  // print the measurement interval
  dt = millis() - last_millis;
  last_millis += dt;
  outfile << dt;

  // print out the presure (this is not an int16 but rather an uint32
  uint32_t pressure = ((uint32_t)sensor_data[0]) + (((uint32_t)sensor_data[1])<<16);
  outfile << ",";
  outfile << pressure;

  // print out all remaining sensors
  for(i=2; i<24; i++){
    outfile << ",";
    outfile << sensor_data[i];
  }

  uint8_t res = 0;

  // finally, print out the calibration status (remotely this is not available and all equal to zero)
  outfile << ",";
  res = calib_status&0x03;
  outfile << (int)res;
  outfile << ",";
  res = (calib_status&0x0C)>>2;
  outfile << (int)res;
  outfile << ",";
  res = (calib_status&0x30)>>4;
  outfile << (int)res;
  outfile << ",";
  res = (calib_status&0xC0)>>6;
  outfile << (int)res;

  outfile << std::endl;
}

#include "Pozyx-custom-library/Pozyx.h"
#include "Pozyx-custom-library/Pozyx_definitions.h"
#include "Pozyx-custom-library/helpers.hh"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <sstream>

int main(int argc, char **argv) {

	uint32_t last_millis;

	ros::init(argc, argv, "pozyx");

	ros::NodeHandle n;

	ros::Publisher pozyx_pub = n.advertise<std_msgs::String>("imu_data", 1000);

	ros::Rate loop_rate(150);

	if(Pozyx.begin(true, MODE_INTERRUPT, POZYX_INT_MASK_IMU, 0) == POZYX_FAILURE){
    std::cerr << "ERROR: Unable to connect to POZYX shield" << std::endl;
    std::cerr << "Reset required" << std::endl;
    delay(100);
    return -1;
  }

  last_millis = millis();
  delay(10);

	while (ros::ok()) {
		std_msgs::String msg;

		int16_t sensor_data[24];
	  uint8_t calib_status = 0;
	  int i, dt;

		std::stringstream ss;

		// wait until this device gives an interrupt
    if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 8))
    {

      // we received an interrupt from pozyx telling us new IMU data is ready, now let's read it!
      Pozyx.regRead(POZYX_PRESSURE, (uint8_t*)&sensor_data, 24*sizeof(int16_t));

      // also read out the calibration status
      Pozyx.regRead(POZYX_CALIB_STATUS, &calib_status, 1);

    }else{

      // we didn't receive an interrupt
      uint8_t interrupt_status = 0;
      Pozyx.regRead(POZYX_INT_STATUS, &interrupt_status, 1);

      continue;
    }

		// print the measurement interval
	  dt = millis() - last_millis;
	  last_millis += dt;

		ss << dt;

		// print out the presure (this is not an int16 but rather an uint32
	  uint32_t pressure = ((uint32_t)sensor_data[0]) + (((uint32_t)sensor_data[1])<<16);
	  ss << "," << pressure;

		// print out all remaining sensors
	  for(i=2; i<24; i++){
	    ss << "," << sensor_data[i];
	  }

		uint8_t res = 0;

	  // finally, print out the calibration status (remotely this is not available and all equal to zero)
	  ss << ",";
	  res = calib_status&0x03;
	  ss << (int)res;
	  ss << ",";
	  res = (calib_status&0x0C)>>2;
	  ss << (int)res;
	  ss << ",";
	  res = (calib_status&0x30)>>4;
	  ss << (int)res;
	  ss << ",";
	  res = (calib_status&0xC0)>>6;
	  ss << (int)res;

	  ss << std::endl;

		msg.data = ss.str();

		pozyx_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}

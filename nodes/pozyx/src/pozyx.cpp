#include "Pozyx-custom-library/Pozyx.h"
#include "Pozyx-custom-library/Pozyx_definitions.h"
#include "Pozyx-custom-library/helpers.hh"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PointStamped.h>

#include <iostream>
#include <sstream>
#include <vector>

#define MG_2_MPSS 0.00980665
#define DEG_2_RAD 0.01745329252

class PozyxROS {
public:
	PozyxROS();

	void update();
	void spin();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	sensor_msgs::Imu imu_msg;

	ros::Publisher imu_pub_;
	ros::Publisher magnetic_pub_;
	ros::Publisher pos_pub_;

	std::string imu_frame_id_;

	ros::Time last_update_;

	int list_size_i;
	uint16_t devices[MAX_ANCHORS_IN_LIST];

};

PozyxROS::PozyxROS() :
	nh_(), private_nh_("~")
{
	imu_pub_ = nh_.advertise<sensor_msgs::Imu>("pozyx/data",10);

	magnetic_pub_ = nh_.advertise<sensor_msgs::MagneticField>("pozyx/mag",10,false);

	pos_pub_ = nh_.advertise<geometry_msgs::PointStamped>("pozyx/pos",10,false);

	std::vector<double> orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance;

	if (orientation_covariance.size() == 9) {
		for (int i = 0; i < 9; i++) {
			imu_msg.orientation_covariance[i] = orientation_covariance[i];
		}
	}

	if (angular_velocity_covariance.size() == 9) {
		for (int i=0; i < 9; i++) {
			imu_msg.angular_velocity_covariance[i] = angular_velocity_covariance[i];
		}
	}

	if (linear_acceleration_covariance.size() == 9) {
		for (int i=0; i < 9; i++) {
			imu_msg.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
		}
	}

	if(Pozyx.begin(true, MODE_INTERRUPT, POZYX_INT_MASK_IMU, 0) == POZYX_FAILURE){
    std::cerr << "ERROR: Unable to connect to POZYX shield" << std::endl;
    std::cerr << "Reset required" << std::endl;
    delay(100);
    throw 1;
  }

	bool done = false;

	int disc_status;
	if ((disc_status = Pozyx.doDiscovery(POZYX_DISCOVERY_ANCHORS_ONLY, 3, 10)) == POZYX_SUCCESS) {
		//delay(1000);

		if (Pozyx.getDeviceIds(devices) == POZYX_SUCCESS) {
			uint8_t list_size;

			if (Pozyx.getDeviceListSize(&list_size) == POZYX_SUCCESS) {
				list_size_i = (int)list_size;
				if (list_size_i > 6) {
					list_size_i = 6;
				}
				std::cout << "Device list size: " << list_size_i << std::endl;
				if (Pozyx.doAnchorCalibration(POZYX_2D, 10, list_size_i, devices) == POZYX_SUCCESS) {
					if (Pozyx.setUpdateInterval(101) == POZYX_SUCCESS) {
						done = true;
					} else {
						std::cerr << "Couldn't start positioning" << std::endl;
					}
				} else {
					std::cerr << "Couldn't calibrate" << std::endl;
				}
			} else {
				std::cerr << "Couldn't get device list size" << std::endl;
			}

		} else {
			std::cerr << "Couldn't get devices" << std::endl;
		}
	} else if (disc_status == POZYX_TIMEOUT) {
		std::cerr << "Couldn't discover (timeout)" << std::endl;
	} else {
		std::cerr << "Couldn't discover" << std::endl;
	}

	if (!done) {
		throw 2;
	}
}

void PozyxROS::update() {
	imu_frame_id_ = "pozyx";
	while (ros::ok()) {
		// get imu data

		int16_t sensor_data[12];
		int32_t pos_data[3];
		coordinates_t pos;
		bool pos_error = false;


		// wait until this device gives an interrupt
    if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 8))
    {
      // we received an interrupt from pozyx telling us new IMU data is ready, now let's read it!
      Pozyx.regRead(POZYX_ACCEL_X, (uint8_t*)&sensor_data, 9*sizeof(int16_t));
			ros::Time current_time = ros::Time::now();
			Pozyx.regRead(POZYX_QUAT_W, (uint8_t*)&sensor_data[9], 4*sizeof(int16_t));
      // also read out the calibration status
      //Pozyx.regRead(POZYX_CALIB_STATUS, &calib_status, 1);

			// Read last position (might be duplicate) -- change 3->9 to get error & covariance

			Pozyx.regRead(POZYX_POS_X, (uint8_t*)&pos_data, 3*sizeof(int32_t));

			/*if (Pozyx.doPositioning(&pos, POZYX_2D, 0) != POZYX_SUCCESS) {
				pos_error = true;
			}*/

    }else{

      // we didn't receive an interrupt
      //uint8_t interrupt_status = 0;
      //Pozyx.regRead(POZYX_INT_STATUS, &interrupt_status, 1);

      continue;
    }

		ros::Time current_time = ros::Time::now();

		// print out the presure (this is not an int16 but rather an uint32
		//uint32_t pressure = ((uint32_t)sensor_data[0]) + (((uint32_t)sensor_data[1])<<16);

		imu_msg.header.stamp = current_time;
		imu_msg.header.frame_id = imu_frame_id_;
		imu_msg.orientation.x = (double)sensor_data[10] / 16384;
		imu_msg.orientation.y = (double)sensor_data[11] / 16384;
		imu_msg.orientation.z = (double)sensor_data[12] / 16384;
		imu_msg.orientation.w = (double)sensor_data[9] / 16384;

		imu_msg.angular_velocity.x = (double)sensor_data[6] * DEG_2_RAD / 16;
		imu_msg.angular_velocity.y = (double)sensor_data[7] * DEG_2_RAD / 16;
		imu_msg.angular_velocity.z = (double)sensor_data[8] * DEG_2_RAD / 16;

		imu_msg.linear_acceleration.x = (double)sensor_data[0] * MG_2_MPSS;
		imu_msg.linear_acceleration.y = (double)sensor_data[1] * MG_2_MPSS;
		imu_msg.linear_acceleration.z = (double)sensor_data[2] * MG_2_MPSS;

		imu_pub_.publish(imu_msg);

		sensor_msgs::MagneticField msg;
		msg.header.frame_id = imu_frame_id_;
		msg.header.stamp = current_time;

		// compareable to xsens output
		msg.magnetic_field.x = (double)sensor_data[3] / (16 * 37);
		msg.magnetic_field.y = (double)sensor_data[4] / (16 * 37);
		msg.magnetic_field.z = (double)sensor_data[5] / (16 * 37);

		magnetic_pub_.publish(msg);

		if (!pos_error) {

			/*uint32_t pos_x = ((uint32_t)pos_data[0]) + (((uint32_t)pos_data[1])<<16);
			uint32_t pos_y = ((uint32_t)pos_data[2]) + (((uint32_t)pos_data[3])<<16);
			uint32_t pos_z = ((uint32_t)pos_data[4]) + (((uint32_t)pos_data[5])<<16);
			*/
			geometry_msgs::PointStamped pos_msg;

			pos_msg.header.stamp = current_time;
			pos_msg.header.frame_id = imu_frame_id_;

			pos_msg.point.x = pos_data[0];
			pos_msg.point.y = pos_data[1];
			pos_msg.point.z = pos_data[2];

			pos_pub_.publish(pos_msg);

			// read out the ranges to each anchor and print it
		  /*for (int i=0; i < list_size_i; i++){
		    device_range_t range;
		    Pozyx.getDeviceRangeInfo(devices[i], &range);
				std::cout << devices[i] << ":" << "D: " << range.distance << ", S: " << range.RSS << std::endl;
		  }*/

		} else {
			std::cerr << "Pos error" << std::endl;
		}

		ros::spinOnce();
	}
}

void PozyxROS::spin() {
	ros::Rate r(500);
	while (ros::ok()) {
		update();
		r.sleep();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "Pozyx_node");

	ROS_INFO("Pozyx Node for ROS");

	PozyxROS pozros;
	pozros.spin();

	return 0;
}

/*
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
*/

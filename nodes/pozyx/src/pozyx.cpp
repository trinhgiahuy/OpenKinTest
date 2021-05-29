#include "Pozyx-custom-library/Pozyx.h"
#include "Pozyx-custom-library/Pozyx_definitions.h"
#include "Pozyx-custom-library/helpers.hh"

#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <sensor_msgs/Imu.h>
#include <imu_sequenced/ImuSequenced.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <pozyx/StringStamped.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <string>
#include <fstream>

#define MG_2_MPSS 0.00980665
#define DEG_2_RAD 0.01745329252

sig_atomic_t sigcaught = 0;

class PozyxROS {
public:
	PozyxROS();

	void update();
	void reset();

private:

	PozyxClass Pozyx;

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	imu_sequenced::ImuSequenced imu_msg;

	//ros::Publisher imu_pub_;
	//ros::Publisher magnetic_pub_;
	ros::Publisher pos_pub_;
	ros::Publisher range_pub_;
	//ros::Publisher anchor_pub_;

	std::string imu_frame_id_;

	ros::Time last_update_;

	int adapter;
	std::string tagfile;

	int list_size_i;
	uint16_t devices[MAX_ANCHORS_IN_LIST];

};

void PozyxROS::reset() {
	Pozyx.resetSystem();
	return;
}

PozyxROS::PozyxROS() :
	nh_(), private_nh_("~")
{

	private_nh_.param("adapter", adapter, int(1));
	private_nh_.param(std::string("tagfile"), tagfile, std::string(""));

	/*
	std::vector<double> orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance;

	if (orientation_covariance.size() == 9) {
		for (int i = 0; i < 9; i++) {
			imu_msg.imu.orientation_covariance[i] = orientation_covariance[i];
		}
	}

	if (angular_velocity_covariance.size() == 9) {
		for (int i=0; i < 9; i++) {
			imu_msg.imu.angular_velocity_covariance[i] = angular_velocity_covariance[i];
		}
	}

	if (linear_acceleration_covariance.size() == 9) {
		for (int i=0; i < 9; i++) {
			imu_msg.imu.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
		}
	}
	*/

	//if(Pozyx.begin(adapter, true, MODE_INTERRUPT, POZYX_INT_MASK_IMU, 0) == POZYX_FAILURE){
	//if(Pozyx.begin(adapter, true, MODE_INTERRUPT, POZYX_INT_MASK_POS | POZYX_INT_MASK_FUNC | POZYX_INT_MASK_ERR , 0) == POZYX_FAILURE){
	if(Pozyx.begin(adapter, true, MODE_INTERRUPT, 0x1F , 0) == POZYX_FAILURE){
		std::cerr << "ERROR: Unable to connect to POZYX shield" << std::endl;
		std::cerr << "Reset required" << std::endl;
		delay(100);
		throw 1;
	}

	bool done = false;
	bool fromfile = false;

	std::cout << "Resetting system..." << std::endl;
	Pozyx.resetSystem();
	delay(1000);
	//Pozyx.clearDevices();
	Pozyx.setInterruptMask(POZYX_INT_MASK_POS | POZYX_INT_MASK_FUNC);

	std::ifstream tagfd;
	tagfd.open(tagfile.c_str());
	if (!tagfd.is_open()) {
		int disc_status = 0;
		std::cout << "Discovering..." << std::endl;
		while ((disc_status = Pozyx.doDiscovery(POZYX_DISCOVERY_ANCHORS_ONLY, 9, 30)) != POZYX_SUCCESS) {
			std::cerr << "Retrying discovery: " << disc_status << std::endl;
			delay(3000);
		}
		//if ((disc_status = Pozyx.doDiscovery(POZYX_DISCOVERY_ANCHORS_ONLY, 3, 20)) == POZYX_SUCCESS) {
		// Needed for discovery to not timeout or not find anything
		delay(3000);
	} else {
		std::cout << "Reading tags from file..." << std::endl;
		std::string line;
		while (std::getline(tagfd, line)) {
			device_coordinates_t newtag;
			std::vector<std::string> tagparams;
			char * parts;
			char * linestr = (char*)line.c_str();
			parts = strtok(linestr,"\t ");
			while (parts != NULL) {
				tagparams.push_back(std::string(parts));
				parts = strtok(NULL,"\t ");
			}
			if (tagparams.size() > 0) {
				std::stringstream converter(tagparams.at(0));
				short unsigned int tmp;
				converter >> std::hex >> tmp;
				std::cout << "Hex: " << std::hex << tmp;
				newtag.network_id = tmp;
				newtag.flag = 2;
				std::cout << "*";
			} else {
				break;
			}
			if (tagparams.size() > 2) {
				std::stringstream converter(tagparams.at(1)); // x in mm
				int tmp;
				converter >> std::dec >> tmp;
				std::cout << " x: " << std::dec << tmp;
				newtag.pos.x = tmp;
				std::stringstream converter2(tagparams.at(2)); // y in mm
				converter2 >> std::dec >> tmp;
				std::cout << " y: " << std::dec << tmp;
				newtag.pos.y = tmp;
			}
			if (tagparams.size() > 3) {
				std::stringstream converter(tagparams.at(3)); // z in mm
				int tmp;
				converter >> std::dec >> tmp;
				std::cout << " z: " << std::dec << tmp;
				newtag.pos.z = tmp;
			}
			if (Pozyx.addDevice(newtag) == POZYX_FAILURE) {
				std::cerr << "ERROR: Unable to add tag from file" << std::endl;
				break;
			}
			std::cout << std::endl;
		}
		fromfile = true;
		std::cout << std::endl;
	}


	std::cout << "Getting device list size..." << std::endl;
	uint8_t list_size;
	if (Pozyx.getDeviceListSize(&list_size) == POZYX_SUCCESS) {
		list_size_i = (int)list_size;
		std::cout << "Device list size: " << list_size_i << std::endl;

		if (Pozyx.getDeviceIds(devices, list_size_i) == POZYX_SUCCESS) {

			// sort array
			std::sort(devices, devices + list_size_i);

			// Set to use all anchors
			int status;
			std::cout << "Setting anchor selection..." << std::endl;
			if (fromfile) {
				status = Pozyx.setPositioningAnchorIds(devices, list_size_i);
				if (status == POZYX_SUCCESS) {
					status = Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, list_size_i);
					if (status != POZYX_SUCCESS) {
						std::cerr << "Couldn't set manual selection of anchors: ";
					}
				} else {
					std::cerr << "Couldn't set positioning anchors: ";
				}
			} else {
				status = Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, list_size_i > 15 ? 15 : list_size_i, 0);
				if (status == POZYX_SUCCESS) {
					std::cout << "Selected" << std::endl;
				} else {
					std::cerr << "Couldn't set anchor selection: ";
				}
			}
			if (status == POZYX_SUCCESS) {
				delay(2000);
				if (fromfile || Pozyx.doAnchorCalibration(POZYX_2D, 10, list_size_i > 6 ? 6 : list_size_i, devices) == POZYX_SUCCESS) {
					std::cout << "Calibrated" << std::endl;
					delay(1000);
					if (fromfile) {
						if (Pozyx.setPositionAlgorithm(POZYX_POS_ALG_UWB_ONLY, POZYX_3D) != POZYX_SUCCESS) {
							std::cout << "Couldn't set position algorithm";
						}
					}
					if (Pozyx.setUpdateInterval(100) == POZYX_SUCCESS) {
						done = true;
						std::cout << "Interval set" << std::endl;
					} else {
						std::cerr << "Couldn't start positioning: ";
					}
				} else {
					std::cerr << "Couldn't calibrate: ";
				}
			} else {
				std::cerr << "Couldn't set anchor selection: ";
			}
		} else {
			std::cerr << "Couldn't get device list size: ";
		}

	} else {
		std::cerr << "Couldn't get devices: ";
	}
	/*} else if (disc_status == POZYX_TIMEOUT) {
		std::cerr << "Couldn't discover (timeout)" << std::endl;
	} else {
		std::cerr << "Couldn't discover" << std::endl;
	}*/

	if (!done) {
		std::string error;
		error = Pozyx.getSystemError();
		std::cerr << "error: " << error << std::endl;
		throw 2;
	}

	//imu_pub_ = nh_.advertise<sensor_msgs::Imu>("pozyx/data",200,false);

	//magnetic_pub_ = nh_.advertise<sensor_msgs::MagneticField>("pozyx/mag",100,false);

	pos_pub_ = nh_.advertise<geometry_msgs::PointStamped>("pozyx/pos",100,false);

	range_pub_ = nh_.advertise<pozyx::StringStamped>("pozyx/range",100,false);

	//anchor_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pozyx/anchors",100,false);
}

void PozyxROS::update() {
	imu_frame_id_ = "pozyx";

	/*
	coordinates_t device_coords[list_size_i];

	for (int i = 0; i < list_size_i; i++) {
		Pozyx.getDeviceCoordinates(devices[i], &device_coords[i]);
	}

	sensor_msgs::PointCloud anchors;
	anchors.header.stamp = ros::Time::now();
	anchors.header.frame_id = imu_frame_id_;

	anchors.points.resize(list_size_i);

	anchors.channels.resize(1);
	anchors.channels[0].name = "identifiers";
	anchors.channels[0].values.resize(list_size_i);

	for (int i = 0; i < list_size_i; i++) {
		anchors.points[i].x = (double)device_coords[i].x / 1000.0;
		anchors.points[i].y = (double)device_coords[i].y / 1000.0;
		anchors.points[i].z = (double)device_coords[i].z / 1000.0;
		anchors.channels[0].values[i] = devices[i];
	}

	anchor_pub_.publish(anchors);
	*/

	std::cout << "Starting streaming data!" << std::endl;

	while (ros::ok()) {
		// get imu data

		//int16_t sensor_data[12];
		int32_t pos_data[3];
		device_range_t ranges[list_size_i];
		coordinates_t pos;
		bool pos_error = false;
		ros::Time current_time;


		// wait until this device gives an interrupt
		if (Pozyx.waitForFlag(POZYX_INT_STATUS_POS, 300, 0))
		{
			// we received an interrupt from pozyx telling us new IMU data is ready, now let's read it!
			//Pozyx.regRead(POZYX_ACCEL_X, (uint8_t*)&sensor_data, 9*sizeof(int16_t));
			current_time = ros::Time::now();
			//Pozyx.regRead(POZYX_QUAT_W, (uint8_t*)&sensor_data[9], 4*sizeof(int16_t));
			// also read out the calibration status
			//Pozyx.regRead(POZYX_CALIB_STATUS, &calib_status, 1);

			// Read last position (might be duplicate) -- change 3->9 to get error & covariance

			Pozyx.regRead(POZYX_POS_X, (uint8_t*)&pos_data, 3*sizeof(int32_t));

			for (int i = 0; i < list_size_i; i++) {
				int status;
				if (status = Pozyx.getDeviceRangeInfo(devices[i], &ranges[i]) == 0) {
					ranges[i].timestamp = 0;
				}
			}

			/*if (Pozyx.doPositioning(&pos, POZYX_2D, 0) != POZYX_SUCCESS) {
				pos_error = true;
			}*/

		}else{

			// we didn't receive an interrupt
			//uint8_t interrupt_status = 0;
			//Pozyx.regRead(POZYX_INT_STATUS, &interrupt_status, 1);

			continue;
		}

		//ros::Time current_time = ros::Time::now();

		// print out the presure (this is not an int16 but rather an uint32
		//uint32_t pressure = ((uint32_t)sensor_data[0]) + (((uint32_t)sensor_data[1])<<16);

		/*
		imu_msg.imu.header.stamp = current_time;
		imu_msg.imu.header.frame_id = imu_frame_id_;
		imu_msg.imu.orientation.x = (double)sensor_data[10] / 16384;
		imu_msg.imu.orientation.y = (double)sensor_data[11] / 16384;
		imu_msg.imu.orientation.z = (double)sensor_data[12] / 16384;
		imu_msg.imu.orientation.w = (double)sensor_data[9] / 16384;

		imu_msg.imu.angular_velocity.x = (double)sensor_data[6] * DEG_2_RAD / 16;
		imu_msg.imu.angular_velocity.y = (double)sensor_data[7] * DEG_2_RAD / 16;
		imu_msg.imu.angular_velocity.z = (double)sensor_data[8] * DEG_2_RAD / 16;

		imu_msg.imu.linear_acceleration.x = (double)sensor_data[0] * MG_2_MPSS;
		imu_msg.imu.linear_acceleration.y = (double)sensor_data[1] * MG_2_MPSS;
		imu_msg.imu.linear_acceleration.z = (double)sensor_data[2] * MG_2_MPSS;

		imu_pub_.publish(imu_msg);

		sensor_msgs::MagneticField msg;
		msg.header.frame_id = imu_frame_id_;
		msg.header.stamp = current_time;

		// compareable to xsens output
		msg.magnetic_field.x = (double)sensor_data[3] / (16 * 37);
		msg.magnetic_field.y = (double)sensor_data[4] / (16 * 37);
		msg.magnetic_field.z = (double)sensor_data[5] / (16 * 37);

		magnetic_pub_.publish(msg);
		*/

		pozyx::StringStamped range_msg;
		std::stringstream ss;

		for (int i = 0; i < list_size_i; i++) {
			if (ranges[i].timestamp != 0) {
				ss << "s=uwb,t=" << std::hex << std::setfill('0') << std::setw(4) << devices[i] << std::dec << ",tu=" << ranges[i].timestamp << ",ts=" << current_time.toNSec() / 1000000 << ",d=" << ranges[i].distance << ",RSS=" << ranges[i].RSS << "|";
			}
		}

		range_msg.data = ss.str();
		range_msg.header.frame_id = imu_frame_id_;
		range_msg.header.stamp = current_time;

		range_pub_.publish(range_msg);

		/*
		sensor_msgs::PointCloud anchors;
		anchors.header.stamp = ros::Time::now();
		anchors.header.frame_id = imu_frame_id_;

		anchors.points.resize(list_size_i);

		anchors.channels.resize(2);
		anchors.channels[0].name = "identifiers";
		anchors.channels[0].values.resize(list_size_i);

		anchors.channels[1].name = "distance";
		anchors.channels[1].values.resize(list_size_i);

		for (int i = 0; i < list_size_i; i++) {
			anchors.points[i].x = (double)device_coords[i].x / 1000.0;
			anchors.points[i].y = (double)device_coords[i].y / 1000.0;
			anchors.points[i].z = (double)device_coords[i].z / 1000.0;
			anchors.channels[0].values[i] = devices[i];
			anchors.channels[1].values[i] = ranges[i].distance;
		}

		anchor_pub_.publish(anchors);
		*/

		if (!pos_error) {

			geometry_msgs::PointStamped pos_msg;

			pos_msg.header.stamp = current_time;
			pos_msg.header.frame_id = imu_frame_id_;

			pos_msg.point.x = pos_data[0] / 1000.0f;
			pos_msg.point.y = pos_data[1] / 1000.0f;
			pos_msg.point.z = pos_data[2] / 1000.0f;

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

		if (sigcaught == 1) {
			return;
		}
	}
}

void signal_handler(int signal) {
	sigcaught = 1;
}

int main(int argc, char** argv) {
	//std::signal(SIGTERM, signal_handler);
	//std::signal(SIGINT, signal_handler);

	struct sigaction cleanup;
	cleanup.sa_handler = signal_handler;
	sigemptyset(&cleanup.sa_mask);
	//sigaddset(&cleanup.sa_mask, SIGTERM);
	cleanup.sa_flags = 0;

	sigaction(SIGINT, &cleanup, NULL);
	sigaction(SIGTERM, &cleanup, NULL);

	ros::init(argc, argv, "Pozyx_node", ros::init_options::NoSigintHandler);

	ROS_INFO("Pozyx Node for ROS");

	while (sigcaught == 0) {
		try {
			PozyxROS pozros;
			pozros.update();
			if (sigcaught == 1) {
				std::cout << "Resetting!" << std::endl;
				pozros.reset();
			}
		} catch (int e) {
			std::cerr << "Error: " << e << std::endl;
		}
		std::cout << "Sigcaught: " << sigcaught << std::endl;
	}
	return 0;
}

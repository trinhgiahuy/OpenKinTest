#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "pozyx");

	ros::NodeHandle n;

	ros::Publisher pozyx_pub = n.advertise<std_msgs::String>("imu_data", 1000);

	ros::Rate loop_rate(1);

	while (ros::ok()) {
		std_msgs::String msg;

		msg.data = "Test";

		pozyx_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}

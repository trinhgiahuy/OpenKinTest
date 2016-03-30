#!/usr/bin/env python

## Simple node to save imu and gps-data to ascii-file

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, NavSatFix
from ublox_msgs.msg import NavSOL

from decimal import *
getcontext().prec = 100

import time
filename = time.strftime("%Y-%m-%d-%H-%M-%S")
filename = "/home/ubuntu/data/" + filename + ".txt"

file = open(filename, "a")

def imucallback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'Imu-data: %s', str(data))
    #rospy.loginfo('IMU angular_vel: %f %f %f', data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
    line = "NaN\t{0}.{1}\t{2}\tNaN\tNaN\tNaN\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}".format(
        # gps seq
        data.header.stamp.secs,
        data.header.stamp.nsecs,
        data.header.seq,
        # gps latitude
        # longitude
        # altitude
        data.angular_velocity.x,
        data.angular_velocity.y,
        data.angular_velocity.z,
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w,
        data.linear_acceleration.x,
        data.linear_acceleration.y,
        data.linear_acceleration.z)

    writeline(line)

def gpscallback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'GPS-data: %s', str(data))
    #rospy.loginfo('GPS latitude: %f, longitude: %f', data.latitude, data.longitude)
    line = "{0}\t{1}.{2}\tNaN\t{3}\t{4}\t{5}\tNaN\tNaN\tNaN\tNaN\tNaN\tNaN\tNaN\tNaN\tNaN\tNaN".format(
        data.header.seq,
        data.header.stamp.secs,
        data.header.stamp.nsecs,
        # imu seq
        data.latitude,
        data.longitude,
        data.altitude
	# angular_velo.x
        # angular_vel.y
        # angular_vel.z
        # orientation.x
        # orientation.y
        # orientation.z
        # orientation.w
        # linear_acc.x
        # linear_acc.y
        # linear_acc.z
        )

    writeline(line)

def writeline(str_towrite):
    global file
    file.write(str_towrite+"\n")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ascii_logger', anonymous=True)

    rospy.Subscriber('imu/data', Imu, imucallback)
    rospy.Subscriber('gps/fix', NavSatFix, gpscallback)

    rospy.loginfo('Starting logging')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    file.close()

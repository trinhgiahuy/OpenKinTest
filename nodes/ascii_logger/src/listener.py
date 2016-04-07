#!/usr/bin/env python

## Simple node to save imu and gps-data to ascii-file

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, NavSatFix
#from ublox_msgs.msg import NavSOL

from decimal import *
getcontext().prec = 100

from collections import deque

import threading

import time
filename = time.strftime("%Y-%m-%d-%H-%M-%S")
filename = "/home/ubuntu/data/" + filename + ".txt"

file = open(filename, "a")

buffer = deque([])
bufferlock = threading.Lock()

counterlock = threading.Lock()
counter = 0

def imucallback(data):
    global buffer, counter

    #rospy.loginfo(rospy.get_caller_id() + 'Imu-data: %s', str(data))
    #rospy.loginfo('IMU angular_vel: %f %f %f', data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
    
    point = {'timestamp.secs': data.header.stamp.secs,
             'timestamp.nsecs': data.header.stamp.nsecs,
             'imuseq': data.header.seq,
             'ang.x': data.angular_velocity.x,
             'ang.y': data.angular_velocity.y,
             'ang.z': data.angular_velocity.z,
             'ori.x': data.orientation.x,
             'ori.y': data.orientation.y,
             'ori.z': data.orientation.z,
             'ori.w': data.orientation.w,
             'acc.x': data.linear_acceleration.x,
             'acc.y': data.linear_acceleration.y,
             'acc.z': data.linear_acceleration.z}

    # append to buffer
    bufferlock.acquire()
    buffer.append(point)
    bufferlock.release()

    # write buffer to file every now and then
    counter += 1
    if counter > 30:
        counter = 0
        writeBuffer()
    
def gpscallback(data):
    global buffer

    #rospy.loginfo(rospy.get_caller_id() + 'GPS-data: %s', str(data))
    #rospy.loginfo('GPS latitude: %f, longitude: %f', data.latitude, data.longitude)

    point = {'gpsseq': data.header.seq,
             'timestamp.secs': data.header.stamp.secs,
             'timestamp.nsecs': data.header.stamp.nsecs,
             'latitude': data.latitude,
             'longitude': data.longitude,
             'altitude': data.altitude}

    # write to buffer
    bufferlock.acquire()
    buffer.append(point)
    bufferlock.release()

def writeline(str_towrite):
    global file
    file.write(str_towrite+"\n")

def writeBuffer():
    global buffer

    # indexes of gps, imu -points to be joined
    joins = []
    # how many points to leave for next round
    leave = 1

    bufferlock.acquire()

    # check more than 20 samples in buffer
    # and find closest imu-points for gps-points
    if len(buffer) > 20:
        for i, j in enumerate(buffer):
            if 'gpsseq' in j:
                # test closest points for imu
                if i == 0:
                    # first, check next
                    if i+1 < len(buffer):
                        if 'imuseq' in buffer[i+1]:
                            # join j to buffer[i+1]
                            joins.append((i,i+1))
                elif i == len(buffer)-1:
                    # last in buffer, save gps and previous imu for next round
                    leave = 2
                else:
                    # if imu points
                    prev_imu = 'imuseq' in buffer[i-1]
                    next_imu = 'imuseq' in buffer[i+1]
                    if prev_imu and next_imu:
                        if abs((buffer[i-1]['timestamp.secs']+(1e-9*buffer[i-1]['timestamp.nsecs'])) -
                           (buffer[i]['timestamp.secs']+(1e-9*buffer[i]['timestamp.nsecs']))) < \
                           abs((buffer[i]['timestamp.secs']+(1e-9*buffer[i]['timestamp.nsecs'])) -
                           (buffer[i]['timestamp.secs']+(1e-9*buffer[i]['timestamp.nsecs']))):
                            # previous closer
                            joins.append((i, i-1))
                        else:
                            # next closer
                            joins.append((i, i+1))
                            if i == len(buffer)-2:
                                # dont leave fused point to buffer
                                leave = 0
                    elif prev_imu:
                        joins.append((i, i-1))
                    elif next_imu:
                        joins.append((i, i+1))
                    #else:
                        # No imu-points on sides, no join
    # pair gps-data with imu-data
    for j in joins:
        buffer[j[1]]['gpsseq'] = buffer[j[0]]['gpsseq']
        buffer[j[1]]['latitude'] = buffer[j[0]]['latitude']
        buffer[j[1]]['longitude'] = buffer[j[0]]['longitude']
        buffer[j[1]]['altitude'] = buffer[j[0]]['altitude']
        buffer[j[0]]['del'] = True

    buffer = [x for x in buffer if not 'del' in x]

    # write to file
    i = 0
    while (i < len(buffer)-leave):
        #rospy.loginfo(buffer)
        line = "{0}\t{1}.{2:09d}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}\t{16}".format(
          buffer[i].get('gpsseq', 'NaN'),
          buffer[i].get('timestamp.secs', 'NaN'),
          buffer[i].get('timestamp.nsecs', 'NaN'),
          buffer[i].get('imuseq', 'NaN'),
          buffer[i].get('latitude', 'NaN'),
          buffer[i].get('longitude', 'NaN'),
          buffer[i].get('altitude', 'NaN'),
          buffer[i].get('ang.x', 'NaN'),
          buffer[i].get('ang.y', 'NaN'),
          buffer[i].get('ang.z', 'NaN'),
          buffer[i].get('ori.x', 'NaN'),
          buffer[i].get('ori.y', 'NaN'),
          buffer[i].get('ori.z', 'NaN'),
          buffer[i].get('ori.w', 'NaN'),
          buffer[i].get('acc.x', 'NaN'),
          buffer[i].get('acc.y', 'NaN'),
          buffer[i].get('acc.z', 'NaN')
        )
        
        writeline(line)
        # remove written
        buffer.remove(buffer[i])

    bufferlock.release()

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

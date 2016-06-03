#!/usr/bin/env python

## Simple node to save imu and gps-data to ascii-file

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, NavSatFix
from ublox_msgs.msg import NavSOL, NavVELNED

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

counter = 0

def imucallback(data):
    global buffer, counter, bufferlock

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
    global buffer, bufferlock

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

def navvelnedcallback(data):
    global buffer, bufferlock

    #rospy.loginfo("NavVELNED: %s", str(data))

    point = {'iTOW': data.iTOW,
             'velN': data.velN,
             'velE': data.velE,
             'velD': data.velD,
             'speed': data.speed,
             'gSpeed': data.gSpeed,
             'heading': data.heading,
             'sAcc': data.sAcc,
             'cAcc': data.cAcc
            }

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
    # indexes of navvelned
    navvels = [] 
    # how many points to leave for next round
    leave = 1

    bufferlock.acquire()

    # check more than 20 samples in buffer
    # and find closest imu-points for gps-points
    if len(buffer) > 60:
        for i, j in enumerate(buffer):
            if 'gpsseq' in j and 'imuseq' not in j:
                #rospy.loginfo("gps in %s", i)
                # test closest points for imu
                if i == 0:
                    # first, check next
                    a = i;
                    while a+1 < len(buffer):
                        if 'imuseq' in buffer[a+1]:
                            # join j to buffer[i+1]
                            joins.append((i,a+1))
                            break
                        a += 1
                elif i >= len(buffer)-20:
                    # end of buffer, save gps and previous imu for next round
                    leave = 24
                else:
                    # if imu points
                    distp = 0
                    distn = 0
                    prev_imu = False
                    next_imu = False
                    while not prev_imu and (i-distp-1) >= 0:
                        distp += 1
                        prev_imu = 'imuseq' in buffer[i-distp]

                    while not next_imu and (i+distn+1) < len(buffer):
                        distn += 1
                        next_imu = 'imuseq' in buffer[i+distn]

                    if prev_imu and next_imu:
                        if abs((buffer[i-distp]['timestamp.secs']+(1e-9*buffer[i-distp]['timestamp.nsecs'])) -
                           (buffer[i]['timestamp.secs']+(1e-9*buffer[i]['timestamp.nsecs']))) < \
                           abs((buffer[i]['timestamp.secs']+(1e-9*buffer[i]['timestamp.nsecs'])) -
                           (buffer[i]['timestamp.secs']+(1e-9*buffer[i]['timestamp.nsecs']))):
                            # previous closer
                            joins.append((i, i-distp))
                        else:
                            # next closer
                            joins.append((i, i+distn))
                            #if i == len(buffer)-2:
                                # dont leave fused point to buffer
                                #leave = 0
                    elif prev_imu:
                        joins.append((i, i-distp))
                    elif next_imu:
                        joins.append((i, i+distn))
                    #else:
                        # No imu-points on sides, no join
            elif 'iTOW' in j and 'imuseq' not in j:
                # merge navvelned
                navvels.append(i)
                #rospy.loginfo("navvel in %s", i)
        

        tmpjoins = joins[:]

        for nav in navvels:
            if nav >= len(buffer)-20:
                #rospy.loginfo("Skipped join: %s, len: %s", nav, len(buffer))
                leave = 24
            else:
                closest = -1
                dist = 99999
                for j in tmpjoins:
                    if abs(j[1]-nav) < dist:
                        closest = j[1]
                        dist = abs(closest-nav)
                if closest != -1:
                    joins.append((nav, closest))
                    #rospy.loginfo("Join navvel: %s and imu: %s, len: %s", nav, closest, len(buffer))
            

        # pair gps-data with imu-data
        for j in joins:
            #rospy.loginfo("Join2: %s and %s, len: %s", j[0], j[1], len(buffer))
            if not 'gpsseq' in buffer[j[1]] and 'gpsseq' in buffer[j[0]]:
                buffer[j[1]]['gpsseq'] = buffer[j[0]].get('gpsseq', 'NaN')
                buffer[j[1]]['latitude'] = buffer[j[0]].get('latitude', 'NaN')
                buffer[j[1]]['longitude'] = buffer[j[0]].get('longitude', 'NaN')
                buffer[j[1]]['altitude'] = buffer[j[0]].get('altitude', 'NaN')
            if not 'iTOW' in buffer[j[1]] and 'iTOW' in buffer[j[0]]:
                buffer[j[1]]['iTOW'] = buffer[j[0]].get('iTOW', 'NaN')
                buffer[j[1]]['velN'] = buffer[j[0]].get('velN', 'NaN')
                buffer[j[1]]['velE'] = buffer[j[0]].get('velE', 'NaN')
                buffer[j[1]]['velD'] = buffer[j[0]].get('velD', 'NaN')
                buffer[j[1]]['speed'] = buffer[j[0]].get('speed', 'NaN')
                buffer[j[1]]['gSpeed'] = buffer[j[0]].get('gSpeed', 'NaN')
                buffer[j[1]]['heading'] = buffer[j[0]].get('heading', 'NaN')
                buffer[j[1]]['sAcc'] = buffer[j[0]].get('sAcc', 'NaN')
                buffer[j[1]]['cAcc'] = buffer[j[0]].get('cAcc', 'NaN')
    
            buffer[j[0]]['del'] = True
    
            #rospy.loginfo("GPS: %s", buffer[j[1]])


        buffer = [x for x in buffer if not 'del' in x]
    
        # write to file
        i = 0
        while (i < len(buffer)-leave):
            line = "{0}\t{1}.{2:09d}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}\t{16}\t{17}\t{18}\t{19}\t{20}\t{21}\t{22}\t{23}\t{24}\t{25}".format(
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
              buffer[i].get('acc.z', 'NaN'),
    
              # 17
              buffer[i].get('iTOW', 'NaN'),
              buffer[i].get('velN', 'NaN'),
              buffer[i].get('velE', 'NaN'),
              buffer[i].get('velD', 'NaN'),
              buffer[i].get('speed', 'NaN'),
              buffer[i].get('gSpeed', 'NaN'),
              buffer[i].get('heading', 'NaN'),
              buffer[i].get('sAcc', 'NaN'),
              buffer[i].get('cAcc', 'NaN')
            )
            
            #rospy.loginfo(line)

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
    rospy.Subscriber('gps/navvelned', NavVELNED, navvelnedcallback)

    rospy.loginfo('Starting logging')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    file.close()

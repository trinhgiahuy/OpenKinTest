#!/usr/bin/env python

## Simple node to save imu and gps-data to ascii-file

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix, MagneticField, Imu
from imu_sequenced.msg import ImuSequenced
from ublox_msgs.msg import NavPVT7 #, NavSOL, NavVELNED
from pozyx.msg import StringStamped

from decimal import *
getcontext().prec = 100

from collections import deque

import threading

import time
filename = time.strftime("%Y-%m-%d-%H-%M-%S")
filename = "/home/pi/data/" + filename + ".txt"

file = open(filename, "a")

buffer = deque([])
bufferlock = threading.Lock()

xsens_ids = []

counter = 0

def pozyxcallback(data):
    imucallback(data, True)

def xsenscallback(data):
    imucallback(data, False)

def imucallback(data, pozyx):
    global buffer, counter, bufferlock

    #rospy.loginfo(rospy.get_caller_id() + 'Imu-data: %s', str(data))
    #rospy.loginfo('IMU angular_vel: %f %f %f', data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)

    point = {'timestamp.secs': data.imu.header.stamp.secs,
             'timestamp.nsecs': data.imu.header.stamp.nsecs,
             'imuseq': data.seq,
             'frame_id': data.imu.header.frame_id,
             'ang.x': data.imu.angular_velocity.x,
             'ang.y': data.imu.angular_velocity.y,
             'ang.z': data.imu.angular_velocity.z,
             'ori.x': data.imu.orientation.x,
             'ori.y': data.imu.orientation.y,
             'ori.z': data.imu.orientation.z,
             'ori.w': data.imu.orientation.w,
             'acc.x': data.imu.linear_acceleration.x,
             'acc.y': data.imu.linear_acceleration.y,
             'acc.z': data.imu.linear_acceleration.z,
             'pozyx': "1" if pozyx else "0"}

    # append to buffer
    bufferlock.acquire()
    buffer.append(point)
    bufferlock.release()

    # write buffer to file every now and then
    counter += 1
    if counter > 360:
        counter = 0
        writeBuffer()

def gpscallback(data):
    global buffer, bufferlock, counter

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

    # write buffer to file every now and then
    counter += 1
    if counter > 360:
        counter = 0
        writeBuffer()

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

def navpvtcallback(data):
    global buffer, bufferlock

    #rospy.loginfo("NavVELNED: %s", str(data))

    point = {'iTOW': data.iTOW,
             #'year': data.year,
             #'month': data.month,
             #'day': data.day,
             #'hour': data.hour,
             #'min': data.min,
             #'sec': data.sec,
             #'valid': data.valid,
             #'tAcc': data.tAcc,
             #'nano': data.nano,
             #'fixType': data.fixType,
             #'flags': data.flags,
             #'flags2': data.flags2,
             #'numSV': data.numSV,
             #'lon': data.lon,
             #'lat': data.lat,
             #'height': data.height,
             #'hMSL': data.hMSL,
             #'hAcc': data.hAcc,
             #'vAcc': data.vAcc,
             'velN': data.velN,
             'velE': data.velE,
             'velD': data.velD,
             'gSpeed': data.gSpeed,
             'heading': data.heading,
             'sAcc': data.sAcc,
             'headAcc': data.headAcc,
             #'pDOP': data.pDOP,
             #'reserved1': data.reserved1
            }

    bufferlock.acquire()
    buffer.append(point)
    bufferlock.release()

def poscallback(data):
    global buffer, bufferlock, counter

    point = {'posseq': data.header.seq,
             'timestamp.secs': data.header.stamp.secs,
             'timestamp.nsecs': data.header.stamp.nsecs,
             'pos.x': data.point.x,
             'pos.y': data.point.y,
             'pos.z': data.point.z}

    bufferlock.acquire()
    buffer.append(point)
    bufferlock.release()

    # write buffer to file every now and then
    counter += 1
    if counter > 360:
        counter = 0
        writeBuffer()


def rangecallback(data):
    global buffer, bufferlock, counter

    point = {'rangeseq': data.header.seq,
             'timestamp.secs': data.header.stamp.secs,
             'timestamp.nsecs': data.header.stamp.nsecs,
             'ranges': data.data}

    bufferlock.acquire()
    buffer.append(point)
    bufferlock.release()

    # write buffer to file every now and then
    counter += 1
    if counter > 360:
        counter = 0
        writeBuffer()


def writeline(str_towrite):
    global file
    file.write(str_towrite+"\n")

def writeBuffer():
    global buffer

    # indexes of gps, imu -points to be joined
    joins = []

    navjoins = []
    # indexes of navvelned
    navvels = []
    # how many points to leave for next round
    leave = 70

    #rospy.loginfo("Running algo")

    bufferlock.acquire()

    # check more than 20 samples in buffer
    # and find closest imu-points for gps-points
    if len(buffer) > 180:
        #rospy.loginfo(">120")

        for i, j in enumerate(buffer):
            if ('frame_id' in j and not j['frame_id'] in xsens_ids):
                xsens_ids.append(j['frame_id'])
                xsens_ids.sort()

        for i, j in enumerate(buffer):
            if 'iTOW' in j and not 'timestamp.secs' in j:
                #rospy.loginfo("found navvel")
                distp = 0
                distn = 0
                prev_time = False
                next_time = False
                while not prev_time and (i-distp-1) >= 0:
                    distp += 1
                    prev_time = 'gpsseq' in buffer[i-distp] and 'timestamp.secs' in buffer[i-distp] and not 'iTOW' in buffer[i-distp]

                while not next_time and (i+distn+1) < len(buffer):
                    distn += 1
                    next_time = 'gpsseq' in buffer[i+distn] and 'timestamp.secs' in buffer[i+distn] and not 'iTOW' in buffer[i+distn]

                if prev_time and next_time:
                    if distp <= distn:
                        navjoins.append((i, i-distp))
                    else:
                        navjoins.append((i, i+distn))
                elif not prev_time and next_time:
                    navjoins.append((i, i+distn))
                    #buffer[i]['timestamp.secs'] = buffer[i+distn]['timestamp.secs']
                    #if buffer[i+distn]['timestamp.nsecs'] > 0:
                    #    buffer[i]['timestamp.nsecs'] = buffer[i+distn]['timestamp.nsecs']-1
                    #else:
                    #    buffer[i]['timestamp.nsecs'] = 999999999
                    #    buffer[i]['timestamp.secs'] -= 1
                #elif i >= len(buffer)-50:
                    # leave for next round
                    #continue
                elif prev_time:
                    navjoins.append((i, i-distp))
                    #buffer[i]['timestamp.secs'] = buffer[i-distp]['timestamp.secs']
                    #if buffer[i-distp]['timestamp.nsecs'] < 999999999:
                    #    buffer[i]['timestamp.nsecs'] = buffer[i-distp]['timestamp.nsecs']+1
                    #else:
                    #    buffer[i]['timestamp.nsecs'] = 0
                    #    buffer[i]['timestamp.secs'] += 1
                #else:
                    # no times around?
            if not 'timestamp.secs' in j or not 'timestamp.nsecs' in j:
                rospy.loginfo("No timestamp!: %s", j)

        for j in navjoins:
	    if 'iTOW' in buffer[j[0]]:
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


        buffer = [x for x in buffer if not 'del' in x]


        buffer = sorted(buffer, key=lambda x: x['timestamp.secs']+(1e-9*x['timestamp.nsecs']));

	#rospy.loginfo("Sorted")

        for i, j in enumerate(buffer):
            if (('gpsseq' in j or 'posseq' in j or 'rangeseq' in j) and not 'imuseq' in j) \
            or ('frame_id' in j and len(xsens_ids) > 1 and j['frame_id'] != xsens_ids[0] and (not 'pozyx' in j or j['pozyx'] == "0")):
                #rospy.loginfo("fusable in %s", i)
                #if 'frame_id' in j:
                    #rospy.loginfo("Fuse imus: ")
                    #rospy.loginfo(j)
                # test closest points for imu
                if i >= len(buffer)-50:
                    # end of buffer, save gps and previous imu for next round
                    #leave = 55
                    continue
                else:
                    # if imu points
                    distp = 0
                    distn = 0
                    prev_imu = False
                    next_imu = False
                    while not prev_imu and (i-distp-1) >= 0:
                        distp += 1
                        prev_imu = 'imuseq' in buffer[i-distp] and 'frame_id' in buffer[i-distp] and len(xsens_ids) > 0 and buffer[i-distp]['frame_id'] == xsens_ids[0]

                    while not next_imu and (i+distn+1) < len(buffer):
                        distn += 1
                        next_imu = 'imuseq' in buffer[i+distn] and 'frame_id' in buffer[i+distn] and len(xsens_ids) > 0 and buffer[i+distn]['frame_id'] == xsens_ids[0]

                    if prev_imu and next_imu:
                        if abs((buffer[i-distp]['timestamp.secs']+(1e-9*buffer[i-distp]['timestamp.nsecs'])) -
                           (buffer[i]['timestamp.secs']+(1e-9*buffer[i]['timestamp.nsecs']))) < \
                           abs((buffer[i+distn]['timestamp.secs']+(1e-9*buffer[i+distn]['timestamp.nsecs'])) -
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
                    else:
                        # No imu-points on sides, no join
                        continue
            #elif 'iTOW' in j and 'imuseq' not in j:
                # merge navvelned
                #navvels.append(i)
                #rospy.loginfo("navvel in %s", i)

        #tmpjoins = joins[:]

        #for nav in navvels:
        #    if nav >= len(buffer)-50:
                #rospy.loginfo("Skipped join: %s, len: %s", nav, len(buffer))
                #leave = 55
        #        continue
        #    else:
        #        closest = -1
        #        dist = 99999
        #        for j in tmpjoins:
                    # navvels only to gps points
        #            if 'gpsseq' in buffer[j[0]] and abs(j[1]-nav) < dist:
        #                closest = j[1]
        #                dist = abs(closest-nav)
        #        if closest != -1:
        #            joins.append((nav, closest))
                    #rospy.loginfo("Join navvel: %s and imu: %s, len: %s", nav, closest, len(buffer))



        # pair gps-data with imu-data
        for j in joins:
            #rospy.loginfo("Join2: %s and %s, len: %s", j[0], j[1], len(buffer))
            nodel = False
            if not 'imuseq' in buffer[j[1]]:
                rospy.loginfo("No imuseq in fusing point!: %s", buffer[j[1]])
                continue
            if 'imuseq' in buffer[j[0]] and 'frame_id' in buffer[j[0]]:
                #rospy.loginfo("Imuseq to be deleted!: %s", buffer[j[0]])
                #continue
                # Fusing multiple IMUS
                pindex = xsens_ids.index(buffer[j[0]]['frame_id'])
                #rospy.loginfo(pindex)
                #rospy.loginfo(buffer[j[0]])
                if 'imuseq'+str(pindex) in buffer[j[1]]:
                    # Already has fused point
                    nodel = True
                else:
                    buffer[j[1]]['imuseq'+str(pindex)] = buffer[j[0]].get('imuseq', 'NaN')
                    buffer[j[1]]['frame_id'+str(pindex)] = buffer[j[0]].get('frame_id', 'NaN')
                    buffer[j[1]]['ang.x'+str(pindex)] = buffer[j[0]].get('ang.x', 'NaN')
                    buffer[j[1]]['ang.y'+str(pindex)] = buffer[j[0]].get('ang.y', 'NaN')
                    buffer[j[1]]['ang.z'+str(pindex)] = buffer[j[0]].get('ang.z', 'NaN')
                    buffer[j[1]]['ori.x'+str(pindex)] = buffer[j[0]].get('ori.x', 'NaN')
                    buffer[j[1]]['ori.y'+str(pindex)] = buffer[j[0]].get('ori.y', 'NaN')
                    buffer[j[1]]['ori.z'+str(pindex)] = buffer[j[0]].get('ori.z', 'NaN')
                    buffer[j[1]]['ori.w'+str(pindex)] = buffer[j[0]].get('ori.w', 'NaN')
                    buffer[j[1]]['acc.x'+str(pindex)] = buffer[j[0]].get('acc.x', 'NaN')
                    buffer[j[1]]['acc.y'+str(pindex)] = buffer[j[0]].get('acc.y', 'NaN')
                    buffer[j[1]]['acc.z'+str(pindex)] = buffer[j[0]].get('acc.z', 'NaN')
            if 'gpsseq' in buffer[j[0]]:
                if 'gpsseq' in buffer[j[1]]:
                    nodel = True
                else:
                    buffer[j[1]]['gpsseq'] = buffer[j[0]].get('gpsseq', 'NaN')
                    buffer[j[1]]['latitude'] = buffer[j[0]].get('latitude', 'NaN')
                    buffer[j[1]]['longitude'] = buffer[j[0]].get('longitude', 'NaN')
                    buffer[j[1]]['altitude'] = buffer[j[0]].get('altitude', 'NaN')
            if 'iTOW' in buffer[j[0]]:
                if 'iTOW' in buffer[j[1]]:
                    nodel = True
                else:
                    buffer[j[1]]['iTOW'] = buffer[j[0]].get('iTOW', 'NaN')
                    buffer[j[1]]['velN'] = buffer[j[0]].get('velN', 'NaN')
                    buffer[j[1]]['velE'] = buffer[j[0]].get('velE', 'NaN')
                    buffer[j[1]]['velD'] = buffer[j[0]].get('velD', 'NaN')
                    buffer[j[1]]['speed'] = buffer[j[0]].get('speed', 'NaN')
                    buffer[j[1]]['gSpeed'] = buffer[j[0]].get('gSpeed', 'NaN')
                    buffer[j[1]]['heading'] = buffer[j[0]].get('heading', 'NaN')
                    buffer[j[1]]['sAcc'] = buffer[j[0]].get('sAcc', 'NaN')
                    buffer[j[1]]['cAcc'] = buffer[j[0]].get('cAcc', 'NaN')
            if 'posseq' in buffer[j[0]]:
                if 'posseq' in buffer[j[1]]:
                    nodel = True
                else:
                    buffer[j[1]]['posseq'] = buffer[j[0]].get('posseq', 'NaN')
                    buffer[j[1]]['pos.x'] = buffer[j[0]].get('pos.x', 'NaN')
                    buffer[j[1]]['pos.y'] = buffer[j[0]].get('pos.y', 'NaN')
                    buffer[j[1]]['pos.z'] = buffer[j[0]].get('pos.z', 'NaN')
            if 'rangeseq' in buffer[j[0]]:
                if 'rangeseq' in buffer[j[1]]:
                    nodel = True
                else:
                    buffer[j[1]]['rangeseq'] = buffer[j[0]].get('rangeseq', 'NaN')
                    buffer[j[1]]['ranges'] = buffer[j[0]].get('ranges', 'NaN')

            if not nodel:
                buffer[j[0]]['del'] = True

            #rospy.loginfo("GPS: %s", buffer[j[1]])


        buffer = [x for x in buffer if not 'del' in x]

        #rospy.loginfo("len to write: %d", len(buffer)-leave)

        # write to file
        i = 0
        while (i < len(buffer)-leave):
            line = line_formatter(buffer[i])
            #rospy.loginfo(line)
            writeline(line)
            # remove written
            buffer.remove(buffer[i])
    #rospy.loginfo("Written to file")
    bufferlock.release()

def line_formatter(point):
    templ = "{0}\t{1}.{2:09d}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}\t{16}\t{17}\t{18}\t{19}\t{20}\t{21}\t{22}\t{23}\t{24}\t{25}\t{26}\t{27}\t{28}\t{29}\t{30}\t{31}\t{32}"
    if len(xsens_ids) > 1:
        for i in range(33, 12*(len(xsens_ids)-1) +33):
            #for a in range(0,11):
            templ += "\t{"+str(i)+"}"
            #i += 12

    #rospy.loginfo(xsens_ids)
    #rospy.loginfo(templ)


    #rospy.loginfo(point)

    vals = [
      point.get('gpsseq', 'NaN'),
      point.get('timestamp.secs', 'NaN'),
      point.get('timestamp.nsecs', 'NaN'),
      point.get('imuseq', 'NaN'),
      point.get('latitude', 'NaN'),
      point.get('longitude', 'NaN'),
      point.get('altitude', 'NaN'),
      point.get('ang.x', 'NaN'),
      point.get('ang.y', 'NaN'),
      point.get('ang.z', 'NaN'),
      point.get('ori.x', 'NaN'),
      point.get('ori.y', 'NaN'),
      point.get('ori.z', 'NaN'),
      point.get('ori.w', 'NaN'),
      point.get('acc.x', 'NaN'),
      point.get('acc.y', 'NaN'),
      point.get('acc.z', 'NaN'),

      # 17
      point.get('iTOW', 'NaN'),
      point.get('velN', 'NaN'),
      point.get('velE', 'NaN'),
      point.get('velD', 'NaN'),
      point.get('speed', 'NaN'), # not in PVT
      point.get('gSpeed', 'NaN'),
      point.get('heading', 'NaN'),
      point.get('sAcc', 'NaN'),
      #point.get('cAcc', 'NaN'),
      point.get('headAcc', 'NaN'),

      # 26
      point.get('pozyx', 'NaN'),
      point.get('posseq', 'NaN'),
      point.get('pos.x', 'NaN'),
      point.get('pos.y', 'NaN'),
      point.get('pos.z', 'NaN'),

      # 31 ranges
      point.get('ranges', 'NaN'),

      # 32 imu frame_id
      point.get('frame_id', 'NaN')
      # 33 multiple imus...
    ]

    if len(xsens_ids) > 1:
        #rospy.loginfo(len(xsens_ids))
        for i in range(1, len(xsens_ids)):
            #rospy.loginfo(i)
            #rospy.loginfo(point)
            vals.append(point.get('imuseq'+str(i), 'NaN'))
            vals.append(point.get('frame_id'+str(i), 'NaN'))
            vals.append(point.get('ang.x'+str(i), 'NaN'))
            vals.append(point.get('ang.y'+str(i), 'NaN'))
            vals.append(point.get('ang.z'+str(i), 'NaN'))
            vals.append(point.get('ori.x'+str(i), 'NaN'))
            vals.append(point.get('ori.y'+str(i), 'NaN'))
            vals.append(point.get('ori.z'+str(i), 'NaN'))
            vals.append(point.get('ori.w'+str(i), 'NaN'))
            vals.append(point.get('acc.x'+str(i), 'NaN'))
            vals.append(point.get('acc.y'+str(i), 'NaN'))
            vals.append(point.get('acc.z'+str(i), 'NaN'))

    line = templ.format(*vals)

    return line

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ascii_logger', anonymous=True)

    rospy.Subscriber('imu/data', ImuSequenced, xsenscallback)
    rospy.Subscriber('gps/fix', NavSatFix, gpscallback)
    #rospy.Subscriber('gps/navvelned', NavVELNED, navvelnedcallback)
    rospy.Subscriber('gps/navpvt', NavPVT7, navpvtcallback)
    rospy.Subscriber('pozyx/data', ImuSequenced, pozyxcallback)
    rospy.Subscriber('pozyx/pos', PointStamped, poscallback)
    #rospy.Subscriber('pozyx/mag', MagneticField, magcallback)
    rospy.Subscriber('pozyx/range', StringStamped, rangecallback)


    rospy.loginfo('Starting logging')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    file.close()

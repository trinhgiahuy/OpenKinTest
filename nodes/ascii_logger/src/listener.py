#!/usr/bin/env python

## Simple node to save imu and gps-data to ascii-file

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix, MagneticField, Imu
from imu_sequenced.msg import ImuSequenced
from ublox_msgs.msg import NavPVT7wH #, NavSOL, NavVELNED
from pozyx.msg import StringStamped
from vectornav.msg import imugps

#from decimal import *
#getcontext().prec = 100

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

def vectornavcallback(data):
    global buffer, counter, bufferlock

    point = {'timestamp.secs': data.header.stamp.secs,
             'timestamp.nsecs': data.header.stamp.nsecs,
             'imuseq': data.header.seq,
             'frame_id': data.header.frame_id,
             'ang.x': data.angular_velocity.x,
             'ang.y': data.angular_velocity.y,
             'ang.z': data.angular_velocity.z,
             'ori.x': data.orientation.x,
             'ori.y': data.orientation.y,
             'ori.z': data.orientation.z,
             'ori.w': data.orientation.w,
             'acc.x': data.linear_acceleration.x,
             'acc.y': data.linear_acceleration.y,
             'acc.z': data.linear_acceleration.z,
             'gpstime': data.time,
             'latitude': data.LLA.x,
             'longitude': data.LLA.y,
             'altitude': data.LLA.z,
             'velN': data.nedvel.x,
             'velE': data.nedvel.y,
             'velD': data.nedvel.z,
             'dtime': data.dtime,
             'dtheta.x': data.dtheta[0],
             'dtheta.y': data.dtheta[1],
             'dtheta.z': data.dtheta[2],
             'dvel.x': data.dvel[0],
             'dvel.y': data.dvel[1],
             'dvel.z': data.dvel[2],
             'gpslatitude': data.gpsLLA.x,
             'gpslongitude': data.gpsLLA.y,
             'gpsaltitude': data.gpsLLA.z,
             'gpsvelN': data.gpsnedvel.x,
             'gpsvelE': data.gpsnedvel.y,
             'gpsvelD': data.gpsnedvel.z,
             'fix': data.fix }

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

    point = {'timestamp.secs': data.header.stamp.secs,
             'timestamp.nsecs': data.header.stamp.nsecs,
             'pvtseq': data.header.seq,
             'iTOW': data.pvt.iTOW,
             #'year': data.pvt.year,
             #'month': data.pvt.month,
             #'day': data.pvt.day,
             #'hour': data.pvt.hour,
             #'min': data.pvt.min,
             #'sec': data.pvt.sec,
             #'valid': data.pvt.valid,
             #'tAcc': data.pvt.tAcc,
             #'nano': data.pvt.nano,
             #'fixType': data.pvt.fixType,
             #'flags': data.pvt.flags,
             #'flags2': data.pvt.flags2,
             #'numSV': data.pvt.numSV,
             #'lon': data.pvt.lon,
             #'lat': data.pvt.lat,
             #'height': data.pvt.height,
             #'hMSL': data.pvt.hMSL,
             #'hAcc': data.pvt.hAcc,
             #'vAcc': data.pvt.vAcc,
             'velN': data.pvt.velN/10.0,
             'velE': data.pvt.velE/10.0,
             'velD': data.pvt.velD/10.0,
             'gSpeed': data.pvt.gSpeed/10.0,
             'heading': data.pvt.heading,
             'sAcc': data.pvt.sAcc/10.0,
             'headAcc': data.pvt.headAcc,
             #'pDOP': data.pvt.pDOP,
             #'reserved1': data.pvt.reserved1
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
            if 'iTOW' in j and 'timestamp.nsecs' in j:
                #rospy.loginfo("found navvel")
                dist = 0
                prev_time = False
                next_time = False
                while not (prev_time or next_time) and dist <= 10:
                    dist += 1
                    if i-dist >= 0:
                        prev_time = 'gpsseq' in buffer[i-dist] and 'timestamp.nsecs' in buffer[i-dist] and buffer[i-dist]['timestamp.nsecs'] == j['timestamp.nsecs']
                    if i+dist < len(buffer):
                        next_time = 'gpsseq' in buffer[i+dist] and 'timestamp.nsecs' in buffer[i+dist] and buffer[i+dist]['timestamp.nsecs'] == j['timestamp.nsecs']

                if prev_time and next_time:
                    navjoins.append((i, i-dist))
                elif not prev_time and next_time:
                    navjoins.append((i, i+dist))
                elif prev_time:
                    navjoins.append((i, i-dist))

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
                buffer[j[1]]['headAcc'] = buffer[j[0]].get('headAcc', 'NaN')
                buffer[j[0]]['del'] = True


        buffer = [x for x in buffer if not 'del' in x]


        buffer = sorted(buffer, key=lambda x: x['timestamp.secs']+(1e-9*x['timestamp.nsecs']));

	#rospy.loginfo("Sorted")

        for i, j in enumerate(buffer):
            if (('gpsseq' in j or 'posseq' in j or 'rangeseq' in j) and not 'imuseq' in j) \
            or ('frame_id' in j and len(xsens_ids) > 1 and j['frame_id'] != xsens_ids[0] and (not 'gpstime' in j or not 'pozyx' in j or j['pozyx'] == "0")):
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
                        if 'frame_id' in j:
                            joins.append((i,i))
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
                    buffer[j[0]]['imuseq'+str(pindex)] = buffer[j[0]].get('imuseq', 'NaN')
                    buffer[j[0]]['frame_id'+str(pindex)] = buffer[j[0]].get('frame_id', 'NaN')
                    buffer[j[0]]['ang.x'+str(pindex)] = buffer[j[0]].get('ang.x', 'NaN')
                    buffer[j[0]]['ang.y'+str(pindex)] = buffer[j[0]].get('ang.y', 'NaN')
                    buffer[j[0]]['ang.z'+str(pindex)] = buffer[j[0]].get('ang.z', 'NaN')
                    buffer[j[0]]['ori.x'+str(pindex)] = buffer[j[0]].get('ori.x', 'NaN')
                    buffer[j[0]]['ori.y'+str(pindex)] = buffer[j[0]].get('ori.y', 'NaN')
                    buffer[j[0]]['ori.z'+str(pindex)] = buffer[j[0]].get('ori.z', 'NaN')
                    buffer[j[0]]['ori.w'+str(pindex)] = buffer[j[0]].get('ori.w', 'NaN')
                    buffer[j[0]]['acc.x'+str(pindex)] = buffer[j[0]].get('acc.x', 'NaN')
                    buffer[j[0]]['acc.y'+str(pindex)] = buffer[j[0]].get('acc.y', 'NaN')
                    buffer[j[0]]['acc.z'+str(pindex)] = buffer[j[0]].get('acc.z', 'NaN')
                    buffer[j[0]].pop('imuseq', None)
                    buffer[j[0]].pop('frame_id', None)
                    buffer[j[0]].pop('ang.x', None)
                    buffer[j[0]].pop('ang.y', None)
                    buffer[j[0]].pop('ang.z', None)
                    buffer[j[0]].pop('ori.x', None)
                    buffer[j[0]].pop('ori.y', None)
                    buffer[j[0]].pop('ori.z', None)
                    buffer[j[0]].pop('ori.w', None)
                    buffer[j[0]].pop('acc.x', None)
                    buffer[j[0]].pop('acc.y', None)
                    buffer[j[0]].pop('acc.z', None)
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
                    if j[0] == j[1]:
                        nodel = True
                        buffer[j[0]].pop('imuseq', None)
                        buffer[j[0]].pop('frame_id', None)
                        buffer[j[0]].pop('ang.x', None)
                        buffer[j[0]].pop('ang.y', None)
                        buffer[j[0]].pop('ang.z', None)
                        buffer[j[0]].pop('ori.x', None)
                        buffer[j[0]].pop('ori.y', None)
                        buffer[j[0]].pop('ori.z', None)
                        buffer[j[0]].pop('ori.w', None)
                        buffer[j[0]].pop('acc.x', None)
                        buffer[j[0]].pop('acc.y', None)
                        buffer[j[0]].pop('acc.z', None)
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
                    buffer[j[1]]['headAcc'] = buffer[j[0]].get('headAcc', 'NaN')
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
    templ = "{0}\t{1}.{2:09d}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}\t{16}\t{17}\t{18}\t{19}\t{20}\t{21}\t{22}\t{23}\t{24}\t{25}\t{26}\t{27}\t{28}\t{29}\t{30}\t{31}\t{32}\t{33}\t{34}\t{35}\t{36}\t{37}\t{38}\t{39}\t{40}\t{41}\t{42}\t{43}\t{44}\t{45}\t{46}\t{47}"
    if len(xsens_ids) > 1:
        for i in range(33+8, 12*(len(xsens_ids)-1) +33+8):
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
      point.get('gpstime', 'NaN'),
      point.get('dtime', 'NaN'),
      point.get('dtheta.x', 'NaN'),
      point.get('dtheta.y', 'NaN'),
      point.get('dtheta.z', 'NaN'),
      point.get('dvel.x', 'NaN'),
      point.get('dvel.y', 'NaN'),
      point.get('dvel.z', 'NaN'),
      point.get('gpsvelN', 'NaN'),
      point.get('gpsvelE', 'NaN'),
      point.get('gpsvelD', 'NaN'),
      point.get('gpslatitude', 'NaN'),
      point.get('gpslongitude', 'NaN'),
      point.get('gpsaltitude', 'NaN'),
      point.get('fix', 'NaN'),

      # 41
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
    rospy.Subscriber('gps/navpvtwh', NavPVT7wH, navpvtcallback)
    rospy.Subscriber('pozyx/data', ImuSequenced, pozyxcallback)
    rospy.Subscriber('pozyx/pos', PointStamped, poscallback)
    #rospy.Subscriber('pozyx/mag', MagneticField, magcallback)
    rospy.Subscriber('pozyx/range', StringStamped, rangecallback)

    rospy.Subscriber('vectornav/imugps', imugps, vectornavcallback)


    rospy.loginfo('Starting logging')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    file.close()

 #!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import time

import pynmea2
from std_msgs.msg import Header
from gps_common.msg import GPSFix


import numpy as np
import math


'''
X轴正方向为北，Y轴正方向为东。
'''

CONSTANTS_RADIUS_OF_EARTH = 6371000


def GPStoXY(lat, lon, ref_lat=30.258824339333334, ref_lon=119.72750057183333):
        # input GPS and Reference GPS in degrees
        # output XY in meters (m) X:North Y:East
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        cos_d_lon = math.cos(lon_rad - ref_lon_rad)

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
        c = math.acos(arg)

        k = 1.0
        if abs(c) > 0:
            k = (c / math.sin(c))

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH)

        return x, y





class RTK:
    def __init__(self, port='/dev/ttyUSB0', ros_pub=None):
        self.serial = serial.Serial(port, 230400)
        self.ros_pub = ros_pub
        if self.ros_pub is None:
            print('No ros_pub !')

        self.speed = 0.
        self.yaw = 0.
        self.latitude = 0.
        self.longitude = 0.

        self.msg_seq = 0

        recv_thread = threading.Thread(
            target = self.read_serial, args=()
        )
        recv_thread.start()

    def read_serial(self):
        while True:
            line = self.serial.readline()
            # print(line[:6])
            # print(line[:6]== b'$GPRMC')
            # line = line.decode('utf8')
            # line = line[1:]
            # print(line)
            # line = '$GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20'
            self.parse(line)
            ros_msg = GPSFix()
            ros_msg.header = Header()
            ros_msg.header.seq = self.msg_seq

            ros_msg.header.stamp = rospy.Time.from_sec(time.time() - 20.0)

            ros_msg.latitude = self.latitude
            ros_msg.longitude = self.longitude
            ros_msg.track = self.yaw
            ros_msg.speed = self.speed
            # print(ros_msg)
            # print('\n\n\n')
            self.ros_pub.publish(ros_msg)
        

    def parse(self, data):
        if data[:6] == b'$GPRMC':
            self.msg_seq += 1
            # print('in here')
            data = data.decode('utf8')
        # if data.startswith('$GPRMC'):
            rmc = pynmea2.parse(data)
            # print('\n')
            # print(rmc)
            # print(rmc.spd_over_grnd, rmc.true_course)
            # print('\n')
            self.latitude = rmc.latitude
            self.longitude = rmc.longitude
            if rmc.spd_over_grnd != None:
                self.speed = rmc.spd_over_grnd
            if rmc.true_course != None:
                self.yaw = rmc.true_course
           # print(self.yaw)


if __name__ == '__main__':
    rospy.init_node('rtk', anonymous=False)
    rtk_pub = rospy.Publisher('/rtk_data', GPSFix, queue_size=1)

    rtk = RTK('/dev/ttyUSB0', rtk_pub)

    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()

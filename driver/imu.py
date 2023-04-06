import rldev

import os, sys
sys.path.insert(0, os.path.expanduser('~/github/zdk/tractor-control'))

import rospy
import serial
import threading
import time

from datetime import datetime, timedelta
import pytz

import pynmea2
from std_msgs.msg import Header, String
from gps_common.msg import GPSFix
from sensor_msgs.msg import Imu


from driver.rtk import PseudoPublisher
from driver.rtk import gps2time





class IMU(object):
    def __init__(self, rospub=False):
        self.serial = serial.Serial('/dev/ttyUSB0', 460800)
        
        if rospub:
            self.publisher_imu = rospy.Publisher('/imu_data', Imu, queue_size=1)
            self.publisher_imu_full = rospy.Publisher('/imu_data_full', String, queue_size=1)
        else:
            self.publisher_imu = PseudoPublisher()
            self.publisher_imu_full = PseudoPublisher()

        self.imu_data = None
        self.stop_recv = threading.Event()
        recv_thread = threading.Thread(target=self.read_serial)
        self.stop_recv.clear()
        recv_thread.start()


    def stop_event(self):
        self.stop_recv.set()

    def close(self):
        self.serial.close()



    def read_serial(self):
        msg_seq = 0
        while not self.stop_recv.is_set():
            line = self.serial.readline()

            if line[:6] != b'$GTIMU':
                continue
            string = line.decode('utf8').replace('\x00', '')

            full_data = String()
            full_data.data = string
            self.publisher_imu_full.publish(full_data)

            msg_seq += 1

            try:
                data = self.string2data(string)
            except:
                import traceback
                traceback.print_exc()
                print('wrong data:', string)
                continue

            ros_msg = Imu()
            ros_msg.header = Header()
            ros_msg.header.seq = msg_seq
            ros_msg.header.stamp = rospy.Time.from_sec(time.time())

            ros_msg.angular_velocity.x = data.wx
            ros_msg.angular_velocity.y = data.wy
            ros_msg.angular_velocity.z = data.wz
            ros_msg.linear_acceleration.x = data.ax
            ros_msg.linear_acceleration.y = data.ay
            ros_msg.linear_acceleration.z = data.az
            self.imu_data = ros_msg
            self.publisher_imu.publish(ros_msg)



    def string2data(self, string):
        data_list = string.split(',')
        # print('[GPCHC] data: ', data_list)

        time_week = float(data_list[2-1])
        time_sec = float(data_list[3-1])
        current_time = gps2time(time_week, time_sec)
        print('current_time: ', current_time)

        wx = float(data_list[4 -1])
        wy = float(data_list[5 -1])
        wz = float(data_list[6 -1])
        ax = float(data_list[7 -1]) *10
        ay = float(data_list[8 -1]) *10
        az = float(data_list[9 -1]) *10
        return rldev.BaseData(wx=wx, wy=wy, wz=wz, ax=ax, ay=ay, az=az)





if __name__ == '__main__':
    rospy.init_node('imu_driver', anonymous=False)
    rtk = IMU(rospub=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

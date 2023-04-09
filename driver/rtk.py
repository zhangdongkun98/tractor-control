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


class PseudoPublisher(object):
    def publish(self, msg):
        return



def gps2time(time_week, time_sec, leap_seconds=18):
    gps_epoch = datetime(1980, 1, 6, tzinfo=pytz.utc)
    current_time =  gps_epoch + timedelta(weeks=time_week, milliseconds=time_sec *1000, seconds=-leap_seconds) + timedelta(hours=8)
    # current_time = current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    current_time = current_time.strftime('%Y-%m-%d %H:%M:%S.%f')
    return current_time



class RTK(object):
    def __init__(self, rospub=False):
        self.serial = serial.Serial('/dev/ttyUSB0', 460800)
        
        if rospub:
            self.publisher_rtk = rospy.Publisher('/rtk_data', GPSFix, queue_size=1)
            self.publisher_rtk_full = rospy.Publisher('/rtk_data_full', String, queue_size=1)
        else:
            self.publisher_rtk = PseudoPublisher()
            self.publisher_rtk_full = PseudoPublisher()

        self.gps_data = None
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

            if line[:6] != b'$GPCHC':
                continue
            string = line.decode('utf8').replace('\x00', '')

            full_data = String()
            full_data.data = string
            self.publisher_rtk_full.publish(full_data)

            msg_seq += 1

            try:
                data = self.string2data(string)
            except ValueError as e:
                import traceback
                traceback.print_exc()
                print('wrong data:', string)
                # import pdb; pdb.set_trace()
                continue
            except:
                import traceback
                traceback.print_exc()
                print('wrong data:', string)
                continue




            ros_msg = GPSFix()
            ros_msg.header = Header()
            ros_msg.header.seq = msg_seq
            ros_msg.header.stamp = rospy.Time.from_sec(time.time())

            ros_msg.latitude = data.lat
            ros_msg.longitude = data.lon
            ros_msg.track = data.heading
            ros_msg.speed = data.speed
            # print(ros_msg)
            # print('\n\n\n')
            self.gps_data = ros_msg
            self.publisher_rtk.publish(ros_msg)




    def string2data(self, string):
        data_list = string.split(',')
        # print('[GPCHC] data: ', data_list)

        time_week = float(data_list[2-1])
        time_sec = float(data_list[3-1])
        current_time = gps2time(time_week, time_sec)
        # print('current_time: ', current_time)

        lat = float(data_list[13 -1])
        lon = float(data_list[14 -1])
        heading = float(data_list[4 -1])
        speed = float(data_list[19 -1])
        return rldev.BaseData(lat=lat, lon=lon, heading=heading, speed=speed)







if __name__ == '__main__':
    rospy.init_node('rtk_driver', anonymous=False)
    rtk = RTK(rospub=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

 #!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import time

import pynmea2
from std_msgs.msg import Header
from gps_common.msg import GPSFix



class RTK(object):
    def __init__(self, port='/dev/ttyUSB0'):
        self.serial = serial.Serial(port, 230400)
        
        self.ros_pub = rospy.Publisher('/rtk_data', GPSFix, queue_size=1)

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
            data = self.serial.readline()
            if data[:6] != b'$GPRMC':
                continue
                # raise NotImplementedError

            rmc = pynmea2.parse(data.decode('utf8'))
            msg_seq += 1

            ros_msg = GPSFix()
            ros_msg.header = Header()
            ros_msg.header.seq = msg_seq
            ros_msg.header.stamp = rospy.Time.from_sec(time.time())

            ros_msg.latitude = rmc.latitude
            ros_msg.longitude = rmc.longitude
            ros_msg.track = rmc.true_course
            ros_msg.speed = rmc.spd_over_grnd
            # print(ros_msg)
            # print('\n\n\n')
            self.ros_pub.publish(ros_msg)




if __name__ == '__main__':
    rospy.init_node('rtk_driver', anonymous=False)
    rtk = RTK()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

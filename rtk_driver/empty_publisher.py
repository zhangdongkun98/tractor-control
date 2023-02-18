
import rospy
# import serial
import threading
import time

# import pynmea2
from std_msgs.msg import Header
from gps_common.msg import GPSFix


import os, sys
print(GPSFix)
print(os.path.abspath(sys.modules[GPSFix.__module__].__file__))


class RTK(object):
    def __init__(self, port='/dev/ttyUSB0'):
        # self.serial = serial.Serial(port, 230400)

        self.speed = 0.
        self.yaw = 0.
        self.latitude = 0.
        self.longitude = 0.

        self.msg_seq = 0

  

    def create_msg(self):
        ros_msg = GPSFix()
        # ros_msg.header = Header()
        ros_msg.header.seq = self.msg_seq

        # ros_msg.header.stamp = rospy.Time.from_sec(time.time() - 20.0)
        ros_msg.header.stamp = rospy.Time.from_sec(time.time())

        ros_msg.latitude = self.latitude
        ros_msg.longitude = self.longitude
        ros_msg.track = self.yaw
        ros_msg.speed = self.speed

        self.msg_seq += 1
        return ros_msg




if __name__ == '__main__':
    rospy.init_node('rtk_publisher', anonymous=False)
    rtk_pub = rospy.Publisher('/rtk_data', GPSFix, queue_size=1)

    rtk = RTK('/dev/ttyUSB0')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = rtk.create_msg()
        # print(msg)
        rtk_pub.publish(msg)
        rate.sleep()

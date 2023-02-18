
import rospy
import threading
import time

import matplotlib.pyplot as plt

from std_msgs.msg import Header
from gps_common.msg import GPSFix




class RTK(object):
    def __init__(self):

        self.speed = 0.
        self.yaw = 0.
        self.latitude = 0.
        self.longitude = 0.

        rtk_pub = rospy.Subscriber('/rtk_data', GPSFix, callback=self.callback, queue_size=1)
        



    def callback(self, msg):
        print(msg)
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        return



if __name__ == '__main__':
    rospy.init_node('rtk_subsriber', anonymous=False)

    rtk = RTK()
    time.sleep(1.0)

    rate = rospy.Rate(10)
    plt.gca().set_aspect('equal')
    while not rospy.is_shutdown():
        plt.plot(rtk.latitude, rtk.longitude, 'or')
        plt.pause(0.001)
        rate.sleep()



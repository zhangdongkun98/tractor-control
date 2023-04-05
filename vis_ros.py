import carla_utils as cu
import carla_utils.ros as ru
from carla_utils import carla, navigation
RoadOption = navigation.local_planner.RoadOption

import time

import numpy as np
import matplotlib.pyplot as plt


from driver.projection import projection
from driver.rtk import RTK
from envs.env_agri import get_global_path

import rospy
from gps_common.msg import GPSFix




class RTK(object):
    def __init__(self):
        self.speed = 0.
        self.yaw = 0.
        self.latitude = 0.
        self.longitude = 0.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rtk_pub = rospy.Subscriber('/rtk_data', GPSFix, callback=self.callback, queue_size=1)


    def callback(self, msg):
        print(msg)
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.x, self.y = projection.gps2xy(msg.latitude, msg.longitude)
        self.theta = projection.track2yaw(msg.track)

        return

    def get_pose(self):
        
        pass



if __name__ == '__main__':
    rtk = RTK()
    time.sleep(3.0)

    clock = cu.system.Clock(100)
    foreground = []
    # plt.gca().set_aspect('equal')

    gp = get_global_path()
    plt.plot(gp.x, gp.y, 'ob')

    while True:
        pass


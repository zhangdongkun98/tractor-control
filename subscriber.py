import carla_utils as cu
from carla_utils import carla, navigation
RoadOption = navigation.local_planner.RoadOption

import rospy
import threading
import time

import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header
from gps_common.msg import GPSFix


from driver.projection import Projection



from envs.env_carla import SR, FREQ
from envs.env_agri import PseudoWaypoint
def get_global_path():
    start_x, start_y = 6.058451788499951, -29.40062252106145
    end_x, end_y = 6.058451788499951, 0.0

    length = np.hypot(end_x-start_x, end_y-start_y)
    theta = np.arctan2(end_y-start_y, end_x-start_x)
    route = []
    current_length = 0.0
    while True:
        if current_length > length:
            break
        x = start_x + current_length * np.cos(theta)
        y = start_y + current_length * np.sin(theta)
        wp = PseudoWaypoint(x, y, theta)
        route.append((wp, RoadOption.LANEFOLLOW))
        current_length += SR
    global_path = cu.GlobalPath(route)
    return global_path



class RTK(object):
    def __init__(self):
        self.projection = Projection()

        self.speed = 0.
        self.yaw = 0.
        self.latitude = 0.
        self.longitude = 0.

        rtk_pub = rospy.Subscriber('/rtk_data', GPSFix, callback=self.callback, queue_size=1)


    def callback(self, msg):
        print(msg)
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.x, self.y = self.projection.gps2xy(msg.latitude, msg.longitude)

        return





if __name__ == '__main__':
    rospy.init_node('rtk_subsriber', anonymous=False)

    rtk = RTK()
    time.sleep(1.0)

    rate = rospy.Rate(100)
    foreground = []
    plt.gca().set_aspect('equal')

    gp = get_global_path()
    plt.plot(gp.x, gp.y, 'ob')

    while not rospy.is_shutdown():
        [i.remove() for i in foreground]
        # plt.plot(rtk.latitude, rtk.longitude, 'or')
        line = plt.plot(rtk.x, rtk.y, 'or')[0]
        foreground.append(line)
        plt.pause(0.001)
        rate.sleep()



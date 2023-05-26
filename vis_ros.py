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
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path, Odometry
from gps_common.msg import GPSFix




class RTKSub(object):
    def __init__(self):
        self.gp_x = []
        self.gp_y = []
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rospy.Subscriber('/rtk_data', GPSFix, callback=self.callback, queue_size=1)
        rospy.Subscriber('/global_path', Path, callback=self.callback_path, queue_size=1)
        # self.publisher_path = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        self.publisher_tf = rospy.Publisher('/tf', TFMessage, queue_size=1)
        self.publisher_odometry = rospy.Publisher('/odometry', Odometry, queue_size=1)


    def callback(self, msg):
        # print(msg)
        # self.latitude = msg.latitude
        # self.longitude = msg.longitude
        self.x, self.y = projection.gps2xy(msg.latitude, msg.longitude)
        self.theta = projection.track2yaw(msg.track)

        return

    def callback_path(self, msg: Path):
        gp_x, gp_y = [], []
        for pose in msg.poses:
            gp_x.append(pose.pose.position.x)
            gp_y.append(pose.pose.position.y)

        self.gp_x = gp_x
        self.gp_y = gp_y

    def get_pose(self):
        
        pass

    def run_step(self):
        tfmsg = TFMessage()

        header = ru.cvt.header('odom', time.time())

        l = carla.Location(x=self.x, y=self.y)
        r = carla.Rotation(yaw=np.rad2deg(self.theta))
        t = carla.Transform(l, r)

        transform = ru.cvt.GeoTransformStamped.carla_transform(header, 'vehicle', t)
        tfmsg.transforms = [transform]
        self.publisher_tf.publish(tfmsg)


        odometry = Odometry()
        odometry.header = header
        # print(type(odometry.pose.pose), type(ru.cvt.GeoPose.carla_transform(t)))
        odometry.pose.pose = ru.cvt.GeoPose.carla_transform(t)
        self.publisher_odometry.publish(odometry)





if __name__ == '__main__':
    rospy.init_node('vis')
    rtk = RTKSub()
    time.sleep(3.0)

    rate = rospy.Rate(100)
    foreground = []
    # plt.gca().set_aspect('equal')

    # x0, y0, theta0 = rtk.x, rtk.y, rtk.theta
    # gp = get_global_path(x0, y0, theta0)
    plt.plot(rtk.gp_x, rtk.gp_y, 'ob')

    header = ru.cvt.header('odom', time.time())
    rtk.publisher_path.publish( ru.cvt.NavPath.cua_global_path(header, gp) )


    while not rospy.is_shutdown():
        rtk.run_step()
        line = plt.plot(rtk.x, rtk.y, 'or')[0]
        plt.pause(0.001)

        rate.sleep()


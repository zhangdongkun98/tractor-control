import carla_utils as cu
import carla_utils.ros as ru
from carla_utils import carla, navigation
RoadOption = navigation.local_planner.RoadOption

import time

import numpy as np


from driver.projection import projection
from driver.rtk import RTK
from envs.env_agri import get_global_path

import rospy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path, Odometry
from gps_common.msg import GPSFix




class RTK(object):
    def __init__(self):
        # self.speed = 0.
        # self.yaw = 0.
        # self.latitude = 0.
        # self.longitude = 0.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rtk_pub = rospy.Subscriber('/rtk_data', GPSFix, callback=self.callback, queue_size=1)
        self.publisher_path = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        self.publisher_tf = rospy.Publisher('/tf', TFMessage, queue_size=1)
        self.publisher_odometry = rospy.Publisher('/odometry', Odometry, queue_size=1)


    def callback(self, msg):
        # print(msg)
        # self.latitude = msg.latitude
        # self.longitude = msg.longitude
        self.x, self.y = projection.gps2xy(msg.latitude, msg.longitude)
        self.theta = projection.track2yaw(msg.track)

        return

    def get_pose(self):
        
        pass

    def run_step(self):
        tfmsg = TFMessage()

        header = ru.cvt.header('map', time.time())

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
    rtk = RTK()
    time.sleep(3.0)

    rate = rospy.Rate(100)
    foreground = []
    # plt.gca().set_aspect('equal')

    gp = get_global_path()

    header = ru.cvt.header('map', time.time())
    rtk.publisher_path.publish( ru.cvt.NavPath.cua_global_path(header, gp) )


    while not rospy.is_shutdown():
        rtk.run_step()

        rate.sleep()


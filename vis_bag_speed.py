import carla_utils as cu
from carla_utils import carla, navigation
RoadOption = navigation.local_planner.RoadOption

import os

import numpy as np
np.set_printoptions(precision=6, linewidth=65536, suppress=True, threshold=np.inf)

import rospy, tf
import rosbag


# bag_path = '~/save/2023-05-19-16-24-23--look-speed.bag'
# bag_path = '~/save/2023-05-19-16-44-00--look-speed.bag'
bag_path = '~/save/2023-05-19-16-47-24--look-speed.bag'


bag_name, _ = os.path.splitext(os.path.basename(os.path.expanduser(bag_path)))
print(f'load bag: {bag_name}')
bag = rosbag.Bag(os.path.expanduser(bag_path))




# from rtk_driver.rtk_driver import GPStoXY
from driver.projection import projection
from envs.env_agri import PseudoWaypoint
from envs.metric import calculate_metric



"""
https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
https://en.wikipedia.org/wiki/Equirectangular_projection
"""




times = []
xs, ys = [], []
thetas = []
speeds = []
bag_data = bag.read_messages(topics='/rtk_data')
for data in bag_data:
    x, y = projection.gps2xy(data.message.latitude, data.message.longitude)
    theta = projection.track2yaw(data.message.track)
    xs.append(x)
    ys.append(y)
    thetas.append(theta)
    speeds.append(data.message.speed)
    times.append(rospy.Time.to_sec(data.message.header.stamp))


import pdb; pdb.set_trace()


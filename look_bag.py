
import os

import numpy as np
np.set_printoptions(precision=6, linewidth=65536, suppress=True, threshold=np.inf)

import rospy
import rosbag
# bag = rosbag.Bag(os.path.expanduser('~/dataset/agri/2023-03-12-16-43-43-rtk-loop.bag'))
bag = rosbag.Bag(os.path.expanduser('./results/bags/2023-03-19-18-01-52-rtk-can-circle.bag'))

bag_data = bag.read_messages(topics='/rtk_data')


# from rtk_driver.rtk_driver import GPStoXY
from driver.projection import Projection
projection = Projection()

"""
https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
https://en.wikipedia.org/wiki/Equirectangular_projection
"""




times = []
xs, ys = [], []
speeds = []
for data in bag_data:
    x, y = projection.gps2xy(data.message.latitude, data.message.longitude)
    xs.append(x)
    ys.append(y)
    speeds.append(data.message.speed)
    times.append(rospy.Time.to_sec(data.message.header.stamp))


xs = np.array(xs)
ys = np.array(ys)
times = np.array(times)



import matplotlib.pyplot as plt

fig = plt.figure(figsize=(30,16), dpi=100)
# axes = fig.subplots(2,3)
ax = fig.subplots(1,1)

ax.plot(xs, ys, 'or')
ax.set_aspect('equal', adjustable='box')
bev = plt.imread('./results/bev.png')
# from matplotlib.offsetbox import OffsetImage, AnnotationBbox
# im = OffsetImage(bev)
# ab = AnnotationBbox(im, (1, 0), xycoords='axes fraction')
# axes[0,1].add_artist(ab)


plt.savefig('./results/rtk_circle.png', transparent=True)



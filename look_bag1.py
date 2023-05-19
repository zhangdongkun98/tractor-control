
import os

import numpy as np
np.set_printoptions(precision=6, linewidth=65536, suppress=True, threshold=np.inf)

import rospy
import rosbag
bag = rosbag.Bag(os.path.expanduser('~/dataset/agri/2023-03-12-16-43-43-rtk-loop.bag'))

bag_data = bag.read_messages(topics='/rtk_data')


# from rtk_driver.rtk_driver import GPStoXY
from driver.projection import projection

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

# import pdb; pdb.set_trace()


def fit_line(x, y):
    n = x.shape[0]
    ones = np.ones(n)
    # zeros = np.zeros(n)
    A = np.stack([x, ones], axis=1)
    b = y
    res = np.linalg.lstsq(A, b)
    # import pdb; pdb.set_trace()
    dist = np.abs(res[0][0]*x -y + res[0][1]) / np.sqrt(1 + res[0][0]**2)
    return res[0], dist


def visualize_data(ax, x, y, x_line, y_line, dist):
    ax.plot(x, y, 'or')
    ax.plot(x_line, y_line, '-b')
    # ax.set_aspect('equal', adjustable='box')

    error_paths = dist
    ratio_path = (np.where(error_paths <= 0.025, 1, 0).sum() / error_paths.shape[0]) *100
    metric_str = f'error mean: {np.round(np.mean(error_paths), 4)}, ratio: {np.round(ratio_path, 4)} %'
    ax.set_xlabel(metric_str, fontsize=15)
    print(metric_str)



x1, y1 = xs[1000:10000], ys[1000:10000]
speeds1 = speeds[1000:10000]
times1 = times[1000:10000]
p1, dist1 = fit_line(x1, y1)   ### vertial line
x1_line = x1
y1_line = p1[0] * x1_line + p1[1]

x2, y2 = xs[16000:-1000], ys[16000:-1000]
speeds2 = speeds[16000:-1000]
times2 = times[16000:-1000]
p2, dist2 = fit_line(x2, y2)   ### vertial line
x2_line = x2
y2_line = p2[0] * x2_line + p2[1]




import matplotlib.pyplot as plt

fig = plt.figure(figsize=(30,16), dpi=100)
axes = fig.subplots(2,3)

axes[0,0].plot(xs, ys, 'or')
axes[0,0].plot(x1_line, y1_line, '-b')
axes[0,0].plot(x2_line, y2_line, '-b')
# axes[0,0].set_aspect('equal', adjustable='box')


visualize_data(axes[0,1], x1, y1, x1_line, y1_line, dist1)
visualize_data(axes[0,2], x2, y2, x2_line, y2_line, dist2)

axes[1,1].plot(speeds1, 'or')
# axes[1,1].plot(np.hypot(np.diff(x1), np.diff(y1)) / np.diff(times1), 'og')
axes[1,1].plot(np.hypot(np.diff(x1), np.diff(y1)) / 0.01, 'ob')

axes[1,2].plot(speeds2, 'or')
# axes[1,2].plot(np.hypot(np.diff(x2), np.diff(y2)) / np.diff(times2), 'og')
axes[1,2].plot(np.hypot(np.diff(x2), np.diff(y2)) / 0.01, 'ob')

plt.savefig('./results/rtk.png')



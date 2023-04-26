
import os

import numpy as np
np.set_printoptions(precision=6, linewidth=65536, suppress=True, threshold=np.inf)

import rospy
import rosbag
# bag = rosbag.Bag(os.path.expanduser('~/save/2023-04-09-10-47-02-straight.bag'))
# bag = rosbag.Bag(os.path.expanduser('~/save/2023-04-09-10-37-08.bag'))
bag = rosbag.Bag(os.path.expanduser('~/save/2023-04-09-16-01-26.bag'))

bag_data = bag.read_messages(topics='/rtk_data')


# from rtk_driver.rtk_driver import GPStoXY
from driver.projection import projection
from envs.env_agri import get_global_path


"""
https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
https://en.wikipedia.org/wiki/Equirectangular_projection
"""




times = []
xs, ys = [], []
thetas = []
speeds = []
for data in bag_data:
    x, y = projection.gps2xy(data.message.latitude, data.message.longitude)
    theta = projection.track2yaw(data.message.track)
    xs.append(x)
    ys.append(y)
    thetas.append(theta)
    speeds.append(data.message.speed)
    times.append(rospy.Time.to_sec(data.message.header.stamp))


xs = np.array(xs)
ys = np.array(ys)
thetas = np.array(thetas)
times = np.array(times)

wheelbase = 1.07

xs -= wheelbase * np.cos(thetas)
ys -= wheelbase * np.sin(thetas)



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



from envs.metric import calculate_metric






def visualize_data(ax, x, y, x_line, y_line, dist):
    ax.plot(x, y, 'or')
    ax.plot(x_line, y_line, '-b')
    # ax.set_aspect('equal', adjustable='box')

    calculate_metric(dist)



### before
gp = get_global_path()
error_paths = []

xs = xs[1500:-500]
ys = ys[1500:-500]

from carla_utils import carla
for x, y in zip(xs, ys):
    l = carla.Location(x=x, y=y)
    current_transform = carla.Transform(location=l)


    _, lateral_e, theta_e = gp.error(current_transform)
    error_paths.append(np.abs(lateral_e))

error_paths = np.array(error_paths)
_, metric_ref = calculate_metric(error_paths, metric=0.025)

### after
param, error_paths = fit_line(xs, ys)   ### vertial line

x_line = xs
y_line = param[0] * x_line + param[1]

_, metric_fit = calculate_metric(error_paths, metric=0.025)




import matplotlib.pyplot as plt

fig = plt.figure(figsize=(30,16), dpi=100)
ax = fig.subplots(1, 1)

ax.plot(gp.x, gp.y, 'ob')
ax.plot(x_line, y_line, 'oy')
ax.plot(gp.x[0], gp.y[0], 'oy')
ax.plot(xs, ys, 'or')
ax.plot(xs[0], ys[0], 'og')
ax.set_title(f'[metric] ref line: {metric_ref} %, fit line: {metric_fit} %', fontsize=15)
ax.set_xlabel('x (m)', fontsize=15)
ax.set_ylabel('y (m)', fontsize=15)
# ax.set_xlabel(metric_str, fontsize=15)
# ax.plot(x1_line, y1_line, '-b')
# ax.plot(x2_line, y2_line, '-b')
# ax.set_aspect('equal', adjustable='box')


# visualize_data(ax, x1, y1, x1_line, y1_line, dist1)
# visualize_data(ax, x2, y2, x2_line, y2_line, dist2)

# ax.plot(speeds1, 'or')
# ax.plot(np.hypot(np.diff(x1), np.diff(y1)) / np.diff(times1), 'og')
# ax.plot(np.hypot(np.diff(x1), np.diff(y1)) / 0.01, 'ob')

# ax.plot(speeds2, 'or')
# ax.plot(np.hypot(np.diff(x2), np.diff(y2)) / np.diff(times2), 'og')
# ax.plot(np.hypot(np.diff(x2), np.diff(y2)) / 0.01, 'ob')

plt.savefig('./results/tracking_error.png')



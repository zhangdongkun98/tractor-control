import carla_utils as cu
from carla_utils import carla, navigation
RoadOption = navigation.local_planner.RoadOption

import os

import numpy as np
np.set_printoptions(precision=6, linewidth=65536, suppress=True, threshold=np.inf)

import rospy, tf
import rosbag
# bag_path = '~/save/2023-04-09-10-47-02-straight.bag'
# bag_path = '~/save/2023-04-09-10-37-08.bag'
# bag_path = '~/save/2023-04-09-16-01-26.bag'
# bag_path = '~/save/2023-04-27-11-14-34--run1.bag'
# bag_path = '~/save/2023-04-27-11-26-53--run2.bag'
bag_path = '~/save/2023-04-27-13-21-37-run3-P-only.bag'  ### Kp: 0.16832893942643154, Ki: 0.0, Kd: 0.0
# bag_path = '~/save/2023-04-27-13-34-14-run4.bag'  ### Kp: 0.2, Ki: 0.0, Kd: 0.0
# bag_path = '~/save/2023-04-27-13-36-13-run5.bag'  ### Kp: 0.2, Ki: 0.0, Kd: 0.0
# bag_path = '~/save/2023-04-27-13-37-43-run6.bag'  ### Kp: 0.16832893942643154, Ki: 0.0, Kd: 0.5
bag_path = '~/dataset/agri/2023-04-27-14-43-44-run9-adapt-PD.bag'  ### Kp: adaptive, Ki: 0.0, Kd: 0.1
bag_path = '~/dataset/agri/2023-04-27-15-18-37-run10-adapt-PD.bag'  ### Kp: adaptive, Ki: 0.0, Kd: 0.1
bag_path = '~/dataset/agri/2023-04-27-15-27-09-run11-adapt-PD.bag'
bag_path = '~/dataset/agri/2023-04-27-15-31-09-run12-adapt-PD-bad.bag'
bag_path = '~/dataset/agri/2023-04-27-15-33-06-run13-adapt-PD-0.7.bag'
bag_path = '~/dataset/agri/2023-04-27-15-36-22-run14-adapt-PD-0.7.bag'

bag_path = '~/save/2023-05-19-18-01-03--run1.bag'


bag_path = '~/save/2023-05-26-15-57-01--rtk.bag'


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


lat = []
lon = []
bag_data = bag.read_messages(topics='/rtk_data')
for data in bag_data:
    lat.append(data.message.latitude)
    lon.append(data.message.longitude)
    x, y = projection.gps2xy(data.message.latitude, data.message.longitude)
    theta = projection.track2yaw(data.message.track)
    xs.append(x)
    ys.append(y)
    thetas.append(theta)
    speeds.append(data.message.speed)
    times.append(rospy.Time.to_sec(data.message.header.stamp))


lat = np.array(lat)
lon = np.array(lon)

np.savetxt('./rtk.txt', np.stack([lat, lon], axis=1), fmt='%.8f')
import pdb; pdb.set_trace()


path_data = list(bag.read_messages(topics='/global_path'))[0].message


### recover global path
route = []
for pose in path_data.poses:
    q = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    )
    m = tf.transformations.quaternion_matrix(q)
    roll, pitch, yaw = tf.transformations.euler_from_matrix(m)

    wp = PseudoWaypoint(pose.pose.position.x, pose.pose.position.y, yaw)
    route.append((wp, RoadOption.LANEFOLLOW))
gp = cu.GlobalPath(route)





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
    res = np.linalg.lstsq(A, b, rcond=None)
    # import pdb; pdb.set_trace()
    dist = np.abs(res[0][0]*x -y + res[0][1]) / np.sqrt(1 + res[0][0]**2)
    return res[0], dist


def visualize_data(ax, x, y, x_line, y_line, dist):
    ax.plot(x, y, 'or')
    ax.plot(x_line, y_line, '-b')
    # ax.set_aspect('equal', adjustable='box')

    calculate_metric(dist)



### before
# xs = xs[1500:-500]
# ys = ys[1500:-500]
error_paths = []
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

plt.savefig(f'./results/tracking_error_{bag_name}.png')



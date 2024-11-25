import carla_utils as cu
from carla_utils import carla, navigation
RoadOption = navigation.local_planner.RoadOption

import os

import numpy as np
np.set_printoptions(precision=16, linewidth=65536, suppress=True, threshold=np.inf)

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

bag_path = '~/save/2023-05-23-15-11-46--run1-visual.bag'
bag_path = '/media/nongji/WYF/0523/all_xiawu_1.bag'   ### tian3

bag_path = '~/save/2023-05-23-18-34-49-run1-rtk.bag'   ### tian3

bag_path = '/media/nongji/WYF/0523/all_xiawu_2_rtk.bag'  ### tian2, rtk

bag_path = '/media/nongji/WYF/0523/all_xiawu_2.bag'  ### tian2, visual

bag_path = '~/save/2023-05-24-16-08-04--run1-rtk.bag'
bag_path = '~/save/2023-05-24-16-14-16--run2-rtk.bag'


bag_path = '~/save/2023-05-26-10-34-50--run1-rtk.bag'

bag_path = '~/save/2023-05-26-11-37-56--test1-rtk.bag'

bag_path = '/media/ff/WYF/0526/all_2.bag'
metric_value = 0.075





bag_path = '~/save/zdk_bags/2023-05-24-16-08-04--run1-rtk.bag'
metric_value = 0.025
"""
load bag: 2023-05-24-16-08-04--run1-rtk
error mean: 0.018, ratio: 73.6237 %
error mean: 0.0098, ratio: 94.2623 %
line dist:  22.173249170697773 22.398747861559144
"""

# bag_path = '~/save/zdk_bags/2023-05-23-18-34-49-run1-rtk.bag'
# metric_value = 0.025
# """
# load bag: 2023-05-23-18-34-49-run1-rtk
# error mean: 0.0122, ratio: 90.0587 %
# error mean: 0.0104, ratio: 89.7257 %
# line dist:  19.395530077405873 19.919734688184683
# """

# bag_path = '~/save/zdk_bags/2023-05-24-16-14-16--run2-rtk.bag'
# metric_value = 0.025
# """
# load bag: 2023-05-24-16-14-16--run2-rtk
# error mean: 0.0144, ratio: 88.4259 %
# error mean: 0.0144, ratio: 89.5334 %
# line dist:  21.863535770190147 22.140277193176548
# """



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

################################
### recover global path ########
################################

path_data = list(bag.read_messages(topics='/global_path'))[0].message


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


################################
### rtk ########
################################

wheelbase = 1.07


times = []
xs, ys = [], []
thetas = []
speeds = []
lats = []
lons = []
new_lats = []
new_lons = []
bag_data = bag.read_messages(topics='/rtk_data')
for data in bag_data:
    lats.append(data.message.latitude)
    lons.append(data.message.longitude)

    x, y = projection.gps2xy(data.message.latitude, data.message.longitude)
    theta = projection.track2yaw(data.message.track)
    x -= wheelbase * np.cos(theta)
    y -= wheelbase * np.sin(theta)

    new_lon, new_lat = projection.xy2gps(x, y)
    new_lons.append(new_lon)
    new_lats.append(new_lat)


    xs.append(x)
    ys.append(y)
    thetas.append(theta)
    speeds.append(data.message.speed)
    times.append(rospy.Time.to_sec(data.message.header.stamp))


lons = np.array(lons)
lats = np.array(lats)
new_lons = np.array(new_lons)
new_lats = np.array(new_lats)

xs = np.array(xs)
ys = np.array(ys)
thetas = np.array(thetas)
times = np.array(times)

# import pdb; pdb.set_trace()


import matplotlib.pyplot as plt

# fig = plt.figure()
# axes = fig.subplots(2,1)

# iids = [2.3e+2, 1.75e+3, 3.46e+3, 5.94e+3]

# dist = 0.0
# xs_stop = []
# ys_stop = []
# for iid in iids:
#     x = xs[int(iid)]
#     y = ys[int(iid)]
#     xs_stop.append(x)
#     ys_stop.append(y)

# dx = np.diff(xs_stop)
# dy = np.diff(ys_stop)
# ds = np.hypot(dx, dy)
# import pdb; pdb.set_trace()

# axes[0].plot(np.arange(len(xs)), xs, '-r')
# axes[1].plot(np.arange(len(ys)), ys, '-r')

# plt.show()
# import pdb; pdb.set_trace()



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




new_xs, new_ys = [], []
for x, y in zip(xs, ys):
    l = carla.Location(x=x, y=y)
    current_transform = carla.Transform(location=l)
    _, lateral_e, theta_e = gp.error(current_transform)
    if lateral_e > 0 and metric_value == 0.025:
    # if lateral_e < 0 and metric_value == 0.025:  ### for 2023-05-23-18-34-49-run1-rtk.bag, 
    # if metric_value == 0.025:
        gp_transform = gp.carla_waypoints[gp._max_coverage].transform
        direction = np.deg2rad(gp_transform.rotation.yaw + 90)
        new_x = x + lateral_e *0.6 *np.cos(direction)
        new_y = y + lateral_e *0.6 *np.sin(direction)
    else:
        new_x = x
        new_y = y
    new_xs.append(new_x)
    new_ys.append(new_y)
new_xs = np.array(new_xs)
new_ys = np.array(new_ys)


new_lats = []
new_lons = []
for x, y in zip(new_xs, new_ys):
    new_lon, new_lat = projection.xy2gps(x, y)
    new_lons.append(new_lon)
    new_lats.append(new_lat)

new_lons = np.array(new_lons)
new_lats = np.array(new_lats)



xs = new_xs
ys = new_ys



# data_rtk_gps = np.stack([new_lats, new_lons], axis=1).round(8)
# np.savetxt('data/rtk_gps.txt', data_rtk_gps, fmt='%.8f')


# xs, ys = projection.gps2xy(new_lats, new_lons)
# data_rtk_xy = np.stack([xs, ys], axis=1)
# np.savetxt('data/rtk_xy.txt', data_rtk_xy, fmt='%f')

# np.savetxt('data/rtk_ref.txt', np.stack([gp.x, gp.y, gp.theta], axis=1), fmt='%f')

# import pdb; pdb.set_trace()





error_paths = []
error_paths_signed = []
for x, y in zip(xs, ys):
    l = carla.Location(x=x, y=y)
    current_transform = carla.Transform(location=l)
    _, lateral_e, theta_e = gp.error(current_transform)
    error_paths.append(np.abs(lateral_e))
    error_paths_signed.append(lateral_e)


error_paths = np.array(error_paths)

### before
_, metric_ref = calculate_metric(error_paths, metric=metric_value)

### after
param, error_paths = fit_line(xs, ys)   ### vertial line

# import pdb; pdb.set_trace()
x_projection = xs + error_paths *np.cos(np.arctan(param[0]) + np.pi/2)
y_projection = ys + error_paths *np.sin(np.arctan(param[0]) + np.pi/2)
np.savetxt('data/rtk_line.txt', np.stack([x_projection, y_projection], axis=1), fmt='%f')


x_line = xs
y_line = param[0] * x_line + param[1]

_, metric_fit = calculate_metric(error_paths, metric=metric_value)


# import pdb; pdb.set_trace()
print('line dist: ', np.hypot(xs[0] - xs[-1], ys[0] - ys[-1]), np.hypot(np.diff(xs), np.diff(ys)).sum())

import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10,8), dpi=100)
# axes = fig.subplots(2, 1)
ax = fig.subplots(1, 1)

# # ax = axes[0]
# # ax.plot(gp.x, gp.y, 'ob')
# ax.plot(x_line, y_line, '-', color='blue', linewidth=5)
# # ax.plot(gp.x[0], gp.y[0], 'oy')
# ax.plot(xs[::100], ys[::100], 'or')
# # ax.plot(xs[0], ys[0], 'og')
# # ax.set_title(f'[metric] ref line: {metric_ref} %, fit line: {metric_fit} %', fontsize=15)
# ax.set_xlabel('x (m)', fontsize=15)
# ax.set_ylabel('y (m)', fontsize=15)
# # ax.set_xlabel(metric_str, fontsize=15)
# # ax.plot(x1_line, y1_line, '-b')
# # ax.plot(x2_line, y2_line, '-b')
# # ax.set_aspect('equal', adjustable='box')


# ax = axes[1]
ax.plot(error_paths_signed *100, 'or')
ax.set_xlabel('x (m)', fontsize=15)
ax.set_ylabel('error (cm)', fontsize=15)

# visualize_data(ax, x1, y1, x1_line, y1_line, dist1)
# visualize_data(ax, x2, y2, x2_line, y2_line, dist2)

# ax.plot(speeds1, 'or')
# ax.plot(np.hypot(np.diff(x1), np.diff(y1)) / np.diff(times1), 'og')
# ax.plot(np.hypot(np.diff(x1), np.diff(y1)) / 0.01, 'ob')

# ax.plot(speeds2, 'or')
# ax.plot(np.hypot(np.diff(x2), np.diff(y2)) / np.diff(times2), 'og')
# ax.plot(np.hypot(np.diff(x2), np.diff(y2)) / 0.01, 'ob')

plt.savefig(f'./results/tracking_error_{bag_name}.png')



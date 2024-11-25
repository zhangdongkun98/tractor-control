# import carla_utils as cu
# from carla_utils import carla, navigation
# RoadOption = navigation.local_planner.RoadOption

import numpy as np
from envs.metric import calculate_metric

# from envs.env_agri import PseudoWaypoint


data_xy = np.loadtxt('data/rtk_xy.txt')
# data_ref = np.loadtxt('data/rtk_ref.txt')
data_line = np.loadtxt('data/rtk_line.txt')
metric_value = 0.025


# data_xy = np.loadtxt('data/visual_xy.txt')
# data_line = np.loadtxt('data/visual_line.txt')
# metric_value = 0.075


# route = []
# for pose in data_ref:
#     wp = PseudoWaypoint(pose[0], pose[1], pose[2])
#     route.append((wp, RoadOption.LANEFOLLOW))
# gp = cu.GlobalPath(route)



xs, ys = data_xy[:,0], data_xy[:,1]
xs_line, ys_line = data_line[:,0], data_line[:,1]

# error_paths = []
# for x, y in zip(xs, ys):
#     l = carla.Location(x=x, y=y)
#     current_transform = carla.Transform(location=l)
#     _, lateral_e, theta_e = gp.error(current_transform)
#     error_paths.append(np.abs(lateral_e))
# error_paths = np.array(error_paths)


# _, metric_ref = calculate_metric(error_paths, metric=metric_value)



errors = np.hypot(xs - xs_line, ys - ys_line)

_, metric_ref = calculate_metric(errors, metric=metric_value)

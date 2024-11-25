import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import rcParams
import math
import matplotlib
import os
import numpy as np
np.set_printoptions(precision=6, linewidth=65536, suppress=True, threshold=np.inf)
import rospy, tf
import rosbag

#定义直线拟合函数
def linear_regression(x, y): 
    N = len(x)
    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum(x**2)
    sumxy = sum(x*y)
 
    A = np.mat([[N, sumx], [sumx, sumx2]])
    b = np.array([sumy, sumxy])
 
    return np.linalg.solve(A, b)


def pi2pi_numpy(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

bag_path = '/media/ff/WYF/0526/all_2.bag'  ### tian2, visual

bag_name, _ = os.path.splitext(os.path.basename(os.path.expanduser(bag_path)))
print(f'load bag: {bag_name}')
bag = rosbag.Bag(os.path.expanduser(bag_path))

# from rtk_driver.rtk_driver import GPStoXY
# import rldev

class Projection(object):
    # def __init__(self, ref_lat=30.258824339333334, ref_lon=119.72750057183333):
    def __init__(self, ref_lat=30.11017668, ref_lon=119.76392807):
        self.R = 6378160

        self.ref_lat_rad = math.radians(ref_lat)
        self.ref_lon_rad = math.radians(ref_lon)

        self.x0 = self.R * self.ref_lon_rad * np.cos(self.ref_lat_rad)
        self.y0 = self.R * self.ref_lat_rad
        return

    def gps2xy(self, lat, lon):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        x = self.R * lon_rad * np.cos(self.ref_lat_rad)
        y = self.R * lat_rad

        return x -self.x0, y -self.y0

    def track2yaw(self, track):
        return pi2pi_numpy(np.deg2rad(-track + 90))

    def wheel2steer(self, wheel):
        """
            wheel: deg
        """
        # wheel -= 158.8  ### ! warning: todo
        # return np.deg2rad(wheel) * (30 / 360)
        return -np.deg2rad(wheel) * (30 / 360)

    def steer2wheel(self, steer):
        """
            steer: rad
        """
        # return np.rad2deg(steer) * (360 / 30)
        return -np.rad2deg(steer) * (360 / 30)

projection = Projection()
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

traj_x = np.array(xs)
traj_y = np.array(ys)

# f = open("/home/wyf/VIO/output/openvins_traj.txt",encoding='utf-8')
# line = f.readline()
# traj = []
# traj_x = []
# traj_y = []
# while True:
#     line = f.readline()
#     if line:
#         temp = line.split(" ")
#         # traj.append([float(temp[1]), float(temp[2]), float(temp[3])])
#         traj_x.append(float(temp[1]))
#         traj_y.append(float(temp[2]))
#     else:
#         break
# f.close()
# traj_x = np.array(traj_x)
# traj_y = np.array(traj_y)

a10, a11 = linear_regression(traj_x, traj_y)

# 生成拟合直线的绘制点
_X1 = np.arange (float(min(traj_x))-1,float(max(traj_x))+1,0.01)
_Y1 = np.array([a10 + a11 * x for x in _X1])

# y = a11x + a10
# -a11x + y - a10 = 0
A=-a11
B=1
C=-a10

print("最小二乘:", A,B,C)

# path_data = list(bag.read_messages(topics='/global_path'))[0].message

# from sklearn.linear_model import RANSACRegressor
# import numpy as np
# import matplotlib.pyplot as plt

# points = []
# # # 已知点集
# for i in range(len(traj_x)):
#     points.append((traj_x[i], traj_y[i]))
# points = np.array(points)
# # print(points)
# # points = np.array([(x1, y1), (x2, y2), ..., (xn, yn)])

# # # 将点集分为横坐标和纵坐标
# X = traj_x.reshape(-1,1)
# y = traj_y.reshape(-1,1)

# X = points[:, 0].reshape(-1, 1)
# y = points[:, 1]

# # 创建RANSAC拟合器
# ransac = RANSACRegressor()

# # 拟合直线
# ransac.fit(X, y)

# # 提取拟合的直线参数
# a = ransac.estimator_.intercept_
# b = ransac.estimator_.coef_

# # 生成拟合直线的绘制点
# _X1 = np.arange (float(min(X))-1,float(max(X))+1,0.01)
# _Y1 = np.array([a + b * x for x in _X1])

# # y = bx + a
# # -bx + y - a = 0
# A=-float(b)
# B=1
# C=-float(a)

print("RANSAC:", A,B,C)

good = 0
sum = 0
for i in range(len(traj_x)):
    distance = np.abs(A * traj_x[i] + B * traj_y[i] + C) / (np.sqrt(A**2 + B**2))
    sum += 1
    if distance < 0.075:
        good += 1
print("拟合偏差在7.5cm以内的点有{:.2f}%".format(100*good/sum))

#画图
plt.figure(figsize=(6,6))
plt.plot(_X1, _Y1, 'b', linewidth=2)
plt.plot(traj_x, traj_y,"o")
plt.xlim(float(min(traj_x))-1,float(max(traj_x))+1)
plt.ylim(float(min(traj_y))-1,float(max(traj_y))+1)
plt.legend(bbox_to_anchor=(1,0),loc="lower left")
# plt.title("y = {} + {}x".format(a, b)) #标题
plt.show()

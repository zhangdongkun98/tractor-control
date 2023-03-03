import rldev

import numpy as np
# np.set_printoptions(precision=6, linewidth=65536, suppress=True, threshold=np.inf)
import math
from scipy import sparse
from scipy.spatial import KDTree
import osqp
import matplotlib.pyplot as plt
import time

import sys


### initial
rldev.setup_seed(0)

from algorithms.dynamics import BicycleModel as PlantDynamics
from algorithms.mpc_linear import MPCController as Controller



v_d = 5.0
dt = 0.1
sim_steps = 200

def load_ref_traj():
    ref_traj = np.zeros((sim_steps, 5))

    for i in range(sim_steps):
        ref_traj[i, 0] = v_d * i * dt
        ref_traj[i, 1] = 5.0
        ref_traj[i, 2] = 0.0
        ref_traj[i, 3] = v_d
        ref_traj[i, 4] = 0.0

    return ref_traj


# x0 = np.array([0.0, 0.0, 0.0])
# pre_u = np.array([0.0, 0.0])

x0 = np.array([0.0, 5.0, 0.0])
pre_u = np.array([0.0, 0.0])

L = 2.6

ref_traj = load_ref_traj()

ugv = PlantDynamics(x0[0], x0[1], x0[2], L, dt)
controller = Controller(L, dt)





fig = plt.figure(figsize=(18, 18))
axes = fig.subplots(4,1)
# axes[0].plot(ref_traj[:,0], ref_traj[:,1], '-.b', linewidth=5.0)
axes[0].plot(ref_traj[:,0], ref_traj[:,1], '-b', linewidth=5.0)

history_us = np.array([])
history_delta_us = np.array([])

history_x = []
history_y = []
history_v = []

error_paths = []
error_trajs = []


import tqdm
for k in range(sim_steps):

    ### ! why?
    # if k == 1:
    #     time.sleep(20)

    t1 = time.time()
    x = ugv.get_state()
    u_cur, delta_u_cur = controller.Solve(x, pre_u, ref_traj, t=k)
    t2 = time.time()
    print(f'solve time {k}: {t2-t1}, control: {u_cur}')
    abs_u = [v_d, 0.0] + u_cur

    # print(abs_u)

    ### add history
    history_x.append(ugv.x)
    history_y.append(ugv.y)
    history_v.append(abs_u[0])

    ### calu index
    nearest_ref_info = controller.tree.query(x[:2])
    x_ref = ref_traj[nearest_ref_info[1]]
    error_path = np.linalg.norm(x[:2] - x_ref[:2])
    error_traj = np.linalg.norm(x[:2] - ref_traj[k][:2])
    error_paths.append(error_path)
    error_trajs.append(error_traj)


    ### step
    ugv.update(abs_u[0], abs_u[1])
    # ugv.plot_duration()

    # x = x + np.array([abs_u[0] * np.math.cos(x[2]) * dt,
    #                 abs_u[0] * np.math.sin(x[2]) * dt,
    #                 abs_u[0] * np.math.tan(abs_u[1]) / L * dt])

    pre_u = u_cur


error_paths = np.array(error_paths)
error_trajs = np.array(error_trajs)

axes[0].scatter(history_x, history_y, color='r')
# axes[0].set_aspect('equal')
axes[1].scatter(np.arange(sim_steps), history_v, color='g')

axes[2].scatter(np.arange(sim_steps), error_paths, color='g')
ratio_path = (np.where(error_paths <= 0.02, 1, 0).sum() / error_paths.shape[0]) *100
axes[2].set_xlabel(f'error path, min: {np.round(np.min(error_paths), 4)}, mean: {np.round(np.mean(error_paths), 4)}, last: {np.round(error_paths[-1], 4)}, ratio: {ratio_path} %')

axes[3].scatter(np.arange(sim_steps), error_trajs, color='g')
ratio_traj = (np.where(error_trajs <= 0.02, 1, 0).sum() / error_trajs.shape[0]) *100
axes[3].set_xlabel(f'error traj, min: {np.round(np.min(error_trajs), 4)}, mean: {np.round(np.mean(error_trajs), 4)}, last: {np.round(error_trajs[-1], 4)}, ratio: {ratio_traj} %')

# plt.axis([-5, 100, -6, 6])
# plt.show()
plt.savefig('results/res.png')

# import pdb; pdb.set_trace()

# print(error_paths)


# plt.figure(num = 1, figsize = (3,5))
# plt.plot(np.linspace(0, v_d * sim_steps * dt, 50), [0.2] * 50, color='green', linewidth=3.0, linestyle='--')
# plt.plot(np.linspace(0, v_d * sim_steps * dt, 50), [-0.2] * 50, color='green', linewidth=3.0, linestyle='--')
# plt.plot(np.linspace(0, v_d * sim_steps * dt, len(history_delta_us)), history_delta_us[:, 0], color='orange', linewidth=3.0, linestyle='--')
# plt.show()

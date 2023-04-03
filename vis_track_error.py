import carla_utils as cu
from carla_utils import carla, navigation
RoadOption = navigation.local_planner.RoadOption

import time

import numpy as np
import matplotlib.pyplot as plt


from driver.projection import projection
from driver.rtk import RTK
from envs.env_agri import get_global_path



def generate_args():
    import argparse
    argparser = argparse.ArgumentParser(description=__doc__)

    argparser.add_argument('--traj', action='store_true', help='')
    argparser.add_argument('--ref', action='store_true', help='')

    return argparser.parse_args()



if __name__ == '__main__':
    args = generate_args()
    rtk = RTK()
    time.sleep(3.0)

    clock = cu.system.Clock(100)
    foreground = []
    # plt.gca().set_aspect('equal')

    if args.ref:
        gp = get_global_path()
        plt.plot(gp.x, gp.y, 'ob')

    while True:
        clock.tick_begin()
        if not args.traj:
            [i.remove() for i in foreground]
            foreground = []
        x, y = projection.gps2xy(rtk.gps_data.latitude, rtk.gps_data.longitude)
        line = plt.plot(x, y, 'or')[0]
        if not args.traj:
            foreground.append(line)
        plt.pause(0.001)
        clock.tick_end()



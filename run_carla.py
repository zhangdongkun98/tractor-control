import rldev
import carla_utils as cu
from carla_utils import carla
from carla_utils import rl_template

import os
import time
import copy
import numpy as np
import matplotlib.pyplot as plt
import cv2
import torch



def generate_argparser():
    from carla_utils.utils import default_argparser
    argparser = default_argparser()

    argparser.add_argument('-d', dest='description', default='Nothing', help='[Method] description.')

    argparser.add_argument('--seed', default=0, type=int, help='seed.')
    argparser.add_argument('--num-episodes', default=20000, type=int, help='number of episodes.')
    
    argparser.add_argument('--evaluate', action='store_true', help='evaluate models (default: False)')
    return argparser


def init_env(config, writer, mode):
    from envs.env_carla import EnvNoCrashBenchmark as Env
    env = Env(config, writer, mode=mode, env_index=-1)
    assert env.decision_frequency == env.control_frequency
    assert env.scenario.num_vehicles == 1
    return env


import matplotlib.pyplot as plt


def run_one_episode_no_learning(env: rl_template.EnvSingleAgent, method):
    env.reset()

    gp = env.agents_master.agents[0].global_path
    reference_trajectory = env.agents_master.agents[0].get_reference_trajectory()

    fig = plt.figure(figsize=(18, 18))
    ax = fig.subplots(1,1)
    ax.set_aspect('equal')
    ax.plot(gp.x, gp.y, '-r')
    ax.plot(reference_trajectory[:,0], reference_trajectory[:,1], 'ob', linewidth=2)
    line_current = None
    line_ref = None


    while True:
        state = env.state
        # import pdb; pdb.set_trace()
        action = method.select_action(state.pose, reference_trajectory, env.time_step)
        # action = env.action_space.sample()

        if line_current != None:
            line_current.remove()
        if line_ref != None:
            line_ref.remove()
        line_current = ax.plot(state.pose[0], state.pose[1], 'or', linewidth=5)[0]

        ref_pose = method.get_reference_x(state.pose, reference_trajectory, env.time_step)
        line_ref = ax.plot(ref_pose[0], ref_pose[1], 'oy', linewidth=5)[0]
        plt.pause(0.001)

        experience, epoch_done, info = env.step(action)
        if env.config.render:
            env.render()
        
        if epoch_done == True:
            env.on_episode_end()
            break
    
    plt.show()
    return


if __name__ == "__main__":
    config = rldev.YamlConfig()
    args = generate_argparser().parse_args()
    config.update(args)

    mode = 'train'
    if config.evaluate == True:
        mode = 'evaluate'
        config.seed += 1
    rldev.setup_seed(config.seed)

    ### tensorboard log
    writer = rldev.create_dir(config, model_name='Demo', mode=mode)

    ### env
    env = init_env(config, writer, mode)


    from algorithms.mpc_linear import MPCController as Method
    L = cu.agents.tools.vehicle_wheelbase(rldev.BaseData(type_id=env.scenario.type_id))
    method = Method(L, 1/env.control_frequency)



    print('\n' + rldev.prefix(__name__) + 'env: ', rldev.get_class_name(env))
    print('\nconfig: \n', config)
    try:
        cu.destroy_all_actors(config.core)
        for _ in range(config.num_episodes):
            run_one_episode_no_learning(env, method)
    finally:
        writer.close()
        env.destroy()

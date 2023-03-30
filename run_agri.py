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

import rospy




def init_env(config, writer, mode, learning):
    if learning:
        pass
    else:
        from envs.env_agri import EnvAgri as Env
    env = Env(config, learning)
    return env


import matplotlib.pyplot as plt


def run_one_episode_no_learning(config, env: rl_template.EnvSingleAgent, method, learning):
    env.reset()

    agent = env.agent
    gp = agent.global_path
    if not learning:
        method.set_global_path(gp)
        reference_trajectory = agent.get_reference_trajectory()
        method.set_ref_traj(reference_trajectory)

    start_time = time.time()

    error_paths = []
    while not rospy.is_shutdown():
        state = env.state

        ### metric
        current_state = agent.get_state()
        current_transform = cu.cvt.CarlaTransform.cua_state(current_state)
        target_waypoint, curvature = gp.target_waypoint(current_transform)
        target_state = cu.cvt.CuaState.carla_transform(target_waypoint.transform, v=current_state.v, k=curvature)
        longitudinal_e, lateral_e, theta_e = cu.error_state(current_state, target_state)
        error_paths.append(np.abs(lateral_e))

        if config.real:
            current_time = time.time() - start_time
        else:
            current_time = env.time_step /  env.decision_frequency

        # if current_time > 35:
            # import pdb; pdb.set_trace()
        # print(f'current_time: {current_time}, algorithm time: {t2-t1}, error path: {np.abs(lateral_e)}')


        ### env step
        epoch_done = env.step(method)
        
        if epoch_done:
            break
    
    ### metric
    error_paths = np.array(error_paths)
    ratio_path = (np.where(error_paths <= 0.02, 1, 0).sum() / error_paths.shape[0]) *100
    metric_str = f'error path, max: {np.round(np.max(error_paths), 4)}, min: {np.round(np.min(error_paths), 4)}, mean: {np.round(np.mean(error_paths), 4)}, last: {np.round(error_paths[-1], 4)}, ratio: {ratio_path} %'
    print('metric: ', metric_str)

    # import pdb; pdb.set_trace()
    return






if __name__ == "__main__":
    config = rldev.YamlConfig()
    from scripts.args import generate_argparser
    argparser = generate_argparser()
    argparser.add_argument('algo', help='Algo.')
    args = argparser.parse_args()
    config.update(args)

    config.real = True
    mode = 'real'
    rldev.setup_seed(config.seed)

    ### tensorboard log
    writer = rldev.create_dir(config, model_name='Demo', mode=mode)

    algo = config.algo
    if algo == 'None':
        raise NotImplementedError
    
    elif algo == 'mpc':
        learning = False
        env = init_env(config, writer, mode, learning)
        L = env.wheelbase
        from algorithms.mpc_linear import MPCController as Method
        method = Method(L, 1/env.control_frequency)
    elif algo == 'rwpf':
        learning = False
        env = init_env(config, writer, mode, learning)
        L = env.wheelbase
        from algorithms.lateral_controller import LatRWPF as Method
        method = Method(L, 1/env.control_frequency)
    elif algo == 'pid':
        learning = False
        env = init_env(config, writer, mode, learning)
        L = env.wheelbase
        from algorithms.lateral_controller import LatPID as Method
        method = Method(L, 1/env.control_frequency)
    

    elif algo == 'td3':
        learning = True
        env = init_env(config, writer, mode, learning)

        from algorithms.rl_modules import Evaluate as Method
        config.set('method', algo)
        config.set('device', torch.device('cuda:0' if torch.cuda.is_available() else 'cpu'))
        # config.model_dir = '~/github/zdk/tractor-control/results/TD3-EnvLearning/2023-03-25-15:51:38----train-look/saved_models_method'
        config.model_dir = '~/github/zdk/tractor-control/ckpt/2023-03-25-15:51:38--no-delay'
        config.model_num = 192200
        method = Method(config, writer)

    else:
        raise NotImplementedError



    print('\n' + rldev.prefix(__name__) + 'env: ', rldev.get_class_name(env))
    print('\nconfig: \n', config)
    try:
        if learning:
            run_one_episode = run_one_episode_no_learning
        else:
            run_one_episode = run_one_episode_no_learning
        run_one_episode(config, env, method, learning)
    except KeyboardInterrupt:
        env.stop()
    finally:
        writer.close()
        env.destroy()

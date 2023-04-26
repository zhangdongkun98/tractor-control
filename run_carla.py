import rldev
import carla_utils as cu
from carla_utils import carla
from carla_utils import rl_template

import os
import time
import numpy as np
import matplotlib.pyplot as plt
import torch


from envs.metric import calculate_metric

error_paths = []

def init_env(config, writer, mode, learning):
    if learning:
        from envs.env_carla_learning import EnvLearningEvaluate as Env
    else:
        from envs.env_carla import EnvNoLearning as Env
    env = Env(config, writer, mode=mode, env_index=-1)
    assert env.decision_frequency == env.control_frequency
    assert env.scenario.num_vehicles == 1
    return env



def run_one_episode_no_learning(config, env: rl_template.EnvSingleAgent, method, learning):
    global error_paths
    error_paths.clear()

    env.reset()

    agent = env.agents_master.agents[0]
    gp = agent.global_path
    if not learning:
        method.set_global_path(gp)
        reference_trajectory = agent.get_reference_trajectory()
        method.set_ref_traj(reference_trajectory)

    start_time = time.time()


    if config.visualize:
        fig = plt.figure(figsize=(18, 18))
        ax = fig.subplots(1,1)
        ax.set_aspect('equal')
        ax.plot(gp.x, gp.y, '-r')
        ax.plot(reference_trajectory[:,0], reference_trajectory[:,1], 'ob', linewidth=2)
        line_current = None
        line_ref = None

    while True:
        if config.real:
            env.clock_decision.tick_begin()
        state = env.state

        ### algorithm
        t1 = time.time()
        if learning:
            action = method.select_action(state)
        else:
            action = method.select_action(state, reference_trajectory, env.time_step)
        t2 = time.time()
        # action = env.action_space.sample()
        # action[:] = 1

        ### visualize
        if config.visualize:
            if line_current != None:
                line_current.remove()
            if line_ref != None:
                line_ref.remove()
            line_current = ax.plot(state.pose[0], state.pose[1], 'or', linewidth=5)[0]

            ref_pose = method.get_reference_x(state, reference_trajectory, env.time_step)
            line_ref = ax.plot(ref_pose[0], ref_pose[1], 'oy', linewidth=5)[0]
            plt.pause(0.001)

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
        print(f'current_time: {current_time}, algorithm time: {t2-t1}, error path: {np.abs(lateral_e)}')


        ### env step
        experience, epoch_done, info = env.step(action)
        if env.config.render:
            env.render()
        
        if epoch_done == True:
            break
        if config.real:
            env.clock_decision.tick_end()
    
    ### metric
    metric_str = calculate_metric(np.array(error_paths))
    print('metric: ', metric_str)
    if config.visualize:
        ax.set_xlabel(metric_str)
        plt.show()
    else:
        import pdb; pdb.set_trace()
    return





def run_one_episode_learning(config, env: rl_template.EnvSingleAgent, method):
    env.reset()

    agent = env.agents_master.agents[0]
    gp = agent.global_path
    method.set_global_path(gp)
    reference_trajectory = agent.get_reference_trajectory()
    method.set_ref_traj(reference_trajectory)

    start_time = time.time()


    if config.visualize:
        fig = plt.figure(figsize=(18, 18))
        ax = fig.subplots(1,1)
        ax.set_aspect('equal')
        ax.plot(gp.x, gp.y, '-r')
        ax.plot(reference_trajectory[:,0], reference_trajectory[:,1], 'ob', linewidth=2)
        line_current = None
        line_ref = None

    error_paths = []
    while True:
        if config.real:
            env.clock_decision.tick_begin()
        state = env.state

        ### algorithm
        t1 = time.time()
        action = method.select_action(state, reference_trajectory, env.time_step)
        t2 = time.time()
        # action = env.action_space.sample()
        # action[:] = 1

        ### visualize
        if config.visualize:
            if line_current != None:
                line_current.remove()
            if line_ref != None:
                line_ref.remove()
            line_current = ax.plot(state.pose[0], state.pose[1], 'or', linewidth=5)[0]

            ref_pose = method.get_reference_x(state, reference_trajectory, env.time_step)
            line_ref = ax.plot(ref_pose[0], ref_pose[1], 'oy', linewidth=5)[0]
            plt.pause(0.001)

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
        print(f'current_time: {current_time}, algorithm time: {t2-t1}, error path: {np.abs(lateral_e)}')


        ### env step
        experience, epoch_done, info = env.step(action)
        if env.config.render:
            env.render()
        
        if epoch_done == True:
            env.on_episode_end()
            break
        if config.real:
            env.clock_decision.tick_end()
    
    ### metric
    error_paths = np.array(error_paths)
    ratio_path = (np.where(error_paths <= 0.02, 1, 0).sum() / error_paths.shape[0]) *100
    metric_str = f'error path, max: {np.round(np.max(error_paths), 4)}, min: {np.round(np.min(error_paths), 4)}, mean: {np.round(np.mean(error_paths), 4)}, last: {np.round(error_paths[-1], 4)}, ratio: {ratio_path} %'
    print('metric: ', metric_str)
    if config.visualize:
        ax.set_xlabel(metric_str)
        plt.show()
    else:
        import pdb; pdb.set_trace()
    return






if __name__ == "__main__":
    config = rldev.YamlConfig()
    from scripts.args import generate_argparser
    argparser = generate_argparser()
    argparser.add_argument('algo', help='Algo.')
    args = argparser.parse_args()
    config.update(args)

    mode = 'train'
    if config.evaluate == True:
        mode = 'evaluate'
        config.seed += 1
    if config.real:
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
        L = cu.agents.tools.vehicle_wheelbase(rldev.BaseData(type_id=env.scenario.type_id))
        from algorithms.mpc_linear import MPCController as Method
        method = Method(L, 1/env.control_frequency)
    elif algo == 'rwpf':
        learning = False
        env = init_env(config, writer, mode, learning)
        L = cu.agents.tools.vehicle_wheelbase(rldev.BaseData(type_id=env.scenario.type_id))
        from algorithms.lateral_controller import LatRWPF as Method
        method = Method(L, 1/env.control_frequency)
    elif algo == 'pid':
        learning = False
        env = init_env(config, writer, mode, learning)
        L = cu.agents.tools.vehicle_wheelbase(rldev.BaseData(type_id=env.scenario.type_id))
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
        cu.destroy_all_actors(config.core)
        for _ in range(config.num_episodes):
            run_one_episode_no_learning(config, env, method, learning)
    finally:
        print('\n\n\n\n\n\n')
        metric_str, _ = calculate_metric(np.array(error_paths))
        print('metric: ', metric_str)
        writer.close()
        env.destroy()

import rllib
import carla_utils as cu
from carla_utils import rl_template

import torch

import os, sys
sys.path.insert(0, os.path.expanduser('~/github/zdk/tractor-control'))



def run_one_episode(env: rl_template.EnvSingleAgent, method: rllib.td3.TD3):
    env.reset()
    while True:
        state = env.state
        action = method.select_action(state)
        experience, epoch_done, info = env.step(action)
        method.store(experience)

        method.update_parameters()
        if epoch_done == True:
            break
    return



if __name__ == "__main__":
    ### config
    config = cu.system.YamlConfig()
    from scripts.args import generate_argparser
    argparser = generate_argparser()
    argparser.add_argument('version', help='Version.')
    args = argparser.parse_args()
    config.update(args)

    mode = 'train'
    if config.evaluate:
        mode = 'evaluate'
        config.seed +=1
    cu.basic.setup_seed(config.seed)

    ### default param
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    config.set('device', device)
    config.set('n_episode', 100000)

    version = config.version
    if version == 'None':
        raise NotImplementedError

    elif version == 'v1':
        if mode != 'train':
            raise NotImplementedError
        from envs.env_carla_learning import EnvLearning as Env
        from algorithms.rl_modules import TD3 as Method
        from algorithms.rl_modules import ReplayBuffer
        
        config.set('buffer', ReplayBuffer)

        model_name = Method.__name__ + '-' + Env.__name__
        writer = cu.basic.create_dir(config, model_name, mode)

        env = Env(config, writer, mode=mode, env_index=-1)
        method = Method(config, writer)

    else:
        raise NotImplementedError

    try:
        cu.destroy_all_actors(config.core)
        for _ in range(config.n_episode):
            # print('\n\n\n\n\n\n-------------------\n\n')
            run_one_episode(env, method)
    finally:
        writer.close()
        env.destroy()
        # cu.kill_process()


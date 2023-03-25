import rllib

import torch



class TD3(rllib.td3.TD3):
    gamma = 0.9
    start_timesteps = 10000

    lr_critic = 5e-4
    lr_actor = 1e-4

    buffer_size = 1000000
    batch_size = 128

    policy_freq = 4
    explore_noise = 0.1
    policy_noise = 0.2
    noise_clip = 0.4
    



class ReplayBuffer(rllib.buffer.ReplayBuffer):
    def _batch_stack(self, batch):
        result = rllib.buffer.stack_data(batch)
        result = result.cat(dim=0)
        result.reward.unsqueeze_(1)
        result.done.unsqueeze_(1)
        return result




class Evaluate(rllib.EvaluateSingleAgent):
    @torch.no_grad()
    def select_action_td3(self, state):
        self.select_action_start()

        state = state.to(self.device)
        action = self.actor(state)
        # value = self.critic(state, action)

        # print('action: ', action.cpu(), 'value', value)
        # import pdb; pdb.set_trace()

        return action




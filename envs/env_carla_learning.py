import rldev
import carla_utils as cu

import collections
import numpy as np
import torch


from .env_carla import EnvNoLearning, SteerModel
from .env_carla import FREQ, LAG, DELAY, steer_model



class Agent(cu.BaseAgent):
    dim_action = 2
    num_horizon = 20
    def __init__(self, *args):
        super().__init__(*args)
        self.steer_model = steer_model
        # self.steer_model.alpha = 0.95

        assert self.num_horizon > int(DELAY * FREQ)
        self.action_buffer = collections.deque(maxlen=self.num_horizon)
        for _ in range(self.num_horizon):
            self.action_buffer.append(np.zeros(self.dim_action, dtype=np.float32))
        return


    def get_target(self, reference):
        ### curvature
        self.action_buffer.append(reference.numpy())
        curvature = np.clip(reference[0].item(), -1,1) * self.max_curvature
        velocity = (np.clip(reference[1].item(), -1,1) + 1) * self.max_velocity /2
        target = (curvature, velocity)
        return target


    def get_control(self, target):
        current_transform = self.get_transform()

        curvature, velocity = target

        current_v = self.get_current_v()
        current_state = cu.cvt.CuaState.carla_transform(current_transform, v=current_v)
        target_state = cu.cvt.CuaState.carla_transform(current_transform, v=velocity, k=curvature)
        control = self.controller.run_step(current_state, target_state)
        return control




class AgentListMaster(cu.AgentListMaster):
    Agent = Agent

    dim_state = 42 + Agent.dim_action * Agent.num_horizon
    dim_action = Agent.dim_action

    def __init__(self, config, **kwargs):
        super().__init__(config)

        self.num_vehicles = config.num_vehicles
        self.dim_state = config.dim_state
        self.max_velocity = config.max_velocity
        self.perception_range = config.perception_range

        self.perp_gt_vehicle = cu.perception.GroundTruthVehicle(config, 10, dim_state=4)
        self.perp_gt_route = cu.perception.GroundTruthRoute(config, dim_state=40, perception_range=10)


    def get_agent_type(self, learnable=True):
        if learnable:
            agent_type = self.Agent
        else:
            raise NotImplementedError
        return agent_type
    

    def perception(self, index, time_step):
        agent = self.agents_learnable[0]
        state = self.perp_gt_vehicle.run_step(agent, self.obstacles)
        route = self.perp_gt_route.run_step(agent)
        state.update(ref=route)
        history_action = list(agent.action_buffer)
        return torch.from_numpy(
            np.hstack([
            state.ref,
            state.ego,
            np.array([agent.steer_model.y], dtype=np.float32),
            np.stack(history_action).flatten(),
        ])).unsqueeze(0)






class RewardFunction(object):
    def __init__(self, config):
        self.config = config

    def run_step(self, state, action, agents_master: cu.AgentListMaster, epoch_info):
        REWARD_C = -500
        REWARD_B = -200
        REWARD_B = -20  ## single_lane

        agent = agents_master.agents_learnable[0]
        current_transform = agent.get_transform()


        ### 1. collision
        reward_collision = int(epoch_info.c) * REWARD_C

        ### 2. boundary
        reward_boundary = int(epoch_info.b | epoch_info.cs) * REWARD_B

        ### 3. velocity
        reward_v = (agent.get_state().v - agent.max_velocity / 2) / (agent.max_velocity / 2)
        reward_v = np.clip(reward_v, -1, 1)

        # reward_v = (agent.get_state().v) / (agent.max_velocity)
        # reward_v = np.clip(reward_v, 0, 1) *0.5

        ### 4. route
        _, lateral_e, theta_e = agent.global_path.error(current_transform)
        # reward_route = (-0.25*abs(lateral_e)+0.5) + (-0.25*abs(theta_e)+0.5)
        # reward_route = (-1*abs(lateral_e)+0.5) + (-1*abs(theta_e)+0.5)
        reward_route = (-2*abs(lateral_e)+1)
        reward_route = np.clip(reward_route, -1, 1)

        def rescale_reward(reward): ## from [-1,1] to [-1,1]
            offset = 0.8
            reward -= offset
            if reward > 0:
                reward /= (1-offset)
            else:
                reward /= (1+offset)
            return reward
        reward_route = rescale_reward(reward_route)
        # if reward_route > 0:
            # reward_route *= 5
            # reward_route *= 2


        reward = reward_collision + reward_boundary + 1* reward_v + reward_route  ## single_lane

        reward /= 100
        return reward




class EnvLearningEvaluate(EnvNoLearning):
    agents_master_cls = AgentListMaster



class EnvLearning(EnvNoLearning):
    agents_master_cls = AgentListMaster
    reward_function_cls = RewardFunction


    def _check_epoch(self):
        '''check if collision, timeout, success, dones'''
        agent = self.agents_master.agents[0]
        timeout = agent.check_timeout(self.time_tolerance)
        collision = agent.check_collision()

        _, lateral_e, theta_e = agent.global_path.error(agent.get_transform())
        touch_boundary = abs(lateral_e) > 1
        collision_with_static = False
        if collision:
            collision_type = agent.sensors_master.get(('sensor.other.collision', 'default')).get_data().other_actor.type_id
            if collision_type.startswith('static') or collision_type.startswith('traffic'):
                collision_with_static = True

        epoch_done = timeout | collision | collision_with_static | touch_boundary
        epoch_info = rldev.Data(done=epoch_done, t=timeout, c=collision, cs=collision_with_static, b=touch_boundary)
        # print('epoch_info: ', epoch_info)
        return epoch_info




    def on_episode_start(self):
        self.metric_rewards = 0.0
        self.metric_total_steps = 0.0
        self.metric_avg_velocity = 0.0
        self.metric_avg_error = 0.0
        self.metric_smoothness = 0.0
        self.metric_collision = 0.0
        self.metric_boundary = 0.0

        self.metric_infos = []
        return

    def on_episode_step(self, reward, epoch_info):
        self.metric_rewards += reward
        self.metric_infos.append(epoch_info)
        self.metric_total_steps += 1

        agent = self.agents_master.agents[0]
        _, lateral_e, theta_e = agent.global_path.error(agent.get_transform())
        self.metric_avg_velocity += np.clip(agent.get_current_v() /agent.max_velocity, 0,1)
        self.metric_avg_error += np.abs(lateral_e)
        self.metric_smoothness += cu.vector3DNorm(agent.vehicle.get_acceleration()) /9.8
        return


    def on_episode_end(self):
        agent = self.agents_master.agents[0]

        self.metric_collision = int(self.metric_infos[-1].c)
        self.metric_boundary = int(self.metric_infos[-1].b)
        self.metric_avg_velocity /= self.metric_total_steps
        self.metric_avg_error /= self.metric_total_steps
        self.metric_smoothness /= self.metric_total_steps

        self.writer.add_scalar('{}/total_reward'.format(self.env_name), self.metric_rewards, self.step_reset)
        self.writer.add_scalar('{}/total_step'.format(self.env_name), self.metric_total_steps / self.time_tolerance, self.step_reset)
        self.writer.add_scalar('{}/collision'.format(self.env_name), self.metric_collision, self.step_reset)
        self.writer.add_scalar('{}/avg_velocity'.format(self.env_name), self.metric_avg_velocity, self.step_reset)
        self.writer.add_scalar('{}/avg_error'.format(self.env_name), self.metric_avg_error, self.step_reset)
        self.writer.add_scalar('{}/smoothness'.format(self.env_name), self.metric_smoothness, self.step_reset)
        self.writer.add_scalar('{}/boundary'.format(self.env_name), self.metric_boundary, self.step_reset)
        
        return


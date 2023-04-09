import rldev
import carla_utils as cu
from carla_utils import carla
from carla_utils import rl_template
from carla_utils.agents import vehicle_model

import copy
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import cv2
import torch



sensors_param_list = [
    {
        'type_id': 'sensor.other.collision',
        'role_name': 'default',
        'transform': carla.Transform(carla.Location(x=2.5, z=0.7)),
    },

    # {
    #     'type_id': 'sensor.camera.rgb',
    #     'role_name': 'view',
    #     'image_size_x': 640,
    #     'image_size_y': 360,
    #     'fov': 120,
    #     'transform': carla.Transform(carla.Location(x=0, z=2.8), carla.Rotation(pitch=-5)),
    # },
]



class SteerModel(vehicle_model.SteerModel):
    def __init__(self, dt, lag=0.4, delay=0.2):
        """
            lag, delay: second
        """
        self.dt = dt
        self.xk, self.y = 0.0, 0.0

        # https://controlsystemsacademy.com/0020/0020.html
        self.alpha = np.exp(-dt/lag)
        self.n = int(delay / dt)
        self.buffer = deque(maxlen=self.n)
        for _ in range(self.n):
            self.buffer.append(0.0)
        return
    
    def forward(self, u):
        """
            u: normalized control
        """
        self.buffer.append(u)
        self.y = self.xk
        # alpha = np.clip(self.alpha + np.clip(np.random.normal(scale=0.2), -0.2, 0.2), 0, 1)
        alpha = self.alpha
        self.xk = alpha * self.xk + (1-alpha) * self.buffer[0]
        return self.y    




SR = 0.02  ### sampling_resolution
FREQ = 50  ### frequency
# DELAY = 0.2
# LAG = 0.4
DELAY = 0.24165010452270508
LAG = 0.7157812540648414

steer_model = SteerModel(1/FREQ, LAG, DELAY)


class ScenarioRandomization(rl_template.ScenarioRandomization):
    def __init__(self, core, spawn_points, num_vehicles, total_distance):
        self.total_distance = total_distance
        super().__init__(core, spawn_points, num_vehicles)
    

    def generate_global_path(self, spawn_point: carla.Location, *args):
        spawn_transform = cu.get_spawn_transform(self.core, spawn_point, height=0.1)
        waypoint = self.town_map.get_waypoint(spawn_transform.location)
        route = cu.get_reference_route_wrt_waypoint(waypoint, sampling_resolution=SR, sampling_number=int(self.total_distance / SR))
        return cu.GlobalPath(route)




class Scenario(rl_template.ScenarioSingleAgent):
    time_tolerance = FREQ *50
    num_vehicles = 1
    max_velocity = 5.0
    type_id = 'vehicle.tesla.model3'
    obstacle_type_id = 'vehicle.*'

    map_name = 'Town01'
    # map_name = 'agri_rounds_v0'


    def get_scenario_randomization(self):
        total_distance = self.time_tolerance / self.config.control_frequency * self.max_velocity
        scenario_randomization = ScenarioRandomization(self.core, self.spawn_points, self.num_vehicles, total_distance)
        return scenario_randomization




class PerceptionImage(object):
    dim_state = cu.basic.Data(ego=1, route=40, obs=(320, 180))
    def __init__(self, config, **kwargs):
        self.config = config
        self.perp_gt_route = cu.perception.GroundTruthRoute(config, self.dim_state.route, perception_range=30)
        self.leading_range = 30.0
        return

    def run_step(self, step_reset, time_step, agents):
        self.step_reset, self.time_step = step_reset, time_step

        agent = agents[0]

        ego = self.get_state_ego(agent)
        route = self.perp_gt_route.run_step(agent)
        image = agent.sensors_master.get_camera().data[...,:-1]
        image = cv2.resize(image, self.dim_state.obs)
        image = image.astype(np.float32) /255

        return rldev.Data(ego=ego, route=route, obs=image)


    def get_state_ego(self, agent):
        state = np.array([
                agent.get_state().v /agent.max_velocity,
            ], dtype=np.float32)
        return state




    # def viz(self, data):
    #     image = data.obs
    #     save_dir = os.path.join(self.config.path_pack.output_path, str(self.step_reset))
    #     cu.system.mkdir(save_dir)
    #     cv2.imwrite(os.path.join(save_dir, '{}.png'.format(self.time_step)), (image*255).astype(np.uint8))
    #     return




class AgentLongitudinalControl(cu.BaseAgent):
    dim_action = 1
    def get_target(self, reference):
        velocity = (np.clip(reference[0].item(), -1,1) + 1) * self.max_velocity /2
        target = velocity
        return target


    def get_control(self, target):
        current_transform = self.get_transform()
        target_waypoint, curvature = self.global_path.target_waypoint(current_transform)

        velocity = target

        current_v = self.get_current_v()
        current_state = cu.cvt.CuaState.carla_transform(current_transform, v=current_v)
        target_state = cu.cvt.CuaState.carla_transform(target_waypoint.transform, v=velocity, k=curvature)
        control = self.controller.run_step(current_state, target_state)
        print('control: ', control)
        return control



    def get_reference_trajectory(self):
        reference_trajectory = np.zeros((self.config.time_tolerance, 5))
        v = self.max_velocity
        for i in range(self.config.time_tolerance):
            index = int(v * i / self.config.control_frequency / SR)
            reference_trajectory[i, 0] = self.global_path.x[index]
            reference_trajectory[i, 1] = self.global_path.y[index]
            reference_trajectory[i, 2] = self.global_path.theta[index]
            reference_trajectory[i, 3] = v
            reference_trajectory[i, 4] = np.arctan(self.global_path.curvatures[index] * self.wheelbase)  ## ! todo check

        return reference_trajectory





class AgentNoLearning(cu.BaseAgent):
    dim_action = 2

    def __init__(self, config, vehicle, sensors_master, global_path):
        super().__init__(config, vehicle, sensors_master, global_path)
        self.steer_model = steer_model
        # self.steer_model.alpha = 0.95

    def get_target(self, reference):
        target = reference
        return target


    def get_control(self, target):
        vr = target[0]
        steer = target[1]
        curvature = np.tan(steer / self.wheelbase)

        current_transform = self.get_transform()
        current_v = self.get_current_v()
        current_state = cu.cvt.CuaState.carla_transform(current_transform, v=current_v)
        target_state = cu.cvt.CuaState.carla_transform(current_transform, v=vr, k=curvature)
        control = self.controller.run_step(current_state, target_state)
        return control


    def extend_route(self):
        return


      ### constant velocity
    def get_reference_trajectory(self):
        reference_trajectory = np.zeros((self.config.time_tolerance, 5))
        v = self.max_velocity
        for i in range(self.config.time_tolerance):
            index = int(v * i / self.config.control_frequency / SR)
            reference_trajectory[i, 0] = self.global_path.x[index]
            reference_trajectory[i, 1] = self.global_path.y[index]
            reference_trajectory[i, 2] = self.global_path.theta[index]
            reference_trajectory[i, 3] = v
            reference_trajectory[i, 4] = np.arctan(self.global_path.curvatures[index] * self.wheelbase)  ## ! todo check

        return reference_trajectory


    def get_reference_trajectory_v2(self):
        reference_trajectory = np.zeros((self.config.time_tolerance, 5))
        for i in range(self.config.time_tolerance):
            t = i / self.config.control_frequency
            v = np.clip(0.1 *t, 0, self.max_velocity)
            index = int(v * t / SR)
            reference_trajectory[i, 0] = self.global_path.x[index]
            reference_trajectory[i, 1] = self.global_path.y[index]
            reference_trajectory[i, 2] = self.global_path.theta[index]
            reference_trajectory[i, 3] = v
            reference_trajectory[i, 4] = np.arctan(self.global_path.curvatures[index] * self.wheelbase)  ## ! todo check

        return reference_trajectory





class AgentListMaster(cu.AgentListMaster):
    Perception = PerceptionImage
    Agent = AgentNoLearning
    # Agent = AgentLongitudinalControl

    dim_state = Perception.dim_state
    dim_action = Agent.dim_action

    def __init__(self, config, **kwargs):
        super().__init__(config)

        self.num_vehicles = config.num_vehicles
        self.dim_state = config.dim_state
        self.max_velocity = config.max_velocity
        self.perception_range = config.perception_range

        self.perp = self.Perception(config)


    def get_agent_type(self, learnable=True):
        if learnable:
            agent_type = self.Agent
        else:
            raise NotImplementedError
        return agent_type
    

    def perception_learning(self, index, time_step):
        state = self.perp.run_step(index, time_step, self.agents + self.obstacles)
        return state.to_tensor().unsqueeze(0)



    def perception(self, index, time_step):
        agent = self.agents[0]

        state = agent.get_state()
        pose = np.array([state.x, state.y, state.theta])

        return rldev.Data(pose=pose, velocity=state.v)






    def visualize(self, state: rldev.Data):
        image = state.obs.squeeze(0).numpy()
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        plt.imshow(image)
        plt.pause(0.001)
        return




class EnvNoLearning(rl_template.EnvSingleAgent):
    scenario_cls = Scenario
    agents_master_cls = AgentListMaster
    recorder_cls = rl_template.PseudoRecorder

    sensors_params = sensors_param_list

    decision_frequency = FREQ
    control_frequency = FREQ

    perception_range = 50.0


    def reset(self):
        super().reset()
        self.state = self.agents_master.perception(self.step_reset, self.time_step)
        return self.state



    @torch.no_grad()
    def _step_train(self, action):
        self.time_step += 1

        ### state
        state = self.state
        ### reward
        epoch_info = self._check_epoch()
        reward = self.reward_function.run_step(state, action, self.agents_master, epoch_info)
        epoch_done = epoch_info.done
        
        ### record
        self.recorder.record_agents(self.time_step, self.agents_master, epoch_info)
        self.recorder.record_experience(self.time_step, self.agents_master, action)
        
        ### callback
        self.on_episode_step(reward, epoch_info)

        ### step
        self.agents_master.run_step(action)

        ### next_state
        next_state = self.agents_master.perception(self.step_reset, self.time_step)
        self.state = copy.copy(next_state)

        ### experience
        reward = torch.tensor([reward], dtype=torch.float32)
        done = torch.tensor([epoch_done], dtype=torch.float32)
        if done == True:
            self.on_episode_end()
        experience = rldev.Data(
            state=state, action=action, next_state=next_state, reward=reward,
            done=done,
        )
        return experience, epoch_done, epoch_info




    @torch.no_grad()
    def _step_real(self, action):
        self.time_step += 1

        ### state
        state = self.state
        ### reward
        epoch_info = self._check_epoch()
        reward = self.reward_function.run_step(state, action, self.agents_master, epoch_info)
        epoch_done = epoch_info.done
        
        ### record
        self.recorder.record_agents(self.time_step, self.agents_master, epoch_info)
        self.recorder.record_experience(self.time_step, self.agents_master, action)
        
        ### callback
        self.on_episode_step(reward, epoch_info)

        ### step
        self.agents_master.run_step(action)

        ### next_state
        next_state = self.agents_master.perception(self.step_reset, self.time_step)
        self.state = copy.copy(next_state)

        ### experience
        reward = torch.tensor([reward], dtype=torch.float32)
        done = torch.tensor([epoch_done], dtype=torch.float32)
        if done == True:
            self.on_episode_end()
        experience = rldev.Data(
            state=state, action=action, next_state=next_state, reward=reward,
            done=done,
        )
        return experience, epoch_done, epoch_info




    def _check_epoch(self):
        '''check if collision, timeout, success, dones'''
        timeouts = [a.check_timeout(self.time_tolerance) for a in self.agents_master.agents]
        # timeouts = [self.check_timeout()]
        epoch_done = timeouts[0]
        epoch_info = rldev.Data(done=epoch_done, t=timeouts[0])
        return epoch_info




    @property
    def settings(self):
        if self.mode == 'real':
            st = cu.default_settings(sync=False, render=True, dt=0.0)
        else:
            st = cu.default_settings(sync=True, render=True, dt=1/self.control_frequency)
        return st




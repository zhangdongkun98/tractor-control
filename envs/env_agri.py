import rldev
import carla_utils as cu
from carla_utils import carla, navigation
RoadOption = navigation.local_planner.RoadOption

import numpy as np

from gps_common.msg import GPSFix

from driver.clock import Clock
from driver.projection import Projection
from driver.rtk import RTK
from driver.can import CanDriver

from .env_carla import SR, FREQ
from .env_carla import EnvNoLearning, AgentNoLearning


### real vehicle
wheelbase = 1.07

projection = Projection()


class PseudoWaypoint(object):
    def __init__(self, x, y, theta):
        l = carla.Location(x=x, y=y)
        r = carla.Rotation(yaw=np.rad2deg(theta))
        self.transform = carla.Transform(l, r)


class PseudoAgentNoLearning(AgentNoLearning):
    def __init__(self, config, global_path):
        self.config = config
        self.global_path = global_path
        self.max_velocity = config.max_velocity
        self.wheelbase = config.wheelbase

        self.rtk_driver = RTK()
        self.can_driver = CanDriver(rospub=True)
        return



    def get_state(self):
        gps_data = self.rtk_driver.gps_data
        # gps_data = GPSFix()

        x, y = projection.gps2xy(gps_data.latitude, gps_data.longitude)
        theta = projection.track2yaw(gps_data.track)
        v = gps_data.speed
        return cu.State(x=x, y=y, theta=theta, v=v)


    def get_transform(self):
        gps_data = self.rtk_driver.gps_data
        # gps_data = GPSFix()

        x, y = projection.gps2xy(gps_data.latitude, gps_data.longitude)
        theta = projection.track2yaw(gps_data.track)

        l = carla.Location(x=x, y=y)
        r = carla.Rotation(yaw=np.rad2deg(theta))
        return carla.Transform(l, r)




class EnvAgri(EnvNoLearning):
    decision_frequency = FREQ
    control_frequency = FREQ

    def __init__(self, config, mode, learning=False):
        self.set_parameters(config, mode, env_index=-1)
        config.set('time_tolerance', 1000)
        config.set('max_velocity', 1.0)
        config.set('wheelbase', wheelbase)

        self.config = config
        self.mode = mode
        self.learning = learning
        self.step_reset = 0

        self.max_velocity = config.max_velocity
        self.wheelbase = config.wheelbase
        self.pseudo_ref_traj = np.array([[0.0, 0.0, 0.0, self.max_velocity]])

        self.clock = Clock(FREQ)




    def reset(self):

        ### global path
        start_x, start_y = projection.gps2xy(30.258824339333334, 119.72750057183333)
        end_x, end_y = projection.gps2xy(30.2585985237, 119.726511246)

        length = np.hypot(end_x-start_x, end_y-start_y)
        theta = np.arctan2(end_y-start_y, end_x-start_x)
        route = []
        current_length = 0.0
        while True:
            if current_length > length:
                break
            x = start_x + current_length * np.cos(theta)
            y = start_y + current_length * np.sin(theta)
            wp = PseudoWaypoint(x, y, theta)
            route.append((wp, RoadOption.LANEFOLLOW))
            current_length += SR
        global_path = cu.GlobalPath(route)

        if self.learning:
            pass
        else:
            self.agent = PseudoAgentNoLearning(self.config, global_path)

        
        ### env param
        self.step_reset += 1
        self.time_step = 0

        self.state = None
        return
    

    def step(self, method):
        self.time_step += 1
        self.clock.tick_begin()

        state = self.get_state()
        if self.learning:
            action = method.select_action(state)
        else:
            action = method.select_action(state, self.pseudo_ref_traj, self.time_step)

        self.clock.tick_end()
        return



    def get_state(self):  ### no learning
        agent = self.agent

        state = agent.get_state()
        pose = np.array([state.x, state.y, state.theta])

        return rldev.Data(pose=pose, velocity=state.v)
    


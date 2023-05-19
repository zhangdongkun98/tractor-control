import rldev
import carla_utils as cu
import carla_utils.ros as ru
from carla_utils import carla, navigation
RoadOption = navigation.local_planner.RoadOption

import numpy as np
import time
import collections

import rospy
from nav_msgs.msg import Path
from gps_common.msg import GPSFix

from driver.clock import Clock
from driver.projection import projection
from driver.rtk import RTK
from driver.can import CanDriver, PseudoCanDriver

from .env_carla import SR, FREQ
from .env_carla import EnvNoLearning, AgentNoLearning


### real vehicle
wheelbase = 1.07



class PseudoWaypoint(object):
    def __init__(self, x, y, theta):
        l = carla.Location(x=x, y=y)
        r = carla.Rotation(yaw=np.rad2deg(theta))
        self.transform = carla.Transform(l, r)



class LongPID(object):
    def __init__(self, dt):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param offset: distance to the center line. If might cause issues if the value
                is large enough to make the vehicle invade other lanes.
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._dt = dt

        lag = 0.4
        delay = 0.2

        Kp = (dt + delay/2) / (lag + delay/2)
        Ki = dt + delay/2
        Kd = dt * delay / (delay + 2*dt)
        self._k_p = Kp
        self._k_i = Ki * dt
        self._k_d = Kd / dt
        # print(rldev.prefix(self) + f'PID parameter: Kp: {self._k_p}, Ki: {self._k_i}, Kd: {self._k_d}')
        self._e_buffer = collections.deque(maxlen=10)

    def run_step(self, v_current, v_target):
        error = v_target - v_current

        self._e_buffer.append(error)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        res = self._k_p * error + self._k_i * _ie + self._k_d * _de
        # print('[long pid] ',  self._k_p * error, self._k_i * _ie, self._k_d * _de)
        return np.clip(res, -1.0, 1.0)



class PseudoAgentNoLearning(AgentNoLearning):
    def __init__(self, config, global_path, rtk_driver):
        self.config = config
        self.global_path = global_path
        self.max_velocity = config.max_velocity
        self.wheelbase = config.wheelbase

        self.long_pid = LongPID(1/FREQ)
        self.rtk_driver = rtk_driver
        if config.pseudo:
            self.can_driver = PseudoCanDriver()
        else:
            self.can_driver = CanDriver(rospub=True)
        return

    def stop(self):
        self.can_driver.stop_event()
        self.rtk_driver.stop_event()

    def destroy(self):
        self.can_driver.stop_event()
        self.rtk_driver.stop_event()
        self.can_driver.close()
        self.rtk_driver.close()


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


    def apply_control(self, action):
        vr = action[0]
        steer = action[1]
        current_state = self.get_state()
        current_steer = projection.wheel2steer(self.can_driver.recv_steer_wheel)

        ### steer
        control_delta_steer = steer - current_steer
        print(f'[steers, rad] target: {steer}, currrent: {current_steer}', np.rad2deg(control_delta_steer))

        control_gear = self.long_pid.run_step(current_state.v, vr)
        print('speed: ', current_state.v, vr)
        control_gear = (control_gear + 1) *0.5
        ### open loop
        control_gear = 0.3

        ### apply
        # control_gear = 0
        # control_steer = np.clip(projection.steer2wheel(control_delta_steer), -90, 90)
        control_steer = np.clip(projection.steer2wheel(control_delta_steer) -3.5, -90, 90)
        print(f'[control] gear: {control_gear}, steer: {control_steer}')
        self.can_driver.set_gear(control_gear)
        self.can_driver.set_rotation(control_steer)



def get_global_path_fix():
    # start_x, start_y = 6.058451788499951, -29.40062252106145
    # start_x, start_y = 6.058451788499951, -29.40062252106145
    # end_x, end_y = 6.058451788499951, 0.0
    start_x, start_y = 8.0, -29.40062252106145
    end_x, end_y = 8.0, 0.0
    # 1.6, -25.6
    start_x, start_y = 1.6, -25.7
    end_x, end_y = -100, -25.7
    start_x, start_y = 1.6, -29.
    end_x, end_y = -100, -29.

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
    return global_path



def get_global_path(x0, y0, theta0):  ### todo: check
    theta0 = rldev.pi2pi(theta0)

    start_x, start_y = x0, y0

    ### v1: east or west
    if theta0 < np.deg2rad(90) and theta0 > -np.deg2rad(90):
        theta = 0.0
    else:
        theta = np.pi

    ### v2
    theta = theta0

    length = 100

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
    return global_path






class EnvAgri(EnvNoLearning):
    decision_frequency = FREQ
    control_frequency = FREQ

    def __init__(self, config, mode, learning=False):
        rospy.init_node('env_agri', anonymous=False)
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

        self.publisher_path = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)



    def reset(self):

        ### global path
        # start_x, start_y = projection.gps2xy(30.258824339333334, 119.72750057183333)
        # end_x, end_y = projection.gps2xy(30.2585985237, 119.726511246)


        rtk_driver = RTK(rospub=True)
        print('sleep 1s, generate global path')
        time.sleep(2)
        gps_data = rtk_driver.gps_data
        x0, y0 = projection.gps2xy(gps_data.latitude, gps_data.longitude)
        theta0 = projection.track2yaw(gps_data.track)
        global_path = get_global_path(x0, y0, theta0)

        header = ru.cvt.header('map', time.time())
        self.publisher_path.publish( ru.cvt.NavPath.cua_global_path(header, global_path) )

        if self.learning:
            pass
        else:
            self.agent = PseudoAgentNoLearning(self.config, global_path, rtk_driver)

        time.sleep(2)
        print('sleep a little')

        ### env param
        self.step_reset += 1
        self.time_step = 0

        self.state = None
        return
    

    def step(self, method):
        self.clock.tick_begin()
        self.time_step += 1

        state = self.get_state()
        if self.learning:
            action = method.select_action(state)
        else:
            action = method.select_action(state, self.pseudo_ref_traj, self.time_step)

        self.agent.apply_control(action[0])

        self.clock.tick_end()
        return



    def get_state(self):  ### no learning
        agent = self.agent

        state = agent.get_state()
        print('state: ', state, self.agent.can_driver.recv_steer_wheel, np.rad2deg(projection.wheel2steer(self.agent.can_driver.recv_steer_wheel)))
        pose = np.array([state.x, state.y, state.theta])


        return rldev.Data(pose=pose, velocity=state.v)
    

    def stop(self):
        self.agent.stop()


    def destroy(self):
        self.agent.destroy()

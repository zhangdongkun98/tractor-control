import rldev
import carla_utils as cu

import numpy as np
from collections import deque



class LatRWPF(object):
    """
        Paper: A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles
    """
    def __init__(self, wheelbase, dt):
        self.wheelbase = wheelbase
        self.max_steer = np.deg2rad(45)
        self.max_curvature = np.tan(self.max_steer) / self.wheelbase
        self.curvature_factor = 1.0
        self.alpha = 1.8

        k_theta, k_e = 0.2, 0.8
        self.w_param = (k_theta, k_e)
    

    def set_global_path(self, global_path: cu.GlobalPath):
        self.global_path = global_path

    def set_ref_traj(self, ref_traj):
        return


    def select_action(self, state, ref_traj, time_step):

        current_state = cu.State(x=state.pose[0], y=state.pose[1], theta=state.pose[2], v=state.velocity)
        current_transform = cu.cvt.CarlaTransform.cua_state(current_state)
        target_waypoint, curvature = self.global_path.target_waypoint(current_transform)
        target_state = cu.cvt.CuaState.carla_transform(target_waypoint.transform, v=current_state.v, k=curvature)
        


        steer = self.run_step(current_state, target_state, self.w_param)

        control = np.array([ref_traj[:,3].max(), steer])
        # print(rldev.prefix(self) + f'step {time_step}: control {control}, {current_state.v}')
        # print(f'current_state: {current_state}')
        # print(f'target_state: {target_state}')
        # print()
        return np.expand_dims(control, axis=0)


    def get_reference_x(self, state, ref_traj, time_step):
        current_state = cu.State(x=state.pose[0], y=state.pose[1], theta=state.pose[2], v=state.velocity)
        current_transform = cu.cvt.CarlaTransform.cua_state(current_state)
        target_waypoint, curvature = self.global_path.target_waypoint(current_transform)
        target_state = cu.cvt.CuaState.carla_transform(target_waypoint.transform, v=current_state.v, k=curvature)
        return np.array([target_state.x, target_state.y, target_state.theta])




    def run_step(self, current_state, target_state, param):
        k_theta, k_e = param[0], param[1]

        longitudinal_e, lateral_e, theta_e = cu.error_state(current_state, target_state)

        vr, kr = target_state.v, target_state.k


        c1 = (kr*self.curvature_factor) *np.cos(theta_e)
        c2 = - k_theta *theta_e
        c3 = (k_e*np.exp(-theta_e**2/self.alpha))*lateral_e
        curvature = c1 + c2 + c3

        curvature = np.clip(curvature, -self.max_curvature, self.max_curvature)
        steer = np.arctan(curvature * self.wheelbase)
        return steer




class LatPID(LatRWPF):
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, wheelbase, dt):
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
        self.wheelbase = wheelbase
        self._dt = dt
        self.max_steer = np.deg2rad(45)

        self._k_p = 0.5
        self._k_i = 5 * dt  ## 0.0
        self._k_d = 0.01 / dt

        lag = 0.4
        delay = 0.2

        Kp = (dt + delay/2) / (lag + delay/2)
        Ki = dt + delay/2
        Kd = dt * delay / (delay + 2*dt)
        self._k_p = Kp
        self._k_i = Ki * dt
        self._k_d = Kd / dt
        print(rldev.prefix(self) + f'PID parameter: Kp: {self._k_p}, Ki: {self._k_i}, Kd: {self._k_d}')

        # self._k_p = 0.5
        # self._k_i = 0.1  ## 0.0
        # self._k_d = 0.6

        self._e_buffer = deque(maxlen=10)
        self.w_param = None

    def run_step(self, current_state, target_state, param):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoin.

            :param waypoint: target waypoint
            :return: steering control in the range [-1, 1] where:
            -1 maximum steering to left
            +1 maximum steering to right
        """
        """
        Estimate the steering angle of the vehicle based on the PID equations

            :param waypoint: target waypoint
            :param vehicle_transform: current transform of the vehicle
            :return: steering control in the range [-1, 1]
        """
        # Get the ego's location and forward vector
        v_vec = np.array([np.cos(current_state.theta), np.sin(current_state.theta), 0.0])

        # Get the vector vehicle-target_wp


        w_vec = np.array([target_state.x - current_state.x,
                          target_state.y - current_state.y,
                          0.0])
        w_vec /= np.linalg.norm(w_vec)

        _dot = np.arccos(np.clip(np.dot(w_vec, v_vec) / (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        longitudinal_e, lateral_e, theta_e = cu.error_state(current_state, target_state)
        _dot = lateral_e

        kr = target_state.k


        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0


        print('\n')
        print('angle: ', np.rad2deg(current_state.theta), np.rad2deg(np.arctan2(target_state.y - current_state.y, target_state.x - current_state.x)))
        print('error angle: ', np.rad2deg(_dot))
        print('pid error: ', _dot, _ie, _de)
        print('pid: ', self._k_p * _dot, self._k_i * _ie, self._k_d * _de)

        steer0 = np.arctan(kr * self.wheelbase)
        steer = steer0 + self._k_p * _dot + self._k_i * _ie + self._k_d * _de
        return np.clip(steer, -1.0, 1.0) *self.max_steer


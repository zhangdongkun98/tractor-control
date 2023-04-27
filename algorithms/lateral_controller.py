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
    def __init__(self, wheelbase, dt, scale=1.0):
        self.wheelbase = wheelbase
        self._dt = dt
        self.scale = scale
        self.max_steer = np.deg2rad(45)

        self.Kp = 0.5
        self.Ki = 5 * dt  ## 0.0
        self.Kd = 0.01 / dt

        from .params import DELAY, LAG
        lag = LAG
        delay = DELAY

        Kp = (dt + delay/2) / (lag + delay/2)
        Ki = dt + delay/2
        Kd = dt * delay / (delay + 2*dt)
        self.Kp = Kp
        self.Ki = Ki * dt
        self.Kd = Kd / dt *50
        # self.Kd = Kd / dt *100
        # self.Kd = Kd / dt /3
        # self.Kp = 0.2
        self.Ki = 0.0
        self.Kd = 0.1

        # self.Kp = 0.2
        # self.Ki = Ki * dt *0
        # self.Kd = 0.1

        print(rldev.prefix(self) + f'PID parameter c: Kp: {Kp}, Ki: {Ki}, Kd: {Kd}')
        print(rldev.prefix(self) + f'PID parameter: Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}')
        # import pdb; pdb.set_trace()

        self.e_buffer = deque([0.0], maxlen=10)

        self.w_param = None

    def run_step(self, current_state, target_state, param):
        longitudinal_e, lateral_e, theta_e = cu.error_state(current_state, target_state)

        if abs(lateral_e) > 0.025:
            Kp = self.Kp *1.5
            # Kp = self.Kp *3
        else:
            # Kp = self.Kp
            Kp = self.Kp *0.7

        error = lateral_e *self.scale

        kr = target_state.k
        steer0 = np.arctan(kr * self.wheelbase)


        error_i = sum(self.e_buffer) + error
        error_d = error - self.e_buffer[-1]
        self.e_buffer.append(error)

        steer = steer0 + Kp * error + self.Ki * error_i + self.Kd * error_d
        print('\n')
        print('error (m): ', lateral_e)
        print('pid error: ', error, error_i, error_d)
        print(f'pid [steer]: ', self.Kp * error, self.Ki * error_i, self.Kd * error_d)

        return np.clip(steer, -1.0, 1.0) *self.max_steer


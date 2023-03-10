import carla_utils as cu

import numpy as np



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


    def select_action(self, state, ref_traj, time_step):

        current_state = cu.State(x=state.pose[0], y=state.pose[1], theta=state.pose[2], v=state.velocity)
        current_transform = cu.cvt.CarlaTransform.cua_state(current_state)
        target_waypoint, curvature = self.global_path.target_waypoint(current_transform)
        target_state = cu.cvt.CuaState.carla_transform(target_waypoint.transform, v=current_state.v, k=curvature)
        


        steer = self.run_step(current_state, target_state, self.w_param)

        control = np.array([ref_traj[:,3].max(), steer])
        print(f'[mpc] step {time_step}: control {control}, {current_state.v}')
        print(f'current_state: {current_state}')
        print(f'target_state: {target_state}')
        print()
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


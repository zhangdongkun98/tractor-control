
import numpy as np


class BicycleModel(object):
    def __init__(self, x0, y0, theta0, L, T): # L:wheel base
        self.x = x0 # X
        self.y = y0 # Y
        self.theta = theta0 # headding
        self.l = L  # wheel base
        self.dt = T  # decision time periodic

        self.noise_std = np.array([0.00, 0.01, 0.])

    def get_state(self):
        return np.array([self.x, self.y, self.theta])


    def update(self, v, delta):  # update ugv's state
        # self.v = v

        dx = v * np.cos(self.theta)
        dy = v * np.sin(self.theta)
        dtheta = v * np.tan(delta) / self.l

        self.x += dx * self.dt
        self.y += dy * self.dt
        self.theta += dtheta * self.dt

        noise = np.random.normal(loc=0.0, scale=self.noise_std)
        self.x += noise[0]
        self.y += noise[1]
        self.theta += noise[2]
        


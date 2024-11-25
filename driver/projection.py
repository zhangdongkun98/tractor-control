import rldev

import math
import numpy as np

class Projection(object):
    # def __init__(self, ref_lat=30.258824339333334, ref_lon=119.72750057183333):
    def __init__(self, ref_lat=30.11017668, ref_lon=119.76392807):
        self.R = 6378160

        self.ref_lat_rad = math.radians(ref_lat)
        self.ref_lon_rad = math.radians(ref_lon)

        self.x0 = self.R * self.ref_lon_rad * np.cos(self.ref_lat_rad)
        self.y0 = self.R * self.ref_lat_rad
        return


    def gps2xy(self, lat, lon):
        lat_rad = np.deg2rad(lat)
        lon_rad = np.deg2rad(lon)

        x = self.R * lon_rad * np.cos(self.ref_lat_rad)
        y = self.R * lat_rad

        return x -self.x0, y -self.y0

    def xy2gps(self, x, y):
        x += self.x0
        y += self.y0
        lon_rad = x / self.R / np.cos(self.ref_lat_rad)
        lat_rad = y / self.R
        return np.rad2deg(lon_rad), np.rad2deg(lat_rad)



    def track2yaw(self, track):
        return rldev.pi2pi_numpy(np.deg2rad(-track + 90))



    def wheel2steer(self, wheel):
        """
            wheel: deg
        """
        # wheel -= 158.8  ### ! warning: todo
        # return np.deg2rad(wheel) * (30 / 360)
        return -np.deg2rad(wheel) * (30 / 360)


    def steer2wheel(self, steer):
        """
            steer: rad
        """
        # return np.rad2deg(steer) * (360 / 30)
        return -np.rad2deg(steer) * (360 / 30)





projection = Projection()

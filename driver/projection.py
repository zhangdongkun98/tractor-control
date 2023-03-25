
import math
import numpy as np

class Projection(object):
    def __init__(self, ref_lat=30.258824339333334, ref_lon=119.72750057183333):
        self.R = 6378160

        self.ref_lat_rad = math.radians(ref_lat)
        self.ref_lon_rad = math.radians(ref_lon)

        self.x0 = self.R * self.ref_lon_rad * np.cos(self.ref_lat_rad)
        self.y0 = self.R * self.ref_lat_rad
        return


    def gps2xy(self, lat, lon):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        x = self.R * lon_rad * np.cos(self.ref_lat_rad)
        y = self.R * lat_rad

        return x -self.x0, y -self.y0


    def track2yaw(self, track):
        return np.deg2rad(-track + 90)


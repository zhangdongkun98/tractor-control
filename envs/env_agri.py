
from driver.projection import Projection
from driver.rtk import RTK
from driver.can import CanDriver



class EnvAgri(object):
    def __init__(self):
        self.projection = Projection()
        self.rtk_driver = RTK()
        self.can_driver = CanDriver(rospub=True)


    def reset(self):
        pass
    

    def step(self, method):
        pass


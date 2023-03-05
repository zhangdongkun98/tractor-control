
# Device: Kvaser leaf light 2xHS
import time
import numpy as np


from can_driver.driver import CanDriver
from can_driver.kb_controller import KeyBoardController



# def 


class Clock(object):
    def __init__(self, frequency):
        self.frequency = float(frequency)
        self.dt = 1 / self.frequency
        self._tick_begin = None

    def tick_begin(self):
        self._tick_begin = time.time()
        return self._tick_begin
    def tick_end(self):
        _tick_end = time.time()
        sleep_time = self.dt - _tick_end + self._tick_begin
        time.sleep( max(0, sleep_time) )
        return _tick_end




if __name__ == '__main__':
    can_driver = CanDriver()
    can_driver.start()
    # can_driver.set_read_angle()
    # sleep(0.1)
    can_driver.set_query_mode(False, 1)
    time.sleep(0.1)


    
    clock = Clock(10)

    start_time = time.time()

    try:
        
        
        # while True:
        #     clock.tick_begin()
        #     deg = (time.time() - start_time) *30
        #     deg2 = np.sin(np.deg2rad(deg)) *50
        #     print('deg: ', deg2)
        #     can_driver.set_rotation(deg2)
        #     # import pdb; pdb.set_trace()
        #     clock.tick_end()
        
        kb_controller = KeyBoardController(can_driver)
        kb_controller.run()



    except KeyboardInterrupt:
        can_driver.stop_event()
    finally:
        can_driver.close()


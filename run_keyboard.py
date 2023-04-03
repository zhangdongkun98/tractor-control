
import time
import numpy as np


from driver.can import CanDriver
from driver.kb_controller import KeyBoardController




if __name__ == '__main__':
    can_driver = CanDriver(rospub=False, steer_enable=False)

    try:
        
        kb_controller = KeyBoardController(can_driver)
        kb_controller.run()



    except KeyboardInterrupt:
        can_driver.stop_event()
    finally:
        can_driver.stop_event()
        can_driver.close()


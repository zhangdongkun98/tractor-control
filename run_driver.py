
import time
import numpy as np
import rospy


from driver.clock import Clock
from driver.rtk import RTK
from driver.can import CanDriver
from kb_controller import KeyBoardController




if __name__ == '__main__':
    rospy.init_node('driver', anonymous=False)
    rtk_driver = RTK()
    can_driver = CanDriver()


    clock = Clock(100)

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
        rtk_driver.stop_event()
        can_driver.stop_event()
    finally:
        rtk_driver.close()
        can_driver.close()


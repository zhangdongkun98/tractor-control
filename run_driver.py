
import time
import numpy as np
import rospy


from driver.clock import Clock
from driver.rtk import RTK
from driver.can import CanDriver
# from driver.kb_controller import KeyBoardController




if __name__ == '__main__':
    rospy.init_node('driver', anonymous=False)
    rtk_driver = RTK(rospub=True)
    can_driver = CanDriver(rospub=True)


    clock = Clock(10)

    start_time = time.time()

    try:
        
        # can_driver.request_max_gear()
        # import pdb; pdb.set_trace()

        # can_driver.request_gear_enable()
        # rospy.spin()

        # can_driver.set_delta_gear( -1000 / can_driver.max_gear)
        # rospy.spin()

        
        while not rospy.is_shutdown():
            clock.tick_begin()
            # print('recv_steer_wheel: ', can_driver.recv_steer_wheel)
            # print(f'steer_enable: {can_driver.steer_enable}')
            # print('\n')
        #     deg = (time.time() - start_time) *30
        #     deg2 = np.sin(np.deg2rad(deg)) *50
        #     # print('deg: ', deg2)
            
        #     # can_driver.set_rotation(deg2)

        #     can_driver.set_gear(000 / can_driver.max_gear)
        #     # can_driver.set_delta_gear( -1000 / can_driver.max_gear)
        #     # can_driver.set_gear(0)

        #     # can_driver.request_gear_enable()
        #     import pdb; pdb.set_trace()
            clock.tick_end()
        
        # kb_controller = KeyBoardController(can_driver)
        # kb_controller.run()



    except KeyboardInterrupt:
        can_driver.stop_event()
        rtk_driver.stop_event()
    finally:
        can_driver.stop_event()
        rtk_driver.stop_event()
        can_driver.close()
        rtk_driver.close()


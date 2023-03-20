
import os, sys
sys.path.insert(0, os.path.expanduser('~/github/zdk/tractor-control'))


from driver.can import CanDriver
from driver.kb_controller import KeyBoardController



if __name__ == '__main__':
    # rospy.init_node('driver', anonymous=False)
    # rtk_driver = RTK()
    can_driver = CanDriver()



    # clock = Clock(100)


    try:
        can_driver.deactivate_gear()
        can_driver.activate_gear()
        
        # can_driver.request_max_gear()
        # import pdb; pdb.set_trace()

        # can_driver.request_gear_enable()
        # rospy.spin()

        # can_driver.set_delta_gear( -1000 / can_driver.max_gear)
        # rospy.spin()

        kb_controller = KeyBoardController(can_driver)
        kb_controller.run()
        
        # while True:
            
        #     # can_driver.set_rotation(deg2)

        #     can_driver.set_delta_gear( -500 / can_driver.max_gear)

        #     import pdb; pdb.set_trace()



    except KeyboardInterrupt:
        can_driver.stop_event()
    finally:
        can_driver.stop_event()
        can_driver.close()



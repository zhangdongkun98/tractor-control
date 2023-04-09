
import time
import numpy as np
import rospy


from driver.projection import projection
from driver.clock import Clock
from driver.rtk import RTK
from driver.can import CanDriver
# from driver.kb_controller import KeyBoardController

import matplotlib.pyplot as plt



if __name__ == '__main__':
    rospy.init_node('recog', anonymous=False)
    can_driver = CanDriver(rospub=True)


    clock = Clock(1000)

    start_time = time.time()

    try:

        ### set to zero
        current_steer = projection.wheel2steer(can_driver.recv_steer_wheel)
        control_delta_steer = 0 - current_steer
        control_steer = projection.steer2wheel(control_delta_steer)
        can_driver.set_rotation(control_steer)

        # import pdb; pdb.set_trace()
        import tqdm
        for i in tqdm.tqdm(range(10)):
            time.sleep(1)

        start_time = time.time()

        ### set to reference
        reference_steer_wheel = 90
        reference_steer = projection.wheel2steer(reference_steer_wheel)
        can_driver.set_rotation(reference_steer_wheel)


        # while not rospy.is_shutdown():
        times = []
        steers = []
        reference_steers = []
        while True:
            clock.tick_begin()

            current_time = time.time() - start_time
            if current_time > 12:
                break


            current_steer = projection.wheel2steer(can_driver.recv_steer_wheel)

            times.append(current_time)
            steers.append(current_steer)
            reference_steers.append(reference_steer)


            # plt.plot(current_time, reference_steer, 'og')
            # plt.plot(current_time, current_steer, 'or')
            # plt.pause(0.001)


            clock.tick_end()
        
        # kb_controller = KeyBoardController(can_driver)
        # kb_controller.run()

        np.save(f'./results/{reference_steer_wheel}_times.npy', np.array(times))
        np.save(f'./results/{reference_steer_wheel}_steers.npy', np.array(steers))
        np.save(f'./results/{reference_steer_wheel}_reference_steers.npy', np.array(reference_steers))

        plt.plot(times, reference_steers, 'og')
        plt.plot(times, steers, 'or')
        plt.show()




    except KeyboardInterrupt:
        can_driver.stop_event()
    finally:
        can_driver.stop_event()
        can_driver.close()


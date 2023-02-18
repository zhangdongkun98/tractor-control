
import rospy
from std_msgs.msg import Float64MultiArray

from can_manager import CANManager
from vehicle_basic import VehicleCtrl

# def callback_cmd(msg):
#     global can_manager
#     flag = msg.data[0]
#     if flag == 1:
#         print('Auto mode')
#         can_manager.change_auto_mode()
#     elif flag == 2:
#         print('D gear')
#         can_manager.change_d_gear()
#     elif flag == 3:
#         print('Warm start')
#         can_manager.startup_warm()
#     elif flag == 4:
#         print('Roll steer')
#         can_manager.roll_steer()
#     elif flag == 5:
#         print('Ctrl')

#         vehicle_ctrl = VehicleCtrl(
#             gas = int(msg.data[1]),
#             steer = int(msg.data[2]),
#             steer_speed = int(msg.data[3]),
#             brake = int(msg.data[4])
#         )
#         gas, steer, steer_speed, brake = vehicle_ctrl.get_value()
#         print(gas, steer, steer_speed, brake)
#         can_manager.vehicle_control(gas, steer, steer_speed, brake)
#     else:
#         print('Stop !!!')
#         can_manager.emergency_stop()


rospy.init_node('chery_can', anonymous=True)
can_rate = rospy.Rate(100)
can_manager = CANManager(rate=can_rate)

# import time
# can_manager.change_auto_mode()
# # can_manager.roll_steer()
# can_manager.change_d_gear()
# print('!!!!!!!!!!!!!!!   start  !!!!!!!!!!!!!!!')

# for j in range(10):
#     for i in range(130):
#         can_manager.vehicle_control(gasSet=0, steerSet=90, steerSpd=180, brakeSet=0)
#         time.sleep(0.01)

#     for i in range(130):
#         can_manager.vehicle_control(gasSet=0, steerSet=-90, steerSpd=180, brakeSet=0)
#         time.sleep(0.01)

# print('******************************************************')

state_pub = rospy.Publisher('/vehicle_state', Float64MultiArray, queue_size=0)
# rospy.Subscriber('/vehicle_ctrl', Float32MultiArray, callback_cmd)

rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    can_manager.read_state()
    if can_manager.vehicle_state.update:
        state = Float64MultiArray()
        state.data = can_manager.vehicle_state.get_value()
        state_pub.publish(state)
        can_manager.vehicle_state.update = False
    rate.sleep()
    

from time import sleep
import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node('chery_can_cmd', anonymous=True)
cmd_pub = rospy.Publisher('/vehicle_ctrl', Float32MultiArray, queue_size=0)

# cmd = Float32MultiArray()
# cmd.data = [1, 0, 0, 0, 0]
# [flag, gas, steer, steer_speed, brake]
# cmd_pub.publish(cmd)
# sleep(5)

cmd = Float32MultiArray()
cmd.data = [2, 0, 0, 0, 0]
cmd_pub.publish(cmd)
sleep(2)

for _ in range(5000):
    cmd = Float32MultiArray()
    cmd.data = [5, 0, 90, 90, 0]
    cmd_pub.publish(cmd)

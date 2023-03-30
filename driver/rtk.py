
import rospy
import serial
import threading
import time

import pynmea2
from std_msgs.msg import Header
from gps_common.msg import GPSFix



class RTK(object):
    def __init__(self, port='/dev/ttyUSB0'):
        self.serial = serial.Serial(port, 230400)
        
        self.ros_pub = rospy.Publisher('/rtk_data', GPSFix, queue_size=1)

        self.gps_data = None
        self.stop_recv = threading.Event()
        recv_thread = threading.Thread(target=self.read_serial)
        self.stop_recv.clear()
        recv_thread.start()


    def stop_event(self):
        self.stop_recv.set()

    def close(self):
        self.serial.close()



    def read_serial(self):
        msg_seq = 0
        while not self.stop_recv.is_set():
            data = self.serial.readline()

            if data[:6] != b'$GPCHC':
                continue

            # data_list = data.decode('utf8').split(',')
            # print('data: ', data_list)
            # lat = data_list[13 -1]
            # lon = data_list[14 -1]

            data_list = data.decode('utf8').split(',')
            # print('[GPCHC] data: ', data_list)

            lat = float(data_list[13 -1])
            lon = float(data_list[14 -1])
            heading = float(data_list[4 -1])
            speed = float(data_list[19 -1])
            # print('[GPCHC] speed: ', speed)
            # print('\n')

            msg_seq += 1
            # print('true course: ', rmc.true_course)

            ros_msg = GPSFix()
            ros_msg.header = Header()
            ros_msg.header.seq = msg_seq
            ros_msg.header.stamp = rospy.Time.from_sec(time.time())

            ros_msg.latitude = lat
            ros_msg.longitude = lon
            ros_msg.track = heading
            ros_msg.speed = speed
            # print(ros_msg)
            # print('\n\n\n')
            self.gps_data = ros_msg
            self.ros_pub.publish(ros_msg)




if __name__ == '__main__':
    rospy.init_node('rtk_driver', anonymous=False)
    rtk = RTK()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

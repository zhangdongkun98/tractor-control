import rldev

import tqdm
import time
import threading
import platform
import numpy as np

import canlib.canlib as canlib
from canlib.canlib import ChannelData

try:
    import rospy
    from std_msgs.msg import Header
    from gps_common.msg import GPSFix
    from std_msgs.msg import Float64MultiArray
except:
    pass


class PseudoChannel(object):
    def write_raw(self, *args, **kwargs):
        return

    def read(self):
        return None, None, None, None, None

    def busOff(self):
        return
    def close(self):
        return


# Device: Kvaser leaf light 2xHS
class CanDriver(object):
    def __init__(self, rospub=False):
        self.send_freq = 100 # 5Hz 发送频率
        self.max_speed = 1000000
        self.msgId = 0x200 # 发的ID
        # self.msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.cmd = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.flg = canlib.canMSG_EXT

        self.send_channel = self.set_up_channel(channel=0)
        # self.send_channel = PseudoChannel()
        self.recv_channel = self.send_channel
        

        ### data
        self.steer_enable = 0
        self.gear_enable = 0
        self.max_gear = 20000

        self.stop_recv = threading.Event()
        self.recv_cyclic = threading.Thread(target=self.recv, args=())
        self.stop_recv.clear()
        self.recv_cyclic.start()


        self.rotation_time = 0.0
        self.rotation = -10000
        self.delta_gear_time = 0.0
        self.delta_gear = -10000
        self.recv_steer_wheel_time = 0.0
        self.recv_steer_wheel = -10000

        self.rospub = rospub
        if rospub:
            self.ros_pub = rospy.Publisher('/can_data', Float64MultiArray, queue_size=1)
            self.stop_pub = threading.Event()
            self.pub_cyclic = threading.Thread(target=self.publish, args=())
            self.stop_pub.clear()
            self.pub_cyclic.start()


        # print('waiting ...')
        # for _ in tqdm.tqdm(range(30)):
        #     time.sleep(0.1)

        # self.request_max_gear()
        self.set_query_mode()

        self.activate_steer()
        self.activate_gear()

        time.sleep(1.1)
        return
    

    def set_up_channel(self,  channel=0,
                    openFlags=canlib.canOPEN_ACCEPT_VIRTUAL,
                    bitrate=canlib.canBITRATE_250K,
                    bitrateFlags=canlib.canDRIVER_NORMAL):
        ch = canlib.openChannel(channel, openFlags)
        print("Using channel: %s, EAN: %s" % (
            ChannelData(channel).channel_name, 
            ChannelData(channel).card_upc_no))
        ch.setBusOutputControl(bitrateFlags)
        ch.setBusParams(bitrate)
        ch.busOn()
        return ch


    def stop_event(self):
        # self.stop_send.set()
        # self.stop_recv.set()

        self.stop_gear()
        self.deactivate_steer()
        # self.deactivate_gear()

    def close(self):
        # self.deactivate_steer()
        # self.deactivate_gear()
        self.stop_recv.set()
        if self.rospub:
            self.stop_pub.set()

        def tear_down_channel(ch):
            ch.busOff()
            ch.close()
        tear_down_channel(self.send_channel)
        tear_down_channel(self.recv_channel)



    def set_send_id(self, id):
        self.msgId = id


    # def set_msg(self, msg):
    #     self.msg = msg


    def recv(self):
        print("Start receiving messages")
        # file=open('msg.txt', 'w+')
        while not self.stop_recv.is_set():
            try:
                (msgId, msg, dlc, flg, _time) = self.recv_channel.read()  # deprecated in v1.5
                if msgId == 0x100:

                    ### steer wheel
                    if msg[0] == 0x81 and msg[1] == 0x01:
                        # print('fff', msg)
                        # value = msg[4]*0xff+msg[5]
                        # print('value', 255-msg[4], 255-msg[5])
                        if msg[4] > 0xf0:
                            value = - ((255-msg[4])*255+ (255-msg[5]))
                        else:
                            value = (msg[4])*255+ msg[5]
                        # print(value, msg[4], msg[5])

                        steer_wheel = value /10  ### deg
                        self.recv_steer_wheel_time = time.time()
                        self.recv_steer_wheel = steer_wheel
                        # steer_wheel = '0b' + str(msg[4]) + str(msg[5])
                        # steer_wheel = int(steer_wheel, 2)
                        print('-----------------------------steer_wheel: ', steer_wheel)
                        # print('\n')

                        # if value > 0x00ff:
                            # print(-(0xffff-value + 1))
                        # else:
                            # print(value)		
                    # file.write(str(hex(msgId)) + '\t' + data+'\n')

                    ### max gear
                    if msg[0] == 0x81 and msg[1] == 0x85:
                        ### deprecate
                        print('MSG: ', msg[0], hex(msg[0]), bin(msg[0]))
                        max_gear = '0b' + bin(msg[2])[2:] + bin(msg[3])[2:] + bin(msg[4])[2:] + bin(msg[5])[2:]
                        max_gear = int(max_gear, 2)
                        self.max_gear = max_gear
                        # print('-----------------------------max gear: ', max_gear)

                    ### gear enbale
                    # if msg[0] == 0x81 and msg[1] == 0x03:
                    #     print('gear enable: ', msg[2])

                    ### steer info
                    if msg[0] == 0x82 and msg[1] == 0x01:
                        steer_info = '0b' + format(msg[5], '#010b')[2:] + format(msg[4], '#010b')[2:] + format(msg[3], '#010b')[2:] + format(msg[2], '#010b')[2:]

                        steer_enable = int(steer_info[-1- 0])
                        self.steer_enable = int(not steer_enable)

                    ### gear info
                    if msg[0] == 0x82 and msg[1] == 0x02:
                        # ii = msg[2]*1000 + msg[3]*100 + msg[4]*10 + msg[5]
                        
                        gear_info = '0b' + format(msg[5], '#010b')[2:] + format(msg[4], '#010b')[2:] + format(msg[3], '#010b')[2:] + format(msg[2], '#010b')[2:]
                        
                        # print('msg: ',  msg[2],  msg[3],  msg[4],  msg[5])
                        # print('msg hex: ',  hex(msg[2]),  hex(msg[3]),  hex(msg[4]),  hex(msg[5]))
                        # print('msg hex: ', format(msg[2], '#010b'), format(msg[3], '#010b'), format(msg[4], '#010b'), format(msg[5], '#010b'))
                        
                        # print('ii: ', ii, format(ii, '#034b'))
                        # gear_info = format(ii, '#034b')
                        # print('gear_info: ', gear_info[-1- 13], gear_info)
                        gear_enable = int(gear_info[-1- 13])
                        self.gear_enable = gear_enable

                        # print('gear enable loop: ', gear_enable, gear_info)
                        # print('\n\n')
                        pass



                # print("time:%9d id:%9d flag:0x%02x dlc:%d data:%s" %
                    # (time, msgId, flg, dlc, data))
            except (canlib.canNoMsg) as ex:
                time.sleep(0.001)
                pass
            except (canlib.canError) as ex:
                print(ex)
        # file.close()
        print("Stopped receiving messages")


    def publish(self):
        msg_seq = 0
        rate = rospy.Rate(10)
        while not self.stop_pub.is_set():
            ros_msg = Float64MultiArray()

            ros_msg.data.append(self.recv_steer_wheel_time)
            ros_msg.data.append(self.rotation_time)
            ros_msg.data.append(self.delta_gear_time)
            ros_msg.data.append(self.recv_steer_wheel)
            ros_msg.data.append(self.rotation)
            ros_msg.data.append(self.delta_gear)

            # ros_msg.header = Header()
            # ros_msg.header.seq = msg_seq
            # ros_msg.header.stamp = rospy.Time.from_sec(time.time())


            self.ros_pub.publish(ros_msg)

            rate.sleep()
            pass





    def send(self, msg, msg_id=None):
        if msg_id == None:
            msg_id = self.msgId
        self.send_channel.write_raw(self.msgId, msg, self.flg)
        # print('send: ', msg)
        return
        
        




    def clear_cmd(self):
        for i in range(8):
            self.cmd[i] = 0x00


    def set_read_angle(self):
        self.clear_cmd()
        self.cmd[0] = 0x0a
        self.cmd[1] = 0x01
        self.send_channel.write_raw(self.msgId, self.cmd, self.flg)



    # def request_gear_enable(self):
    #     msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    #     msg[0] = 0x0b
    #     msg[1] = 0x03
    #     self.send(msg)



    def request_max_gear(self):
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x0b
        msg[1] = 0x85
        self.send(msg)


    # def request_enable(self):
    #     msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    #     msg[0] = 0x0b



    def set_query_mode(self):
        # cycle 0: 1s, 1: 100ms, 2:200ms, 3: 500ms
        self.clear_cmd()
        self.cmd[0] = 0x03
        self.cmd[1] = 0x01#0x00 if once else 0x01
        self.cmd[2] = 0x01#cycle & 0xff
        self.send_channel.write_raw(self.msgId, self.cmd, self.flg)




    def activate_steer(self):
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x00
        msg[1] = 0x01
        print(rldev.prefix(self) + f'{msg}')
        print(rldev.prefix(self) + 'activating steer')
        while not self.steer_enable:
            self.send(msg)
            time.sleep(0.1)
        print(rldev.prefix(self) + 'activated steer')

    def deactivate_steer(self):
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x00
        msg[1] = 0x00
        print(rldev.prefix(self) + f'{msg}')
        print(rldev.prefix(self) + 'deactivating steer')
        while self.steer_enable:
            self.send(msg)
            time.sleep(0.1)
        print(rldev.prefix(self) + 'deactivated steer')


    def activate_gear(self):
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x09
        msg[1] = 0x01
        print(rldev.prefix(self) + 'activating gear')
        while not self.gear_enable:
            self.send(msg)
            time.sleep(0.1)
        print(rldev.prefix(self) + 'activated gear')
        return

    def deactivate_gear(self):
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x09
        msg[1] = 0x00
        print(rldev.prefix(self) + 'deactivating gear')
        while self.gear_enable:
            self.send(msg)
            time.sleep(0.1)
        print(rldev.prefix(self) + 'deactivated gear')
        return

    def stop_gear(self):
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x13
        msg[1] = 0x01
        self.send(msg)
        print(rldev.prefix(self) + 'stopped gear')



    def set_gear(self, gear):
        """
            gear: [0, 1]
        """
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x05
        if gear >= 0:
            msg[1] = 0x00
        else:
            msg[1] = 0x01
            gear *= -1
        gear = int(gear * self.max_gear)
        msg[2] = (gear >> 24) & 0xff#0x03
        msg[3] = (gear >> 16) & 0xff#0x03
        msg[4] = (gear >> 8) & 0xff#0x00
        msg[5] = gear & 0xff#0x00
        self.send(msg)



    def set_delta_gear(self, gear):
        """
            gear: [-1, 1]
        """
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x07
        if gear >= 0:
            msg[1] = 0x00
        else:
            msg[1] = 0x01
            gear *= -1
        gear = int(gear * self.max_gear)
        msg[2] = (gear >> 24) & 0xff#0x03
        msg[3] = (gear >> 16) & 0xff#0x03
        msg[4] = (gear >> 8) & 0xff#0x00
        msg[5] = gear & 0xff#0x00
        print(rldev.prefix(self) + str(msg))
        self.delta_gear_time = time.time()
        self.delta_gear = gear
        self.send(msg)





    def set_rotation(self, rotation_in):
        """
            steering wheel position increment
            [-90, 90] deg
        """
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x02
        if rotation_in < 0:
            msg[1] = 0x01 # left
        elif rotation_in > 0:
            msg[1] = 0x02 # right
        else:
            msg[1] = 0x00 # to zero
            
        rotation = abs(int(rotation_in*10)) # 0~900
        msg[2] = (rotation >> 8) & 0xff
        msg[3] = rotation & 0xff
        self.rotation_time = time.time()
        self.rotation = rotation_in
        self.send(msg)



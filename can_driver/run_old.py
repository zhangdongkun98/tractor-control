# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Device: Kvaser leaf light 2xHS
from time import sleep
import time
import threading
import platform
import numpy as np

import canlib.canlib as canlib
from canlib.canlib import ChannelData


class Controller:
    def __init__(self, change_channel=False):
        self.change_channel = change_channel

        self.send_freq = 100 # 5Hz 发送频率
        self.max_speed = 1000000
        self.msgId = 0x200 # 发的ID
        self.msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.cmd = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.flg = canlib.canMSG_EXT

        self.send_channel = self.set_up_channel(channel=0 if self.change_channel else 1)
        # self.recv_channel = self.set_up_channel(channel=1 if self.change_channel else 0)
        self.recv_channel = self.send_channel
        
        self.stop_send = threading.Event()
        self.stop_recv = threading.Event()
        self.send_cyclic = threading.Thread(target=self.send, args=())
        self.recv_cyclic = threading.Thread(target=self.recv, args=())

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

    def tear_down_channel(self, ch):
        ch.busOff()
        ch.close()

    def set_send_id(self, id):
        self.msgId = id

    def set_msg(self, msg):
        self.msg = msg

    def start(self):
        self.stop_send.clear()
        self.stop_recv.clear()
        
        self.send_cyclic.start()
        self.recv_cyclic.start()

    def stop_event(self):
        self.stop_send.set()
        self.stop_recv.set()

    def close(self):
        self.tear_down_channel(self.send_channel)
        self.tear_down_channel(self.recv_channel)

    def send(self):
        return
        self.send_channel
        i = 0
        while not self.stop_send.is_set():
            i += 1
            self.send_channel.write_raw(self.msgId, self.msg, self.flg)
            # print(f"tx: {self.msg}")
            # print('self: ', 'one time')
            sleep(1./self.send_freq) # 10Hz
            # if i > 10:
                # return

        print("Stopped sending messages")

    def recv(self):
        print("Start receiving messages")
        # file=open('msg.txt', 'w+')
        while not self.stop_recv.is_set():
            try:
                (msgId, msg, dlc, flg, time) = self.recv_channel.read()  # deprecated in v1.5
                if msgId == 0x100:
                    if msg[0] == 0x81 and msg[1] == 0x01:
                        # print('fff', msg)
                        # value = msg[4]*0xff+msg[5]
                        # print('value', 255-msg[4], 255-msg[5])
                        if msg[4] > 0xf0:
                            value = - ((255-msg[4])*255+ (255-msg[5]))
                        else:
                            value = (msg[4])*255+ msg[5]
                        print(value)
                        # if value > 0x00ff:
                            # print(-(0xffff-value + 1))
                        # else:
                            # print(value)		
                    # file.write(str(hex(msgId)) + '\t' + data+'\n')
                # print("time:%9d id:%9d flag:0x%02x dlc:%d data:%s" %
                    # (time, msgId, flg, dlc, data))
            except (canlib.canNoMsg) as ex:
                # print("nothing")
                sleep(0.001)
                pass
            except (canlib.canError) as ex:
                print(ex)
        # file.close()
        print("Stopped receiving messages")



    # 0~1
    def set_speed(self, speed):
        speed = int(speed * self.max_speed)
        self.msg[0] = 0x05
        self.msg[1] = 0x00
        self.msg[2] = (speed >> 16) & 0xff#0x03
        self.msg[3] = (speed >> 8) & 0xff#0x00
        self.msg[4] = speed & 0xff#0x00
        self.msg[5] = 0x00
        self.msg[6] = 0x00
        self.msg[7] = 0x00

    # -90. ~ +90.
    def set_rotation(self, rotation):
        self.msg[0] = 0x02
        if rotation < 0:
            self.msg[1] = 0x01 # left
        elif rotation > 0:
            self.msg[1] = 0x02 # right
        else:
            self.msg[1] = 0x00 # to zero
            
        rotation = abs(int(rotation*10)) # 0~900
        self.msg[2] = (rotation >> 8) & 0xff
        self.msg[3] = rotation & 0xff

    def set_read_angle(self):
        self.clear_cmd()
        self.cmd[0] = 0x0a
        self.cmd[1] = 0x01
        self.send_channel.write_raw(self.msgId, self.cmd, self.flg)

    def clear_cmd(self):
        for i in range(8):
            self.cmd[i] = 0x00

    # cycle 0: 1s, 1: 100ms, 2:200ms, 3: 500ms
    def set_query_mode(self, once: bool, cycle: int):
        self.clear_cmd()
        self.cmd[0] = 0x03
        self.cmd[1] = 0x01#0x00 if once else 0x01
        self.cmd[2] = 0x01#cycle & 0xff
        self.send_channel.write_raw(self.msgId, self.cmd, self.flg)

if __name__ == '__main__':
    import math
    ctrl = Controller()
    ctrl.start()

    # ctrl.set_read_angle()
    # sleep(0.1)
    ctrl.set_query_mode(False, 1)
    sleep(0.1)
    cnt = 0
    steps = 10
    # import time
    # start = time.time()

    # ctrl.set_rotation(30)
    ctrl.set_rotation(90)

    while True:
        ctrl.set_rotation(50)
        # import pdb; pdb.set_trace()
        ctrl.send_channel.write_raw(ctrl.msgId, ctrl.msg, ctrl.flg)
        time.sleep(0.1)
        # sleep(1./ctrl.send_freq) # 10Hz
        # ctrl.send_channel.write_raw(ctrl.msgId, ctrl.msg, ctrl.flg)
        # sleep(1./ctrl.send_freq) # 10Hz
        # ctrl.send_channel.write_raw(ctrl.msgId, ctrl.msg, ctrl.flg)
    # sleep(1./ctrl.send_freq) # 10Hz
    # ctrl.send_channel.write_raw(ctrl.msgId, ctrl.msg, ctrl.flg)
    # sleep(1./ctrl.send_freq) # 10Hz
    # ctrl.send_channel.write_raw(ctrl.msgId, ctrl.msg, ctrl.flg)
    # sleep(1./ctrl.send_freq) # 10Hz

    print('//////////////////// hewer')
    exit(0)

    while True:
        cnt += steps

        ctrl.set_rotation(-30)
        # sleep(0.1)
        sleep(0.1)



        # ctrl.set_speed(0.5)
        # sleep(0.5)
        # sleep(5)
        # ctrl.set_rotation(100)
        # sleep(0.1)

        # ctrl.set_speed(1)
        # sleep(15)

        # ctrl.set_rotation(-100)
        # sleep(0.1)

        # ctrl.set_speed(0)
        # sleep(15)

        # ctrl.set_rotation(-2)
        # ctrl.set_speed(1)
        # sleep(1)
        # # ctrl.set_rotation(2)
        # ctrl.set_speed(0)
        # sleep(1)
        # ctrl.set_speed(1)
        theta = (math.pi/180) * cnt
        print('------------------------theta: ', np.rad2deg(theta))
        value = math.sin(theta)
        # ctrl.set_speed(value)
        ctrl.set_rotation(30*value)
        sleep(0.2)
        # ctrl.set_speed(0)
        # sleep(12)
        # ctrl.set_speed(1)
        # sleep(12)

        # if time.time() < 10:
            # ctrl.set_rotation(-2.5)
        # else:
            # ctrl.set_rotation(0.0)

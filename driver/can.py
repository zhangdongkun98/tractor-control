
import time
import threading
import platform
import numpy as np

import canlib.canlib as canlib
from canlib.canlib import ChannelData



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
    def __init__(self, change_channel=False):
        self.change_channel = change_channel

        self.send_freq = 100 # 5Hz 发送频率
        self.max_speed = 1000000
        self.msgId = 0x200 # 发的ID
        # self.msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.cmd = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.flg = canlib.canMSG_EXT

        self.send_channel = self.set_up_channel(channel=0 if self.change_channel else 1)
        # self.send_channel = PseudoChannel()
        self.recv_channel = self.send_channel
        
        self.stop_recv = threading.Event()
        self.recv_cyclic = threading.Thread(target=self.recv, args=())
        self.stop_recv.clear()
        self.recv_cyclic.start()

        ### set_query_mode
        # cycle 0: 1s, 1: 100ms, 2:200ms, 3: 500ms
        self.clear_cmd()
        self.cmd[0] = 0x03
        self.cmd[1] = 0x01#0x00 if once else 0x01
        self.cmd[2] = 0x01#cycle & 0xff
        self.send_channel.write_raw(self.msgId, self.cmd, self.flg)

        time.sleep(0.1)
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
        self.stop_recv.set()

    def close(self):
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
                import time
                time.sleep(0.001)
                pass
            except (canlib.canError) as ex:
                print(ex)
        # file.close()
        print("Stopped receiving messages")



    def send(self, msg, msg_id=None):
        if msg_id == None:
            msg_id = self.msgId
        self.send_channel.write_raw(self.msgId, msg, self.flg)
        print('send: ', msg)
        return
        
        




    def clear_cmd(self):
        for i in range(8):
            self.cmd[i] = 0x00


    def set_read_angle(self):
        self.clear_cmd()
        self.cmd[0] = 0x0a
        self.cmd[1] = 0x01
        self.send_channel.write_raw(self.msgId, self.cmd, self.flg)





    # 0~1
    def set_speed(self, speed):  ## ! todo
        speed = int(speed * self.max_speed)
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x05
        msg[1] = 0x00
        msg[2] = (speed >> 16) & 0xff#0x03
        msg[3] = (speed >> 8) & 0xff#0x00
        msg[4] = speed & 0xff#0x00
        msg[5] = 0x00
        msg[6] = 0x00
        msg[7] = 0x00
        self.send(msg)

    def set_rotation(self, rotation):
        """
            steering wheel position increment
            [-90, 90] deg
        """
        msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        msg[0] = 0x02
        if rotation < 0:
            msg[1] = 0x01 # left
        elif rotation > 0:
            msg[1] = 0x02 # right
        else:
            msg[1] = 0x00 # to zero
            
        rotation = abs(int(rotation*10)) # 0~900
        msg[2] = (rotation >> 8) & 0xff
        msg[3] = rotation & 0xff
        self.send(msg)



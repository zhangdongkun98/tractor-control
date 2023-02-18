

import time
import canlib.canlib as canlib
from vehicle_basic import VehicleState, msg_translate, dec2bin, hex2bin,\
    AUTO_MODE_MSG, WARM_START_MSG, D_GEAR_MSG, ROLL_STEER_MSG, STOP_MSG, SEND_MSG_ID

class CANManager(object):
    def __init__(self, rate = None):
        self.rate = rate
        self.vehicle_state = VehicleState()
        self.send_channel = self.set_up_channel(channel=0)
        # self.read_channel = self.set_up_channel(channel=1)
        self.read_channel = self.send_channel

    def set_up_channel(self, channel=0,
                     openFlags=canlib.canOPEN_ACCEPT_VIRTUAL,
                     bitrate=canlib.canBITRATE_500K,
                     bitrateFlags=canlib.canDRIVER_NORMAL):
        ch = canlib.openChannel(channel, openFlags)
        print("Using channel: %s, EAN: %s" % (ch.getChannelData_Name(),
                                              ch.getChannelData_EAN()))
        ch.setBusOutputControl(bitrateFlags)
        ch.setBusParams(bitrate)
        ch.busOn()
        return ch

    def tear_down_channel(self, ch):
        ch.busOff()
        ch.close()

    def write_msg(self, msg, msg_id=SEND_MSG_ID, flg=canlib.canMSG_STD):
        self.send_channel.write(msg_id, msg, flg)

    def change_auto_mode(self):
        i = 0
        while (i < 200):
            modeNow = self.read_mode_data()
            if modeNow:
                print("in auto mode")
                return True
            self.write_msg(AUTO_MODE_MSG)
            self.sleep()
            i += 1

        return False

    def change_d_gear(self):
        i = 0
        while (i < 1000):
            GearNow = self.read_gear_data()
            if GearNow == 2:
                print("in D Gear")
                return True
            self.write_msg(D_GEAR_MSG)
            self.sleep()
            i += 1
        return False

    def startup_warm(self):
        i = 0
        while(i < 100):
            self.write_msg(WARM_START_MSG)
            self.sleep()
            i += 1
        print("Warm up done!")
        return

    def roll_steer(self):
        i = 0
        while (i < 1000):
            self.write_msg(ROLL_STEER_MSG)
            self.sleep()
        print("Warm up done!")
        return

    def emergency_stop(self):
        self.write_msg(STOP_MSG, SEND_MSG_ID, canlib.canMSG_EXT)

    def vehicle_control(self, gasSet=0, steerSet=0, steerSpd=0, brakeSet=0):
        """
        real control interface
        :param gasSet: range 0~100 %
        :param steerSet: range -2048~2047 deg , actual range -540~540 deg
        :param steerSpd:  range 0~1016 deg/s
        :param brakeSet: range 0~25.5 Mpa
        :return:
        """
        msg = [0x00, 0x0D, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]

        steerSetBin = dec2bin(str(int((steerSet + 2048) * 16)))
        while len(steerSetBin) < 16:
            steerSetBin = "0" + steerSetBin

        msg[0] = int(float(brakeSet) * 10)
        msg[3] = int(float(gasSet) / 0.392)
        highSteerSetBin = steerSetBin[0:8]
        lowSteerSetBin = steerSetBin[8:]
        msg[4] = int(highSteerSetBin, 2)
        msg[5] = int(lowSteerSetBin, 2)
        msg[6] = int(float(steerSpd) / 4)

        self.write_msg(msg)
        # self.sleep()
        return

    def sleep(self):
        if self.rate is None:
            time.sleep(0.01)
        else:
            self.rate.sleep()

    def read_mode_data(self):
        """
        :return:
        auto: True
        manual: False
        """
        while True:
            try:
                (msgId, msg, dlc, flg, time) = self.read_channel.read()
                if msgId == 0x400:
                    bindata = msg_translate(msg)

                    if bindata[40] == '1':
                        print("auto is on")
                        return True
                    else:
                        return False

                else:
                    pass
            except:
                #print("no msg!")
                return False

    def read_gear_data(self):
        """
        :return:
        R Gear: 0
        N Gear: 1
        D Gear: 2
        """
        while True:
            try:
                (msgId, msg, dlc, flg, _time) = self.read_channel.read()
                if msgId == 0x403:
                    bindata = msg_translate(msg)
                    gearbin = bindata[3:-1]
                    gearint = int(gearbin, 2)
                    if gearint == 0x0B:
                        print("D Gear is in")
                        return 2
                    elif gearint == 0x0D:
                        print("R Gear is in")
                        return 0
                    elif gearint == 0x0C:
                        print("N Gear is in")
                        return 1
                    else:
                        return -1
                else:
                    pass
            except:
                #print("no msg!")
                return -1

    def read_state(self):
        try:
            (msgId, msg, dlc, flg, _time) = self.read_channel.read()

            data = ''.join(format(x, '02x') for x in msg)
            if msgId == 0x394:
                reverse_data = ""
                for index in range(len(data)):
                    if index % 2 == 1:
                        reverse_data += data[index]
                        reverse_data += data[index - 1]
                data = reverse_data
                bindata = ""
                for subdata in data:
                    subbindata = hex2bin(subdata)
                    while len(subbindata) < 4:
                        subbindata = "0" + subbindata
                    bindata += subbindata[::-1]

                steer_data = bindata[7::-1] + bindata[15:8:-1]
                steer_int = int(steer_data, 2)
                steer = steer_int * 0.04375

                self.vehicle_state.steer.update(steer, time.time())
                self.vehicle_state.update = True

            if msgId == 0x403:
                reverse_data = ""
                for index in range(len(data)):
                    if index % 2 == 1:
                        reverse_data += data[index]
                        reverse_data += data[index - 1]
                data = reverse_data
                bindata = ""
                for subdata in data:
                    subbindata = hex2bin(subdata)
                    while len(subbindata) < 4:
                        subbindata = "0" + subbindata
                    bindata += subbindata[::-1]

                speed_data = bindata[10:7:-1] + bindata[23:15:-1] + bindata[31:29:-1]
                gas_data = bindata[55:47:-1]
                speed_int = int(speed_data, 2)
                gas_int = int(gas_data, 2)

                speed = speed_int * 0.0625
                gas = gas_int # TODO

                now = time.time()
                self.vehicle_state.speed.update(speed, now)
                self.vehicle_state.gas.update(gas, now)
                self.vehicle_state.update = True

        except (canlib.canNoMsg) as ex:
            pass
        except (canlib.canError) as ex:
            print(ex)
import canlib.canlib as canlib
import rospy
from std_msgs.msg import String


class MsgManager(object):
    def __init__(self):
        rospy.init_node('chery', anonymous=True)
        self.rate = rospy.Rate(100)
        self.pub = rospy.Publisher('/CheryState', String, queue_size=1)

        self.base = [str(x) for x in range(10)] + [chr(x) for x in range(ord('A'), ord('A') + 6)]
        self.cl = canlib.canlib()
        print("canlib version: %s" % self.cl.getVersion())
        print("CANlib is start! BE CAREFUL!!!\n")

        self.ch0 = self.setUpChannel(channel=0)

    def setUpChannel(self, channel=0,
                     openFlags=canlib.canOPEN_ACCEPT_VIRTUAL,
                     bitrate=canlib.canBITRATE_500K,
                     bitrateFlags=canlib.canDRIVER_NORMAL):
        cl = canlib.canlib()
        ch = cl.openChannel(channel, openFlags)
        print("Using channel: %s, EAN: %s" % (ch.getChannelData_Name(),
                                              ch.getChannelData_EAN()))
        ch.setBusOutputControl(bitrateFlags)
        ch.setBusParams(bitrate)
        ch.busOn()
        return ch

    def tearDownChannel(self, ch):
        ch.busOff()
        ch.close()

    def loadAutoMode(self):
        msg = [0x00, 0x09, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]
        return msg

    def changeAutoMode(self):
        i = 0
        while (i < 200):
            # print("starting automode")
            modeNow = self.readModeData()
            if modeNow:
                print("in auto mode")
                return True
            msg = self.loadAutoMode()
            msgId = 0x20
            flg = canlib.canMSG_STD
            self.publish(msgId, msg, flg)
            self.spaceTick()
            i += 1

        return False

    def changeDireGear(self):
        i = 0
        while (i < 1000):
            GearNow = self.readGearData()
            if GearNow == 2:
                print("in D Gear")
                return True
            msg = [0x00, 0x0D, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]
            msgId = 0x20
            flg = canlib.canMSG_STD
            #print(msg)
            self.publish(msgId, msg, flg)
            self.spaceTick()
            i += 1

        return False

    def startupWarm(self):
        i = 0
        while(i < 100):
            msg = [0x00, 0x0c, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]
            msgId = 0x20
            flg = canlib.canMSG_STD
            #print(msg)
            self.publish(msgId, msg, flg)
            self.spaceTick()
            i += 1

        print("Warm up done!")
        return

    def rollSteer(self):
        i = 0
        while (i < 1000):
            msg = [0x00, 0x0D, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00]
            msgId = 0x20
            flg = canlib.canMSG_STD
            #print(msg)
            self.publish(msgId, msg, flg)
            self.spaceTick()

        print("Warm up done!")
        return

    def loadControl(self, gasSet=0, steerSet=0, steerSpd=0, brakeSet=0):
        """
        real control interface
        :param gasSet: range 0~100 %,input int style
        :param steerSet: range -2048~2047 deg , actual range -540~540 deg, input int style
        :param steerSpd:  range 0~1016 deg/s, input int style
        :param brakeSet: range 0~25.5 Mpa, input int style
        :return:
        """
        basemsg = [0x00, 0x0D, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]

        # gasSetBin = self.dec2bin(str(int(float(gasSet) / 0.392)))
        steerSetBin = self.dec2bin(str(int((steerSet + 2048) * 16)))
        # steerSpdBin = self.dec2bin(str(int(float(steerSpd) / 4)))
        # brakeSetBin = self.dec2bin(str(brakeSet))

        # while len(gasSetBin) < 8:
        #     gasSetBin = "0" + gasSetBin

        while len(steerSetBin) < 16:
            steerSetBin = "0" + steerSetBin

        # while len(steerSpdBin) < 8:
        #     steerSpdBin = "0" + steerSpdBin

        # while len(brakeSetBin) < 8:
        #     brakeSetBin = "0" + brakeSetBin

        basemsg[0] = int(float(brakeSet) * 10)
        basemsg[3] = int(float(gasSet) / 0.392)
        highSteerSetBin = steerSetBin[0:8]
        lowSteerSetBin = steerSetBin[8:]
        basemsg[4] = int(highSteerSetBin, 2)
        basemsg[5] = int(lowSteerSetBin, 2)
        basemsg[6] = int(float(steerSpd) / 4)

        msgId = 0x20
        flg = canlib.canMSG_STD
        self.publish(msgId, basemsg, flg)
        self.spaceTick()
        return

    def spaceTick(self):
        self.rate.sleep()

    def publish(self, msgId, msg, flg):
        self.ch0.write(msgId, msg, flg)

    def emergencyStop(self):
        stopMsg = [0x00 for i in range(8)]
        flg = canlib.canMSG_EXT
        self.publish(0x20, stopMsg, flg)

    def readModeData(self):
        """
        :return:
        auto: True
        manual: False
        """
        while True:
            try:
                (msgId, msg, dlc, flg, time) = self.ch0.read()
                if msgId == 0x400:
                    bindata = self.msg_translate(msg)

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

    def readGearData(self):
        """
        :return:
        R Gear: 0
        N Gear: 1
        D Gear: 2
        """
        while True:
            try:
                (msgId, msg, dlc, flg, time) = self.ch0.read()
                if msgId == 0x403:
                    bindata = self.msg_translate(msg)
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

    def readSpeedData(self):
        for i in range(30000):
            try:
                (msgId, msg, dlc, flg, time) = self.ch0.read()
                if msgId == 0x403:
                    bindata = self.msg_translate(msg)
                    vehicle_data = bindata[10:7:-1] + bindata[23:15:-1] + bindata[31:29:-1]
                    vehicle_int = int(vehicle_data, 2)
                    return vehicle_int * 0.0625
            except:
                pass
                #print("wrong speed data!!!!!!!!")

        return 10

    def msg_translate(self, msg):
        data = ''.join(format(x, '02x') for x in msg)

        reverse_data = ""
        for index in range(len(data)):
            if index % 2 == 1:
                reverse_data += data[index]
                reverse_data += data[index - 1]
        data = reverse_data
        bindata = ""
        for subdata in data:
            subbindata = self.hex2bin(subdata)
            while len(subbindata) < 4:
                subbindata = "0" + subbindata
            bindata += subbindata[::-1]

        return bindata

    def dec2bin(self, string_num):
        num = int(string_num)
        mid = []
        while True:
            if num == 0:
                break
            num, rem = divmod(num, 2)
            mid.append(self.base[rem])

        return ''.join([str(x) for x in mid[::-1]])

    def hex2dec(self, string_num):
        return str(int(string_num.upper(), 16))

    def hex2bin(self, string_num):
        return self.dec2bin(self.hex2dec(string_num.upper()))



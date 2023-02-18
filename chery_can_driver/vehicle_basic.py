
SEND_MSG_ID = 0x20
AUTO_MODE_MSG = [0x00, 0x09, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]
WARM_START_MSG = [0x00, 0x0c, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]
D_GEAR_MSG = [0x00, 0x0D, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]
ROLL_STEER_MSG = [0x00, 0x0D, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00]
STOP_MSG = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
BASE_MSG = [0x00, 0x0D, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]

BASE = [str(x) for x in range(10)] + [chr(x) for x in range(ord('A'), ord('A') + 6)]

class VehicleCtrl(object):
    def __init__(self, gas=0, steer=0, steer_speed=0, brake=0):
        """
        :param gas: range 0~100 %
        :param steer: range -540~540 deg
        :param steer_speed:  range 0~1016 deg/s
        :param brake: range 0~25.5 Mpa
        """

        self.gas = gas
        self.steer = steer
        self.steer_speed = steer_speed
        self.brake = brake
    
    def clip(self, value, min_value, max_value):
        return min(max_value, max(min_value, value))

    def get_value(self):
        gas = self.clip(self.gas, 0, 100)
        steer = self.clip(self.steer, -540, 540)
        steer_speed = self.clip(self.steer_speed, 0, 1016)
        brake = self.clip(self.gas, 0, 25.5)
        return gas, steer, steer_speed, brake

class ValueWithTime(object):
    def __init__(self, value=None, ts=None):
        self.value = value
        self.ts = ts
    
    def update(self, value, ts):
        self.value = value
        self.ts = ts
    
    def get_value(self):
        if self.value is None:
            value = -9999
            ts = -9999
        else:
            value = self.value
            ts = self.ts
        return value, ts

class VehicleState(object):
    def __init__(self):
        self.speed = ValueWithTime()
        self.steer = ValueWithTime()
        self.gas = ValueWithTime()
        # check if update
        self.update = False

    def get_value(self):
        speed, speed_ts = self.speed.get_value()
        steer, steer_ts = self.steer.get_value()
        gas, gas_ts = self.gas.get_value()
        return [speed, speed_ts, steer, steer_ts, gas, gas_ts]

    def clear(self):
        self.speed = ValueWithTime()
        self.steer = ValueWithTime()
        self.gas = ValueWithTime()
        self.update = False


def dec2bin(string_num):
    num = int(string_num)
    mid = []
    while True:
        if num == 0:
            break
        num, rem = divmod(num, 2)
        mid.append(BASE[rem])

    return ''.join([str(x) for x in mid[::-1]])

def hex2dec(string_num):
    return str(int(string_num.upper(), 16))

def hex2bin(string_num):
    return dec2bin(hex2dec(string_num.upper()))

def msg_translate(msg):
    data = ''.join(format(x, '02x') for x in msg)

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

    return bindata
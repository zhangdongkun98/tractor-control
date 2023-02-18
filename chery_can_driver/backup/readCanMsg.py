import canlib.canlib as canlib

base = [str(x) for x in range(10)] + [chr(x) for x in range(ord('A'), ord('A') + 6)]


def setUpChannel(channel=0,
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


def tearDownChannel(ch):
    ch.busOff()
    ch.close()


cl = canlib.canlib()
print("canlib version: %s" % cl.getVersion())

channel_0 = 0
channel_1 = 1
ch0 = setUpChannel(channel=0)


# ch1 = setUpChannel(channel=1)

def dec2bin(string_num):
    num = int(string_num)
    mid = []
    while True:
        if num == 0: break
        num, rem = divmod(num, 2)
        mid.append(base[rem])

    return ''.join([str(x) for x in mid[::-1]])


def hex2dec(string_num):
    return str(int(string_num.upper(), 16))


def hex2bin(string_num):
    return dec2bin(hex2dec(string_num.upper()))


f = open("vehicle.txt", 'w')
while True:
    try:
        (msgId, msg, dlc, flg, time) = ch0.read()

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
            # print("steer_data", steer_data, float(steer_int)*0.04375)

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

            vehicle_data = bindata[10:7:-1] + bindata[23:15:-1] + bindata[31:29:-1]
            gas_data = bindata[55:47:-1]
            vehicle_int = int(vehicle_data, 2)
            gas_int = int(gas_data, 2)
            print("vehicle_speed", vehicle_int * 0.0625)
            f.write(str(steer_int * 0.04375))
            f.write("\n")
            f.write(str(vehicle_int * 0.0625))
            f.write("\n")

    except (canlib.canNoMsg) as ex:
        None
    except (canlib.canError) as ex:
        print(ex)

tearDownChannel(ch0)
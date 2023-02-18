import canlib.canlib as canlib


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

def get_msg():
    msg = [1, 2, 3, 4]
    msgId = 100
    return msg, msgId

cl = canlib.canlib()
print("canlib version: %s" % cl.getVersion())

channel_0 = 0
channel_1 = 1
ch0 = setUpChannel(channel=0)

msgId = 100
msg = [1, 2, 3, 4]
flg = canlib.canMSG_EXT
ch0.write(msgId, msg, flg)

while True:
    msg, msgId = get_msg()
    flg = canlib.canMSG_EXT
    ch0.write(msgId, msg, flg)

tearDownChannel(ch0)

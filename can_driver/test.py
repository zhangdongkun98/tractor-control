import canlib.canlib as canlib
from canlib.canlib import ChannelData

print(dir(canlib))


def setUpChannel(channel=0,
                openFlags=canlib.canOPEN_ACCEPT_VIRTUAL,
                bitrate=canlib.canBITRATE_500K,
                bitrateFlags=canlib.canDRIVER_NORMAL):
    ch = canlib.openChannel(channel, openFlags)
    print(
        "Using channel: %s, EAN: %s" %
        (ChannelData(channel).channel_name, ChannelData(channel).card_upc_no))
    ch.setBusOutputControl(bitrateFlags)
    ch.setBusParams(bitrate)
    ch.busOn()
    return ch


def tearDownChannel(ch):
    ch.busOff()
    ch.close()


ch0 = setUpChannel(channel=0)
ch1 = setUpChannel(channel=1)

msgId = 100
msg = [1, 2, 3, 4]
flg = canlib.canMSG_EXT

ch1.write_raw(msgId, msg, flg)

while True:
    try:
        (msgId, msg, dlc, flg, time) = ch0.read()  # deprecated in v1.5
        data = ''.join(format(x, '02x') for x in msg)
        print("time:%9d id:%9d flag:0x%02x dlc:%d data:%s" %
              (time, msgId, flg, dlc, data))
        break
    except (canlib.canNoMsg) as ex:
        pass
    except (canlib.canError) as ex:
        print(ex)

tearDownChannel(ch0)
tearDownChannel(ch1)
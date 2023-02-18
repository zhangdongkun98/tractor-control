# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Device: Kvaser leaf light 2xHS
from time import sleep
import threading
import platform

import canlib.canlib as canlib
from canlib.canlib import ChannelData


class Controller:
	def __init__(self, change_channel=False):
		self.change_channel = change_channel

		self.send_freq = 50

		self.msgId = 100
		self.msg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		self.flg = canlib.canMSG_EXT

		self.send_channel = self.set_up_channel(channel=0)
		# self.send_channel = self.set_up_channel(channel=0 if self.change_channel else 1)
		# self.recv_channel = self.set_up_channel(channel=1 if self.change_channel else 0)

		self.stop_send = threading.Event()
		# self.stop_recv = threading.Event()
		self.send_cyclic = threading.Thread(target=self.send, args=())
		# self.recv_cyclic = threading.Thread(target=self.recv, args=())

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
		# self.stop_recv.clear()
		
		self.send_cyclic.start()
		# self.recv_cyclic.start()

	def stop_event(self):
		self.stop_send.set()
		# self.stop_recv.set()

	def close(self):
		self.tear_down_channel(self.send_channel)
		# self.tear_down_channel(self.recv_channel)

	def send(self):
		self.send_channel
		while not self.stop_send.is_set():
			self.send_channel.write_raw(self.msgId, self.msg, self.flg)
			print(f"send tx: {self.msg}")
			sleep(1./self.send_freq) # 10Hz

		print("Stopped sending messages")

	def recv(self):
		print("Start receiving messages")
		while not self.stop_recv.is_set():
			try:
				(msgId, msg, dlc, flg, time) = self.recv_channel.read()  # deprecated in v1.5
				data = ''.join(format(x, '02x') for x in msg)
				print("time:%9d id:%9d flag:0x%02x dlc:%d data:%s" %
					(time, msgId, flg, dlc, data))
			except (canlib.canNoMsg) as ex:
				# print("nothing")
				sleep(0.001)
				pass
			except (canlib.canError) as ex:
				print(ex)
		print("Stopped receiving messages")

	# 0~10
	def set_speed(self, speed: int):
		self.msg[0] = 0x05
		self.msg[1] = speed & 0xff

	    # -90. ~ +90.
	def set_rotation(self, rotation):
		self.msg[0] = 0x01
		if rotation < 0:
			self.msg[1] = 0x01  # left
		elif rotation > 0:
			self.msg[1] = 0x02  # right
		else:
			self.msg[1] = 0x00  # to zero

		rotation = abs(int(rotation * 10))  # 0~900
		self.msg[2] = (rotation >> 8) & 0xff
		self.msg[3] = rotation & 0xff

	# cycle 0: 1s, 1: 100ms, 2:200ms, 3: 500ms
	def set_query_mode(self, once: bool, cycle: int):
		self.msg[0] = 0x02
		self.msg[1] = 0x00 if once else 0x01
		self.msg[2] = cycle & 0xff

if __name__ == '__main__':
	ctrl = Controller(change_channel=False)
	# ctrl.start()
	ctrl.set_speed(10)
	# ctrl.set_rotation(45)
	ctrl.start()
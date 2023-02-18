# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Device: Kvaser leaf light 2xHS
from time import sleep
import threading
import platform

import canlib.canlib as canlib
from canlib.canlib import ChannelData


class Controller:
	def __init__(self, change_channel=True):
		self.change_channel = change_channel

		self.send_freq = 50

		self.msgId = 0x203
		self.msg = [0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		self.flg = canlib.canMSG_EXT

		self.max_speed = 1000
		self.max_rotation = 500
		self.acc_time = 10
		self.raw_rotation = 0
		self.has_reversed = False

		self.send_channel = self.set_up_channel(channel=0 if self.change_channel else 1)
		self.recv_channel = self.set_up_channel(channel=1 if self.change_channel else 0)

		self.stop_send = threading.Event()
		self.stop_recv = threading.Event()
		self.send_cyclic = threading.Thread(target=self.send, args=())
		self.recv_cyclic = threading.Thread(target=self.recv, args=())

	def set_up_channel(self,  channel=0,
					openFlags=canlib.canOPEN_ACCEPT_VIRTUAL,
					bitrate=canlib.canBITRATE_500K,
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
		self.send_channel
		while not self.stop_send.is_set():
			self.send_channel.write_raw(self.msgId, self.msg, self.flg)
			print(f"tx: {self.msg}")
			sleep(1./self.send_freq)

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

    # input (0, 1)
	def set_speed(self, speed):
		speed = min(1.0, max(speed, 0.0)) 
		speed = int(self.max_speed * speed)
		self.msg[0] = speed & 0xff
		self.msg[0] = (speed & 0xff00) >> 8

	    # acc 0.2~25.5s
	def set_acc_time(self, acc_time):
		acc_time = min(30, max(5, acc_time))
		self.acc_time = acc_time
		acc_time = int(acc_time)
		self.msg[3] = acc_time & 0xff


	def set_max_speed(self, max_speed):
		max_speed = min(2700, max(0, max_speed))
		self.max_speed = max_speed

	def set_max_rotation(self, max_rotation):
		self.max_rotation = min(500, max(0, max_rotation))

	def set_forward(self):
		if self.msg[0] == 0x05:
			self.has_reversed = True
		self.msg[0] = 0x03

	def set_backward(self):
		if self.msg[0] == 0x03:
			self.has_reversed = True
		self.msg[0] = 0x05

	def set_stop(self):
		self.msg[0] = 0x09
		self.has_reversed = False


    # input (-1, 1)
	def set_rotation(self, rotation):
		rotation = max(-1.0, min(rotation, 1.0))
		rotation = int(self.max_rotation * rotation)
		self.raw_rotation = rotation
		symbol = 0
		if(rotation < 0):
			symbol = 1
			rotation = -rotation
			rotation = (rotation ^ 0xffff) + 1
		self.msg[5] = rotation & 0xff
		self.msg[6] = ((rotation & 0xff00) >> 8) | (symbol << 7)

if __name__ == '__main__':
	ctrl = Controller()
	ctrl.start()
	while True:
		ctrl.set_rotation(0.5)
		sleep(3)
		ctrl.set_rotation(-0.5)
		sleep(3)
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import threading
import platform
import struct
import serial
import serial.tools.list_ports
from queue import Queue
from can import BusABC, Message

BAUD_RATE = 115200
SEND_ID = 0x02000000
RECEIVE_ID_LOW = 0x183
RECEIVE_ID_HIGH = 0x283
RECEIVE_ID_ROTATION = 0x279


def scan_usb(device_type='CAN'):
    """
    input: 'CAN'
    output: port name
    """
    port_list = list(serial.tools.list_ports.comports())
    if len(port_list) == 0:
        print('Cannot find any serial port !')
    else:
        for port in port_list:
            name = port.device
            hwid = port.hwid
            vid = port.vid
            if device_type == 'CAN' and vid == 6790:
                print('Found CAN Bus', name)
                return name
    print('No serial port matches !', [device.name for device in port_list])
    return None


CHANNEL = scan_usb('CAN')


class SerialBus(BusABC):
    def __init__(self,
                 channel,
                 baudrate=BAUD_RATE,
                 timeout=1,
                 rtscts=False,
                 *args,
                 **kwargs):
        """ Baud rate of the serial device in bit/s (default BAUD_RATE). """
        if not channel:
            raise ValueError("Must specify a serial port.")

        self.channel_info = "Serial interface: " + channel
        self.ser = serial.serial_for_url(channel,
                                         baudrate=baudrate,
                                         timeout=timeout,
                                         rtscts=rtscts)
        super().__init__(channel=channel, *args, **kwargs)

    def shutdown(self):
        self.ser.close()

    def send(self, msg):
        byte_msg = bytearray()
        byte_msg.append(0xaa)
        byte_msg.append(0xc8)
        byte_msg.append(msg.arbitration_id & 0x00ff)  # arbitration_id low
        byte_msg.append(
            (msg.arbitration_id >> 8) & 0x00ff)  # arbitration_id high
        byte_msg.append(
            (msg.arbitration_id >> 16) & 0x00ff)  # arbitration_id high
        byte_msg.append(
            (msg.arbitration_id >> 24) & 0x00ff)  # arbitration_id high
        for i in range(msg.dlc):
            byte_msg.append(msg.data[i])
            # print(hex(msg.data[i]))
        # print("\n")
        byte_msg.append(0x55)
        self.ser.write(byte_msg)

    def recv(self):
        try:
            # ser.read can return an empty string
            # or raise a SerialException
            rx_bytes = self.ser.read(13)
        except serial.SerialException:
            return None

        if rx_bytes and struct.unpack_from("B", rx_bytes, offset=0)[0] == 0xaa:
            # print(hex(struct.unpack_from("B",rx_bytes, offset = i)[0]))
            # print("\n")
            config = struct.unpack_from("B", rx_bytes, offset=1)[0]
            if config & 0x10:
                print("Oh shit!!! This is remote frame!!!")
            if config & 0x20:
                arb_id_length = 4
            else:
                arb_id_length = 2

            arb_id = 0
            for i in range(arb_id_length):
                arb_id += struct.unpack_from("B", rx_bytes,
                                             offset=i + 2)[0] << (i * 8)
            dlc = config & 0x0f
            data = [
                struct.unpack_from("B", rx_bytes,
                                   offset=i + 2 + arb_id_length)[0]
                for i in range(dlc)
            ]
            rxd_byte = struct.unpack_from("B",
                                          rx_bytes,
                                          offset=2 + arb_id_length + dlc)[0]
            if rxd_byte == 0x55:
                # received message data okay
                msg = Message(arbitration_id=arb_id,
                              dlc=dlc,
                              data=data,
                              is_extended_id=True)
                return msg

        else:
            return None


class Controller:
    def __init__(self, channel=CHANNEL, baudrate=BAUD_RATE, send_id=SEND_ID):
        self.bus = SerialBus(channel=channel, baudrate=baudrate)
        self.send_id = send_id
        self.has_reversed = False
        self.stop_send = threading.Event()
        self.stop_receive = threading.Event()
        self.send_cyclic = threading.Thread(target=self.send, args=())
        self.receive = threading.Thread(target=self.receive, args=())
        self.cmd_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.start_time = time.time()

    def start(self):
        self.stop_send.clear()
        self.stop_receive.clear()
        #self.send_cyclic.start()
        self.receive.start()

    def stop_event(self):
        self.stop_send.set()
        self.stop_receive.set()
        self.bus.shutdown()

    # 0~10
    def set_speed(self, speed: int):
        self.cmd_data[0] = 0x05
        self.cmd_data[1] = speed & 0xff
        msg = Message(arbitration_id=self.send_id,
                      data=self.cmd_data,
                      is_extended_id=True)
        msg.timestamp = time.time() - self.start_time
        self.bus.send(msg)
        print(f"tx: {msg}")

    # -90. ~ +90.
    def set_rotation(self, rotation):
        self.cmd_data[0] = 0x01
        if rotation < 0:
            self.cmd_data[1] = 0x01  # left
        elif rotation > 0:
            self.cmd_data[1] = 0x02  # right
        else:
            self.cmd_data[1] = 0x00  # to zero

        rotation = abs(int(rotation * 10))  # 0~900
        self.cmd_data[2] = (rotation >> 8) & 0xff
        self.cmd_data[3] = rotation & 0xff
        msg = Message(arbitration_id=self.send_id,
                      data=self.cmd_data,
                      is_extended_id=True)
        msg.timestamp = time.time() - self.start_time
        self.bus.send(msg)
        print(f"tx: {msg}")

    # cycle 0: 1s, 1: 100ms, 2:200ms, 3: 500ms
    def set_query_mode(self, once: bool, cycle: int):
        self.cmd_data[0] = 0x02
        self.cmd_data[1] = 0x00 if once else 0x01
        self.cmd_data[2] = cycle & 0xff
        msg = Message(arbitration_id=self.send_id,
                      data=self.cmd_data,
                      is_extended_id=True)
        msg.timestamp = time.time() - self.start_time
        self.bus.send(msg)
        print(f"tx: {msg}")

    def query_rotation_once(self):
        self.cmd_data[0] = 0x0a
        msg = Message(arbitration_id=self.send_id,
                      data=self.cmd_data,
                      is_extended_id=True)
        msg.timestamp = time.time() - self.start_time
        self.bus.send(msg)

    def send(self):
        """The loop for sending."""
        print("Start to send messages")
        start_time = time.time()
        while not self.stop_send.is_set():
            msg = Message(arbitration_id=self.send_id,
                          data=self.cmd_data,
                          is_extended_id=True)
            msg.timestamp = time.time() - start_time
            self.bus.send(msg)
            print(f"tx: {msg}")
            time.sleep(0.001)
        print("Stopped sending messages")

    def receive(self):
        print("Start receiving messages")
        while not self.stop_receive.is_set():
            rx_msg = self.bus.recv()
            if rx_msg is not None:
                if rx_msg[0] == 0x80:
                    print('0x80: ', rx_msg[1])
                elif rx_msg[0] == 0x81:
                    print('0x81: ', rx_msg[1], 'is 01 ?')
                    print(rx_msg[2], rx_msg[3])
        print("Stopped receiving messages")


if __name__ == "__main__":
    ctrl = Controller()
    ctrl.start()
    ctrl.set_speed(5)
    ctrl.set_rotation(-61.2)
    time.sleep(5)
    ctrl.stop_event()
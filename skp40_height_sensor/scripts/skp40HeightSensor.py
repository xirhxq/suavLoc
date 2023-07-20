#! /usr/bin/env python3

import threading
from os import system, path, readlink
from struct import pack, unpack
from sys import exit
from time import time, sleep
from collections import deque

import rospy
import serial
from geometry_msgs.msg import Vector3Stamped

HZ = 50

DOWN_FRAME_HEAD_1 = b'\x55'
DOWN_FRAME_HEAD_2 = b'\x07'
DOWN_FRAME_HEAD = DOWN_FRAME_HEAD_1 + DOWN_FRAME_HEAD_2

DOWN_FRAME_TAIL = b'\xAA'
FRAME_LEN = 6

bAny, bS8, bU8, bS16, bU16, bS32, bU32, bF = 'x', 'b', 'B', 'h', 'H', 'i', 'I', 'f'
DOWN_PROTO = '<' + bU8 * 6

WAITING_DOWN_FRAME_HEAD_1 = 1
WAITING_DOWN_FRAME_HEAD_2 = 2
READING_DATA = 3

from glob import glob

PORT = glob('/dev/ttyUSB[0-9]*')[0]
PORT = path.join('/dev', readlink(glob('/dev/serial/by-path/*usb-0:2.2:1.0*')[0]).split('/')[-1])

from signal import signal, SIGINT


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)


signal(SIGINT, signal_handler)


def timer(tol=1):
    def decorator(func):
        def wrapper(*args, **kwargs):
            startTime = time()
            result = func(*args, **kwargs)
            endTime = time()
            # print(f'Time elap: {endTime - startTime:.2f}')
            if endTime - startTime < tol:
                sleep(tol - endTime + startTime)
            # print(f'Ended')
            return result

        return wrapper

    return decorator


class HEIGHTSENSORCOMM:
    def __init__(self):
        self.init = False
        self.state = WAITING_DOWN_FRAME_HEAD_1

        self.height = -1.0
        self.heightDeque = deque(maxlen=10)
        self.heightAvg = -1.0

        self.downSer = serial.Serial(
            port=PORT,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        self.dataBuf = bytearray()

        rospy.init_node('height_sensor_comm', anonymous=True)
        rospy.Rate(10)
        self.heightPub = rospy.Publisher('/skp40/height', Vector3Stamped, queue_size=1)

    def getTime_now(self):
        return time()

    def readData(self):
        while True:
            data = self.downSer.read(1)
            if data:
                if self.state == WAITING_DOWN_FRAME_HEAD_1:
                    if data == DOWN_FRAME_HEAD_1:
                        self.state = WAITING_DOWN_FRAME_HEAD_2
                elif self.state == WAITING_DOWN_FRAME_HEAD_2:
                    if data == DOWN_FRAME_HEAD_2:
                        self.state = READING_DATA
                        dataBuf = bytearray()
                elif self.state == READING_DATA:
                    # print(f'reading data buffer len {len(dataBuf)}')
                    dataBuf.append(data[0])
                    if len(dataBuf) == FRAME_LEN:
                        #print('down: ', DOWN_FRAME_HEAD.hex(), dataBuf[:-1].hex(), dataBuf[-1:].hex())
                        downData = unpack(DOWN_PROTO, dataBuf)
                        frameTail = downData[-1]
                        if frameTail != ord(DOWN_FRAME_TAIL):
                            #print('Tail error')
                            #print(f'Tail should be {ord(DOWN_FRAME_TAIL)} but is {frameTail}')
                            # raise AssertionError
                            pass
                        elif downData[0] != 0:
                            pass
                        else:
                            self.height = (downData[-3] + (downData[-4] << 8) + (downData[-5] << 16)) / 1000
                            #print(f'Height is : {self.height}')
                            
                            self.heightDeque.append(self.height)
                            
                            if len(self.heightDeque) > 0:
                                self.heightAvg = sum(self.heightDeque) / len(self.heightDeque)
                            else:
                                self.heightAvg = self.height

                        self.state = WAITING_DOWN_FRAME_HEAD_1

            if self.state == WAITING_DOWN_FRAME_HEAD_1:
                dataBuf = bytearray()

    def startRead(self):
        tRead = threading.Thread(target=self.readData)
        tRead.start()

    def printState(self):
        system('clear')
        print('-' * 20)
        print(f'Height: {self.height:.3f}m')
        print(f'Deque: {self.heightDeque}')

    def rosPub(self):
        msg = Vector3Stamped()
        msg.vector.x = self.height
        msg.vector.y = self.heightAvg
        self.heightPub.publish(msg)

    @timer(tol=1 / HZ)
    def spinOnce(self):
        #self.printState()
        self.rosPub()

    def spin(self):
        self.startRead()
        while not rospy.is_shutdown():
            self.spinOnce()


if __name__ == '__main__':
    print(f'PORT is {PORT}')
    hsc = HEIGHTSENSORCOMM()
    hsc.spin()

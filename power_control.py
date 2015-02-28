#!/usr/bin/env python

import serial, sys, time
from config import *

ser_dev = None

def power_cycle(down_time=2):
    '''cycle power to test boards and servos'''
    global ser_dev
    if ser_dev is None:
        ser_dev = serial.Serial(FTDI_POWER, rtscts=True)
    print("powering off")
    ser_dev.setRTS(1)
    time.sleep(down_time)
    print("powering on")
    ser_dev.setRTS(0)
    
if __name__ == '__main__':
    power_cycle()

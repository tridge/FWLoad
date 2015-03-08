#!/usr/bin/env python

import serial, sys, time
from config import *
import glob, util

ser_dev = None

def power_cycle(down_time=4):
    '''cycle power to test boards and servos'''
    global ser_dev
    if ser_dev is None:
        ftdi_devices = glob.glob(FTDI_POWER)
        if len(ftdi_devices) != 1:
            util.failure("Must be exactly 1 FTDI device - %u found" % len(ftdi_devices))
        ser_dev = serial.Serial(ftdi_devices[0], rtscts=True)
    print("power cycling")
    ser_dev.setRTS(1)
    time.sleep(down_time)
    ser_dev.setRTS(0)
    
if __name__ == '__main__':
    power_cycle()

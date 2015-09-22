#!/usr/bin/env python

import serial, sys, time
import logger
from config import *
from PixETE import PixETE

def power_cycle(down_time=4):
    if ETE == 0:
        '''cycle power to test boards and servos'''
        ser_dev = logger.get_ftdi()
        logger.info("power cycling")
        ser_dev.setRTS(1)
        time.sleep(down_time)
        ser_dev.setRTS(0)
    elif ETE == 1:
        ete = PixETE()
        logger.info("ETE power cycling")
        ete.command_bytes('power_cycle')
        time.sleep(down_time)


def on():
    ser_dev = logger.get_ftdi()
    ser_dev.setRTS(0)
    
if __name__ == '__main__':
    power_cycle()
    
            

#!/usr/bin/env python

import serial, sys, time
import logger

def power_cycle(down_time=4):
    '''cycle power to test boards and servos'''
    ser_dev = logger.get_ftdi()
    logger.info("power cycling")
    ser_dev.setRTS(1)
    time.sleep(down_time)
    ser_dev.setRTS(0)
    
if __name__ == '__main__':
    power_cycle()

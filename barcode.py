#!/usr/bin/env python
'''
monitors a serial port for a barcode
'''

import sys, util, serial, threading, time
from StringIO import StringIO
from config import *

barcode = None
baudrate = 9600

tty = serial.Serial(port=BARCODE_SCANNER, baudrate=baudrate, timeout=None)

def handle_data(data):
    global barcode
    barcode = data

def read_from_port(ser):
    while True:
        try:
            raw_data = tty.read(1)
            if raw_data:
                raw_data += ser.read(ser.inWaiting())
                handle_data(raw_data)
        except Exception as ex:
            #continue
            raise ex

def monitor_scanner():
    thread = threading.Thread(target=read_from_port, args=(tty,))
    thread.daemon = True
    thread.start()

def get_barcode():
    global barcode
    temp = None
    if barcode:
        temp = barcode
        barcode = None
    return temp

if __name__ == '__main__':
    monitor_scanner()
    while threading.active_count() > 0:
        code = get_barcode()
        if code:
            print("Barcode detected: %s" % code)
            barcode = None
        time.sleep(0.1)

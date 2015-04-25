#!/usr/bin/env python
'''
monitors a serial port for a barcode
'''

import sys, util, serial
from config import *

def barcode_read():
    '''block waiting for a barcode to be entered'''
    try:
        tty = serial.Serial(port=BARCODE_SCANNER, baudrate=9600, timeout=1)
    except Exception as ex:
        print("Failed to open barcode scanner: %s" % ex)
        return None

    barcode = ''
    while True:
        try:
            raw_data = tty.read(1)
        except Exception as ex:
            # this should happen on 1 second timeout
            if barcode != '':
                break
        barcode += raw_data
    if barcode == '':
        return None
    return barcode

if __name__ == '__main__':
    code = barcode_read()
    print("Barcode: %s" % code)

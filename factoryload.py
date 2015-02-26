#!/usr/bin/env python
'''
  run factory load, calibration and test
'''

import accelcal
import jtag
import power_control
import time
import util
import sys
from config import *

util.power_wait_devices()
jtag.load_all_firmwares()
util.wait_devices([USB_DEV_TEST, USB_DEV_REFERENCE, FMU_DEBUG])
time.sleep(1)
accelcal.accel_calibrate()
print("Factory install complete")
sys.exit(0)

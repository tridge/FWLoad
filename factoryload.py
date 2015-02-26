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

util.power_wait_devices([FMU_JTAG, IO_JTAG, FMU_DEBUG])
jtag.load_all_firmwares(retries=3)
time.sleep(1)
accelcal.accel_calibrate()
print("Factory install complete")
sys.exit(0)

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
if not jtag.load_all_firmwares(retries=3):
    print("**** Factory install FAILED (JTAG) ***")
    sys.exit(1)
time.sleep(1)
if not accelcal.accel_calibrate_retries(retries=2):
    print("**** Factory install FAILED (ACCELCAL) ***")
    sys.exit(1)
print("Factory install complete - PASSED")
sys.exit(0)

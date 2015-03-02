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

while not util.power_wait_devices([FMU_JTAG, IO_JTAG, FMU_DEBUG]):
    print("waiting for power up....")

if not jtag.load_all_firmwares(retries=3):
    print("FAILED: JTAG firmware install failed")
    sys.exit(1)

if not accelcal.accel_calibrate_retries(retries=4):
    print("FAILED: accelerometer calibration failed")
    sys.exit(1)

print("PASSED: Factory install complete")
sys.exit(0)

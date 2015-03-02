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

while True:
    start_time = time.time()
    
    while not util.power_wait_devices([FMU_JTAG, IO_JTAG, FMU_DEBUG]):
        print("waiting for power up....")

    if not jtag.load_all_firmwares(retries=3):
        print("FAILED: JTAG firmware install failed")
        power_control.power_cycle()
        continue
    
    if not accelcal.accel_calibrate_retries(retries=4):
        print("FAILED: accelerometer calibration failed")
        power_control.power_cycle()
        continue

    print("PASSED: Factory install complete (%u seconds)" % (time.time() - start_time))

    # power cycle at the end, simulating new board put in
    power_control.power_cycle()


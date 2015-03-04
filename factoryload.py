#!/usr/bin/env python
'''
  run factory load, calibration and test
'''

import accelcal
import jtag
import power_control
import time
import util
import sys, os
import colour_text
from config import *

# disable stdout buffering
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--test", dest="test", default=False, action='store_true', help="run in test loop")
args = parser.parse_args()


colour_text.print_blue("Starting up")

while True:

    util.kill_processes(['mavproxy.py', GDB])

    if args.test:
        # power cycle each time, simulating new board put in
        power_control.power_cycle()
    else:
        # wait for the power to be switched off
        print("waiting for power off")
        util.wait_no_device([FMU_JTAG, IO_JTAG], timeout=600)

    # wait for the power to come on again
    while not util.wait_devices([FMU_JTAG, IO_JTAG, FMU_DEBUG]):
        print("waiting for power up....")

    start_time = time.time()

    colour_text.clear_screen()

    colour_text.print_blue('''
=======================
| Starting installation
=======================
''')
    
    if not jtag.load_all_firmwares(retries=3):
        colour_text.print_fail('''
======================================
| FAILED: JTAG firmware install failed
======================================
''')
        continue
    
    if not accelcal.accel_calibrate_retries(retries=4):
        colour_text.print_fail('''
==========================================
| FAILED: accelerometer calibration failed
==========================================
''')
        continue

    # all OK
    colour_text.print_green('''
================================================
| PASSED: Factory install complete (%u seconds)
================================================
''' %  (time.time() - start_time))

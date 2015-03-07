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
import logging
import colour_text
from config import *

# disable stdout buffering
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--test", dest="test", default=False, action='store_true', help="run in test loop")
parser.add_argument("--once", dest="once", default=False, action='store_true', help="run one install only")
parser.add_argument("--nofw", dest="nofw", default=False, action='store_true', help="don't reload firmware")
args = parser.parse_args()


colour_text.print_blue("Starting up")

def factory_install():
    '''main factory installer'''
    start_time = time.time()

    if not args.test:
        colour_text.clear_screen()

    logdir = logging.new_log_dir()

    print("Logging to %s" % logdir)
    tee = util.Tee(os.path.join(logdir, "run.log"))

    colour_text.print_blue('''
=======================
| Starting installation
=======================
''')

    print(time.ctime())
    
    if not args.nofw and not jtag.load_all_firmwares(retries=3):
        colour_text.print_fail('''
======================================
| FAILED: JTAG firmware install failed
======================================
''')
        tee.close()
        return False
    
    if not accelcal.accel_calibrate_retries(retries=4):
        colour_text.print_fail('''
==========================================
| FAILED: accelerometer calibration failed
==========================================
''')
        tee.close()
        return False

    # all OK
    colour_text.print_green('''
================================================
| PASSED: Factory install complete (%u seconds)
================================================
''' %  (time.time() - start_time))
    tee.close()
    return True

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

    ret = factory_install()

    if args.once:
        sys.exit(int(not ret))

#!/usr/bin/env python
'''
  run factory load, calibration and test
'''

from config import *
import logger
import accelcal
import jtag
import power_control
import time
import util
import sys, os, fcntl
import logger
import colour_text
import connection
import barcode
import savedstate

fh = open(os.path.realpath(__file__), 'r')
try:
    fcntl.flock(fh, fcntl.LOCK_EX|fcntl.LOCK_NB)
except:
    print("another instance of this script is already running. exiting...")
    sys.exit(0)

# disable stdout buffering
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--test", default=False, action='store_true', help="run in test loop")
parser.add_argument("--once", default=False, action='store_true', help="run one install only")
parser.add_argument("--nofw", default=False, action='store_true', help="don't reload firmware")
parser.add_argument("--erase", default=False, action='store_true', help="erase firmware and parameters")
parser.add_argument("--monitor", default=None, help="monitor address")
args = parser.parse_args()

if args.monitor:
    REMOTE_MONITOR['ref'] = args.monitor + ":16550"
    REMOTE_MONITOR['test'] = args.monitor + ":16551"

colour_text.print_blue("Starting up")

def factory_install(device_barcode):
    '''main factory installer'''
    start_time = time.time()

    if not args.test:
        colour_text.clear_screen()

    # start a new log directory on each run
    logger.new_log_dir()
    logger.reopen_logfile()

    logdir = logger.get_log_dir()
    logger.info("Logging to %s" % logdir)

    colour_text.print_blue('''
==================================================
| Starting installation. Barcode is %s
==================================================
''' % device_barcode)

    logger.info(time.ctime())
    
    if args.erase:
        if not jtag.erase_firmwares():
            colour_text.print_fail('''
======================================
| FAILED: JTAG firmware erase failed
======================================
''')
            logger.critical("JTAG firmware erase failed")
            return False
    
    if not args.nofw and not jtag.load_all_firmwares(retries=3):
        colour_text.print_fail('''
======================================
| FAILED: JTAG firmware install failed
======================================
''')
        logger.critical("JTAG firmware install failed")
        return False

    if args.erase:
        if not connection.erase_parameters():
            colour_text.print_fail('''
==========================================
| FAILED: Failed to erase parameters
==========================================
''')
            logger.critical("Failed to erase parameters")
            return False

    if not accelcal.accel_calibrate_retries(retries=4):
        colour_text.print_fail('''
==========================================
| FAILED: Accelerometer calibration failed
==========================================
''')
        logger.critical("Accelerometer calibration failed")
        return False

    # all OK
    colour_text.print_green('''
================================================
| Device: %s
| PASSED: Factory install complete (%u seconds)
================================================
''' %  (device_barcode, (time.time() - start_time)))
    logger.info("Factory install complete (%u seconds)" % (time.time() - start_time))
    return True

# load the jig state file
savedstate.init()
savedstate.reset('current_cycles')

while True:
    logger.get_ftdi()
    jigstate = savedstate.get()
    logger.info("jigstate: total_cycles = %i" % jigstate['total_cycles'])
    logger.info("jigstate: current_cycles = %i" % jigstate['current_cycles'])

    util.kill_processes(['mavproxy.py', GDB])

    if args.test:
        # power cycle each time, simulating new board put in
        power_control.power_cycle()
    else:
        # wait for the power to be switched off, disable serial logging
        logger.info("waiting for power off")
        util.wait_no_device([FMU_JTAG, IO_JTAG], timeout=600)

    device_barcode = None
    if not args.test:
        colour_text.print_blue('''
==========================================
| PLEASE SWIPE DEVICE BARCODE
==========================================
''')
        device_barcode = barcode.barcode_read()
        if device_barcode is None:
            colour_text.print_fail('''
            ==========================================
            | FAILED: Barcode not detected
            ==========================================
            ''')
            logger.critical("Barcode not detected")
            time.sleep(2)
            continue
        
        # log the barcode
        logger.info("Barcode detected: %s" % device_barcode)
    
    # wait for the power to come on again
    while not util.wait_devices([FMU_JTAG, IO_JTAG, FMU_DEBUG]):
        logger.info("waiting for power up....")

    ret = factory_install(device_barcode)

    # increment the cycles counters
    savedstate.incr('current_cycles')
    savedstate.incr('total_cycles')

    if args.once:
        sys.exit(int(not ret))

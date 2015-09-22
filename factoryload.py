#!/usr/bin/env python
'''
  run factory load, calibration and test
'''

from config import *
from subprocess import Popen, PIPE
import configcheck
import time
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
import rotate
import barcode
import savedstate
import otp_program_mod
import uuid
import test_sensors
from PixETE import PixETE

ete = PixETE()

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
parser.add_argument("--barcode", default=None, help="override barcode")
parser.add_argument("--otp-show", default=False, action='store_true', help="show_otp")
parser.add_argument("--otp-write", default=False, action='store_true', help="write_otp")

args = parser.parse_args()

if args.monitor:
    REMOTE_MONITOR['ref'] = args.monitor + ":16550"
    REMOTE_MONITOR['test'] = args.monitor + ":16551"

colour_text.print_blue("Starting up")

def factory_install(device_barcode):
    '''main factory installer'''
    start_time = time.time()

    script_dir = os.path.dirname(os.path.abspath(__file__))

    ete.command_bytes('test_work')

    if not args.test:
        colour_text.clear_screen()

    # start a new log directory on each run
    logger.new_log_dir()
    logger.reopen_logfile()

    logdir = logger.get_log_dir()
    logger.info("Logging to %s" % logdir)
    #logger.info("Device barcode %s" % device_barcode)

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
            ete.command_bytes('test_fail')
            ete.command_bytes('reset')
            ete.accel(10000)
            ete.yawspeed(5000)
            ete.rollspeed(10000)
            return False
    
    if not args.nofw and not jtag.load_all_firmwares(retries=3):
        colour_text.print_fail('''
======================================
| FAILED: JTAG firmware install failed
======================================
''')
        logger.critical("JTAG firmware install failed")
        ete.command_bytes('test_fail')
        ete.command_bytes('reset')
        ete.accel(100000)
        ete.yawspeed(5000)
        ete.rollspeed(10000)
        try:
            conn = connection.Connection(ref_only=True)
            rotate.center_servos(conn)
        except Exception as ex:
            print("Failed to center servos: %s" % ex)
            pass
        return False

    if args.erase:
        if not connection.erase_parameters():
            colour_text.print_fail('''
==========================================
| FAILED: Failed to erase parameters
==========================================
''')
            logger.critical("Failed to erase parameters")
            ete.command_bytes('test_fail')
            ete.position(0, 0)
            ete.command_bytes('reset')
            ete.accel(100000)
            ete.yawspeed(5000)
            ete.rollspeed(10000)
            return False

    if not accelcal.accel_calibrate_retries(retries=4):
        colour_text.print_fail('''
==========================================
| FAILED: Accelerometer calibration failed
==========================================
''')
        logger.critical("Accelerometer calibration failed")
        ete.command_bytes('test_fail')
        ete.position(0, 0)
        ete.command_bytes('reset')
        ete.accel(100000)
        ete.yawspeed(5000)
        ete.rollspeed(10000)
        return False

    # all OK

    #Add OTP HERE
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if args.otp_show:
        p1 = Popen(['python', script_dir + '/otp_program.py', '--port', FMU_DEBUG,'--only-display',"abc"], stdin=PIPE, stdout=PIPE, stderr=PIPE)
        output, err = p1.communicate()
        logger.info(output)
    #conn = connection.Connection(ref_only=False)
    #otp_program_mod.Display_OTP(conn)


    

    def getMacAddress(): 
        if sys.platform == 'win32': 
            for line in os.popen("ipconfig /all"): 
                if line.lstrip().startswith('Physical Address'): 
                    mac = line.split(':')[1].strip().replace('-',':') 
                    break 
        else: 
            for line in os.popen("/sbin/ifconfig"): 
                if line.find('Ether') > -1: 
                    mac = line.split()[4] 
                    break 
        return mac 

    print("Manufacturer info : 3D Robotics, Inc, \xA9 2015")
    print "MAC Address:",getMacAddress() #.join(['{:02x}'.format((uuid.getnode() >> i) & 0xff) for i in range(0,8*6,8)][::-1])

    colour_text.print_blue('''Barcode is %s''' % device_barcode)
    print("date of testing :" + time.strftime("%x"))
    print("time of testing :" + time.strftime("%X"))
    accel_data0 = "%f,%f,%f,%f,%f,%f" % (test_sensors.offset[0][0] ,test_sensors.offset[0][1] ,test_sensors.offset[0][2],test_sensors.scale_factor[0][0],test_sensors.scale_factor[0][1],test_sensors.scale_factor[0][2])
    accel_data2 = "%f,%f,%f,%f,%f,%f" % (test_sensors.offset[2][0] ,test_sensors.offset[2][1] ,test_sensors.offset[2][2],test_sensors.scale_factor[2][0],test_sensors.scale_factor[2][1],test_sensors.scale_factor[2][2])
    print "Accel :", accel_data0
    print "Accel :", accel_data2
    if args.otp_write:
	#Manufacturing Info
        p2 = Popen(['python', script_dir + '/otp_program.py', '--port', FMU_DEBUG,'3D Robotics, Inc, \xA9 2015',getMacAddress(),device_barcode,time.strftime("%x"),time.strftime("%X"),'--',str(accel_data0),str(accel_data2)], stdin=PIPE, stdout=PIPE, stderr=PIPE)
        output, err = p2.communicate()
        logger.info(output)
	logger.info(err)
	time.sleep(1)
        #Display
        p3 = Popen(['python', script_dir + '/otp_program.py', '--port', FMU_DEBUG,'--only-display',"abc"], stdin=PIPE, stdout=PIPE, stderr=PIPE)
        output, err = p3.communicate()
        logger.info(output)

    colour_text.print_green('''
================================================
| Device: %s
| PASSED: Factory install complete (%u seconds)
================================================
''' %  (device_barcode, (time.time() - start_time)))
    logger.info("Factory install complete (%u seconds)" % (time.time() - start_time))
    ete.command_bytes('test_pass')
    ete.position(0, 0)
    ete.command_bytes('reset')
    ete.accel(100000)
    ete.yawspeed(5000)
    ete.rollspeed(10000)
    return True

# load the jig state file
savedstate.init()
savedstate.reset('current_cycles')

while True:
    logger.get_ftdi()
    jigstate = savedstate.get()
    logger.info("jigstate: total_cycles = %i" % jigstate['total_cycles'])
    logger.info("jigstate: current_cycles = %i" % jigstate['current_cycles'])
    # Set ETE speeds
    if ETE == 1:
        ete = PixETE()
        ete.yawspeed(5000)
        ete.rollspeed(10000)

    util.kill_processes(['mavproxy.py', GDB])

    if args.test:
        # power cycle each time, simulating new board put in
        power_control.power_cycle()
    else:
        # wait for the power to be switched off, disable serial logging
        logger.info("waiting for power off")
        util.wait_no_device([FMU_JTAG, IO_JTAG], timeout=600)

    device_barcode = args.barcode
    if not args.test and device_barcode is None:
        ete.command_bytes('test_wait')
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
        
        # we don't use logger for the barcode here as we are still on the previous
        # boards log
        print("Got barcode: %s" % device_barcode)
        logger.info("Barcode detected")
    
    # wait for the power to come on again
    while not util.wait_devices([FMU_JTAG, IO_JTAG, FMU_DEBUG]):
        logger.info("waiting for power up....")

    ret = factory_install(device_barcode)

    # increment the cycles counters
    savedstate.incr('current_cycles')
    savedstate.incr('total_cycles')

    if args.once:
        sys.exit(int(not ret))

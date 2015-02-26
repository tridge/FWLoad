#!/usr/bin/env python
'''
run an accelcal on the test jig
'''

import pexpect, sys, time
from StringIO import StringIO
from config import *
from util import *
import rotate
from pymavlink import mavutil
import expect_callback
import test_sensors


def accel_calibrate_run(ref, refmav, test, testmav):
    '''run accelcal'''
    print("STARTING ACCEL CALIBRATION")

    test.send("accelcal\n")
    for rotation in ['level', 'left', 'right', 'up', 'down', 'back']:
        test.expect("Place vehicle")
        test.expect("and press any key")
        rotate.set_rotation(ref, refmav, rotation)
        test.send("\n")
    i = test.expect(["Calibration successful","Calibration FAILED"])
    if i != 0:
        util.failed("Accel calibration failed")
    test.send("\n")
    test.expect("RTL>")
    test.send("param fetch\n")
    rotate.set_rotation(ref, refmav, 'level')
    test.expect('Received [0-9]+ parameters')


def accel_calibrate():
    '''run full accel calibration'''
    reflog = StringIO()
    testlog = StringIO()
    try:
        print("CONNECTING TO REFERENCE BOARD")
        ref  = pexpect.spawn("mavproxy.py --master %s --out 59.167.251.244:15550 --out 127.0.0.1:14550 --aircraft RefBoard" % USB_DEV_REFERENCE,
                             logfile=reflog, timeout=10)
        ref.expect(['Received [0-9]+ parameters', 'MANUAL>'])
        ref.expect(['Received [0-9]+ parameters', 'MANUAL>'])

        print("CONNECTING MAVLINK TO REFERENCE BOARD")
        refmav = mavutil.mavlink_connection('127.0.0.1:14550', robust_parsing=True)
        refmav.wait_heartbeat()
    except Exception as ex:
        show_error('Connecting to reference board', ex, reflog)
    
    try:
        print("CONNECTING TO TEST BOARD")
        test = pexpect.spawn("mavproxy.py --master %s --out 127.0.0.1:14551 --aircraft TestBoard" % USB_DEV_TEST,
                             logfile=testlog, timeout=10)
        test.expect(['Received [0-9]+ parameters', 'RTL>'])
        test.expect(['Received [0-9]+ parameters', 'RTL>'])
        
        print("CONNECTING MAVLINK TO TEST BOARD")
        testmav = mavutil.mavlink_connection('127.0.0.1:14551', robust_parsing=True)
        testmav.wait_heartbeat()
    except Exception as ex:
        show_error('Connecting to test board', ex, testlog)

    expect_callback.expect_callback_mav([refmav, testmav])

    accel_calibrate_run(ref, refmav, test, testmav)
    test_sensors.check_accel_cal(ref, refmav, test, testmav)
    test_sensors.check_gyro_cal(ref, refmav, test, testmav)
    print("Accel calibration complete")

    test_sensors.check_baro(ref, refmav, test, testmav)
    test_sensors.check_mag(ref, refmav, test, testmav)

if __name__ == '__main__':
    accel_calibrate()

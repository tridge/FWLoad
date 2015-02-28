#!/usr/bin/env python
'''
run an accelcal on the test jig
'''

import pexpect, sys, time
from StringIO import StringIO
from config import *
import util
import rotate
from pymavlink import mavutil
import test_sensors
import mav_reference
import mav_test


def adjust_ahrs_trim(test, level_attitude):
    '''adjust AHRS_TRIM_{X,y} on test board based on attitude of reference
    board when level in accelcal.
    Note that increasing AHRS_TRIM_X decreases roll
    '''
    trim_x = util.param_value(test, 'AHRS_TRIM_X')
    trim_y = util.param_value(test, 'AHRS_TRIM_Y')
    trim_x -= level_attitude.roll
    trim_y -= level_attitude.pitch
    util.param_set(test, 'AHRS_TRIM_X', trim_x)
    util.param_set(test, 'AHRS_TRIM_Y', trim_y)
    print("Trim: %f %f %f %f" % (level_attitude.roll, level_attitude.pitch, trim_x, trim_y))

def accel_calibrate_run(ref, refmav, test, testmav, testlog):
    '''run accelcal'''
    print("STARTING ACCEL CALIBRATION")

    level_attitude = None
    test.send("accelcal\n")
    for rotation in ['level', 'left', 'right', 'up', 'down', 'back']:
        try:
            test.expect("Place vehicle")
            test.expect("and press any key")
        except Exception as ex:
            util.show_tail(testlog)
            util.failure("Failed to get place vehicle message for %s" % rotation)
        attitude = rotate.set_rotation(ref, refmav, rotation)
        if rotation == 'level':
            level_attitude = attitude
        test.send("\n")
    i = test.expect(["Calibration successful","Calibration FAILED"])
    if i != 0:
        util.show_tail(testlog)
        util.failure("Accel calibration failed")
    test.send("\n")
    test.expect("RTL>")
    test.send("param fetch\n")
    rotate.set_rotation(ref, refmav, 'level', wait=False)
    test.expect('Received [0-9]+ parameters')
    adjust_ahrs_trim(test, level_attitude)

def accel_calibrate():
    '''run full accel calibration'''
    reflog = StringIO()
    testlog = StringIO()
    try:
        ref = mav_reference.mav_reference(reflog)
        ref.expect(['Received [0-9]+ parameters', 'MANUAL>'])
        ref.expect(['Received [0-9]+ parameters', 'MANUAL>'])

        print("CONNECTING MAVLINK TO REFERENCE BOARD")
        refmav = mavutil.mavlink_connection('127.0.0.1:14550', robust_parsing=True)
        refmav.wait_heartbeat()
    except Exception as ex:
        util.show_error('Connecting to reference board', ex, reflog)
    
    try:
        test = mav_test.mav_test(testlog)
        test.expect(['Received [0-9]+ parameters', 'RTL>'])
        test.expect(['Received [0-9]+ parameters', 'RTL>'])
        
        print("CONNECTING MAVLINK TO TEST BOARD")
        testmav = mavutil.mavlink_connection('127.0.0.1:14551', robust_parsing=True)
        testmav.wait_heartbeat()
    except Exception as ex:
        util.show_error('Connecting to test board', ex, testlog)

    # get all parms again so they are in the log
    test.send("param fetch\n")
    test.expect('Received [0-9]+ parameters')

    accel_calibrate_run(ref, refmav, test, testmav, testlog)
    test_sensors.check_accel_cal(ref, refmav, test, testmav)
    test_sensors.check_gyro_cal(ref, refmav, test, testmav)
    print("Accel calibration complete")

    test_sensors.check_baro(ref, refmav, test, testmav)
    test_sensors.check_mag(ref, refmav, test, testmav)
    test_sensors.check_power(ref, refmav, test, testmav)

def accel_calibrate_retries(retries=2):
    '''run full accel calibration with retries
    return True on success, False on failure
    '''
    while retries > 0:
        retries -= 1
        try:
            accel_calibrate()
        except Exception as ex:
            if retries == 0:
                return False
            print("RETRYING ACCEL CAL")
            continue
        print("PASSED ACCEL CAL")
        return True
    print("no more retries")
    return False

if __name__ == '__main__':
    accel_calibrate()

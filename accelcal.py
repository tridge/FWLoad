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
import power_control
from pymavlink.rotmat import Matrix3, Vector3

def adjust_ahrs_trim(ref, refmav, test, testmav, level_attitude):
    '''
    force the AHRS trim to zero, which removes the effect of the jig not being quite
    level. This is only incorrect if the accels are not aligned on the board, which
    we check in check_accel_cal later
    '''
    trim_x = util.param_value(test, 'AHRS_TRIM_X')
    trim_y = util.param_value(test, 'AHRS_TRIM_Y')
    print("Trim: %f %f %f %f" % (level_attitude.roll, level_attitude.pitch, trim_x, trim_y))
    # check ref accel to all 3 accels on test board
    # include log of the discrepancy
    util.param_set(test, 'AHRS_TRIM_X', 0)
    util.param_set(test, 'AHRS_TRIM_Y', 0)

    # check all accels are in range
    util.discard_messages(refmav)
    util.discard_messages(testmav)
    ref_imu = refmav.recv_match(type='RAW_IMU', blocking=True, timeout=3)
    test_imu1 = testmav.recv_match(type='RAW_IMU', blocking=True, timeout=3)
    test_imu2 = testmav.recv_match(type='SCALED_IMU2', blocking=True, timeout=3)
    test_imu3 = testmav.recv_match(type='SCALED_IMU3', blocking=True, timeout=3)
    if ref_imu is None:
        util.failure("Lost comms to reference board in ahrs trim")
    if test_imu1 is None or test_imu2 is None or test_imu3 is None:
        util.failure("Lost comms to test board in ahrs trim")
    ref_accel = Vector3(ref_imu.xacc, ref_imu.yacc, ref_imu.zacc)*9.81*0.001
    test_accel1 = Vector3(test_imu1.xacc, test_imu1.yacc, test_imu1.zacc)*9.81*0.001
    test_accel2 = Vector3(test_imu2.xacc, test_imu2.yacc, test_imu2.zacc)*9.81*0.001
    test_accel3 = Vector3(test_imu3.xacc, test_imu3.yacc, test_imu3.zacc)*9.81*0.001

    test_error1 = (ref_accel-test_accel1).length()
    test_error2 = (ref_accel-test_accel2).length()
    test_error3 = (ref_accel-test_accel3).length()
    (ref_roll, ref_pitch) = util.attitude_estimate(ref_imu)
    (test_roll1, test_pitch1) = util.attitude_estimate(test_imu1)
    (test_roll2, test_pitch2) = util.attitude_estimate(test_imu2)
    (test_roll3, test_pitch3) = util.attitude_estimate(test_imu3)

    print("Tilt Ref=(%.1f %.1f) Test1=(%.1f %.1f) Test2=(%.1f %.1f) Test3=(%.1f %.1f)" % (
        ref_roll, ref_pitch,
        test_roll1, test_pitch1,
        test_roll2, test_pitch2,
        test_roll3, test_pitch3))

def accel_calibrate_run(ref, refmav, test, testmav, testlog):
    '''run accelcal'''
    print("STARTING ACCEL CALIBRATION")

    # use zero trims on reference board
    util.param_set(ref, 'AHRS_TRIM_X', 0)
    util.param_set(ref, 'AHRS_TRIM_Y', 0)

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
    util.wait_prompt(test)
    test.send("param fetch\n")
    rotate.set_rotation(ref, refmav, 'slant', wait=True)
    test.expect('Received [0-9]+ parameters')
    adjust_ahrs_trim(ref, refmav, test, testmav, level_attitude)

def accel_calibrate():
    '''run full accel calibration'''
    reflog = StringIO()
    testlog = StringIO()
    try:
        ref = mav_reference.mav_reference(reflog)
        ref.expect(['MANUAL>'], timeout=15)

        print("CONNECTING MAVLINK TO REFERENCE BOARD")
        refmav = mavutil.mavlink_connection('127.0.0.1:14550', robust_parsing=True)
        refmav.wait_heartbeat()
    except Exception as ex:
        util.show_error('Connecting to reference board', ex, reflog)
    
    try:
        test = mav_test.mav_test(testlog)
        util.wait_prompt(test)
        
        print("CONNECTING MAVLINK TO TEST BOARD")
        testmav = mavutil.mavlink_connection('127.0.0.1:14551', robust_parsing=True)
        testmav.wait_heartbeat()
    except Exception as ex:
        util.show_error('Connecting to test board', ex, testlog)

    accel_calibrate_run(ref, refmav, test, testmav, testlog)
    test_sensors.check_accel_cal(ref, refmav, test, testmav)
    test_sensors.check_gyro_cal(ref, refmav, test, testmav)
    print("Accel calibration complete")

    # we run the sensor checks from here to avoid re-opening the links
    test_sensors.check_all_sensors(ref, refmav, test, testmav)

def accel_calibrate_retries(retries=4):
    '''run full accel calibration with retries
    return True on success, False on failure
    '''
    while retries > 0:
        retries -= 1
        if not util.wait_devices([USB_DEV_TEST, USB_DEV_REFERENCE]):
            print("FAILED to find USB test and reference devices")
            power_control.power_cycle(down_time=4)
            continue
        try:
            time.sleep(2)
            accel_calibrate()
        except Exception as ex:
            print("accel cal failed: %s" % ex)
            if retries > 0:
                print("RETRYING ACCEL CAL")
                power_control.power_cycle(down_time=4)
            continue
        print("PASSED ACCEL CAL")
        return True
    print("accelcal: no more retries")
    return False

if __name__ == '__main__':
    accel_calibrate_retries()

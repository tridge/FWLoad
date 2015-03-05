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

    # get the board level
    rotate.set_rotation(ref, refmav, 'level')

    # we need to work out what the error in attitude of the 3 IMUs on the test jig is
    # to do that we start with it level, and measure the roll/pitch as compared to the reference
    # then we rotate it to pitch 90 and measure the yaw error

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

    (ref_roll, ref_pitch) = util.attitude_estimate(ref_imu)
    (test_roll1, test_pitch1) = util.attitude_estimate(test_imu1)
    (test_roll2, test_pitch2) = util.attitude_estimate(test_imu2)
    (test_roll3, test_pitch3) = util.attitude_estimate(test_imu3)

    # get the roll and pitch errors
    roll_error1 = (test_roll1 - ref_roll)
    roll_error2 = (test_roll2 - ref_roll)
    roll_error3 = (test_roll3 - ref_roll)
    pitch_error1 = (test_pitch1 - ref_pitch)
    pitch_error2 = (test_pitch2 - ref_pitch)
    pitch_error3 = (test_pitch3 - ref_pitch)

    # now rotate to pitch 90 to measure the yaw error
    rotate.set_rotation(ref, refmav, 'up')

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

    # start rotating back to level ready for the end of the test
    rotate.set_rotation(ref, refmav, 'level', wait=False)

    # when pointing straight up the roll_esimtate() actually estimates yaw error in body frame
    ref_yaw = util.roll_estimate(ref_imu)
    test_yaw1 = util.roll_estimate(test_imu1)
    test_yaw2 = util.roll_estimate(test_imu2)
    test_yaw3 = util.roll_estimate(test_imu3)
    yaw_error1 = test_yaw1 - ref_yaw
    yaw_error2 = test_yaw2 - ref_yaw
    yaw_error3 = test_yaw3 - ref_yaw

    print("Tilt Ref=(%.1f %.1f %.1f) Test1=(%.1f %.1f %.1f) Test2=(%.1f %.1f %.1f) Test3=(%.1f %.1f %.1f)" % (
        ref_roll, ref_pitch, ref_yaw,
        test_roll1, test_pitch1, test_yaw1,
        test_roll2, test_pitch2, test_yaw2,
        test_roll3, test_pitch3, test_yaw3))

    if (abs(ref_roll) > ROTATION_TOLERANCE or
        abs(ref_pitch) > ROTATION_TOLERANCE or
        abs(ref_yaw) > ROTATION_TOLERANCE):
        util.failure("Reference board rotation error")

    print("Tilt errors: Roll(%.1f %.1f %.1f) Pitch(%.1f %.1f %.1f) Yaw(%.1f %.1f %.1f) " % (
        roll_error1, roll_error2, roll_error3,
        pitch_error1, pitch_error2, pitch_error3,
        yaw_error1, yaw_error2, yaw_error3))

    if (abs(roll_error1) > TILT_TOLERANCE1 or
        abs(roll_error2) > TILT_TOLERANCE1 or
        abs(roll_error3) > TILT_TOLERANCE3):
        util.failure("Test board roll error")

    if (abs(pitch_error1) > TILT_TOLERANCE1 or
        abs(pitch_error2) > TILT_TOLERANCE1 or
        abs(pitch_error3) > TILT_TOLERANCE3):
        util.failure("Test board pitch error")

    if (abs(yaw_error1) > TILT_TOLERANCE1 or
        abs(yaw_error2) > TILT_TOLERANCE1 or
        abs(yaw_error3) > TILT_TOLERANCE3):
        util.failure("Test board yaw error")

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
    rotate.set_rotation(ref, refmav, 'level', wait=False)
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
        util.wait_heartbeat(refmav)
    except Exception as ex:
        util.show_error('Connecting to reference board', ex, reflog)
    
    try:
        test = mav_test.mav_test(testlog)
        util.wait_prompt(test)
        
        print("CONNECTING MAVLINK TO TEST BOARD")
        testmav = mavutil.mavlink_connection('127.0.0.1:14551', robust_parsing=True)
        util.wait_heartbeat(testmav)
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

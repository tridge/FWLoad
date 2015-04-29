#!/usr/bin/env python
'''
run an accelcal on the test jig
'''

import pexpect, sys, time
from config import *
import util
import rotate
from pymavlink import mavutil
import test_sensors
import mav_reference
import mav_test
import power_control
import connection
import logger
from pymavlink.rotmat import Matrix3, Vector3

def adjust_ahrs_trim(conn, level_attitude):
    '''
    force the AHRS trim to zero, which removes the effect of the jig not being quite
    level. This is only incorrect if the accels are not aligned on the board, which
    we check in check_accel_cal later
    '''
    util.param_set(conn.test, 'AHRS_TRIM_X', 0)
    util.param_set(conn.test, 'AHRS_TRIM_Y', 0)

    # get the board level
    rotate.set_rotation(conn, 'level')

    # be really sure it has stopped moving. This next measurement is critical
    time.sleep(2)

    # we need to work out what the error in attitude of the 3 IMUs on the test jig is
    # to do that we start with it level, and measure the roll/pitch as compared to the reference
    # then we rotate it to pitch 90 and measure the yaw error

    # check all accels are in range
    conn.discard_messages()
    ref_imu = conn.refmav.recv_match(type='RAW_IMU', blocking=True, timeout=3)
    test_imu1 = conn.testmav.recv_match(type='RAW_IMU', blocking=True, timeout=3)
    test_imu2 = conn.testmav.recv_match(type='SCALED_IMU2', blocking=True, timeout=3)
    test_imu3 = conn.testmav.recv_match(type='SCALED_IMU3', blocking=True, timeout=3)
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

    # rotate to up position while integrating gyros to test
    # all gyros are working
    rotate.gyro_integrate(conn)

    # finish rotation to pitch 90 to measure the yaw error
    rotate.set_rotation(conn, 'up')

    conn.discard_messages()
    ref_imu = conn.refmav.recv_match(type='RAW_IMU', blocking=True, timeout=3)
    test_imu1 = conn.testmav.recv_match(type='RAW_IMU', blocking=True, timeout=3)
    test_imu2 = conn.testmav.recv_match(type='SCALED_IMU2', blocking=True, timeout=3)
    test_imu3 = conn.testmav.recv_match(type='SCALED_IMU3', blocking=True, timeout=3)
    if ref_imu is None:
        util.failure("Lost comms to reference board in ahrs trim")
    if test_imu1 is None or test_imu2 is None or test_imu3 is None:
        util.failure("Lost comms to test board in ahrs trim")
    ref_accel = Vector3(ref_imu.xacc, ref_imu.yacc, ref_imu.zacc)*9.81*0.001

    # start rotating back to level ready for the end of the test
    rotate.set_rotation(conn, 'level', wait=False)

    # when pointing straight up the roll_esimtate() actually estimates yaw error in body frame
    ref_yaw = util.roll_estimate(ref_imu)
    test_yaw1 = util.roll_estimate(test_imu1)
    test_yaw2 = util.roll_estimate(test_imu2)
    test_yaw3 = util.roll_estimate(test_imu3)
    yaw_error1 = test_yaw1 - ref_yaw
    yaw_error2 = test_yaw2 - ref_yaw
    yaw_error3 = test_yaw3 - ref_yaw

    logger.debug("Tilt Ref=(%.1f %.1f %.1f) Test1=(%.1f %.1f %.1f) Test2=(%.1f %.1f %.1f) Test3=(%.1f %.1f %.1f)" % (
        ref_roll, ref_pitch, ref_yaw,
        test_roll1, test_pitch1, test_yaw1,
        test_roll2, test_pitch2, test_yaw2,
        test_roll3, test_pitch3, test_yaw3))

    if (abs(ref_roll) > ROTATION_TOLERANCE or
        abs(ref_pitch) > ROTATION_TOLERANCE or
        abs(ref_yaw) > ROTATION_TOLERANCE):
        util.failure("Reference board rotation error")

    logger.debug("Tilt offsets: Roll(%.1f %.1f %.1f) Pitch(%.1f %.1f %.1f) Yaw(%.1f %.1f %.1f) " % (
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

    # setting a positive trim value reduces the attitude that is
    # read. So setting a trim of 0.1 when level results in a attitude
    # reading of -5.8 degrees

    # this assumes the reference board always reads correct attitude
    # and that there is no attitude discrepance between test and
    # reference boards    
    trim_x = radians((roll_error1+roll_error2)/2)
    trim_y = radians((pitch_error1+pitch_error2)/2)
    logger.debug("OLD Set trims AHRS_TRIM_X=%.4f AHRS_TRIM_Y=%.4f" % (trim_x, trim_y))

    # this new approach assumes the mpu6000 on the FMU (IMU3) is level
    # with respect to the board, and that any attitude error is due to
    # the isolation board mount. We use the average of the error from
    # IMU1 and IMU2
    trim_x = radians((test_roll1+test_roll2)*0.5 - test_roll3)
    trim_y = radians((test_pitch1+test_pitch2)*0.5 - test_pitch3)

    util.param_set(conn.test, 'AHRS_TRIM_X', trim_x)
    time.sleep(0.2)
    util.param_set(conn.test, 'AHRS_TRIM_Y', trim_y)
    time.sleep(0.2)
    logger.debug("Set trims AHRS_TRIM_X=%.4f AHRS_TRIM_Y=%.4f" % (trim_x, trim_y))
    

def wait_gyros_healthy(conn):
    '''wait for gyros to be healthy'''
    util.wait_heartbeat(conn.testmav)

    # we must have AHRS_ORIENTATION 0 for the accelcal
    # we will fix for AHRS_ORIENTATION=12 later
    util.param_set(conn.test, 'AHRS_ORIENTATION', 0)
    util.param_set(conn.ref, 'AHRS_ORIENTATION', 0)

    # setup right reference board parameters, in case someone has reset them
    util.param_set(conn.ref, 'THR_FAILSAFE', 0)
    util.param_set(conn.ref, 'RCMAP_ROLL', 5)
    util.param_set(conn.ref, 'RCMAP_PITCH', 6)

    # give time for 1Hz loop to set orientation
    time.sleep(2)

    logger.info("Waiting for gyro health")
    start_time = time.time()
    ref_gyros_healthy = False
    test_gyros_healthy = False
    conn.discard_messages()
    while time.time() < start_time + 20 and (not ref_gyros_healthy or not test_gyros_healthy):
        ref_sys_status = conn.refmav.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if ref_sys_status:
            ref_gyros_healthy = (ref_sys_status.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO) != 0
        test_sys_status = conn.testmav.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if test_sys_status:
            test_gyros_healthy = (test_sys_status.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO) != 0
    if not ref_gyros_healthy:
        logger.error("Failed to get healthy reference gyros")
        return False
    if not test_gyros_healthy:
        logger.error("Failed to get healthy test gyros")
        return False
    logger.info("Gyros are healthy")
    return True

def wait_gyros(conn):
    '''wait for gyros to be ready'''
    tries = 3
    while tries > 0:
        tries -= 1
        if not wait_gyros_healthy(conn):
            util.failure("Failed to get healthy gyros")
        rotate.wait_quiescent(conn.refmav)
        logger.info("Reference is quiescent")
        try:
            rotate.wait_quiescent_list(conn.testmav, ['RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3'])
            logger.info("Test is quiescent")
        except Exception as ex:
            logger.debug("Recalibrating gyros : %s" % ex)
            conn.test.send('gyrocal\n')
            conn.expect('Calibrated')
            continue
        break
    if tries == 0:
        util.failure("Failed waiting for gyros")
    logger.info("Gyros ready")

def accel_calibrate_run(conn):
    '''run accelcal'''
    logger.info("STARTING ACCEL CALIBRATION")

    wait_gyros(conn)

    logger.info("Turning safety off")
    rotate.set_rotation(conn, 'level', wait=False)
    util.safety_off(conn.refmav)

    # use zero trims on reference board
    util.param_set(conn.ref, 'AHRS_TRIM_X', 0)
    util.param_set(conn.ref, 'AHRS_TRIM_Y', 0)

    level_attitude = None
    conn.test.send("accelcal\n")
    for rotation in ['level', 'left', 'right', 'up', 'down', 'back']:
        try:
            conn.test.expect("Place vehicle")
            conn.test.expect("and press any key")
        except Exception as ex:
            util.show_tail(conn.testlog)
            util.failure("Failed to get place vehicle message for %s" % rotation)
        attitude = rotate.set_rotation(conn, rotation)
        if rotation == 'level':
            level_attitude = attitude
        conn.test.send("\n")
    i = conn.test.expect(["Calibration successful","Calibration FAILED"])
    if i != 0:
        logger.error(conn.test.before)
        logger.error("Calibration FAILED")
        util.show_tail(conn.testlog)
        util.failure("Accel calibration failed at %s" % time.ctime())
    #logger.info(conn.test.before)
    logger.info("Calibration successful")
    rotate.set_rotation(conn, 'level', wait=False)
    adjust_ahrs_trim(conn, level_attitude)


def accel_calibrate():
    '''run full accel calibration'''

    logger.info("Starting accel cal at %s" % time.ctime())

    conn = connection.Connection()

    # lock the two telemetry ports to prevent the COMMAND_ACK messages in accel cal
    # from looping back between the two telemetry ports
    logger.info("Locking telemetry ports")
    util.lock_serial_port(conn.testmav, mavutil.mavlink.SERIAL_CONTROL_DEV_TELEM1)
    util.lock_serial_port(conn.testmav, mavutil.mavlink.SERIAL_CONTROL_DEV_TELEM2)

    try:
        accel_calibrate_run(conn)
        test_sensors.check_accel_cal(conn)
        test_sensors.check_gyro_cal(conn)
    except Exception as ex:
        conn.close()
        util.show_error('Accel calibration complete???',  ex)

    try:
        # we run the sensor checks from here to avoid re-opening the links
        test_sensors.check_all_sensors(conn)
    except Exception as ex:
        conn.close()
        util.show_error('Test sensors failed', ex)

    try:
        logger.info("Loading factory parameters")
        conn.test.send('param load %s\n' % FACTORY_PARM)
        conn.test.expect('Loaded \d+ parameters from')
        logger.info("Parameters loaded OK")
    except Exception as ex:
        conn.close()
        util.show_error('Parameter load failed', ex)

    conn.close()

def accel_calibrate_retries(retries=4):
    '''run full accel calibration with retries
    return True on success, False on failure
    '''
    while retries > 0:
        retries -= 1
        if not util.wait_devices([USB_DEV_TEST, USB_DEV_REFERENCE]):
            logger.error("FAILED to find USB test and reference devices")
            power_control.power_cycle(down_time=4)
            continue
        try:
            time.sleep(2)
            accel_calibrate()
        except Exception as ex:
            logger.error("accel cal failed: %s" % ex)
            if retries > 0:
                logger.info("RETRYING ACCEL CAL")
                power_control.power_cycle(down_time=4)
            continue
        logger.info("PASSED ACCEL CAL")
        return True
    logger.error("accelcal: no more retries")
    return False


def accel_calibrate_reference():
    '''run accelcal on reference board'''
    logger.info("STARTING REFERENCE ACCEL CALIBRATION")

    conn = connection.Connection(ref_only=True)

    logger.info("Turning safety off")
    rotate.set_rotation(conn, 'level', wait=False)
    util.safety_off(conn.refmav)

    conn.ref.send("accelcal\n")
    for rotation in ['level', 'left', 'right', 'up', 'down', 'back']:
        try:
            conn.ref.expect("Place vehicle")
            conn.ref.expect("and press any key")
        except Exception as ex:
            util.failure("Failed to get place vehicle message for %s" % rotation)
        logger.debug("Rotating %s" % rotation)
        attitude = rotate.set_rotation(conn, rotation, wait=False)
        time.sleep(13)
        conn.ref.send("\n")
    i = conn.ref.expect(["Calibration successful","Calibration FAILED"])
    if i != 0:
        util.failure("Accel calibration failed at %s" % time.ctime())
    logger.info("Calibration successful")
    rotate.set_rotation(conn, 'level', wait=False)
    util.param_set(conn.ref, 'AHRS_TRIM_X', 0)
    util.param_set(conn.ref, 'AHRS_TRIM_Y', 0)
    util.discard_messages(conn.refmav)
    util.wait_heartbeat(conn.refmav)

if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)

    parser.add_argument("--reference", action='store_true', default=False, help="calibrate reference board")
    args = parser.parse_args()

    if args.reference:
        accel_calibrate_reference()
    else:
        accel_calibrate_retries()

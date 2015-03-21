#!/usr/bin/env python
'''
rotation control of the test jig, based on a quaternion controller
'''

import pexpect, sys, time
from math import *
from config import *
import util
import logger
from pymavlink import mavutil
from pymavlink.rotmat import Matrix3, Vector3
from pymavlink.quaternion import Quaternion
import mav_reference
import connection
import random

def quat_division(quat, rquat):
    '''quaternion division'''
    quat0 = quat[0]
    quat1 = quat[1]
    quat2 = quat[2]
    quat3 = quat[3]

    rquat0 = rquat[0]
    rquat1 = rquat[1]
    rquat2 = rquat[2]
    rquat3 = rquat[3]

    q1 = (rquat0*quat0 + rquat1*quat1 + rquat2*quat2 + rquat3*quat3)
    q2 = (rquat0*quat1 - rquat1*quat0 - rquat2*quat3 + rquat3*quat2)
    q3 = (rquat0*quat2 + rquat1*quat3 - rquat2*quat0 - rquat3*quat1)
    q4 = (rquat0*quat3 - rquat1*quat2 + rquat2*quat1 - rquat3*quat0)
    return [q1, q2, q3, q4]

def gimbal_controller(dcm_estimated, dcm_demanded, chan1):
    '''return the delta to chan1 and chan2'''
    quat_est = Quaternion(dcm_estimated)
    quat_dem = Quaternion(dcm_demanded)

    quatErr = quat_division(quat_dem, quat_est)
    
    if quatErr[0] >= 0.0:
        scaler = 2.0
    else:
        scaler = -2.0
    bf_rates = scaler * Vector3(quatErr[1], quatErr[2], quatErr[3])

    yaw_joint_rad = radians((chan1 - ROTATIONS['level'].chan1) * YAW_SCALE)
    chan1_change = degrees(bf_rates.z) * YAW_SCALE
    chan2_change = degrees(bf_rates.x * sin(yaw_joint_rad) +
                           bf_rates.y * cos(yaw_joint_rad)) / PITCH_SCALE
    return (chan1_change, chan2_change)

def attitude_error(attitude, target_roll, target_pitch):
    '''return tuple with attitude error as (err_roll, err_pitch)'''
    roll = degrees(attitude.roll)
    pitch = degrees(attitude.pitch)

    if target_roll == 180 and roll < 0:
        roll += 360
    if target_roll is None:
        err_roll = 0
    else:
        err_roll = abs(roll - target_roll)
    err_pitch = abs(pitch - target_pitch)
    return (err_roll, err_pitch)


def wait_quiescent(mav, type='RAW_IMU'):
    '''wait for movement to stop'''
    t1 = time.time()
    util.discard_messages(mav)
    raw_imu = None
    while time.time() < t1+20:
        raw_imu = mav.recv_match(type=type, blocking=True, timeout=4)
        if raw_imu is None:
            util.failure("communication with board lost for %s" % type)
        if time.time() > t1+10:
            logger.debug("x not quiescent: %s" % abs(degrees(raw_imu.xgyro*0.001)))
            logger.debug("y not quiescent: %s" % abs(degrees(raw_imu.ygyro*0.001)))
            logger.debug("z not quiescent: %s" % abs(degrees(raw_imu.zgyro*0.001)))
        if (abs(degrees(raw_imu.xgyro*0.001)) < GYRO_TOLERANCE and
            abs(degrees(raw_imu.ygyro*0.001)) < GYRO_TOLERANCE and
            abs(degrees(raw_imu.zgyro*0.001)) < GYRO_TOLERANCE):
            logger.debug("x is  quiescent: %s" % abs(degrees(raw_imu.xgyro*0.001)))
            logger.debug("y is  quiescent: %s" % abs(degrees(raw_imu.ygyro*0.001)))
            logger.debug("z is  quiescent: %s" % abs(degrees(raw_imu.zgyro*0.001)))
            break
    if raw_imu is None:
        util.failure("Failed to reach quiescent state")
    attitude = mav.recv_match(type='ATTITUDE', blocking=True, timeout=3)
    if attitude is None:
        util.failure("Failed to receive ATTITUDE message")
    return attitude

def wait_quiescent_list(mav, types):
    '''wait for movement to stop on a list of gyros'''
    attitude = None
    for type in types:
        attitude = wait_quiescent(mav, type)
    return attitude

def optimise_attitude(conn, rotation, tolerance):
    '''optimise attitude using servo changes'''
    expected_roll = ROTATIONS[rotation].roll
    expected_pitch = ROTATIONS[rotation].pitch
    chan1 = ROTATIONS[rotation].chan1
    chan2 = ROTATIONS[rotation].chan2

    attitude = wait_quiescent(conn.refmav)
    time_start = time.time()

    # we always do at least 2 tries. This means the attitude accuracy
    # will tend to improve over time, while not adding excessive time
    # per board
    tries = 0
    
    while time.time() < time_start+25:
        dcm_estimated = Matrix3()
        dcm_estimated.from_euler(attitude.roll, attitude.pitch, attitude.yaw)
    
        droll = expected_roll
        if droll is None:
            droll = attitude.roll
        else:
            droll = radians(droll)
        dpitch = radians(expected_pitch)

        dcm_demanded = Matrix3()
        dcm_demanded.from_euler(droll, dpitch, attitude.yaw)

        (chan1_change, chan2_change) = gimbal_controller(dcm_estimated,
                                                         dcm_demanded, chan1)
        (err_roll, err_pitch) = attitude_error(attitude, expected_roll, expected_pitch)
        logger.debug("%s offsets: %.2f %.2f chan1=%u chan2=%u" % (rotation, err_roll, err_pitch, chan1, chan2))
        if (tries > 0 and (abs(err_roll)+abs(err_pitch) < tolerance or
                           (abs(chan1_change)<1 and abs(chan2_change)<1))):
            logger.info("%s converged %.2f %.2f tolerance %.1f at %s" % (rotation, err_roll, err_pitch, tolerance, time.ctime()))
            # update optimised rotations to save on convergence time for the next board
            ROTATIONS[rotation].chan1 = chan1
            ROTATIONS[rotation].chan2 = chan2
            return True
        chan1 += chan1_change
        chan2 += chan2_change
        if chan1 < 700 or chan1 > 2300 or chan2 < 700 or chan2 > 2300:
            logger.debug("servos out of range - failed")
            return False
        util.set_servo(conn.refmav, YAW_CHANNEL, chan1)
        util.set_servo(conn.refmav, PITCH_CHANNEL, chan2)
        attitude = wait_quiescent(conn.refmav)
        tries += 1
        
    logger.error("timed out rotating to %s" % rotation)
    return False

def set_rotation(conn, rotation, wait=True):
    '''set servo rotation'''
    if not rotation in ROTATIONS:
        util.failure("No rotation %s" % rotation)
    expected_roll = ROTATIONS[rotation].roll
    expected_pitch = ROTATIONS[rotation].pitch

    # start with initial settings from the table
    util.set_servo(conn.refmav, YAW_CHANNEL, ROTATIONS[rotation].chan1)
    util.set_servo(conn.refmav, PITCH_CHANNEL, ROTATIONS[rotation].chan2)
    if not wait:
        return conn.refmav.recv_match(type='ATTITUDE', blocking=True, timeout=3)

    time.sleep(1)
    util.discard_messages(conn.refmav)
    
    if expected_roll == 0 and expected_pitch == 0:
        tolerance = ROTATION_LEVEL_TOLERANCE
    else:
        tolerance = ROTATION_TOLERANCE

    # now optimise it
    if not optimise_attitude(conn, rotation, tolerance):
        util.failure("Failed to reach target attitude")
    return conn.refmav.recv_match(type='ATTITUDE', blocking=True, timeout=3)

def gyro_integrate(conn):
    '''test gyros by integrating while rotating to the given rotations'''
    conn.ref.send('set streamrate -1\n')
    conn.test.send('set streamrate -1\n')
    util.param_set(conn.ref, 'SR0_RAW_SENS', 20)
    util.param_set(conn.test, 'SR0_RAW_SENS', 20)

    logger.info("Starting gyro integration at %s" % time.ctime())
    wait_quiescent(conn.refmav)
    conn.discard_messages()

    util.set_servo(conn.refmav, YAW_CHANNEL, ROTATIONS['level'].chan1+200)
    util.set_servo(conn.refmav, PITCH_CHANNEL, ROTATIONS['level'].chan2+200)

    start_time = time.time()
    ref_tstart = None
    test_tstart = [None]*3
    ref_sum = Vector3()
    test_sum = [Vector3(), Vector3(), Vector3()]
    msgs = { 'RAW_IMU' : 0, 'SCALED_IMU2' : 1, 'SCALED_IMU3' : 2 }
    while time.time() < start_time+20:
        imu = conn.refmav.recv_match(type='RAW_IMU', blocking=False)
        if imu is not None:
            gyro = util.gyro_vector(imu)
            tnow = imu.time_usec*1.0e-6
            if ref_tstart is not None:
                deltat = tnow - ref_tstart
                ref_sum += gyro * deltat
            ref_tstart = tnow
            if time.time() - start_time > 2 and gyro.length() < GYRO_TOLERANCE:
                break
        imu = conn.testmav.recv_match(type=msgs.keys(), blocking=False)
        if imu is not None:
            idx = msgs[imu.get_type()]
            gyro = util.gyro_vector(imu)
            if imu.get_type().startswith("SCALED_IMU"):
                tnow = imu.time_boot_ms*1.0e-3
            else:
                tnow = imu.time_usec*1.0e-6
            if test_tstart[idx] is not None:
                deltat = tnow - test_tstart[idx]
                test_sum[idx] += gyro * deltat
            test_tstart[idx] = tnow
    logger.debug("Gyro ref  sums: %s" % ref_sum)
    logger.debug("Gyro test sum1: %s" % test_sum[0])
    logger.debug("Gyro test sum2: %s" % test_sum[1])
    logger.debug("Gyro test sum3: %s" % test_sum[2])
    for idx in range(3):
        err = test_sum[idx] - ref_sum
        if abs(err.x) > GYRO_SUM_TOLERANCE:
            util.failure("X gyro %u error: %.1f" % (idx, err.x))
        if abs(err.y) > GYRO_SUM_TOLERANCE:
            util.failure("Y gyro %u error: %.1f" % (idx, err.y))
        if abs(err.z) > GYRO_SUM_TOLERANCE:
            util.failure("Z gyro %u error: %.1f" % (idx, err.z))


def unjam_servos(conn):
    '''try to unjam servos with random movement'''
    logger.info("Starting unjamming")
    conn.discard_messages()

    rotations = ROTATIONS.keys()

    util.param_set(conn.ref, 'SR0_RAW_SENS', 10)

    last_change = time.time()
    while True:
        imu = conn.refmav.recv_match(type='RAW_IMU', blocking=False)
        if imu is not None:
            gyro = util.gyro_vector(imu)
            logger.info('Gyro: %s' % gyro)
            if abs(gyro.x) > 5 or abs(gyro.y) > 5:
                logger.info("Unjammed: ", gyro)
                break
        if time.time() > last_change+0.7:
            last_change = time.time()
            r1 = int(random.uniform(800, 2100))
            r2 = int(random.uniform(800, 2100))
            logger.debug("%s %s" % (r1, r2))
            util.set_servo(conn.refmav, YAW_CHANNEL, r1)
            util.set_servo(conn.refmav, PITCH_CHANNEL, r2)
            
if __name__ == '__main__':
    from argparse import ArgumentParser
    import power_control
    parser = ArgumentParser(description=__doc__)

    parser.add_argument("--tolerance", type=float, dest="tolerance", default=ROTATION_LEVEL_TOLERANCE,
                        help="rotation tolerance")
    parser.add_argument("--wait", dest="wait", action='store_true', default=False,
                        help="wait for completion")
    parser.add_argument("--yaw-zero", type=int, default=ROTATIONS['level'].chan1,
                        help="zero on yaw channel")
    parser.add_argument("--unjam", action='store_true', help="unjam servos")
    parser.add_argument("rotation", default="level", help="target rotation")
    args = parser.parse_args()

    ROTATION_LEVEL_TOLERANCE = args.tolerance
    ROTATION_TOLERANCE = args.tolerance
    ROTATIONS['level'].chan1 = args.yaw_zero

    power_control.power_cycle()

    conn = connection.Connection(ref_only=True)

    print("Turning safety off")
    util.safety_off(conn.refmav)

    if args.unjam:
        unjam_servos(conn)
        
    print("Rotating to %s" % args.rotation)
    set_rotation(conn, args.rotation, wait=args.wait)

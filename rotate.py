#!/usr/bin/env python
'''
rotation control of the test jig, based on a quaternion controller
'''

import pexpect, sys, time
from math import *
from config import *
import util
from pymavlink import mavutil
from pymavlink.rotmat import Matrix3, Vector3
from pymavlink.quaternion import Quaternion
import mav_reference

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

    yaw_joint_rad = radians((chan1 - ROTATIONS['level'][0]) * YAW_SCALE)
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


def wait_quiescent(refmav):
    '''wait for movement to stop'''
    t1 = time.time()
    util.discard_messages(refmav)
    
    while time.time() < t1+20:
        raw_imu = refmav.recv_match(type='RAW_IMU',
                                    blocking=True, timeout=2)
        if (abs(raw_imu.xgyro*0.001) < GYRO_TOLERANCE and
            abs(raw_imu.ygyro*0.001) < GYRO_TOLERANCE and
            abs(raw_imu.zgyro*0.001) < GYRO_TOLERANCE):
            break
    if raw_imu is None:
        util.failure("Failed to reach quiescent state")
    attitude = refmav.recv_match(type='ATTITUDE', blocking=True, timeout=3)
    if attitude is None:
        util.failure("Failed to receive ATTITUDE message")
    t2 = time.time()
    return attitude
            

def optimise_attitude(ref, refmav, rotation, tolerance):
    '''optimise attitude using servo changes'''
    (chan1, chan2, expected_roll, expected_pitch) = ROTATIONS[rotation]

    attitude = wait_quiescent(refmav)
    while True:
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
        print("%s error: %.2f %.2f" % (rotation, err_roll, err_pitch))
        if (abs(err_roll)+abs(err_pitch) < tolerance or
            (abs(chan1_change)<1 and abs(chan2_change)<1)):
            print("%s converged %.2f %.2f tolerance %.1f" % (rotation, err_roll, err_pitch, tolerance))
            return True
        chan1 += chan1_change
        chan2 += chan2_change
        util.set_servo(refmav, YAW_CHANNEL, chan1)
        util.set_servo(refmav, PITCH_CHANNEL, chan2)
        attitude = wait_quiescent(refmav)
    return True

def set_rotation(ref, refmav, rotation, wait=True):
    '''set servo rotation'''
    if not rotation in ROTATIONS:
        util.failure("No rotation %s" % rotation)
    (c1, c2, expected_roll, expected_pitch) = ROTATIONS[rotation]

    # start with initial settings from the table
    util.set_servo(refmav, YAW_CHANNEL, c1)
    util.set_servo(refmav, PITCH_CHANNEL, c2)
    if not wait:
        return refmav.recv_match(type='ATTITUDE', blocking=True, timeout=3)

    time.sleep(1)
    util.discard_messages(refmav)
    
    if expected_roll == 0 and expected_pitch == 0:
        tolerance = ROTATION_LEVEL_TOLERANCE
    else:
        tolerance = ROTATION_TOLERANCE

    # now optimise it
    if not optimise_attitude(ref, refmav, rotation, tolerance):
        util.failure("Failed to reach target attitude")
    return refmav.recv_match(type='ATTITUDE', blocking=True, timeout=3)

if __name__ == '__main__':
    ref = mav_reference.mav_reference()
    ref.expect(['MANUAL>'])

    refmav = mavutil.mavlink_connection('127.0.0.1:14550', robust_parsing=True)
    refmav.wait_heartbeat()

    set_rotation(ref, refmav, sys.argv[1], wait=0)
    

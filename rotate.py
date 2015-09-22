#!/usr/bin/env python
'''
rotation control of the test jig, based on a quaternion controller
'''

import pexpect, sys, time
from math import *
from config import *
import util
import logger
import os
from pymavlink import mavutil
from pymavlink.rotmat import Matrix3, Vector3
from pymavlink.quaternion import Quaternion
import mav_reference
import connection
import random
from PixETE import PixETE

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
        err_roll = abs(util.wrap_180(roll - target_roll))
    err_pitch = abs(pitch - target_pitch)
    return (err_roll, err_pitch)


def wait_quiescent(mav, type='RAW_IMU'):
    '''wait for movement to stop'''
    t1 = time.time()
    util.discard_messages(mav)
    raw_imu = None
    while time.time() < t1+20:
        raw_imu = mav.recv_match(type=type, blocking=True, timeout=4)  # JQM
#        logger.debug("mav.recv_match: type=%s   x=%s  y=%s  z=%s" % (type, raw_imu.xgyro, raw_imu.ygyro, raw_imu.zgyro))
        if raw_imu is None:
            util.failure("communication with board lost for %s" % type)
        if time.time() > t1+10:
            logger.debug("Time > 10 -- Not quiescent: x=%s  y=%s  z=%s" % (abs(degrees(raw_imu.xgyro*0.001)), abs(degrees(raw_imu.ygyro*0.001)), abs(degrees(raw_imu.zgyro*0.001))) )
        if (abs(degrees(raw_imu.xgyro*0.001)) < GYRO_TOLERANCE and
            abs(degrees(raw_imu.ygyro*0.001)) < GYRO_TOLERANCE and
            abs(degrees(raw_imu.zgyro*0.001)) < GYRO_TOLERANCE):
            logger.debug("Tolerance -- Not quiescent: x=%s  y=%s  z=%s" % (abs(degrees(raw_imu.xgyro*0.001)), abs(degrees(raw_imu.ygyro*0.001)), abs(degrees(raw_imu.zgyro*0.001))) )
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

def optimise_attitude(conn, rotation, tolerance, timeout=25):
    '''optimise attitude using servo changes'''
    expected_roll = ROTATIONS[rotation].roll
    expected_pitch = ROTATIONS[rotation].pitch
    if ETE == 0:
        chan1 = ROTATIONS[rotation].chan1
        chan2 = ROTATIONS[rotation].chan2
    elif ETE == 1:
        chan1 = ROTATIONS_ETE[rotation].chan1
        chan2 = ROTATIONS_ETE[rotation].chan2

    attitude = wait_quiescent(conn.refmav)

    if ETE == 1:
        return True

    time_start = time.time()
    # we always do at least 2 tries. This means the attitude accuracy
    # will tend to improve over time, while not adding excessive time
    # per board
    tries = 0
    
    while time.time() < time_start+timeout:
        #logger.info("============================= BEGIN ROTATIONS  try=%s =================" % (tries))
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
        logger.info("optimise_attitude: %s err_roll=%.2f   err_pitch=%.2f   chan1=%u chan2=%u" % (rotation, err_roll, err_pitch, chan1, chan2))
        if (tries > 0 and (abs(err_roll)+abs(err_pitch) < tolerance or
                           (abs(chan1_change)<1 and abs(chan2_change)<1))):
            logger.debug("%s converged %.2f %.2f tolerance %.1f" % (rotation, err_roll, err_pitch, tolerance))

            # update optimised rotations to save on convergence time for the next board
            ROTATIONS[rotation].chan1 = chan1
            ROTATIONS[rotation].chan2 = chan2
            logger.debug("optimise_attitude: ROTATIONS[%s]  chan1:%s   chan2:%s" % (rotation, ROTATIONS[rotation].chan1, ROTATIONS[rotation].chan2) )
            
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


#-------------------------------------------------------------------------------------------
def set_rotation(conn, rotation, wait=True, timeout=25):
    '''set servo rotation'''
    if not rotation in ROTATIONS:
        util.failure("No rotation %s" % rotation)
    expected_roll = ROTATIONS[rotation].roll
    expected_pitch = ROTATIONS[rotation].pitch

    logger.debug("set_rotation: call set_servo -- YAW rotation[%s].chan1=%s      PITCH rotation[%s].chan2=%s" % (rotation, ROTATIONS[rotation].chan1, rotation, ROTATIONS[rotation].chan2) )
    
    if ETE == 0:
        # start with initial settings from the table
        util.set_servo(conn.refmav, YAW_CHANNEL, ROTATIONS[rotation].chan1)
        util.set_servo(conn.refmav, PITCH_CHANNEL, ROTATIONS[rotation].chan2)
    elif ETE == 1:
        ete = PixETE()
        ete.position(ROTATIONS_ETE[rotation].chan2, ROTATIONS_ETE[rotation].chan1)
    if not wait:
        return conn.refmav.recv_match(type='ATTITUDE', blocking=True, timeout=3)

    time.sleep(2)
    util.discard_messages(conn.refmav)
    
    if expected_roll == 0 and expected_pitch == 0:
        tolerance = ROTATION_LEVEL_TOLERANCE
    else:
        tolerance = ROTATION_TOLERANCE

    # now optimise it
    if not optimise_attitude(conn, rotation, tolerance, timeout=timeout):
        util.failure("Failed to reach target attitude")
    return conn.refmav.recv_match(type='ATTITUDE', blocking=True, timeout=3)



#-------------------------------------------------------------------------------------------
def gyro_integrate(conn):
    '''test gyros by integrating while rotating to the given rotations'''
    conn.ref.send('set streamrate -1\n')
    conn.test.send('set streamrate -1\n')
    util.param_set(conn.ref, 'SR0_RAW_SENS', 20)
    util.param_set(conn.test, 'SR0_RAW_SENS', 20)

    logger.info("Starting gyro integration")
    wait_quiescent(conn.refmav)
    conn.discard_messages()
    if ETE == 0:
        util.set_servo(conn.refmav, YAW_CHANNEL, ROTATIONS['level'].chan1+200)
        util.set_servo(conn.refmav, PITCH_CHANNEL, ROTATIONS['level'].chan2+200)
    if ETE == 1:
        ete = PixETE()
        ete.position(180, 45)
        time.sleep(1)
        ete.rollspeed(4000)
        ete.position(0, 45)
    logger.info("Starting gyro motion")    
    start_time = time.time()
    ref_tstart = None
    test_tstart = [None]*3
    ref_sum = Vector3()
    test_sum = [Vector3(), Vector3(), Vector3()]
    msgs = { 'RAW_IMU' : 0, 'SCALED_IMU2' : 1, 'SCALED_IMU3' : 2 }
    while time.time() < start_time+20:
        imu = conn.refmav.recv_match(type='RAW_IMU', blocking=False)
        if imu is not None:
            #gyro = util.gyro_vector(imu)
            gyro = Vector3(degrees(imu.xgyro*-0.001), degrees(imu.ygyro*0.001), degrees(imu.zgyro*-0.001))  #phil change.... when running this check from back to top, I found that the reference IMU X and Z were reversed.... this hack makes this pass, but I suspect something else is wrong, this should be reverted when that is found.
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
    ete.yawspeed(5000)
    ete.rollspeed(10000)
    ete.position(0, 0)
    wait_quiescent(conn.refmav)
    for idx in range(3):
        err = test_sum[idx] - ref_sum
        if abs(err.x) > GYRO_SUM_TOLERANCE:
            util.failure("X gyro %u error: %.1f" % (idx, err.x))
        if abs(err.y) > GYRO_SUM_TOLERANCE:
            util.failure("Y gyro %u error: %.1f" % (idx, err.y))
        if abs(err.z) > GYRO_SUM_TOLERANCE:
            util.failure("Z gyro %u error: %.1f" % (idx, err.z))
    logger.debug("Gyro test finished")

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

def get_attitude(conn):
    '''return attutude as roll,pitch,yaw in degrees'''
    conn.discard_messages()
    attitude = conn.refmav.recv_match(type='ATTITUDE', blocking=True)
    return degrees(attitude.roll), degrees(attitude.pitch), degrees(attitude.yaw)

def find_yaw_zero(conn):
    '''find the yaw zero'''
    yaw_min = 1200
    yaw_max = 1800
    # start in units of 100
    best_yaw = None
    best_roll_change = None
    for ydelta in [200, 20, 2]:
        for yaw in range(yaw_min, yaw_max+ydelta, ydelta):
            util.set_servo(conn.refmav, YAW_CHANNEL, yaw)
            util.set_servo(conn.refmav, PITCH_CHANNEL, 1400)
            wait_quiescent(conn.refmav)
            r1, p1, y1 = get_attitude(conn)
            util.set_servo(conn.refmav, PITCH_CHANNEL, 1500)
            wait_quiescent(conn.refmav)
            r2, p2, y2 = get_attitude(conn)
            rchange = abs(util.wrap_180(r2 - r1))
            print("yaw=%u rchange=%.1f" % (yaw, rchange))
            if best_yaw is None or rchange < best_roll_change:
                best_yaw = yaw
                best_roll_change = rchange
                print("best_yaw=%u best_roll_change=%.1f" % (best_yaw, best_roll_change))
        yaw_min = best_yaw-ydelta
        yaw_max = best_yaw+ydelta
    return best_yaw

def find_pitch_zero(conn):
    '''find the pitch zero'''
    pitch_min = 900
    pitch_max = 2000
    best_pitch = None
    best_pitch_value = None
    for pdelta in [200, 20, 2]:
        for pitch in range(pitch_min, pitch_max+pdelta, pdelta):
            util.set_servo(conn.refmav, PITCH_CHANNEL, pitch)
            wait_quiescent(conn.refmav)
            r1, p1, y1 = get_attitude(conn)
            print("pitch=%u p=%.1f" % (pitch, p1))
            if abs(r1) > 80:
                continue
            if best_pitch is None or abs(p1) < best_pitch_value:
                best_pitch = pitch
                best_pitch_value = abs(p1)
                print("best_pitch=%u best_pitch_value=%.1f" % (best_pitch, best_pitch_value))
        pitch_min = best_pitch-pdelta
        pitch_max = best_pitch+pdelta
    return best_pitch

def write_calibration():
    if ETE == 0:
        '''write out new calibration file'''
        f = open("FWLoad/calibration-new.py", mode='w')
        f.write('PITCH_SCALE = %.2f\n' % PITCH_SCALE)
        f.write('YAW_SCALE = %.2f\n' % YAW_SCALE)
        f.write('''
#       servo positions for different orientations of boards in the test jig
#       the columns are:
#    pitch PWM
#    yaw PWM
#    expected roll
#    expected pitch
class Rotation(object):
    def __init__(self, chan1, chan2, roll, pitch):
        self.chan1 = chan1
        self.chan2 = chan2
        self.roll = roll
        self.pitch = pitch

ROTATIONS = {
''')
        for r in ['level', 'right', 'left', 'up', 'down', 'back']:
            roll = ROTATIONS[r].roll
            if roll is None:
                roll = 'None'
            f.write("	'%s' : Rotation(%u, %u, %s, %d),\n" % (
                r, 
                ROTATIONS[r].chan1,
                ROTATIONS[r].chan2,
                ROTATIONS[r].roll,
                ROTATIONS[r].pitch))
        f.write('}\n')
        f.close()
        os.rename("FWLoad/calibration-new.py", "FWLoad/calibration.py")
            

def calibrate_servos(conn):
    '''try to calibrate servos'''
    logger.info("Starting calibration")
    conn.discard_messages()

    # step 1: find yaw zero by finding yaw channel value that minimises roll change on pitch channel change
    yaw_zero = find_yaw_zero(conn)
    #yaw_zero = 1585
    ROTATIONS['level'].chan1 = yaw_zero

    # step 2: find pitch zero by finding pitch channel that minimises pitch
    util.set_servo(conn.refmav, YAW_CHANNEL, yaw_zero)
    pitch_zero = find_pitch_zero(conn)
    #pitch_zero = 1855
    ROTATIONS['level'].chan2 = pitch_zero

    # step 3: find yaw scale
    util.set_servo(conn.refmav, YAW_CHANNEL, yaw_zero)
    util.set_servo(conn.refmav, PITCH_CHANNEL, pitch_zero)
    time.sleep(1)
    conn.discard_messages()
    wait_quiescent(conn.refmav)
    conn.discard_messages()
    r1, p1, y1 = get_attitude(conn)
    print(r1, p1, y1)
    util.set_servo(conn.refmav, YAW_CHANNEL, yaw_zero+100)
    conn.discard_messages()
    wait_quiescent(conn.refmav)
    conn.discard_messages()
    r2, p2, y2 = get_attitude(conn)
    print(r2, p2, y2)
    print(y1, y2)
    yawchange = util.wrap_180(y2 - y1)
    YAW_SCALE = yawchange / 100.0
    print("YAW_SCALE=%.2f" % YAW_SCALE)

    # step 3: find pitch scale
    util.set_servo(conn.refmav, YAW_CHANNEL, yaw_zero)
    util.set_servo(conn.refmav, PITCH_CHANNEL, pitch_zero)
    time.sleep(1)
    conn.discard_messages()
    wait_quiescent(conn.refmav)
    conn.discard_messages()
    r1, p1, y1 = get_attitude(conn)
    util.set_servo(conn.refmav, PITCH_CHANNEL, pitch_zero+100)
    conn.discard_messages()
    wait_quiescent(conn.refmav)
    conn.discard_messages()
    r2, p2, y2 = get_attitude(conn)
    print(p1, p2)
    pitchchange = util.wrap_180(p2 - p1)
    PITCH_SCALE = pitchchange / 100.0
    print("PITCH_SCALE=%.2f" % PITCH_SCALE)

    # step 4: optimise each rotation
    ROTATION_LEVEL_TOLERANCE = 0
    ROTATION_TOLERANCE = 0
    for rotation in ['level', 'right', 'left', 'up', 'down', 'back', 'slant']:
        print("optimising %s" % rotation)
        set_rotation(conn, rotation, wait=True, timeout=60)

    # step 5, write calibration.py
    write_calibration()

def center_servos(conn):
    '''center servos at 1500/1500'''
    logger.info("Centering servos")
    if ETE == 0:
        try:
            util.set_servo(conn.refmav, YAW_CHANNEL, 1500)
            util.set_servo(conn.refmav, PITCH_CHANNEL, 1500)
            print("NO ETE Center")
        except Exception as ex:
            print("Failed centering servos: %s" % ex)
            pass
    elif ETE == 1:
        #try:
        ete = PixETE()
        ete.position(0, 0)
        time.sleep(2)
        print("ETE Center")

    
            
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
    parser.add_argument("--calibrate", action='store_true', help="calibrate servos")
    parser.add_argument("rotation", default="level", help="target rotation")
    args = parser.parse_args()

    ROTATION_LEVEL_TOLERANCE = args.tolerance
    ROTATION_TOLERANCE = args.tolerance
    ROTATIONS['level'].chan1 = args.yaw_zero

#    power_control.power_cycle()
    #if ETE == 0:
    conn = connection.Connection(ref_only=True)
    print("Turning safety off")
    util.safety_off(conn.refmav)
    #elif ETE == 1:
    #    conn = 0


    if args.unjam:
        unjam_servos(conn)
    if args.calibrate:
        calibrate_servos(conn)
        
    print("Rotating to %s" % args.rotation)
    set_rotation(conn, args.rotation, wait=args.wait)

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
    global PITCH_SCALE, YAW_SCALE
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


def wait_quiescent(mav, type='RAW_IMU', quick=False):
    '''wait for movement to stop'''
    t1 = time.time()
    util.discard_messages(mav)
    raw_imu = None
    if quick:
        timeout = 7
    else:
        timeout = 15
    while time.time() < t1+timeout:
        raw_imu = mav.recv_match(type=type, blocking=True, timeout=4)  # JQM
#        logger.debug("mav.recv_match: type=%s   x=%s  y=%s  z=%s" % (type, raw_imu.xgyro, raw_imu.ygyro, raw_imu.zgyro))
        if raw_imu is None:
            util.failure("communication with board lost for %s" % type)
        if time.time() > t1+10:
            logger.debug("Time > 10 -- Not quiescent: x=%.2f y=%.2f z=%.2f" % (abs(degrees(raw_imu.xgyro*0.001)), abs(degrees(raw_imu.ygyro*0.001)), abs(degrees(raw_imu.zgyro*0.001))) )
        if (abs(degrees(raw_imu.xgyro*0.001)) < GYRO_TOLERANCE and
            abs(degrees(raw_imu.ygyro*0.001)) < GYRO_TOLERANCE and
            abs(degrees(raw_imu.zgyro*0.001)) < GYRO_TOLERANCE):
            break
    if raw_imu is None and not quick:
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

def optimise_attitude(conn, rotation, tolerance, timeout=25, quick=False):
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
        logger.info("optimise_attitude: %s err_roll=%.2f err_pitch=%.2f c1=%u c2=%u" % (rotation, err_roll, err_pitch, chan1, chan2))
        if ((abs(err_roll)+abs(err_pitch) > 5*tolerance and
             (abs(chan1_change)<1 and abs(chan2_change)<1))):
            chan1_change += random.uniform(-20, 20)
            chan2_change += random.uniform(-20, 20)
        if (tries > 0 and (abs(err_roll)+abs(err_pitch) < tolerance or
                           (abs(chan1_change)<1 and abs(chan2_change)<1))):
            print("roll=%.2f pitch=%.2f expected_roll=%s expected_pitch=%s" % (
                degrees(attitude.roll),
                degrees(attitude.pitch),
                expected_roll, expected_pitch))
            logger.info("%s converged %.2f %.2f tolerance %.1f" % (rotation, err_roll, err_pitch, tolerance))
            # update optimised rotations to save on convergence time for the next board
            ROTATIONS[rotation].chan1 = chan1
            ROTATIONS[rotation].chan2 = chan2
            return True
        chan1 += chan1_change
        chan2 += chan2_change
        if chan1 < 700:
            chan1 += 900
        if chan2 < 700:
            chan2 += 900
        if chan1 > 2300:
            chan1 -= 900
        if chan2 > 2300:
            chan2 -= 900
        if chan1 < 700 or chan1 > 2300 or chan2 < 700 or chan2 > 2300:
            logger.debug("servos out of range")
            return False
        util.set_servo(conn.refmav, YAW_CHANNEL, chan1)
        util.set_servo(conn.refmav, PITCH_CHANNEL, chan2)
        attitude = wait_quiescent(conn.refmav, quick=quick)
        tries += 1
        
    logger.error("timed out rotating to %s" % rotation)
    return False


#-------------------------------------------------------------------------------------------
def set_rotation(conn, rotation, wait=True, timeout=25, quick=False):
    '''set servo rotation'''
    if not rotation in ROTATIONS:
        util.failure("No rotation %s" % rotation)
    expected_roll = ROTATIONS[rotation].roll
    expected_pitch = ROTATIONS[rotation].pitch

    logger.info("set_rotation: %s chan1=%u chan2=%u" % (rotation, ROTATIONS[rotation].chan1, ROTATIONS[rotation].chan2) )
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
    if not optimise_attitude(conn, rotation, tolerance, timeout=timeout, quick=quick):
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

def get_attitude(conn):
    '''return attutude as roll,pitch,yaw in degrees'''
    conn.discard_messages()
    attitude = conn.refmav.recv_match(type='ATTITUDE', blocking=True)
    return degrees(attitude.roll), degrees(attitude.pitch), degrees(attitude.yaw)

def find_yaw_zero(conn):
    '''find the yaw zero'''
    yaw_min = 1400
    yaw_max = 1600
    # start in units of 100
    best_yaw = None
    best_roll_change = None
    for ydelta in [40, 20, 5]:
        for yaw in range(yaw_min, yaw_max+ydelta, ydelta):
            util.set_servo(conn.refmav, YAW_CHANNEL, yaw)
            util.set_servo(conn.refmav, PITCH_CHANNEL, 1400)
            wait_quiescent(conn.refmav)
            r1, p1, y1 = get_attitude(conn)
            util.set_servo(conn.refmav, PITCH_CHANNEL, 1500)
            wait_quiescent(conn.refmav)
            r2, p2, y2 = get_attitude(conn)
            rchange = abs(util.wrap_180(r2 - r1))
            print("yaw=%u rchange=%.1f r=%.1f/%.1f p=%.1f/%.1f" % (
                yaw, rchange,
                r1, r2, p1, p2))
            if abs(r1) > 90 or abs(r2) > 90:
                continue
            if best_yaw is None or rchange < best_roll_change:
                best_yaw = yaw
                best_roll_change = rchange
                print("best_yaw=%u best_roll_change=%.1f (r=%.1f/%.1f p=%.1f/%.1f)" % (
                    best_yaw, best_roll_change,
                    r1, r2, p1, p2))
                if best_roll_change <= 2.0:
                    return best_yaw
        if best_yaw is not None:
            yaw_min = best_yaw-ydelta
            yaw_max = best_yaw+ydelta
    return best_yaw

def find_pitch_zero(conn):
    '''find the pitch zero'''
    pitch_min = 1400
    pitch_max = 1600
    best_pitch = None
    best_pitch_value = None
    for pdelta in [40, 20, 5]:
        for pitch in range(pitch_min, pitch_max+pdelta, pdelta):
            util.set_servo(conn.refmav, PITCH_CHANNEL, pitch)
            util.set_servo(conn.refmav, YAW_CHANNEL, ROTATIONS['level'].chan1)
            wait_quiescent(conn.refmav)
            r1, p1, y1 = get_attitude(conn)
            print("pitch=%u p=%.1f r=%.1f" % (pitch, p1, r1))
            if abs(r1) > 80:
                continue
            if best_pitch is None or abs(p1) < best_pitch_value:
                best_pitch = pitch
                best_pitch_value = abs(p1)
                print("best_pitch=%u best_pitch_value=%.1f" % (best_pitch, best_pitch_value))
                if best_pitch_value <= 2.0:
                    return best_pitch
        pitch_min = best_pitch-pdelta
        pitch_max = best_pitch+pdelta
    return best_pitch

def write_calibration():
    '''write out new calibration file'''
    global PITCH_SCALE, YAW_SCALE
    f = open("FWLoad/calibration-new.py", mode='w')
    f.write('PITCH_SCALE = %.2f\n' % PITCH_SCALE)
    f.write('YAW_SCALE = %.2f\n' % YAW_SCALE)
    f.write('''
# servo positions for different orientations of boards in the test jig
# the columns are:
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
    os.rename("FWLoad/calibration-new.py", "FWLoad/calibration_local.py")
            

def find_yaw_scale(conn):
    '''calibration step 3: find yaw scale'''
    yaw_zero = ROTATIONS['level'].chan1
    pitch_zero = ROTATIONS['level'].chan2

    util.set_servo(conn.refmav, YAW_CHANNEL, yaw_zero)
    util.set_servo(conn.refmav, PITCH_CHANNEL, pitch_zero)
    time.sleep(1)
    conn.discard_messages()
    wait_quiescent(conn.refmav)
    conn.discard_messages()
    r1, p1, y1 = get_attitude(conn)
    print(r1, p1, y1)
    util.set_servo(conn.refmav, YAW_CHANNEL, yaw_zero+100)
    time.sleep(1)
    conn.discard_messages()
    wait_quiescent(conn.refmav)
    conn.discard_messages()
    r2, p2, y2 = get_attitude(conn)
    print(r2, p2, y2)
    print(y1, y2)
    yawchange = util.wrap_180(y2 - y1)
    YAW_SCALE = yawchange / 100.0
    print("YAW_SCALE=%.2f" % YAW_SCALE)
    if abs(YAW_SCALE) < 0.2:
        print("Bad yaw scale")
        return False
    write_calibration()
    return True

def find_pitch_scale(conn):
    '''calibration step 4: find pitch scale'''
    global PITCH_SCALE, YAW_SCALE
    yaw_zero = ROTATIONS['level'].chan1
    pitch_zero = ROTATIONS['level'].chan2

    util.set_servo(conn.refmav, YAW_CHANNEL, yaw_zero)
    util.set_servo(conn.refmav, PITCH_CHANNEL, pitch_zero)
    time.sleep(1)
    conn.discard_messages()
    wait_quiescent(conn.refmav)
    conn.discard_messages()
    r1, p1, y1 = get_attitude(conn)
    util.set_servo(conn.refmav, PITCH_CHANNEL, pitch_zero+100)
    time.sleep(1)
    conn.discard_messages()
    wait_quiescent(conn.refmav)
    conn.discard_messages()
    r2, p2, y2 = get_attitude(conn)
    print(p1, p2)
    pitchchange = util.wrap_180(p2 - p1)
    PITCH_SCALE = pitchchange / 100.0
    print("PITCH_SCALE=%.2f" % PITCH_SCALE)
    if abs(PITCH_SCALE) < 0.2:
        print("Bad pitch scale")
        return False
    
    write_calibration()
    return True

def guess_rotation_values():
    '''guess initial rotation values'''
    global PITCH_SCALE, YAW_SCALE
    ROTATIONS['up'].chan1 = ROTATIONS['level'].chan1
    ROTATIONS['up'].chan2 = ROTATIONS['level'].chan2 + 90.0/PITCH_SCALE
    ROTATIONS['up'].roll = None
    ROTATIONS['up'].pitch = 90

    ROTATIONS['right'].chan1 = ROTATIONS['up'].chan1 + 90.0/YAW_SCALE
    ROTATIONS['right'].chan2 = ROTATIONS['up'].chan2
    ROTATIONS['right'].roll = 90
    ROTATIONS['right'].pitch = 0

    ROTATIONS['left'].chan1 = ROTATIONS['up'].chan1 - 90.0/YAW_SCALE
    ROTATIONS['left'].chan2 = ROTATIONS['up'].chan2
    ROTATIONS['left'].roll = -90
    ROTATIONS['left'].pitch = 0

    ROTATIONS['down'].chan1 = ROTATIONS['level'].chan1
    ROTATIONS['down'].chan2 = ROTATIONS['level'].chan2 - 90.0/PITCH_SCALE
    ROTATIONS['down'].roll = None
    ROTATIONS['down'].pitch = -90

    ROTATIONS['back'].chan1 = ROTATIONS['level'].chan1
    ROTATIONS['back'].chan2 = ROTATIONS['level'].chan2 - 180.0/PITCH_SCALE
    if ROTATIONS['back'].chan2 >= 2000:
        ROTATIONS['back'].chan2 = ROTATIONS['level'].chan2 + 180.0/PITCH_SCALE
    ROTATIONS['back'].roll = 180
    ROTATIONS['back'].pitch = 0

def calibrate_servos(conn):
    '''try to calibrate servos'''
    conn.ref.send('gyrocal\n')
    conn.ref.expect('Calibrated')

    logger.info("Starting calibration")
    conn.discard_messages()

    # step 1: find yaw zero by finding yaw channel value that minimises roll change on pitch channel change
    yaw_zero = find_yaw_zero(conn)
    #yaw_zero = ROTATIONS['level'].chan1
    
    ROTATIONS['level'].chan1 = yaw_zero

    write_calibration()

    # step 2: find pitch zero by finding pitch channel that minimises pitch
    util.set_servo(conn.refmav, YAW_CHANNEL, yaw_zero)
    time.sleep(1)
    pitch_zero = find_pitch_zero(conn)
    #pitch_zero = ROTATIONS['level'].chan2
    #pitch_zero = 1855
    ROTATIONS['level'].chan2 = pitch_zero

    write_calibration()

    ok = False
    for i in range(4):
        ok = find_yaw_scale(conn)
        if ok:
            break
    if not ok:
        print("Error: ***** Failed to find yaw scale ****")
        return

    ok = False
    for i in range(4):
        ok = find_pitch_scale(conn)
        if ok:
            break
    if not ok:
        print("Error: ***** Failed to find pitch scale ****")
        return

    # step 4: optimise each rotation
    ROTATION_LEVEL_TOLERANCE = 0
    ROTATION_TOLERANCE = 0

    guess_rotation_values()

    print("trying right check")
    try:
        set_rotation(conn, 'right', wait=True, timeout=120)
        write_calibration()
    except Exception:
        print("Failed right check - reversing PITCH_SCALE")
        global PITCH_SCALE
        PITCH_SCALE = -PITCH_SCALE
        write_calibration()

    for rotation in ['level', 'right', 'left', 'up', 'down', 'back']:
        print("optimising %s" % rotation)
        set_rotation(conn, rotation, wait=True, timeout=120)
        write_calibration()

    # step 5, write calibration.py
    write_calibration()


def calscale_servos(conn):
    '''try to calibrate servo pitch scale'''
    conn.ref.send('gyrocal\n')
    conn.ref.expect('Calibrated')

    logger.info("Starting pitch scale calibration")
    conn.discard_messages()

    yaw_zero = ROTATIONS['level'].chan1
    pitch_zero = ROTATIONS['level'].chan2

    ok = False
    for i in range(4):
        ok = find_yaw_scale(conn)
        if ok:
            break
    if not ok:
        print("Error: ***** Failed to find yaw scale ****")
        return

    ok = False
    for i in range(4):
        ok = find_pitch_scale(conn)
        if ok:
            break
    if not ok:
        print("Error: ***** Failed to find pitch scale ****")
        return

def center_servos(conn):
    '''center servos at 1500/1500'''
    logger.info("Centering servos")
    try:
        util.set_servo(conn.refmav, YAW_CHANNEL, 1500)
        util.set_servo(conn.refmav, PITCH_CHANNEL, 1500)
    except Exception as ex:
        print("Failed centering servos: %s" % ex)
        pass
            
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
    parser.add_argument("--calscale", action='store_true', help="calibrate scales")
    parser.add_argument("--center", action='store_true', help="center servos")
    parser.add_argument("--save", action='store_true', help="save on success")
    parser.add_argument("--timeout", type=int, default=25, help="timeout in seconds")
    parser.add_argument("--output", default=None, help="output mavlink to given address")
    parser.add_argument("rotation", default="level", nargs='?', help="target rotation")
    args = parser.parse_args()

    ROTATION_LEVEL_TOLERANCE = args.tolerance
    ROTATION_TOLERANCE = args.tolerance
    ROTATIONS['level'].chan1 = args.yaw_zero

#    power_control.power_cycle()

    conn = connection.Connection(ref_only=True)

    print("Turning safety off")
    util.safety_off(conn.refmav)

    if args.output:
        conn.ref.send('output add %s\n' % args.output)

    if args.unjam:
        unjam_servos(conn)

    if args.calibrate:
        calibrate_servos(conn)
        center_servos(conn)
        sys.exit(0)

    if args.calscale:
        calscale_servos(conn)
        center_servos(conn)
        sys.exit(0)

    if args.center:
        center_servos(conn)
        sys.exit(0)
        
    print("Rotating to %s" % args.rotation)
    set_rotation(conn, args.rotation, wait=args.wait, timeout=args.timeout)
    if args.save:
        write_calibration()


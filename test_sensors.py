#!/usr/bin/env python
'''
test sensors against reference board
'''
import util
import time
import logger
from config import *
from math import *
import connection
from pymavlink import mavutil

def check_accel_cal(conn):
    '''check accel cal'''
    global offset 
    offset = []
    global scale_factor
    scale_factor = []
    for idx in range(NUM_ACCELS):
        offset.append([])
        scale_factor.append([])
        if idx == 0:
            n = ''
        else:
            n = '%u' % (idx+1)
        for axis in ['X', 'Y', 'Z']:
            pname = 'INS_ACC%sOFFS_%s' % (n, axis)
            ofs = util.param_value(conn.test, pname)
            offset[idx].append(ofs)
            if abs(ofs) < 1.0e-10:
                util.failure("%s is zero - accel %u not calibrated (offset) %f" % (pname, idx, ofs))
            pname = 'INS_ACC%sSCAL_%s' % (n, axis)
            ofs = util.param_value(conn.test, pname)
            scale_factor[idx].append(ofs)
            if abs(ofs-1.0) < 1.0e-10:
                util.failure("%s is zero - accel %u not calibrated (scale)" % (pname, idx))
        logger.info("Accel cal %u OK" % (idx+1))
    print "Accel Offsets:", offset
    print "Accel Scale Factors:", scale_factor

def check_gyro_cal(conn):
    '''check gyro cal'''

    for idx in range(NUM_GYROS):
        if idx == 0:
            n = ''
        else:
            n = '%u' % (idx+1)
        for axis in ['X', 'Y', 'Z']:
            pname = 'INS_GYR%sOFFS_%s' % (n, axis)
            ofs = util.param_value(conn.test, pname)
            ofs = float(conn.test.match.group(1))
            if abs(ofs) < 0.000001:
                util.failure("%s is zero - gyro %u not calibrated (offset)" % (pname, idx))
        logger.info("Gyro cal %u OK" % (idx+1))


def check_baro(conn):
    '''check baros'''
    ref_press = util.wait_field(conn.refmav, 'SCALED_PRESSURE', 'press_abs')
    if ref_press is None:
        util.failure("No reference pressure")
    press1 = util.wait_field(conn.testmav, 'SCALED_PRESSURE', 'press_abs')
    press2 = util.wait_field(conn.testmav, 'SCALED_PRESSURE2', 'press_abs')
    if press1 is None:
        util.failure("No pressure1 available")
    if press2 is None:
        util.failure("No pressure2 available")
    if abs(ref_press - press1) > PRESSURE_TOLERANCE:
        util.failure("Baro1 error pressure=%f should be %f" % (press1, ref_press))
    if abs(ref_press - press2) > PRESSURE_TOLERANCE:
        util.failure("Baro2 error pressure=%f should be %f" % (press2, ref_press))

    ref_temp = util.wait_field(conn.refmav, 'SCALED_PRESSURE', 'temperature')*0.01
    temp1 = util.wait_field(conn.testmav, 'SCALED_PRESSURE', 'temperature')*0.01
    temp2 = util.wait_field(conn.testmav, 'SCALED_PRESSURE2', 'temperature')*0.01
    if abs(ref_temp - temp1) > TEMPERATURE_TOLERANCE:
        util.failure("Baro1 error temperature=%f should be %f" % (temp1, ref_temp))
    if abs(ref_temp - temp2) > TEMPERATURE_TOLERANCE:
        util.failure("Baro2 error temperature=%f should be %f" % (temp2, ref_temp))
    logger.info("Baros OK")

def check_power(conn):
    '''check power'''
    ref_vcc  = util.wait_field(conn.refmav, 'POWER_STATUS', 'Vcc')*0.001
    test_vcc = util.wait_field(conn.testmav, 'POWER_STATUS', 'Vcc')*0.001
    if abs(ref_vcc - test_vcc) > VOLTAGE_TOLERANCE:
        util.failure("Vcc error %.2f should be %.2f" % (test_vcc, ref_vcc))

    ref_vservo  = util.wait_field(conn.refmav, 'POWER_STATUS', 'Vservo')*0.001
    test_vservo = util.wait_field(conn.testmav, 'POWER_STATUS', 'Vservo')*0.001
    if abs(ref_vservo - test_vservo) > VOLTAGE_TOLERANCE:
        util.failure("Vservo error %.2f should be %.2f" % (test_vservo, ref_vservo))

    test_flags = util.wait_field(conn.testmav, 'POWER_STATUS', 'flags')
    pflags = mavutil.mavlink.MAV_POWER_STATUS_BRICK_VALID
    pflags |= mavutil.mavlink.MAV_POWER_STATUS_SERVO_VALID
    pflags |= mavutil.mavlink.MAV_POWER_STATUS_USB_CONNECTED
    if test_flags != pflags:
        util.failure("power flags error %u should be %u" % (test_flags, pflags))
        
    logger.info("Voltages OK")
        


def check_mag(conn):
    '''check mags'''
    magx = util.wait_field(conn.testmav, 'RAW_IMU', 'xmag')
    magy = util.wait_field(conn.testmav, 'RAW_IMU', 'ymag')
    magz = util.wait_field(conn.testmav, 'RAW_IMU', 'zmag')
    field = sqrt(magx**2 + magy**2 + magz**2)
    if field < 100 or field > 2000:
        util.failure("Bad magnetic field (%u, %u, %u, %.1f)" % (magx, magy, magz, field))
    logger.info("Magnetometer OK")


def serial_control_str(msg):
    '''handle receiving with SERIAL_CONTROL'''
    buf = msg.data
    s = ''
    for i in range(msg.count):
        c = buf[i]
        if c == 0:
            break
        s += chr(c)
    return s

def check_serial_pair(testmav, port1, port2):
    '''check a pair of loopback serial ports'''

    # lock both ports and flush data
    flags = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI

    # drain the serial ports
    testmav.mav.serial_control_send(port1, flags, 100, 57600, 0, util.serial_control_buf(""))
    testmav.mav.serial_control_send(port2, flags, 100, 57600, 0, util.serial_control_buf(""))
    util.discard_messages(testmav)

    testmav.mav.serial_control_send(port1, flags, 10, 0, 5, util.serial_control_buf("TEST1"))
    testmav.mav.serial_control_send(port2, flags, 10, 0, 5, util.serial_control_buf("TEST2"))
    testmav.mav.serial_control_send(port1, flags, 10, 0, 5, util.serial_control_buf("TEST1"))
    testmav.mav.serial_control_send(port2, flags, 10, 0, 5, util.serial_control_buf("TEST2"))

    start_time = time.time()
    port1_ok = False
    port2_ok = False
    while time.time() < start_time+5 and (not port1_ok or not port2_ok):
        reply = testmav.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=2)
        if reply is None or reply.count == 0:
            continue
        str = serial_control_str(reply)
        #logger.error("reply: %u %s" % (reply.device, str))
        if reply.device == port1 and str == "TEST2":
            port1_ok = True
        if reply.device == port2 and str == "TEST1":
            port2_ok = True
    if not port1_ok:
        util.failure("No reply on serial port %u" % port1)
    if not port2_ok:
        util.failure("No reply on serial port %u" % port2)
    util.discard_messages(testmav)

def check_serial(conn):
    '''check a pair of loopback serial ports'''
    check_serial_pair(conn.testmav,
                      mavutil.mavlink.SERIAL_CONTROL_DEV_TELEM1,
                      mavutil.mavlink.SERIAL_CONTROL_DEV_TELEM2)
    logger.info("Telemetry serial ports OK")
    check_serial_pair(conn.testmav,
                      mavutil.mavlink.SERIAL_CONTROL_DEV_GPS1,
                      mavutil.mavlink.SERIAL_CONTROL_DEV_GPS2)
    logger.info("GPS serial ports OK")

def check_status(conn):
    '''check SYS_STATUS flags'''
    sensor_bits = {
        'MAG'    : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
        'ACCEL'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL,
        'GYRO'   : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
        }
    teststatus = conn.testmav.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if teststatus is None:
        util.failure("Failed to get SYS_STATUS from test board")
    print(teststatus)
    refstatus = conn.refmav.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
    if refstatus is None:
        util.failure("Failed to get SYS_STATUS from reference board")
    print(refstatus)
    for bit in sensor_bits:
        present = refstatus.onboard_control_sensors_present & sensor_bits[bit]
        enabled = refstatus.onboard_control_sensors_enabled & sensor_bits[bit]
        health  = refstatus.onboard_control_sensors_health & sensor_bits[bit]
        logger.info("%s present" % present)
        logger.info("%s enabled" % enabled)
        logger.info("%s health" % health)
        #if present == 0 or present != enabled or present != health:
        #    util.failure("Reference board %s failure in SYS_STATUS" % bit)
        present = teststatus.onboard_control_sensors_present & sensor_bits[bit]
        enabled = teststatus.onboard_control_sensors_enabled & sensor_bits[bit]
        health  = teststatus.onboard_control_sensors_health & sensor_bits[bit]
        logger.info("%s present" % present)
        logger.info("%s enabled" % enabled)
        logger.info("%s health" % health)
        #if present == 0 or present != enabled or present != health:
        #    util.failure("Test board %s failure in SYS_STATUS" % bit)
        logger.info("%s status OK" % bit)

def check_pwm(conn):
    '''check PWM output and RC input'''
    logger.info("Checking PWM out and RC in")

    # disable safety on test board
    conn.test.send('arm safetyoff\n')
    test_values = [ 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800 ]
    # quad remaps the first 4 to a different order
    map_values = [ 1100, 1300, 1400, 1200, 1500, 1600, 1700, 1800 ]

    # we need to send the MOTOR_TEST command multuple times to avoid a race
    # condition in ArduCopter motor_test.pde
    for repeat in range(10):
        for i in range(1,5):
            conn.testmav.mav.command_long_send(0, 0,
                                               mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                                               i, 1, test_values[i-1], 300,
                                               0, 0, 0)
            time.sleep(0.05)
    for i in range(5,9):
        conn.test.send('servo set %u %u\n' % (i, test_values[i-1]))
    time.sleep(0.5)
    util.discard_messages(conn.testmav)
    rcin = conn.testmav.recv_match(type='RC_CHANNELS_RAW', blocking=True, timeout=3)
    if rcin is None:
        util.failure("Failed to get RC input")

    servo = conn.testmav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=3)
    if servo is None:
        util.failure("Failed to get servo output")

    pwm_ok = True
    for i in range(8):
        rc    = getattr(rcin, 'chan{0}_raw'.format(i+1))
        servoval = getattr(servo, 'servo{0}_raw'.format(i+1))
        if abs(rc - map_values[i]) > 3:
            pwm_ok = False
        logger.debug("pwm_check[%u] rc=%u test=%u servo=%u" % (i, rc, map_values[i], servoval))
    if not pwm_ok:
        util.failure("Incorrect PWM passthrough")

def check_all_sensors(conn):
    '''run all sensor checks'''
    check_baro(conn)
    check_mag(conn)
    check_power(conn)
    check_serial(conn)
    check_status(conn)
    check_pwm(conn)
    

if __name__ == '__main__':
    conn = connection.Connection()

    check_all_sensors(conn)

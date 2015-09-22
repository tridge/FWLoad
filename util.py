import sys, time, os
from config import *
from pymavlink import mavutil
from math import *
import logger

class FirmwareLoadError(Exception):
    '''firmload exception class'''
    def __init__(self, msg):
        Exception.__init__(self, msg)
        self.message = msg

def show_tail(logstr):
    '''show last lines of a log'''
    ofs = logstr.tell()
    logstr.seek(0)
    lines = logstr.readlines()
    n = 20
    if len(lines) < n:
        n = len(lines)
    for i in range(n):
        logger.debug(lines[len(lines)-n+i].rstrip())
    logstr.seek(ofs)

def show_error(test, ex, logstr=None):
    '''display an error then raise an exception'''
    if logstr is not None:
        show_tail(logstr)
    raise(FirmwareLoadError("FAILED: %s (%s)" % (test, ex)))

def wait_devices(devices, timeout=15):
    '''wait for devices to appear'''
    start_time = time.time()
    missing = []
    while start_time+timeout >= time.time():
        missing = []
        all_exist = True
        for dev in devices:
            if not os.path.exists(dev):
                all_exist = False
                missing.append(dev)
        if all_exist:
            return True
        time.sleep(0.1)
    return False

def wait_no_device(devices, timeout=10):
    '''wait for devices to disappear'''
    start_time = time.time()
    while start_time+timeout >= time.time():
        found_device = False
        for dev in devices:
            if os.path.exists(dev):
                found_device = True
        if not found_device:
            return True
        time.sleep(0.1)
    return False


def failure(msg):
    '''show a failure msg and raise an exception'''
    logger.error(msg)
    raise(FirmwareLoadError(msg))

def wait_field(refmav, msg_type, field):
    '''wait for a field value'''
    msg = None
    # get the latest available msg
    while True:
        msg2 = refmav.recv_match(type=msg_type, blocking=(msg==None), timeout=5)
        if msg2 is None:
            break
        msg = msg2
    if msg is None:
        failure("failed to reveive message %s" % msg_type)
    return getattr(msg, field)

def param_value(test, pname):
    '''get a param value given a mavproxy connection'''
    test.send('param fetch %s\n' % pname)
    try:
        test.expect('%s\s+=\s+(-?\d+\.\d+)\r\n' % pname, timeout=3)
    except Exception as ex:
        return None
    return float(test.match.group(1))

def param_set(test, pname, value):
    '''get a param value given a mavproxy connection'''
    test.send('param set %s %f\n' % (pname, value))
    test.expect('>')

def set_servo(mav, servo, value):
    '''set a servo to a value'''
    mav.mav.command_long_send(0, 0,
                              mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                              servo, value,
                              0, 0, 0, 0, 0)

def factory_test (test):
    '''send a command to set jig mode'''
    test.send('factory_test start\n')


def wrap_180(angle):
    while angle > 180:
        angle -= 360.0
    while angle < -180:
        angle += 360.0
    return angle

def discard_messages(mav):
    '''discard any buffered messages'''
    while True:
        msg = mav.recv_msg()
        if msg is None:
            return

def roll_estimate(RAW_IMU):
    '''estimate roll from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    return degrees(-asin(ry/sqrt(rx**2+ry**2+rz**2)))

def pitch_estimate(RAW_IMU):
    '''estimate pitch from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    return degrees(asin(rx/sqrt(rx**2+ry**2+rz**2)))

def attitude_estimate(RAW_IMU):
    '''return roll/pitch estimate as tuple'''
    return (roll_estimate(RAW_IMU), pitch_estimate(RAW_IMU))

def kill_processes(process_list):
    '''kill some processes by name'''
    from subprocess import call
    for p in process_list:
        call(['/usr/bin/pkill', '-9', '-f', p])
    reap_children()

def wait_heartbeat(mav, timeout=10):
    '''wait for a heartbeat'''
    start_time = time.time()
    while time.time() < start_time+timeout:
        if mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5) is not None:
            return
    failure("Failed to get heartbeat")    

def safety_off(mav):
    '''turn off safety switch'''
    mav.mav.set_mode_send(0, mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, 0)

def wait_mode(mav, modes, timeout=10):
    '''wait for one of a set of flight modes'''
    start_time = time.time()
    last_mode = None
    while time.time() < start_time+timeout:
        wait_heartbeat(mav, timeout=10)
        if mav.flightmode != last_mode:
            logger.info("Flightmode %s" % mav.flightmode)
            last_mode = mav.flightmode
        if mav.flightmode in modes:
            return
    failure("Failed to get mode from %s" % modes)

def gyro_vector(raw_imu):
    '''return a gyro vector in degrees/sec from a raw_imu message'''
    from pymavlink.rotmat import Vector3
    return Vector3(degrees(raw_imu.xgyro*0.001),
                   degrees(raw_imu.ygyro*0.001),
                   degrees(raw_imu.zgyro*0.001))

def serial_control_buf(str):
    '''format for sending with SERIAL_CONTROL'''
    buf = [ord(x) for x in str]
    buf.extend([0]*(70-len(buf)))
    return buf

def lock_serial_port(testmav, port):
    '''lock a serial port'''
    flags = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
    testmav.mav.serial_control_send(port, flags, 0, 0, 0, serial_control_buf(""))
    

def reap_children():
    '''reap any child processes'''
    try:
        while True:
            os.waitpid(-1, os.WNOHANG)
    except OSError:
        pass

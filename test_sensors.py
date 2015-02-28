#!/usr/bin/env python
'''
test sensors against reference board
'''
import util
from config import *
from math import *

def check_accel_cal(ref, refmav, test, testmav):
    '''check accel cal'''

    for idx in range(NUM_ACCELS):
        if idx == 0:
            n = ''
        else:
            n = '%u' % (idx+1)
        for axis in ['X', 'Y', 'Z']:
            pname = 'INS_ACC%sOFFS_%s' % (n, axis)
            ofs = util.param_value(test, pname)
            if abs(ofs) < 0.000001:
                util.failure("%s is zero - accel %u not calibrated (offset)" % (pname, idx))
            pname = 'INS_ACC%sSCAL_%s' % (n, axis)
            ofs = util.param_value(test, pname)
            if abs(ofs-1.0) < 0.000001:
                util.failure("%s is zero - accel %u not calibrated (scale)" % (pname, idx))
        print("Accel cal %u OK" % (idx+1))

def check_gyro_cal(ref, refmav, test, testmav):
    '''check gyro cal'''

    for idx in range(NUM_GYROS):
        if idx == 0:
            n = ''
        else:
            n = '%u' % (idx+1)
        for axis in ['X', 'Y', 'Z']:
            pname = 'INS_GYR%sOFFS_%s' % (n, axis)
            ofs = util.param_value(test, pname)
            ofs = float(test.match.group(1))
            if abs(ofs) < 0.000001:
                util.failure("%s is zero - gyro %u not calibrated (offset)" % (pname, idx))
        print("Gyro cal %u OK" % (idx+1))


def check_baro(ref, refmav, test, testmav):
    '''check baros'''
    ref_press = util.wait_field(refmav, 'SCALED_PRESSURE', 'press_abs')
    press1 = util.wait_field(testmav, 'SCALED_PRESSURE', 'press_abs')
    press2 = util.wait_field(testmav, 'SCALED_PRESSURE2', 'press_abs')
    if abs(ref_press - press1) > PRESSURE_TOLERANCE:
        util.failure("Baro1 error pressure=%f should be %f" % (press1, ref_press))
    if abs(ref_press - press2) > PRESSURE_TOLERANCE:
        util.failure("Baro2 error pressure=%f should be %f" % (press2, ref_press))

    ref_temp = util.wait_field(refmav, 'SCALED_PRESSURE', 'temperature')*0.01
    temp1 = util.wait_field(testmav, 'SCALED_PRESSURE', 'temperature')*0.01
    temp2 = util.wait_field(testmav, 'SCALED_PRESSURE2', 'temperature')*0.01
    if abs(ref_temp - temp1) > TEMPERATURE_TOLERANCE:
        util.failure("Baro1 error temperature=%f should be %f" % (temp1, ref_temp))
    if abs(ref_temp - temp2) > TEMPERATURE_TOLERANCE:
        util.failure("Baro2 error temperature=%f should be %f" % (temp2, ref_temp))
    print("Baros OK")

def check_power(ref, refmav, test, testmav):
    '''check power'''
    ref_vcc  = util.wait_field(refmav, 'POWER_STATUS', 'Vcc')*0.001
    test_vcc = util.wait_field(testmav, 'POWER_STATUS', 'Vcc')*0.001
    if abs(ref_vcc - test_vcc) > VOLTAGE_TOLERANCE:
        util.failure("Vcc error %.2f should be %.2f" % (test_vcc, ref_vcc))

    ref_vservo  = util.wait_field(refmav, 'POWER_STATUS', 'Vservo')*0.001
    test_vservo = util.wait_field(testmav, 'POWER_STATUS', 'Vservo')*0.001
    if abs(ref_vservo - test_vservo) > VOLTAGE_TOLERANCE:
        util.failure("Vservo error %.2f should be %.2f" % (test_vservo, ref_vservo))

    ref_flags  = util.wait_field(refmav, 'POWER_STATUS', 'flags')
    test_flags = util.wait_field(testmav, 'POWER_STATUS', 'flags')
    if ref_flags != test_flags:
        util.failure("power flags error %u should be %u" % (ref_flags, test_flags))
        
    print("Voltages OK")
        


def check_mag(ref, refmav, test, testmav):
    '''check mags'''
    magx = util.wait_field(testmav, 'RAW_IMU', 'xmag')
    magy = util.wait_field(testmav, 'RAW_IMU', 'ymag')
    magz = util.wait_field(testmav, 'RAW_IMU', 'zmag')
    field = sqrt(magx**2 + magy**2 + magz**2)
    if field < 100 or field > 1000:
        print("Bad magnetic field (%u, %u, %u)" % (magx, magy, magz))
    print("Magnetometer OK")
    

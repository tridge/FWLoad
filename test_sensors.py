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
            test.send('param fetch %s\n' % pname)
            test.expect('Requested parameter %s' % pname)
            test.expect('%s = (-?\d+\.\d+)\r\n' % pname)
            ofs = float(test.match.group(1))
            if abs(ofs) < 0.000001:
                raise("%s is zero - accel %u not calibrated (offset)" % (pname, idx))
            pname = 'INS_ACC%sSCAL_%s' % (n, axis)
            test.send('param fetch %s\n' % pname)
            test.expect('Requested parameter %s' % pname)
            test.expect('%s = (-?\d+\.\d+)\r\n' % pname)
            ofs = float(test.match.group(1))
            if abs(ofs-1.0) < 0.000001:
                failure("%s is zero - accel %u not calibrated (scale)" % (pname, idx))
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
            test.send('param fetch %s\n' % pname)
            test.expect('Requested parameter %s' % pname)
            test.expect('%s = (-?\d+\.\d+)\r\n' % pname)
            ofs = float(test.match.group(1))
            if abs(ofs) < 0.000001:
                raise("%s is zero - gyro %u not calibrated (offset)" % (pname, idx))
        print("Gyro cal %u OK" % (idx+1))


def check_baro(ref, refmav, test, testmav):
    '''check baros'''
    ref_press = util.wait_field(refmav, 'SCALED_PRESSURE', 'press_abs')
    press1 = util.wait_field(testmav, 'SCALED_PRESSURE', 'press_abs')
    press2 = util.wait_field(testmav, 'SCALED_PRESSURE2', 'press_abs')
    if abs(ref_press - press1) > 10:
        util.failure("Baro1 error pressure=%f should be %f" % (press1, ref_press))
    if abs(ref_press - press2) > 10:
        util.failure("Baro2 error pressure=%f should be %f" % (press2, ref_press))

    ref_temp = util.wait_field(refmav, 'SCALED_PRESSURE', 'temperature')*0.01
    temp1 = util.wait_field(testmav, 'SCALED_PRESSURE', 'temperature')*0.01
    temp2 = util.wait_field(testmav, 'SCALED_PRESSURE2', 'temperature')*0.01
    if abs(ref_temp - temp1) > 20:
        util.failure("Baro1 error temperature=%f should be %f" % (temp1, ref_temp))
    if abs(ref_temp - temp2) > 20:
        util.failure("Baro2 error temperature=%f should be %f" % (temp2, ref_temp))
    print("Baros OK")
        


def check_mag(ref, refmav, test, testmav):
    '''check mags'''
    magx = util.wait_field(testmav, 'RAW_IMU', 'xmag')
    magy = util.wait_field(testmav, 'RAW_IMU', 'ymag')
    magz = util.wait_field(testmav, 'RAW_IMU', 'zmag')
    field = sqrt(magx**2 + magy**2 + magz**2)
    if field < 100 or field > 1000:
        print("Bad magnetic field (%u, %u, %u)" % (magx, magy, magz))
    print("Magnetometer OK")
    

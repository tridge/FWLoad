#!/usr/bin/env python

'''
open connections to ref and test boards
'''

import mav_reference
import mav_test
import nsh_console
import util, time
import logger
from config import *
from StringIO import StringIO
from pymavlink import mavutil
from pymavlink.rotmat import Vector3
import rotate

def ref_gyro_offset_ok(refmav):
    '''check if ref gyro offsets are in range

    This copes with the reference board moving while in startup, which
    can lead to bad gyro calibration
    '''
    ref_ofs = refmav.recv_match(type='SENSOR_OFFSETS', blocking=True, timeout=10)
    if ref_ofs is None:
        logger.error("No SENSOR_OFFSETS message")
        return False
    gyros = Vector3(degrees(ref_ofs.gyro_cal_x),
                    degrees(ref_ofs.gyro_cal_y),
                    degrees(ref_ofs.gyro_cal_z))

    logger.debug("Gyro reference %.2f" % gyros.length())
    return gyros.length() < REF_GYRO_TOLERANCE
    

class Connection(object):
    '''open connections to ref and test boards'''
    def __init__(self, ref_only=False):
        util.kill_processes(['mavproxy.py', GDB])
        
        self.reflog = StringIO()
        self.testlog = StringIO()
        self.ref = None
        self.test = None
        self.nsh = None
        self.refmav = None
        self.testmav = None
        
        try:
            if not ref_only:
                self.nsh = nsh_console.nsh_console()
        except Exception as ex:
            self.close()
            util.show_error('Connecting to nsh console', ex, self.testlog)

        try:
            self.ref = mav_reference.mav_reference(self.reflog)
        except Exception as ex:
            self.close()
            util.show_error('Connecting to reference board1', ex, self.reflog)

        try:
            if not ref_only:
                self.test = mav_test.mav_test(self.testlog)
        except Exception as ex:
            self.close()
            util.show_error('Connecting to test board1', ex, self.testlog)

        try:
            logger.info("CONNECTING MAVLINK TO REFERENCE BOARD")
            self.refmav = mavutil.mavlink_connection('127.0.0.1:14550')
            util.wait_heartbeat(self.refmav, timeout=30)
            util.wait_mode(self.refmav, IDLE_MODES)
        except Exception as ex:
            self.close()
            util.show_error('Connecting to reference board2', ex, self.reflog)

        try:
            if not ref_only:
                logger.info("CONNECTING MAVLINK TO TEST BOARD")
                self.testmav = mavutil.mavlink_connection('127.0.0.1:14551')
                util.wait_heartbeat(self.testmav, timeout=30)
                logger.info("got heartbeat")
                util.wait_mode(self.testmav, IDLE_MODES)
                logger.info("Waiting for 'Ready to FLY'")
                self.fw_version = None
                self.px4_version = None
                self.nuttx_version = None
                self.stm32_serial = None
                ready = False
                self.test.send("param fetch\n")
                # log version information for later reference
                while (self.fw_version is None or
                       self.px4_version is None or
                       self.nuttx_version is None or
                       self.stm32_serial is None or
                       not ready):
                    i = self.test.expect(['APM: ([^\r\n]*Copter[^\r\n]*)\r\n',
                                          'APM: PX4: ([0-9a-f]+) NuttX: ([0-9a-f]+)\r\n',
                                          'APM: PX4v2 ([0-9A-F]+ [0-9A-F]+ [0-9A-F]+)\r\n',
                                          '(Ready to FLY)'],
                                         timeout=20)
                    if i == 3:
                        ready = True
                    elif i == 0:
                        self.fw_version = self.test.match.group(1)
                    elif i == 1:
                        self.px4_version = self.test.match.group(1)
                        self.nuttx_version = self.test.match.group(2)
                    elif i == 2:
                        self.stm32_serial = self.test.match.group(1)
        except Exception as ex:
            self.close()
            util.show_error('Connecting to test board2', ex, self.testlog)

        if self.nsh is not None:
            # log any extra nsh data
            logger.debug("Draining nsh")
            try:
                self.nsh.read_nonblocking(4096, 1)
            except Exception as ex:
                pass

        try:
            if not ref_only and not ref_gyro_offset_ok(self.refmav):
                self.close()
                util.failure("Bad reference gyro - FAILED")
        except Exception as ex:
            self.close()
            util.show_error('testing reference gyros', ex)

#        logger.info("Setting rotation level")
#        try:
#            rotate.set_rotation(self, 'level', wait=False)
#        except Exception as ex:
#            self.close()
#            util.show_error("unable to set safety off", ex)

    def discard_messages(self):
        '''discard pending mavlink messages'''
        if self.refmav:
            util.discard_messages(self.refmav)
        if self.testmav:
            util.discard_messages(self.testmav)
        if self.test:
            try:
                self.test.read_nonblocking(4096, 0)
            except Exception:
                pass
        if self.ref:
            try:
                self.ref.read_nonblocking(4096, 0)
            except Exception:
                pass

    def close(self):
        '''close all connections'''
        logger.info("Closing all connections")
        if self.refmav:
            self.refmav.close()
            self.refmav = None
        if self.testmav:
            self.testmav.close()
            self.testmav = None
        if self.ref:
            self.ref.close()
            self.ref = None
        if self.test:
            self.test.close()
            self.test = None
        if self.nsh:
            self.nsh.close()
            self.nsh = None

    def __del__(self):
        '''auto-close'''
        self.close()

def erase_parameters():
    '''erase parameters on test board'''
    conn = Connection()
    logger.info("Setting SYSID_SW_MREV to 0")
    conn.test.send('param set SYSID_SW_MREV 0\n')
    time.sleep(1)
    logger.info("rebooting")
    conn.test.send('reboot\n')
    util.discard_messages(conn.testmav)
    util.wait_mode(conn.testmav, IDLE_MODES, timeout=30)
    logger.info("Parameters erased")
    return True

if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)

    parser.add_argument("--ref-only", action='store_true', default=False, help="only connect to ref board")
    parser.add_argument("--erase", action='store_true', default=False, help="erase parameters")
    args = parser.parse_args()

    if args.erase:
        erase_parameters()
    else:
        conn = Connection(ref_only=args.ref_only)

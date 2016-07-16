#!/usr/bin/env python
'''
load firmwares over JTAG with gdb under pexpect
'''

import pexpect, sys, util
import logger
from StringIO import StringIO
from config import *
import power_control
import nsh_console
import colour_text

def load_firmware(device, firmware, mcu_id, run=False):
    '''load a given firmware'''
    cmd = "%s %s" % (GDB, firmware)
    logger.info("Loading firmware %s" % firmware)
    log = StringIO()
    try:
        gdb = pexpect.spawn(cmd, logfile=log, timeout=10)
        gdb.expect("Reading symbols from")
        gdb.expect("done")
        gdb.expect("(gdb)")
        gdb.send("target extended %s\n" % device)
        gdb.expect("Remote debugging using")
        gdb.expect("(gdb)")
        gdb.send("monitor swdp_scan\n")
        ids = CPUID_IO + CPUID_FMU
        cpu = gdb.expect(ids)
        if ids[cpu] not in mcu_id:
            util.failure("Incorrect CPU ID '%s' - expected '%s'" % (ids[cpu], mcu_id))
        gdb.expect("(gdb)")
        gdb.send("attach 1\n")
        gdb.expect("(gdb)")
        gdb.send("set mem inaccessible-by-default off\n")
        gdb.expect("(gdb)")
        gdb.send("load\n")
        gdb.expect("Loading section .text", timeout=20)
        gdb.expect("Loading section .data", timeout=30)
        gdb.expect("Start address", timeout=10)
        gdb.expect("Transfer rate", timeout=10)
        gdb.expect("(gdb)")
        if run:
            gdb.send("run\n")
            gdb.expect("Start it from the beginning?")
            gdb.send("y\n")
    except Exception as ex:
        util.show_error('Loading firmware %s' % firmware, ex, log)        


def load_firmware_USB(device, firmware):
    '''load a given firmware via USB'''
    cmd = "%s --port %s %s" % (PX_UPLOADER, device, firmware)
    logger.info("Loading firmware %s via USB" % firmware)
    log = StringIO()
    try:
        gdb = pexpect.spawn(cmd, logfile=log, timeout=20)
        gdb.expect("Found board")
        gdb.expect("Erase")
        gdb.expect("Program")
        gdb.expect("Verify")
        gdb.expect("Rebooting")
    except Exception as ex:
        util.show_error('Loading firmware %s' % firmware, ex, log)        
        
def erase_firmware(device, mcu_id):
    '''erase a firmware'''
    cmd = GDB
    logger.info("Erasing firmware for '%s'" % mcu_id)
    try:
        util.kill_processes([GDB])
        gdb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
        gdb.expect("(gdb)")
        gdb.send("target extended %s\n" % device)
        gdb.expect("Remote debugging using")
        gdb.expect("(gdb)")
        gdb.send("monitor swdp_scan\n")
        ids = CPUID_IO + CPUID_FMU
        cpu = gdb.expect(ids)
        if ids[cpu] not in mcu_id:
            util.failure("Incorrect CPU ID '%s' - expected '%s'" % (ids[cpu], mcu_id))
        gdb.expect("(gdb)")
        gdb.send("attach 1\n")
        gdb.expect("(gdb)")
        gdb.send("set mem inaccessible-by-default off\n")
        gdb.expect("(gdb)")
        gdb.send("monitor erase\n")
        gdb.expect("(gdb)", timeout=20)
        gdb.send('quit\n')
        gdb.expect('Quit anyway')
        gdb.send('y\n')
        gdb.expect("Detached from remote")
        gdb.close()
        logger.info("closed")
    except Exception as ex:
        util.show_error('Erasing firmware', ex)
    logger.info("Erase done")

def attach_gdb(device, mcu_id, firmware=None):
    '''attach to gdb'''
    cmd = GDB
    if firmware is not None:
        cmd += " " + firmware
    gdb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
    gdb.expect("(gdb)")
    gdb.send("target extended %s\n" % device)
    gdb.expect("Remote debugging using")
    gdb.expect("(gdb)")
    gdb.send("monitor swdp_scan\n")
    ids = CPUID_IO + CPUID_FMU
    cpu = gdb.expect(ids)
    if ids[cpu] not in mcu_id:
        util.failure("Incorrect CPU ID '%s' - expected '%s'" % (ids[cpu], mcu_id))
    gdb.expect("(gdb)")
    gdb.send("attach 1\n")
    gdb.expect("(gdb)")
    gdb.send("set mem inaccessible-by-default off\n")
    gdb.expect("(gdb)")
    if firmware is not None:
        logger.info("Use 'load' to load the firmware")
        logger.info("Use 'quit' to quit")
    gdb.send('\n')
    gdb.logfile = None
    gdb.interact()

def load_all_firmwares(retries=3):
    '''load 4 firmwares. Return True on success, False on failure'''
    while retries > 0:
        retries -= 1
        if not util.wait_devices([IO_JTAG, FMU_JTAG, FMU_DEBUG]):
            if retries == 1:
                logger.info("RETRIES=1 - POWER CYCLING")
                power_control.power_cycle(down_time=4)
            continue

        try:
            #load_firmware(IO_JTAG, FW_IO, CPUID_IO)
            load_firmware(IO_JTAG, BL_IO, CPUID_IO)

            load_firmware(FMU_JTAG, BL_FMU, CPUID_FMU, run=True)
            load_firmware_USB(USB_DEV_FMU_BL, FW_FMU_PX4)
        except Exception as ex:
            logger.error("error loading firmwares %s" % ex)
            continue

        # power cycle after loading to ensure the boards can come up cleanly
        power_retries = 6
        while power_retries > 0:
            power_retries -= 1
            power_control.power_cycle(down_time=4)
            logger.debug("restarting power")
            if util.wait_devices([FMU_DEBUG], timeout=10):
                break
            logger.info("Retrying power cycle - tries left %u" % power_retries)
            
        if not util.wait_devices([FMU_DEBUG], timeout=20):
            logger.info("Failed to find nsh console device")
            continue

        logger.debug("Checking nsh console")
        nsh = nsh_console.nsh_console()
        failure = None
        i = -1
        try:
            i = nsh.expect(['No MPU6000 external',
                            'l3gd20: driver start failed',
                            'Error in startup',
                            'ArduPilot started OK',
                            'format failed',
                            'Opening USB nsh',
                            'no RGB led',
                            'rgbled: init failed'])
        except Exception as ex:
            failure = "******* Failed to get data from NSH console *******"
            pass
        try:
            nsh.send("\nver all\n")
            nsh.expect("UID:")
            nsh.expect("nsh>")
            nsh.close()
        except Exception as ex:
            if i == 3:
                failure = "******* failed to get version from nsh *******"
            pass
        if failure is None:
            if i == 0:
                failure = "******* No external mpu6000 found - is IMU board connected? *****"
            if i == 1:
                failure = "******* No external l3gd20 found - is IMU board connected? *****"
            if i == 2:
                failure = "******* Failed to startup ArduPilot - is IMU board connected? *****"
            if i == 4:
                failure = "******* microSD card failure ********"
            if i == 5:
                failure = "****** ArduPilot failed to start - general failure ******"
            if i == 6:
                failure = "****** RGB LED not found on I2C ******"
            if i == 7:
                failure = "****** RGB LED initialisation failed on I2C ******"
        if failure is not None:
            logger.info(failure)
            colour_text.print_fail(failure)
            continue

        if util.wait_devices([USB_DEV_TEST, USB_DEV_REFERENCE]):
            break

        logger.info("Failed to find USB devices")  
        if retries > 0:
            logger.info("RETRIES %u - TRYING AGAIN" % retries)
    if retries == 0:
        logger.error("FAILED TO LOAD FIRMWARES")
        return False

    logger.info("All firmwares loaded OK")
    return True

def erase_firmwares(retries=3):
    '''erase both firmwares. Return True on success, False on failure'''
    if not util.wait_devices([IO_JTAG, FMU_JTAG, FMU_DEBUG]):
        logger.info("jtag devices not ready")
        return False

    try:
        erase_firmware(IO_JTAG, CPUID_IO)
        erase_firmware(FMU_JTAG, CPUID_FMU)
    except Exception as ex:
        logger.error(ex)
        return False
    logger.info("All firmwares erased OK")
    return True


if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)

    parser.add_argument("--erase", default=False, action='store_true', help="erase flash")
    parser.add_argument("--io", default=False, action='store_true', help="attach gdb to io")
    parser.add_argument("--fmu", default=False, action='store_true', help="attach gdb to fmu")
    parser.add_argument("--firmware", default=None, help="firmware to use in attach")
    args = parser.parse_args()
    if args.io:
        attach_gdb(IO_JTAG, CPUID_IO, args.firmware)
    elif args.fmu:
        attach_gdb(FMU_JTAG, CPUID_FMU, args.firmware)
    elif args.erase:
        erase_firmwares()
    else:
        load_all_firmwares()

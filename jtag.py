#!/usr/bin/env python
'''
load firmwares over JTAG with gdb under pexpect
'''

import pexpect, sys, util
from StringIO import StringIO
from config import *

def load_firmware(device, firmware, mcu_id, run=False):
    '''load a given firmware'''
    cmd = "%s %s" % (GDB, firmware)
    print("Loading firmware %s" % firmware)
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
        cpu = gdb.expect([CPUID_IO, CPUID_FMU])
        if cpu == 0:
            if mcu_id != CPUID_IO:
                util.failure("Incorrect CPU ID '%s' - expected '%s'" % (CPUID_IO, mcu_id))
        else:
            if mcu_id != CPUID_FMU:
                util.failure("Incorrect CPU ID '%s' - expected '%s'" % (CPUID_FMU, mcu_id))
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

def erase_firmware(device, mcu_id):
    '''erase a firmware'''
    cmd = GDB
    print("Erasing firmware for '%s'" % mcu_id)
    try:
        util.kill_processes([GDB])
        gdb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
        gdb.expect("(gdb)")
        gdb.send("target extended %s\n" % device)
        gdb.expect("Remote debugging using")
        gdb.expect("(gdb)")
        gdb.send("monitor swdp_scan\n")
        cpu = gdb.expect([CPUID_IO, CPUID_FMU])
        if cpu == 0:
            if mcu_id != CPUID_IO:
                util.failure("Incorrect CPU ID '%s' - expected '%s'" % (CPUID_IO, mcu_id))
        else:
            if mcu_id != CPUID_FMU:
                util.failure("Incorrect CPU ID '%s' - expected '%s'" % (CPUID_FMU, mcu_id))
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
        print("closed")
    except Exception as ex:
        util.show_error('Erasing firmware', ex)
    print("Erase done")

def attach_gdb(device, mcu_id):
    '''attach to gdb'''
    cmd = GDB
    gdb = pexpect.spawn(cmd, logfile=None, timeout=10)
    gdb.expect("(gdb)")
    gdb.send("target extended %s\n" % device)
    gdb.expect("Remote debugging using")
    gdb.expect("(gdb)")
    gdb.send("monitor swdp_scan\n")
    cpu = gdb.expect([CPUID_IO, CPUID_FMU])
    if cpu == 0:
        if mcu_id != CPUID_IO:
            util.failure("Incorrect CPU ID '%s' - expected '%s'" % (CPUID_IO, mcu_id))
    else:
        if mcu_id != CPUID_FMU:
            util.failure("Incorrect CPU ID '%s' - expected '%s'" % (CPUID_FMU, mcu_id))
    gdb.expect("(gdb)")
    gdb.send("attach 1\n")
    gdb.expect("(gdb)")
    gdb.send("set mem inaccessible-by-default off\n")
    gdb.expect("(gdb)")
    gdb.interact()

def load_all_firmwares(retries=3):
    '''load 4 firmwares. Return True on success, False on failure'''
    while retries > 0:
        retries -= 1
        if not util.wait_devices([IO_JTAG, FMU_JTAG, FMU_DEBUG]):
            if retries == 1:
                print("RETRIES=1 - POWER CYCLING")
                power_control.power_cycle(down_time=4)
            continue

        try:
            load_firmware(IO_JTAG, FW_IO, CPUID_IO)
            load_firmware(IO_JTAG, BL_IO, CPUID_IO, run=True)

            load_firmware(FMU_JTAG, BL_FMU, CPUID_FMU)
            load_firmware(FMU_JTAG, FW_FMU, CPUID_FMU, run=True)
        except Exception as ex:
            continue

        if util.wait_devices([USB_DEV_TEST, USB_DEV_REFERENCE]):
            break
        if retries > 0:
            print("RETRIES %u - TRYING AGAIN" % retries)
    if retries == 0:
        print("FAILED TO LOAD FIRMWARES")
        return False

    if not util.wait_devices([USB_DEV_TEST, USB_DEV_REFERENCE]):
        print("Failed to find USB devices")
        return False

    print("All firmwares loaded OK")
    return True

def erase_firmwares(retries=3):
    '''erase both firmwares. Return True on success, False on failure'''
    if not util.wait_devices([IO_JTAG, FMU_JTAG, FMU_DEBUG]):
        print("jtag devices not ready")
        return False

    try:
        erase_firmware(IO_JTAG, CPUID_IO)
        erase_firmware(FMU_JTAG, CPUID_FMU)
    except Exception as ex:
        print(ex)
        return False
    print("All firmwares erased OK")
    return True


if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)

    parser.add_argument("--erase", default=False, action='store_true', help="erase flash")
    parser.add_argument("--io", default=False, action='store_true', help="attach gdb to io")
    parser.add_argument("--fmu", default=False, action='store_true', help="attach gdb to fmu")
    args = parser.parse_args()
    if args.io:
        attach_gdb(IO_JTAG, CPUID_IO)
    elif args.fmu:
        attach_gdb(FMU_JTAG, CPUID_FMU)
    elif args.erase:
        erase_firmwares()
    else:
        load_all_firmwares()

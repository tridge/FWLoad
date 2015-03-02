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
        gdb.expect(mcu_id)
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
    except Exceptions as ex:
        util.show_error('Loading firmware %s' % firmware, ex, log)        

def load_all_firmwares(retries=3):
    '''load 4 firmwares. Return True on success, False on failure'''
    while retries > 0:
        retries -= 1
        if not util.wait_devices([IO_JTAG, FMU_JTAG, FMU_DEBUG]):
            if retries == 1:
                print("RETRIES=1 - POWER CYCLING")
                power_control.power_cycle()
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
    if not util.wait_devices([USB_DEV_TEST, USB_DEV_REFERENCE]):
        print("Failed to find USB devices")
        return False

    print("All firmwares loaded OK")
    return True


if __name__ == '__main__':
    load_all_firmwares()

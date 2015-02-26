import sys, time, os
import power_control
from config import *

def show_error(test, ex, logstr):
    '''display an error then exit'''
    lines = logstr.readlines()
    n = 10
    if len(lines) < n:
        n = len(lines)
    for i in range(n):
        print(lines[len(lines)-10+i])
    print("FAILED: %s" % test)
    sys.exit(1)

def wait_devices(devices, timeout=10):
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
        time.sleep(0.25)
    print("Missing devices: %s" % missing)
    return False

def power_wait_devices():
    '''wait for all needed JTAG devices'''
    retries = 5
    while retries > 0:
        retries -= 1
        power_control.power_cycle()
        print("Waiting for power up")
        if wait_devices([FMU_JTAG, IO_JTAG, FMU_DEBUG]):
            time.sleep(1)
            return True
    print("Failed to power up devices")
    sys.exit(1)


def failure(msg):
    '''show a failure msg and exit'''
    print(msg)
    sys.exit(1)
    

def wait_field(refmav, msg_type, field):
    '''wait for a field value'''
    msg = None
    # get the latest available msg
    while True:
        msg2 = refmav.recv_match(type=msg_type, blocking=(msg==None))
        if msg2 is None:
            break
        msg = msg2
    return getattr(msg, field)


#!/usr/bin/env python
'''
connect to nsh console
'''

import pexpect, sys
from config import *
import logging

def mav_test(testlog=None):
    '''connect to test board'''
    print("CONNECTING TO TEST BOARD")
    logfile = logging.new_tlog("TestBoard")
#    cmd = "strace -f -ttT -s 200 -o %s.trace mavproxy.py --master %s --out 127.0.0.1:14551 --logfile %s" % (logfile, USB_DEV_TEST, logfile)
    cmd = "mavproxy.py --master %s --out 127.0.0.1:14551 --logfile %s" % (USB_DEV_TEST, logfile)
    if REMOTE_MONITOR2:
        cmd += " --out %s" % REMOTE_MONITOR2
    return pexpect.spawn(cmd, logfile=testlog, timeout=10)

if __name__ == '__main__':
    test = mav_test()
    test.interact()

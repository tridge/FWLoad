#!/usr/bin/env python
'''
connect to nsh console
'''

import pexpect, sys
from config import *
import logging

def mav_reference(reflog=None, extra_args=None):
    '''connect to reference board'''
    print("CONNECTING TO REFERENCE BOARD")
    logfile = logging.new_tlog("RefBoard")
    cmd = "mavproxy.py --master %s --out 127.0.0.1:14550 --logfile %s" % (USB_DEV_REFERENCE, logfile)
    if REMOTE_MONITOR['ref']:
        cmd += " --out %s" % REMOTE_MONITOR['ref']
    if extra_args:
        cmd += " %s" % extra_args
    return pexpect.spawn(cmd, logfile=reflog, timeout=10)

if __name__ == '__main__':
    ref = mav_reference(None, ' '.join(sys.argv[1:]))
    ref.interact()

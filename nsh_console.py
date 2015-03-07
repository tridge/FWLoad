#!/usr/bin/env python
'''
connect to nsh console
'''

import pexpect, sys
from config import *
import logging

def nsh_console(interactive=False):
    '''connect to nsh on test board'''
    print("CONNECTING TO NSH CONSOLE")
    if interactive:
        log = None
    else:
        logfile = logging.new_tlog("TestNSH", extension='log')
        log = open(logfile, 'w')
    cmd = "mavproxy.py --baudrate 57600 --setup --master %s" % FMU_DEBUG
    ret = pexpect.spawn(cmd, logfile=log, timeout=10)
    ret.send('\n')
    return ret

if __name__ == '__main__':
    nsh = nsh_console(interactive=True)
    nsh.interact()

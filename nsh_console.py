#!/usr/bin/env python
'''
connect to nsh console
'''

import pexpect, sys
from config import *
import logger, time

def nsh_console(interactive=False):
    '''connect to nsh on test board'''
    logger.info("CONNECTING TO NSH CONSOLE")
    if interactive:
        logfile = None
    else:
        logfile = logger.new_tlog("TestNSH", extension='log')
        logger.info("nsh logging to %s" % logfile)
    cmd = "mavproxy.py --baudrate 57600 --setup --master %s" % FMU_DEBUG
    if logfile is not None:
        cmd += " --logfile %s" % logfile
    ret = pexpect.spawn(cmd, logfile=open("/dev/null", 'w'), timeout=30)
    ret.send('\r\n')
    return ret

if __name__ == '__main__':
    nsh = nsh_console(interactive=True)
    nsh.interact()

#!/usr/bin/env python

from config import *
import pexpect, sys, time

def nsh_command(cmd, dev=FMU_DEBUG):
    '''run a nsh command on the FMU debug console'''
    cons  = pexpect.spawn("mavproxy.py --master %s --baudrate 57600 --setup --aircraft RefBoardConsole" % dev,
                          logfile=sys.stdout, timeout=10)
    cons.write("\n")
    cons.expect("nsh>")
    cons.write("%s\n" % cmd)
    time.sleep(1)

if __name__ == '__main__':
    nsh_command(' '.join(sys.argv[1:]))

#!/usr/bin/env python
'''
connect to nsh console
'''

import pexpect, sys
from config import *

con = pexpect.spawn("mavproxy.py --setup --baudrate 57600 --master %s" % FMU_DEBUG)
con.interact()

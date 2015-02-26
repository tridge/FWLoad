#!/usr/bin/env python
'''
run an accelcal on the test jig
'''

import pexpect, sys, time
from math import *
from config import *
import util

def set_rotation(ref, refmav, rotation, wait=20):
    '''set servo rotation'''
    if not rotation in ROTATIONS:
        raise("No rotation %s" % rotation)
    (s5, s6) = ROTATIONS[rotation]
    ref.send("servo set 5 %u\n" % s5)
    ref.expect("COMMAND_ACK {command : 183, result : 0}")
    ref.send("servo set 6 %u\n" % s6)
    ref.expect("COMMAND_ACK {command : 183, result : 0}")
    time.sleep(2)
    util.wait_field(refmav, 'ATTITUDE', 'roll')
    attitude = refmav.recv_match(type='ATTITUDE', condition='abs(degrees(ATTITUDE.rollspeed))<0.1 and abs(degrees(ATTITUDE.pitchspeed))<0.1 and abs(degrees(ATTITUDE.yawspeed))<0.1', blocking=True)
    print("Orientation %s: Roll=%.1f Pitch=%.1f RollSpeed=%.1f PitchSpeed=%.1f" % (
        rotation,
        degrees(attitude.roll), degrees(attitude.pitch),
        degrees(attitude.rollspeed), degrees(attitude.pitchspeed)))

if __name__ == '__main__':
    ref  = pexpect.spawn("mavproxy.py --master %s --out 59.167.251.244:15550 --out 127.0.0.1:14550" % USB_DEV_REFERENCE,
                         logfile=sys.stdout, timeout=10)
    ref.expect(['Received [0-9]+ parameters', 'MANUAL>'])
    ref.expect(['Received [0-9]+ parameters', 'MANUAL>'])

    refmav = mavutil.mavlink_connection('127.0.0.1:14550', robust_parsing=True)
    refmav.wait_heartbeat()

    set_rotation(ref, refmav, sys.argv[1], wait=0)
    

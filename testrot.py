#!/usr/bin/env python

from config import *
from pymavlink.rotmat import Vector3, Matrix3
from math import *

for rotation in ROTATIONS:
    zero_chan1 = ROTATIONS['level'][0]
    zero_chan2 = ROTATIONS['level'][1]
    chan1 = ROTATIONS[rotation][0]
    chan2 = ROTATIONS[rotation][1]
    dcm = Matrix3()
    dcm.from_euler(0, radians(PITCH_SCALE) * (chan2 - zero_chan2), 0)
    dcm.rotate(Vector3(0, 0, radians(YAW_SCALE) * (chan1 - zero_chan1)))
    dcm.normalize()
    (r, p, y) = dcm.to_euler()
    print("%s: r=%.1f p=%.1f y=%.1f" % (rotation, degrees(r), degrees(p), degrees(y)))
    

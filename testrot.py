#!/usr/bin/env python

from config import *
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink.quaternion import Quaternion
from math import *

def gimbal_model(chan1, chan2):
    '''model the gimbal setup'''
    zero_chan1 = ROTATIONS['level'][0]
    zero_chan2 = ROTATIONS['level'][1]
    dcm = Matrix3()
    dcm1 = Matrix3()
    dcm1.from_euler(0, radians(PITCH_SCALE) * (chan2 - zero_chan2), 0)
    dcm2 = Matrix3()
    dcm2.from_euler(0, 0, radians(YAW_SCALE)*(chan1-zero_chan1))
    dcm *= dcm1 * dcm2
    return dcm.to_euler()

def quat_division(quat, rquat):
    '''quaternion division'''
    quat0 = quat[0]
    quat1 = quat[1]
    quat2 = quat[2]
    quat3 = quat[3]

    rquat0 = rquat[0]
    rquat1 = rquat[1]
    rquat2 = rquat[2]
    rquat3 = rquat[3]

    q1 = (rquat0*quat0 + rquat1*quat1 + rquat2*quat2 + rquat3*quat3)
    q2 = (rquat0*quat1 - rquat1*quat0 - rquat2*quat3 + rquat3*quat2)
    q3 = (rquat0*quat2 + rquat1*quat3 - rquat2*quat0 - rquat3*quat1)
    q4 = (rquat0*quat3 - rquat1*quat2 + rquat2*quat1 - rquat3*quat0)
    return [q1, q2, q3, q4]

def gimbal_controller(dcm_estimated, dcm_demanded, chan1):
    '''return the delta to chan1 and chan2'''
    quat_est = Quaternion(dcm_estimated)
    quat_dem = Quaternion(dcm_demanded)

    quatErr = quat_division(quat_dem, quat_est)
    
    if quatErr[0] >= 0.0:
        scaler = 2.0
    else:
        scaler = -2.0
    bf_rates = scaler * Vector3(quatErr[1], quatErr[2], quatErr[3])

    yaw_joint_rad = radians((chan1 - ROTATIONS['level'][0]) * YAW_SCALE)
    chan1_change = degrees(bf_rates.z) * YAW_SCALE
    chan2_change = degrees(bf_rates.x * sin(yaw_joint_rad) +
                           bf_rates.y * cos(yaw_joint_rad)) / PITCH_SCALE
    return (chan1_change, chan2_change)


def try_rotation(rotation,
                 demanded_roll,
                 demanded_pitch,
                 chan1, chan2):
    (r, p, y) = gimbal_model(chan1, chan2)
    print("%s: %.1f %.1f %.1f" % (rotation,
                                  degrees(r),
                                  degrees(p),
                                  degrees(y)))

    for idx in range(90):
        dcm_estimated = Matrix3()
        dcm_estimated.from_euler(r,p,y)
    
        droll = demanded_roll
        if droll is None:
            droll = r
        else:
            droll = radians(droll)
        dpitch = radians(demanded_pitch)

        dcm_demanded = Matrix3()
        dcm_demanded.from_euler(droll, dpitch, y)

        (chan1_change, chan2_change) = gimbal_controller(dcm_estimated,
                                                         dcm_demanded, chan1)

        chan1 += chan1_change
        chan2 += chan2_change
        (r, p, y) = gimbal_model(chan1, chan2)
        print("-> %.1f %.1f %.1f" % (degrees(r),
                                     degrees(p),
                                     degrees(y)))


for rotation in ROTATIONS:
    chan1 = ROTATIONS[rotation][0]
    chan2 = ROTATIONS[rotation][1]+20
    demanded_roll  = ROTATIONS[rotation][2]
    demanded_pitch = ROTATIONS[rotation][3]

    try_rotation(rotation, demanded_roll, demanded_pitch, chan1, chan2)
    
    

#step 2: joint_rates.z = bf_rates.z
#step 3: joint_rates.y = bf_rates.x * sin(joint.z) + bf_rates.y * cos(joint.z)



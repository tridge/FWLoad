PITCH_SCALE = 0.41
YAW_SCALE = -0.38

# servo positions for different orientations of boards in the test jig
# the columns are:
#    pitch PWM
#    yaw PWM
#    expected roll
#    expected pitch
class Rotation(object):
    def __init__(self, chan1, chan2, roll, pitch):
        self.chan1 = chan1
        self.chan2 = chan2
        self.roll = roll
        self.pitch = pitch

ROTATIONS = {
	'level' : Rotation(1500, 1500, 0, 0),
	'right' : Rotation(1700, 1720, 90, 0),
	'left' : Rotation(1300, 1420, -90, 0),
	'up' : Rotation(1500, 1720, None, 90),
	'down' : Rotation(1900, 1720, None, -90),
	'back' : Rotation(1900, 1920, 180, 0),
}

ROTATIONS_ETE = {
	# Yaw , Pitch
        'level' : Rotation(0, 0, 0, 0),
        'right' : Rotation(00, 90, -90, 0),
        'left' : Rotation(180, 90, 90, 0),
        'up' : Rotation(90, 90, None, 90),
        'down' : Rotation(270, 90, None, -90),
        'back' : Rotation(180, 180, 180, 0)
}

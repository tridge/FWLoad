'''
config for factory test rig
'''

from math import *

GDB="arm-none-eabi-gdb"
FMU_BMAGIC_SERIAL="B5DEADF0"
IO_BMAGIC_SERIAL="B5DFADF1"

FMU_JTAG="/dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_%s-if00" % FMU_BMAGIC_SERIAL
IO_JTAG="/dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_%s-if00" % IO_BMAGIC_SERIAL

FMU_DEBUG="/dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_%s-if02" % FMU_BMAGIC_SERIAL

FW_IO="FW/px4io.elf"
BL_IO="FW/px4io_bl.elf"

FW_FMU="FW/firmware-test.elf"
BL_FMU="FW/px4fmuv3_bl.elf"

# the expected CPU IDs shown by gdb. This allows us to tell if the
# jtag connections are the right way around
CPUID_IO="STM32, Medium density"
CPUID_FMU="STM32F4xx"

USB_DEV_TEST="/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00"
USB_DEV_REFERENCE="/dev/serial/by-id/usb-3D_Robotics_PH_REFERENCE_0-if00"

FTDI_POWER="/dev/serial/by-id/usb-FTDI_TTL232R_FTFX6YMW-if00-port0"

# how many accels and gyros we expect to find
NUM_ACCELS=3
NUM_GYROS=3

# the address for remote mavlink UDP monitoring. This allows an
# operator on a remote VPN to watch the process
REMOTE_MONITOR="10.26.1.200:16550"
REMOTE_MONITOR2="10.26.1.200:16551"

# the tolerances in degrees for rotation to the level position and
# other positions
ROTATION_LEVEL_TOLERANCE = 3.0
ROTATION_TOLERANCE = 5.0

# tolerance of quiescent state in degrees/second. This is how still
# the jig needs to be between movements of the servos
GYRO_TOLERANCE = 0.2

# tolerances for the various sensors when testing against the
# reference board
PRESSURE_TOLERANCE = 10
TEMPERATURE_TOLERANCE = 20
VOLTAGE_TOLERANCE = 0.4
TILT_TOLERANCE = 0.5

# what channels control pitch and yaw in body frame the gimbal is a
# (3,2) arrangement, so the yaw channel is always yaw in body
# frame. The pitch channel is pitch when in the home position

# yaw in body frame
YAW_CHANNEL = 5
# +100 change == -20 degrees
YAW_SCALE = -22.0 / 100

# pitch in earth frame
PITCH_CHANNEL = 6
# +100 change == -34 degrees
PITCH_SCALE = -34.0/100

# acceptable modes when the test board is idle. This works for both
# plane and copter
IDLE_MODES = ["RTL>","CIRCLE>","MANUAL>","STABILIZE>"]


# servo positions for different orientations of boards in the test jig
# the columns are:
#    servo5 PWM
#    servo6 PWM
#    expected roll
#    expected pitch
class Rotation(object):
    def __init__(self, chan1, chan2, roll, pitch):
        self.chan1 = chan1
        self.chan2 = chan2
        self.roll = roll
        self.pitch = pitch

ROTATIONS = {
    'level' : Rotation(1272, 1687,    0,    0),
    'right' : Rotation(855,  1420,   90,    0),
    'left'  : Rotation(1660, 1420,  -90,    0),
    'up'    : Rotation(1260, 1420,  None,  90),
    'down'  : Rotation(1274, 1950,  None, -90),
    'back'  : Rotation(1255, 1180,  180,    0),
    'slant' : Rotation(1097, 1563,   30,   30)
    }

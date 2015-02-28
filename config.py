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

CPUID_IO="STM32, Medium density"
CPUID_FMU="STM32F4xx"

USB_DEV_TEST="/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00"
USB_DEV_REFERENCE="/dev/serial/by-id/usb-3D_Robotics_PH_REFERENCE_0-if00"

FTDI_POWER="/dev/serial/by-id/usb-FTDI_TTL232R_FTFX6YMW-if00-port0"

NUM_ACCELS=3
NUM_GYROS=3

REMOTE_MONITOR="10.26.1.200:16550"
REMOTE_MONITOR2="10.26.1.200:16551"

ROTATION_LEVEL_TOLERANCE = 2.0
ROTATION_TOLERANCE = 5.0

GYRO_TOLERANCE = radians(0.2)

PRESSURE_TOLERANCE = 10
TEMPERATURE_TOLERANCE = 20
VOLTAGE_TOLERANCE = 0.2

# what channels control pitch and yaw in body frame

# yaw in body frame
YAW_CHANNEL = 5
# +100 change == -20 degrees
YAW_SCALE = -22.0 / 100




# pitch in earth frame
PITCH_CHANNEL = 6
# +100 change == -34 degrees
PITCH_SCALE = -34.0/100




# servo positions for different orientations of boards in the test jig
# the columns are:
#    servo5 PWM
#    servo6 PWM
#    expected roll
#    expected pitch
ROTATIONS = {
    'level' : (1272, 1687,    0,    0),
    'right' : (855,  1420,   90,    0),
    'left'  : (1660, 1420,  -90,    0),
    'up'    : (1260, 1420,  None,  90),
    'down'  : (1274, 1950,  None, -90),
    'back'  : (1255, 1180,  180,    0)
    }

'''
level: yaw:5 pitch:6



'''

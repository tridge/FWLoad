
'''
config for factory test rig
'''

from math import *
import os, sys

GDB="arm-none-eabi-gdb"

# serial numbers of black magic probes in the various test jigs
FMU_BMAGIC_SERIAL=[ "B5D9B0CD", "B5DFB7C6","B5DDB0F0","B5DDB0D6","B5DCB0E4" ]
IO_BMAGIC_SERIAL=[ "B5DBB0CE", "B5DDB7C5", "B5DAB7D3","B5DDB0E3","B5DEB0F1" ]

# serial numbers of the barcode scanners
BARCODE_SCANNER_SERIAL=[ "16C0_XXXXXX" ]

FW_IO="FW/px4io.elf"
BL_IO="FW/px4io_bl.elf"

FW_FMU="FW/firmware-test.elf"
BL_FMU="FW/px4fmuv2_bl.elf"

# the expected CPU IDs shown by gdb. This allows us to tell if the
# jtag connections are the right way around
CPUID_IO="STM32, Medium density"
CPUID_FMU="STM32F4xx"

USB_DEV_TEST="/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00"
USB_DEV_REFERENCE="/dev/serial/by-id/usb-3D_Robotics_PH_REFERENCE_0-if00"

# there must be exactly one FTDI device
FTDI_POWER="/dev/serial/by-id/usb-FTDI_FT*"

# file containing factory parameters to load
FACTORY_PARM="FW/factory.parm"

# how many accels and gyros we expect to find
NUM_ACCELS=3
NUM_GYROS=3

# the address for remote mavlink UDP monitoring. This allows an
# operator on a remote VPN to watch the process
REMOTE_MONITOR = { "ref" : None, "test" : None}

# the tolerances in degrees for rotation to the level position and
# other positions
ROTATION_LEVEL_TOLERANCE = 3.0
ROTATION_TOLERANCE = 5.0

# tolerance of quiescent state in degrees/second. This is how still
# the jig needs to be between movements of the servos
GYRO_TOLERANCE = 0.3

# if gyro offsets are off by more than this value then the ref board was probably moving
# and we need to power cycle
REF_GYRO_TOLERANCE = 10.0

# tolerances for the various sensors when testing against the
# reference board
PRESSURE_TOLERANCE = 10
TEMPERATURE_TOLERANCE = 30
VOLTAGE_TOLERANCE = 0.4

# tolerance in gyro integration
GYRO_SUM_TOLERANCE = 10.0

# TILT_TOLERANCE1 is the tilt tolerance for the isolated sensors. We expect that these
# may be off by a few degrees, which we correct with AHRS trim
TILT_TOLERANCE1 = 5.0

# TILT_TOLERANCE3 is for the non-isolated sensor. It should be correctly aligned with
# the circuit board, so tolerance is smaller
TILT_TOLERANCE3 = 2.5

# what channels control pitch and yaw in body frame the gimbal is a
# (3,2) arrangement, so the yaw channel is always yaw in body
# frame. The pitch channel is pitch when in the home position

# yaw in body frame
YAW_CHANNEL = 2

# pitch in earth frame
PITCH_CHANNEL = 1

from calibration import *

# acceptable modes when the test board is idle. This works for both
# plane and copter
IDLE_MODES = ["RTL","CIRCLE","MANUAL","STABILIZE"]


# find the black magic probes from the list
FMU_JTAG = None
IO_JTAG = None
for id in FMU_BMAGIC_SERIAL:
    p = "/dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_%s-if00" % id
    if os.path.exists(p):
        FMU_JTAG = p
        FMU_DEBUG="/dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_%s-if02" % id
for id in IO_BMAGIC_SERIAL:
    p = "/dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_%s-if00" % id
    if os.path.exists(p):
        IO_JTAG = p
if FMU_JTAG is None:
    print("No JTAG device for FMU found")
    sys.exit(1)

if IO_JTAG is None:
    print("No JTAG device for IO found")
    sys.exit(1)

# find the bardode scanner
BARCODE_SCANNER = None
for id in BARCODE_SCANNER_SERIAL:
    p = "/dev/serial/by-id/usb-LineFotuneImaging___Mobility_%s-if00" % id
    if os.path.exists(p):
        BARCODE_SCANNER = p

if BARCODE_SCANNER is None:
    print("No barcode scanner found")
    sys.exit(1)


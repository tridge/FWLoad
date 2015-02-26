'''
config for factory test rig
'''

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

# servo positions for different orientations of boards
ROTATIONS = {
    'level' : (1260, 1687),
    'right' : (855,  1420),
    'left'  : (1660, 1420),
    'down'  : (1260, 1420),
    'up'    : (1274, 1950),
    'back'  : (1255, 1180)
    }

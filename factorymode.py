#!/usr/bin/env python
from config import *
import configcheck
import logger
import accelcal
import jtag
import power_control
import time
import util
import sys, os, fcntl
import logger
import colour_text
import connection
import rotate
import barcode
import savedstate
import otp_program_mod
from pymavlink import mavutil

logger.info("Setting Pixhawk into Factory Mode %s" % time.ctime())
power_control.power_cycle(down_time=4)

# disable stdout buffering
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

# wait for the power to come on again
while not util.wait_devices([FMU_JTAG, IO_JTAG, FMU_DEBUG]):
    logger.info("waiting for power up....")
util.kill_processes(['mavproxy.py', GDB])
time.sleep(2)

while not util.wait_devices([USB_DEV_TEST, USB_DEV_REFERENCE]):
   logger.error("FAILED to find USB test and reference devices")
   power_control.power_cycle(down_time=4)

conn = connection.Connection()
logger.info("FW version: %s" % conn.fw_version)
logger.info("PX4 version: %s" % conn.px4_version)
logger.info("NuttX version: %s" % conn.nuttx_version)
logger.info("STM32 serial: %s" % conn.stm32_serial)

# lock the two telemetry ports to prevent the COMMAND_ACK messages in accel cal
# from looping back between the two telemetry ports
logger.info("Locking telemetry ports")
util.lock_serial_port(conn.testmav, mavutil.mavlink.SERIAL_CONTROL_DEV_TELEM1)
util.lock_serial_port(conn.testmav, mavutil.mavlink.SERIAL_CONTROL_DEV_TELEM2)

logger.info("Trying to set Factory Mode")
conn.test.send("factory_test start")
logger.info("FACTORY_MODE_SET")
sys.exit(1)


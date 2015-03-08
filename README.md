# Firmware Loader

This is a firmware loader and factory test tool. It is designed to load firmware 
on a Pixhawk2 in a factory, and test sensors and other peripherals for correct operation.

## Hardware Environment

The system assumes it has two Pixhawks in the test jig. One is the
'reference' Pixhawk and the other is the 'test' Pixhawk. The reference
Pixhawk may be a bare board without any isolated IMUs. The reference
Pixhawk should have no microSD.

### Test Jig

The test jig is assumed to have two servos in a gimbal
arrangement. The gimbal is assumed to be a (3,2) setup, so that the
servo closest to the Pixhawk is a body-yaw servo.  

The rotation rates, reversals and set-points of the servos are set in
config.py. The servos are assumed to be attached to the first two
IO channels of the reference board.

### USB Devices

The scripts assume the following USB devices, configured in config.py:

 * two black magic probes. Serial numbers must be in config.py. 
 * nsh console debug port on test Pixhawk is assumed to be attached to
   the FMU black magic probe
 * one FTDI adapter. The RTS pin on that adapter is used for power
   control to the jig. Active high.
 * USB ports for the two Pixhawks

### Environment

All tools should be run from the home directory, running from FWLoad
directory like this:

> FWLoad/some_script.py

The FW/ directory contains the firmwares which will be loaded. The key
firmwares are:

* FW/px4io.elf - the px4io firmware
* FW/px4io_bl.elf - the IO bootloader
* FW/px4fmuv2_bl.elf - the bootloader for the FMU
* FW/firmware-test.elf - the firmware to be loaded onto test boards

## Tools

The FWLoad package consists of a set of python scripts. The main
script is factoryload.py, and that is what is run for continuous
factory operation. The other tools are for setup and diagnostics.

### factoryload.py

The FWLoad/factoryload.py script takes the following commands:

  -h, --help         show this help message and exit
  --test             run in test loop
  --once             run one install only
  --nofw             don't reload firmware
  --erase            erase firmware and parameters
  --monitor MONITOR  monitor address

Normal operation is to run factoryload.py with no arguments. That will
do the following:

 * wait for power on, by watching for the appearance of the black
   magic probes and the FTDI adapter on the bus. Power on should be
   triggered by a microswitch in the lid of the device

 * 4 firmwares (two bootloaders, IO firmware and FMU firmware) are
   loaded using scripted gdb over jtag on the black magic probes

 * MAVLink connections are made to both the reference and test boards

 * the gyros are monitored to wait for any startup movement to stop

 * the accelerometers on the test board are calibrated, by starting
   accelcal on the test board while moving the servos on the reference
   board

 * the AHRS trim values are set by comparing the accelerometers of the
   reference and test boards

 * a series of sensor tests are run, including internal magnetometer,
   barometer, analog voltages and serial-loopback tests

 * if everything passed then the system is powered off via the FTDI
   cable, ready for the next board

The command line opttions can be used to run in different modes.

 * the --test option puts the system into a mode where it doesn't wait
   for the operator to use the lid. The FTDI power control is used to
   simulate the power-on/power-off of the operator. The screen is also
   not cleared between runs. This mode is very useful for continuous
   unattended operation to validate a test jig.

 * the --once option makes the script do a single run instead of
   looping. 

 * the --nofw option skips the firmware load step

 * the --erase option forces an erase of the firmwares and FRAM
   parameters

 * the --monitor option allows for a remote MAVLink monitoring system
   to be specified. This should be a IPv4 address of a system
   listening on UDP ports 16550 and 16551 (for reference and test
   boards). This can be used to remotely monitor the factory load
   process

### power_control.py

The power_control.py tool can be used to power cycle the system via
FTDI. Just run:

> FWLoad/power_control.py

by default the power is cut for 4 seconds then re-applied

### jtag.py

The FWLoad/jtag.py script is the gdb script for loading firmwares. You
can run it standalone to test that part of the process.

You can optionally give a --erase option to erase all firmwares on the
board, like this:

> FWLoad/jtag.py --erase

### rotate.py

The FWLoad/rotate.py script allows for testing the rotation code, to
ensure the servo set points are correct. The setpoints are in
config.py.

You can specify a rotation on the command line. For example:

> FWLoad/rotate.py left

will rotate into the left orientation.

### accelcal.py

The FWLoad/accelcal.py script runs just the accel calibration part of
the scripts, skipping the firmware load. This is useful for debugging.

### mav_reference.py and mav_test.py

The FWLoad/mav_reference.py and FWLoad/mav_test.py scripts can be used
to attach to the reference or test boards with MAVProxy, which can be
useful for diagnostics.

### connection.py

The FWLoad/connection.py script can be used to test the complete
connection logic for connecting to the two boards.

## Logging

The system keeps logs of all interactions with each board in the logs
directory. With a subdirectory per day and per run.

## Loading reference firmware

Put the reference board in the test slot in the jig, then use the
jtag.py tool to load the reference firmwares:

> FWLoad/jtag.py --fmu --firmware FW/px4fmuv2_bl_REFERENCE.elf

> FWLoad/jtag.py --fmu --firmware FW/firmware-REFERENCE.elf

> FWLoad/jtag.py --io --firmware FW/px4io_bl.elf

> FWLoad/jtag.py --io --firmware FW/px4io.elf

After each command use "load" to load the firmware

## Calibrating reference board

The reference board needs to have its accelerometers calibrated before
being used. Use this command:

> FWLoad/accelcal.py --reference

the reference board can be in either slot while being calibrated. It
will rotate the board while doing accel calibration. The calibration
assumes the accelerometer is correctly aligned with the board.

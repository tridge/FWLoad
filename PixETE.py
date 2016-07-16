#!/usr/bin/env python

'''
Control ETE gimbal. Based on code by Alan Sanchez, Philip Rowse, Angus Peart
'''

import serial, sys, time


# header: 05 30 30 46 46 57
# Command: #Read: 52 31 #Write: 57 31 
# Address : 5 bytes
# Constant: 30 31 
# Data: 4 bytes


class PixETE(object):
        def __init__(self, port='/dev/ttyUSB0', delay=0.1, yaw_steps=28800, roll_steps=9600):
                if port is not None:
                        self.ser = serial.Serial(port=port,
                                                 baudrate=9600, bytesize=7, parity='E', stopbits=2,
                                                 timeout=None)
                else:
                    self.ser = None       
                 
                self.delay = delay
                self.roll_steps = roll_steps
                self.yaw_steps = yaw_steps
                self.start = "05 30 30 46 46 57 "
                self.cmd = "57 31 "
                self.fixed_bytes ="30 31 "
                self.end   = "00"
                self.run   =      "30 30 32 32 " 
                self.reset =      "30 30 34 34 " 
                self.test_pass =  "30 30 30 31"
                self.test_fail =  "30 30 30 32"
                self.test_wait =  "30 30 30 30"
                self.test_work =  "30 30 30 33"
                self.power_cycle =  "30 30 35 35"
                self.power_cycle_time =  "30 30 35 35"

	   #Addresses:
                self.ADDRESS={'yaw_pos': '44 30 31 31 30 ', #Command dictionary
                              'yaw_speed' :'44 30 31 31 32 ',
                              'roll_pos': '44 30 31 30 30 ',
                              'roll_speed': '44 30 31 30 32 ',
                              'accel': '44 30 32 30 30', 
                              'run':'44 30 31 30 36 ',
                              'reset':'44 30 31 30 36 ',
                              'accel':'44 30 32 30 30 '}  

        def command_hex(self, address, ndec=None):
                '''generate a command'''

        #Fixed Messages:
                #print("1")
                if address == 'run':
                        PixETE_run =   "05 30 30 46 46 57 57 31 44 30 31 30 36 30 31 30 30 32 32"
                        return PixETE_run
                if address == 'reset':
                        return "05 30 30 46 46 57 57 31 44 30 31 30 36 30 31 30 30 34 34"
                if address == 'test_wait':
                        return "05 30 30 46 46 57 57 31 44 30 30 36 30 30 31 30 30 30 30"
                if address == 'test_pass':
                        return "05 30 30 46 46 57 57 31 44 30 30 36 30 30 31 30 30 30 31"
                if address == 'test_fail':
                        return "05 30 30 46 46 57 57 31 44 30 30 36 30 30 31 30 30 30 32"
                if address == 'test_work':
                        return "05 30 30 46 46 57 57 31 44 30 30 36 30 30 31 30 30 30 33"
                if address == 'power_cycle':
                        return "05 30 30 46 46 57 57 31 44 30 31 30 36 30 31 30 30 35 35"
                if address == 'power_cycle_time':
                        return "05 30 30 46 46 57 57 31 44 30 31 38 30 30 31 30 30 35 35"
                #print("1.5")

        #Data Translation:
                nhex = format(ndec, '04X') #convert dec to hex
                HLhex = nhex[0]+nhex[1]+nhex[2]+nhex[3] #removed swap hi and low
                shex = '%s' %HLhex #convert hex to string
                idec = [ord(i) for i in shex]#convert each charachter to ascii decimal
                ihex = [format(i,'02X') for i in idec]#convert each ascii decimal to hex
                data = '%s %s %s %s '% (ihex[0], ihex[1], ihex[2], ihex[3])

		#Print PixETE commands:
                PixETE = self.start+self.cmd+self.ADDRESS[address]+self.fixed_bytes+data+self.end 
                #print("INIT: %s" % self.start)
                #print("READ/WRITE: %s" % self.cmd)
                #print("ADDRESS: %s" % self.ADDRESS[address])
                #print("SPACE: %s" % self.fixed_bytes)
                #print("DATA: %s" % data)
                #print("Would send: %s" % PixETE)
                
                return PixETE #Returns control command for ETE

        def command_bytes(self, address, d=None):
                '''write command to serial port'''
                hex = self.command_hex(address, d)
                if self.ser is None:
                        print("Would send: %s" % hex)
                        return
                bytes = ''
                a = hex.split(' ')
                for v in a:
                        bytes += chr(int(v, base=16))
                #print("sending: %s" % hex)
                time.sleep(self.delay)
                self.ser.write(bytes)

        def position(self, roll, yaw): #specify position for roll and yaw
                '''position at given roll and yaw'''
                yaw_pos = int(yaw * self.yaw_steps / 360.0)
                roll_pos = int(roll * self.roll_steps / 360.0)
                self.command_bytes('roll_pos', roll_pos)
                self.command_bytes('yaw_pos', yaw_pos)
                self.command_bytes('run', 'run')
                

        def accel(self, accel):  #specify accel for roll and yaw
                self.command_bytes('accel', accel)
                      

        def rollspeed(self, roll_speed):  #specify speed for roll
                self.command_bytes('roll_speed', roll_speed)
                

        def yawspeed(self, yaw_speed):  #specify speed for yaw
                self.command_bytes('yaw_speed', yaw_speed)
                


if __name__ == '__main__':
        from argparse import ArgumentParser
        parser = ArgumentParser(description=__doc__)
        
        parser.add_argument("--port", default='/dev/ttyUSB0', help="serial port") #Used to be default=NONE
        parser.add_argument("--reset", action='store_true', help="reset jig")
        parser.add_argument("--delay", type=float, default=0.1, help='command delay')
        parser.add_argument("--yaw-steps", type=int, default=9600, help='yaw step size')
        parser.add_argument("--roll-steps", type=int, default=28800, help='roll step size')
        parser.add_argument("--test_pass" , action='store_true', help='show pass screen')
        parser.add_argument("--test_fail" , action='store_true', help='show fail screen')
        parser.add_argument("--test_work" , action='store_true', help='show work screen')
        parser.add_argument("--test_wait" , action='store_true', help='show wait screen')
        parser.add_argument("--power_cycle" , action='store_true', help='cycle the power')
        parser.add_argument("--power_cycle_time" , action='store_true', help='set the off time duration')
        parser.add_argument("roll", type=float, default=0, nargs='?', help="roll angle (degrees)")
        parser.add_argument("yaw", type=float, default=0, nargs='?', help="yaw angle (degrees)")
        parser.add_argument("--roll_speed", type=int, default=None, help='roll speed (pulses/sec')
        parser.add_argument("--yaw_speed", type=int, default=None, help='yaw speed (pulses/sec)')
        parser.add_argument("--accel", type=int, default=100000, help='Acceleration')


        args = parser.parse_args()

        if args.roll > 360:
                print("Roll too large")
                sys.exit(1)

        if args.yaw > 335:
                print("Yaw too large")
                sys.exit(1)

        ete = PixETE(port=args.port, delay=args.delay, yaw_steps=args.yaw_steps, roll_steps=args.roll_steps)

        if args.reset:
                print("Resetting")
                ete.command_bytes('reset')
                ete.accel(10000)
                sys.exit(0)

        if args.test_pass:
                print("displaying pass")
                ete.command_bytes('test_pass')
                sys.exit(0)

        if args.test_fail:
                print("displaying fail")
                ete.command_bytes('test_fail')
                sys.exit(0)

        if args.test_work:
                print("displaying work")
                ete.command_bytes('test_work')
                sys.exit(0)

        if args.test_wait:
                print("displaying wait")
                ete.command_bytes('test_wait')
                sys.exit(0)

        if args.power_cycle:
                print("power cycling")
                ete.command_bytes('power_cycle')
                sys.exit(0)

        if args.power_cycle_time:
                print("setting off time")
                ete.command_bytes('power_cycle_time')
                

        if args.roll_speed:
            print("changing roll speed")
            ete.rollspeed(args.roll_speed)
            

        if args.yaw_speed:
            print("changing yaw speed")
            ete.yawspeed(args.yaw_speed)

        if args.accel:
            print("changing accel")
            ete.accel(args.accel)   
        
        ete.position(args.roll, args.yaw)


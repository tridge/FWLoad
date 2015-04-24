#!/usr/bin/env python

'''
Control PTE gimbal. Based on code by Alan Sanchez
'''

import serial, sys, time


class PixPTE(object):
        def __init__(self, port=None, delay=0.1, yaw_steps=38800, roll_steps=9600):
                if port is not None:
                        self.ser = serial.Serial(port=port,
                                                 baudrate=9600, bytesize=7, parity='E', stopbits=2,
                                                 timeout=None)
                else:
                        self.ser = None
                self.delay = delay
                self.roll_steps = roll_steps
                self.yaw_steps = yaw_steps
                self.start = "02 "
                self.cmd = "31 "
                self.fixed_bytes ="30 32 "
                self.end   = "03"
                self.run   =      "31 30 44 34 " #D106 this is the same as reset. Double check!!!
                self.reset =      "31 30 44 34 " 
		#Addresses
                # yaw_pos =      "31 30 43 38 " #D100
                # yaw_speed =    "31 30 43 43 " #D102
                # roll_pos =     "31 30 44 43 " #D110 
                # roll_speed =   "31 30 45 30 " #D112 
                # run =          "31 30 44 34"  #D106
                # reset =        "31 30 44 34"  #D106 
                # accel = "31 30 44 38"  #D108 Need to verify this is indeed acceleration
                self.ADDRESS={'yaw_pos': '31 30 43 38 ',
                              'yaw_speed' :'31 30 43 43 ',
                              'roll_pos': '31 30 44 43 ',
                              'roll_speed': '31 30 45 30 ', 
                              'run':'31 30 44 34 ',
                              'reset':'31 30 44 34 ',
                              'accel':'31 30 44 34 '}  #Command dictionary

        def command_hex(self, address, ndec=None):
                '''generate a command'''
		#-- Data Translation ------------------
                if address == 'run':
                        PixETE_run =   "02 31 31 30 44 34 30 32 32 32 30 30 03 33 33"
                        return PixETE_run
                if address == 'reset':
                        return "02 31 31 30 44 34 30 32 34 34 30 30 03 33 37"

                nhex = format(ndec, '04X') #convert dec to hex
                HLhex = nhex[2]+nhex[3]+nhex[0]+nhex[1] #swap hi and low
                shex = '%s' %HLhex #convert hex to string
                idec = [ord(i) for i in shex]#convert each charachter to ascii decimal
                ihex = [format(i,'02X') for i in idec]#convert each ascii decimal to hex

                data = '%s %s %s %s '% (ihex[0], ihex[1], ihex[2], ihex[3])
		#checks 
                # print "Your decimal number is %d" %ndec
                # print "Your hexadecimal number is %s" %nhex
                # print "Your hexadecimal H <> L number is %s" %HLhex
                # print shex
                # print idec
                # print "Your hex ascii decimal is"
                # print ihex
                
		#-- Checksum calculation --------------
		#cksum = [(int(g,16) for g in ETEchkArray)]
                ETEchk = self.cmd+self.ADDRESS[address]+self.fixed_bytes+data+self.end
                ETE = ETEchk.split(' ') #split ETEchk into a string array
                cksum = format((int(ETE[0],16)+int(ETE[1],16)+int(ETE[2],16)+int(ETE[3],16)+
                                int(ETE[4],16)+int(ETE[5],16)+int(ETE[6],16)+int(ETE[7],16)+int(ETE[8],16)+
                                int(ETE[9],16)+int(ETE[10],16)+int(ETE[11],16)),'02X')
		#print "this is the checksum hex %s" %cksum
                l = [ord(i) for i in cksum] #convert each character to ascii decimal
                o = [format(n,'02X') for n in l] #convert each ascii decimal to hex
                size = len(o)
                ck2 = ' %s' %o[size-1]
                ck1 = ' %s' %o[size-2]
		#Print PixETE commands:
                PixETE = self.start+self.cmd+self.ADDRESS[address]+self.fixed_bytes+data+self.end+ck1+ck2 
                return PixETE

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
                print("sending: %s" % hex)
                time.sleep(self.delay)
                self.ser.write(bytes)

        def position(self, roll, yaw):
                '''position at given roll and yaw'''
                yaw_pos = int(yaw * self.yaw_steps / 360.0)
                roll_pos = int(roll * self.roll_steps / 360.0)
                self.command_bytes('roll_pos', roll_pos)
                self.command_bytes('yaw_pos', yaw_pos)
                self.command_bytes('run', 'run')

if __name__ == '__main__':
        from argparse import ArgumentParser
        parser = ArgumentParser(description=__doc__)
        
        parser.add_argument("--port", default=None, help="serial port")
        parser.add_argument("--reset", action='store_true', help="reset jig")
        parser.add_argument("--delay", type=float, default=0.1, help='command delay')
        parser.add_argument("--yaw-steps", type=int, default=38800, help='yaw step size')
        parser.add_argument("--roll-steps", type=int, default=9600, help='roll step size')
        
        parser.add_argument("roll", type=float, default=0, help="roll angle (degrees)")
        parser.add_argument("yaw", type=float, default=0, help="yaw angle (degrees)")
        args = parser.parse_args()

        if args.roll > 360:
                print("Roll too large")
                sys.exit(1)

        if args.yaw > 338:
                print("Yaw too large")
                sys.exit(1)

        pte = PixPTE(port=args.port, delay=args.delay, yaw_steps=args.yaw_steps, roll_steps=args.roll_steps)

        if args.reset:
                print("Resetting")
                pte.command_bytes('reset')
                sys.exit(0)

        pte.position(args.roll, args.yaw)

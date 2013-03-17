#!/usr/bin/python

import sys
import getopt
import serial

class SX(object):
    def __init__(self, ser):
        self.ser = ser

    def set_channel(self, channel, value):
        self.ser.write('=' + chr(channel) + chr(value))

    def get_channel(self, channel):
        self.ser.write('?' + chr(channel))
        return ord(self.ser.read(1))
        
    def get_state(self):
        self.ser.write('#')
        return ord(self.ser.read(1))
        
        
if __name__ == '__main__':
    def printUsage(msg=''):
        print 'Usage: %s bla foo error: %s' % (sys.argv[0], msg)
    
    def parseByteValue(s):
        x = int(s)
        if x not in range(256):
            raise ValueError('value not in byte range')
        return x
        
        
    OP_READ = 1
    OP_WRITE = 2
    OP_MONITOR = 3
    
    (opts, args) = getopt.gnu_getopt(sys.argv, 'd:r:w:m:', [])
    opts = dict(opts)
    
    operation = 0
    channel = -1
    value = -1
    
    if '-d' in opts.keys():
        ser_dev = opts['-d']
    else:
        printUsage()
        sys.exit(1)
        
    if '-w' in opts.keys():
        if len(args) != 2:
            printUsage('Please specify a value to write.')
            sys.exit(1)

        value = parseByteValue(args[1])
        channel = parseByteValue(opts['-w'])
        operation = OP_WRITE
    elif '-r' in opts.keys():
        channel = parseByteValue(opts['-r'])
        operation = OP_READ
    
    ser = serial.Serial(ser_dev, 9600)
    sx = SX(ser)
    
    if operation == OP_READ:
        print sx.get_channel(channel)
    elif operation == OP_WRITE:
        sx.set_channel(channel, value)
    
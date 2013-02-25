import serial
import time
import sys

def testBit(int_type, offset):
    mask = 1 << offset
    if (int_type & mask):
        return 1
    else:
        return 0

def printbyte(inp):
    byte = ''
    byte += str(testBit(inp, 0))
    byte += str(testBit(inp, 1))
    byte += str(testBit(inp, 2))
    byte += str(testBit(inp, 3))
    byte += str(testBit(inp, 4))
    byte += str(testBit(inp, 5))
    byte += str(testBit(inp, 6))
    byte += str(testBit(inp, 7))
    return byte

ser = serial.Serial('/dev/tty.usbmodem12341',9600)
buf = ''
sx = [0] * (8*16+10000)

printbytes = 2
while True:
	buf += ser.read(1)
	if buf[-1] == '\n' and len(buf) >= printbytes:
		for i in range(0, printbytes):
			sys.stdout.write(printbyte(ord(buf[i])) + ' ')
		sys.stdout.write('\n')
		buf = ''


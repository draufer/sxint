import serial
import time
import sys

def testBit(int_type, offset):
    if (int_type & (1<<offset)):
        return 1
    return 0

def printbyte(inp):
    byte = ''
    for bit in range(0, 8):
        byte += str(testBit(inp, bit))
    return byte

ser = serial.Serial('/dev/tty.usbmodem12341',9600)
buf = ''

printbytes = 3
while True:
	buf += ser.read(1)
	if buf[-1] == '\n' and len(buf) >= printbytes:
		for i in range(0, printbytes):
			sys.stdout.write(printbyte(ord(buf[i])) + ' ')
		sys.stdout.write('\n')
		buf = ''


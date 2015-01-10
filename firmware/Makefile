# sx.c
#
#  Created on: 23.02.2013
#  Changed on: 23.02.2013
#  Version:    0.1
#  Copyright:  Jan Seiffert, Frederic Endner-Duehr, Dirk Raufer
#
# SX functionality inspired from the original Arduino library of Michael Blank:
#  http://www.oscale.net/selectrix
#
# USB code taken from PJRC.COM, LLC
#  http://www.pjrc.com/teensy/usb_serial.html
#  The USB code was modifed by us for our specific needs.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#				 usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#				 uploading to the AVR and the interface where this hardware
#				 is connected. We recommend that you leave it undefined and
#				 add settings like this to your ~/.avrduderc file:
#					default_programmer = "stk500v2"
#					default_serial = "avrdoper"
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE	   = atmega32u4
CLOCK	   = 16000000
PROGRAMMER = -c avr109 -P /dev/tty.usbmodem411
#-c stk500v2 -P avrdoper
OBJECTS	   = main.o usb_serial.o sx.o
FUSES	   = #-U hfuse:w:0xd9:m -U lfuse:w:0x24:m
#AVRDUDE_BIN = ./flash.py
AVRDUDE_BIN = avrdude
CFLAGS = 
CFLAGS += -D CMDBUF=1

# Causes the program to break!
#CFLAGS += -D REVOKE_SYNC_ON_USB_INTERRUPT=1

# ATMega8 fuse bits used above (fuse bits for other devices are different!):
# Example for 8 MHz internal oscillator
# Fuse high byte:
# 0xd9 = 1 1 0 1   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
#		 ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#		 | | | |   | +-------- BOOTSZ1
#		 | | | |   +---------- EESAVE (set to 0 to preserve EEPROM over chip erase)
#		 | | | +-------------- CKOPT (clock option, depends on oscillator type)
#		 | | +---------------- SPIEN (if set to 1, serial programming is disabled)
#		 | +------------------ WDTON (if set to 0, watchdog is always on)
#		 +-------------------- RSTDISBL (if set to 0, RESET pin is disabled)
# Fuse low byte:
# 0x24 = 0 0 1 0   0 1 0 0
#		 ^ ^ \ /   \--+--/
#		 | |  |		  +------- CKSEL 3..0 (8M internal RC)
#		 | |  +--------------- SUT 1..0 (slowly rising power)
#		 | +------------------ BODEN (if 0, brown-out detector is enabled)
#		 +-------------------- BODLEVEL (if 0: 4V, if 1: 2.7V)
#
# For computing fuse byte values for other devices and options see
# the fuse bit calculator at http://www.engbedded.com/fusecalc/


# Tune the lines below only if you know what you are doing:

AVRDUDE = $(AVRDUDE_BIN) $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) $(CFLAGS)

# symbolic targets:
all:	main.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash
#fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID main.hex

clean:
	rm -f main.hex main.elf $(OBJECTS)

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=$(DEVICE) main.elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm: main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c

# file deps
usb_serial.o: usb_serial.c usb_serial.h helper.h
sx.o: sx.c sx.h helper.h

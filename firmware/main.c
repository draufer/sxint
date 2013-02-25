/*
 * sx.c
 *
 *  Created on: 23.02.2013
 *  Changed on: 23.02.2013
 *  Version:    0.1
 *  Copyright:  Jan Seiffert, Frederic Endner-Duehr, Dirk Raufer
 *
 * SX functionality inspired from the original Arduino library of Michael Blank:
 *  http://www.oscale.net/selectrix
 *
 * USB code taken from PJRC.COM, LLC
 *  http://www.pjrc.com/teensy/usb_serial.html
 *  The USB code was modifed by us for our specific needs.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usb_serial.h"
#include "sx.h"

#define LED_CONFIG  (DDRD |= (1<<6))
#define LED_ON      (PORTD |= (1<<6))
#define LED_OFF     (PORTD &= ~(1<<6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

uint8_t recv_str(char *buf, uint8_t size);
void parse_and_execute_command(const char *buf, uint8_t num);

#if 1
// SX Base Function (Read) Implementation Test
int main(void)
{
    uint16_t counter = 0;
	cli();
	DDRC |= (1<<PC7);
	CPU_PRESCALE(0);
    sx_init();
	usb_init();
	sei();
	while (1) {
		sx_tick();
        
		//PORTC |= (1<<PC7);
        //if (sx_nxt_bit_num == 1000) {
		    //int n = usb_serial_getchar();
        if (counter > 50000) {
            usb_serial_putchar(sx_get_state());
		    usb_serial_putchar(sx_get_channel(1));
            //usb_serial_putchar(sx_get(179));
            //usb_serial_putchar(sx_get(180));
            usb_serial_putchar('\n');
            counter = 0;
            //}
        }
        counter++;
	}

}

#else

// Basic command interpreter for controlling port pins
int main(void)
{
	char buf[32];
	uint8_t n;

	// set for 16 MHz clock, and turn on the LED
	CPU_PRESCALE(0);
	LED_CONFIG;
	LED_ON;

	// initialize the USB, and then wait for the host
	// to set configuration.  If the Teensy is powered
	// without a PC connected to the USB port, this 
	// will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;
	_delay_ms(1000);

	while (1) {
		// wait for the user to run their terminal emulator program
		// which sets DTR to indicate it is ready to receive.
		while (!(usb_serial_get_control() & USB_SERIAL_DTR)) /* wait */ ;

		// discard anything that was received prior.  Sometimes the
		// operating system or other software will send a modem
		// "AT command", which can still be buffered.
		usb_serial_flush_input();

		// print a nice welcome message
		usb_serial_write_str_PGM(PSTR("\r\nTeensy USB Serial Example, "
					"Simple Pin Control Shell\r\n\r\n"
					"Example Commands\r\n"
					"  B0?   Read Port B, pin 0\r\n"
					"  C2=0  Write Port C, pin 1 LOW\r\n"
					"  D6=1  Write Port D, pin 6 HIGH  (D6 is LED pin)\r\n\r\n"));

		// and then listen for commands and process them
		while (1) {
			usb_serial_write_str_PGM(PSTR("> "));
			n = recv_str(buf, sizeof(buf));
			if (n == 255) break;
			usb_serial_write_str_PGM(PSTR("\r\n"));
			parse_and_execute_command(buf, n);
		}
	}
}
#endif

// Receive a string from the USB serial port.  The string is stored
// in the buffer and this function will not exceed the buffer size.
// A carriage return or newline completes the string, and is not
// stored into the buffer.
// The return value is the number of characters received, or 255 if
// the virtual serial connection was closed while waiting.
//
uint8_t recv_str(char *buf, uint8_t size)
{
	int16_t r;
	uint8_t count=0;

	while (count < size) {
		r = usb_serial_getchar();
		if (r != -1) {
			if (r == '\r' || r == '\n') return count;
			if (r >= ' ' && r <= '~') {
				*buf++ = r;
				usb_serial_putchar(r);
				count++;
			}
		} else {
			if (!usb_configured() ||
					!(usb_serial_get_control() & USB_SERIAL_DTR)) {
				// user no longer connected
				return 255;
			}
			// just a normal timeout, keep waiting
		}
	}
	return count;
}

// parse a user command and execute it, or print an error message
//
void parse_and_execute_command(const char *buf, uint8_t num)
{
	uint8_t port, pin, val;

	if (num < 3) {
		usb_serial_write_str_PGM(PSTR("unrecognized format, 3 chars min req'd\r\n"));
		return;
	}
	// first character is the port letter
	if (buf[0] >= 'A' && buf[0] <= 'F') {
		port = buf[0] - 'A';
	} else if (buf[0] >= 'a' && buf[0] <= 'f') {
		port = buf[0] - 'a';
	} else {
		usb_serial_write_str_PGM(PSTR("Unknown port \""));
		usb_serial_putchar(buf[0]);
		usb_serial_write_str_PGM(PSTR("\", must be A - F\r\n"));
		return;
	}
	// second character is the pin number
	if (buf[1] >= '0' && buf[1] <= '7') {
		pin = buf[1] - '0';
	} else {
		usb_serial_write_str_PGM(PSTR("Unknown pin \""));
		usb_serial_putchar(buf[0]);
		usb_serial_write_str_PGM(PSTR("\", must be 0 to 7\r\n"));
		return;
	}
	// if the third character is a question mark, read the pin
	if (buf[2] == '?') {
		// make the pin an input
		*(uint8_t *)(0x21 + port * 3) &= ~(1 << pin);
		// read the pin
		val = *(uint8_t *)(0x20 + port * 3) & (1 << pin);
		usb_serial_putchar(val ? '1' : '0');
		usb_serial_write_str_PGM(PSTR("\r\n"));
		return;
	}
	// if the third character is an equals sign, write the pin
	if (num >= 4 && buf[2] == '=') {
		if (buf[3] == '0') {
			// make the pin an output
			*(uint8_t *)(0x21 + port * 3) |= (1 << pin);
			// drive it low
			*(uint8_t *)(0x22 + port * 3) &= ~(1 << pin);
			return;
		} else if (buf[3] == '1') {
			// make the pin an output
			*(uint8_t *)(0x21 + port * 3) |= (1 << pin);
			// drive it high
			*(uint8_t *)(0x22 + port * 3) |= (1 << pin);
			return;
		} else {
			usb_serial_write_str_PGM(PSTR("Unknown value \""));
			usb_serial_putchar(buf[3]);
			usb_serial_write_str_PGM(PSTR("\", must be 0 or 1\r\n"));
			return;
		}
	}
	// otherwise, error message
	usb_serial_write_str_PGM(PSTR("Unknown command \""));
	usb_serial_putchar(buf[0]);
	usb_serial_write_str_PGM(PSTR("\", must be ? or =\r\n"));
}



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

#define CMDBUF_SIZE 32

enum serial_state
{
    WAIT,
    READ,
    WRITE_ADDRESS,
    WRITE_VALUE,
//	READ_ALL
} __attribute__((__packed__));

uint8_t recv_str(char *buf, uint8_t size);
void parse_and_execute_command(const char *buf, uint8_t num);

/* Create a command buffer */
int cmdbuf_start = 0; /* Start position in cmdbuf ring buffer */
int cmdbuf_length = 0; /* length of cmdbuf data */
char cmdbuf[CMDBUF_SIZE][3];

// ringbuffer for command queueing

int cmdbuf_push(uint8_t cmd, uint8_t value1, uint8_t value2) {
	if (cmdbuf_length < (CMDBUF_SIZE)) {
		int pos = (cmdbuf_start + cmdbuf_length) % CMDBUF_SIZE;
		cmdbuf[pos][0] = cmd;
		cmdbuf[pos][1] = value1;
		cmdbuf[pos][2] = value2;
		cmdbuf_length += 1;
		return 0;
	}
	return -1;
}

int cmdbuf_pop(uint8_t* cmd, uint8_t* value1, uint8_t* value2) {
	if (cmdbuf_length > 0) {
		*cmd = cmdbuf[cmdbuf_start][0];
		*value1 = cmdbuf[cmdbuf_start][1];
		*value2 = cmdbuf[cmdbuf_start][2];
		cmdbuf_start = (cmdbuf_start + 1) % CMDBUF_SIZE;
		cmdbuf_length -= 1;
		return 0;
	}
	return -1;
}

/* draufer */
void switch_to_sx (void) {
	cli();
	usb_disable_interrupts();
	sx_revoke_sync();
	sx_enable_interrupts();
	sei();
}

void switch_to_usb (void) {
	cli();
	sx_disable_interrupts();
	usb_enable_interrupts();
	sei();
}


// SX Base Function (Read) Implementation Test
int main(void)
{
    enum serial_state serial_command_state = WAIT;
    int16_t r;
    uint8_t ser_addr = 0;
	uint8_t v, cmd, value1, value2;
    cli();
    DDRC |= _BV(PC7)|_BV(PC6);
	DDRD |= _BV(PD7);
    CPU_PRESCALE(0);
    sx_init();
    usb_init();
	switch_to_usb();
    sei();
    while (1) {
        //sx_tick();
        
        /* Serial Communication State Machine */
        r = usb_serial_getchar();
        if (r != -1) {
            switch(serial_command_state)
            {
                case WAIT:
                    if (r == '?')
                        serial_command_state = READ;
                    else if (r == '=')
                        serial_command_state = WRITE_ADDRESS;
                    else if (r == '#') {
						cmdbuf_push(r, 0, 0);
					}
					//else if (r == '*')
					//	serial_command_state = READ_ALL;
                break;
                
                case READ:
					cmdbuf_push('?', r, 0);
                    serial_command_state = WAIT;
                break;
 
                //case READ_ALL:
                //	usb_serial_write(sx_get_data_pointer() ,/*sx_get_data_size()*/ 4);
				//	usb_serial_putchar('\n');
				//	serial_command_state = WAIT;
                //break;
                
                case WRITE_ADDRESS:
                    ser_addr = (uint8_t)r;
                    serial_command_state = WRITE_VALUE;
                break;
                
                case WRITE_VALUE:
					cmdbuf_push('=', ser_addr, r);
                    serial_command_state = WAIT;
                break;
            }			
        }

		// Process command buffer
		if (cmdbuf_length > 0) {
			switch_to_sx();
			while (cmdbuf_length > 0) {
				
				if (sx_tick() == SX_CONTROL_SYNC) {
					cmdbuf_pop(&cmd, &value1, &value2);
					switch(cmd){
						case '?':
							v = sx_get_channel(value1);
							switch_to_usb();
							usb_serial_putchar(v);
							switch_to_sx();
						break;
						case '=':
							sx_set_channel(value1, value2);
							wait_for_full_transmission_init();
							while(!wait_for_full_transmission())
								/* nop */;
							switch_to_usb();
							usb_serial_putchar('!');
							switch_to_sx();
						break;				
						case '#':
							v = sx_get_state();
							switch_to_usb();
							usb_serial_putchar(v);
							switch_to_sx();
						break;
					}
				} // else nop and wait for sync
			}
			switch_to_usb();
		}
    }
}

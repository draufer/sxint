/*
 * sx.h
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

#ifndef SX_H_
#define SX_H_

#include <avr/io.h>
#include <inttypes.h>

// define arduino pins

/* Port D, Pin 0, must be INT0!! */
#define SX_CLK   PD0
#define SX_DATA  PD4
//#define SCOPE    3
#define SX_WRITE PD1

#define UINT8_T_BIT (sizeof(uint8_t)*CHAR_BIT)

#define SX_BITCOUNT 12
#define SX_BASE_FRAME_INFO_CNT 7
#define SX_BASE_FRAME_LEN (1+SX_BASE_FRAME_INFO_CNT)
#define SX_BASE_FRAME_BYTES ((SX_BASE_FRAME_LEN * SX_BITCOUNT)/UINT8_T_BIT)
#define SX_BASE_FRAME_CNT 16
#define SX_FRAME_LEN (SX_BASE_FRAME_CNT*SX_BASE_FRAME_LEN)
#define SX_FRAME_BIT (SX_FRAME_LEN * SX_BITCOUNT)
#define SX_FRAME_BYTES (SX_FRAME_BIT/UINT8_T_BIT)
#define SX_CHAN_MAX (SX_BASE_FRAME_CNT*SX_BASE_FRAME_INFO_CNT)

/*
 * ! ! ! Attention ! ! !
 * the internal state is not really for consumption, put practical
 * for debugging.
 * Some real life use cases may show the path to a canned interface
 * around it.
 */
enum sx_internal_state
{
	SX_SEARCH_SYNC,
	SX_WAIT_FOR_FRAME_AFTER_SYNC_INIT,
	SX_WAIT_FOR_FRAME_AFTER_SYNC,
	SX_SEARCH_BASE_FRAME0,
	SX_WAIT_FOR_FRAME_AFTER_BASE_FRAME0_INIT,
	SX_WAIT_FOR_FRAME_AFTER_BASE_FRAME0,
	SX_CONTROL_SYNC,
} __attribute__((__packed__));
enum sx_internal_state sx_get_state(void);

void    sx_init(void);
uint8_t sx_get_channel(uint8_t);
void    sx_set_channel(uint8_t, uint8_t);
void    sx_tick(void);

#endif /* SX_H_ */

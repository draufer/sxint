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


/*
	Basic SX Setup (for maerklin Z gauge):

	Trix Control Handy + Booster/Control Station
	Arduino Micro attached (Mini DIN Selectrix male plug in Trix Control
	Station) on "Steckboard" to the Selectrix bus.

	The Trix components are powered with 9V.

	Mini DIN Cable pin description:
	    [Px1]  [Px2]  [Sx-T1]

	[Px-EN]  (Gap)  [GND]  [Sx-T0]

	        [Sx-D]  [+B]

	Legend:
	Px1: Power BUS1 (not needed here) [green cable]
	Px2: Power BUS2 (not needed here) [?]
	Sx-T1: Data coming from central station. [grey cable]
		SX_DATA == PD4 == Digital Pin 4
	Px-EN: unknown. (not needed here) [?]
	Gap: A gap in the plug. [no cable]
	GND: Ground. Needs to be connected with the Arduino GND pin. [red cable]
	Sx-T0: Clock coming from central station. [pink cable]
		SX_CLK == PD0 == Digital Pin 3. Important: It ust be this PIN! (INT0)
	Sx-D: Data from the Arduino to the central Station. [blue cable]
		SX_WRITE == PD1 == Digital Pin 2. (Maybe there has to be a pullup resistor active?)
	+B: Beware! 20 Volts. Do not use! [yellow]

	External sources:
	- http://www.mttm.de/Internals.htm

	Selectrix Data Signal Layout

	channel: 12 bits (either data channel or sync channel)
	base frame: 8 channels (1 sync channel + 7 data channels)
	frame: 16 base frames

	Channel bit layout:

	         Time 0 ----------------------> t
	sync channel:  0  0  0  1  X  1 B3 B2  1 B1 B0  1 (X = Power on rails 0 or 1) B0(lsb) - B3(msb) = sync channel index 0-15
	data channel: B7 B6  1 B5 B4  1 B3 B2  1 B1 B0  1 (B0 - B7 = Payload)

	Data channel position in frame formula:

	computation example:
	channel      = 1 (lokomotive 01)           1
	t_frame      = channel / 16                0
	t_base       = channel % 16                1
	data_channel = 7 - t_frame                 7
	base_frame   = 15 - t_base                 14
	bit_addr     = base_frame * 8 * 12 + data_channel * 12 = 1428
	

*/

#include <limits.h>
#include <string.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "sx.h"
#include "helper.h"

/*
 * Sync Channel Format
 * 0  0  0  1  X   1  B3  B2  1  B1  B0  1 == sync frame of 12 bits
 *
 * SX Timing
 *    1   Bit             50 µs
 *    1   Kanal          600 µs (= 12 Bit)
 *    1   Grundrahmen    ca. 4,8 ms
 *    1   Gesamtrahmen   ca.  80 ms (= 16 Grundrahmen)
 */
#define PADDING_BIT_MSK 0x924
#define PADDING_BIT_MSK_N ((~PADDING_BIT_MSK)&0xfff)

/**************** types ********************************/

/* data type to hold an bit-unaligned channel */
#if _GNUC_PREREQ(4,7)
typedef __uint24 sx_wild_channel;
# define ARR_EXTRA_SP 1
#else
typedef uint32_t sx_wild_channel;
# define ARR_EXTRA_SP 2
#endif

/****************** vars ******************************/

/* raw bit arrays */

/* selectrix data from central station */
static uint8_t sx_raw_bits_in[DIV_ROUNDUP(SX_FRAME_BIT, UINT8_T_BIT) + ARR_EXTRA_SP]; /* use more bytes to help with overread and overwrite */

/* data direction bit mask: 1 means to write our data to the selectrix bus (SX-D) */
static uint8_t sx_raw_bits_out_dir[DIV_ROUNDUP(SX_FRAME_BIT, UINT8_T_BIT) + ARR_EXTRA_SP];

/* outgoing data, to be written to SX-D */
static uint8_t sx_raw_bits_out_data[DIV_ROUNDUP(SX_FRAME_BIT, UINT8_T_BIT) + ARR_EXTRA_SP];

/* interrupt position */
volatile uint16_t sx_nxt_bit_num;
volatile uint8_t sx_frame_gen_cnt;

/* frame wait */
static struct {
	uint16_t bit;
	uint8_t gen;
} sx_frame_wait, transmission_wait;
	


/* internal state */
static enum sx_internal_state internal_state;

/**************************** init **********************/

/* initialize function
 * initialize pins and variables
 */
void sx_init()
{
	DDRD &= ~_BV(SX_CLK); /* SX-Clock as Input */
	DDRD &= ~_BV(SX_DATA); /* SX-Data as Input */
	DDRD &= ~_BV(SX_WRITE); /* SX-Output as High impedance^w^w Input */
#if 0
	PORTD |= _BV(SX_DATA); /* enable pullup resistors */
	PORTD |= _BV(SX_CLK); /* enable pullup resistors */
#endif
	PORTD &= ~_BV(SX_WRITE); /* disable pullup resistors */

	/* bring raw bits array in start state */
	memset(sx_raw_bits_in, 0xff, sizeof(sx_raw_bits_in));
	/* set all out bits to direction input */
	memset(sx_raw_bits_out_dir, 0x0, sizeof(sx_raw_bits_out_dir));
	/* set all out bits to 0/pullups disabled */
	memset(sx_raw_bits_out_data, 0x0, sizeof(sx_raw_bits_out_data));

	/* reset vars */
	sx_nxt_bit_num = 0;
	sx_frame_gen_cnt = 0;
	internal_state = SX_SEARCH_SYNC;

	/* Set interrupt control register */
/*************/
	/*
	 * Bad diagrams FTW....
	 *
	 * It is NOT clear when data on the wire will be valid/sampled by which party.
	 *
	 * From lurking and a LOT of reading, it seams that:
	 *
	 * 1) on falling edge the main station sets up the data
	 *   -> every other writer should also set up it's data to have it
	 *      ready for the rising edge.
	 * 2) on rising edge data will be read
	 *   -> every one and the main Station sample the data
	 *
	 * This looks to be the rules of SX
	 */
/*************/
	
	/* rising and falling edge creates Interrupt */
	EICRA  = (EICRA & ~(_BV(ISC00)|_BV(ISC01))) | _BV(ISC00);
	/* activate external interupt */
	EIMSK |= _BV(INT0);
	
	/* debug led lighting means: no sync */
	PORTC |= _BV(PC6);
	/* debug led signaling sx int0 */
	//PORTD |= _BV(PD7);
}

/*************************** functions ********************/

enum sx_internal_state sx_get_state(void)
{
	return internal_state;
}

//#ifdef REVOKE_SYNC_ON_USB_INTERRUPT
void sx_revoke_sync(void)
{
	sx_nxt_bit_num = 0;
	sx_frame_gen_cnt = 0;
	internal_state = SX_SEARCH_SYNC;
}
//#endif

/* sx_get_startbit (uint8_t channel)
 *
 * compute the first bit (absolute position) of data channel "channel"
 * to fetch the data from sx_raw_bits_in[]
 */
static noinline uint16_t GCC_ATTR_OPTIMIZE("O3")  sx_get_startbit(uint8_t channel)
{
	/* naming:
	 *
	 * Number of base frames: SX_BASE_FRAME_CNT (16)
	 * Number of data channels per base frame: SX_BASE_FRAME_INFO_CNT (7)
	 * Number of bits in an SX byte (data channel): SX_BITCOUNT (12)
	 *
	 * computation example:
	 * channel      = 1 (lokomotive 01)           1
	 * t_frame      = channel / 16                0
	 * t_base       = channel % 16                1
	 * data_channel = 7 - t_frame                 7
	 * base_frame   = 15 - t_base                 14
	 * bit_addr     = base_frame * 8 * 12 + data_channel * 12 = 1428
	 */
	return (((SX_BASE_FRAME_CNT - 1) - (channel % SX_BASE_FRAME_CNT))) * 8 * SX_BITCOUNT
	       + (SX_BASE_FRAME_INFO_CNT - (channel / SX_BASE_FRAME_CNT)) * SX_BITCOUNT;
}

/* expand data to wire bit pattern */
static uint16_t sx_data_expand(uint8_t data)
{
	uint16_t res;
	/* spread out date bytes in buffer */
	res  =            (data & (0x3 << 0))  << 0;
	res |=            (data & (0x3 << 2))  << 1;
	res |=            (data & (0x3 << 4))  << 2;
	res |= ((uint16_t)(data & (0x3 << 6))) << 3;
	/* add the padding bits */
	return res | PADDING_BIT_MSK;
}

/* extract data bits from wire bit pattern */
static uint8_t sx_data_collapse(uint16_t wdata)
{
	uint8_t res;
#ifdef I_WANT_CLEAN
	/* this code lets the compiler get a little freaky: 33 instructions + extra register
	 * which means more push & pop */
	res  = (((wdata) & (0x3 << 0)) >> 0);
	res |= (((wdata) & (0x3 << 3)) >> 1);
	res |= (((wdata) & (0x3 << 6)) >> 2);
	res |= (((wdata) & (0x3 << 9)) >> 3);
#else
	/* if we reach for asm, use instructions the compiler will never create */
	asm (
		"bst	%A1, 0\n\t"
		"bld	%0, 0\n\t"
		"bst	%A1, 1\n\t"
		"bld	%0, 1\n\t"
		"bst	%A1, 3\n\t"
		"bld	%0, 2\n\t"
		"bst	%A1, 4\n\t"
		"bld	%0, 3\n\t"
		"bst	%A1, 6\n\t"
		"bld	%0, 4\n\t"
		"bst	%A1, 7\n\t"
		"bld	%0, 5\n\t"
		"bst	%B1, 1\n\t"
		"bld	%0, 6\n\t"
		"bst	%B1, 2\n\t"
		"bld	%0, 7\n\t"
		: /* %0 */ "=&r" (res)
		: /* %1 */ "r" (wdata)
	);
#endif
	return res;
}

uint8_t GCC_ATTR_OPTIMIZE("O3") sx_get_channel(uint8_t channel)
{
	sx_wild_channel bytebuffer; /* temporary buffer for read in data */
	uint16_t bitpos;            /* the postion of the first bit in the buffer */
	uint8_t bit_offset;         /* the offset from the beginning of the first byte it is stored in */
	uint8_t start_byte;         /* the first byte that contains the payload */		

	/* get channel bit position */
	bitpos = sx_get_startbit(channel);
	/* precalc byte stuff */
	bit_offset = bitpos % UINT8_T_BIT;
	start_byte = bitpos / UINT8_T_BIT;
	/* get data */
#ifdef I_WANT_CLEAN
	bytebuffer  =                   sx_raw_bits_in[start_byte + 0]  << (0 * UINT8_T_BIT);
	bytebuffer |= ((uint16_t)       sx_raw_bits_in[start_byte + 1]) << (1 * UINT8_T_BIT);
	bytebuffer |= ((sx_wild_channel)sx_raw_bits_in[start_byte + 2]) << (2 * UINT8_T_BIT);
#else
	/* avr has no alignment, is little endian, so data comes the right way from mem, and as long
	 * as the compiler does not start to whine like a sissy about aliasing, this generates much better
	 * code, till someone subregs avr cleanly...
	 */
	bytebuffer = *((sx_wild_channel *)(sx_raw_bits_in + start_byte));
#endif
	/* shift it down on a byte boundery, extract the data bits from the padding */
	return sx_data_collapse(bytebuffer >> bit_offset);
}

void GCC_ATTR_OPTIMIZE("O3") sx_set_channel(uint8_t channel, uint8_t data)
{
	sx_wild_channel bytemask, bytebuffer;
	uint16_t bytebuff_s;
	uint16_t bitpos;            /* the postion of the first bit in the buffer */
	uint8_t bit_offset;         /* the offset from the beginning of the first byte it is stored in */
	uint8_t start_byte;         /* the first byte that contains the payload */

	/* spread out date bytes in buffer */
	bytebuff_s = sx_data_expand(data);
	/* get channel bit position */
	bitpos     = sx_get_startbit(channel);
	/* precalc byte stuff */
	bit_offset = bitpos % UINT8_T_BIT;
	start_byte = bitpos / UINT8_T_BIT;

	/* init vars */
	bytemask   = PADDING_BIT_MSK_N; /* b011011011011 */
	bytebuffer = bytebuff_s;
	/* shift the byte mask up to the right offset */
#ifdef I_WANT_CLEAN
	bytebuffer <<= bit_offset;
	bytemask   <<= bit_offset;
#else
	/* the controller only can to single bit shifts, help compiler to do both shift
	 * in one loop, saving one time loop overhead */
	while (bit_offset--) {
		bytebuffer <<= 1;
		bytemask   <<= 1;
	}
#endif
	/* force compiler to execute shift from above before the following operations */
	asm("" : : "r" (bytebuffer));

// TODO: How to prevent a half write?
	/* if the interrupt handler is in the middle of the write data, we are
	 * about to set, we will put funny things on the SX bus.
	 */

#ifdef I_WANT_CLEAN
	{
	/* use pointer, helps the compiler not get to fancy */
	uint8_t *rbo_dat = &sx_raw_bits_out_data[start_byte], t_byte, *rbo_dir;
	rbo_dir = &sx_raw_bits_out_dir[start_byte];

	cli();
	/* show the interrupt that we want to write these bits */
	*rbo_dir++ |= bytemask >> (0 * UINT8_T_BIT);
	*rbo_dir++ |= bytemask >> (1 * UINT8_T_BIT);
	*rbo_dir++ |= bytemask >> (2 * UINT8_T_BIT);
	bytemask = ~bytemask;
	/* bring data in place */
	t_byte = *rbo_dat;
	*rbo_dat++ = (t_byte & ((uint8_t)(bytemask >> (0 * UINT8_T_BIT)))) | (uint8_t)(bytebuffer >> (0 * UINT8_T_BIT));
	t_byte = *rbo_dat;
	*rbo_dat++ = (t_byte & ((uint8_t)(bytemask >> (1 * UINT8_T_BIT)))) | (uint8_t)(bytebuffer >> (1 * UINT8_T_BIT));
	t_byte = *rbo_dat;
	*rbo_dat++ = (t_byte & ((uint8_t)(bytemask >> (2 * UINT8_T_BIT)))) | (uint8_t)(bytebuffer >> (2 * UINT8_T_BIT));
	sei();
	}
#else
	/* avr has no alignment, is little endian, so data comes the right way to mem, and as long
	 * as the compiler does not start to whine like a sissy about aliasing, this generates much better
	 * code, till someone subregs avr cleanly...
	 * If the compiler is game, this should be 35 clock cycles with interrupts off == 2.19µs@16MHz
	 */
	{
	sx_wild_channel *tp_dat = (sx_wild_channel *)(sx_raw_bits_out_data + start_byte), *tp_dir;
	tp_dir = (sx_wild_channel *)(sx_raw_bits_out_dir + start_byte);

	cli();
	/* show the interrupt that we want to write these bits */
	*tp_dir |= bytemask;
	/* bring data in place */
	*tp_dat = (*tp_dat & ~bytemask) | bytebuffer;
	sei();
	}
#endif
}

/*
uint8_t* GCC_ATTR_OPTIMIZE("O3") sx_get_data_pointer() {
	
	return sx_raw_bits_in;
}

uint16_t GCC_ATTR_OPTIMIZE("O3") sx_get_data_size() {
	
	return (DIV_ROUNDUP(SX_FRAME_BIT, UINT8_T_BIT) + ARR_EXTRA_SP);
}
*/

/******************** helper ****************************/

#if 0
/* did a full base frame arrived in the mean time? */
static noinline bool sx_wait_base_frame(void)
{
	static uint16_t last_bit_num;
	uint16_t local_bit_num = sx_nxt_bit_num, t;

	/* this uses a benign race, to prevent switching interrupts off */
	t = local_bit_num;
	if(last_bit_num > local_bit_num)
		local_bit_num += SX_FRAME_BIT;

	if(last_bit_num + (SX_BASE_FRAME_LEN * SX_BITCOUNT) < local_bit_num) {
		last_bit_num = t;
		return true;
	}
	return false;
}
#endif

void wait_for_full_transmission_init(void) {
	cli();
	transmission_wait.bit = sx_nxt_bit_num;
	transmission_wait.gen = sx_frame_gen_cnt;
	sei();
}

bool wait_for_full_transmission(void) {
	uint8_t diff;
	uint16_t bdiff;
	
	/* this uses a benign race, to prevent switching interrupts off */
	diff = sx_frame_gen_cnt - transmission_wait.gen;
	if(0 == diff)
		return false;
	if(diff > 2) /* TODO: buggy on generation roll over 0xff->0 */
		return true;

	bdiff = (sx_nxt_bit_num + SX_FRAME_BIT) - transmission_wait.bit;
	if(bdiff > SX_FRAME_BIT)
		return true;
	return false;
}

/* init wait for full frame */
static noinline void sx_wait_frame_init(void)
{
	cli();
	sx_frame_wait.bit = sx_nxt_bit_num;
	sx_frame_wait.gen = sx_frame_gen_cnt;
	sei();
}

/* did we had a full frame */
static noinline bool sx_wait_frame(void)
{
	uint8_t diff;
	uint16_t bdiff;

	/* this uses a benign race, to prevent switching interrupts off */
	diff = sx_frame_gen_cnt - sx_frame_wait.gen;
	if(0 == diff)
		return false;
	if(diff > 2) /* TODO: buggy on generation roll over 0xff->0 */
		return true;

	bdiff = (sx_nxt_bit_num + SX_FRAME_BIT) - sx_frame_wait.bit;
	if(bdiff > SX_FRAME_BIT)
		return true;
	return false;
}

/************************ sx state functions ****************************/
/*
 * For for a sync channel at the start of the buffer, move offset to
 * align sync frame in buffer
 *
 * after this function has done it's thing, better wait for a full frame
 */
static noinline bool sx_search_sync(void)
{
	unsigned i;
	uint16_t local_bit_num = sx_nxt_bit_num;
	uint16_t fb;
	bool ret_val = false;

	if(local_bit_num < (SX_BASE_FRAME_LEN * 2 * SX_BITCOUNT))
		return ret_val;

	fb  = sx_raw_bits_in[0];
	fb |= ((uint16_t)sx_raw_bits_in[1]) << 8;
	for(i = 0; i < (((SX_BASE_FRAME_LEN * 2 * SX_BITCOUNT))-3);)
	{
		if(0 == (fb & 0x7))
		{
			/* found sync */
			cli();
			local_bit_num = sx_nxt_bit_num;
			if(local_bit_num > i) {
				sx_nxt_bit_num = local_bit_num - i;
				ret_val = true;
			} else {
				/* simply retry when bit number has moved on? */
			}
			sei();
			break;
		}
		fb >>= 1;
		i++;
		if(0 == (i % 8)) {
			fb |= ((uint16_t)sx_raw_bits_in[(i/8)+2]) << 8;
		}
	}
	return ret_val;
}

/*
 * Search for the base frame 0 in buffer, update bit offset
 *
 * need to wait for a full frame after it has done it's thing
 */
static noinline bool sx_search_base_frame0(void)
{
	unsigned off;

	/* iterate over all sync channel */
	for(off = 0; off < SX_FRAME_BYTES; off += SX_BASE_FRAME_BYTES)
	{
		uint16_t in;
		in  = sx_raw_bits_in[off];
		in |= ((uint16_t)sx_raw_bits_in[off+1]) << 8;

		in &= 0xFEF;
		if(0x928 == in)
		{
			/* frame 0 found */
			uint16_t local_bit;
			off *= UINT8_T_BIT;
			cli();
			local_bit = sx_nxt_bit_num;
			if(local_bit > off)
				sx_nxt_bit_num = local_bit - off;
			else
				sx_nxt_bit_num = local_bit + SX_FRAME_BIT - off;
			sei();
			return true;
		}
	}
	if(off >= SX_FRAME_BYTES) {
		/* huh? something went wrong */
		internal_state = SX_SEARCH_SYNC;
		/* TODO: manipulating state from here is dirty */
	}
	return false;
}

/*
 * Check if buffer is still in sync
 * It only checks the first 12 bits, no guarantee, that all data is in sync!
 */
static noinline bool sx_control_sync(void)
{
	uint16_t in;

	/* is start of buffer still sync channel 0 ? */
	in  = sx_raw_bits_in[0];
	in |= ((uint16_t)sx_raw_bits_in[1]) << 8;
	in &= 0xFEF;
	if(0x928 == in)
		return true;
	return false;
}


/*
 * Tick function
 * call every time through main to update sx state
 */
enum sx_internal_state sx_tick(void)
{
	switch(internal_state)
	{
	case SX_SEARCH_SYNC:
		PORTC |= _BV(PC6);
		if(sx_search_sync())
			internal_state = SX_WAIT_FOR_FRAME_AFTER_SYNC_INIT;
		else
			break;
	case SX_WAIT_FOR_FRAME_AFTER_SYNC_INIT:
		sx_wait_frame_init();
		internal_state = SX_WAIT_FOR_FRAME_AFTER_SYNC;
	case SX_WAIT_FOR_FRAME_AFTER_SYNC:
		if(sx_wait_frame())
			internal_state = SX_SEARCH_BASE_FRAME0;
		break;
	case SX_SEARCH_BASE_FRAME0:
		if(sx_search_base_frame0())
			internal_state = SX_WAIT_FOR_FRAME_AFTER_BASE_FRAME0_INIT;
		else
			break;
	case SX_WAIT_FOR_FRAME_AFTER_BASE_FRAME0_INIT:
		sx_wait_frame_init();
		internal_state = SX_WAIT_FOR_FRAME_AFTER_BASE_FRAME0;
	case SX_WAIT_FOR_FRAME_AFTER_BASE_FRAME0:
		if(sx_wait_frame())
			internal_state = SX_CONTROL_SYNC;
		break;
	case SX_CONTROL_SYNC:
		PORTC &= ~_BV(PC6);
		if(!sx_control_sync()) {
			internal_state = SX_SEARCH_SYNC;
			//PORTC |= _BV(PC6);			
			/* TODO: reset write state on reset */
		}
		break;
	}
	return internal_state;
}

/* draufer */
void sx_enable_interrupts (void) {
		EIMSK |= _BV(INT0);
}
void sx_disable_interrupts (void) {
		EIMSK &= ~_BV(INT0);
}

/*********************** interrupts **********************/

/*
 * interrupt service routine (AVR INT0)
 * driven by FALLING clock signal T0 (SX pin 1)
 *
 * yes, the compiler whines that this is not a know signal,
 * that is OK, all he should do is fill in the special interrupt
 * prolouge and epilouge, since we branch here without.
 *
 */
static __attribute__((__signal__, __used__)) GCC_ATTR_OPTIMIZE("O3") void edge_falling(void)
{
	static const uint8_t bit_masks[8] PROGMEM = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
	uint16_t local_bit_num = sx_nxt_bit_num;
	uint16_t byte_num;
	uint8_t bit_mask, l_dir;

	/* toggle led for debug */
	//PORTC ^= (1<<PC7);

	/* precalc offsets */
	byte_num = local_bit_num / UINT8_T_BIT;
	bit_mask = pgm_read_byte(bit_masks + (local_bit_num % UINT8_T_BIT));
	
	/* get direction */
	l_dir = sx_raw_bits_out_dir[byte_num];

	/* act on direction */
	if(unlikely(!!(l_dir & bit_mask)))
	{
		/* Set the dir bit to 0 since we will write the bit */
		sx_raw_bits_out_dir[byte_num] = l_dir & ~bit_mask;
		/* get write data */
		l_dir = sx_raw_bits_out_data[byte_num];
		/* set direction */
		DDRD |= _BV(SX_WRITE);
		/* put write data on pin */
		if(!!(l_dir & bit_mask))
			PORTD |= _BV(SX_WRITE);
		else
			PORTD &= ~_BV(SX_WRITE);
        
        /* Enable the LED for triggering */
        PORTC |= (1<<PC7);
	}
	else
	{
		DDRD &= ~_BV(SX_WRITE);
		PORTD &= ~_BV(SX_WRITE);

        /* Disable LED */
        PORTC &= ~(1<<PC7);
		
		/* debug led off */
		PORTD &= ~_BV(PD7);
		
	}
}

/*
 * interrupt service routine (AVR INT0)
 * driven by RISING clock signal T0 (SX pin 1)
 *
 * same as above
 */
static __attribute__((__signal__, __used__)) GCC_ATTR_OPTIMIZE("O3") void edge_rising(void)
{
	static uint8_t c_byte;
	uint16_t local_bit_num = sx_nxt_bit_num;
	uint8_t lc_byte;

	/* debug led on */
	PORTD |= _BV(PD7);

	/* compiletime_assert(0 == SX_FRAME_BIT % UINT8_T_BIT) */

	/* toggle led for debug */
	//PORTC ^= _BV(PC7);

	/* shift current byte down */
	lc_byte = c_byte >> 1;

	/* copy data bit state */
	if (!!(PIND & _BV(SX_DATA)))
		lc_byte |= 0x80;

	/* put new byte in place or write back byte under construction */
	local_bit_num++;
	if (unlikely(0 == (local_bit_num % UINT8_T_BIT)))
		sx_raw_bits_in[(local_bit_num / UINT8_T_BIT)-1] = lc_byte;
	else
		c_byte = lc_byte;

	/* check if we received one complete Frame */
	if(unlikely(local_bit_num >= SX_FRAME_BIT)) {
		local_bit_num = 0; /* overflow */
		sx_frame_gen_cnt++; /* create new generation */
	}
	/* write bit number back */
	sx_nxt_bit_num = local_bit_num;
	
	
}


/*
 * Interrupt routine for INT0 -> SX_CLK
 */
ISR(INT0_vect, __attribute__((__naked__)))
{
	/*
	 * the work on falling edge (write setup time)
	 * and rising edge (data read time) is a little
	 * different.
	 * Since we only have 10us for write setup but
	 * 40us for read, hard code a min latency (no prolouge)
	 * jump to the right function to handle it.
	 * Combining it all in one func lets the compiler explode
	 * the prologue
	 */
#if 0
	if(PIND & (1<<SX_CLK)) { /* clock hi */
		edge_rising();
	} else { /* clock lo */
		edge_falling();
	}
#endif

	asm (
		"sbis	%0, %1\n\t" /* when clock high, skip...  */
		"rjmp	edge_falling\n\t" /* ...jump to edge_falling */
		"rjmp	edge_rising" /* else jump to edge_rising */
		:
		: /* %0 */ "I" (_SFR_IO_ADDR(PIND)),
		  /* %1 */ "I" (SX_CLK)
	);
}


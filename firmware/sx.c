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
    Obere Reihe von links nach rechts: Px1, Px2, Sx-T1(Daten von der Zentrale)
    Mittlere Reihe von links nach rechts: Px-EN, GND, Sx-T0(Takt)
    Untere Reihe von links nach rechts: Sx-D(Zur Zentrale), +B
*/

#include <limits.h>
#include <string.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "sx.h"

#define anum(x) (sizeof(x)/sizeof(*(x)))
#define noinline __attribute__((__noinline__))
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

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


/****************** vars ******************************/

/* raw bit arrays */
static uint8_t sx_raw_bits_in[((SX_FRAME_LEN * SX_BITCOUNT) + (UINT8_T_BIT-1))/UINT8_T_BIT];
static uint8_t sx_raw_bits_out_dir[((SX_FRAME_LEN * SX_BITCOUNT) + (UINT8_T_BIT-1))/UINT8_T_BIT];
static uint8_t sx_raw_bits_out_data[((SX_FRAME_LEN * SX_BITCOUNT) + (UINT8_T_BIT-1))/UINT8_T_BIT];
/* interrupt position */
volatile uint16_t sx_nxt_bit_num;
volatile uint8_t sx_frame_gen_cnt;
/* frame wait */
static struct {
	uint16_t bit;
	uint8_t gen;
} sx_frame_wait;
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

	//PORTC |= _BV(PC7);

	/* Set interrupt control register */
/* TODO: timing is still unclear */
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
	 * But this is all a guess....
	 */
/*************/
    /* rising edge creates Interrupt */
    EICRA  = (EICRA & ~(_BV(ISC00)|_BV(ISC01))) | _BV(ISC00);
    /* activate external interupt */
	EIMSK |= _BV(INT0);

}

/*************************** functions ********************/

enum sx_internal_state sx_get_state(void)
{
	return internal_state;
}

uint16_t sx_get_startbit(uint8_t channel)
{
    /* naming:
       Number of base frames: SX_BASE_FRAME_CNT (16)
       Number of info channels per base frame: SX_BASE_FRAME_INFO_CNT (7)
    
	   formula in arduino code: _channel = (15-_baseAdr) + ((6-_dataFrameCount)<<4);
       example bitpos lok01: 1428

       computation:
       
       channel  = 1 (lok01)                   1
	   tFrame   = channel / 16;               0
	   tBase    = channel % 16;               1
	   Frame    = 7 - tFrame;                 7
	   Base     = 15 - tBase;                 14
	   bit_addr = Base * 8 * 12 + frame * 12; 1428
    */

    return (((SX_BASE_FRAME_CNT - 1) - (channel % SX_BASE_FRAME_CNT))) * 8 * 12
        + (SX_BASE_FRAME_INFO_CNT - (channel / SX_BASE_FRAME_CNT)) * 12;
    
}

uint8_t sx_get_channel(uint8_t channel)
{
    /*
        TODO: The variables need to be cleaned
    */
        
    uint8_t channelbyte = 0;            // The channel information result (8 bit payload)
    uint16_t bitpos = sx_get_startbit(channel); // the postion of the first bit in the buffer
    uint8_t bit_offset = bitpos % 8;    // the offset from the beginning of the first byte it is stored in
    uint8_t start_byte = bitpos / 8;    // the first byte that contains the payload
    uint32_t bytebuffer = 0;
    uint8_t i;
    
    for (i = 0; i < 3; i++)
    {
        bytebuffer |= (((uint32_t)sx_raw_bits_in[(start_byte + i)
            % (SX_FRAME_BYTES*SX_BASE_FRAME_CNT)])<<(i*8));
    }
    
    uint16_t bytebuff_s = (((uint32_t)bytebuffer) >> bit_offset);
    channelbyte |= ((bytebuff_s & (0x3 << 0)) >> 0);
    channelbyte |= ((bytebuff_s & (0x3 << 3)) >> 1);
    channelbyte |= ((bytebuff_s & (0x3 << 6)) >> 2);
    channelbyte |= ((bytebuff_s & (0x3 << 9)) >> 3);
    return channelbyte;

}

void sx_set_channel(uint8_t channel, uint8_t data)
{
    uint16_t bitpos = sx_get_startbit(channel); // the postion of the first bit in the buffer
    uint8_t bit_offset = bitpos % 8;    // the offset from the beginning of the first byte it is stored in
    uint8_t start_byte = bitpos / 8;    // the first byte that contains the payload
    uint32_t bytebuffer = 0xffffffff;
    uint32_t bytemask = 0;
    uint8_t i;
    
    // The bit positions we want to write are turned to 1
    bytemask = (((uint32_t)0x3ff)<<bit_offset);
    
    uint16_t databuffer = (uint16_t)data;
    
    // bytebuffer_tmp is initialized with ones
    uint16_t bytebuffer_tmp = 0xffff;
    
    // the bits that are zeros in data/databuffer are set zero in bytebuffer_tmp, too.
    bytebuffer_tmp &= ~((~databuffer & (0x3 << 0)) << 0);
    bytebuffer_tmp &= ~((~databuffer & (0x3 << 2)) << 1);
    bytebuffer_tmp &= ~((~databuffer & (0x3 << 4)) << 2);
    bytebuffer_tmp &= ~((~databuffer & (0x3 << 6)) << 3);
    
    // bytebuffer_tmp is copied to bytebuffer (including the offset).
    bytebuffer &= (((uint32_t)bytebuffer_tmp)<<bit_offset);
    
    // the offset bits are set to ones.
    bytebuffer |= ((uint32_t)0xff>>(8 - bit_offset));

    // the bits are transferred in the data array
    for (i = 0; i < 3; i++)
    {
        // All bits we need to write are initialized with ones.
        sx_raw_bits_out_data[(start_byte + i) % (SX_FRAME_BYTES*SX_BASE_FRAME_CNT)]
            |= ((uint8_t)(bytemask>>(i * 8)));
        
        // We set the bits that need to be zero to zero (before all relevant bits were turned to one).
        sx_raw_bits_out_data[(start_byte + i) % (SX_FRAME_BYTES*SX_BASE_FRAME_CNT)]
            &= ((uint8_t)(bytebuffer>>(i * 8)));
        
        // Show the interrupt, that we want to write
        sx_raw_bits_out_dir[(start_byte + i) % (SX_FRAME_BYTES*SX_BASE_FRAME_CNT)]
            |= ((uint8_t)(bytemask>>(i * 8)));
    } 
}

/******************** helper ****************************/

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
	if(diff > 2) /* TODO: buggy on generation roll over */
		return true;

	bdiff = (sx_nxt_bit_num + SX_FRAME_BIT) - sx_frame_wait.bit;
	if(bdiff > SX_FRAME_BIT)
		return true;
	return false;
}

/************************ sx state functions ****************************/
/*
 * For for a sync channel at the start of the buffer, move offset to
 * align snc frame in buffer
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
				/* simply retry wenn bit number has moved on? */
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

	/* itterate over all sync channel */
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
 * Check if buffer is stil synced
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
void sx_tick(void)
{
	switch(internal_state)
	{
	case SX_SEARCH_SYNC:
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
		if(!sx_control_sync()) {
			internal_state = SX_SEARCH_SYNC;
			/* TODO: reset write state on reset */
		}
		break;
	}
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
static __attribute__((__signal__, __used__, __optimize__("O3"))) void edge_falling(void)
{
    //static const uint8_t bit_masks[8] PROGMEM = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
    static const uint8_t bit_masks[8] PROGMEM = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
    uint16_t local_bit_num = sx_nxt_bit_num;
    uint16_t byte_num;
    uint8_t bit_mask;

    /* toggle led for debug */
    PORTC ^= (1<<PC7);

    /* precalc offsets */
    byte_num = local_bit_num / UINT8_T_BIT;
    bit_mask = pgm_read_byte(bit_masks + (local_bit_num % UINT8_T_BIT));

    /* get direction */
    if(unlikely(!!(sx_raw_bits_out_dir[byte_num] & bit_mask)))
    {
        DDRD |= _BV(SX_WRITE);
        /* get write data */
        if(!!(sx_raw_bits_out_data[byte_num] & bit_mask))
            PORTD |= _BV(SX_WRITE);
        else
            PORTD &= ~_BV(SX_WRITE);
        
        /* Set the dir byte to 0 after the bit was written */
        sx_raw_bits_out_dir[byte_num] &= ~bit_mask;
    }
    else
    {
        DDRD &= ~_BV(SX_WRITE);
        PORTD &= ~_BV(SX_WRITE);
    }
}

/*
 * interrupt service routine (AVR INT0)
 * driven by RISING clock signal T0 (SX pin 1)
 *
 * same as above
 */
static __attribute__((__signal__, __used__, __optimize__("O3"))) void edge_rising(void)
{
	static uint8_t c_byte;
	uint16_t local_bit_num = sx_nxt_bit_num;
	uint8_t lc_byte;

	/* compiletime_assert(0 == SX_FRAME_BIT % UINT8_T_BIT) */

	/* toggle led for debug */
	PORTC ^= _BV(PC7);

	/* shift current byte up */
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
	if(local_bit_num >= SX_FRAME_BIT) {
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


/*
 * sx.h
 *
 *  Created on: 02.01.2012
 *  Changed on: 13.01.2012
 *  Version:    1.1
 *  Copyright:  Michael Blank

*  interface hardware needed ! see www.oscale.net/SX

 Read SX Signal - SX Clock must be connected to Pin2=INT0 and
 SX Data must be connected to Pin4. For triggering a scope, a signal 
 can be generated at Pin3 at a defined base adress.
 
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


 */

#ifndef SX_H_
#define SX_H_

#include <avr/io.h>
#include <inttypes.h>

// define arduino pins

// PD0,PORTD
#define SX_CLK   PD0     // must be INT0 !!
#define SX_DATA  PD4
//#define SCOPE    3
//#define SX_WRITE PD1


// defines for state machine
#define SYNC  0
#define ADDR  1
#define DATA  2

#define MAX_CHANNELCOUNT    16
#define MAX_DATACOUNT       7   // 7 dataframes in 1 SYNC Channel
#define MAX_DATABITCOUNT    12  // 12 bits in 1 frame
#define MAX_CHANNEL_NUMBER  112 // SX channels

#define SX_SEGMENT_BYTES    (MAX_DATACOUNT + 1) * MAX_CHANNELCOUNT * MAX_DATABITCOUNT / 8

void sx_init(void);
//void sx_init(uint8_t); 
uint8_t sx_get(uint8_t);
void sx_isr(void);

//private:
void sx_switch_addr(void);
void sx_switch_data(void);
    
    /* SX Timing
     1   Bit             50 µs
     1   Kanal          600 µs (= 12 Bit)
     1   Grundrahmen    ca. 4,8 ms
     1   Gesamtrahmen   ca.  80 ms (= 16 Grundrahmen)
     */


#endif /* SX_H_ */

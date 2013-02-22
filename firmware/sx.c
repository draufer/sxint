/*
 * sx.c
 *
 *  Created on: 02.01.2012
 *  Changed on: 13.01.2012
 *  Version:    1.1
 *  Copyright:  Michael Blank
 *
 *  interface hardware needed ! see www.oscale.net/SX

 Read SX Signal - SX Clock must be connected to Pin2=INT0 and
 SX Data must be connected to Pin4. For triggering a scope, a signal 
 can be generated at Pin3 at a defined base adress.

 (Uses digitalRead() function for maximum portability - is not optimized 
 for speed of ISR routine!)
 
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

/*
     Obere Reihe von links nach rechts: Px1, Px2, Sx-T1(Daten von der Zentrale)
     Mittlere Reihe von links nach rechts: Px-EN, GND, Sx-T0(Takt)
     Untere Reihe von links nach rechts: Sx-D(Zur Zentrale), +B
*/

#include <avr/interrupt.h>
#include "sx.h"

static uint8_t _toggle;
static uint8_t _zeroCount;
static uint8_t _adrCount;
static uint8_t _state;
static uint8_t _dataBitCount;               // bit counting
static uint8_t _dataFrameCount;             // frame counting

static uint8_t _data;                       // 1 data uint8_t
static uint8_t _baseAdr;                    // base address
//static uint8_t _triggerBaseAdr;
//static uint8_t _scopeFlag;                // generate scope trigger signal if != 0

static uint8_t _bit;
static uint8_t _sx[MAX_CHANNEL_NUMBER];     // to store the SX data

static uint8_t _sx_buffer[SX_SEGMENT_BYTES];

static uint8_t _channel;                    // channel from 0 to 15, B3..B0 in sync data


void sx_init() {
    // initialize function
    // initialize pins and variables
    // and start looking for a SYNC signal

    // Set interrupt control register

    DDRD &= ~(1<<SX_CLK);
    DDRD &= ~(1<<SX_DATA);

    EICRA |= (1<<ISC00) | (1<<ISC01);
    EIMSK |= (1<<INT0);

    int i = 0;
    for (i=0; i<SX_SEGMENT_BYTES;i++) {
          // reset sx variable to zero
        _sx_buffer[i]=0xff;
    }
     _toggle=0;
    _adrCount =0;

    // start always with search for SYNC
    _state = SYNC;
    _zeroCount = 0;

}

void sx_switch_addr() {
     // a SYNC signal was received, now look for a valid
     // base address
    switch(_adrCount) {
    case 0:   // this is the GLEISSPANNUNG bit
    case 1:
    case 4:
        break; // ignore
    case 2:  // B3
        _baseAdr |= (_bit<<3);
        break;
    case 3:  // B2
        _baseAdr |= (_bit<<2);
        break;
    case 5:  // B1
        _baseAdr |= (_bit<<1);
        break;
    case 6:  // B0
        _baseAdr |= (_bit<<0);
        break;
    case 7: // last "1"
        // _baseAdr is complete !
 
        // advance to next state - next we are looking
        // for the 7 data bytes (i.e. 7 SX Channels)
        _state = DATA;  
        _dataFrameCount = 0;
        _dataBitCount = 0;
        _data=0;
        break;
    }
}

void sx_switch_data() {
    // continue reading _data
    // a total of 7 DATA blocks will be received
    // for a certain base-address

    switch(_dataBitCount) {
    case 2:  // "Trenn_bits"
    case 5:
    case 8:
        _dataBitCount++;
        break; // ignore
    case 0:  // D0
        _data=0;
        _data |= (_bit<<0);
        _dataBitCount++;
        break;
    case 1:  // D1
        _data |= (_bit<<1);
        _dataBitCount++;
        break;
    case 3:  // D2
        _data |= (_bit<<2);
        _dataBitCount++;
        break;
    case 4:  // D3
        _data |= (_bit<<3);
        _dataBitCount++;
        break;
    case 6:  // D4
        _data |= (_bit<<4);
        _dataBitCount++;
        break;
    case 7:  // D5
        _data |= (_bit<<5);
        _dataBitCount++;
        break;
    case 9:  // D6
        _data |= (_bit<<6);
        _dataBitCount++;
        break;
    case 10: // D7
        _data |= (_bit<<7);
        _dataBitCount++;
        break;
    case 11:  // == MAX_DATABITCOUNT
        // _bit value should always equal HIGH, not tested here.

        // calc sx channel from baseAdr and dataFrameCount
        _channel = (15-_baseAdr) + ((6-_dataFrameCount)<<4);
        
        // copy _data byte to SX _channel
        _sx[_channel] = _data;

        // increment dataFrameCount to move on the next DATA byte
        // check, if we already reached the last DATA block - in this
        // case move on to the next SX-Datenpaket, i.e. look for SYNC
        _dataFrameCount ++;
        if (_dataFrameCount == MAX_DATACOUNT) {
            // and move on to SYNC _state
            _dataFrameCount=0;
            _state =SYNC;
            _zeroCount = 0;
            _dataBitCount=0;
        } else {
            _dataBitCount = 0;  // reset _bit counter
            _data = 0;
            // continue with reading next _data uint8_t
        }
    }  //end switch/case _dataBitCount
}

uint8_t sx_get(uint8_t channel) {
    // returns the value of a SX channel
    if (channel < MAX_CHANNEL_NUMBER)
       return _sx[channel];
    else
       return 0;
}

ISR(INT0_vect) {

    // interrupt service routine (AVR INT0)
    // driven by RISING clock signal T0 (SX pin 1)

    // 3 different states are distinguished
    //     1. SYNC = looking for a SYNC signal
    //     2. ADDR = (after SYNC received) look for base address (0..15)
    //     3. DATA = (after ADDR decoded) decode the 7 data-bytes

    //PORTC ^= (1<<PC7);

    _bit = (PIND & (1<<SX_DATA)) ? 1 : 0;
    switch(_state) {
    case SYNC:

        if (_bit == 0)  {
            _zeroCount++;
        } else {
            if (_zeroCount == 3)  {     // sync bits 0 0 0 1 found
                _state = ADDR;          // advance to next state
                _baseAdr = 0;           //init
                _adrCount = 0;          //init
            } else {                    // no valid sync, try again ...
                _zeroCount = 0;         // reset _zeroCounter
            }                           // endif _zeroCount
        }                               // endif _bit==LOW
        break;
    case ADDR:
        sx_switch_addr();
        _adrCount++;
        break;
    case DATA:
        sx_switch_data();
    }
}


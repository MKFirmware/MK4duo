/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Copyright (c) 2013 Arduino LLC. All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * Defines for 16 bit timers used with Servo library
 *
 * If _useTimerX is defined then TimerX is a 16 bit timer on the current board
 * timer16_Sequence_t enumerates the sequence that the timers should be allocated
 * _Nbr_16timers indicates how many 16 bit timers are available.
 */

/**
 * AVR Only definitions
 * --------------------
 */

#define TRIM_DURATION 2   // compensation ticks to trim adjust for digitalWrite delays
#define PRESCALER     8   // timer prescaler

// Say which 16 bit timers can be used and in what order
#if ENABLED(__AVR_ATmega1280__) || ENABLED(__AVR_ATmega2560__)
  #define _useTimer3
  #define _useTimer4
  #define _useTimer5
#elif ENABLED(__AVR_ATmega32U4__)
  #define _useTimer3
#elif ENABLED(__AVR_AT90USB646__) || ENABLED(__AVR_AT90USB1286__)
  #define _useTimer3
#elif ENABLED(__AVR_ATmega128__) || ENABLED(__AVR_ATmega1281__) || ENABLED(__AVR_ATmega1284P__) || ENABLED(__AVR_ATmega2561__)
  #define _useTimer3
#else
  // everything else
#endif

typedef enum {
  #if ENABLED(_useTimer1)
    _timer1,
  #endif
  #if ENABLED(_useTimer3)
    _timer3,
  #endif
  #if ENABLED(_useTimer4)
    _timer4,
  #endif
  #if ENABLED(_useTimer5)
    _timer5,
  #endif
  _Nbr_16timers
} timer16_Sequence_t;

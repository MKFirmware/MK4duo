/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
#pragma once

FORCE_INLINE static void _NOP1 (void) { __asm__ volatile (A("nop")); }
FORCE_INLINE static void _NOP2 (void) { __asm__ volatile (A("rjmp 1f") L("1")); }
FORCE_INLINE static void _NOP3 (void) { __asm__ volatile (A("lpm")); }
FORCE_INLINE static void _NOP4 (void) { _NOP3(); _NOP1(); }
FORCE_INLINE static void _NOP5 (void) { _NOP3(); _NOP2(); }
FORCE_INLINE static void _NOP6 (void) { _NOP3(); _NOP3(); }
FORCE_INLINE static void _NOP7 (void) { _NOP3(); _NOP3(); _NOP1(); }
FORCE_INLINE static void _NOP8 (void) { _NOP3(); _NOP3(); _NOP2(); }
FORCE_INLINE static void _NOP9 (void) { _NOP3(); _NOP3(); _NOP3(); }
FORCE_INLINE static void _NOP10(void) { _NOP3(); _NOP3(); _NOP3(); _NOP1(); }
FORCE_INLINE static void _NOP11(void) { _NOP3(); _NOP3(); _NOP3(); _NOP2(); }
FORCE_INLINE static void _NOP12(void) { _NOP3(); _NOP3(); _NOP3(); _NOP3(); }

FORCE_INLINE static void HAL_delay_loop_1_x(uint8_t tick) {
  __asm__ volatile (
    L("1")
    A("dec  %0")
    A("breq 2f")
    L("2")
    A("brne 1b")
    : "=r" (tick)
    : "0" (tick)
  );
}

FORCE_INLINE static void HAL_delay_loop_2_x(uint16_t tick) {
  __asm__ volatile (
    L("1")
    A("sbiw %0,1")
    A("brne 1b")
    A("nop")
    : "=w" (tick)
    : "0" (tick)
  );
}

FORCE_INLINE static void HAL_delay_loop_3_x(uint32_t tick) {
  __asm__ volatile (
    L("1")
    A("sbiw %A0,1")
    A("sbc  %C0,__zero_reg__")
    A("sbc  %D0,__zero_reg__")
    A("nop")
    A("breq 2f")
    L("2")
    A("brne 1b")
    : "=w" (tick)
    : "0" (tick)
  );
}

FORCE_INLINE static void HAL_delay_cycles(uint32_t cycles) {

  uint32_t padding;
  uint32_t loops;

  if (cycles <= 12)
    padding = cycles;
  else if (cycles <= 0x400) {
    cycles -= 1;
    loops = cycles / 4;
    padding = cycles % 4;
    if (loops) HAL_delay_loop_1_x((uint8_t)loops);
  }
  else if (cycles <= 0x40001) {
    cycles -= 2;
    loops = cycles / 4;
    padding = cycles % 4;
    if (loops) HAL_delay_loop_2_x((uint16_t)loops);
  }
  else {
    cycles -= 4;
    loops = cycles / 8;
    padding = cycles % 8;
    if (loops) HAL_delay_loop_3_x(loops);
  }

  if (padding ==  1)  _NOP1();
  if (padding ==  2)  _NOP2();
  if (padding ==  3)  _NOP3();
  if (padding ==  4)  _NOP4();
  if (padding ==  5)  _NOP5();
  if (padding ==  6)  _NOP6();
  if (padding ==  7)  _NOP7();
  if (padding ==  8)  _NOP8();
  if (padding ==  9)  _NOP9();
  if (padding == 10) _NOP10();
  if (padding == 11) _NOP11();
  if (padding == 12) _NOP12();

}

/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Based on u8g_com_st7920_hw_spi.c
 *
 * Universal 8bit Graphics Library
 *
 * Copyright (c) 2011, olikraus@gmail.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list
 *    of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "../../../MK4duo.h"

#if ENABLED(ARDUINO_ARCH_SAMD) && HAS_GRAPHICAL_LCD

#include <U8glib.h>
#include <Arduino.h>

void u8g_SetPIOutput_SAMD_hw_spi(u8g_t *u8g, uint8_t pin_index) {
  SET_OUTPUT(u8g->pin_list[pin_index]);
}

void u8g_SetPILevel_SAMD_hw_spi(u8g_t *u8g, uint8_t pin_index, uint8_t level) {
  if (U8G_PIN_NONE != u8g->pin_list[pin_index])
    WRITE(u8g->pin_list[pin_index], level);
}

static void writebyte(uint8_t rs, uint8_t val) {
  uint8_t i;

  if (rs == 0)
    HAL::spiSend(0x0f8);  // command
  else if ( rs == 1 )
    HAL::spiSend(0x0fa);  // data

  HAL::spiSend(val & 0x0f0);
  HAL::spiSend(val << 4);
  HAL::delayMicroseconds(50);
}

uint8_t u8g_com_HAL_SAMD_shared_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
  if (!HAL::SPIReady) return 1; // U8G problem, called before setup on SAMD will crash

  switch (msg) {
    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_INIT:
      u8g_SetPIOutput_SAMD_hw_spi(u8g, U8G_PI_CS);
      u8g_SetPILevel_SAMD_hw_spi(u8g, U8G_PI_CS, 1);

      HAL::spiBegin();

      #ifndef SPI_SPEED
        #define SPI_SPEED SPI_FULL_SPEED  // use same SPI speed as SD card
      #endif

      u8g->pin_list[U8G_PI_A0_STATE] = 0;
      break;

    case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g->pin_list[U8G_PI_A0_STATE] = arg_val;
      break;

    case U8G_COM_MSG_CHIP_SELECT:
      if (arg_val == 0) {
        HAL::delayMicroseconds(5);
        SPI.endTransaction();
        u8g_SetPILevel_SAMD_hw_spi(u8g, U8G_PI_CS, 0);
      }
      else {
         HAL::spiInit(0);
         u8g_SetPILevel_SAMD_hw_spi(u8g, U8G_PI_CS, 1);
         HAL::delayMicroseconds(5);
      }
      break;

    case U8G_COM_MSG_RESET:
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      writebyte(u8g->pin_list[U8G_PI_A0_STATE], arg_val);
      break;

    case U8G_COM_MSG_WRITE_SEQ: {
        uint8_t *ptr = (uint8_t*) arg_ptr;
        while (arg_val > 0) {
          writebyte(u8g->pin_list[U8G_PI_A0_STATE], *ptr++);
          arg_val--;
        }
      }
      break;

    case U8G_COM_MSG_WRITE_SEQ_P: {
        uint8_t *ptr = (uint8_t*) arg_ptr;
        while (arg_val > 0) {
          writebyte(u8g->pin_list[U8G_PI_A0_STATE], *ptr++);
          arg_val--;
        }
      }
      break;
  }

  return 1;

}

#endif  // ENABLED(ARDUINO_ARCH_SAMD) && HAS_GRAPHICAL_LCD

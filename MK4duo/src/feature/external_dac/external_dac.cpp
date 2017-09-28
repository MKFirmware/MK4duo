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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/***************************************************************
 *
 * External DAC for Alligator Board
 *
 ****************************************************************/

#include "../../../base.h"

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)

  ExternalDac::ExternalDac() { return; }

  void ExternalDac::begin() {
    uint8_t externalDac_buf[2] = {0x20, 0x00};  // all off

    // All SPI chip-select HIGH
    pinMode (DAC0_SYNC, OUTPUT);
    digitalWrite(DAC0_SYNC, HIGH);
    #if DRIVER_EXTRUDERS > 1
      pinMode (DAC1_SYNC, OUTPUT);
      digitalWrite(DAC1_SYNC, HIGH);
    #endif
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    digitalWrite(SPI_EEPROM2_CS, HIGH);
    digitalWrite(SPI_FLASH_CS, HIGH);
    digitalWrite(SS_PIN, HIGH);
    HAL::spiBegin();

    //init onboard DAC
    HAL::delayMicroseconds(2U);
    digitalWrite(DAC0_SYNC, LOW);
    HAL::delayMicroseconds(2U);
    digitalWrite(DAC0_SYNC, HIGH);
    HAL::delayMicroseconds(2U);
    digitalWrite(DAC0_SYNC, LOW);

    HAL::spiSend(SPI_CHAN_DAC, externalDac_buf , 2);
    digitalWrite(DAC0_SYNC, HIGH);

    #if DRIVER_EXTRUDERS > 1
      //init Piggy DAC
      HAL::delayMicroseconds(2U);
      digitalWrite(DAC1_SYNC, LOW);
      HAL::delayMicroseconds(2U);
      digitalWrite(DAC1_SYNC, HIGH);
      HAL::delayMicroseconds(2U);
      digitalWrite(DAC1_SYNC, LOW);

      HAL::spiSend(SPI_CHAN_DAC, externalDac_buf, 2);
      digitalWrite(DAC1_SYNC, HIGH);
    #endif

    return;
  }

  void ExternalDac::setValue(uint8_t channel, uint8_t value) {
    if(channel >= 7) // max channel (X,Y,Z,E0,E1,E2,E3)
      return;

    uint8_t externalDac_buf[2] = {0x10, 0x00};

    if (channel > 3)
      externalDac_buf[0] |= ((7 - channel) << 6);
    else
      externalDac_buf[0] |= ((3 - channel) << 6);

    externalDac_buf[0] |= (value >> 4);
    externalDac_buf[1] |= (value << 4);

    // All SPI chip-select HIGH
    digitalWrite(DAC0_SYNC, HIGH);
    #if DRIVER_EXTRUDERS > 1
      digitalWrite(DAC1_SYNC, HIGH);
    #endif
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    digitalWrite(SPI_EEPROM2_CS, HIGH);
    digitalWrite(SPI_FLASH_CS, HIGH);
    digitalWrite(SS_PIN, HIGH);

    if (channel > 3) { // DAC Piggy E1,E2,E3
      digitalWrite(DAC1_SYNC, LOW);
      HAL::delayMicroseconds(2U);
      digitalWrite(DAC1_SYNC, HIGH);
      HAL::delayMicroseconds(2U);
      digitalWrite(DAC1_SYNC, LOW);
    }

    else { // DAC onboard X,Y,Z,E0
      digitalWrite(DAC0_SYNC , LOW);
      HAL::delayMicroseconds(2U);
      digitalWrite(DAC0_SYNC , HIGH);
      HAL::delayMicroseconds(2U);
      digitalWrite(DAC0_SYNC , LOW);
    }

    HAL::delayMicroseconds(2U);
    HAL::spiSend(SPI_CHAN_DAC, externalDac_buf, 2);

    return;
  }

#endif

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
 * Software SPI functions originally from Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 *
 * Completely rewritten and tuned by Eduardo José Tagle in 2017/2018
 * in ARM thumb2 inline assembler and tuned for maximum speed and performance
 * allowing SPI clocks of up to 12 Mhz to increase SD card read/write performance
 */

/**
 * Description: HAL for Arduino Due and compatible (SAM3X8E)
 *
 * For ARDUINO_ARCH_SAM
 */

#ifdef ARDUINO_ARCH_SAM

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../../MK4duo.h"

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

#if ENABLED(SOFTWARE_SPI)

  // --------------------------------------------------------------------------
  // software SPI
  // --------------------------------------------------------------------------

  // bitbanging transfer
  // run at ~100KHz (necessary for init)
  uint8_t HAL::spiTransfer(uint8_t b) { // using Mode 0
    for (int bits = 0; bits < 8; bits++) {
      if (b & 0x80) {
        WRITE(MOSI_PIN, HIGH);
      }
      else {
        WRITE(MOSI_PIN, LOW);
      }
      b <<= 1;

      WRITE(SCK_PIN, HIGH);
      delayMicroseconds(5U);

      if (READ(MISO_PIN)) {
        b |= 1;
      }
      WRITE(SCK_PIN, LOW);
      delayMicroseconds(5U);
    }
    return b;
  }

  void HAL::spiBegin(void) {
    SET_OUTPUT(SS_PIN);
    WRITE(SS_PIN, HIGH);
    SET_OUTPUT(SCK_PIN);
    SET_INPUT(MISO_PIN);
    SET_OUTPUT(MOSI_PIN);
  }

  void HAL::spiInit(uint8_t spiRate) {
    UNUSED(spiRate);
    WRITE(SS_PIN, HIGH);
    WRITE(MOSI_PIN, HIGH);
    WRITE(SCK_PIN, LOW);
  }

  uint8_t HAL::spiReceive(void) {
    WRITE(SS_PIN, LOW);
    uint8_t b = spiTransfer(0xFF);
    WRITE(SS_PIN, HIGH);
    return b;
  }

  void HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
    if (nbyte == 0) return;
    WRITE(SS_PIN, LOW);
    for (int i = 0; i < nbyte; i++) {
      buf[i] = spiTransfer(0xFF);
    }
    WRITE(SS_PIN, HIGH);
  }

  void HAL::spiSend(uint8_t b) {
    WRITE(SS_PIN, LOW);
    uint8_t response = spiTransfer(b);
    UNUSED(response);
    WRITE(SS_PIN, HIGH);
  }

  void HAL::spiSend(const uint8_t* buf, size_t n) {
    uint8_t response;
    if (n == 0) return;
    WRITE(SS_PIN, LOW);
    for (uint16_t i = 0; i < n; i++) {
      response = spiTransfer(buf[i]);
    }
    UNUSED(response);
    WRITE(SS_PIN, HIGH);
  }

  void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
    uint8_t response;

    WRITE(SS_PIN, LOW);
    response = spiTransfer(token);

    for (uint16_t i = 0; i < 512; i++) {
      response = spiTransfer(buf[i]);
    }
    UNUSED(response);
    WRITE(SS_PIN, HIGH);
  }

#else

  // --------------------------------------------------------------------------
  // hardware SPI
  // --------------------------------------------------------------------------
  // 8.4 MHz, 4 MHz, 2 MHz, 1 MHz, 0.5 MHz, 0.329 MHz, 0.329 MHz
  int spiDueDividors[] = { 10, 21, 42, 84, 168, 255, 255 };
  static bool spiInitMaded = false;

  void HAL::spiBegin() {
    if (!spiInitMaded) {
      SPI.begin();
      spiInit(1);
      spiInitMaded = true;
    }
  }

  void HAL::spiInit(uint8_t spiRate) {
    if (spiInitMaded == false) {
      if (spiRate > 6) spiRate = 1;

      #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
        // Set SPI mode 1, clock, select not active after transfer, with delay between transfers
        SPI_ConfigureNPCS(SPI0, SPI_CHAN_DAC,
                          SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiRate]) |
                          SPI_CSR_DLYBCT(1));
        // Set SPI mode 0, clock, select not active after transfer, with delay between transfers
        SPI_ConfigureNPCS(SPI0, SPI_CHAN_EEPROM1, SPI_CSR_NCPHA |
                          SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiRate]) |
                          SPI_CSR_DLYBCT(1));
      #endif // MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)

      // Set SPI mode 0, clock, select not active after transfer, with delay between transfers
      SPI_ConfigureNPCS(SPI0, SPI_CHAN, SPI_CSR_NCPHA |
                        SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiRate]) |
                        SPI_CSR_DLYBCT(1));

      SPI_Enable(SPI0);
    }
  }

  // Write single byte to SPI
  void HAL::spiSend(byte b) {
    // write byte with address and end transmission flag
    SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(SPI_CHAN) | SPI_TDR_LASTXFER;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // clear status
    SPI0->SPI_RDR;
    //delayMicroseconds(1U);
  }

  void HAL::spiSend(const uint8_t* buf, size_t n) {
    if (n == 0) return;
    for (size_t i = 0; i < n - 1; i++) {
      // write buf with address
      SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(SPI_CHAN);
      // wait for transmit register empty
      while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      // wait for receive register
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      SPI0->SPI_RDR;
    }
    spiSend(buf[n - 1]);
  }

  void HAL::spiSend(uint32_t chan, byte b) {
    uint8_t dummy_read = 0;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    // write byte with address and end transmission flag
    SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(chan) | SPI_TDR_LASTXFER;
    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // clear status
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
      dummy_read = SPI0->SPI_RDR;
    UNUSED(dummy_read);
  }

  void HAL::spiSend(uint32_t chan, const uint8_t* buf, size_t n) {
    uint8_t dummy_read = 0;
    if (n == 0) return;
    for (int i = 0; i < (int)n - 1; i++) {
      while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(chan);
      // wait for receive register
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      // clear status
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
        dummy_read = SPI0->SPI_RDR;
      UNUSED(dummy_read);
    }
    spiSend(chan, buf[n - 1]);
  }

  // Read single byte from SPI
  uint8_t HAL::spiReceive(void) {
    // write dummy byte with address and end transmission flag
    SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN) | SPI_TDR_LASTXFER;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // get byte from receive register
    return SPI0->SPI_RDR;
  }

  uint8_t HAL::spiReceive(uint32_t chan) {
    uint8_t spirec_tmp;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
      spirec_tmp = SPI0->SPI_RDR;
    UNUSED(spirec_tmp);

    // write dummy byte with address and end transmission flag
    SPI0->SPI_TDR = 0x000000FF | SPI_PCS(chan) | SPI_TDR_LASTXFER;

    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // get byte from receive register
    return SPI0->SPI_RDR;
  }

  // Read from SPI into buffer
  void HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
    if (nbyte-- == 0) return;

    for (int i = 0; i < nbyte; i++) {
      //while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      buf[i] = SPI0->SPI_RDR;
    }
    buf[nbyte] = spiReceive();
  }

  // Write from buffer to SPI
  void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
    SPI0->SPI_TDR = (uint32_t)token | SPI_PCS(SPI_CHAN);
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    //while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    //SPI0->SPI_RDR;
    for (int i = 0; i < 511; i++) {
      SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(SPI_CHAN);
      while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      SPI0->SPI_RDR;
    }
    spiSend(buf[511]);
  }

#endif // ENABLED(SOFTWARE_SPI)

#endif // ARDUINO_ARCH_SAM

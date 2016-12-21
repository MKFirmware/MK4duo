/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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
 * This is the main Hardware Abstraction Layer (HAL).
 * To make the firmware work with different processors and toolchains,
 * all hardware related code should be packed into the hal files.
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
 *
 * Description:          *** HAL for Arduino Due ***
 *
 * Contributors:
 * Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
 *                    Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * Copyright (c) 2015 - 2016 Alberto Cotronei @MagoKimbra
 *
 * ARDUINO_ARCH_SAM
 */

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../../base.h"

#if ENABLED(ARDUINO_ARCH_SAM)

#include <Wire.h>

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------
uint8_t MCUSR;

// disable interrupts
void cli(void) {
  noInterrupts();
}

// enable interrupts
void sei(void) {
  interrupts();
}

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

#ifndef DUE_SOFTWARE_SPI
  int spiDueDividors[] = {10,21,42,84,168,255,255};
#endif

HAL::HAL() {
  // ctor
}

HAL::~HAL() {
  // dtor
}

// Print apparent cause of start/restart
void HAL::showStartReason() {
  int mcu = (RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos;
  switch (mcu) {
    case 0:
      Com::printInfoLN(Com::tPowerUp);
      break;
    case 1:
      // this is return from backup mode on SAM
      Com::printInfoLN(Com::tBrownOut);
    case 2:
      Com::printInfoLN(Com::tWatchdog);
      break;
    case 3:
      Com::printInfoLN(Com::tSoftwareReset);
      break;
    case 4:
      Com::printInfoLN(Com::tExternalReset);
  }
}

// Return available memory
int HAL::getFreeRam() {
  int free_memory;
  int heap_end = (int)_sbrk(0);

  if(heap_end == 0)
    free_memory = ((int)&free_memory) - ((int)&_ebss);
  else
    free_memory = ((int)&free_memory) - heap_end;

  return free_memory;
}

// Reset peripherals and cpu
void HAL::resetHardware() {
  RSTC->RSTC_CR = RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;
}

#ifdef DUE_SOFTWARE_SPI
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
      delayMicroseconds(5);

      if (READ(MISO_PIN)) {
        b |= 1;
      }
      WRITE(SCK_PIN, LOW);
      delayMicroseconds(5);
    }
    return b;
  }

  void HAL::spiBegin() {
    SET_OUTPUT(SS_PIN);
    WRITE(SS_PIN, HIGH);
    SET_OUTPUT(SCK_PIN);
    SET_INPUT(MISO_PIN);
    SET_OUTPUT(MOSI_PIN);
  }

  void HAL::spiInit(uint8_t spiClock) {
    WRITE(SS_PIN, HIGH);
    WRITE(MOSI_PIN, HIGH);
    WRITE(SCK_PIN, LOW);
  }

  uint8_t HAL::spiReceive() {
    WRITE(SS_PIN, LOW);
    uint8_t b = spiTransfer(0xff);
    WRITE(SS_PIN, HIGH);
    return b;
  }

  void HAL::spiReadBlock(uint8_t*buf, uint16_t nbyte) {
    if (nbyte == 0) return;
    WRITE(SS_PIN, LOW);
    for (int i = 0; i < nbyte; i++) {
      buf[i] = spiTransfer(0xff);
    }
    WRITE(SS_PIN, HIGH);
  }

  void HAL::spiSend(uint8_t b) {
    WRITE(SS_PIN, LOW);
    uint8_t response = spiTransfer(b);
    WRITE(SS_PIN, HIGH);
  }

  void HAL::spiSend(const uint8_t* buf , size_t n) {
    uint8_t response;
    if (n == 0) return;
    WRITE(SS_PIN, LOW);
    for (uint16_t i = 0; i < n; i++) {
      response = spiTransfer(buf[i]);
    }
    WRITE(SS_PIN, HIGH);
  }

  void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
    uint8_t response;

    WRITE(SS_PIN, LOW);
    response = spiTransfer(token);

    for (uint16_t i = 0; i < 512; i++) {
      response = spiTransfer(buf[i]);
    }
    WRITE(SS_PIN, HIGH);
  }

#else

  // --------------------------------------------------------------------------
  // hardware SPI
  // --------------------------------------------------------------------------
  #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
    bool spiInitMaded = false;
  #endif

  void HAL::spiBegin() {
#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
    if(spiInitMaded == false) {
#endif
      // Configre SPI pins
      PIO_Configure(
         g_APinDescription[SCK_PIN].pPort,
         g_APinDescription[SCK_PIN].ulPinType,
         g_APinDescription[SCK_PIN].ulPin,
         g_APinDescription[SCK_PIN].ulPinConfiguration);
      PIO_Configure(
         g_APinDescription[MOSI_PIN].pPort,
         g_APinDescription[MOSI_PIN].ulPinType,
         g_APinDescription[MOSI_PIN].ulPin,
         g_APinDescription[MOSI_PIN].ulPinConfiguration);
      PIO_Configure(
         g_APinDescription[MISO_PIN].pPort,
         g_APinDescription[MISO_PIN].ulPinType,
         g_APinDescription[MISO_PIN].ulPin,
         g_APinDescription[MISO_PIN].ulPinConfiguration);

      // set master mode, peripheral select, fault detection
      SPI_Configure(SPI0, ID_SPI0, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PS);
      SPI_Enable(SPI0);

      #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
        SET_OUTPUT(DAC0_SYNC);
        #if EXTRUDERS > 1
          SET_OUTPUT(DAC1_SYNC);
          WRITE(DAC1_SYNC, HIGH);
        #endif
        SET_OUTPUT(SPI_EEPROM1_CS);
        SET_OUTPUT(SPI_EEPROM2_CS);
        SET_OUTPUT(SPI_FLASH_CS);
        WRITE(DAC0_SYNC, HIGH);
        WRITE(SPI_EEPROM1_CS, HIGH );
        WRITE(SPI_EEPROM2_CS, HIGH );
        WRITE(SPI_FLASH_CS, HIGH );
        WRITE(SS_PIN, HIGH );
      #endif // MB(ALLIGATOR)
      PIO_Configure(
        g_APinDescription[SPI_PIN].pPort,
        g_APinDescription[SPI_PIN].ulPinType,
        g_APinDescription[SPI_PIN].ulPin,
        g_APinDescription[SPI_PIN].ulPinConfiguration);
      spiInit(1);
#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      spiInitMaded = true;
    }
#endif
  }

  void HAL::spiInit(uint8_t spiClock) {
#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
    if(spiInitMaded == false) {
#endif
      if(spiClock > 4) spiClock = 1;
      #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
        // Set SPI mode 1, clock, select not active after transfer, with delay between transfers  
        SPI_ConfigureNPCS(SPI0, SPI_CHAN_DAC,
                          SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiClock]) |
                          SPI_CSR_DLYBCT(1));
        // Set SPI mode 0, clock, select not active after transfer, with delay between transfers 
        SPI_ConfigureNPCS(SPI0, SPI_CHAN_EEPROM1, SPI_CSR_NCPHA |
                          SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiClock]) |
                          SPI_CSR_DLYBCT(1));
      #endif//MB(ALLIGATOR)

      // Set SPI mode 0, clock, select not active after transfer, with delay between transfers
      SPI_ConfigureNPCS(SPI0, SPI_CHAN, SPI_CSR_NCPHA |
                        SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiClock]) |
                        SPI_CSR_DLYBCT(1));
      SPI_Enable(SPI0);
#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      spiInitMaded = true;
    }
#endif
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
    //delayMicroseconds(1);
  }

  void HAL::spiSend(const uint8_t* buf, size_t n) {
    if (n == 0) return;
    for (size_t i = 0; i < n - 1; i++) {
      SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(SPI_CHAN);
      while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      SPI0->SPI_RDR;
      //        delayMicroseconds(1);
    }
    spiSend(buf[n - 1]);
  }

  void HAL::spiSend(uint32_t chan, byte b) {
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    // write byte with address and end transmission flag
    SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(chan) | SPI_TDR_LASTXFER;
    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // clear status
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
      SPI0->SPI_RDR;
  }

  void HAL::spiSend(uint32_t chan, const uint8_t* buf, size_t n) {
    if (n == 0) return;
    for (uint8_t i = 0; i < n - 1; i++) {
      while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(chan);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
        SPI0->SPI_RDR;
    }
    spiSend(chan, buf[n - 1]);
  }

  // Read single byte from SPI
  uint8_t HAL::spiReceive() {
    // write dummy byte with address and end transmission flag
    SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN) | SPI_TDR_LASTXFER;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);

    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // get byte from receive register
    //delayMicroseconds(1);
    return SPI0->SPI_RDR;
  }

  uint8_t HAL::spiReceive(uint32_t chan) {
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
      SPI0->SPI_RDR;

    // write dummy byte with address and end transmission flag
    SPI0->SPI_TDR = 0x000000FF | SPI_PCS(chan) | SPI_TDR_LASTXFER;

    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    // get byte from receive register
    return SPI0->SPI_RDR;
  }

  // Read from SPI into buffer
  void HAL::spiReadBlock(uint8_t*buf, uint16_t nbyte) {
    if (nbyte-- == 0) return;

    for (int i = 0; i < nbyte; i++) {
      //while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      buf[i] = SPI0->SPI_RDR;
      // delayMicroseconds(1);
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
      //        delayMicroseconds(1);
    }
    spiSend(buf[511]);
  }

#endif // DISABLED(SOFTWARE_SPI)

// --------------------------------------------------------------------------
// eeprom
// --------------------------------------------------------------------------

#ifdef SPI_EEPROM

  #define CMD_WREN  6 // WREN
  #define CMD_READ  3 // READ
  #define CMD_WRITE 2 // WRITE

  uint8_t eeprom_read_byte(uint8_t* pos) {
    uint8_t eeprom_temp[3] = {0};
    uint8_t v;

    // set read location
    // begin transmission from device
    eeprom_temp[0] = CMD_READ;
    eeprom_temp[1] = ((unsigned)pos >> 8) & 0xFF; // addr High
    eeprom_temp[2] = (unsigned)pos & 0xFF;        // addr Low
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    digitalWrite(SPI_EEPROM1_CS, LOW);
    HAL::spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);

    v = HAL::spiReceive(SPI_CHAN_EEPROM1);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    return v;
  }

  void eeprom_read_block(void* pos, const void* eeprom_address, size_t n) {
    uint8_t eeprom_temp[3] = {0};
    uint8_t *p_pos = (uint8_t *)pos;

    // set read location
    // begin transmission from device
    eeprom_temp[0] = CMD_READ;
    eeprom_temp[1] = ((unsigned)eeprom_address >> 8) & 0xFF; // addr High
    eeprom_temp[2] = (unsigned)eeprom_address & 0xFF;        // addr Low
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    digitalWrite(SPI_EEPROM1_CS, LOW);
    HAL::spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);

    while (n--)
      *p_pos++ = HAL::spiReceive(SPI_CHAN_EEPROM1);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
  }

  void eeprom_write_byte(uint8_t* pos, uint8_t value) {
    uint8_t eeprom_temp[3] = {0};

    // write enable
    eeprom_temp[0] = CMD_WREN;
    digitalWrite(SPI_EEPROM1_CS, LOW);
    HAL::spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 1);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    HAL::delayMilliseconds(1);

    // write addr
    eeprom_temp[0] = CMD_WRITE;
    eeprom_temp[1] = ((unsigned)pos >> 8) & 0xFF; // addr High
    eeprom_temp[2] = (unsigned)pos & 0xFF;        // addr Low
    digitalWrite(SPI_EEPROM1_CS, LOW);
    HAL::spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);

    HAL::spiSend(SPI_CHAN_EEPROM1, value);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    HAL::delayMilliseconds(7);   // wait for page write to complete
  }

  void eeprom_update_block(const void* pos, void* eeprom_address, size_t n) {
    uint8_t eeprom_temp[3] = {0};

    // write enable
    eeprom_temp[0] = CMD_WREN;
    digitalWrite(SPI_EEPROM1_CS, LOW);
    HAL::spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 1);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    HAL::delayMilliseconds(1);

    // write addr
    eeprom_temp[0] = CMD_WRITE;
    eeprom_temp[1] = ((unsigned)eeprom_address >> 8) & 0xFF; // addr High
    eeprom_temp[2] = (unsigned)eeprom_address & 0xFF;        // addr Low
    digitalWrite(SPI_EEPROM1_CS, LOW);
    HAL::spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);

    HAL::spiSend(SPI_CHAN_EEPROM1, (const uint8_t*)pos, n);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    HAL::delayMilliseconds(7); // wait for page write to complete
  }

#else // !SPI_EEPROM

  static bool eeprom_initialised = false;
  static uint8_t eeprom_device_address = 0x50;

  static void eeprom_init(void) {
    if (!eeprom_initialised) {
      Wire.begin();
      eeprom_initialised = true;
    }
  }

  unsigned char eeprom_read_byte(unsigned char *pos) {
    byte data = 0xFF;
    unsigned eeprom_address = (unsigned) pos;

    eeprom_init();

    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)(eeprom_address >> 8));   // MSB
    Wire.write((int)(eeprom_address & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(eeprom_device_address, (byte)1);
    if (Wire.available())
      data = Wire.read();
    return data;
  }

  // maybe let's not read more than 30 or 32 bytes at a time!
  void eeprom_read_block(void* pos, const void* eeprom_address, size_t n) {
    eeprom_init();

    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)((unsigned)eeprom_address >> 8));   // MSB
    Wire.write((int)((unsigned)eeprom_address & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(eeprom_device_address, (byte)n);
    for (byte c = 0; c < n; c++ )
      if (Wire.available()) *((uint8_t*)pos + c) = Wire.read();
  }

  void eeprom_write_byte(uint8_t *pos, uint8_t value) {
    unsigned eeprom_address = (unsigned) pos;

    eeprom_init();

    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)(eeprom_address >> 8));   // MSB
    Wire.write((int)(eeprom_address & 0xFF)); // LSB
    Wire.write(value);
    Wire.endTransmission();

    // wait for write cycle to complete
    // this could be done more efficiently with "acknowledge polling"
    HAL::delayMilliseconds(5);
  }

  // WARNING: address is a page address, 6-bit end will wrap around
  // also, data can be maximum of about 30 bytes, because the Wire library has a buffer of 32 bytes
  void eeprom_update_block(const void* pos, void* eeprom_address, size_t n) {
    uint8_t eeprom_temp[32] = {0};
    uint8_t flag = 0;

    eeprom_init();

    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)((unsigned)eeprom_address >> 8));   // MSB
    Wire.write((int)((unsigned)eeprom_address & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(eeprom_device_address, (byte)n);
    for (byte c = 0; c < n; c++) {
      if (Wire.available()) eeprom_temp[c] = Wire.read();
      if (flag = (eeprom_temp[c] ^ *((uint8_t*)pos + c))) break;
    }

    if (flag) {
      Wire.beginTransmission(eeprom_device_address);
      Wire.write((int)((unsigned)eeprom_address >> 8));   // MSB
      Wire.write((int)((unsigned)eeprom_address & 0xFF)); // LSB
      Wire.write((uint8_t*)(pos), n);
      Wire.endTransmission();

      // wait for write cycle to complete
      // this could be done more efficiently with "acknowledge polling"
      HAL::delayMilliseconds(5);
    }
  }

#endif // I2C_EEPROM

void HAL_timer_start(uint8_t timer_num, uint8_t priority, uint32_t frequency, uint32_t clock, uint8_t prescale) {
  // Get the ISR from table
  Tc *tc = TimerConfig[timer_num].pTimerRegs;
  uint32_t channel = TimerConfig[timer_num].channel;
  IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  NVIC_SetPriority(irq, priority);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | clock);

  TC_SetRC(tc, channel, VARIANT_MCK / prescale / frequency);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IDR = TC_IER_CPCS; // disable interrupt

  NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IER = TC_IER_CPCS; // enable interrupt on timer match with register C
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IDR = ~TC_IER_CPCS; // remove disable interrupt
}

void HAL_timer_disable_interrupt (uint8_t timer_num) {
	const tTimerConfig *pConfig = &TimerConfig [timer_num];
	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IDR = TC_IER_CPCS; //disable interrupt
}

// Due have no tone, this is from Repetier 0.92.3
static uint32_t tone_pin;
unsigned long _nt_time; // Time note should end.

void tone(uint8_t pin, int frequency, unsigned long duration) {
  // set up timer counter 1 channel 0 to generate interrupts for
  // toggling output pin.  
  Tc *tc = TimerConfig [BEEPER_TIMER].pTimerRegs;
  IRQn_Type irq = TimerConfig [BEEPER_TIMER].IRQ_Id;
	uint32_t channel = TimerConfig [BEEPER_TIMER].channel;

  if (duration > 0) _nt_time = millis() + duration; else _nt_time = 0xFFFFFFFF; // Set when the note should end, or play "forever".

  SET_OUTPUT(pin);
  tone_pin = pin;
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);

  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |
               TC_CMR_TCCLKS_TIMER_CLOCK4);  // TIMER_CLOCK4 -> 128 divisor
  uint32_t rc = VARIANT_MCK / 128 / frequency;
  TC_SetRA(tc, channel, rc/2);                     // 50% duty cycle
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ((IRQn_Type)irq);
}

void noTone(uint8_t pin) {
  Tc *tc = TimerConfig [BEEPER_TIMER].pTimerRegs;
  uint32_t channel = TimerConfig [BEEPER_TIMER].channel;

  TC_Stop(tc, channel);
  WRITE_VAR(pin, LOW);
}

// IRQ handler for tone generator
HAL_BEEPER_TIMER_ISR {
  static bool toggle;

  if (millis() >= _nt_time) noTone(tone_pin); // Check to see if it's time for the note to end.

  HAL_timer_isr_status(BEEPER_TIMER_COUNTER, BEEPER_TIMER_CHANNEL);
  WRITE_VAR(tone_pin, toggle);
  toggle = !toggle;
}

// A/D converter
uint16_t getAdcReading(adc_channel_num_t chan) {
  if ((ADC->ADC_ISR & _BV(chan)) == (uint32_t)_BV(chan)) {
    uint16_t rslt = ADC->ADC_CDR[chan];
    SBI(ADC->ADC_CHDR, chan);
    return rslt;
  }
  else {
    SERIAL_LM(ER, "error getAdcReading");
    return 0;
  }
}

void startAdcConversion(adc_channel_num_t chan) {
  SBI(ADC->ADC_CHER, chan);
}

// Convert an Arduino Due pin number to the corresponding ADC channel number
adc_channel_num_t pinToAdcChannel(int pin) {
  if (pin < A0) pin += A0;
  return (adc_channel_num_t) (int)g_APinDescription[pin].ulADCChannelNumber;
}

uint16_t getAdcFreerun(adc_channel_num_t chan, bool wait_for_conversion) {
  if (wait_for_conversion) while (!((ADC->ADC_ISR & _BV(chan)) == (uint32_t)_BV(chan)));
  if ((ADC->ADC_ISR & _BV(chan)) == (uint32_t)_BV(chan)) {
    uint16_t rslt = ADC->ADC_CDR[chan];
    return rslt;
  }
  else {
    SERIAL_LM(ER, "wait freerun");
    return 0;
  }
}

uint16_t getAdcSuperSample(adc_channel_num_t chan) {
  uint16_t rslt = 0;
  for (int i = 0; i < 8; i++) rslt += getAdcFreerun(chan, true);
  return rslt / 4;
}

void setAdcFreerun(void) {
  // ADC_MR_FREERUN_ON: Free Run Mode. It never waits for any trigger.
  ADC->ADC_MR |= ADC_MR_FREERUN_ON | ADC_MR_LOWRES_BITS_12;
}

void stopAdcFreerun(adc_channel_num_t chan) {
  SBI(ADC->ADC_CHDR, chan);
}

// LASER
#if ENABLED(LASERBEAM)
  static void TC_SetCMR_ChannelA(Tc *tc, uint32_t chan, uint32_t v) {
    tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xFFF0FFFF) | v;
  }

  static void TC_SetCMR_ChannelB(Tc *tc, uint32_t chan, uint32_t v) {
    tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xF0FFFFFF) | v;
  }

  static uint32_t chA, chNo;
  static Tc *chTC;
  static uint32_t TC_const;

  void HAL_laser_init_pwm(uint8_t ulPin, uint16_t ulFreq) {
    uint32_t attr = g_APinDescription[ulPin].ulPinAttribute;
    if ((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER) {
      // We use MCLK/2 as clock.
      const uint32_t TC = VARIANT_MCK / 2 / ulFreq;
      TC_const = TC/TC_MAX_DUTY_CYCLE;

      // Setup Timer for this pin
      ETCChannel channel = g_APinDescription[ulPin].ulTCChannel;
      static const uint32_t channelToChNo[] = { 0, 0, 1, 1, 2, 2, 0, 0, 1, 1, 2, 2, 0, 0, 1, 1, 2, 2 };
      static const uint32_t channelToAB[]   = { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 };

      static Tc *channelToTC[] = {
        TC0, TC0, TC0, TC0, TC0, TC0,
        TC1, TC1, TC1, TC1, TC1, TC1,
        TC2, TC2, TC2, TC2, TC2, TC2
      };

      static const uint32_t channelToId[] = { 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8 };
      chNo = channelToChNo[channel];
      chA  = channelToAB[channel];
      chTC = channelToTC[channel];
      uint32_t interfaceID = channelToId[channel];
      pmc_enable_periph_clk(TC_INTERFACE_ID + interfaceID);
      TC_Configure(chTC, chNo,
      TC_CMR_TCCLKS_TIMER_CLOCK1 |
      TC_CMR_WAVE |         // Waveform mode
      TC_CMR_WAVSEL_UP_RC | // Counter running up and reset when equals to RC
      TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
      TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
      TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
      TC_SetRC(chTC, chNo, TC);

      if (chA)
        TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR);
      else
        TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);

      PIO_Configure(g_APinDescription[ulPin].pPort,
        g_APinDescription[ulPin].ulPinType,
        g_APinDescription[ulPin].ulPin,
        g_APinDescription[ulPin].ulPinConfiguration);

      TC_Start(chTC, chNo);
    }
  }

  void HAL_laser_intensity(uint8_t intensity) {
    uint32_t ulValue = intensity * TC_const;

    if (ulValue == 0) {
      if (chA)
        TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR);
      else
        TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
    }
    else {
      if (chA) {
        TC_SetRA(chTC, chNo, ulValue);
        TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
      }
      else {
        TC_SetRB(chTC, chNo, ulValue);
        TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET);
      }
    }
  }
#endif

#endif // ARDUINO_ARCH_SAM

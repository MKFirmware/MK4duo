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
 * Description: HAL for Arduino Due and compatible (SAM3X8E)
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

#include <malloc.h>
#include <Wire.h>

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------
extern "C" char *sbrk(int i);
uint8_t MCUSR;

#if ANALOG_INPUTS > 0
  int32_t   HAL::AnalogInputRead[ANALOG_INPUTS],
            HAL::AnalogSamples[ANALOG_INPUTS][MEDIAN_COUNT],
            HAL::AnalogSamplesSum[ANALOG_INPUTS],
            HAL::adcSamplesMin[ANALOG_INPUTS],
            HAL::adcSamplesMax[ANALOG_INPUTS];
  int       HAL::adcCounter = 0,
            HAL::adcSamplePos = 0;
  uint32_t  HAL::adcEnable = 0;
  bool      HAL::Analog_is_ready = false;
#endif

const uint8_t HAL::AnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
volatile uint HAL::AnalogInputValues[ANALOG_INPUTS] = { 0 };

// disable interrupts
void cli(void) {
  noInterrupts();
}

// enable interrupts
void sei(void) {
  interrupts();
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

void HAL::clear_reset_source(void) { }

uint8_t HAL::get_reset_source(void) {
  switch ((RSTC->RSTC_SR >> 8) & 7) {
    case 0: return RST_POWER_ON; break;
    case 1: return RST_BACKUP; break;
    case 2: return RST_WATCHDOG; break;
    case 3: return RST_SOFTWARE; break;
    case 4: return RST_EXTERNAL; break;
    default:
      return 0;
  }
}

// Return available memory
int HAL::getFreeRam() {
  struct mallinfo memstruct = mallinfo();
  register char * stack_ptr asm ("sp");

  // avail mem in heap + (bottom of stack addr - end of heap addr)
  return (memstruct.fordblks + (int)stack_ptr -  (int)sbrk(0));
}

#if ANALOG_INPUTS > 0

  // Convert an Arduino Due pin number to the corresponding ADC channel number
  adc_channel_num_t HAL::pinToAdcChannel(int pin) {
    if (pin < A0) pin += A0;
    return (adc_channel_num_t) (int)g_APinDescription[pin].ulADCChannelNumber;
  }

  // Initialize ADC channels
  void HAL::analogStart(void) {

    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      PIO_Configure(
        g_APinDescription[58].pPort,
        g_APinDescription[58].ulPinType,
        g_APinDescription[58].ulPin,
        g_APinDescription[58].ulPinConfiguration);
      PIO_Configure(
        g_APinDescription[59].pPort,
        g_APinDescription[59].ulPinType,
        g_APinDescription[59].ulPin,
        g_APinDescription[59].ulPinConfiguration);
    #endif // MB(ALLIGATOR) || MB(ALLIGATOR_V3)

    // ensure we can write to ADC registers
    ADC->ADC_WPMR = 0x41444300u;    // ADC_WPMR_WPKEY(0);
    pmc_enable_periph_clk(ID_ADC);  // enable adc clock

    for (int i = 0; i < ANALOG_INPUTS; i++) {
      AnalogInputValues[i] = 0;
      adcSamplesMin[i] = 100000;
      adcSamplesMax[i] = 0;
      adcEnable |= (0x1u << pinToAdcChannel(AnalogInputChannels[i]));
      AnalogSamplesSum[i] = 2048 * MEDIAN_COUNT;
      for (int j = 0; j < MEDIAN_COUNT; j++)
        AnalogSamples[i][j] = 2048;
    }
    // enable channels
    ADC->ADC_CHER = adcEnable;
    ADC->ADC_CHDR = !adcEnable;

    // Initialize ADC mode register (some of the following params are not used here)
    // HW trigger disabled, use external Trigger, 12 bit resolution
    // core and ref voltage stays on, normal sleep mode, normal not free-run mode
    // startup time 16 clocks, settling time 17 clocks, no changes on channel switch
    // convert channels in numeric order
    // set prescaler rate  MCK/((PRESCALE+1) * 2)
    // set tracking time  (TRACKTIM+1) * clock periods
    // set transfer period  (TRANSFER * 2 + 3)
    ADC->ADC_MR = ADC_MR_TRGEN_DIS | ADC_MR_TRGSEL_ADC_TRIG0 | ADC_MR_LOWRES_BITS_12 |
                  ADC_MR_SLEEP_NORMAL | ADC_MR_FWUP_OFF | ADC_MR_FREERUN_OFF |
                  ADC_MR_STARTUP_SUT64 | ADC_MR_SETTLING_AST17 | ADC_MR_ANACH_NONE |
                  ADC_MR_USEQ_NUM_ORDER |
                  ADC_MR_PRESCAL(AD_PRESCALE_FACTOR) |
                  ADC_MR_TRACKTIM(AD_TRACKING_CYCLES) |
                  ADC_MR_TRANSFER(AD_TRANSFER_CYCLES);

    ADC->ADC_IER = 0;             // no ADC interrupts
    ADC->ADC_CGR = 0;             // Gain = 1
    ADC->ADC_COR = 0;             // Single-ended, no offset

    // start first conversion
    ADC->ADC_CR = ADC_CR_START;
  }

  void HAL::analogRead() {

    // conversion finished?
    if ((ADC->ADC_ISR & adcEnable) == adcEnable) {
      adcCounter++;
      for (int i = 0; i < ANALOG_INPUTS; i++) {
        int32_t cur = ADC->ADC_CDR[pinToAdcChannel(AnalogInputChannels[i])];
        cur = (cur >> (2 - ANALOG_REDUCE_BITS)); // Convert to 10 bit result
        AnalogInputRead[i] += cur;
        adcSamplesMin[i] = min(adcSamplesMin[i], cur);
        adcSamplesMax[i] = max(adcSamplesMax[i], cur);
        if (adcCounter >= NUM_ADC_SAMPLES) { // store new conversion result
          AnalogInputRead[i] = AnalogInputRead[i] + (1 << (OVERSAMPLENR - 1)) - (adcSamplesMin[i] + adcSamplesMax[i]);
          adcSamplesMin[i] = 100000;
          adcSamplesMax[i] = 0;
          AnalogSamplesSum[i] -= AnalogSamples[i][adcSamplePos];
          AnalogSamplesSum[i] += (AnalogSamples[i][adcSamplePos] = AnalogInputRead[i] >> OVERSAMPLENR);
          AnalogInputValues[i] = AnalogSamplesSum[i] / MEDIAN_COUNT;
          AnalogInputRead[i] = 0;
        } // adcCounter >= NUM_ADC_SAMPLES
      } // for i
      if (adcCounter >= NUM_ADC_SAMPLES) {
        adcCounter = 0;
        adcSamplePos++;
        if (adcSamplePos >= MEDIAN_COUNT) {
          adcSamplePos = 0;
          Analog_is_ready = true;
        }
      }
      ADC->ADC_CR = ADC_CR_START; // reread values
    }
  }

#endif

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
    SET_OUTPUT(SDSS);
    WRITE(SDSS, HIGH);
    SET_OUTPUT(SCK_PIN);
    SET_INPUT(MISO_PIN);
    SET_OUTPUT(MOSI_PIN);
  }

  void HAL::spiInit(uint8_t spiClock) {
    WRITE(SDSS, HIGH);
    WRITE(MOSI_PIN, HIGH);
    WRITE(SCK_PIN, LOW);
  }

  uint8_t HAL::spiReceive() {
    WRITE(SDSS, LOW);
    uint8_t b = spiTransfer(0xff);
    WRITE(SDSS, HIGH);
    return b;
  }

  void HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
    if (nbyte == 0) return;
    WRITE(SDSS, LOW);
    for (int i = 0; i < nbyte; i++) {
      buf[i] = spiTransfer(0xff);
    }
    WRITE(SDSS, HIGH);
  }

  void HAL::spiSend(uint8_t b) {
    WRITE(SDSS, LOW);
    uint8_t response = spiTransfer(b);
    WRITE(SDSS, HIGH);
  }

  void HAL::spiSend(const uint8_t* buf , size_t n) {
    uint8_t response;
    if (n == 0) return;
    WRITE(SDSS, LOW);
    for (uint16_t i = 0; i < n; i++) {
      response = spiTransfer(buf[i]);
    }
    WRITE(SDSS, HIGH);
  }

  void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
    uint8_t response;

    WRITE(SDSS, LOW);
    response = spiTransfer(token);

    for (uint16_t i = 0; i < 512; i++) {
      response = spiTransfer(buf[i]);
    }
    WRITE(SDSS, HIGH);
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
      if (spiInitMaded == false) {
    #endif

    // Configre SPI pins
    PIO_Configure(
      g_APinDescription[SCK_PIN].pPort,
      g_APinDescription[SCK_PIN].ulPinType,
      g_APinDescription[SCK_PIN].ulPin,
      g_APinDescription[SCK_PIN].ulPinConfiguration
    );
    PIO_Configure(
      g_APinDescription[MOSI_PIN].pPort,
      g_APinDescription[MOSI_PIN].ulPinType,
      g_APinDescription[MOSI_PIN].ulPin,
      g_APinDescription[MOSI_PIN].ulPinConfiguration
    );
    PIO_Configure(
      g_APinDescription[MISO_PIN].pPort,
      g_APinDescription[MISO_PIN].ulPinType,
      g_APinDescription[MISO_PIN].ulPin,
      g_APinDescription[MISO_PIN].ulPinConfiguration
    );

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
      WRITE(SDSS, HIGH );
    #endif // MB(ALLIGATOR) || MB(ALLIGATOR_V3)

    PIO_Configure(
      g_APinDescription[SPI_PIN].pPort,
      g_APinDescription[SPI_PIN].ulPinType,
      g_APinDescription[SPI_PIN].ulPin,
      g_APinDescription[SPI_PIN].ulPinConfiguration
    );
    spiInit(1);
    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      spiInitMaded = true;
      }
    #endif

  }

  void HAL::spiInit(uint8_t spiClock) {
    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      if (spiInitMaded == false) {
    #endif

    if (spiClock > 4) spiClock = 1;

    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      // Set SPI mode 1, clock, select not active after transfer, with delay between transfers  
      SPI_ConfigureNPCS(SPI0, SPI_CHAN_DAC,
                        SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiClock]) |
                        SPI_CSR_DLYBCT(1));

      // Set SPI mode 0, clock, select not active after transfer, with delay between transfers 
      SPI_ConfigureNPCS(SPI0, SPI_CHAN_EEPROM1, SPI_CSR_NCPHA |
                        SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiClock]) |
                        SPI_CSR_DLYBCT(1));

    #endif // MB(ALLIGATOR) || MB(ALLIGATOR_V3) || MB(ULTRATRONICS)

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

  #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)

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
    }

    void HAL::spiSend(uint32_t chan, const uint8_t* buf, size_t n) {
      uint8_t dummy_read = 0;
      if (n == 0) return;
      for (int i = 0; i < n - 1; i++) {
        while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
        SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(chan);
        while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
        while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
          dummy_read = SPI0->SPI_RDR;
      }
      spiSend(chan, buf[n - 1]);
    }

    uint8_t HAL::spiReceive(uint32_t chan) {
      uint8_t spirec_tmp;
      // wait for transmit register empty
      while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
        spirec_tmp =  SPI0->SPI_RDR;

      // write dummy byte with address and end transmission flag
      SPI0->SPI_TDR = 0x000000FF | SPI_PCS(chan) | SPI_TDR_LASTXFER;

      // wait for receive register
      while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
      // get byte from receive register
      return SPI0->SPI_RDR;
    }

  #endif // MB(ALLIGATOR) || MB(ALLIGATOR_V3)

  // Read from SPI into buffer
  void HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
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
      // delayMicroseconds(1);
    }
    spiSend(buf[511]);
  }

#endif // DISABLED(SOFTWARE_SPI)

// --------------------------------------------------------------------------
// eeprom
// --------------------------------------------------------------------------

#ifdef SPI_EEPROM

  static constexpr uint8_t CMD_WREN   = 6;  // WREN
  static constexpr uint8_t CMD_READ   = 3;  // READ
  static constexpr uint8_t CMD_WRITE  = 2;  // WRITE

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

  #include <Wire.h>

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

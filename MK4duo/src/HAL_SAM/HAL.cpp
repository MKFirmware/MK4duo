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

#include "../../base.h"

#ifdef __SAM3X8E__

#include <Wire.h>

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

  void HAL::spiReadBlock(uint8_t*buf, uint16_t nbyte) {
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
  #if MB(ALLIGATOR)
    bool spiInitMaded = false;
  #endif

  void HAL::spiBegin() {
#if MB(ALLIGATOR)
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

      #if MB(ALLIGATOR)
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
      #endif // MB(ALLIGATOR)
      PIO_Configure(
        g_APinDescription[SPI_PIN].pPort,
        g_APinDescription[SPI_PIN].ulPinType,
        g_APinDescription[SPI_PIN].ulPin,
        g_APinDescription[SPI_PIN].ulPinConfiguration);
      spiInit(1);
#if MB(ALLIGATOR)
      spiInitMaded = true;
    }
#endif
  }

  void HAL::spiInit(uint8_t spiClock) {
#if MB(ALLIGATOR)
    if(spiInitMaded == false) {
#endif
      if(spiClock > 4) spiClock = 1;
      #if MB(ALLIGATOR)
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
#if MB(ALLIGATOR)
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

static bool eeprom_initialised = false;
static uint8_t eeprom_device_address = 0x50;

static void eeprom_init(void) {
  #if MB(ALLIGATOR)
  #else
    if (!eeprom_initialised) {
      Wire.begin();
      eeprom_initialised = true;
    }
  #endif// MB(ALLIGATOR)
}

#if MB(ALLIGATOR)
  static void eprBurnValue(unsigned int pos, int size, unsigned char * newvalue) {
    uint8_t eeprom_temp[3];

    /*write enable*/
    eeprom_temp[0] = 6;//WREN
    digitalWrite( SPI_EEPROM1_CS, LOW );
    HAL::spiSend(SPI_CHAN_EEPROM1, eeprom_temp , 1);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    HAL::delayMilliseconds(1);

    /*write addr*/
    eeprom_temp[0] = 2;//WRITE
    eeprom_temp[1] = ((pos>>8) & 0xFF);//addrH
    eeprom_temp[2] = (pos& 0xFF);//addrL
    digitalWrite(SPI_EEPROM1_CS, LOW);
    HAL::spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);        

    HAL::spiSend(SPI_CHAN_EEPROM1 ,newvalue , 1);
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    HAL::delayMilliseconds(7);   // wait for page write to complete
  }

  // Read any data type from EEPROM that was previously written by eprBurnValue
  static uint8_t eprGetValue(unsigned int pos) {
    int i = 0;
    uint8_t v;
    uint8_t eeprom_temp[3];
    // set read location
    // begin transmission from device

    eeprom_temp[0] = 3;//READ
    eeprom_temp[1] = ((pos>>8) & 0xFF);//addrH
    eeprom_temp[2] = (pos& 0xFF);//addrL
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    digitalWrite(SPI_EEPROM1_CS, LOW);
    HAL::spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);

    v = HAL::spiReceive(SPI_CHAN_EEPROM1); 
    digitalWrite(SPI_EEPROM1_CS, HIGH);
    return v;
  }
#endif

void eeprom_write_byte(unsigned char *pos, unsigned char value) {
  #if MB(ALLIGATOR)
    eprBurnValue((unsigned) pos, 1, &value);
  #else

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
  #endif// MB(ALLIGATOR)
}

unsigned char eeprom_read_byte(unsigned char *pos) {
  #if MB(ALLIGATOR)
    return eprGetValue((unsigned) pos);
  #else
    byte data = 0xFF;
    unsigned eeprom_address = (unsigned) pos;

    eeprom_init ();

    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)(eeprom_address >> 8));   // MSB
    Wire.write((int)(eeprom_address & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(eeprom_device_address, (byte)1);
    if (Wire.available())
      data = Wire.read();
    return data;
  #endif// MB(ALLIGATOR)
}

// --------------------------------------------------------------------------
// Timers
// --------------------------------------------------------------------------

typedef struct {
  Tc          *pTimerRegs;
  uint16_t    channel;
  IRQn_Type   IRQ_Id;
} tTimerConfig;

#define  NUM_HARDWARE_TIMERS 9

static const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] =
{
  { TC0, 0, TC0_IRQn},
  { TC0, 1, TC1_IRQn},
  { TC0, 2, TC2_IRQn},
  { TC1, 0, TC3_IRQn},
  { TC1, 1, TC4_IRQn},
  { TC1, 2, TC5_IRQn},
  { TC2, 0, TC6_IRQn},
  { TC2, 1, TC7_IRQn},
  { TC2, 2, TC8_IRQn},
};

/*
	Timer_clock1: Prescaler 2 -> 42MHz
	Timer_clock2: Prescaler 8 -> 10.5MHz
	Timer_clock3: Prescaler 32 -> 2.625MHz
	Timer_clock4: Prescaler 128 -> 656.25kHz
*/

uint8_t bestClock(double frequency, uint32_t& retRC){
	/*
		Pick the best Clock, thanks to Ogle Basil Hall!
		Timer		Definition
		TIMER_CLOCK1	MCK /  2
		TIMER_CLOCK2	MCK /  8
		TIMER_CLOCK3	MCK / 32
		TIMER_CLOCK4	MCK /128
	*/
	const struct {
		uint8_t flag;
		uint8_t divisor;
	} clockConfig[] = {
		{ TC_CMR_TCCLKS_TIMER_CLOCK1,   2 },
		{ TC_CMR_TCCLKS_TIMER_CLOCK2,   8 },
		{ TC_CMR_TCCLKS_TIMER_CLOCK3,  32 },
		{ TC_CMR_TCCLKS_TIMER_CLOCK4, 128 }
	};
	float ticks;
	float error;
	int clkId = 3;
	int bestClock = 3;
	float bestError = 9.999e99;
	do {
		ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[clkId].divisor;
		// error = abs(ticks - round(ticks));
		error = clockConfig[clkId].divisor * abs(ticks - round(ticks));	// Error comparison needs scaling
		if (error < bestError) {
			bestClock = clkId;
			bestError = error;
		}
	} while (clkId-- > 0);
	ticks = (float) VARIANT_MCK / frequency / (float) clockConfig[bestClock].divisor;
	retRC = (uint32_t) round(ticks);
	return clockConfig[bestClock].flag;
}

// new timer by Ps991
// thanks for that work
// http://forum.arduino.cc/index.php?topic=297397.0

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  TcChannel *extruderChannel = (ADVANCE_EXTRUDER_TIMER_COUNTER->TC_CHANNEL + ADVANCE_EXTRUDER_TIMER_CHANNEL);
#endif
TcChannel* stepperChannel = (STEP_TIMER_COUNTER->TC_CHANNEL + STEP_TIMER_CHANNEL);

void HAL_step_timer_start() {
  pmc_set_writeprotect(false); // remove write protection on registers

  // Get the ISR from table
  Tc *tc = STEP_TIMER_COUNTER;
  IRQn_Type irq = STEP_TIMER_IRQN;
  uint32_t channel = STEP_TIMER_CHANNEL;

  pmc_enable_periph_clk((uint32_t)irq); // we need a clock?
  NVIC_SetPriority(irq, 1); // highest priority - no surprises here wanted

  tc->TC_CHANNEL[channel].TC_CCR = TC_CCR_CLKDIS;

  tc->TC_CHANNEL[channel].TC_SR; // clear status register
  tc->TC_CHANNEL[channel].TC_CMR =  TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK1;

  tc->TC_CHANNEL[channel].TC_IER /*|*/= TC_IER_CPCS; // enable interrupt on timer match with register C
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_RC   = (VARIANT_MCK >> 1) / STEP_FREQUENCY; // start with 1kHz as frequency; //interrupt occurs every x interations of the timer counter

  tc->TC_CHANNEL[channel].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  NVIC_EnableIRQ(irq); // enable Nested Vector Interrupt Controller
}

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  void HAL_advance_extruder_timer_start() {
    // Get the ISR from table
    Tc *tc = ADVANCE_EXTRUDER_TIMER_COUNTER;
    IRQn_Type irq = ADVANCE_EXTRUDER_TIMER_IRQN;
    uint32_t channel = ADVANCE_EXTRUDER_TIMER_CHANNEL;
    uint32_t rc = 0;
    uint8_t clock;

    // Find the best clock for the wanted frequency
    clock = bestClock(ADVANCE_EXTRUDER_FREQUENCY, rc);

    pmc_set_writeprotect(false); // remove write protection on registers
    pmc_enable_periph_clk((uint32_t)irq);
    NVIC_SetPriority(irq, 6);

    tc->TC_CHANNEL[channel].TC_CCR = TC_CCR_CLKDIS;

    tc->TC_CHANNEL[channel].TC_SR; // clear status register
    tc->TC_CHANNEL[channel].TC_CMR =  TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | clock;

    tc->TC_CHANNEL[channel].TC_IER /*|*/= TC_IER_CPCS; // enable interrupt on timer match with register C
    tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_RC  = rc;

    tc->TC_CHANNEL[channel].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

    NVIC_EnableIRQ(irq); // enable Nested Vector Interrupt Controller
  }
#endif

void HAL_temp_timer_start (uint8_t timer_num) {
	Tc *tc = TimerConfig [timer_num].pTimerRegs;
	IRQn_Type irq = TimerConfig [timer_num].IRQ_Id;
	uint32_t channel = TimerConfig [timer_num].channel;

	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t)irq);

	//NVIC_SetPriorityGrouping(4);
	//NVIC_SetPriority(irq, NVIC_EncodePriority(4, 6, 0));
  NVIC_SetPriority(irq, 15);

	TC_Configure (tc, channel, TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK4);
  tc->TC_CHANNEL[channel].TC_IER |= TC_IER_CPCS; //enable interrupt on timer match with register C

	tc->TC_CHANNEL[channel].TC_RC   = (VARIANT_MCK >> 7) / TEMP_FREQUENCY;
	TC_Start(tc, channel);

	NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt (uint8_t timer_num) {
	const tTimerConfig *pConfig = &TimerConfig [timer_num];
	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IER = TC_IER_CPCS; //enable interrupt
	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IDR = ~TC_IER_CPCS;//remove disable interrupt
}

void HAL_timer_disable_interrupt (uint8_t timer_num) {
	const tTimerConfig *pConfig = &TimerConfig [timer_num];
	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IDR = TC_IER_CPCS; //disable interrupt
}

int HAL_timer_get_count (uint8_t timer_num) {
	Tc *tc = TimerConfig [timer_num].pTimerRegs;
	uint32_t channel = TimerConfig [timer_num].channel;
	return tc->TC_CHANNEL[channel].TC_RC;
}

// Due have no tone, this is from Repetier 0.92.3
static uint32_t tone_pin;
unsigned long _nt_time; // Time note should end.

void tone(uint8_t pin, int frequency, unsigned long duration) {
  // set up timer counter 1 channel 0 to generate interrupts for
  // toggling output pin.  

  /*TC1, 1, TC4_IRQn*/
  uint8_t timer_num = BEEPER_TIMER_NUM;

  Tc *tc = TimerConfig [timer_num].pTimerRegs;
  IRQn_Type irq = TimerConfig [timer_num].IRQ_Id;
	uint32_t channel = TimerConfig [timer_num].channel;

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
  uint8_t timer_num = BEEPER_TIMER_NUM;

  Tc *tc = TimerConfig [timer_num].pTimerRegs;
  uint32_t channel = TimerConfig [timer_num].channel;

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

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

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
  if (pin < A0)
    pin += A0;
  return (adc_channel_num_t) (int) g_APinDescription[pin].ulADCChannelNumber;
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
  return rslt /4 ;
}

void stopAdcFreerun(adc_channel_num_t chan) {
  SBI(ADC->ADC_CHDR, chan);
}

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

#endif // __SAM3X8E__
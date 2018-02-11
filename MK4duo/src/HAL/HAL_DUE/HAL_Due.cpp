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

#include "../../../MK4duo.h"

#if ENABLED(ARDUINO_ARCH_SAM)

#include <malloc.h>
#include <Wire.h>

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------
#if ENABLED(MOVE_DEBUG)
  unsigned int  numInterruptsScheduled    = 0,
                numInterruptsExecuted     = 0;
  uint32_t      nextInterruptTime         = 0,
                nextInterruptScheduledAt  = 0,
                lastInterruptTime         = 0,
                acceleration_step_rate    = 0,
                deceleration_step_rate    = 0;
#endif

uint8_t MCUSR;

#if ANALOG_INPUTS > 0
  int16_t HAL::AnalogInputValues[NUM_ANALOG_INPUTS] = { 0 };
  bool    HAL::Analog_is_ready = false;
#endif

#if HEATER_COUNT > 0
  ADCAveragingFilter HAL::sensorFilters[HEATER_COUNT];
#endif

#if HAS_FILAMENT_SENSOR
  ADCAveragingFilter  HAL::filamentFilter;
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  ADCAveragingFilter  HAL::powerFilter;
#endif

#if HAS_MCU_TEMPERATURE
  ADCAveragingFilter  HAL::mcuFilter;
#endif

// disable interrupts
void cli(void) {
  noInterrupts();
}

// enable interrupts
void sei(void) {
  interrupts();
}

// Tone for due
// input parameters: Arduino pin number, frequency in Hz, duration in milliseconds
void tone(const pin_t t_pin, const uint16_t frequency, const uint16_t duration) {

  millis_t endTime = millis() + duration;
  const uint32_t halfPeriod = 1000000L / frequency / 2;

  HAL::pinMode(t_pin, OUTPUT_LOW);

  while (PENDING(millis(),  endTime)) {
    HAL::digitalWrite(t_pin, HIGH);
    HAL::delayMicroseconds(halfPeriod);
    HAL::digitalWrite(t_pin, LOW);
    HAL::delayMicroseconds(halfPeriod);
  }
  HAL::pinMode(t_pin, OUTPUT_LOW);
}

static inline void ConfigurePin(const PinDescription& pinDesc) {
  PIO_Configure(pinDesc.pPort, pinDesc.ulPinType, pinDesc.ulPin, pinDesc.ulPinConfiguration);
}

// This intercepts the 1ms system tick. It must return 'false', otherwise the Arduino core tick handler will be bypassed.
extern "C" int sysTickHook() {
  HAL::Tick();
  return 0;
}

HAL::HAL() {
  // ctor
}

HAL::~HAL() {
  // dtor
}

bool HAL::execute_100ms = false;

// do any hardware-specific initialization here
void HAL::hwSetup(void) {

  #if DISABLED(USE_WATCHDOG)
    // Disable watchdog
    WDT_Disable(WDT);
  #endif

  TimeTick_Configure(F_CPU);

  NVIC_SetPriority(SysTick_IRQn, NvicPrioritySystick);
  NVIC_SetPriority(UART_IRQn, NvicPriorityUart);

  #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)

    // All SPI chip-select HIGH
    OUT_WRITE(DAC0_SYNC, HIGH);
    #if EXTRUDERS > 1
      OUT_WRITE(DAC1_SYNC, HIGH);
    #endif
    OUT_WRITE(SPI_EEPROM1_CS, HIGH);
    OUT_WRITE(SPI_EEPROM2_CS, HIGH);
    OUT_WRITE(SPI_FLASH_CS, HIGH);
    SET_INPUT(MOTOR_FAULT_PIN);
    #if MB(ALLIGATOR_V3)
      SET_INPUT(MOTOR_FAULT_PIGGY_PIN);
      SET_INPUT(FTDI_COM_RESET_PIN);
      SET_INPUT(ESP_WIFI_MODULE_RESET_PIN);
      OUT_WRITE(EXP1_OUT_ENABLE_PIN, HIGH);
    #elif MB(ALLIGATOR)
      // Init Expansion Port Voltage logic Selector
      OUT_WRITE(EXP_VOLTAGE_LEVEL_PIN, UI_VOLTAGE_LEVEL);
    #endif

    #if HAS_BUZZER
      BUZZ(10,10);
    #endif

  #elif MB(ULTRATRONICS)

    /* avoid floating pins */
    OUT_WRITE(ORIG_FAN0_PIN, LOW);
    OUT_WRITE(ORIG_FAN1_PIN, LOW);

    OUT_WRITE(ORIG_HEATER_0_PIN, LOW);
    OUT_WRITE(ORIG_HEATER_1_PIN, LOW);
    OUT_WRITE(ORIG_HEATER_2_PIN, LOW);
    OUT_WRITE(ORIG_HEATER_3_PIN, LOW);

    OUT_WRITE(ENC424_SS_PIN, HIGH);

  #endif

  #if PIN_EXISTS(SS)
    OUT_WRITE(SS_PIN, HIGH);
  #endif

  #if HAS_MAX31855_SS0
    OUT_WRITE(MAX31855_SS0_PIN, HIGH);
  #endif
  #if HAS_MAX31855_SS1
    OUT_WRITE(MAX31855_SS1_PIN, HIGH);
  #endif
  #if HAS_MAX31855_SS2
    OUT_WRITE(MAX31855_SS2_PIN, HIGH);
  #endif
  #if HAS_MAX31855_SS3
    OUT_WRITE(MAX31855_SS3_PIN, HIGH);
  #endif

}

// Print apparent cause of start/restart
void HAL::showStartReason() {

  int mcu = (RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos;
  switch (mcu) {
    case 0:
      SERIAL_EM(MSG_POWERUP); break;
    case 1:
      // this is return from backup mode on SAM
      SERIAL_EM(MSG_BROWNOUT_RESET); break;
    case 2:
      SERIAL_EM(MSG_WATCHDOG_RESET); break;
    case 3:
      SERIAL_EM(MSG_SOFTWARE_RESET); break;
    case 4:
      SERIAL_EM(MSG_EXTERNAL_RESET); break;
  }
}

// Return available memory
int HAL::getFreeRam() {
  struct mallinfo memstruct = mallinfo();
  register char * stack_ptr asm ("sp");

  // avail mem in heap + (bottom of stack addr - end of heap addr)
  return (memstruct.fordblks + (int)stack_ptr -  (int)sbrk(0));
}

// Convert an Arduino Due analog pin number to the corresponding ADC channel number
adc_channel_num_t PinToAdcChannel(pin_t pin) {
  if (pin == ADC_TEMPERATURE_SENSOR) return (adc_channel_num_t)ADC_TEMPERATURE_SENSOR; // MCU TEMPERATURE SENSOR

  // Arduino Due uses separate analog pin numbers
  if (pin < A0) pin += A0;
  return (adc_channel_num_t)g_APinDescription[pin].ulADCChannelNumber;
}

// Start converting the enabled channels
void AnalogInStartConversion() {
  // Clear out any existing conversion complete bits in the status register
  for (uint32_t chan = 0; chan < 16; ++chan) {
    if ((adc_get_status(ADC) & (1 << chan)) != 0) {
      (void)adc_get_channel_value(ADC, static_cast<adc_channel_num_t>(chan));
    }
  }
  ADC->ADC_CR = ADC_CR_START;
}

// Enable or disable a channel.
void AnalogInEnablePin(const pin_t r_pin, const bool enable) {
  adc_channel_num_t adc_ch = PinToAdcChannel(r_pin);
  if ((unsigned int)adc_ch < NUM_ANALOG_INPUTS) {
    if (enable) {
      adc_enable_channel(ADC, adc_ch);
      if (r_pin == ADC_TEMPERATURE_SENSOR)
        ADC->ADC_ACR |= ADC_ACR_TSON;
    }
    else {
      adc_disable_channel(ADC, adc_ch);
      if (r_pin == ADC_TEMPERATURE_SENSOR)
        ADC->ADC_ACR &= ~ADC_ACR_TSON;
    }
  }
}   

// Read the most recent 12-bit result from a pin
uint16_t AnalogInReadPin(const pin_t r_pin) {

  adc_channel_num_t adc_ch = PinToAdcChannel(r_pin);
  if ((unsigned int)adc_ch < NUM_ANALOG_INPUTS)
    return adc_get_channel_value(ADC, adc_ch);
  else
    return 0;
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

  #if HEATER_COUNT > 0
    LOOP_HEATER() {
      if (WITHIN(heaters[h].sensor.pin, 0, 15)) {
        AnalogInEnablePin(heaters[h].sensor.pin, true);
        sensorFilters[h].Init(0);
      }
    }
  #endif

  #if HAS_FILAMENT_SENSOR
    AnalogInEnablePin(FILWIDTH_PIN, true);
    filamentFilter.Init(0);
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    AnalogInEnablePin(POWER_CONSUMPTION_PIN, true);
    powerFilter.Init(0);
  #endif

  #if HAS_MCU_TEMPERATURE
    AnalogInEnablePin(ADC_TEMPERATURE_SENSOR, true);
    mcuFilter.Init(0);
  #endif

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
  ADC->ADC_COR = 0;             // Single-ended, no offset

  // start first conversion
  AnalogInStartConversion();
}

void HAL::AdcChangePin(const pin_t old_pin, const pin_t new_pin) {
  AnalogInEnablePin(old_pin, false);
  AnalogInEnablePin(new_pin, true);
}

// Reset peripherals and cpu
void HAL::resetHardware() {
	// BANZAIIIIIII!!!
	RSTC->RSTC_CR =	RSTC_CR_KEY(0xA5) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
}

// --------------------------------------------------------------------------
// Analogic write to a PWM Pin
// --------------------------------------------------------------------------
static bool     PWMEnabled      = false;
static uint16_t PWMChanFreq[8]  = {0},
                PWMChanPeriod[8];

static const uint32_t PwmFastClock =  25000 * 255;        // fast PWM clock for Intel spec PWM fans that need 25kHz PWM
static const uint32_t PwmSlowClock = (25000 * 255) / 256; // slow PWM clock to allow us to get slow speeds

static inline uint32_t ConvertRange(const float f, const uint32_t top) { return LROUND(f * (float)top); }

// AnalogWritePwm to a PWM pin
// Return true if successful, false if we need to call software pwm
static void AnalogWritePwm(const PinDescription& pinDesc, const float ulValue, const uint16_t freq) {

  const uint32_t chan = pinDesc.ulPWMChannel;

  if (freq == 0) {
    PWMChanFreq[chan] = freq;
    return;
  }
  else if (PWMChanFreq[chan] != freq) {
    if (!PWMEnabled) {
      // PWM Startup code
      pmc_enable_periph_clk(PWM_INTERFACE_ID);
      PWMC_ConfigureClocks(PwmSlowClock, PwmFastClock, VARIANT_MCK);
      PWM->PWM_SCM = 0;     // ensure no sync channels
      PWMEnabled = true;
    }

    const bool useFastClock = (freq >= PwmFastClock / 65535);
    const uint32_t period = ((useFastClock) ? PwmFastClock : PwmSlowClock) / freq;
    const uint32_t duty = ConvertRange(ulValue, period);

    PWMChanFreq[chan] = freq;
    PWMChanPeriod[chan] = (uint16_t)period;

    // Set up the PWM channel
    // We need to work around a bug in the SAM PWM channels. Enabling a channel is supposed to clear the counter, but it doesn't.
    // A further complication is that on the SAM3X, the update-period register doesn't appear to work.
    // So we need to make sure the counter is less than the new period before we change the period.
    for (uint8_t j = 0; j < 5; ++j) {  // twice through should be enough, but just in case...
    
      PWMC_DisableChannel(PWM, chan);
      uint32_t oldCurrentVal = PWM->PWM_CH_NUM[chan].PWM_CCNT & 0xFFFF;
      if (oldCurrentVal < period || oldCurrentVal > 65536 - 10) // if counter is already small enough or about to wrap round, OK
        break;
      oldCurrentVal += 2;											// note: +1 doesn't work here, has to be at least +2
      PWM->PWM_CH_NUM[chan].PWM_CPRD = oldCurrentVal;				// change the period to be just greater than the counter
      PWM->PWM_CH_NUM[chan].PWM_CMR = PWM_CMR_CPRE_CLKB;			// use the fast clock to avoid waiting too long
      PWMC_EnableChannel(PWM, chan);
      for (uint16_t i = 0; i < 1000; ++i) {
        const uint32_t newCurrentVal = PWM->PWM_CH_NUM[chan].PWM_CCNT & 0xFFFF;
        if (newCurrentVal < period || newCurrentVal > oldCurrentVal)
          break;    // get out when we have wrapped round, or failed to
      }
    }

    PWMC_ConfigureChannel(PWM, chan, ((useFastClock) ? PWM_CMR_CPRE_CLKB : PWM_CMR_CPRE_CLKA), 0, 0);
    PWMC_SetPeriod(PWM, chan, period);
    PWMC_SetDutyCycle(PWM, chan, duty);
    PWMC_EnableChannel(PWM, chan);

    // Now setup the PWM output pin for PWM this channel - do this after configuring the PWM to avoid glitches
    ConfigurePin(pinDesc);
  }
  else {
    const uint32_t ul_period = (uint32_t)PWMChanPeriod[chan];
    PWMC_SetDutyCycle(PWM, chan, ConvertRange(ulValue, ul_period));
  }
  return;
}

// --------------------------------------------------------------------------
// Analogic Write to a TC Pin
// --------------------------------------------------------------------------
const unsigned int numTcChannels = 9;

// Map from timer channel to TC channel number
static const uint8_t channelToChNo[] = { 0, 1, 2, 0, 1, 2, 0, 1, 2 };

// Map from timer channel to TC number
static Tc * const channelToTC[] = { TC0, TC0, TC0,
                                    TC1, TC1, TC1,
                                    TC2, TC2, TC2 };

// Map from timer channel to TIO number
static const uint8_t channelToId[] = {  ID_TC0, ID_TC1, ID_TC2,
                                        ID_TC3, ID_TC4, ID_TC5,
                                        ID_TC6, ID_TC7, ID_TC8 };

// Current frequency of each TC channel
static uint16_t TCChanFreq[numTcChannels] = {0};

static inline void TC_SetCMR_ChannelA(Tc *tc, uint32_t chan, uint32_t v) {
  tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xFFF0FFFF) | v;
}
static inline void TC_SetCMR_ChannelB(Tc *tc, uint32_t chan, uint32_t v) {
  tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xF0FFFFFF) | v;
}

static inline void TC_WriteCCR(Tc *tc, uint32_t chan, uint32_t v) {
  tc->TC_CHANNEL[chan].TC_CCR = v;
}

static inline uint32_t TC_read_ra(Tc *tc, uint32_t chan) {
  return tc->TC_CHANNEL[chan].TC_RA;
}
static inline uint32_t TC_read_rb(Tc *tc, uint32_t chan) {
  return tc->TC_CHANNEL[chan].TC_RB;
}
static inline uint32_t TC_read_rc(Tc *tc, uint32_t chan) {
  return tc->TC_CHANNEL[chan].TC_RC;
}

// AnalogWriteTc to a TC pin
// Return true if successful, false if we need to call software pwm
static void AnalogWriteTc(const PinDescription& pinDesc, const float ulValue, const uint16_t freq) {

  const uint32_t chan = (uint32_t)pinDesc.ulTCChannel >> 1;
  if (freq == 0) {
    TCChanFreq[chan] = freq;
    return;
  }
  else {
    Tc * const chTC = channelToTC[chan];
    const uint32_t chNo = channelToChNo[chan];
    const bool doInit = (TCChanFreq[chan] != freq);

    if (doInit) {
      TCChanFreq[chan] = freq;

      // Enable the peripheral clock to this timer
      pmc_enable_periph_clk(channelToId[chan]);

      // Set up the timer mode and top count
      TC_Configure(chTC, chNo,
              TC_CMR_TCCLKS_TIMER_CLOCK2 |    // clock is MCLK/8 to save a little power and avoid overflow later on
              TC_CMR_WAVE |                   // Waveform mode
              TC_CMR_WAVSEL_UP_RC |           // Counter running up and reset when equals to RC
              TC_CMR_EEVT_XC0 |               // Set external events from XC0 (this setup TIOB as output)
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR |
              TC_CMR_ASWTRG_SET | TC_CMR_BSWTRG_SET);	// Software trigger will let us set the output high
      const uint32_t top = (VARIANT_MCK / 8) / (uint32_t)freq;  // with 120MHz clock this varies between 228 (@ 65.535kHz) and 15 million (@ 1Hz)
      // The datasheet doesn't say how the period relates to the RC value, but from measurement it seems that we do not need to subtract one from top
      TC_SetRC(chTC, chNo, top);

      // When using TC channels to do PWM control of heaters with active low outputs on the Duet WiFi, if we don't take precautions
      // then we get a glitch straight after initialising the channel, because the compare output starts in the low state.
      // To avoid that, set the output high here if a high PWM was requested.
      if (ulValue >= 0.5)
        TC_WriteCCR(chTC, chan, TC_CCR_SWTRG);
    }

    const uint32_t threshold = ConvertRange(ulValue, TC_read_rc(chTC, chNo));
    if (threshold == 0) {
      if (((uint32_t)pinDesc.ulTCChannel & 1) == 0) {
        TC_SetRA(chTC, chNo, 1);
        TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR);
      }
      else {
        TC_SetRB(chTC, chNo, 1);
        TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
      }
    }
    else {
      if (((uint32_t)pinDesc.ulTCChannel & 1) == 0) {
        TC_SetRA(chTC, chNo, threshold);
        TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
      }
      else {
        TC_SetRB(chTC, chNo, threshold);
        TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET);
      }
    }

    if (doInit) {
      ConfigurePin(pinDesc);
      TC_Start(chTC, chNo);
    }
  }
  return;
}

bool HAL::pwm_status(const pin_t pin) {
  const PinDescription& pinDesc = g_APinDescription[pin];
  const uint32_t attr = pinDesc.ulPinAttribute;
  if (attr & PIN_ATTR_PWM) return true;
  else return false;
}
  
bool HAL::tc_status(const pin_t pin) {
  const PinDescription& pinDesc = g_APinDescription[pin];
  const uint32_t attr = pinDesc.ulPinAttribute;
  if (attr & PIN_ATTR_TIMER) return true;
  else return false;
}

void HAL::analogWrite(pin_t pin, const uint8_t value, const uint16_t freq/*=1000*/) {

  if (isnan(value) || pin <= 0) return;

  const PinDescription& pinDesc = g_APinDescription[pin];
  if (pinDesc.ulPinType == PIO_NOT_A_PIN) return;

  const float ulValue = constrain((float)value / 255.0, 0.0, 1.0);
  const uint32_t attr = pinDesc.ulPinAttribute;

  if (attr & PIN_ATTR_PWM) {
    AnalogWritePwm(pinDesc, ulValue, freq);
    g_pinStatus[pin] = (g_pinStatus[pin] & 0xF0) | PIN_STATUS_PWM;
  }
  else if (attr & PIN_ATTR_TIMER) {
    AnalogWriteTc(pinDesc, ulValue, freq);
    g_pinStatus[pin] = (g_pinStatus[pin] & 0xF0) | PIN_STATUS_TIMER;
  }
  else {
    HAL::pinMode(pin, (ulValue < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
  }
}

/**
 * Tick is is called 1000 timer per second.
 * It is used to update pwm values for heater and some other frequent jobs.
 *
 *  - Manage PWM to all the heaters and fan
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Step the babysteps value for each axis towards 0
 *  - For PINS_DEBUGGING, monitor and report endstop pins
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 */
void HAL::Tick() {

  static uint8_t  cycle_100ms = 0;

  if (!printer.IsRunning()) return;

  #if HEATER_COUNT > 0
    LOOP_HEATER() heaters[h].SetHardwarePwm();
  #endif

  #if FAN_COUNT > 0
    LOOP_FAN() fans[f].SetHardwarePwm();
  #endif

  // Calculation cycle approximate a 100ms
  cycle_100ms++;
  if (cycle_100ms >= 100) {
    cycle_100ms = 0;
    execute_100ms = true;
  }

  // read analog values
  #if ANALOG_INPUTS > 0

    if (adc_get_status(ADC)) { // conversion finished?

      #if HEATER_COUNT > 0
        for (uint8_t h = 0; h < HEATER_COUNT; h++) {
          if (WITHIN(heaters[h].sensor.pin, 0, 15)) {
            ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(sensorFilters[h]);
            currentFilter.ProcessReading(AnalogInReadPin(heaters[h].sensor.pin));
            if (currentFilter.IsValid()) {
              AnalogInputValues[heaters[h].sensor.pin] = currentFilter.GetSum() / (NUM_ADC_SAMPLES >> OVERSAMPLENR);
              Analog_is_ready = true;
            }
          }
        }
      #endif

      #if HAS_FILAMENT_SENSOR
        const_cast<ADCAveragingFilter&>(filamentFilter).ProcessReading(AnalogInReadPin(FILWIDTH_PIN));
        if (filamentFilter.IsValid())
          AnalogInputValues[FILWIDTH_PIN] = filamentFilter.GetSum() / (NUM_ADC_SAMPLES >> OVERSAMPLENR);
      #endif

      #if HAS_POWER_CONSUMPTION_SENSOR
        const_cast<ADCAveragingFilter&>(powerFilter).ProcessReading(AnalogInReadPin(POWER_CONSUMPTION_PIN));
        if (powerFilter.IsValid())
          AnalogInputValues[POWER_CONSUMPTION_PIN] = powerFilter.GetSum() / (NUM_ADC_SAMPLES >> OVERSAMPLENR);
      #endif

      #if HAS_MCU_TEMPERATURE
        const_cast<ADCAveragingFilter&>(mcuFilter).ProcessReading(AnalogInReadPin(ADC_TEMPERATURE_SENSOR));
        if (mcuFilter.IsValid())
          thermalManager.mcu_current_temperature_raw = mcuFilter.GetSum() / (NUM_ADC_SAMPLES >> OVERSAMPLENR);
      #endif

    }

    AnalogInStartConversion();

    // Update the raw values if they've been read. Else we could be updating them during reading.
    if (HAL::Analog_is_ready) thermalManager.set_current_temp_raw();

  #endif

  #if ENABLED(BABYSTEPPING)
    LOOP_XYZ(axis) {
      int curTodo = mechanics.babystepsTodo[axis]; //get rid of volatile for performance

      if (curTodo) {
        stepper.babystep((AxisEnum)axis, curTodo > 0);
        if (curTodo > 0) mechanics.babystepsTodo[axis]--;
                    else mechanics.babystepsTodo[axis]++;
      }
    }
  #endif //BABYSTEPPING

  #if ENABLED(PINS_DEBUGGING)
    extern bool endstop_monitor_flag;
    // run the endstop monitor at 15Hz
    static uint8_t endstop_monitor_count = 16;  // offset this check from the others
    if (endstop_monitor_flag) {
      endstop_monitor_count += _BV(1);  //  15 Hz
      endstop_monitor_count &= 0x7F;
      if (!endstop_monitor_count) endstops.endstop_monitor();  // report changes in endstop status
    }
  #endif

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    if (endstops.e_hit && ENDSTOPS_ENABLED) {
      endstops.update();  // call endstop update routine
      endstops.e_hit--;
    }
  #endif

}

#endif // ARDUINO_ARCH_SAM

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
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 *
 * ARDUINO_ARCH_SAM
 */

#ifdef ARDUINO_ARCH_SAM

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "../../../MK4duo.h"
#include <malloc.h>
#include <Wire.h>

/** Public Parameters */
uint8_t MCUSR;

/** Private Parameters */
#if HAS_HOTENDS
  ADCAveragingFilter HAL::HOTENDsensorFilters[MAX_HOTEND];
#endif
#if HAS_BEDS
  ADCAveragingFilter HAL::BEDsensorFilters[MAX_BED];
#endif
#if HAS_CHAMBERS
  ADCAveragingFilter HAL::CHAMBERsensorFilters[MAX_CHAMBER];
#endif
#if HAS_COOLERS
  ADCAveragingFilter HAL::COOLERsensorFilters[MAX_COOLER];
#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  ADCAveragingFilter  HAL::filamentFilter;
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  ADCAveragingFilter  HAL::powerFilter;
#endif

#if HAS_MCU_TEMPERATURE
  ADCAveragingFilter  HAL::mcuFilter;
#endif

__attribute__ ((aligned(256)))
static DeviceVectors ram_tab = { NULL };

static pfnISR_Handler* get_relocated_table_addr() {
  // Get the address of the interrupt/exception table
  uint32_t isrtab = SCB->VTOR;

  // If already relocated, we are done!
  if (isrtab >= IRAM0_ADDR)
    return (pfnISR_Handler*)isrtab;

  // Get the address of the table stored in FLASH
  const pfnISR_Handler* romtab = (const pfnISR_Handler*)isrtab;

  // Copy it to SRAM
  memcpy(&ram_tab, romtab, sizeof(ram_tab));

  CRITICAL_SECTION_START();
    // Set the vector table base address to the SRAM copy
    SCB->VTOR = (uint32_t)(&ram_tab);
  CRITICAL_SECTION_END();

  return (pfnISR_Handler*)(&ram_tab);
}

pfnISR_Handler install_isr(IRQn_Type irq, pfnISR_Handler newHandler) {
  // Get the address of the relocated table
  pfnISR_Handler *isrtab = get_relocated_table_addr();

  CRITICAL_SECTION_START();

    // Get the original handler
    pfnISR_Handler oldHandler = isrtab[irq + 16];

    // Install the new one
    isrtab[irq + 16] = newHandler;

  CRITICAL_SECTION_END();

  return oldHandler;
}

// disable interrupts
void cli() {
  noInterrupts();
}

// enable interrupts
void sei() {
  interrupts();
}

// Return available memory
extern "C" {
  extern unsigned int _ebss; // end of bss section
  int freeMemory() {
    int free_memory, heap_end = (int)_sbrk(0);
    return (int)&free_memory - (heap_end ? heap_end : (int)&_ebss);
  }
}

// Tone for due
static pin_t tone_pin;
volatile static int32_t toggles;

void tone(const pin_t _pin, const uint16_t frequency, const uint16_t duration) {
  tone_pin = _pin;
  toggles = 2 * frequency * duration / 1000;
  HAL_timer_start(TONE_TIMER_NUM, 2 * frequency);
}

void noTone(const pin_t _pin) {
  HAL_timer_disable_interrupt(TONE_TIMER_NUM);
  HAL::digitalWrite(_pin, LOW);
}

HAL::HAL() {
  // ctor
}

HAL::~HAL() {
  // dtor
}

// do any hardware-specific initialization here
void HAL::hwSetup() {
  TimeTick_Configure(F_CPU);
  NVIC_SetPriority(SysTick_IRQn, NvicPrioritySystick);
  NVIC_SetPriority(UART_IRQn, NvicPriorityUart);
}

// Print apparent cause of start/restart
void HAL::showStartReason() {
  switch ((RSTC->RSTC_SR >> 8) & 0x07) {
    case 0: SERIAL_EM(STR_POWERUP); break;
    case 1: SERIAL_EM(STR_BROWNOUT_RESET); break;
    case 2: SERIAL_EM(STR_WATCHDOG_RESET); break;
    case 3: SERIAL_EM(STR_SOFTWARE_RESET); break;
    case 4: SERIAL_EM(STR_EXTERNAL_RESET); break;
    default: break;
  }
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
void HAL::analogStart() {

  #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
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
  #endif // MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)

  // ensure we can write to ADC registers
  ADC->ADC_WPMR = 0x41444300u;    // ADC_WPMR_WPKEY(0);
  pmc_enable_periph_clk(ID_ADC);  // enable adc clock

  #if HAS_HOTENDS
    LOOP_HOTEND() {
      if (WITHIN(hotends[h]->data.sensor.pin, 0, 15)) {
        AnalogInEnablePin(hotends[h]->data.sensor.pin, true);
      }
    }
  #endif
  #if HAS_BEDS
    LOOP_BED() {
      if (WITHIN(beds[h]->data.sensor.pin, 0, 15)) {
        AnalogInEnablePin(beds[h]->data.sensor.pin, true);
      }
    }
  #endif
  #if HAS_CHAMBERS
    LOOP_CHAMBER() {
      if (WITHIN(chambers[h]->data.sensor.pin, 0, 15)) {
        AnalogInEnablePin(chambers[h]->data.sensor.pin, true);
      }
    }
  #endif
  #if HAS_COOLERS
    LOOP_COOLER() {
      if (WITHIN(coolers[h]->data.sensor.pin, 0, 15)) {
        AnalogInEnablePin(coolers[h]->data.sensor.pin, true);
      }
    }
  #endif

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    if (WITHIN(FILWIDTH_PIN, 0, 15) {
      AnalogInEnablePin(FILWIDTH_PIN, true);
    }
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    if (WITHIN(POWER_CONSUMPTION_PIN, 0, 15) {
      AnalogInEnablePin(POWER_CONSUMPTION_PIN, true);
    }
  #endif

  #if HAS_MCU_TEMPERATURE
    AnalogInEnablePin(ADC_TEMPERATURE_SENSOR, true);
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
  RSTC->RSTC_CR = RSTC_CR_KEY(0xA5) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
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

/**
 * PWM output only work on the pins with hardware support.
 *  For the rest of the pins, we default to digital output
 */

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
  if (from == to)
    return value;
  if (from > to)
    return value >> (from - to);
  else
    return value << (to - from);
}

static void TC_SetCMR_ChannelA(Tc *tc, uint32_t chan, uint32_t v) {
  tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xFFF0FFFF) | v;
}
static void TC_SetCMR_ChannelB(Tc *tc, uint32_t chan, uint32_t v) {
  tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xF0FFFFFF) | v;
}

void HAL::analogWrite(const pin_t pin, uint32_t ulValue, const uint16_t freq/*=1000U*/) {

  static bool PWMEnabled = false;
  static uint8_t TCChanEnabled[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  const int writeResolution = 8;

  if (isnan(ulValue) || pin <= 0) return;

  const PinDescription& pinDesc = g_APinDescription[pin];
  if (pinDesc.ulPinType == PIO_NOT_A_PIN) return;

  const uint32_t attr = pinDesc.ulPinAttribute;

  if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG) {
    EAnalogChannel channel = pinDesc.ulADCChannelNumber;
    if (channel == DA0 || channel == DA1) {
      uint32_t chDACC = ((channel == DA0) ? 0 : 1);
      if (dacc_get_channel_status(DACC_INTERFACE) == 0) {

        /* Enable clock for DACC_INTERFACE */
        pmc_enable_periph_clk(DACC_INTERFACE_ID);

        /* Reset DACC registers */
        dacc_reset(DACC_INTERFACE);

        /* Half word transfer mode */
        dacc_set_transfer_mode(DACC_INTERFACE, 0);

        /* Power save:
         * sleep mode  - 0 (disabled)
         * fast wakeup - 0 (disabled)
         */
        dacc_set_power_save(DACC_INTERFACE, 0, 0);

        /* Timing:
         * refresh        - 0x08 (1024*8 dacc clocks)
         * max speed mode -    0 (disabled)
         * startup time   - 0x10 (1024 dacc clocks)
         */
        dacc_set_timing(DACC_INTERFACE, 0x08, 0, 0x10);

        /* Set up analog current */
        dacc_set_analog_control(DACC_INTERFACE, DACC_ACR_IBCTLCH0(0x02) |
                                DACC_ACR_IBCTLCH1(0x02) |
                                DACC_ACR_IBCTLDACCORE(0x01)
        );
      }

      /* Disable TAG and select output channel chDACC */
      dacc_set_channel_selection(DACC_INTERFACE, chDACC);

      if ((dacc_get_channel_status(DACC_INTERFACE) & (1 << chDACC)) == 0)
        dacc_enable_channel(DACC_INTERFACE, chDACC);

      // Write user value
      ulValue = mapResolution(ulValue, writeResolution, DACC_RESOLUTION);
      dacc_write_conversion_data(DACC_INTERFACE, ulValue);
      while ((dacc_get_interrupt_status(DACC_INTERFACE) & DACC_ISR_EOC) == 0);
      return;
    }
  }

  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM) {

    ulValue = mapResolution(ulValue, writeResolution, PWM_RESOLUTION);

    if (!PWMEnabled) {
      // PWM Startup code
      pmc_enable_periph_clk(PWM_INTERFACE_ID);
      PWMC_ConfigureClocks(freq * PWM_MAX_DUTY_CYCLE, 0, VARIANT_MCK);
      PWM_INTERFACE->PWM_SCM = 0; // ensure no sync channels
      PWMEnabled = true;
    }

    uint32_t chan = pinDesc.ulPWMChannel;
    if ((g_pinStatus[pin] & 0xF) != PIN_STATUS_PWM) {
      // Setup PWM for this pin
      PIO_Configure(pinDesc.pPort,
          pinDesc.ulPinType,
          pinDesc.ulPin,
          pinDesc.ulPinConfiguration);
      PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
      PWMC_SetPeriod(PWM_INTERFACE, chan, PWM_MAX_DUTY_CYCLE);
      PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue);
      PWMC_EnableChannel(PWM_INTERFACE, chan);
      g_pinStatus[pin] = (g_pinStatus[pin] & 0xF0) | PIN_STATUS_PWM;
    }

    PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue);
    return;
  }

  if ((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER) {

    static const uint32_t channelToChNo[] = { 0, 0, 1, 1, 2, 2, 0, 0, 1, 1, 2, 2, 0, 0, 1, 1, 2, 2 };
    static const uint32_t channelToAB[]   = { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 };
    static const uint32_t channelToId[] = { 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8 };

    static Tc *channelToTC[] = {
      TC0, TC0, TC0, TC0, TC0, TC0,
      TC1, TC1, TC1, TC1, TC1, TC1,
      TC2, TC2, TC2, TC2, TC2, TC2
    };

    // We use MCLK/2 as clock.
    const uint32_t TC = VARIANT_MCK / 2 / freq;

    // Map value to Timer ranges 0..255 => 0..TC
    ulValue = mapResolution(ulValue, writeResolution, TC_RESOLUTION);
    ulValue *= TC / TC_MAX_DUTY_CYCLE;

    // Setup Timer for this pin
    ETCChannel channel = pinDesc.ulTCChannel;
    uint32_t chNo = channelToChNo[channel];
    uint32_t chA  = channelToAB[channel];
    Tc *chTC = channelToTC[channel];
    uint32_t interfaceID = channelToId[channel];

    if (!TCChanEnabled[interfaceID]) {
      pmc_enable_periph_clk(TC_INTERFACE_ID + interfaceID);
      TC_Configure(chTC, chNo,
        TC_CMR_TCCLKS_TIMER_CLOCK1 |
        TC_CMR_WAVE |         // Waveform mode
        TC_CMR_WAVSEL_UP_RC | // Counter running up and reset when equals to RC
        TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
        TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
        TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
      chTC->TC_CHANNEL[chNo].TC_RC = TC;
    }

    if (ulValue == 0) {
      if (chA)
        TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR);
      else
        TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
    }
    else {
      if (chA) {
        chTC->TC_CHANNEL[chNo].TC_RA = ulValue;
        TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
      }
      else {
        chTC->TC_CHANNEL[chNo].TC_RB = ulValue;
        TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET);
      }
    }

    if ((g_pinStatus[pin] & 0xF) != PIN_STATUS_PWM) {
      PIO_Configure(pinDesc.pPort,
          pinDesc.ulPinType,
          pinDesc.ulPin,
          pinDesc.ulPinConfiguration);
      g_pinStatus[pin] = (g_pinStatus[pin] & 0xF0) | PIN_STATUS_PWM;
    }

    if (!TCChanEnabled[interfaceID]) {
      TC_Start(chTC, chNo);
      TCChanEnabled[interfaceID] = 1;
    }

    return;
  }

}

/**
 * Task Tick is is called 1000 timer per second.
 * It is used to update pwm values for heater and some other frequent jobs.
 *
 *  - Manage PWM to all the heaters and fan
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Step the babysteps value for each axis towards 0
 *  - For PINS_DEBUGGING, monitor and report endstop pins
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 */
void HAL::Tick() {

  static short_timer_t  cycle_1s_timer(millis()),
                        cycle_100_timer(millis());

  if (printer.isStopped()) return;

  // Heaters set output PWM
  tempManager.set_output_pwm();

  // Fans set output PWM
  fanManager.set_output_pwm();

  // Event 100 ms
  if (cycle_100_timer.expired(100)) tempManager.spin();

  // Event 1.0 Second
  if (cycle_1s_timer.expired(1000)) printer.check_periodical_actions();

  // Read analog or SPI values
  if (adc_get_status(ADC)) { // conversion finished?

    #if HAS_HOTENDS
      LOOP_HOTEND() {
        if (WITHIN(hotends[h]->data.sensor.pin, 0, 15)) {
          ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(HOTENDsensorFilters[h]);
          currentFilter.process_reading(AnalogInReadPin(hotends[h]->data.sensor.pin));
          if (currentFilter.IsValid())
            hotends[h]->data.sensor.adc_raw = currentFilter.GetSum();
        }
      }
    #endif
    #if HAS_BEDS
      LOOP_BED() {
        if (WITHIN(beds[h]->data.sensor.pin, 0, 15)) {
          ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(BEDsensorFilters[h]);
          currentFilter.process_reading(AnalogInReadPin(beds[h]->data.sensor.pin));
          if (currentFilter.IsValid())
            beds[h]->data.sensor.adc_raw = currentFilter.GetSum();
        }
      }
    #endif
    #if HAS_CHAMBERS
      LOOP_CHAMBER() {
        if (WITHIN(chambers[h]->data.sensor.pin, 0, 15)) {
          ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(CHAMBERsensorFilters[h]);
          currentFilter.process_reading(AnalogInReadPin(chambers[h]->data.sensor.pin));
          if (currentFilter.IsValid())
            chambers[h]->data.sensor.adc_raw = currentFilter.GetSum();
        }
      }
    #endif
    #if HAS_COOLERS
      LOOP_COOLER() {
        if (WITHIN(coolers[h]->data.sensor.pin, 0, 15)) {
          ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(COOLERsensorFilters[h]);
          currentFilter.process_reading(AnalogInReadPin(coolers[h]->data.sensor.pin));
          if (currentFilter.IsValid())
            coolers[h]->data.sensor.adc_raw = currentFilter.GetSum();
        }
      }
    #endif

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      const_cast<ADCAveragingFilter&>(filamentFilter).process_reading(AnalogInReadPin(FILWIDTH_PIN));
      if (filamentFilter.IsValid())
        tempManager.current_raw_filwidth = filamentFilter.GetSum();
    #endif

    #if HAS_POWER_CONSUMPTION_SENSOR
      const_cast<ADCAveragingFilter&>(powerFilter).process_reading(AnalogInReadPin(POWER_CONSUMPTION_PIN));
      if (powerFilter.IsValid())
        powerManager.current_raw_powconsumption = powerFilter.GetSum();
    #endif

    #if HAS_MCU_TEMPERATURE
      const_cast<ADCAveragingFilter&>(mcuFilter).process_reading(AnalogInReadPin(ADC_TEMPERATURE_SENSOR));
      if (mcuFilter.IsValid())
        tempManager.mcu_current_temperature_raw = mcuFilter.GetSum();
    #endif

  }

  AnalogInStartConversion();

  // Tick endstops state, if required
  endstops.Tick();

}

int32_t HAL::analog2tempMCU(const int16_t adc_raw) {
  const float voltage = (float)adc_raw * ((HAL_VOLTAGE_PIN) / (float)AD_RANGE);
  return (voltage - 0.8f) * (1000.0f / 2.65f) + 27.0f; // + mcuTemperatureAdjust;
}

pin_t HAL::digital_value_pin() {
  const pin_t pin = parser.value_pin();
  return WITHIN(pin, 0 , NUM_DIGITAL_PINS - 1) ? pin : NoPin;
}

pin_t HAL::analog_value_pin() {
  const pin_t pin = parser.value_pin();
  return WITHIN(pin, 0 , NUM_ANALOG_INPUTS - 1) ? pin : NoPin;
}

/**
 * Interrupt Service Routines
 */

// This intercepts the 1ms system tick. It must return 'false', otherwise the Arduino core tick handler will be bypassed.
extern "C" int sysTickHook() {
  HAL::Tick();
  return 0;
}

HAL_TONE_TIMER_ISR() {
  static uint8_t pin_state = 0;
  HAL_timer_isr_prologue(TONE_TIMER_NUM);

  if (toggles) {
    toggles--;
    HAL::digitalWrite(tone_pin, (pin_state ^= 1));
  }
  else noTone(tone_pin);
}

HAL_STEPPER_TIMER_ISR() {
  HAL_timer_isr_prologue(STEPPER_TIMER_NUM);
  // Call the Step
  stepper.Step();
}

#endif // ARDUINO_ARCH_SAM

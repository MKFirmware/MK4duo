/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../../MK4duo.h"
#include <Wire.h>

/** Public Parameters */
uint8_t MCUSR;

/** Private Parameters */
#if HAS_HOTENDS
  ADCAveragingFilter  HAL::HOTENDsensorFilters[MAX_HOTEND];
#endif
#if HAS_BEDS
  ADCAveragingFilter  HAL::BEDsensorFilters[MAX_BED];
#endif
#if HAS_CHAMBERS
  ADCAveragingFilter  HAL::CHAMBERsensorFilters[MAX_CHAMBER];
#endif
#if HAS_COOLERS
  ADCAveragingFilter  HAL::COOLERsensorFilters[MAX_COOLER];
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

// Return available memory
extern "C" {
  extern unsigned int _ebss; // end of bss section
  int freeMemory() {
    volatile char top;
    return &top - reinterpret_cast<char*>(_sbrk(0));
  }
}

// Tone for due
static pin_t tone_pin;
volatile static int32_t toggles;

void tone(const pin_t _pin, const uint16_t frequency, const uint16_t duration) {
  /*
  tone_pin = _pin;
  toggles = 2 * frequency * duration / 1000;
  HAL_timer_start(TONE_TIMER_NUM, 2 * frequency);
  */
}

void noTone(const pin_t _pin) {
  /*
  HAL_timer_disable_interrupt(TONE_TIMER_NUM);
  HAL::digitalWrite(_pin, LOW);
  */
}

// do any hardware-specific initialization here
void HAL::hwSetup() {

  HAL_InitTick(NvicPrioritySystick); // Start SysTick to priority low

  FastIO_init();

  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

}

// Print apparent cause of start/restart
void HAL::showStartReason() {
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)  != RESET) SERIAL_EM(MSG_HOST_WATCHDOG_RESET);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)   != RESET) SERIAL_EM(MSG_HOST_SOFTWARE_RESET);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)   != RESET) SERIAL_EM(MSG_HOST_EXTERNAL_RESET);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)   != RESET) SERIAL_EM(MSG_HOST_POWERUP);
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

// Initialize ADC channels
void HAL::analogStart() {

  analogReadResolution(ANALOG_INPUT_BITS);

  #if HAS_HOTENDS
    LOOP_HOTEND() {
      pinMode(hotends[h]->data.sensor.pin, INPUT);
    }
  #endif
  #if HAS_BEDS
    LOOP_BED() {
      pinMode(beds[h]->data.sensor.pin, INPUT);
    }
  #endif
  #if HAS_CHAMBERS
    LOOP_CHAMBER() {
      pinMode(chambers[h]->data.sensor.pin, INPUT);
    }
  #endif
  #if HAS_COOLERS
    LOOP_COOLER() {
      pinMode(coolers[h]->data.sensor.pin, INPUT);
    }
  #endif

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    pinMode(FILWIDTH_PIN, INPUT);
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    pinMode(POWER_CONSUMPTION_PIN, INPUT);
  #endif

  #if HAS_MCU_TEMPERATURE
    pinMode(ADC_TEMPERATURE_SENSOR, INPUT);
  #endif

}

void HAL::AdcChangePin(const pin_t old_pin, const pin_t new_pin) {
  UNUSED(old_pin);
  pinMode(new_pin, INPUT);
}

// Reset peripherals and cpu
void HAL::resetHardware() { NVIC_SystemReset(); }

/**
 * PWM output only work on the pins with hardware support.
 *  For the rest of the pins, we default to digital output
 */

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
  if (from != to) {
    if (from > to)
      value = (value < (uint32_t)(1 << (from - to))) ? 0 : ((value + 1) >> (from - to)) - 1;
    else if (value != 0)
      value = ((value + 1) << (to - from)) - 1;
  }
  return value;
}

void HAL::analogWrite(const pin_t pin, uint32_t ulValue, const uint16_t PWM_freq/*=1000U*/) {

  HardwareTimer* MK_pwm;

  PinName p = digitalPinToPinName(pin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(p, PinMap_PWM);
  uint32_t index = get_timer_index(Instance);

  if (HardwareTimer_Handle[index] == NULL)
    HardwareTimer_Handle[index]->__this = new HardwareTimer(Instance);

  MK_pwm = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(p, PinMap_PWM));

  MK_pwm->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, p);
  MK_pwm->setOverflow(PWM_freq, HERTZ_FORMAT);
  MK_pwm->setCaptureCompare(channel, ulValue, RESOLUTION_8B_COMPARE_FORMAT);
  MK_pwm->resume();

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

  #if HAS_HOTENDS
    LOOP_HOTEND() {
      ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(HOTENDsensorFilters[h]);
      currentFilter.process_reading(analogRead(hotends[h]->data.sensor.pin));
      if (currentFilter.IsValid())
        hotends[h]->data.sensor.adc_raw = (currentFilter.GetSum() / NUM_ADC_SAMPLES);
    }
  #endif
  #if HAS_BEDS
    LOOP_BED() {
      ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(BEDsensorFilters[h]);
      currentFilter.process_reading(analogRead(beds[h]->data.sensor.pin));
      if (currentFilter.IsValid())
        beds[h]->data.sensor.adc_raw = (currentFilter.GetSum() / NUM_ADC_SAMPLES);
    }
  #endif
  #if HAS_CHAMBERS
    LOOP_CHAMBER() {
      ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(CHAMBERsensorFilters[h]);
      currentFilter.process_reading(analogRead(chambers[h]->data.sensor.pin));
      if (currentFilter.IsValid())
        chambers[h]->data.sensor.adc_raw = (currentFilter.GetSum() / NUM_ADC_SAMPLES);
    }
  #endif
  #if HAS_COOLERS
    LOOP_COOLER() {
      ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(COOLERsensorFilters[h]);
      currentFilter.process_reading(analogRead(coolers[h]->data.sensor.pin));
      if (currentFilter.IsValid())
        coolers[h]->data.sensor.adc_raw = (currentFilter.GetSum() / NUM_ADC_SAMPLES);
    }
  #endif

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    const_cast<ADCAveragingFilter&>(filamentFilter).process_reading(analogRead(FILWIDTH_PIN));
    if (filamentFilter.IsValid())
      tempManager.current_raw_filwidth = (filamentFilter.GetSum() / NUM_ADC_SAMPLES);
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    const_cast<ADCAveragingFilter&>(powerFilter).process_reading(analogRead(POWER_CONSUMPTION_PIN));
    if (powerFilter.IsValid())
      powerManager.current_raw_powconsumption = (powerFilter.GetSum() / NUM_ADC_SAMPLES);
  #endif

  #if HAS_MCU_TEMPERATURE
    const_cast<ADCAveragingFilter&>(mcuFilter).process_reading(analogRead(ADC_TEMPERATURE_SENSOR));
    if (mcuFilter.IsValid())
      tempManager.mcu_current_temperature_raw = (mcuFilter.GetSum() / NUM_ADC_SAMPLES);
  #endif

  // Tick endstops state, if required
  endstops.Tick();

}

/**
 * Interrupt Service Routines
 */

// This intercepts the 1ms system tick.
extern "C" void HAL_SYSTICK_Callback(void) { HAL::Tick(); }

void Step_Handler(HardwareTimer*) { stepper.Step(); }


#endif // ARDUINO_ARCH_STM32

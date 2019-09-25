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

#ifdef ARDUINO_ARCH_STM32 && !defined(STM32GENERIC)

#include "../../../MK4duo.h"

/** Public Parameters */
uint8_t MCUSR;

int16_t HAL::AnalogInputValues[NUM_DIGITAL_PINS] = { 0 };
bool    HAL::Analog_is_ready = false;

/** Private Parameters */
#if MAX_HOTEND > 0
  ADCAveragingFilter HAL::HOTENDsensorFilters[MAX_HOTEND];
#endif
#if MAX_BED > 0
  ADCAveragingFilter HAL::BEDsensorFilters[MAX_BED];
#endif
#if MAX_CHAMBER > 0
  ADCAveragingFilter HAL::CHAMBERsensorFilters[MAX_CHAMBER];
#endif
#if MAX_COOLER > 0
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

HAL::HAL() {
  // ctor
}

HAL::~HAL() {
  // dtor
}

// do any hardware-specific initialization here
void HAL::hwSetup() {
  FastIO_init();
}

// Print apparent cause of start/restart
void HAL::showStartReason() {
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)  SERIAL_EM(MSG_WATCHDOG_RESET);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)   SERIAL_EM(MSG_SOFTWARE_RESET);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)   SERIAL_EM(MSG_EXTERNAL_RESET);
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)   SERIAL_EM(MSG_POWERUP);
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

// Initialize ADC channels
void HAL::analogStart() {

  analogReadResolution(ANALOG_INPUT_BITS);

  #if MAX_HOTEND > 0
    LOOP_HOTEND() {
      pinMode(hotends[h]->data.sensor.pin, INPUT);
      HOTENDsensorFilters[h].Init(0);
    }
  #endif
  #if MAX_BED > 0
    LOOP_BED() {
      pinMode(beds[h]->data.sensor.pin, INPUT);
      BEDsensorFilters[h].Init(0);
    }
  #endif
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER() {
      pinMode(chambers[h]->data.sensor.pin, INPUT);
      CHAMBERsensorFilters[h].Init(0);
    }
  #endif
  #if MAX_COOLER > 0
    LOOP_COOLER() {
      pinMode(coolers[h]->data.sensor.pin, INPUT);
      COOLERsensorFilters[h].Init(0);
    }
  #endif

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    pinMode(FILWIDTH_PIN, INPUT);
    filamentFilter.Init(0);
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    pinMode(POWER_CONSUMPTION_PIN, INPUT);
    powerFilter.Init(0);
  #endif

  #if HAS_MCU_TEMPERATURE
    pinMode(ADC_TEMPERATURE_SENSOR, INPUT);
    mcuFilter.Init(0);
  #endif

  HAL_timer_start(TEMP_TIMER_NUM, TEMP_TIMER_FREQUENCY);
  ENABLE_TEMP_INTERRUPT();

}

void HAL::AdcChangePin(const pin_t old_pin, const pin_t new_pin) {
  UNUSED(old_pin);
  pinMode(new_pin, INPUT);
}

// Reset peripherals and cpu
void HAL::resetHardware() {}

bool HAL::pwm_status(const pin_t pin) {
  return false;
}

bool HAL::tc_status(const pin_t pin) {
  return false;
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

void HAL::analogWrite(const pin_t pin, uint32_t ulValue, const uint16_t freq/*=1000U*/) {

  PinName p = digitalPinToPinName(pin);

  #ifdef HAL_DAC_MODULE_ENABLED
    if (pin_in_pinmap(p, PinMap_DAC)) {
      ulValue = mapResolution(ulValue, 8, DACC_RESOLUTION);
      dac_write_value(p, ulValue, 1);
      return;
    }
  #endif

  #ifdef HAL_TIM_MODULE_ENABLED
    if (pin_in_pinmap(p, PinMap_PWM)) {
      ulValue = mapResolution(ulValue, 8, PWM_RESOLUTION);
      pwm_start(p, freq * PWM_MAX_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE, ulValue, 1);
      return;
    }
  #endif

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

  static millis_s cycle_1s_ms   = millis(),
                  cycle_100_ms  = millis();

  if (printer.isStopped()) return;

  // Heaters set output PWM
  #if MAX_HOTEND > 0
    LOOP_HOTEND() hotends[h]->set_output_pwm();
  #endif
  #if MAX_BED > 0
    LOOP_BED() beds[h]->set_output_pwm();
  #endif
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER() chambers[h]->set_output_pwm();
  #endif
  #if MAX_COOLER > 0
    LOOP_COOLER() coolers[h]->set_output_pwm();
  #endif

  // Fans set output PWM
  #if MAX_FAN > 0
    LOOP_FAN() {
      if (fans[f]->kickstart) fans[f]->kickstart--;
      fans[f]->set_output_pwm();
    }
  #endif

  // Event 100 ms
  if (expired(&cycle_100_ms, 100U)) thermalManager.spin();

  // Event 1.0 Second
  if (expired(&cycle_1s_ms, 1000U)) printer.check_periodical_actions();

  #if MAX_HOTEND > 0
    LOOP_HOTEND() {
      ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(HOTENDsensorFilters[h]);
      currentFilter.ProcessReading(analogRead(hotends[h]->data.sensor.pin));
      if (currentFilter.IsValid()) {
        AnalogInputValues[hotends[h]->data.sensor.pin] = (currentFilter.GetSum() / NUM_ADC_SAMPLES);
        Analog_is_ready = true;
      }
    }
  #endif
  #if MAX_BED > 0
    LOOP_BED() {
      ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(BEDsensorFilters[h]);
      currentFilter.ProcessReading(analogRead(beds[h]->data.sensor.pin));
      if (currentFilter.IsValid()) {
        AnalogInputValues[beds[h]->data.sensor.pin] = (currentFilter.GetSum() / NUM_ADC_SAMPLES);
        Analog_is_ready = true;
      }
    }
  #endif
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER() {
      ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(CHAMBERsensorFilters[h]);
      currentFilter.ProcessReading(analogRead(chambers[h]->data.sensor.pin));
      if (currentFilter.IsValid()) {
        AnalogInputValues[chambers[h]->data.sensor.pin] = (currentFilter.GetSum() / NUM_ADC_SAMPLES);
        Analog_is_ready = true;
      }
    }
  #endif
  #if MAX_COOLER > 0
    LOOP_COOLER() {
      ADCAveragingFilter& currentFilter = const_cast<ADCAveragingFilter&>(COOLERsensorFilters[h]);
      currentFilter.ProcessReading(analogRead(coolers[h]->data.sensor.pin));
      if (currentFilter.IsValid()) {
        AnalogInputValues[coolers[h]->data.sensor.pin] = (currentFilter.GetSum() / NUM_ADC_SAMPLES);
        Analog_is_ready = true;
      }
    }
  #endif

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    const_cast<ADCAveragingFilter&>(filamentFilter).ProcessReading(analogRead(FILWIDTH_PIN));
    if (filamentFilter.IsValid())
      AnalogInputValues[FILWIDTH_PIN] = (filamentFilter.GetSum() / NUM_ADC_SAMPLES);
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    const_cast<ADCAveragingFilter&>(powerFilter).ProcessReading(analogRead(POWER_CONSUMPTION_PIN));
    if (powerFilter.IsValid())
      AnalogInputValues[POWER_CONSUMPTION_PIN] = (powerFilter.GetSum() / NUM_ADC_SAMPLES);
  #endif

  #if HAS_MCU_TEMPERATURE
    const_cast<ADCAveragingFilter&>(mcuFilter).ProcessReading(analogRead(ADC_TEMPERATURE_SENSOR));
    if (mcuFilter.IsValid())
      thermalManager.mcu_current_temperature_raw = (mcuFilter.GetSum() / NUM_ADC_SAMPLES);
  #endif

  // Update the raw values if they've been read. Else we could be updating them during reading.
  if (Analog_is_ready) thermalManager.set_current_temp_raw();

  // Tick endstops state, if required
  endstops.Tick();

}

/**
 * Interrupt Service Routines
 */
HAL_STEPPER_TIMER_ISR() { stepper.Step(); }

HAL_TEMP_TIMER_ISR() {
  if (printer.isStopped()) return;
  HAL::Tick();
}

#endif // ARDUINO_ARCH_STM32

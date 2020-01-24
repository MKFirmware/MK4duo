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
 * Description: HAL for Arduino and compatible
 *
 * Contributors:
 * Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
 *                    Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 *
 * ARDUINO_ARCH_ARM
 */

#ifdef __AVR__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "../../../MK4duo.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

uint16_t  HAL_pulse_high_tick     = 0,
          HAL_pulse_low_tick      = 0;
uint32_t  HAL_min_pulse_cycle     = 0,
          HAL_frequency_limit[8]  = { 0 };

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

extern "C" {
  extern char __bss_end;
  extern char __heap_start;
  extern void* __brkval;

  int freeMemory() {
    int free_memory;
    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}

void(* resetFunc) (void) = 0; // declare reset function @ address 0

#if ANALOG_INPUTS > 0
  int16_t HAL::AnalogInputValues[NUM_ANALOG_INPUTS] = { 0 };
#endif

const uint8_t AnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;

HAL::HAL() {
  // ctor
}

HAL::~HAL() {
  // dtor
}

void HAL_timer_start(const uint8_t timer_num) {

  switch (timer_num) {

    case STEPPER_TIMER_NUM:
      // waveform generation = 0100 = CTC
      SET_WGM(1, CTC_OCRnA);

      // output mode = 00 (disconnected)
      SET_COMA(1, NORMAL);

      // Set the timer pre-scaler
      // Generally we use a divider of 8, resulting in a 2MHz timer
      // frequency on a 16MHz MCU. If you are going to change this, be
      // sure to regenerate speed_lookuptable.h with
      // create_speed_lookuptable.py
      SET_CS(1, PRESCALER_8);  //  CS 2 = 1/8 prescaler

      // Init Stepper ISR to 122 Hz for quick starting
      OCR1A = 0x4000;
      TCNT1 = 0;
      break;

    case TEMP_TIMER_NUM:
      // Use timer0 for temperature measurement
      // Interleave temperature interrupt with millies interrupt
      TEMP_OCR = 128;
      break;
  }
}

uint32_t HAL_isr_execuiton_cycle(const uint32_t rate) {
  return (ISR_BASE_CYCLES + ISR_BEZIER_CYCLES + (ISR_LOOP_CYCLES) * rate + ISR_LA_BASE_CYCLES + ISR_LA_LOOP_CYCLES) / rate;
}

uint32_t HAL_ns_to_pulse_tick(const uint32_t ns) {
  return (ns + STEPPER_TIMER_PULSE_TICK_NS / 2) / STEPPER_TIMER_PULSE_TICK_NS;
}

void HAL_calc_pulse_cycle() {

  const uint32_t  HAL_min_step_period_ns = 1000000000UL / stepper.data.maximum_rate;
  uint32_t        HAL_min_pulse_high_ns,
                  HAL_min_pulse_low_ns;

  HAL_min_pulse_cycle = MAX(uint32_t((F_CPU) / stepper.data.maximum_rate), ((F_CPU) / 500000UL) * MAX(uint32_t(stepper.data.minimum_pulse), 1UL));

  if (stepper.data.minimum_pulse) {
    HAL_min_pulse_high_ns = uint32_t(stepper.data.minimum_pulse) * 1000UL;
    HAL_min_pulse_low_ns  = MAX((HAL_min_step_period_ns - MIN(HAL_min_step_period_ns, HAL_min_pulse_high_ns)), HAL_min_pulse_high_ns);
  }
  else {
    HAL_min_pulse_high_ns = 500000000UL / stepper.data.maximum_rate;
    HAL_min_pulse_low_ns  = HAL_min_pulse_high_ns;
  }

  HAL_pulse_high_tick = uint16_t(HAL_ns_to_pulse_tick(HAL_min_pulse_high_ns - MIN(HAL_min_pulse_high_ns, (TIMER_SETUP_NS))));
  HAL_pulse_low_tick  = uint16_t(HAL_ns_to_pulse_tick(HAL_min_pulse_low_ns - MIN(HAL_min_pulse_low_ns, (TIMER_SETUP_NS))));

  // The stepping frequency limits for each multistepping rate
  HAL_frequency_limit[0] = ((F_CPU) / HAL_isr_execuiton_cycle(1))       ;
  HAL_frequency_limit[1] = ((F_CPU) / HAL_isr_execuiton_cycle(2))   >> 1;
  HAL_frequency_limit[2] = ((F_CPU) / HAL_isr_execuiton_cycle(4))   >> 2;
  HAL_frequency_limit[3] = ((F_CPU) / HAL_isr_execuiton_cycle(8))   >> 3;
  HAL_frequency_limit[4] = ((F_CPU) / HAL_isr_execuiton_cycle(16))  >> 4;
  HAL_frequency_limit[5] = ((F_CPU) / HAL_isr_execuiton_cycle(32))  >> 5;
  HAL_frequency_limit[6] = ((F_CPU) / HAL_isr_execuiton_cycle(64))  >> 6;
  HAL_frequency_limit[7] = ((F_CPU) / HAL_isr_execuiton_cycle(128)) >> 7;
}

// Reset peripherals and cpu
void HAL::resetHardware() { resetFunc(); }

void HAL::showStartReason() {
  const uint8_t mcu = MCUSR;
  if (mcu &  1) SERIAL_EM(MSG_HOST_POWERUP);
  if (mcu &  2) SERIAL_EM(MSG_HOST_EXTERNAL_RESET);
  if (mcu &  4) SERIAL_EM(MSG_HOST_BROWNOUT_RESET);
  if (mcu &  8) SERIAL_EM(MSG_HOST_WATCHDOG_RESET);
  if (mcu & 32) SERIAL_EM(MSG_HOST_SOFTWARE_RESET);
  MCUSR = 0;
}

#if ANALOG_INPUTS > 0

  void HAL::analogStart() {

    #if MB(RUMBA) && ((TEMP_SENSOR_HE0==-1) || (TEMP_SENSOR_HE1==-1) || (TEMP_SENSOR_HE2==-1) || (TEMP_SENSOR_BED0==-1) || (TEMP_SENSOR_CHAMBER0==-1))
      // disable RUMBA JTAG in case the thermocouple extension is plugged on top of JTAG connector
      MCUCR = _BV(JTD);
      MCUCR = _BV(JTD);
    #endif

    ADMUX = ANALOG_REF; // refernce voltage

    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIF) | 0x07;
    DIDR0 = 0;
    #ifdef DIDR2
      DIDR2 = 0;
    #endif

    while (ADCSRA & _BV(ADSC) ) {} // wait for conversion

    const uint8_t channel = pgm_read_byte(&AnalogInputChannels[0]);

    #if ENABLED(MUX5)
      if (channel > 7)  // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
      else
        ADCSRB = 0;
    #else
      ADCSRB = 0;
    #endif

    ADMUX = _BV(REFS0) | (channel & 0x07);
    SBI(ADCSRA, ADSC);

    // Use timer for temperature measurement
    // Interleave temperature interrupt with millies interrupt
    HAL_timer_start(TEMP_TIMER_NUM);
    ENABLE_TEMP_INTERRUPT();

  }

  void HAL::AdcChangePin(const pin_t old_pin, const pin_t new_pin) {
    UNUSED(old_pin);
    UNUSED(new_pin);
  }

#endif

void HAL::hwSetup() { /*nope*/ }

void HAL::setPwmFrequency(const pin_t pin, uint8_t val) {
  val &= 0x07;
  switch (digitalPinToTimer(pin)) {

    #if ENABLED(TCCR0A)
      case TIMER0A:
      case TIMER0B:
        // TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
        // TCCR0B |= val;
        break;
    #endif

    #if ENABLED(TCCR1A)
      case TIMER1A:
      case TIMER1B:
        // TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
        // TCCR1B |= val;
        break;
    #endif

    #if ENABLED(TCCR2)
      case TIMER2:
      case TIMER2:
        TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
        TCCR2 |= val;
        break;
    #endif

    #if ENABLED(TCCR2A)
      case TIMER2A:
      case TIMER2B:
        TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
        TCCR2B |= val;
        break;
    #endif

    #if ENABLED(TCCR3A)
      case TIMER3A:
      case TIMER3B:
      case TIMER3C:
        TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
        TCCR3B |= val;
        break;
    #endif

    #if ENABLED(TCCR4A)
      case TIMER4A:
      case TIMER4B:
      case TIMER4C:
        TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
        TCCR4B |= val;
        break;
    #endif

    #if ENABLED(TCCR5A)
      case TIMER5A:
      case TIMER5B:
      case TIMER5C:
        TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
        TCCR5B |= val;
        break;
    #endif
  }
}

void HAL::analogWrite(const pin_t pin, const uint8_t uValue, const uint16_t freq/*=1000u*/) {
  UNUSED(freq);
  ::analogWrite(pin, uValue);
}

void HAL::Tick() {

  static short_timer_t  cycle_1s_timer(millis()),
                        cycle_100_timer(millis());

  static uint32_t AnalogInputRead[ANALOG_INPUTS]          = { 0 };
  static uint16_t sample[ANALOG_INPUTS][NUM_ADC_SAMPLES]  = { 0 };
  static uint8_t  adcCounter[ANALOG_INPUTS]               = { 0 },
                  adcSamplePos                            = 0;

  uint8_t channel = 0;

  if (printer.isStopped()) return;

  // Heaters set output PWM
  tempManager.set_output_pwm();

  // Fans set output PWM
  fanManager.set_output_pwm();

  // Event 100 ms
  if (cycle_100_timer.expired(100)) tempManager.spin();

  // Event 1.0 Second
  if (cycle_1s_timer.expired(1000)) printer.check_periodical_actions();

  if ((ADCSRA & _BV(ADSC)) == 0) {  // Conversion finished?
    channel = pgm_read_byte(&AnalogInputChannels[adcSamplePos]);
    const uint16_t read_adc = ADC;
    AnalogInputRead[adcSamplePos] += uint32_t(read_adc) - uint32_t(sample[adcSamplePos][adcCounter[adcSamplePos]]);
    sample[adcSamplePos][adcCounter[adcSamplePos]] = read_adc;
    if (++adcCounter[adcSamplePos] >= (NUM_ADC_SAMPLES)) {
      // update temperatures
      AnalogInputValues[channel] = AnalogInputRead[adcSamplePos] / (NUM_ADC_SAMPLES);
      adcCounter[adcSamplePos] = 0;
    }

    // Start next conversion
    if (++adcSamplePos >= ANALOG_INPUTS) adcSamplePos = 0;

    channel = pgm_read_byte(&AnalogInputChannels[adcSamplePos]);
    #if ENABLED(MUX5)
      if (channel > 7)  // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
      else
        ADCSRB = 0;
    #else
      ADCSRB = 0;
    #endif
    ADMUX = _BV(REFS0) | (channel & 0x07);
    SBI(ADCSRA, ADSC);

  }

  // Update the raw values if they've been read. Else we could be updating them during reading.
  set_current_temp_raw();

  // Tick endstops state, if required
  endstops.Tick();

}

pin_t HAL::digital_value_pin() {
  const pin_t pin = parser.value_pin();
  return WITHIN(pin, 0 , NUM_DIGITAL_PINS - 1) ? pin : NoPin;
}

pin_t HAL::analog_value_pin() { return NoPin; }

/** Private Function */
void HAL::set_current_temp_raw() {

  #if HAS_HOTENDS
    LOOP_HOTEND() hotends[h]->data.sensor.adc_raw = AnalogInputValues[hotends[h]->data.sensor.pin];
  #endif
  #if HAS_BEDS
    LOOP_BED() beds[h]->data.sensor.adc_raw = AnalogInputValues[beds[h]->data.sensor.pin];
  #endif
  #if HAS_CHAMBERS
    LOOP_CHAMBER() chambers[h]->data.sensor.adc_raw = AnalogInputValues[chambers[h]->data.sensor.pin];
  #endif
  #if HAS_COOLERS
    LOOP_COOLER() coolers[h]->data.sensor.adc_raw = AnalogInputValues[coolers[h]->data.sensor.pin];
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    powerManager.current_raw_powconsumption = AnalogInputValues[POWER_CONSUMPTION_PIN];
  #endif

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    current_raw_filwidth = AnalogInputValues[FILWIDTH_PIN];
  #endif

}

/**
 * Timer 0 is shared with millies so don't change the prescaler.
 *
 * On AVR this ISR uses the compare method so it runs at the base
 * frequency (16 MHz / 64 / 256 = 976.5625 Hz), but at the TCNT0 set
 * in OCR0B above (128 or halfway between OVFs).
 *
 *  - Manage PWM to all the heaters and fan
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 */
HAL_TEMP_TIMER_ISR {
  if (printer.isStopped()) return;
  HAL::Tick();
}

/**
 * Interrupt Service Routines
 */
HAL_STEPPER_TIMER_ISR() { stepper.Step(); }

#endif // __AVR__

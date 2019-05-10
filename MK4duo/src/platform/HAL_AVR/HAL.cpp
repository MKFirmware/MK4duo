/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

uint16_t  HAL_min_pulse_cycle     = 0,
          HAL_min_pulse_tick      = 0,
          HAL_add_pulse_ticks     = 0;

uint32_t  HAL_frequency_limit[8]  = { 0 };

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

#if ANALOG_INPUTS > 0
  int32_t AnalogInputRead[ANALOG_INPUTS];
  uint8_t adcCounter[ANALOG_INPUTS],
          adcSamplePos = 0;

  int16_t HAL::AnalogInputValues[NUM_ANALOG_INPUTS] = { 0 };
  bool    HAL::Analog_is_ready = false;
#endif

const uint8_t AnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;

HAL::HAL() {
  // ctor
}

HAL::~HAL() {
  // dtor
}

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {

  UNUSED(frequency);

  switch (timer_num) {

    case STEPPER_TIMER:
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

    case TEMP_TIMER:
      // Use timer0 for temperature measurement
      // Interleave temperature interrupt with millies interrupt
      TEMP_OCR  = 256;
      break;
  }
}

uint32_t HAL_isr_execuiton_cycle(const uint32_t rate) {
  return (ISR_BASE_CYCLES + ISR_BEZIER_CYCLES + (ISR_LOOP_CYCLES) * rate + ISR_LA_BASE_CYCLES + ISR_LA_LOOP_CYCLES) / rate;
}

void HAL_calc_pulse_cycle() {
  HAL_min_pulse_cycle = MAX((uint32_t)((F_CPU) / stepper.maximum_rate), ((F_CPU) / 500000UL) * MAX((uint32_t)stepper.minimum_pulse, 1UL));
  HAL_min_pulse_tick  = uint32_t(stepper.minimum_pulse) * (STEPPER_TIMER_TICKS_PER_US);
  HAL_add_pulse_ticks = (HAL_min_pulse_cycle / (PULSE_TIMER_PRESCALE)) - HAL_min_pulse_tick;

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

// Return available memory
extern "C" {
  extern char __bss_end;
  extern char __heap_start;
  extern void* __brkval;

  int HAL::getFreeRam() {
    int free_memory;
    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}

void(* resetFunc) (void) = 0; // declare reset function @ address 0

// Reset peripherals and cpu
void HAL::resetHardware() { resetFunc(); }

void HAL::showStartReason() {

  // Check startup - does nothing if bootloader sets MCUSR to 0
  const uint8_t mcu = MCUSR;
  if (mcu & 1)  SERIAL_EM(MSG_POWERUP);
  if (mcu & 2)  SERIAL_EM(MSG_EXTERNAL_RESET);
  if (mcu & 4)  SERIAL_EM(MSG_BROWNOUT_RESET);
  if (mcu & 8)  SERIAL_EM(MSG_WATCHDOG_RESET);
  if (mcu & 32) SERIAL_EM(MSG_SOFTWARE_RESET);

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
    for (uint8_t i = 0; i < ANALOG_INPUTS; i++) {
      adcCounter[i] = 0;
      AnalogInputRead[i] = 0;
    }

    ADCSRA = _BV(ADEN) | _BV(ADSC) | ANALOG_PRESCALER;

    DIDR0 = 0;
    #ifdef DIDR2
      DIDR2 = 0;
    #endif

    while (ADCSRA & _BV(ADSC) ) {} // wait for conversion

    const uint8_t channel = pgm_read_byte(&AnalogInputChannels[adcSamplePos]);

    #if ENABLED(ADCSRB) && ENABLED(MUX5)
      if (channel & 8)  // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
      else
        ADCSRB &= ~_BV(MUX5);
    #endif

    ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
    ADCSRA |= _BV(ADSC); // start conversion without interrupt!

    // Use timer for temperature measurement
    // Interleave temperature interrupt with millies interrupt
    HAL_timer_start(TEMP_TIMER, TEMP_TIMER_FREQUENCY);

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

void HAL::analogWrite(const pin_t pin, const uint8_t uValue, const uint16_t freq/*=1000U*/, const bool hwpwm/*=true*/) {
  UNUSED(freq);
  UNUSED(hwpwm);
  softpwm.set(pin, uValue);
}

void HAL::Tick() {

  static millis_s cycle_100_ms  = millis();
  static uint8_t  channel       = 0;

  // Heaters set output PWM
  #if HOTENDS > 0
    LOOP_HOTEND() hotends[h].set_output_pwm();
  #endif
  #if BEDS > 0
    LOOP_BED() beds[h].set_output_pwm();
  #endif
  #if CHAMBERS > 0
    LOOP_CHAMBER() chambers[h].set_output_pwm();
  #endif

  // Fans set output PWM
  #if FAN_COUNT > 0
    LOOP_FAN() fans[f].set_output_pwm();
  #endif

  // Software PWM modulation
  softpwm.spin();

  // Calculation cycle approximate a 100ms
  if (expired(&cycle_100_ms, 100U)) {
    // Temperature Spin
    thermalManager.spin();
    #if ENABLED(FAN_KICKSTART_TIME) && FAN_COUNT > 0
      LOOP_FAN() {
        if (fans[f].kickstart) fans[f].kickstart--;
      }
    #endif
  }

  // read analog values
  #if ANALOG_INPUTS > 0

    if ((ADCSRA & _BV(ADSC)) == 0) {  // Conversion finished?
      channel = pgm_read_byte(&AnalogInputChannels[adcSamplePos]);
      AnalogInputRead[adcSamplePos] += ADCW;
      if (++adcCounter[adcSamplePos] >= (OVERSAMPLENR)) {

        // update temperatures
        HAL::AnalogInputValues[channel] = AnalogInputRead[adcSamplePos] / (OVERSAMPLENR);

        AnalogInputRead[adcSamplePos] = 0;
        adcCounter[adcSamplePos] = 0;

        // Start next conversion
        if (++adcSamplePos >= ANALOG_INPUTS) {
          adcSamplePos = 0;
          HAL::Analog_is_ready = true;
        }
        channel = pgm_read_byte(&AnalogInputChannels[adcSamplePos]);
        #if ENABLED(ADCSRB) && ENABLED(MUX5)
          if (channel & 8)  // Reading channel 0-7 or 8-15?
            ADCSRB |= _BV(MUX5);
          else
            ADCSRB &= ~_BV(MUX5);
        #endif
        ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
      }
      ADCSRA |= _BV(ADSC);  // start next conversion
    }

    // Update the raw values if they've been read. Else we could be updating them during reading.
    if (HAL::Analog_is_ready) thermalManager.set_current_temp_raw();

  #endif

  // Tick endstops state, if required
  endstops.Tick();

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
  TEMP_OCR += 256;
  HAL::Tick();
}

/**
 * Interrupt Service Routines
 */
HAL_STEPPER_TIMER_ISR() { stepper.Step(); }

#endif // __AVR__

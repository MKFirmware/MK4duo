/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2015 - 2016 Alberto Cotronei @MagoKimbra
 *
 * ARDUINO_ARCH_ARM
 */

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../../base.h"

#if ENABLED(ARDUINO_ARCH_AVR)

#if ANALOG_INPUTS > 0
  int32_t AnalogInputRead[ANALOG_INPUTS];
  uint8_t adcCounter[ANALOG_INPUTS],
          adcSamplePos = 0;
  bool    Analog_is_ready = false;

  int16_t HAL::AnalogInputValues[ANALOG_INPUTS] = { 0 };
#endif

const uint8_t AnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
static unsigned int cycle_100ms = 0;

HAL::HAL() {
  // ctor
}

HAL::~HAL() {
  // dtor
}

void HAL_stepper_timer_start() {
  // waveform generation = 0100 = CTC
  CBI(TCCR1B, WGM13);
  SBI(TCCR1B, WGM12);
  CBI(TCCR1A, WGM11);
  CBI(TCCR1A, WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  // Init Stepper ISR to 122 Hz for quick starting
  OCR1A = 0x4000;
  TCNT1 = 0;
}

void HAL_temp_timer_start() {
  TCCR0A      =  0; // set entire TCCR2A register to 0
  TEMP_TCCR   =  0; // set entire TEMP_TCCR register to 0
  TEMP_TIMER  = 64; // Set divisor for 64 3906 Hz
  // Set CS01 and CS00 bits for 64 prescaler
  TEMP_TCCR |= (1 << CS01) | (1 << CS00);
}

bool HAL::execute_100ms = false;

// Return available memory
int HAL::getFreeRam() {
  int freeram = 0;
  InterruptProtectedBlock noInts;
  uint8_t * heapptr, * stackptr;
  heapptr = (uint8_t *)malloc(4);          // get heap pointer
  free(heapptr);      // free up the memory again (sets heapptr to 0)
  stackptr =  (uint8_t *)(SP);           // save value of stack pointer
  freeram = (int)stackptr-(int)heapptr;
  return freeram;
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

void HAL::analogStart() {

  #if ANALOG_INPUTS > 0

    ADMUX = ANALOG_REF; // refernce voltage
    for (uint8_t i = 0; i < ANALOG_INPUTS; i++) {
      adcCounter[i] = 0;
      AnalogInputRead[i] = 0;
    }

    ADCSRA = _BV(ADEN)|_BV(ADSC)|ANALOG_PRESCALER;

    while (ADCSRA & _BV(ADSC) ) {} // wait for conversion

    uint8_t channel = pgm_read_byte(&AnalogInputChannels[adcSamplePos]);

    #if ENABLED(ADCSRB) && ENABLED(MUX5)
      if (channel & 8)  // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
      else
        ADCSRB &= ~_BV(MUX5);
    #endif

    ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
    ADCSRA |= _BV(ADSC); // start conversion without interrupt!

  #endif
}

void HAL::setPwmFrequency(uint8_t pin, uint8_t val) {
  val &= 0x07;
  switch(digitalPinToTimer(pin)) {

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

/**
 * Timer 0 is is called 3906 timer per second.
 * It is used to update pwm values for heater and some other frequent jobs.
 *
 *  - Manage PWM to all the heaters and fan
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Step the babysteps value for each axis towards 0
 *  - For PINS_DEBUGGING, monitor and report endstop pins
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 */
HAL_TEMP_TIMER_ISR {

  // Allow UART ISRs
  _DISABLE_ISRs();

  TEMP_TIMER += 64;

  static uint8_t  pwm_count_heater        = 0,
                  pwm_count_fan           = 0;

  #if HAS_TEMP_HOTEND
    static uint8_t  pwm_heater_pos[HOTENDS] = { 0 };
  #endif
  #if HAS_HEATER_BED
    static uint8_t  pwm_bed_pos = 0;
  #endif
  #if HAS_HEATER_CHAMBER
    static uint8_t  pwm_chamber_pos = 0;
  #endif
  #if HAS_COOLER
    static uint8_t  pwm_cooler_pos = 0;
  #endif
  #if FAN_COUNT > 0
    static uint8_t  pwm_fan_pos[FAN_COUNT]  = { 0 };
  #endif
  #if HAS_CONTROLLERFAN
    uint8_t         pwm_controller_pos = 0;
  #endif

  #if ENABLED(FILAMENT_SENSOR)
    static unsigned long raw_filwidth_value = 0;
  #endif

  /**
   * Standard PWM modulation
   */
  if (pwm_count_heater == 0) {
    #if HOTENDS > 0
      if ((pwm_heater_pos[0] = (thermalManager.soft_pwm[0] & HEATER_PWM_MASK)) > 0)
        WRITE_HEATER_0(HIGH);
      #if HOTENDS > 1
        if ((pwm_heater_pos[1] = (thermalManager.soft_pwm[1] & HEATER_PWM_MASK)) > 0)
          WRITE_HEATER_1(HIGH);
        #if HOTENDS > 2
          if ((pwm_heater_pos[2] = (thermalManager.soft_pwm[2] & HEATER_PWM_MASK)) > 0)
            WRITE_HEATER_2(HIGH);
          #if HOTENDS > 3
            if ((pwm_heater_pos[3] = (thermalManager.soft_pwm[3] & HEATER_PWM_MASK)) > 0)
              WRITE_HEATER_0(HIGH);
          #endif
        #endif
      #endif
    #endif

    #if HAS_HEATER_BED && HAS_TEMP_BED
      if ((pwm_bed_pos = (thermalManager.soft_pwm_bed & HEATER_PWM_MASK)) > 0)
        WRITE_HEATER_BED(HIGH);
    #endif

    #if HAS_HEATER_CHAMBER && HAS_TEMP_CHAMBER
      if ((pwm_chamber_pos = (thermalManager.soft_pwm_chamber & HEATER_PWM_MASK)) > 0)
        WRITE_HEATER_CHAMBER(HIGH);
    #endif

    #if HAS_COOLER && HAS_TEMP_COOLER
      if ((pwm_cooler_pos = (thermalManager.soft_pwm_cooler & HEATER_PWM_MASK)) > 0)
        WRITE_COOLER(HIGH);
    #endif

  }

  if (pwm_count_fan == 0) {
    #if HAS_FAN0
      if ((pwm_fan_pos[0] = (printer.fanSpeeds[0] & FAN_PWM_MASK)) > 0)
        WRITE_FAN(HIGH);
    #endif
    #if HAS_FAN1
      if ((pwm_fan_pos[1] = (printer.fanSpeeds[1] & FAN_PWM_MASK)) > 0)
        WRITE_FAN1(HIGH);
    #endif
    #if HAS_FAN2
      if ((pwm_fan_pos[2] = (printer.fanSpeeds[2] & FAN_PWM_MASK)) > 0)
        WRITE_FAN2(HIGH);
    #endif
    #if HAS_FAN3
      if ((pwm_fan_pos[3] = (printer.fanSpeeds[3] & FAN_PWM_MASK)) > 0)
        WRITE_FAN3(HIGH);
    #endif
    #if HAS_CONTROLLERFAN
      if ((pwm_controller_pos = (printer.controller_fanSpeeds & FAN_PWM_MASK)) > 0)
        WRITE(CONTROLLERFAN_PIN, HIGH);
    #endif
  }

  #if HOTENDS > 0
    if (pwm_heater_pos[0] == pwm_count_heater && pwm_heater_pos[0] != HEATER_PWM_MASK) WRITE_HEATER_0(LOW);
    #if HOTENDS > 1
      if (pwm_heater_pos[1] == pwm_count_heater && pwm_heater_pos[1] != HEATER_PWM_MASK) WRITE_HEATER_1(LOW);
      #if HOTENDS > 2
        if (pwm_heater_pos[2] == pwm_count_heater && pwm_heater_pos[2] != HEATER_PWM_MASK) WRITE_HEATER_2(LOW);
        #if HOTENDS > 3
          if (pwm_heater_pos[3] == pwm_count_heater && pwm_heater_pos[3] != HEATER_PWM_MASK) WRITE_HEATER_3(LOW);
        #endif
      #endif
    #endif
  #endif

  #if HAS_HEATER_BED
    if (pwm_bed_pos == pwm_count_heater && pwm_bed_pos != HEATER_PWM_MASK) WRITE_HEATER_BED(LOW);
  #endif

  #if HAS_HEATER_CHAMBER && HAS_TEMP_CHAMBER
    if (pwm_chamber_pos == pwm_count_heater && pwm_chamber_pos != HEATER_PWM_MASK) WRITE_HEATER_CHAMBER(LOW);
  #endif

  #if HAS_COOLER && HAS_TEMP_COOLER
    if (pwm_cooler_pos == pwm_count_heater && pwm_cooler_pos != HEATER_PWM_MASK) WRITE_COOLER(LOW);
  #endif

  #if ENABLED(FAN_KICKSTART_TIME)
    if (printer.fanKickstart == 0)
  #endif
  {
    #if HAS_FAN0
      if (pwm_fan_pos[0] == pwm_count_fan && pwm_fan_pos[0] != FAN_PWM_MASK)
        WRITE_FAN(LOW);
    #endif
    #if HAS_FAN1
      if (pwm_fan_pos[1] == pwm_count_fan && pwm_fan_pos[1] != FAN_PWM_MASK)
        WRITE_FAN1(LOW);
    #endif
    #if HAS_FAN2
      if (pwm_fan_pos[2] == pwm_count_fan && pwm_fan_pos[2] != FAN_PWM_MASK)
        WRITE_FAN2(LOW);
    #endif
    #if HAS_FAN3
      if (pwm_fan_pos[3] == pwm_count_fan && pwm_fan_pos[3] != FAN_PWM_MASK)
        WRITE_FAN3(LOW);
    #endif
    #if HAS_CONTROLLERFAN
      if (pwm_controller_pos == pwm_count_fan && printer.controller_fanSpeeds != FAN_PWM_MASK)
        WRITE(CONTROLLERFAN_PIN, LOW);
    #endif
  }

  // Calculation cycle approximate a 100ms
  cycle_100ms++;
  if (cycle_100ms >= 390) {
    cycle_100ms = 0;
    HAL::execute_100ms = true;
    #if ENABLED(FAN_KICKSTART_TIME)
      if (printer.fanKickstart) printer.fanKickstart--;
    #endif
  }

  // read analog values
  #if ANALOG_INPUTS > 0

    if ((ADCSRA & _BV(ADSC)) == 0) {  // Conversion finished?
      AnalogInputRead[adcSamplePos] += ADCW;
      if (++adcCounter[adcSamplePos] >= _BV(OVERSAMPLENR)) {
        HAL::AnalogInputValues[adcSamplePos] =
          AnalogInputRead[adcSamplePos] >> (OVERSAMPLENR);
        AnalogInputRead[adcSamplePos] = 0;
        adcCounter[adcSamplePos] = 0;
        // Start next conversion
        if (++adcSamplePos >= ANALOG_INPUTS) {
          adcSamplePos = 0;
          Analog_is_ready = true;
        }
        uint8_t channel = pgm_read_byte(&AnalogInputChannels[adcSamplePos]);
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
    if (Analog_is_ready) thermalManager.set_current_temp_raw();

  #endif

  pwm_count_heater  += HEATER_PWM_STEP;
  pwm_count_fan     += FAN_PWM_STEP;

  #if ENABLED(BABYSTEPPING)
    LOOP_XYZ(axis) {
      int curTodo = thermalManager.babystepsTodo[axis]; //get rid of volatile for performance

      if (curTodo > 0) {
        stepper.babystep((AxisEnum)axis,/*fwd*/true);
        thermalManager.babystepsTodo[axis]--; //fewer to do next time
      }
      else if (curTodo < 0) {
        stepper.babystep((AxisEnum)axis,/*fwd*/false);
        thermalManager.babystepsTodo[axis]++; //fewer to do next time
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

  _ENABLE_ISRs(); // re-enable ISRs
}

#endif // ARDUINO_ARCH_AVR

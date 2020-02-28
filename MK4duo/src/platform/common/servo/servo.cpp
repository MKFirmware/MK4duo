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
 * servo.cpp - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
 * Copyright (c) 2009 Michael Margolis.  All right reserved.
 */

/**
 * A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
 * The servos are pulsed in the background using the value most recently written using the write() method
 *
 * Note that analogWrite of PWM on pins associated with the timer are disabled when the first servo is attached.
 * Timers are seized as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.
 *
 * The methods are:
 *
 * Servo - Class for manipulating servo motors connected to Arduino pins.
 *
 * attach(pin)           - Attach a servo motor to an i/o pin.
 * attach(pin, min, max) - Attach to a pin, setting min and max values in microseconds
 *                         Default min is 544, max is 2400
 *
 * write()               - Set the servo angle in degrees. (Invalid angles —over MIN_PULSE_WIDTH— are treated as µs.)
 * writeMicroseconds()   - Set the servo pulse width in microseconds.
 * move(pin, angle)      - Sequence of attach(pin), write(angle), safe_delay(servo_delay[servoIndex]).
 *                         With DEACTIVATE_SERVOS_AFTER_MOVE it detaches after servo_delay[servoIndex].
 * read()                - Get the last-written servo pulse width as an angle between 0 and 180.
 * readMicroseconds()    - Get the last-written servo pulse width in microseconds.
 * attached()            - Return true if a servo is attached.
 * detach()              - Stop an attached servo from pulsing its i/o pin.
 *
 */

#include "../../../../MK4duo.h"

#if HAS_SERVOS

MKServo servo[NUM_SERVOS];

void servo_init() {

  #if HAS_SERVO_0
    servo[0].attach(SERVO0_PIN);
    servo[0].detach();
  #endif
  #if HAS_SERVO_1
    servo[1].attach(SERVO1_PIN);
    servo[1].detach();
  #endif
  #if HAS_SERVO_2
    servo[2].attach(SERVO2_PIN);
    servo[2].detach();
  #endif
  #if HAS_SERVO_3
    servo[3].attach(SERVO3_PIN);
    servo[3].detach();
  #endif

  #if HAS_DONDOLO
    servo[DONDOLO_SERVO_INDEX].attach(0);
    servo[DONDOLO_SERVO_INDEX].write(DONDOLO_SERVOPOS_E0);
    #if (DONDOLO_SERVO_DELAY > 0)
      HAL::delayMilliseconds(DONDOLO_SERVO_DELAY);
      servo[DONDOLO_SERVO_INDEX].detach();
    #endif
  #endif

  #if HAS_Z_SERVO_PROBE
    /**
     * Set position of Z Servo Endstop
     *
     * The servo might be deployed and positioned too low to stow
     * when starting up the machine or rebooting the board.
     * There's no way to know where the nozzle is positioned until
     * homing has been done - no homing with z-probe without init!
     *
     */
    STOW_Z_SERVO();
  #endif
}

#endif // HAS_SERVOS

#if SHARED_SERVOS

ServoInfo_t servo_info[MAX_SERVOS]; // static array of servo structures
uint8_t ServoCount = 0;             // the total number of attached servo_info

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4) // minimum value in µs for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4) // maximum value in µs for this servo


/************ static functions common to all instances ***********************/
static bool isTimerActive(timer16_Sequence_t timer) {
  // returns true if any servo is active on this timer
  for (uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++) {
    if (SERVO(timer, channel).Pin.isActive)
      return true;
  }
  return false;
}
/****************** end of static functions ******************************/

MKServo::MKServo() {
  if (ServoCount < MAX_SERVOS) {
    index = ServoCount++;                                     // assign a servo index to this instance
    servo_info[index].ticks = usToTicks(DEFAULT_PULSE_WIDTH); // store default values
  }
  else {
    index = INVALID_SERVO;                                    // too many servos
  }
}

int8_t MKServo::attach(const pin_t inPin) {
  return attach(inPin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

int8_t MKServo::attach(const pin_t inPin, const int inMin, const int inMax) {

  if (index >= MAX_SERVOS) return -1;

  if (inPin > 0) servo_info[index].Pin.nbr = inPin;
  HAL::pinMode(servo_info[index].Pin.nbr, OUTPUT); // set servo pin to output

  // todo min/max check: ABS(min - MIN_PULSE_WIDTH) /4 < 128
  min = (MIN_PULSE_WIDTH - inMin) / 4;  // resolution of min/max is 4 µs
  max = (MAX_PULSE_WIDTH - inMax) / 4;

  // initialize the timer if it has not already been initialized
  timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(index);
  if (!isTimerActive(timer)) initISR(timer);
  servo_info[index].Pin.isActive = true;  // this must be set after the check for isTimerActive

  return index;
}

void MKServo::detach() {
  servo_info[index].Pin.isActive = false;
  digitalWrite(servo_info[index].Pin.nbr, LOW);
  timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(index);
  if (!isTimerActive(timer)) finISR(timer);
}

void MKServo::write(int value) {
  if (value < MIN_PULSE_WIDTH)  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    value = map(constrain(value, 0, 180), 0, 180, SERVO_MIN(), SERVO_MAX());

  writeMicroseconds(value);
}

void MKServo::writeMicroseconds(int value) {
  // calculate and store the values for the given channel
  byte channel = index;
  if (channel < MAX_SERVOS) {  // ensure channel is valid
    // ensure pulse width is valid
    value = constrain(value, SERVO_MIN(), SERVO_MAX()) - (TRIM_DURATION);
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead

    CRITICAL_SECTION_START();
      servo_info[channel].ticks = value;
    CRITICAL_SECTION_END();
  }
}

// return the value as degrees
int MKServo::read() { return map(readMicroseconds() + 1, SERVO_MIN(), SERVO_MAX(), 0, 180); }

int MKServo::readMicroseconds() {
  return (index == INVALID_SERVO) ? 0 : ticksToUs(servo_info[index].ticks) + TRIM_DURATION;
}

bool MKServo::attached() { return servo_info[index].Pin.isActive; }

void MKServo::move(const int value) {
  if (attach(0) >= 0) {
    write(value);
    HAL::delayMilliseconds(SERVO_DEACTIVATION_DELAY);
    #if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE)
      detach();
    #endif
  }
}

void MKServo::print_M281() {
  SERIAL_LM(CFG, "Servo Angles: P<Servo> L<Low> U<Up>:");
  SERIAL_SMV(CFG, "  M281 P", (int)index);
  SERIAL_MV(" L", angle[0]);
  SERIAL_MV(" U", angle[1]);
  SERIAL_EOL();
}

#endif // SHARED_SERVOS

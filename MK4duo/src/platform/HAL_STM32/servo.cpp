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

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../../MK4duo.h"

#if HAS_SERVOS

ServoInfo_t servo_info[MAX_SERVOS]; // static array of servo structures
uint8_t ServoCount = 0;             // the total number of attached servo_info

#define SERVO_MIN() (MIN_PULSE_WIDTH - min * 4)   // minimum value in µs for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - max * 4)   // maximum value in µs for this servo

/************ static functions common to all instances ***********************/
static HardwareTimer TimerServo(SERVO_TIMER);
static volatile int8_t timerChannel[_Nbr_16timers] = {-1};  // counter for the servo being pulsed for each timer (or -1 if refresh interval)
volatile uint32_t CumulativeCountSinceRefresh = 0;

static void Servo_PeriodElapsedCallback(HardwareTimer*) {
  // Only 1 timer used
  timer16_Sequence_t timer_id = _timer1;

  if (timerChannel[timer_id] < 0) // Restart from 1st servo
    CumulativeCountSinceRefresh = 0;
  else {
    if (timerChannel[timer_id] < ServoCount && servo_info[timerChannel[timer_id]].Pin.isActive == true)
      digitalWrite(servo_info[timerChannel[timer_id]].Pin.nbr, LOW); // pulse this channel low if activated
  }

  timerChannel[timer_id]++;    // increment to the next channel
  if (timerChannel[timer_id] < ServoCount && timerChannel[timer_id] < SERVOS_PER_TIMER) {
    TimerServo.setOverflow(servo_info[timerChannel[timer_id]].ticks);
    CumulativeCountSinceRefresh += servo_info[timerChannel[timer_id]].ticks;
    if (servo_info[timerChannel[timer_id]].Pin.isActive == true)
      digitalWrite(servo_info[timerChannel[timer_id]].Pin.nbr, HIGH);
  }
  else {
    if (CumulativeCountSinceRefresh + 4 < REFRESH_INTERVAL)
      TimerServo.setOverflow(REFRESH_INTERVAL - CumulativeCountSinceRefresh);
    else
      TimerServo.refresh();
    timerChannel[timer_id] = -1;
  }
}

static void TimerServoInit() {
  // prescaler is computed so that timer tick correspond to 1 microsecond
  uint32_t prescaler = TimerServo.getTimerClkFreq() / 1000000;
  TimerServo.setMode(1, TIMER_OUTPUT_COMPARE);
  TimerServo.setInterruptPriority(1, 0);
  TimerServo.setPrescaleFactor(prescaler);
  TimerServo.setOverflow(REFRESH_INTERVAL);
  TimerServo.attachInterrupt(Servo_PeriodElapsedCallback);
  TimerServo.resume();
}

static bool isTimerActive() {
  // returns true if any servo is active on this timer
  for (uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++) {
    if (servo_info[channel].Pin.isActive == true) return true;
  }
  return false;
}
/****************** end of static functions ******************************/

MKServo::MKServo() {
  if (ServoCount < MAX_SERVOS) {
    index = ServoCount++;                           // assign a servo index to this instance
    servo_info[index].ticks = DEFAULT_PULSE_WIDTH;  // store default values
  }
  else
    index = INVALID_SERVO;  // too many servo_info
}

int8_t MKServo::attach(const pin_t inPin) {
  return attach(inPin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

int8_t MKServo::attach(const pin_t inPin, const int inMin, const int inMax) {

  if (index >= MAX_SERVOS) return -1;

  if (inPin > 0) {
    servo_info[index].Pin.nbr = inPin;
    HAL::pinMode(servo_info[index].Pin.nbr, OUTPUT); // set servo pin to output
  }

  servo_info[index].ticks = DEFAULT_PULSE_WIDTH;

  // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128
  min  = (MIN_PULSE_WIDTH - inMin) / 4; //resolution of min/max is 4 µs
  max  = (MAX_PULSE_WIDTH - inMax) / 4;

  // initialize the timer if it has not already been initialized
  if (isTimerActive() == false) TimerServoInit();
  servo_info[index].Pin.isActive = true;  // this must be set after the check for isTimerActive

  return index;
}

void MKServo::detach() {
  servo_info[index].Pin.isActive = false;
  if (isTimerActive() == false) TimerServo.pause();
}

void MKServo::write(int value) {
  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
  if (value < MIN_PULSE_WIDTH)
    value = map(value, 0, 180, SERVO_MIN(), SERVO_MAX());

  writeMicroseconds(value);
}

void MKServo::writeMicroseconds(int value) {
  // calculate and store the values for the given channel
  byte channel = index;
  if ((channel < MAX_SERVOS)) { // ensure channel is valid
    LIMIT(value, SERVO_MIN(), SERVO_MAX());
    servo_info[channel].ticks = value;
  }
}

// return the value as degrees
int MKServo::read() { return map(readMicroseconds() + 1, SERVO_MIN(), SERVO_MAX(), 0, 180); }

int MKServo::readMicroseconds() {
  unsigned int pulsewidth;
  if (index != INVALID_SERVO) pulsewidth = servo_info[index].ticks;
  else pulsewidth  = 0;
  return pulsewidth;
}

bool MKServo::attached()  { return servo_info[index].Pin.isActive; }

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

#endif // HAS_SERVOS

#endif // ARDUINO_ARCH_STM32 && !defined(STM32GENERIC)

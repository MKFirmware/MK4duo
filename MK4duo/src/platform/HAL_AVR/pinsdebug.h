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
 * PWM print routines for Atmel 8 bit AVR CPUs
 */

#include "../../../MK4duo.h"

#define NUMBER_PINS_TOTAL NUM_DIGITAL_PINS

#if AVR_AT90USB1286_FAMILY
  // Working with Teensyduino extension so need to re-define some things
  #include "pinsDebug_Teensyduino.h"
  // Can't use the "digitalPinToPort" function from the Teensyduino type IDEs
  // portModeRegister takes a different argument
  #define digitalPinToTimer_DEBUG(p) digitalPinToTimer(p)
  #define digitalPinToBitMask_DEBUG(p) digitalPinToBitMask(p)
  #define digitalPinToPort_DEBUG(p) digitalPinToPort_Teensy(p)
  #define GET_PINMODE(pin) (*portModeRegister(pin) & digitalPinToBitMask_DEBUG(pin))
#elif AVR_ATmega2560_FAMILY_PLUS_70   // So we can access/display all the pins on boards using more than 70
  #include "pinsDebug_plus_70.h"
  #define digitalPinToTimer_DEBUG(p) digitalPinToTimer_plus_70(p)
  #define digitalPinToBitMask_DEBUG(p) digitalPinToBitMask_plus_70(p)
  #define digitalPinToPort_DEBUG(p) digitalPinToPort_plus_70(p)
  bool GET_PINMODE(pin_t pin) {return *portModeRegister(digitalPinToPort_DEBUG(pin)) & digitalPinToBitMask_DEBUG(pin); }

#else
  #define digitalPinToTimer_DEBUG(p) digitalPinToTimer(p)
  #define digitalPinToBitMask_DEBUG(p) digitalPinToBitMask(p)
  #define digitalPinToPort_DEBUG(p) digitalPinToPort(p)
  bool GET_PINMODE(pin_t pin) {return *portModeRegister(digitalPinToPort_DEBUG(pin)) & digitalPinToBitMask_DEBUG(pin); }
  #define GET_ARRAY_PIN(p) pgm_read_byte(&pin_array[p].pin)
#endif

#define VALID_PIN(pin) (pin >= 0 && pin < NUM_DIGITAL_PINS ? 1 : 0)
#if AVR_ATmega1284_FAMILY
  #define DIGITAL_PIN_TO_ANALOG_PIN(P) int(analogInputToDigitalPin(0) - (P))
  #define IS_ANALOG(P) ((P) >= analogInputToDigitalPin(7) && (P) <= analogInputToDigitalPin(0))
#else
  #define DIGITAL_PIN_TO_ANALOG_PIN(P) int((P) - analogInputToDigitalPin(0))
  #define IS_ANALOG(P) ((P) >= analogInputToDigitalPin(0) && ((P) <= analogInputToDigitalPin(15) || (P) <= analogInputToDigitalPin(7)))
#endif
#define GET_ARRAY_PIN(p) pgm_read_byte(&pin_array[p].pin)
#define MULTI_NAME_PAD 26 // space needed to be pretty if not first name assigned to a pin

void PRINT_ARRAY_NAME(uint8_t x) {
  char *name_mem_pointer = (char*)pgm_read_ptr(&pin_array[x].name);
  for (uint8_t y = 0; y < MAX_NAME_LENGTH; y++) {
    char temp_char = pgm_read_byte(name_mem_pointer + y);
    if (temp_char != 0)
      SERIAL_CHR(temp_char);
    else {
      for (uint8_t i = 0; i < MAX_NAME_LENGTH - y; i++) SERIAL_CHR(' ');
      break;
    }
  }
}

#define GET_ARRAY_IS_DIGITAL(x)   pgm_read_byte(&pin_array[x].is_digital)

#if ENABLED(__AVR_ATmega1284P__)  // 1284 IDE extensions set this to the number of
  #undef NUM_DIGITAL_PINS         // digital only pins while all other CPUs have it
  #define NUM_DIGITAL_PINS 32     // set to digital only + digital/analog
#endif

#define PWM_PRINT(V) do{ sprintf_P(buffer, PSTR("PWM:  %4d"), V); SERIAL_PGM(buffer); }while(0)
#define PWM_CASE(N,Z)                                           \
  case TIMER##N##Z:                                             \
    if (TCCR##N##A & (_BV(COM##N##Z##1) | _BV(COM##N##Z##0))) { \
      PWM_PRINT(OCR##N##Z);                                     \
      return true;                                              \
    } else return false



/**
 * Print a pin's PWM status.
 * Return true if it's currently a PWM pin.
 */
static bool pwm_status(pin_t pin) {
  char buffer[20];   // for the sprintf statements

  switch (digitalPinToTimer_DEBUG(pin)) {

    #if ENABLED(TCCR0A) && ENABLED(COM0A1)
      #if ENABLED(TIMER0A)
        PWM_CASE(0, A);
      #endif
      PWM_CASE(0, B);
    #endif

    #if ENABLED(TCCR1A) && ENABLED(COM1A1)
      PWM_CASE(1, A);
      PWM_CASE(1, B);
     #if ENABLED(COM1C1) && ENABLED(TIMER1C)
      PWM_CASE(1, C);
     #endif
    #endif

    #if ENABLED(TCCR2A) && ENABLED(COM2A1)
      PWM_CASE(2, A);
      PWM_CASE(2, B);
    #endif

    #if ENABLED(TCCR3A) && ENABLED(COM3A1)
      PWM_CASE(3, A);
      PWM_CASE(3, B);
      #if ENABLED(COM3C1)
        PWM_CASE(3, C);
      #endif
    #endif

    #if ENABLED(TCCR4A)
      PWM_CASE(4, A);
      PWM_CASE(4, B);
      PWM_CASE(4, C);
    #endif

    #if ENABLED(TCCR5A) && ENABLED(COM5A1)
      PWM_CASE(5, A);
      PWM_CASE(5, B);
      PWM_CASE(5, C);
    #endif

    case NOT_ON_TIMER:
    default:
      return false;
  }

  SERIAL_SP(2);
} // pwm_status


const volatile uint8_t* const PWM_other[][3] PROGMEM = {
    { &TCCR0A, &TCCR0B, &TIMSK0 },
    { &TCCR1A, &TCCR1B, &TIMSK1 },
  #if ENABLED(TCCR2A) && ENABLED(COM2A1)
    { &TCCR2A, &TCCR2B, &TIMSK2 },
  #endif
  #if ENABLED(TCCR3A) && ENABLED(COM3A1)
    { &TCCR3A, &TCCR3B, &TIMSK3 },
  #endif
  #ifdef TCCR4A
    { &TCCR4A, &TCCR4B, &TIMSK4 },
  #endif
  #if ENABLED(TCCR5A) && ENABLED(COM5A1)
    { &TCCR5A, &TCCR5B, &TIMSK5 },
  #endif
};


const volatile uint8_t* const PWM_OCR[][3] PROGMEM = {

  #if ENABLED(TIMER0A)
    { &OCR0A, &OCR0B, 0 },
  #else
    { 0, &OCR0B, 0 },
  #endif

  #if ENABLED(COM1C1) && ENABLED(TIMER1C)
   { (const uint8_t*)&OCR1A, (const uint8_t*)&OCR1B, (const uint8_t*)&OCR1C },
  #else
   { (const uint8_t*)&OCR1A, (const uint8_t*)&OCR1B, 0 },
  #endif

  #if ENABLED(TCCR2A) && ENABLED(COM2A1)
    { &OCR2A, &OCR2B, 0 },
  #endif

  #if ENABLED(TCCR3A) && ENABLED(COM3A1)
    #ifdef COM3C1
      { (const uint8_t*)&OCR3A, (const uint8_t*)&OCR3B, (const uint8_t*)&OCR3C },
    #else
      { (const uint8_t*)&OCR3A, (const uint8_t*)&OCR3B, 0 },
    #endif
  #endif

  #if ENABLED(TCCR4A)
    { (const uint8_t*)&OCR4A, (const uint8_t*)&OCR4B, (const uint8_t*)&OCR4C },
  #endif

  #if ENABLED(TCCR5A) && ENABLED(COM5A1)
    { (const uint8_t*)&OCR5A, (const uint8_t*)&OCR5B, (const uint8_t*)&OCR5C },
  #endif
};


#define TCCR_A(T)   pgm_read_word(&PWM_other[T][0])
#define TCCR_B(T)   pgm_read_word(&PWM_other[T][1])
#define TIMSK(T)    pgm_read_word(&PWM_other[T][2])
#define CS_0       0
#define CS_1       1
#define CS_2       2
#define WGM_0      0
#define WGM_1      1
#define WGM_2      3
#define WGM_3      4
#define TOIE       0

#define OCR_VAL(T, L)   pgm_read_word(&PWM_OCR[T][L])

static void err_is_counter()     { SERIAL_MSG("   non-standard PWM mode"); }
static void err_is_interrupt()   { SERIAL_MSG("   compare interrupt enabled"); }
static void err_prob_interrupt() { SERIAL_MSG("   overflow interrupt enabled"); }
static void print_is_also_tied() { SERIAL_MSG(" is also tied to this pin"); SERIAL_SP(14); }

void com_print(uint8_t N, uint8_t Z) {
  const uint8_t *TCCRA = (uint8_t*)TCCR_A(N);
  SERIAL_MSG("    COM");
  SERIAL_CHR(N + '0');
  switch (Z) {
    case 'A':
      SERIAL_MV("A: ", ((*TCCRA & (_BV(7) | _BV(6))) >> 6));
      break;
    case 'B':
      SERIAL_MV("B: ", ((*TCCRA & (_BV(5) | _BV(4))) >> 4));
      break;
    case 'C':
      SERIAL_MV("C: ", ((*TCCRA & (_BV(3) | _BV(2))) >> 2));
      break;
  }
}

void timer_prefix(uint8_t T, char L, uint8_t N) {  // T - timer    L - pwm  N - WGM bit layout
  char buffer[20];   // for the sprintf statements
  const uint8_t *TCCRB = (uint8_t*)TCCR_B(T),
                *TCCRA = (uint8_t*)TCCR_A(T);
  uint8_t WGM = (((*TCCRB & _BV(WGM_2)) >> 1) | (*TCCRA & (_BV(WGM_0) | _BV(WGM_1))));
  if (N == 4) WGM |= ((*TCCRB & _BV(WGM_3)) >> 1);

  SERIAL_MSG("    TIMER");
  SERIAL_CHR(T + '0');
  SERIAL_CHR(L);
  SERIAL_SP(3);

  if (N == 3) {
    const uint8_t *OCRVAL8 = (uint8_t*)OCR_VAL(T, L - 'A');
    PWM_PRINT(*OCRVAL8);
  }
  else {
    const uint16_t *OCRVAL16 = (uint16_t*)OCR_VAL(T, L - 'A');
    PWM_PRINT(*OCRVAL16);
  }
  SERIAL_MV("    WGM: ", WGM);
  com_print(T,L);
  SERIAL_MV("    CS: ", (*TCCRB & (_BV(CS_0) | _BV(CS_1) | _BV(CS_2)) ));

  SERIAL_MSG("    TCCR");
  SERIAL_CHR(T + '0');
  SERIAL_MV("A: ", *TCCRA);

  SERIAL_MSG("    TCCR");
  SERIAL_CHR(T + '0');
  SERIAL_MV("B: ", *TCCRB);

  const uint8_t *TMSK = (uint8_t*)TIMSK(T);
  SERIAL_MSG("    TIMSK");
  SERIAL_CHR(T + '0');
  SERIAL_MV(": ", *TMSK);

  const uint8_t OCIE = L - 'A' + 1;
  if (N == 3) { if (WGM == 0 || WGM == 2 || WGM ==  4 || WGM ==  6) err_is_counter(); }
  else        { if (WGM == 0 || WGM == 4 || WGM == 12 || WGM == 13) err_is_counter(); }
  if (TEST(*TMSK, OCIE)) err_is_interrupt();
  if (TEST(*TMSK, TOIE)) err_prob_interrupt();
}

static void pwm_details(pin_t pin) {
  switch (digitalPinToTimer_DEBUG(pin)) {

    #if ENABLED(TCCR0A) && ENABLED(COM0A1)
      #if ENABLED(TIMER0A)
        case TIMER0A: timer_prefix(0, 'A', 3); break;
      #endif
      case TIMER0B: timer_prefix(0, 'B', 3); break;
    #endif

    #if ENABLED(TCCR1A) && ENABLED(COM1A1)
      case TIMER1A: timer_prefix(1, 'A', 4); break;
      case TIMER1B: timer_prefix(1, 'B', 4); break;
      #if ENABLED(COM1C1) && ENABLED(TIMER1C)
        case TIMER1C: timer_prefix(1, 'C', 4); break;
      #endif
    #endif

    #if ENABLED(TCCR2A) && ENABLED(COM2A1)
      case TIMER2A: timer_prefix(2, 'A', 3); break;
      case TIMER2B: timer_prefix(2, 'B', 3); break;
    #endif

    #if ENABLED(TCCR3A) && ENABLED(COM3A1)
      case TIMER3A: timer_prefix(3, 'A', 4); break;
      case TIMER3B: timer_prefix(3, 'B', 4); break;
      #ifdef COM3C1
        case TIMER3C: timer_prefix(3, 'C', 4); break;
      #endif
    #endif

    #if ENABLED(TCCR4A)
      case TIMER4A: timer_prefix(4, 'A', 4); break;
      case TIMER4B: timer_prefix(4, 'B', 4); break;
      case TIMER4C: timer_prefix(4, 'C', 4); break;
    #endif

    #if ENABLED(TCCR5A) && ENABLED(COM5A1)
      case TIMER5A: timer_prefix(5, 'A', 4); break;
      case TIMER5B: timer_prefix(5, 'B', 4); break;
      case TIMER5C: timer_prefix(5, 'C', 4); break;
    #endif

    case NOT_ON_TIMER: break;

  }
  SERIAL_MSG("  ");

  // on pins that have two PWMs, print info on second PWM
  #if AVR_ATmega2560_FAMILY || AVR_AT90USB1286_FAMILY
    // looking for port B7 - PWMs 0A and 1C
    if (digitalPinToPort_DEBUG(pin) == 'B' - 64 && 0x80 == digitalPinToBitMask_DEBUG(pin)) {
      #if !AVR_AT90USB1286_FAMILY
        SERIAL_MSG("\n .");
        SERIAL_SP(18);
        SERIAL_MSG("TIMER1C");
        print_is_also_tied();
        timer_prefix(1, 'C', 4);
      #else
        SERIAL_MSG("\n .");
        SERIAL_SP(18);
        SERIAL_MSG("TIMER0A");
        print_is_also_tied();
        timer_prefix(0, 'A', 3);
      #endif
    }
  #endif
} // pwm_details


#if DISABLED(digitalRead_mod)
  int digitalRead_mod(const pin_t pin) { // same as digitalRead except the PWM stop section has been removed
    const uint8_t port = digitalPinToPort_DEBUG(pin);
    return (port != NOT_A_PIN) && (*portInputRegister(port) & digitalPinToBitMask_DEBUG(pin)) ? HIGH : LOW;
  }
#endif

#if DISABLED(PRINT_PORT)

  void print_port(pin_t pin) {   // print port number
    #if ENABLED(digitalPinToPort_DEBUG)
      uint8_t x;
      SERIAL_MSG("  Port: ");
      #if AVR_AT90USB1286_FAMILY
        x = (pin == 46 || pin == 47) ? 'E' : digitalPinToPort_DEBUG(pin) + 64;
      #else
        x = digitalPinToPort_DEBUG(pin) + 64;
      #endif
      SERIAL_CHR(x);

      #if AVR_AT90USB1286_FAMILY
        if (pin == 46)
          x = '2';
        else if (pin == 47)
          x = '3';
        else {
          uint8_t temp = digitalPinToBitMask_DEBUG(pin);
          for (x = '0'; x < '9' && temp != 1; x++) temp >>= 1;
        }
      #else
        uint8_t temp = digitalPinToBitMask_DEBUG(pin);
        for (x = '0'; x < '9' && temp != 1; x++) temp >>= 1;
      #endif
      SERIAL_CHR(x);
    #else
      SERIAL_SP(10);
    #endif
  }

  #define PRINT_PORT(p) print_port(p)

#endif

#define PRINT_PIN(p) do {sprintf_P(buffer, PSTR("%3d "), p); SERIAL_PGM(buffer);} while (0)

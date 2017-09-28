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
 * Copyright (c) 2013 Arduino LLC. All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "../../../base.h"

#if HAS_SERVOS

  #include "servo.h"

  Servo servo[NUM_SERVOS];

  void servo_init() {

    #if NUM_SERVOS >= 1 && HAS_SERVO_0
      servo[0].attach(SERVO0_PIN);
      servo[0].detach(); // Just set up the pin. We don't have a position yet. Don't move to a random position.
    #endif
    #if NUM_SERVOS >= 2 && HAS_SERVO_1
      servo[1].attach(SERVO1_PIN);
      servo[1].detach();
    #endif
    #if NUM_SERVOS >= 3 && HAS_SERVO_2
      servo[2].attach(SERVO2_PIN);
      servo[2].detach();
    #endif
    #if NUM_SERVOS >= 4 && HAS_SERVO_3
      servo[3].attach(SERVO3_PIN);
      servo[3].detach();
    #endif

    #if HAS_DONDOLO
      servo[DONDOLO_SERVO_INDEX].attach(0);
      servo[DONDOLO_SERVO_INDEX].write(DONDOLO_SERVOPOS_E0);
      #if (DONDOLO_SERVO_DELAY > 0)
        printer.safe_delay(DONDOLO_SERVO_DELAY);
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

  #define usToTicks(_us)    (( clockCyclesPerMicrosecond() * _us) / SERVO_TIMER_PRESCALER)              // converts microseconds to tick (PRESCALER depends on architecture)
  #define ticksToUs(_ticks) (((unsigned)_ticks * SERVO_TIMER_PRESCALER) / clockCyclesPerMicrosecond())  // converts from ticks back to microseconds

  static ServoInfo_t servo_info[MAX_SERVOS];                  // static array of servo structures
  static volatile int8_t Channel[_Nbr_16timers ];             // counter for the servo being pulsed for each timer (or -1 if refresh interval)

  uint8_t ServoCount = 0;                                     // the total number of attached servo_info

  // convenience macros
  #define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER)) // returns the timer controlling this servo
  #define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)       // returns the index of the servo on this timer
  #define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)     // macro to access servo index by timer and channel
  #define SERVO(_timer,_channel)  (servo_info[SERVO_INDEX(_timer, _channel)])      // macro to access servo class by timer and channel

  #define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
  #define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo

  /************ static functions common to all instances ***********************/

  #if ENABLED(ARDUINO_ARCH_SAM)
    //------------------------------------------------------------------------------
    /// Interrupt handler for the TC0 channel 1.
    //------------------------------------------------------------------------------
    void Servo_Handler(timer16_Sequence_t timer, Tc *pTc, uint8_t channel);
    #if ENABLED (_useTimer1)
      void HANDLER_FOR_TIMER1(void) {
        Servo_Handler(_timer1, TC_FOR_TIMER1, CHANNEL_FOR_TIMER1);
      }
    #endif
    #if ENABLED (_useTimer2)
      void HANDLER_FOR_TIMER2(void) {
        Servo_Handler(_timer2, TC_FOR_TIMER2, CHANNEL_FOR_TIMER2);
      }
    #endif
    #if ENABLED (_useTimer3)
      void HANDLER_FOR_TIMER3(void) {
        Servo_Handler(_timer3, TC_FOR_TIMER3, CHANNEL_FOR_TIMER3);
      }
    #endif
    #if ENABLED (_useTimer4)
      void HANDLER_FOR_TIMER4(void) {
        Servo_Handler(_timer4, TC_FOR_TIMER4, CHANNEL_FOR_TIMER4);
      }
    #endif
    #if ENABLED (_useTimer5)
      void HANDLER_FOR_TIMER5(void) {
        Servo_Handler(_timer5, TC_FOR_TIMER5, CHANNEL_FOR_TIMER5);
      }
    #endif

    void Servo_Handler(timer16_Sequence_t timer, Tc *tc, uint8_t channel) {
      // clear interrupt
      tc->TC_CHANNEL[channel].TC_SR;
      if (Channel[timer] < 0) {
        tc->TC_CHANNEL[channel].TC_CCR |= TC_CCR_SWTRG; // channel set to -1 indicated that refresh interval completed so reset the timer
      }
      else {
        if (SERVO_INDEX(timer,Channel[timer]) < ServoCount && SERVO(timer,Channel[timer]).Pin.isActive == true) {
          digitalWrite(SERVO(timer,Channel[timer]).Pin.nbr, LOW); // pulse this channel low if activated
        }
      }

      Channel[timer]++;    // increment to the next channel
      if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
        tc->TC_CHANNEL[channel].TC_RA = tc->TC_CHANNEL[channel].TC_CV + SERVO(timer,Channel[timer]).ticks;
        if(SERVO(timer,Channel[timer]).Pin.isActive == true) {    // check if activated
          digitalWrite(SERVO(timer,Channel[timer]).Pin.nbr,HIGH); // its an active channel so pulse it high
        }
      }
      else {
        // finished all channels so wait for the refresh period to expire before starting over
        if( (tc->TC_CHANNEL[channel].TC_CV) + 4 < usToTicks(REFRESH_INTERVAL) ) { // allow a few ticks to ensure the next OCR1A not missed
          tc->TC_CHANNEL[channel].TC_RA = (unsigned int)usToTicks(REFRESH_INTERVAL);
        }
        else {
          tc->TC_CHANNEL[channel].TC_RA = tc->TC_CHANNEL[channel].TC_CV + 4;  // at least REFRESH_INTERVAL has elapsed
        }
        Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel
      }
    }

    static void _initISR(Tc *tc, uint32_t channel, uint32_t id, IRQn_Type irqn) {
      pmc_enable_periph_clk(id);
      TC_Configure(tc, channel,
        TC_CMR_TCCLKS_TIMER_CLOCK3 | // MCK/32
        TC_CMR_WAVE |                // Waveform mode
        TC_CMR_WAVSEL_UP_RC );       // Counter running up and reset when equals to RC

      /* 84MHz, MCK/32, for 1.5ms: 3937 */
      TC_SetRA(tc, channel, 2625); // 1ms

      /* Configure and enable interrupt */
      NVIC_EnableIRQ(irqn);
      // TC_IER_CPAS: RA Compare
      tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPAS;

      // Enables the timer clock and performs a software reset to start the counting
      TC_Start(tc, channel);
    }

    static void initISR(timer16_Sequence_t timer) {
      #if ENABLED (_useTimer1)
        if (timer == _timer1)
          _initISR(TC_FOR_TIMER1, CHANNEL_FOR_TIMER1, ID_TC_FOR_TIMER1, IRQn_FOR_TIMER1);
      #endif
      #if ENABLED (_useTimer2)
        if (timer == _timer2)
          _initISR(TC_FOR_TIMER2, CHANNEL_FOR_TIMER2, ID_TC_FOR_TIMER2, IRQn_FOR_TIMER2);
      #endif
      #if ENABLED (_useTimer3)
        if (timer == _timer3)
          _initISR(TC_FOR_TIMER3, CHANNEL_FOR_TIMER3, ID_TC_FOR_TIMER3, IRQn_FOR_TIMER3);
      #endif
      #if ENABLED (_useTimer4)
        if (timer == _timer4)
          _initISR(TC_FOR_TIMER4, CHANNEL_FOR_TIMER4, ID_TC_FOR_TIMER4, IRQn_FOR_TIMER4);
      #endif
      #if ENABLED (_useTimer5)
        if (timer == _timer5)
          _initISR(TC_FOR_TIMER5, CHANNEL_FOR_TIMER5, ID_TC_FOR_TIMER5, IRQn_FOR_TIMER5);
      #endif
    }

    static void finISR(timer16_Sequence_t timer) {
      UNUSED(timer);
      #if ENABLED (_useTimer1)
        TC_Stop(TC_FOR_TIMER1, CHANNEL_FOR_TIMER1);
      #endif
      #if ENABLED (_useTimer2)
        TC_Stop(TC_FOR_TIMER2, CHANNEL_FOR_TIMER2);
      #endif
      #if ENABLED (_useTimer3)
        TC_Stop(TC_FOR_TIMER3, CHANNEL_FOR_TIMER3);
      #endif
      #if ENABLED (_useTimer4)
        TC_Stop(TC_FOR_TIMER4, CHANNEL_FOR_TIMER4);
      #endif
      #if ENABLED (_useTimer5)
        TC_Stop(TC_FOR_TIMER5, CHANNEL_FOR_TIMER5);
      #endif
    }

  #else // !ARDUINO_ARCH_SAM

    static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA) {
      if (Channel[timer] < 0)
        *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer
      else {
        if (SERVO_INDEX(timer,Channel[timer]) < ServoCount && SERVO(timer,Channel[timer]).Pin.isActive == true)
          digitalWrite(SERVO(timer, Channel[timer]).Pin.nbr, LOW); // pulse this channel low if activated
      }

      Channel[timer]++;    // increment to the next channel
      if (SERVO_INDEX(timer, Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
        *OCRnA = *TCNTn + SERVO(timer, Channel[timer]).ticks;
        if (SERVO(timer, Channel[timer]).Pin.isActive)     // check if activated
          digitalWrite(SERVO(timer, Channel[timer]).Pin.nbr, HIGH); // its an active channel so pulse it high
      }
      else {
        // finished all channels so wait for the refresh period to expire before starting over
        if (((unsigned)*TCNTn) + 4 < usToTicks(REFRESH_INTERVAL))  // allow a few ticks to ensure the next OCR1A not missed
          *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);
        else
          *OCRnA = *TCNTn + 4;  // at least REFRESH_INTERVAL has elapsed
        Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel
      }
    }

    #ifndef WIRING // Wiring pre-defines signal handlers so don't define any if compiling for the Wiring platform

      // Interrupt handlers for Arduino
      #if ENABLED(_useTimer1)
        SIGNAL (TIMER1_COMPA_vect) { handle_interrupts(_timer1, &TCNT1, &OCR1A); }
      #endif

      #if ENABLED(_useTimer3)
        SIGNAL (TIMER3_COMPA_vect) { handle_interrupts(_timer3, &TCNT3, &OCR3A); }
      #endif

      #if ENABLED(_useTimer4)
        SIGNAL (TIMER4_COMPA_vect) { handle_interrupts(_timer4, &TCNT4, &OCR4A); }
      #endif

      #if ENABLED(_useTimer5)
        SIGNAL (TIMER5_COMPA_vect) { handle_interrupts(_timer5, &TCNT5, &OCR5A); }
      #endif

    #else // !WIRING
    
      // Interrupt handlers for Wiring
      #if ENABLED(_useTimer1)
        void Timer1Service() { handle_interrupts(_timer1, &TCNT1, &OCR1A); }
      #endif
      #if ENABLED(_useTimer3)
        void Timer3Service() { handle_interrupts(_timer3, &TCNT3, &OCR3A); }
      #endif
    
    #endif // !WIRING


    static void initISR(timer16_Sequence_t timer) {
      #if ENABLED(_useTimer1)
        if (timer == _timer1) {
          TCCR1A = 0;             // normal counting mode
          TCCR1B = _BV(CS11);     // set prescaler of 8
          TCNT1 = 0;              // clear the timer count
        #if ENABLED(__AVR_ATmega8__) || ENABLED(__AVR_ATmega128__)
          SBI(TIFR, OCF1A);      // clear any pending interrupts;
          SBI(TIMSK, OCIE1A);    // enable the output compare interrupt
        #else
          // here if not ATmega8 or ATmega128
          SBI(TIFR1, OCF1A);     // clear any pending interrupts;
          SBI(TIMSK1, OCIE1A);   // enable the output compare interrupt
        #endif
        #if ENABLED(WIRING)
          timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service);
        #endif
        }
      #endif

      #if ENABLED(_useTimer3)
        if (timer == _timer3) {
          TCCR3A = 0;             // normal counting mode
          TCCR3B = _BV(CS31);     // set prescaler of 8
          TCNT3 = 0;              // clear the timer count
          #ifdef __AVR_ATmega128__
            SBI(TIFR, OCF3A);     // clear any pending interrupts;
            SBI(ETIMSK, OCIE3A);  // enable the output compare interrupt
          #else
            SBI(TIFR3, OCF3A);   // clear any pending interrupts;
            SBI(TIMSK3, OCIE3A); // enable the output compare interrupt
          #endif
          #ifdef WIRING
            timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service);  // for Wiring platform only
          #endif
        }
      #endif

      #if ENABLED(_useTimer4)
        if (timer == _timer4) {
          TCCR4A = 0;             // normal counting mode
          TCCR4B = _BV(CS41);     // set prescaler of 8
          TCNT4 = 0;              // clear the timer count
          TIFR4 = _BV(OCF4A);     // clear any pending interrupts;
          TIMSK4 = _BV(OCIE4A);   // enable the output compare interrupt
        }
      #endif

      #if ENABLED(_useTimer5)
        if (timer == _timer5) {
          TCCR5A = 0;             // normal counting mode
          TCCR5B = _BV(CS51);     // set prescaler of 8
          TCNT5 = 0;              // clear the timer count
          TIFR5 = _BV(OCF5A);     // clear any pending interrupts;
          TIMSK5 = _BV(OCIE5A);   // enable the output compare interrupt
        }
      #endif
    }

    static void finISR(timer16_Sequence_t timer) {
      //disable use of the given timer
      #ifdef WIRING
        if (timer == _timer1) {
          #if ENABLED(__AVR_ATmega1281__) || ENABLED(__AVR_ATmega2561__)
            TIMSK1 &= ~_BV(OCIE1A);  // disable timer 1 output compare interrupt
          #else
            TIMSK  &= ~_BV(OCIE1A);  // disable timer 1 output compare interrupt
          #endif
          timerDetach(TIMER1OUTCOMPAREA_INT);
        }
        else if (timer == _timer3) {
          #if ENABLED(__AVR_ATmega1281__) || ENABLED(__AVR_ATmega2561__)
            TIMSK3 &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
          #else
            ETIMSK &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
          #endif
          timerDetach(TIMER3OUTCOMPAREA_INT);
        }
      #else
        //For arduino - in future: call here to a currently undefined function to reset the timer
      #endif
    }

  #endif

  static bool isTimerActive(timer16_Sequence_t timer) {
    // returns true if any servo is active on this timer
    for (uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++) {
      if (SERVO(timer, channel).Pin.isActive == true)
        return true;
    }
    return false;
  }

  /****************** end of static functions ******************************/

  Servo::Servo() {
    if (ServoCount < MAX_SERVOS) {
      this->servoIndex = ServoCount++;  // assign a servo index to this instance
      servo_info[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);  // store default values
    }
    else {
      this->servoIndex = INVALID_SERVO;  // too many servos
    }
  }

  int8_t Servo::attach(int pin) {
    return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  }

  int8_t Servo::attach(int pin, int min, int max) {

    if (this->servoIndex >= MAX_SERVOS) return -1;

    if (pin > 0) servo_info[this->servoIndex].Pin.nbr = pin;
    HAL::pinMode(servo_info[this->servoIndex].Pin.nbr, OUTPUT); // set servo pin to output

    // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128
    this->min = (MIN_PULSE_WIDTH - min) / 4; //resolution of min/max is 4 uS
    this->max = (MAX_PULSE_WIDTH - max) / 4;

    // initialize the timer if it has not already been initialized
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if (!isTimerActive(timer)) initISR(timer);
    servo_info[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive

    return this->servoIndex;
  }

  void Servo::detach() {
    servo_info[this->servoIndex].Pin.isActive = false;
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if (!isTimerActive(timer)) finISR(timer);
  }

  void Servo::write(int value) {
    if (value < MIN_PULSE_WIDTH) { // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
      value = map(constrain(value, 0, 180), 0, 180, SERVO_MIN(), SERVO_MAX());
    }
    this->writeMicroseconds(value);
  }

  void Servo::writeMicroseconds(int value) {
    // calculate and store the values for the given channel
    byte channel = this->servoIndex;
    if (channel < MAX_SERVOS) {  // ensure channel is valid
      // ensure pulse width is valid
      value = constrain(value, SERVO_MIN(), SERVO_MAX()) - TRIM_DURATION;
      value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead

      CRITICAL_SECTION_START;
      servo_info[channel].ticks = value;
      CRITICAL_SECTION_END;
    }
  }

  // return the value as degrees
  int Servo::read() { return map(this->readMicroseconds() + 1, SERVO_MIN(), SERVO_MAX(), 0, 180); }

  int Servo::readMicroseconds() {
    return (this->servoIndex == INVALID_SERVO) ? 0 : ticksToUs(servo_info[this->servoIndex].ticks) + TRIM_DURATION;
  }

  bool Servo::attached() { return servo_info[this->servoIndex].Pin.isActive; }

  void Servo::move(int value) {
    if (this->attach(0) >= 0) {
      this->write(value);
      HAL::delayMilliseconds(SERVO_DEACTIVATION_DELAY);
      #if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE)
        this->detach();
      #endif
    }
  }

#endif

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

#ifdef ARDUINO_ARCH_SAM

#include "../../../MK4duo.h"

#if HAS_SERVOS

static volatile int8_t Channel[_Nbr_16timers ]; // counter for the servo being pulsed for each timer (or -1 if refresh interval)

//------------------------------------------------------------------------------
/// Interrupt handler for the TC0 channel 1.
//------------------------------------------------------------------------------
void Servo_Handler(timer16_Sequence_t timer, Tc *pTc, uint8_t channel);

#if ENABLED (_useTimer1)
  void HANDLER_FOR_TIMER1() {
    Servo_Handler(_timer1, TC_FOR_TIMER1, CHANNEL_FOR_TIMER1);
  }
#endif
#if ENABLED (_useTimer2)
  void HANDLER_FOR_TIMER2() {
    Servo_Handler(_timer2, TC_FOR_TIMER2, CHANNEL_FOR_TIMER2);
  }
#endif
#if ENABLED (_useTimer3)
  void HANDLER_FOR_TIMER3() {
    Servo_Handler(_timer3, TC_FOR_TIMER3, CHANNEL_FOR_TIMER3);
  }
#endif
#if ENABLED (_useTimer4)
  void HANDLER_FOR_TIMER4() {
    Servo_Handler(_timer4, TC_FOR_TIMER4, CHANNEL_FOR_TIMER4);
  }
#endif
#if ENABLED (_useTimer5)
  void HANDLER_FOR_TIMER5() {
    Servo_Handler(_timer5, TC_FOR_TIMER5, CHANNEL_FOR_TIMER5);
  }
#endif

void Servo_Handler(timer16_Sequence_t timer, Tc *tc, uint8_t channel) {
  // clear interrupt
  tc->TC_CHANNEL[channel].TC_SR;
  if (Channel[timer] < 0)
    tc->TC_CHANNEL[channel].TC_CCR |= TC_CCR_SWTRG; // channel set to -1 indicated that refresh interval completed so reset the timer
  else if (SERVO_INDEX(timer, Channel[timer]) < ServoCount && SERVO(timer, Channel[timer]).Pin.isActive == true)
    digitalWrite(SERVO(timer, Channel[timer]).Pin.nbr, LOW);  // pulse this channel low if activated

  Channel[timer]++;    // increment to the next channel
  if (SERVO_INDEX(timer,Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
    tc->TC_CHANNEL[channel].TC_RA = tc->TC_CHANNEL[channel].TC_CV + SERVO(timer, Channel[timer]).ticks;
    if (SERVO(timer, Channel[timer]).Pin.isActive == true)      // check if activated
      digitalWrite(SERVO(timer, Channel[timer]).Pin.nbr,HIGH);  // its an active channel so pulse it high
  }
  else {
    // finished all channels so wait for the refresh period to expire before starting over
    if ((tc->TC_CHANNEL[channel].TC_CV) + 4 < usToTicks(REFRESH_INTERVAL))  // allow a few ticks to ensure the next OCR1A not missed
      tc->TC_CHANNEL[channel].TC_RA = (unsigned int)usToTicks(REFRESH_INTERVAL);
    else
      tc->TC_CHANNEL[channel].TC_RA = tc->TC_CHANNEL[channel].TC_CV + 4;  // at least REFRESH_INTERVAL has elapsed

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

void initISR(timer16_Sequence_t timer) {
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

void finISR(timer16_Sequence_t timer) {
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

#endif // HAS_SERVOS

#endif // ARDUINO_ARCH_SAM

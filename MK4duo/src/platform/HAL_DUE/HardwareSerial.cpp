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
 * HardwareSerial.cpp - Hardware serial library for Arduino DUE
 * Copyright (c) 2015 MagoKimbra. All right reserved.
 *
 * Modified 14 February   2016 by Andreas Hardtung (added tx buffer)
 * Modified 01 October    2017 by Eduardo José Tagle (added XON/XOFF)
 */

#ifdef ARDUINO_ARCH_SAM

#include "../../../MK4duo.h"

template<int portNr>
  typename MKHardwareSerial<portNr>::ring_buffer_r
    MKHardwareSerial<portNr>::rx_buffer = { { 0 }, 0, 0 };

template<int portNr>
  typename MKHardwareSerial<portNr>::ring_buffer_t
  MKHardwareSerial<portNr>::tx_buffer = { 0 };

template<int portNr>
  bool MKHardwareSerial<portNr>::_written = false;

template<int portNr>
  uint8_t MKHardwareSerial<portNr>::xon_xoff_state = MKHardwareSerial<portNr>::XON_XOFF_CHAR_SENT | MKHardwareSerial<portNr>::XON_CHAR;

template<int portNr>
  uint8_t MKHardwareSerial<portNr>::rx_dropped_bytes = 0;

template<int portNr>
  uint8_t MKHardwareSerial<portNr>::rx_buffer_overruns = 0;

template<int portNr>
  uint8_t MKHardwareSerial<portNr>::rx_framing_errors = 0;

template<int portNr>
  uint16_t MKHardwareSerial<portNr>::rx_max_enqueued = 0;

#define sw_barrier() asm volatile("": : :"memory");

template<int portNr>
  void MKHardwareSerial<portNr>::store_rxd_char() {

    #if ENABLED(EMERGENCY_PARSER)
      static EmergencyStateEnum emergency_state; // = EP_RESET
    #endif

    // Get the tail pointer - Nothing can alter its value while we are at this ISR
    const uint16_t t = rx_buffer.tail;

    // Get the head pointer - This ISR is the only one that modifies its value, so it's safe to read here
    uint16_t h = rx_buffer.head;

    // Get the next element
    uint16_t i = (h + 1) & (RX_BUFFER_SIZE - 1);

    // Read the character from the USART
    uint8_t c = _pUart->UART_RHR;

    #if ENABLED(EMERGENCY_PARSER)
      emergency_parser.update(emergency_state, c);
    #endif

    // If the character is to be stored at the index just before the tail
    // (such that the head would advance to the current tail), the RX FIFO is
    // full, so don't write the character or advance the head.
    if (i != t) {
      rx_buffer.buffer[h] = c;
      h = i;
    }
    #if ENABLED(SERIAL_STATS_DROPPED_RX)
      else if (!++rx_dropped_bytes) --rx_dropped_bytes;
    #endif

    const uint16_t rx_count = (h - t) & (RX_BUFFER_SIZE - 1);
    // Calculate count of bytes stored into the RX buffer

    #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
      // Keep track of the maximum count of enqueued bytes
      NOLESS(rx_max_enqueued, rx_count);
    #endif

    #if ENABLED(SERIAL_XON_XOFF)
      // If the last char that was sent was an XON
      if ((xon_xoff_state & XON_XOFF_CHAR_MASK) == XON_CHAR) {

        // Bytes stored into the RX buffer
        const uint16_t rx_count = (h - t) & (RX_BUFFER_SIZE - 1);

        // If over 12.5% of RX buffer capacity, send XOFF before running out of
        // RX buffer space .. 325 bytes @ 250kbits/s needed to let the host react
        // and stop sending bytes. This translates to 13mS propagation time.
        if (rx_count >= (RX_BUFFER_SIZE) / 8) {

          // At this point, definitely no TX interrupt was executing, since the TX ISR can't be preempted.
          // Don't enable the TX interrupt here as a means to trigger the XOFF char, because if it happens
          // to be in the middle of trying to disable the RX interrupt in the main program, eventually the
          // enabling of the TX interrupt could be undone. The ONLY reliable thing this can do to ensure
          // the sending of the XOFF char is to send it HERE AND NOW.

          // About to send the XOFF char
          xon_xoff_state = XOFF_CHAR | XON_XOFF_CHAR_SENT;

          // Wait until the TX register becomes empty and send it - Here there could be a problem
          // - While waiting for the TX register to empty, the RX register could receive a new
          //   character. This must also handle that situation!
          uint32_t status;
          while (!((status = _pUart->UART_SR) & UART_SR_TXRDY)) {

            if (status & UART_SR_RXRDY) {
              // A char arrived while waiting for the TX buffer to be empty - Receive and process it!

              i = (h + 1) & (RX_BUFFER_SIZE - 1);

              // Read the character from the USART
              c = _pUart->UART_RHR;

              #if ENABLED(EMERGENCY_PARSER)
                emergency_parser.update(emergency_state, c);
              #endif

              // If the character is to be stored at the index just before the tail
              // (such that the head would advance to the current tail), the FIFO is
              // full, so don't write the character or advance the head.
              if (i != t) {
                rx_buffer.buffer[h] = c;
                h = i;
              }
              #if ENABLED(SERIAL_STATS_DROPPED_RX)
                else if (!++rx_dropped_bytes) --rx_dropped_bytes;
              #endif
            }
            sw_barrier();
          }

          _pUart->UART_THR = XOFF_CHAR;

          // At this point there could be a race condition between the write() function
          // and this sending of the XOFF char. This interrupt could happen between the
          // wait to be empty TX buffer loop and the actual write of the character. Since
          // the TX buffer is full because it's sending the XOFF char, the only way to be
          // sure the write() function will succeed is to wait for the XOFF char to be
          // completely sent. Since an extra character could be received during the wait
          // it must also be handled!
          while (!((status = _pUart->UART_SR) & UART_SR_TXRDY)) {

            if (status & UART_SR_RXRDY) {
              // A char arrived while waiting for the TX buffer to be empty - Receive and process it!

              i = (h + 1) & (RX_BUFFER_SIZE - 1);

              // Read the character from the USART
              c = _pUart->UART_RHR;

              #if ENABLED(EMERGENCY_PARSER)
                emergency_parser.update(emergency_state, c);
              #endif

              // If the character is to be stored at the index just before the tail
              // (such that the head would advance to the current tail), the FIFO is
              // full, so don't write the character or advance the head.
              if (i != t) {
                rx_buffer.buffer[h] = c;
                h = i;
              }
              #if ENABLED(SERIAL_STATS_DROPPED_RX)
                else if (!++rx_dropped_bytes) --rx_dropped_bytes;
              #endif
            }
            sw_barrier();
          }

          // At this point everything is ready. The write() function won't
          // have any issues writing to the UART TX register if it needs to!
        }
      }
    #endif // SERIAL_XON_XOFF

    // Store the new head value - The main loop will retry until the value is stable
    rx_buffer.head = h;
  }

template<int portNr>
  void MKHardwareSerial<portNr>::_tx_thr_empty_irq(void) {

    #if TX_BUFFER_SIZE > 0

      // Read positions
      uint8_t t = tx_buffer.tail;
      const uint8_t h = tx_buffer.head;
    
      #if ENABLED(SERIAL_XON_XOFF)

        // If an XON char is pending to be sent, do it now
        if (xon_xoff_state == XON_CHAR) {
    
          // Send the character
          _pUart->UART_THR = XON_CHAR;
    
          // Remember we sent it.
          xon_xoff_state = XON_CHAR | XON_XOFF_CHAR_SENT;
    
          // If nothing else to transmit, just disable TX interrupts.
          if (h == t) _pUart->UART_IDR = UART_IDR_TXRDY;
    
          return;
        }

      #endif

      // If nothing to transmit, just disable TX interrupts. This could
      // happen as the result of the non atomicity of the disabling of RX
      // interrupts that could end reenabling TX interrupts as a side effect.
      if (h == t) {
        _pUart->UART_IDR = UART_IDR_TXRDY;
        return;
      }

      // There is something to TX, Send the next byte
      const uint8_t c = tx_buffer.buffer[t];
      t = (t + 1) & (TX_BUFFER_SIZE - 1);
      _pUart->UART_THR = c;
      tx_buffer.tail = t;

      // Disable interrupts if there is nothing to transmit following this byte
      if (h == t) _pUart->UART_IDR = UART_IDR_TXRDY;

    #endif // TX_BUFFER_SIZE > 0
  }

template<int portNr>
  void MKHardwareSerial<portNr>::UART_ISR(void) {

    const uint32_t status = _pUart->UART_SR;

    // Data received?
    if (status & UART_SR_RXRDY) store_rxd_char();

    #if TX_BUFFER_SIZE > 0
      // Something to send, and TX interrupts are enabled (meaning something to send)?
      if ((status & UART_SR_TXRDY) && (_pUart->UART_IMR & UART_IMR_TXRDY)) _tx_thr_empty_irq();
    #endif

    // Acknowledge errors
    if ((status & UART_SR_OVRE) || (status & UART_SR_FRAME)) {
      #if ENABLED(SERIAL_STATS_DROPPED_RX)
        if (status & UART_SR_OVRE && !++rx_dropped_bytes) --rx_dropped_bytes;
      #endif
      #if ENABLED(SERIAL_STATS_RX_BUFFER_OVERRUNS)
        if (status & UART_SR_OVRE && !++rx_buffer_overruns) --rx_buffer_overruns;
      #endif
      // TODO: error reporting outside ISR
      _pUart->UART_CR = UART_CR_RSTSTA;
    }
  }

/** Public Function */
template<int portNr>
  void MKHardwareSerial<portNr>::begin(const long baud) {

    // Disable UART interrupt in NVIC
    NVIC_DisableIRQ( _dwIrq );

    // We NEED memory barriers to ensure Interrupts are actually disabled!
    // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
    __DSB();
    __ISB();

    // Disable clock
    pmc_disable_periph_clk( _dwId );

    // Configure PMC
    pmc_enable_periph_clk( _dwId );

    // Disable PDC channel
    _pUart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

    // Reset and disable receiver and transmitter
    _pUart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

    // Configure mode: 8bit, No parity, 1 bit stop
    _pUart->UART_MR = UART_MR_CHMODE_NORMAL | US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO;

    // Configure baudrate (asynchronous, no oversampling)
    _pUart->UART_BRGR = (SystemCoreClock / (baud << 4));

    // Configure interrupts
    _pUart->UART_IDR = 0xFFFFFFFF;
    _pUart->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;

    // Install interrupt handler
    install_isr(_dwIrq, UART_ISR);

    // Configure priority. We need a very high priority to avoid losing characters
    // and we need to be able to preempt the Stepper ISR and everything else!
    // (this could probably be fixed by using DMA with the Serial port)
    NVIC_SetPriority(_dwIrq, 1);

    // Enable UART interrupt in NVIC
    NVIC_EnableIRQ(_dwIrq);

    // Enable receiver and transmitter
    _pUart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

    #if TX_BUFFER_SIZE > 0
      _written = false;
    #endif
  }

template<int portNr>
  void MKHardwareSerial<portNr>::end() {
    // Disable UART interrupt in NVIC
    NVIC_DisableIRQ( _dwIrq );

    // We NEED memory barriers to ensure Interrupts are actually disabled!
    // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
    __DSB();
    __ISB();

    pmc_disable_periph_clk( _dwId );
  }

template<int portNr>
  int MKHardwareSerial<portNr>::peek(void) {
    const int v = rx_buffer.head == rx_buffer.tail ? -1 : rx_buffer.buffer[rx_buffer.tail];
    return v;
  }

template<int portNr>
  int MKHardwareSerial<portNr>::read(void) {

    const uint16_t h = rx_buffer.head;
    uint16_t t = rx_buffer.tail;

    if (h == t) return -1;

    int v = rx_buffer.buffer[t];
    t = (t + 1) & (RX_BUFFER_SIZE - 1);

    // Advance tail
    rx_buffer.tail = t;

    #if ENABLED(SERIAL_XON_XOFF)

      // If the XOFF char was sent, or about to be sent...
      if ((xon_xoff_state & XON_XOFF_CHAR_MASK) == XOFF_CHAR) {
        // Get count of bytes in the RX buffer
        const uint16_t rx_count = (h - t) & (RX_BUFFER_SIZE - 1);
        // When below 10% of RX buffer capacity, send XON before running out of RX buffer bytes
        if (rx_count < (RX_BUFFER_SIZE) / 10) {
          #if TX_BUFFER_SIZE > 0
            // Signal we want an XON character to be sent.
            xon_xoff_state = XON_CHAR;
            // Enable TX isr.
            _pUart->UART_IER = UART_IER_TXRDY;
          #else
            // If not using TX interrupts, we must send the XON char now
            xon_xoff_state = XON_CHAR | XON_XOFF_CHAR_SENT;
            while (!(_pUart->UART_SR & UART_SR_TXRDY)) sw_barrier();
            _pUart->UART_THR = XON_CHAR;
          #endif
        }
      }

    #endif

    return v;
  }

template<int portNr>
  uint16_t MKHardwareSerial<portNr>::available(void) {
      const uint16_t h = rx_buffer.head, t = rx_buffer.tail;
      return (RX_BUFFER_SIZE + h - t) & (RX_BUFFER_SIZE - 1);
    }

template<int portNr>
  void MKHardwareSerial<portNr>::flush(void) {
    rx_buffer.tail = rx_buffer.head;

    #if ENABLED(SERIAL_XON_XOFF)

      if ((xon_xoff_state & XON_XOFF_CHAR_MASK) == XOFF_CHAR) {
        #if TX_BUFFER_SIZE > 0
          // Signal we want an XON character to be sent.
          xon_xoff_state = XON_CHAR;
          // Enable TX isr.
          _pUart->UART_IER = UART_IER_TXRDY;
        #else
          // If not using TX interrupts, we must send the XON char now
          xon_xoff_state = XON_CHAR | XON_XOFF_CHAR_SENT;
          while (!(_pUart->UART_SR & UART_SR_TXRDY)) sw_barrier();
          _pUart->UART_THR = XON_CHAR;
        #endif
      }

    #endif
  }

template<int portNr>
  void MKHardwareSerial<portNr>::write(const uint8_t c) {

    _written = true;

    #if TX_BUFFER_SIZE == 0

      while (!(_pUart->UART_SR & UART_SR_TXRDY)) sw_barrier();
      _pUart->UART_THR = c;

    #else

      // If the TX interrupts are disabled and the data register
      // is empty, just write the byte to the data register and
      // be done. This shortcut helps significantly improve the
      // effective datarate at high (>500kbit/s) bitrates, where
      // interrupt overhead becomes a slowdown.
      // Yes, there is a race condition between the sending of the
      // XOFF char at the RX isr, but it is properly handled there
      if (!(_pUart->UART_IMR & UART_IMR_TXRDY) && (_pUart->UART_SR & UART_SR_TXRDY)) {
        _pUart->UART_THR = c;
        return;
      }
    
      const uint8_t i = (tx_buffer.head + 1) & (TX_BUFFER_SIZE - 1);
    
      // If global interrupts are disabled (as the result of being called from an ISR)...
      if (!ISRS_ENABLED()) {
    
        // Make room by polling if it is possible to transmit, and do so!
        while (i == tx_buffer.tail) {
          // If we can transmit another byte, do it.
          if (_pUart->UART_SR & UART_SR_TXRDY) _tx_thr_empty_irq();
          // Make sure compiler rereads tx_buffer.tail
          sw_barrier();
        }
      }
      else {
        // Interrupts are enabled, just wait until there is space
        while (i == tx_buffer.tail) sw_barrier();
      }
    
      // Store new char. head is always safe to move
      tx_buffer.buffer[tx_buffer.head] = c;
      tx_buffer.head = i;
    
      // Enable TX isr - Non atomic, but it will eventually enable TX isr
      _pUart->UART_IER = UART_IER_TXRDY;

    #endif

  }

template<int portNr>
  void MKHardwareSerial<portNr>::flushTX(void) {

    // TX
    #if TX_BUFFER_SIZE == 0

      // No bytes written, no need to flush. This special case is needed since there's
      // no way to force the TXC (transmit complete) bit to 1 during initialization.
      if (!_written) return;

      // Wait until everything was transmitted
      while (!(_pUart->UART_SR & UART_SR_TXEMPTY)) sw_barrier();

    #else

      // If we have never written a byte, no need to flush. This special
      // case is needed since there is no way to force the TXC (transmit
      // complete) bit to 1 during initialization
      if (!_written) return;
    
      // If global interrupts are disabled (as the result of being called from an ISR)...
      if (!ISRS_ENABLED()) {
    
        // Wait until everything was transmitted - We must do polling, as interrupts are disabled
        while (tx_buffer.head != tx_buffer.tail || !(_pUart->UART_SR & UART_SR_TXEMPTY)) {
          // If there is more space, send an extra character
          if (_pUart->UART_SR & UART_SR_TXRDY) _tx_thr_empty_irq();
          sw_barrier();
        }
    
      }
      else {
        // Wait until everything was transmitted
        while (tx_buffer.head != tx_buffer.tail || !(_pUart->UART_SR & UART_SR_TXEMPTY)) sw_barrier();
      }

    #endif
  }

template class MKHardwareSerial<0>;
template class MKHardwareSerial<1>;
template class MKHardwareSerial<2>;
template class MKHardwareSerial<3>;

MKHardwareSerial<0> MKSerial;
MKHardwareSerial<1> MKSerial1;
MKHardwareSerial<2> MKSerial2;
MKHardwareSerial<3> MKSerial3;

#endif // ARDUINO_ARCH_SAM

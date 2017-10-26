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
 * HardwareSerial.cpp - Hardware serial library for Arduino DUE
 * Copyright (c) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(ARDUINO_ARCH_SAM)

  #include "HardwareSerial_Due.h"

  MK_RingBuffer::MK_RingBuffer(void) {
    memset((void *)_aucBuffer, 0, SERIAL_BUFFER_SIZE);
    _iHead = 0;
    _iTail = 0;
  }

  #if ENABLED(EMERGENCY_PARSER)

    enum e_parser_state {
      state_RESET,
      state_N,
      state_M,
      state_M1,
      state_M10,
      state_M108,
      state_M11,
      state_M112,
      state_M4,
      state_M41,
      state_M410,
      state_IGNORE // to '\n'
    };

    // Currently looking for: M108, M112, M410
    // If you alter the parser please don't forget to update the capabilities in Conditionals_post.h

    FORCE_INLINE void emergency_parser(const uint8_t c) {

      static e_parser_state state = state_RESET;

      switch (state) {
        case state_RESET:
          switch (c) {
            case ' ': break;
            case 'N': state = state_N;      break;
            case 'M': state = state_M;      break;
            default: state = state_IGNORE;
          }
        break;

        case state_N:
          switch (c) {
            case '0': case '1': case '2':
            case '3': case '4': case '5':
            case '6': case '7': case '8':
            case '9': case '-': case ' ':   break;
            case 'M': state = state_M;      break;
            default:  state = state_IGNORE;
          }
        break;

        case state_M:
          switch (c) {
            case ' ': break;
            case '1': state = state_M1;     break;
            case '4': state = state_M4;     break;
            default: state = state_IGNORE;
          }
        break;

        case state_M1:
          switch (c) {
            case '0': state = state_M10;    break;
            case '1': state = state_M11;    break;
            default: state = state_IGNORE;
          }
        break;

        case state_M10:
          state = (c == '8') ? state_M108 : state_IGNORE;
        break;

        case state_M11:
          state = (c == '2') ? state_M112 : state_IGNORE;
        break;

        case state_M4:
          state = (c == '1') ? state_M41 : state_IGNORE;
        break;

        case state_M41:
          state = (c == '0') ? state_M410 : state_IGNORE;
        break;

        case state_IGNORE:
          if (c == '\n') state = state_RESET;
          break;

        default:
          if (c == '\n') {
            switch (state) {
              case state_M108:
                printer.wait_for_user = thermalManager.wait_for_heatup = false;
              break;
              case state_M112:
                printer.kill(PSTR(MSG_KILLED));
              break;
              case state_M410:
                stepper.quickstop_stepper();
              break;
              default:
              break;
            }
            state = state_RESET;
          }
      }
    }

  #endif // EMERGENCY_PARSER

  void MK_RingBuffer::store_char(const uint8_t c) {

    int i = (uint32_t)(_iHead + 1) % SERIAL_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != _iTail) {
      _aucBuffer[_iHead] = c;
      _iHead = i;
    }

    #if ENABLED(EMERGENCY_PARSER)
      emergency_parser(c);
    #endif
  }

  // The relocated Exception/Interrupt Table - Must be aligned to 128bytes,
  // as bits 0-6 on VTOR register are reserved and must be set to 0
  __attribute__ ((aligned(128)))
  static DeviceVectors ram_tab = { NULL };

  static pfnISR_Handler* get_relocated_table_addr(void) {
    // Get the address of the interrupt/exception table
    uint32_t isrtab = SCB->VTOR;

    // If already relocated, we are done!
    if (isrtab >= IRAM0_ADDR)
      return (pfnISR_Handler*)isrtab;

    // Get the address of the table stored in FLASH
    const pfnISR_Handler*	romtab = (const pfnISR_Handler*)isrtab;

    // Copy it to SRAM
    memcpy(&ram_tab, romtab, sizeof(ram_tab));

    CRITICAL_SECTION_START

      // Set the vector table base address to the SRAM copy
      SCB->VTOR = (uint32_t)(&ram_tab);

    CRITICAL_SECTION_END

    return (pfnISR_Handler*)(&ram_tab);
  }

  pfnISR_Handler install_isr(IRQn_Type irq, pfnISR_Handler newHandler) {

    // Get the address of the relocated table
    pfnISR_Handler* isrtab = get_relocated_table_addr();

    CRITICAL_SECTION_START

      // Get the original handler
      pfnISR_Handler oldHandler = isrtab[irq + 16];

      // Install the new one
      isrtab[irq + 16] = newHandler;

    CRITICAL_SECTION_END

    return oldHandler;
  }

  /** 
   * MKUARTCLASS
   */

  // Constructors
  MKUARTClass::MKUARTClass(Uart *pUart, IRQn_Type dwIrq, uint32_t dwId, MK_RingBuffer *pRx_buffer, MK_RingBuffer *pTx_buffer) {
    _rx_buffer = pRx_buffer;
    _tx_buffer = pTx_buffer;

    _pUart  = pUart;
    _dwIrq  = dwIrq;
    _dwId   = dwId;
  }

  static void MK_UART_ISR(void) {
    MKSerial.IrqHandler();
  }

  // Public Methods
  void MKUARTClass::begin(const uint32_t dwBaudRate) {
    begin(dwBaudRate, Mode_8N1);
  }

  void MKUARTClass::begin(const uint32_t dwBaudRate, const UARTModes config) {
    uint32_t modeReg = static_cast<uint32_t>(config) & 0x00000E00;
    init(dwBaudRate, modeReg | UART_MR_CHMODE_NORMAL);
  }

  void MKUARTClass::init(const uint32_t dwBaudRate, const uint32_t modeReg) {

    // Disable UART interrupt in NVIC
    NVIC_DisableIRQ(_dwIrq);

    // Configure PMC
    pmc_enable_periph_clk(_dwId);

    // Disable PDC channel
    _pUart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

    // Reset and disable receiver and transmitter
    _pUart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

    // Configure mode
    _pUart->UART_MR = modeReg;

    // Configure baudrate (asynchronous, no oversampling)
    _pUart->UART_BRGR = (SystemCoreClock / dwBaudRate) >> 4;

    // Configure interrupts
    _pUart->UART_IDR = 0xFFFFFFFF;
    _pUart->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;

    // Install interrupt handler
    install_isr(_dwIrq, MK_UART_ISR);

    // Enable UART interrupt in NVIC
    NVIC_EnableIRQ(_dwIrq);

    // Enable receiver and transmitter
    _pUart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

  }

  void MKUARTClass::end(void) {

    // Clear any received data
    _rx_buffer->_iHead = _rx_buffer->_iTail;

    // Wait for any outstanding data to be sent
    flush();

    // Disable UART interrupt in NVIC
    NVIC_DisableIRQ(_dwIrq);

    pmc_disable_periph_clk(_dwId);
  }

  void MKUARTClass::flush(void) {
    while (_tx_buffer->_iHead != _tx_buffer->_iTail); // wait for transmit data to be sent
    // Wait for transmission to complete
    while ((_pUart->UART_SR & UART_SR_TXEMPTY) != UART_SR_TXEMPTY);
  }

  void MKUARTClass::checkRx(void) {
    if ((_pUart->UART_SR & UART_SR_RXRDY) == UART_SR_RXRDY) {
      CRITICAL_SECTION_START
        _rx_buffer->store_char(_pUart->UART_RHR);
      CRITICAL_SECTION_END
    }
  }

  void MKUARTClass::write(const uint8_t uc_data) {

    // Is the hardware currently busy?
    if (((_pUart->UART_SR & UART_SR_TXRDY) != UART_SR_TXRDY) |
        (_tx_buffer->_iTail != _tx_buffer->_iHead)) {

      // If busy we buffer
      int nextWrite = (_tx_buffer->_iHead + 1) % SERIAL_BUFFER_SIZE;
      while (_tx_buffer->_iTail == nextWrite); // Spin locks if we're about to overwrite the buffer. This continues once the data is sent

      _tx_buffer->_aucBuffer[_tx_buffer->_iHead] = uc_data;
      _tx_buffer->_iHead = nextWrite;
      // Make sure TX interrupt is enabled
      _pUart->UART_IER = UART_IER_TXRDY;
    }
    else {
      // Bypass buffering and send character directly
      _pUart->UART_THR = uc_data;
    }
  }

  int MKUARTClass::peek(void) {
    CRITICAL_SECTION_START
      const int v = _rx_buffer->_iHead == _rx_buffer->_iTail ? -1 : _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
    CRITICAL_SECTION_END
    return v;
  }

  int MKUARTClass::read(void) {
    int v;
    CRITICAL_SECTION_START
      if (_rx_buffer->_iHead == _rx_buffer->_iTail)
        v = -1;
      else {
        v = _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
        _rx_buffer->_iTail = (unsigned int)(_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
      }
    CRITICAL_SECTION_END
    return v;
  }

  int MKUARTClass::available(void) {
    CRITICAL_SECTION_START
      const uint8_t head = _rx_buffer->_iHead, tail = _rx_buffer->_iTail;
    CRITICAL_SECTION_END
    return (uint32_t)(SERIAL_BUFFER_SIZE + head - tail) % SERIAL_BUFFER_SIZE;
  }

  int MKUARTClass::availableForWrite(void) {
    CRITICAL_SECTION_START
      const uint8_t head = _tx_buffer->_iHead, tail = _tx_buffer->_iTail;
    CRITICAL_SECTION_END

    if (head >= tail)
      return SERIAL_BUFFER_SIZE - 1 - head + tail;
    else
      return tail - head - 1;
  }

  void MKUARTClass::IrqHandler(void) {

    uint32_t status = _pUart->UART_SR;

    if ((status & UART_SR_RXRDY) == UART_SR_RXRDY)
      _rx_buffer->store_char(_pUart->UART_RHR);

    if ((status & UART_SR_TXRDY) == UART_SR_TXRDY) {
      if (_tx_buffer->_iTail != _tx_buffer->_iHead) {
        _pUart->UART_THR = _tx_buffer->_aucBuffer[_tx_buffer->_iTail];
        _tx_buffer->_iTail = (unsigned int)(_tx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
      }
      else {
        // Mask off transmit interrupt so we don't get it anymore
        _pUart->UART_IDR = UART_IDR_TXRDY;
      }
    }

    // Acknowledge errors
    if ((status & UART_SR_OVRE) == UART_SR_OVRE || (status & UART_SR_FRAME) == UART_SR_FRAME) {
      // TODO: error reporting outside ISR
      _pUart->UART_CR |= UART_CR_RSTSTA;
    }
  }

  void MKUARTClass::print(char c, int base) {
    print((long)c, base);
  }

  void MKUARTClass::print(unsigned char b, int base) {
    print((unsigned long)b, base);
  }

  void MKUARTClass::print(int n, int base) {
    print((long)n, base);
  }

  void MKUARTClass::print(unsigned int n, int base) {
    print((unsigned long)n, base);
  }

  void MKUARTClass::print(long n, int base) {
    if (base == 0)
      write(n);
    else if (base == 10) {
      if (n < 0) {
        print('-');
        n = -n;
      }
      printNumber(n, 10);
    }
    else
      printNumber(n, base);
  }

  void MKUARTClass::print(unsigned long n, int base) {
    if (base == 0) write(n);
    else printNumber(n, base);
  }

  void MKUARTClass::print(double n, int digits) {
    printFloat(n, digits);
  }

  void MKUARTClass::println(void) {
    print('\r');
    print('\n');
  }

  void MKUARTClass::println(const String& s) {
    print(s);
    println();
  }

  void MKUARTClass::println(const char c[]) {
    print(c);
    println();
  }

  void MKUARTClass::println(char c, int base) {
    print(c, base);
    println();
  }

  void MKUARTClass::println(unsigned char b, int base) {
    print(b, base);
    println();
  }

  void MKUARTClass::println(int n, int base) {
    print(n, base);
    println();
  }

  void MKUARTClass::println(unsigned int n, int base) {
    print(n, base);
    println();
  }

  void MKUARTClass::println(long n, int base) {
    print(n, base);
    println();
  }

  void MKUARTClass::println(unsigned long n, int base) {
    print(n, base);
    println();
  }

  void MKUARTClass::println(double n, int digits) {
    print(n, digits);
    println();
  }

  // Private Methods

  void MKUARTClass::printNumber(unsigned long n, uint8_t base) {
    if (n) {
      unsigned char buf[8 * sizeof(long)]; // Enough space for base 2
      int8_t i = 0;
      while (n) {
        buf[i++] = n % base;
        n /= base;
      }
      while (i--)
        print((char)(buf[i] + (buf[i] < 10 ? '0' : 'A' - 10)));
    }
    else
      print('0');
  }

  void MKUARTClass::printFloat(double number, uint8_t digits) {
    // Handle negative numbers
    if (number < 0.0) {
      print('-');
      number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
      rounding *= 0.1;

    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits) {
      print('.');
      // Extract digits from the remainder one at a time
      while (digits--) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        print(toPrint);
        remainder -= toPrint;
      }
    }
  }

  // Construction MKSerial
  MK_RingBuffer MK_rx_buffer;
  MK_RingBuffer MK_tx_buffer;

  // Based on selected port, use the proper configuration
  #if SERIAL_PORT == 0
    MKUARTClass MKSerial(UART, UART_IRQn, ID_UART, &MK_rx_buffer, &MK_tx_buffer);
  #endif

#endif // ARDUINO_ARCH_SAM

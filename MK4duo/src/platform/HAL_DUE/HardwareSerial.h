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

#ifndef _HARDWARESERIAL_H_
#define _HARDWARESERIAL_H_

#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 32
#endif

template <typename S, unsigned int addr> struct ApplyAddrReg {
  constexpr ApplyAddrReg(int) {}
  FORCE_INLINE S* operator->() const { return (S*)addr; }
};

template <int IDPort, int RX_SIZE = 128, int TX_SIZE = 32>
  class MKHardwareSerial {

    public: /** Constructor */

      MKHardwareSerial() {}

    private: /** Private Parameters */

      static constexpr uint32_t   ADDR_REG[]  = { 0x400E0800U,  0x40098000U,  0x4009C000U,  0x400A0000U,  0x400A4000U };
      static constexpr IRQn_Type  IRQ[]       = { UART_IRQn,    USART0_IRQn,  USART1_IRQn,  USART2_IRQn,  USART3_IRQn };
      static constexpr uint32_t   IRQ_ID[]    = { ID_UART,      ID_USART0,    ID_USART1,    ID_USART2,    ID_USART3 };

    protected: /** Protected Function */

      static constexpr ApplyAddrReg<Uart, ADDR_REG[IDPort]> _pUart = 0;
      static constexpr IRQn_Type  _dwIrq  = IRQ[IDPort];
      static constexpr uint32_t   _dwId   = IRQ_ID[IDPort];

      struct ring_buffer_r {
        unsigned char buffer[RX_SIZE];
        volatile uint16_t head, tail;
      };

      struct ring_buffer_t {
        unsigned char buffer[TX_SIZE];
        volatile uint8_t head, tail;
      };

      static ring_buffer_r rx_buffer;
      static ring_buffer_t tx_buffer;
      static bool _written;

      static constexpr uint8_t  XON_XOFF_CHAR_SENT = 0x80,  // XON / XOFF Character was sent
                                XON_XOFF_CHAR_MASK = 0x1F;  // XON / XOFF character to send

      // XON / XOFF character definitions
      static constexpr uint8_t  XON_CHAR  = 17,
                                XOFF_CHAR = 19;

      static uint8_t  xon_xoff_state,
                      rx_dropped_bytes,
                      rx_buffer_overruns,
                      rx_framing_errors;

      static uint16_t rx_max_enqueued;

      static void store_rxd_char();
      static void _tx_thr_empty_irq(void);
      
      static void UART_ISR(void);

    public: /** Public Function */

      static void begin(const long);
      static void end();
      static int peek(void);
      static int read(void);
      static void flush(void);
      static uint16_t available(void);
      static void write(const uint8_t c);
      static void flushTX(void);

      #if ENABLED(SERIAL_STATS_DROPPED_RX)
        FORCE_INLINE static uint32_t dropped() { return rx_dropped_bytes; }
      #endif

      #if ENABLED(SERIAL_STATS_RX_BUFFER_OVERRUNS)
        FORCE_INLINE static uint32_t buffer_overruns() { return rx_buffer_overruns; }
      #endif

      #if ENABLED(SERIAL_STATS_RX_FRAMING_ERRORS)
        FORCE_INLINE static uint32_t framing_errors() { return rx_framing_errors; }
      #endif

      #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
        FORCE_INLINE static uint16_t rxMaxEnqueued() { return rx_max_enqueued; }
      #endif

      FORCE_INLINE static void write(PGM_P str) { while (*str) write(*str++); }
      FORCE_INLINE static void write(const uint8_t* buffer, size_t size) { while (size--) write(*buffer++); }
      FORCE_INLINE static void print(const String& s) { for (int i = 0; i < (int)s.length(); i++) write(s[i]); }
      FORCE_INLINE static void print(PGM_P str) { write(str); }

      static void print(char, int = BYTE);
      static void print(unsigned char, int = DEC);
      static void print(int, int = DEC);
      static void print(unsigned int, int = DEC);
      static void print(long, int = DEC);
      static void print(unsigned long, int = DEC);
      static void print(double, int = 2);

      static void println(const String& s);
      static void println(const char[]);
      static void println(char, int = BYTE);
      static void println(unsigned char, int = BYTE);
      static void println(int, int = DEC);
      static void println(unsigned int, int = DEC);
      static void println(long, int = DEC);
      static void println(unsigned long, int = DEC);
      static void println(double, int = 2);
      static void println(void);
      operator bool() { return true; }

    private: /** Private Function */

      static void printNumber(unsigned long, const uint8_t);
      static void printFloat(double, uint8_t);

  };

extern MKHardwareSerial<0, RX_BUFFER_SIZE, TX_BUFFER_SIZE> MKSerial;
extern MKHardwareSerial<1, RX_BUFFER_SIZE, TX_BUFFER_SIZE> MKSerial1;
extern MKHardwareSerial<2, RX_BUFFER_SIZE, TX_BUFFER_SIZE> MKSerial2;
extern MKHardwareSerial<3, RX_BUFFER_SIZE, TX_BUFFER_SIZE> MKSerial3;

#endif /* _HARDWARESERIAL_H_ */

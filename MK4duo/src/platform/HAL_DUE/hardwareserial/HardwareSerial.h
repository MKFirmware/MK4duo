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
#pragma once

template <typename S, unsigned int addr> struct ApplyAddrReg {
  constexpr ApplyAddrReg(int) {}
  FORCE_INLINE S* operator->() const { return (S*)addr; }
};

template<typename Cfg>
class MKHardwareSerial {

  public: /** Constructor */

    MKHardwareSerial() {}

  protected: /** Protected Parameters */

    static constexpr uint32_t   ADDR_REG[]  = { 0x400E0800U,  0x40098000U,  0x4009C000U,  0x400A0000U,  0x400A4000U };
    static constexpr IRQn_Type  IRQ[]       = { UART_IRQn,    USART0_IRQn,  USART1_IRQn,  USART2_IRQn,  USART3_IRQn };
    static constexpr int        IRQ_ID[]    = { ID_UART,      ID_USART0,    ID_USART1,    ID_USART2,    ID_USART3 };

    static constexpr ApplyAddrReg<Uart,ADDR_REG[Cfg::PORT]> HWUART = 0;
    static constexpr IRQn_Type  HWUART_IRQ    = IRQ[Cfg::PORT];
    static constexpr int        HWUART_IRQ_ID = IRQ_ID[Cfg::PORT];

    // Base size of type on buffer size
    typedef typename TypeSelector<(Cfg::RX_SIZE>256), uint16_t, uint8_t>::type ring_buffer_pos_t;

    struct ring_buffer_r {
      volatile ring_buffer_pos_t head, tail;
      unsigned char buffer[Cfg::RX_SIZE];
    };

    struct ring_buffer_t {
      volatile uint8_t head, tail;
      unsigned char buffer[Cfg::TX_SIZE];
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

    static ring_buffer_pos_t rx_max_enqueued;

  protected: /** Protected Function */

    FORCE_INLINE static void store_rxd_char();
    FORCE_INLINE static void _tx_thr_empty_irq(void);

    static void UART_ISR(void);

  public: /** Public Function */

    static void begin(const long);
    static void end();
    static int peek(void);
    static int read(void);
    static void flush(void);
    static ring_buffer_pos_t available(void);
    static void write(const uint8_t c);
    static void flushTX(void);
    static size_t readBytes(char* buffer, size_t size);

    FORCE_INLINE static uint8_t dropped() { return Cfg::DROPPED_RX ? rx_dropped_bytes : 0; }
    FORCE_INLINE static uint8_t buffer_overruns() { return Cfg::RX_OVERRUNS ? rx_buffer_overruns : 0; }
    FORCE_INLINE static uint8_t framing_errors() { return Cfg::RX_FRAMING_ERRORS ? rx_framing_errors : 0; }
    FORCE_INLINE static ring_buffer_pos_t rxMaxEnqueued() { return Cfg::MAX_RX_QUEUED ? rx_max_enqueued : 0; }

    FORCE_INLINE static void write(const char* str) { while (*str) write(*str++); }
    FORCE_INLINE static void write(const uint8_t* buffer, size_t size) { while (size--) write(*buffer++); }
    FORCE_INLINE static void print(const String& s) { for (int i = 0; i < (int)s.length(); i++) write(s[i]); }
    FORCE_INLINE static void print(const char* str) { write(str); }

    static void print(char, int=BYTE);
    static void print(unsigned char, int=DEC);
    static void print(int, int=DEC);
    static void print(unsigned int, int=DEC);
    static void print(long, int=DEC);
    static void print(unsigned long, int=DEC);
    static void print(double, int=2);

    static void println(void);

    operator bool() { return true; }

  private: /** Private Function */

    static void printNumber(unsigned long, const uint8_t);
    static void printFloat(double, uint8_t);

};

#if SERIAL_PORT_1 >= 0
  extern MKHardwareSerial<MK4duoSerialHostCfg<SERIAL_PORT_1>> MKSerial1;
#endif

#if SERIAL_PORT_2 >= 0
  extern MKHardwareSerial<MK4duoSerialHostCfg<SERIAL_PORT_2>> MKSerial2;
#endif

#if ENABLED(NEXTION) && NEXTION_SERIAL > 0
  extern MKHardwareSerial<MK4duoSerialCfg<NEXTION_SERIAL>> nexSerial;
#endif

#if HAS_MMU2 && MMU2_SERIAL > 0
  extern MKHardwareSerial<MK4duoSerialCfg<MMU2_SERIAL>> mmuSerial;
#endif

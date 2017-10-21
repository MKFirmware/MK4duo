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
 * HardwareSerial.h - Hardware serial library for Arduino DUE
 * Copyright (c) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _HARDWARESERIAL_H_
#define _HARDWARESERIAL_H_

#ifndef SERIAL_PORT
  #define SERIAL_PORT 0
#endif

#define FORMAT_STRING_LENGTH 256

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer, in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
#define SERIAL_BUFFER_SIZE 128

class MK_RingBuffer {

  public: /** Constructor */

    MK_RingBuffer(void);

  public: /** Public Parameters */

    volatile uint8_t _aucBuffer[SERIAL_BUFFER_SIZE] ;
    volatile int _iHead ;
    volatile int _iTail ;

  public: /** Public Function */

    void store_char(const unsigned char c);

};

typedef void (*pfnISR_Handler)(void);
pfnISR_Handler install_isr(IRQn_Type irq, pfnISR_Handler newHandler);

class MKUARTClass {

  public: /** Constructor */

    MKUARTClass(Uart* pUart, IRQn_Type dwIrq, uint32_t dwId, MK_RingBuffer* pRx_buffer, MK_RingBuffer* pTx_buffer);

  public: /** Public Function */

    enum UARTModes {
      Mode_8N1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO,
      Mode_8E1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_EVEN,
      Mode_8O1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_ODD,
      Mode_8M1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_MARK,
      Mode_8S1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_SPACE,
    };

    void begin(const uint32_t dwBaudRate);
    void begin(const uint32_t dwBaudRate, const UARTModes config);

    void end(void);
    void flush(void);
    void checkRx(void);
    void write(const uint8_t uc_data);
    int peek(void);
    int read(void);
    int available(void);
    int availableForWrite(void);

    void IrqHandler(void);

    FORCE_INLINE void write(const char* str) { while (*str) write(*str++); }
    FORCE_INLINE void write(const uint8_t* buffer, size_t size) { while (size--) write(*buffer++); }
    FORCE_INLINE void print(const String& s) { for (int i = 0; i < (int)s.length(); i++) write(s[i]); }
    FORCE_INLINE void print(const char* str) { write(str); }

    void print(char, int = BYTE);
    void print(unsigned char, int = BYTE);
    void print(int, int = DEC);
    void print(unsigned int, int = DEC);
    void print(long, int = DEC);
    void print(unsigned long, int = DEC);
    void print(double, int = 2);

    void println(const String& s);
    void println(const char[]);
    void println(char, int = BYTE);
    void println(unsigned char, int = BYTE);
    void println(int, int = DEC);
    void println(unsigned int, int = DEC);
    void println(long, int = DEC);
    void println(unsigned long, int = DEC);
    void println(double, int = 2);
    void println(void);

  private: /** Private Function */

    void printNumber(unsigned long, const uint8_t);
    void printFloat(double, uint8_t);

  protected:

    void init(const uint32_t dwBaudRate, const uint32_t modeReg);

    MK_RingBuffer *_rx_buffer;
    MK_RingBuffer *_tx_buffer;

    Uart* _pUart;
    IRQn_Type _dwIrq;
    uint32_t _dwId;

};

class MKUSARTClass : public MKUARTClass {

  public: /** Constructor */
  
    MKUSARTClass(Usart* pUsart, IRQn_Type dwIrq, uint32_t dwId, MK_RingBuffer* pRx_buffer, MK_RingBuffer* pTx_buffer);

  public: /** Public Function */

    // 8x1 bit modes are inherited from UARTClass
    enum USARTModes {
      Mode_5N1 = US_MR_CHRL_5_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_1_BIT,
      Mode_6N1 = US_MR_CHRL_6_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_1_BIT,
      Mode_7N1 = US_MR_CHRL_7_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_1_BIT,
      Mode_5N2 = US_MR_CHRL_5_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_2_BIT,
      Mode_6N2 = US_MR_CHRL_6_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_2_BIT,
      Mode_7N2 = US_MR_CHRL_7_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_2_BIT,
      Mode_8N2 = US_MR_CHRL_8_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_2_BIT,
      Mode_5E1 = US_MR_CHRL_5_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_1_BIT,
      Mode_6E1 = US_MR_CHRL_6_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_1_BIT,
      Mode_7E1 = US_MR_CHRL_7_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_1_BIT,
      Mode_5E2 = US_MR_CHRL_5_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_2_BIT,
      Mode_6E2 = US_MR_CHRL_6_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_2_BIT,
      Mode_7E2 = US_MR_CHRL_7_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_2_BIT,
      Mode_8E2 = US_MR_CHRL_8_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_2_BIT,
      Mode_5O1 = US_MR_CHRL_5_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_1_BIT,
      Mode_6O1 = US_MR_CHRL_6_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_1_BIT,
      Mode_7O1 = US_MR_CHRL_7_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_1_BIT,
      Mode_5O2 = US_MR_CHRL_5_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_2_BIT,
      Mode_6O2 = US_MR_CHRL_6_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_2_BIT,
      Mode_7O2 = US_MR_CHRL_7_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_2_BIT,
      Mode_8O2 = US_MR_CHRL_8_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_2_BIT,
      Mode_5M1 = US_MR_CHRL_5_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_1_BIT,
      Mode_6M1 = US_MR_CHRL_6_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_1_BIT,
      Mode_7M1 = US_MR_CHRL_7_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_1_BIT,
      Mode_5M2 = US_MR_CHRL_5_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_2_BIT,
      Mode_6M2 = US_MR_CHRL_6_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_2_BIT,
      Mode_7M2 = US_MR_CHRL_7_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_2_BIT,
      Mode_8M2 = US_MR_CHRL_8_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_2_BIT,
      Mode_5S1 = US_MR_CHRL_5_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_1_BIT,
      Mode_6S1 = US_MR_CHRL_6_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_1_BIT,
      Mode_7S1 = US_MR_CHRL_7_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_1_BIT,
      Mode_5S2 = US_MR_CHRL_5_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_2_BIT,
      Mode_6S2 = US_MR_CHRL_6_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_2_BIT,
      Mode_7S2 = US_MR_CHRL_7_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_2_BIT,
      Mode_8S2 = US_MR_CHRL_8_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_2_BIT,
    };

    void begin(const uint32_t dwBaudRate);
    void begin(const uint32_t dwBaudRate, const USARTModes config);
    void begin(const uint32_t dwBaudRate, const UARTModes config);

  protected:

    Usart* _pUsart;

};

#if SERIAL_PORT == 0
  extern MKUARTClass MKSerial;
#else
  extern MKUSARTClass MKSerial;
#endif

#endif /* _HARDWARESERIAL_H_ */

/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

/**
 * driver.h
 *
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
 */

#define DRV_X_LABEL "X", 0
#define DRV_Y_LABEL "Y", 1
#define DRV_Z_LABEL "Z", 2

#define DRV_X2_LABEL "X2", 0
#define DRV_Y2_LABEL "Y2", 1
#define DRV_Z2_LABEL "Z2", 2
#define DRV_Z3_LABEL "Z3", 2

#define DRV_E0_LABEL "T0", 3
#define DRV_E1_LABEL "T1", 4
#define DRV_E2_LABEL "T2", 5
#define DRV_E3_LABEL "T3", 6
#define DRV_E4_LABEL "T4", 7
#define DRV_E5_LABEL "T5", 8

union driver_flag_t {
  bool all;
  struct {
    bool  enable      : 1;
    bool  dir         : 1;
    bool  step        : 1;
    bool  bit_3       : 1;
    bool  bit_4       : 1;
    bool  bit_5       : 1;
    bool  bit_6       : 1;
    bool  bit_7       : 1;
  };
  driver_flag_t() { all = false; }
};

struct driver_pin_t {
  pin_t enable,
        dir,
        step;
};

struct driver_data_t {
  driver_pin_t  pin;
  driver_flag_t flag;
};

class Driver {

  public: /** Constructor */

    Driver(const char* AXIS_LETTER, uint8_t) :
      axis_letter(AXIS_LETTER)
      {}

    #if HAVE_DRV(TMC2208)

      Driver(const char* AXIS_LETTER, const uint8_t DRIVER_ID, Stream * SerialPort, const float RS) :
        axis_letter(AXIS_LETTER)
        { tmc = new MKTMC(DRIVER_ID, SerialPort, RS); }

      Driver(const char* AXIS_LETTER, const uint8_t DRIVER_ID, const uint16_t RX, const uint16_t TX, const float RS, const bool has_rx) :
        axis_letter(AXIS_LETTER)
        { tmc = new MKTMC(DRIVER_ID, RX, TX, RS, has_rx); }

    #elif HAVE_DRV(TMC2660)

      Driver(const char* AXIS_LETTER, const uint8_t DRIVER_ID, const uint16_t CS, const float RS) :
        axis_letter(AXIS_LETTER)
        { tmc = new MKTMC(DRIVER_ID, CS, RS); }

      Driver(const char* AXIS_LETTER, const uint8_t DRIVER_ID, const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK) :
        axis_letter(AXIS_LETTER)
        { tmc = new MKTMC(DRIVER_ID, CS, RS, pinMOSI, pinMISO, pinSCK); }

    #elif HAS_TMCX1X0

      Driver(const char* AXIS_LETTER, const uint8_t DRIVER_ID, const uint16_t CS, const float RS) :
        axis_letter(AXIS_LETTER)
        { tmc = new MKTMC(DRIVER_ID, CS, RS); }

      Driver(const char* AXIS_LETTER, const uint8_t DRIVER_ID, const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK) :
        axis_letter(AXIS_LETTER)
        { tmc = new MKTMC(DRIVER_ID, CS, RS, pinMOSI, pinMISO, pinSCK); }

    #endif

  public: /** Public Parameters */

    driver_data_t data;
    const char*   axis_letter;

    #if HAS_TRINAMIC
      MKTMC* tmc = nullptr;
    #endif

  public: /** Public Function */

    /**
     * Initialize Driver hardware
     */
    FORCE_INLINE void init() {
      if (data.pin.enable != NoPin && data.pin.dir != NoPin && data.pin.step != NoPin) {
        dir_init();
        enable_init();
        if (!isEnable()) enable_write(HIGH);
        step_init();
        step_write(isStep());
      }
    }

    FORCE_INLINE void printLabel()                    { SERIAL_TXT(axis_letter); }

    FORCE_INLINE void setEnable(const bool onoff)     { data.flag.enable = onoff; }
    FORCE_INLINE bool isEnable()                      { return data.flag.enable; }
    FORCE_INLINE void setDir(const bool onoff)        { data.flag.dir = onoff; }
    FORCE_INLINE bool isDir()                         { return data.flag.dir; }
    FORCE_INLINE void setStep(const bool onoff)       { data.flag.step = onoff; }
    FORCE_INLINE bool isStep()                        { return data.flag.step; }

    FORCE_INLINE void enable_init()                   { HAL::pinMode(data.pin.enable, OUTPUT); }
    FORCE_INLINE void enable_write(const bool state)  { HAL::digitalWrite(data.pin.enable, state); }
    FORCE_INLINE bool enable_read()                   { return HAL::digitalRead(data.pin.enable); }
    FORCE_INLINE void dir_init()                      { HAL::pinMode(data.pin.dir, OUTPUT); }
    FORCE_INLINE void dir_write(const bool state)     { HAL::digitalWrite(data.pin.dir, state); }
    FORCE_INLINE bool dir_read()                      { return HAL::digitalRead(data.pin.dir); }
    FORCE_INLINE void step_init()                     { HAL::pinMode(data.pin.step, OUTPUT); }
    FORCE_INLINE void step_write(const bool state)    { HAL::digitalWrite(data.pin.step, state); }
    FORCE_INLINE void step_toggle(const bool state)   { /*if (state) TOGGLE(data.pin.step); */}
    FORCE_INLINE bool step_read()                     { return HAL::digitalRead(data.pin.step); }

};

extern Driver* driver[MAX_DRIVER];

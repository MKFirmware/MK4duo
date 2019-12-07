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
    bool  enable        : 1;
    bool  dir           : 1;
    bool  step          : 1;
    bool  enable_status : 1;
    bool  dir_status    : 1;
    bool  step_status   : 1;
    bool  bit_6         : 1;
    bool  bit_7         : 1;
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
  #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
    uint16_t ma;
  #endif
};

class Driver {

  public: /** Constructor */

    Driver(const char* AXIS_LETTER) :
      axis_letter(AXIS_LETTER)
      {}

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
    void init();

    FORCE_INLINE void printLabel()                    { SERIAL_TXT(axis_letter); }

    FORCE_INLINE void setEnable(const bool onoff)     { data.flag.enable = onoff; }
    FORCE_INLINE bool isEnable()                      { return data.flag.enable; }
    FORCE_INLINE void setDir(const bool onoff)        { data.flag.dir = onoff; }
    FORCE_INLINE bool isDir()                         { return data.flag.dir; }
    FORCE_INLINE void setStep(const bool onoff)       { data.flag.step = onoff; }
    FORCE_INLINE bool isStep()                        { return data.flag.step; }

    /**
     * Function for enable
     */
    FORCE_INLINE void enable_init() {
      #if HAS_TRINAMIC && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
        if (tmc) return;
      #endif
      HAL::pinMode(data.pin.enable, OUTPUT);
      if (!isEnable()) enable_write(HIGH);
    }
    FORCE_INLINE void enable_write(const bool state) {
      #if HAS_TRINAMIC && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
        if (tmc)
          return tmc->toff(state == isEnable() ? chopper_timing.toff : 0);
      #endif
      HAL::digitalWrite(data.pin.enable, state);
      data.flag.enable_status = state;
    }
    FORCE_INLINE bool enable_read() {
      #if HAS_TRINAMIC && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
        if (tmc) return tmc->isEnabled();
      #endif
      return data.flag.enable_status;
    }

    /**
     * Function for dir
     */
    FORCE_INLINE void dir_init() {
      HAL::pinMode(data.pin.dir, OUTPUT);
    }
    FORCE_INLINE void dir_write(const bool state) {
      HAL::digitalWrite(data.pin.dir, state);
      data.flag.dir_status = state;
    }
    FORCE_INLINE bool dir_read() {
      return data.flag.dir_status;
    }

    /**
     * Function for step
     */
    FORCE_INLINE void step_init() {
      HAL::pinMode(data.pin.step, OUTPUT);
      step_write(isStep());
    }
    FORCE_INLINE void step_write(const bool state) {
      #if ENABLED(SQUARE_WAVE_STEPPING)
        if (tmc) return step_toggle(state);
      #endif
      HAL::digitalWrite(data.pin.step, state);
      data.flag.step_status = state;
    }
    FORCE_INLINE void step_toggle(const bool state) {
      if (state) {
        HAL::digitalWrite(data.pin.step, !data.flag.step_status);
        data.flag.step_status = !data.flag.step_status;
      }
    }
    FORCE_INLINE bool step_read() {
      return data.flag.step_status;
    }

};

struct driver_t {
  union {
    struct { Driver *x, *y, *z
      , *e[MAX_DRIVER_E]
      #if X_STEPPER_COUNT == 2
        , *x2
      #endif
      #if Y_STEPPER_COUNT == 2
        , *y2
      #endif
      #if Z_STEPPER_COUNT >= 2
        , *z2
      #endif
      #if Z_STEPPER_COUNT == 3
        , *z3
      #endif
      ;
    };
    Driver* drv[MAX_DRIVER];
  };

  Driver*   operator[](const int i) { return this->drv[i]; }

};

extern driver_t driver;

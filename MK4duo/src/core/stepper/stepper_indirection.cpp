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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * stepper_indirection.cpp
 *
 * Stepper motor driver indirection to allow some stepper functions to
 * be done via SPI/I2c instead of direct pin manipulation.
 *
 * Part of MK4duo
 *
 * Copyright (c) 2015 Dominik Wenger
 */

#include "../../../MK4duo.h"
#include "stepper_indirection.h"

//
// TMC26X Driver objects and inits
//
#if HAVE_DRV(TMC26X)

  #include <TMC26XStepper.h>

  #define _TMC26X_DEFINE(ST) TMC26XStepper stepper##ST(200, ST##_ENABLE_PIN, ST##_STEP_PIN, ST##_DIR_PIN, ST##_MAX_CURRENT, ST##_SENSE_RESISTOR)

  #if X_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(X);
  #endif
  #if X2_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(X2);
  #endif
  #if Y_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(Y);
  #endif
  #if Y2_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(Y2);
  #endif
  #if Z_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(Z);
  #endif
  #if Z2_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(Z2);
  #endif
  #if Z3_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(Z3);
  #endif
  #if E0_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(E0);
  #endif
  #if E1_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(E1);
  #endif
  #if E2_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(E2);
  #endif
  #if E3_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(E3);
  #endif
  #if E4_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(E4);
  #endif
  #if E5_HAS_DRV(TMC26X)
    _TMC26X_DEFINE(E5);
  #endif

  #define _TMC26X_INIT(A) do{ \
    stepper##A.setMicrosteps(A##_MICROSTEPS); \
    stepper##A.start(); \
  } while(0)

  void tmc26x_init_to_defaults() {
    #if X_HAS_DRV(TMC26X)
      _TMC26X_INIT(X);
    #endif
    #if X2_HAS_DRV(TMC26X)
      _TMC26X_INIT(X2);
    #endif
    #if Y_HAS_DRV(TMC26X)
      _TMC26X_INIT(Y);
    #endif
    #if Y2_HAS_DRV(TMC26X)
      _TMC26X_INIT(Y2);
    #endif
    #if Z_HAS_DRV(TMC26X)
      _TMC26X_INIT(Z);
    #endif
    #if Z2_HAS_DRV(TMC26X)
      _TMC26X_INIT(Z2);
    #endif
    #if Z3_HAS_DRV(TMC26X)
      _TMC26X_INIT(Z3);
    #endif
    #if E0_HAS_DRV(TMC26X)
      _TMC26X_INIT(E0);
    #endif
    #if E1_HAS_DRV(TMC26X)
      _TMC26X_INIT(E1);
    #endif
    #if E2_HAS_DRV(TMC26X)
      _TMC26X_INIT(E2);
    #endif
    #if E3_HAS_DRV(TMC26X)
      _TMC26X_INIT(E3);
    #endif
    #if E4_HAS_DRV(TMC26X)
      _TMC26X_INIT(E4);
    #endif
    #if E5_HAS_DRV(TMC26X)
      _TMC26X_INIT(E5);
    #endif
  }

#endif // HAVE_DRV(TMC26X)

//
// TMC2130 Driver objects and inits
//
#if HAVE_DRV(TMC2130)

  #include <TMC2130Stepper.h>

  #if TMC2130STEPPER_VERSION < 0x020201
    #error "DEPENDENCY ERROR: Update TMC2130Stepper library to 2.2.1 or newer."
  #endif

  #if ENABLED(SOFT_SPI_TMC2130)
    #define _TMC2130_DEFINE(ST) TMC2130Stepper stepper##ST = TMC2130Stepper(ST##_ENABLE_PIN, ST##_DIR_PIN, ST##_STEP_PIN, ST##_CS_PIN, SOFT_MOSI_PIN, SOFT_MISO_PIN, SOFT_SCK_PIN)
  #else
    #define _TMC2130_DEFINE(ST) TMC2130Stepper stepper##ST = TMC2130Stepper(ST##_ENABLE_PIN, ST##_DIR_PIN, ST##_STEP_PIN, ST##_CS_PIN)
  #endif

  // Stepper objects of TMC2130 steppers used
  #if X_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(X);
  #endif
  #if X2_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(X2);
  #endif
  #if Y_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(Y);
  #endif
  #if Y2_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(Y2);
  #endif
  #if Z_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(Z);
  #endif
  #if Z2_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(Z2);
  #endif
  #if Z3_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(Z3);
  #endif
  #if E0_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(E0);
  #endif
  #if E1_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(E1);
  #endif
  #if E2_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(E2);
  #endif
  #if E3_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(E3);
  #endif
  #if E4_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(E4);
  #endif
  #if E5_HAS_DRV(TMC2130)
    _TMC2130_DEFINE(E5);
  #endif

  // Use internal reference voltage for current calculations. This is the default.
  // Following values from Trinamic's spreadsheet with values for a NEMA17 (42BYGHW609)
  // https://www.trinamic.com/products/integrated-circuits/details/tmc2130/
  void tmc2130_init(TMC2130Stepper &st, const uint16_t mA, const uint16_t microsteps, const uint32_t thrs, const float spmm, const bool tmc_stealthchop=false) {
    while(!st.stst()); // Wait for motor stand-still
    #if DISABLED(STEALTHCHOP) || DISABLED(HYBRID_THRESHOLD)
      UNUSED(thrs);
      UNUSED(spmm);
    #endif
    st.begin();
    st.setCurrent(mA, R_SENSE, HOLD_MULTIPLIER);
    st.microsteps(microsteps);
    st.interpolate(INTERPOLATE);
    st.stealth_amplitude(STEALTH_AMPL);
    st.stealth_gradient(STEALTH_GRAD);
    st.stealth_autoscale(STEALTH_AUTOSCALE);
    st.stealth_freq(STEALTH_FREQ);
    st.stealthChop(tmc_stealthchop);
    st.sgt(0);
    #if ENABLED(HYBRID_THRESHOLD)
      st.stealth_max_speed(12650000UL * microsteps / (256 * thrs * spmm));
    #endif
    #if ENABLED(TMC2130_LINEARITY_CORRECTION_PRESET)
      tmc2130_set_fixed_wave(st, wave);
    #endif
    st.reset(); // Clear GSTAT
  }

  #define _TMC2130_INIT(ST, SPMM) tmc2130_init(stepper##ST, ST##_CURRENT, ST##_MICROSTEPS, ST##_HYBRID_THRESHOLD, SPMM, ST##_STEALTHCHOP)

  void tmc2130_init_to_defaults() {
    #if X_HAS_DRV(TMC2130)
      _TMC2130_INIT( X, mechanics.axis_steps_per_mm[X_AXIS]);
    #endif
    #if X2_HAS_DRV(TMC2130)
      _TMC2130_INIT(X2, mechanics.axis_steps_per_mm[X_AXIS]);
    #endif
    #if Y_HAS_DRV(TMC2130)
      _TMC2130_INIT( Y, mechanics.axis_steps_per_mm[Y_AXIS]);
    #endif
    #if Y2_HAS_DRV(TMC2130)
      _TMC2130_INIT(Y2, mechanics.axis_steps_per_mm[Y_AXIS]);
    #endif
    #if Z_HAS_DRV(TMC2130)
      _TMC2130_INIT( Z, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if Z2_HAS_DRV(TMC2130)
      _TMC2130_INIT(Z2, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if Z3_HAS_DRV(TMC2130)
      _TMC2130_INIT(Z3, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if E0_HAS_DRV(TMC2130)
      _TMC2130_INIT(E0, mechanics.axis_steps_per_mm[E_AXIS]);
    #endif
    #if E1_HAS_DRV(TMC2130)
      _TMC2130_INIT(E1, mechanics.axis_steps_per_mm[E_AXIS + 1]);
    #endif
    #if E2_HAS_DRV(TMC2130)
      _TMC2130_INIT(E2, mechanics.axis_steps_per_mm[E_AXIS + 2]);
    #endif
    #if E3_HAS_DRV(TMC2130)
      _TMC2130_INIT(E3, mechanics.axis_steps_per_mm[E_AXIS + 3]);
    #endif
    #if E4_HAS_DRV(TMC2130)
      _TMC2130_INIT(E4, mechanics.axis_steps_per_mm[E_AXIS + 4]);
    #endif
    #if E5_HAS_DRV(TMC2130)
      _TMC2130_INIT(E5, mechanics.axis_steps_per_mm[E_AXIS + 5]);
    #endif

    #if ENABLED(SENSORLESS_HOMING)
      #if X_SENSORLESS
        #if X_HAS_DRV(TMC2130)
          stepperX.sgt(X_HOMING_SENSITIVITY);
        #endif
        #if X2_HAS_DRV(TMC2130)
          stepperX2.sgt(X_HOMING_SENSITIVITY);
        #endif
      #endif
      #if Y_SENSORLESS
        #if Y_HAS_DRV(TMC2130)
          stepperY.sgt(Y_HOMING_SENSITIVITY);
        #endif
        #if Y2_HAS_DRV(TMC2130)
          stepperY2.sgt(Y_HOMING_SENSITIVITY);
        #endif
      #endif
      #if Z_SENSORLESS
        #if Z_HAS_DRV(TMC2130)
          stepperZ.sgt(Z_HOMING_SENSITIVITY);
        #endif
        #if Z2_HAS_DRV(TMC2130)
          stepperZ2.sgt(Z_HOMING_SENSITIVITY);
        #endif
        #if Z3_HAS_DRV(TMC2130)
          stepperZ3.sgt(Z_HOMING_SENSITIVITY);
        #endif
      #endif
    #endif
  }

  #if ENABLED(TMC2130_LINEARITY_CORRECTION) // TMC2130_LINEARITY_CORRECTION

    void tmc2130_wr_MSLUTSTART(TMC2130Stepper &st, uint8_t start_sin, uint8_t start_sin90) {
      const uint32_t val = uint32_t(start_sin) | (uint32_t(start_sin90) << 16);
      SERIAL_EMV(" MSLUTSTART: ", val);
      st.lut_mslutstart(val);
    }

    void tmc2130_wr_MSLUTSEL(TMC2130Stepper &st, uint8_t x1, uint8_t x2, uint8_t x3, int8_t w0, int8_t w1, int8_t w2, int8_t w3) {
      const uint32_t val =  uint32_t(w0)
                         | (uint32_t(w1) <<  2) | (uint32_t(w2) <<  4) | (uint32_t(w3) <<  6)
                         | (uint32_t(x1) <<  8) | (uint32_t(x2) << 16) | (uint32_t(x3) << 24);
      SERIAL_EMV(" MSLUTSEL: ", val);
      st.lut_msutsel(val);
    }

    void tmc2130_wr_MSLUT(TMC2130Stepper &st, uint8_t i, uint32_t val) {
      SERIAL_MV(" MSLUT", i);
      SERIAL_EMV(" : ", val);
      switch (i) {
        case 0: st.lut_mslut0(val); break;
        case 1: st.lut_mslut1(val); break;
        case 2: st.lut_mslut2(val); break;
        case 3: st.lut_mslut3(val); break;
        case 4: st.lut_mslut4(val); break;
        case 5: st.lut_mslut5(val); break;
        case 6: st.lut_mslut6(val); break;
        case 7: st.lut_mslut7(val); break;
        default: break;
      }
    }

    void tmc2130_reset_wave(TMC2130Stepper &st) { // TMC2130_LINEARITY_CORRECTION
      SERIAL_EM(" MSLUT RESET ");
      st.lut_mslutstart(0x00F70000UL);      // 0x00F70000 16187392
      st.lut_mslut0(0xAAAAB554UL);          // 0xAAAAB554 2863314260
      st.lut_mslut1(0x4A9554AAUL);          // 0x4A9554AA 1251300522
      st.lut_mslut2(0x24492929UL);          // 0x24492929 608774441
      st.lut_mslut3(0x10104222UL);          // 0x10104222 269500962
      st.lut_mslut4(0xFBFFFFFFUL);          // 0xFBFFFFFF 4227858431
      st.lut_mslut5(0xB5BB777DUL);          // 0xB5BB777D 3048961917
      st.lut_mslut6(0x49295556UL);          // 0x49295556 1227445590
      st.lut_mslut7(0x00404222UL);          // 0x00404222 4211234
      st.lut_msutsel(0xFFFF8056UL);         // 0xFFFF8056 4294934614
    }

    void tmc2130_set_fixed_wave(TMC2130Stepper &st, const uint8_t i) { // TMC2130_LINEARITY_CORRECTION
      SERIAL_MSG(" MSLUT Wave: ");
      switch (i) {
        case 1:
          SERIAL_EM("SoundCorrection [X]");
          st.lut_mslutstart(0x00FC0000UL);
          st.lut_mslut0(0xAAAAB554UL);
          st.lut_mslut1(0x52AAAAAAUL);
          st.lut_mslut2(0x91252529UL);
          st.lut_mslut3(0x10208848UL);
          st.lut_mslut4(0xBFFC0200UL);
          st.lut_mslut5(0xB6DBBBDFUL);
          st.lut_mslut6(0x24A55556UL);
          st.lut_mslut7(0x00041089UL);
          st.lut_msutsel(0xFFFF9156UL);
          break;
        case 2:
          SERIAL_EM("LaserCorrection [X]");
          st.lut_mslutstart(0x00FC0000UL);
          st.lut_mslut0(0xD52ADB54UL);
          st.lut_mslut1(0x52AAAAAAUL);
          st.lut_mslut2(0x91252529UL);
          st.lut_mslut3(0x10208848UL);
          st.lut_mslut4(0xBFFC0200UL);
          st.lut_mslut5(0xB6DBBBDFUL);
          st.lut_mslut6(0x24A55556UL);
          st.lut_mslut7(0x00040089UL);
          st.lut_msutsel(0xFFFF9156UL);
          break;
        case 3:
          SERIAL_EM("SoundCorrection [E]");
          st.lut_mslutstart(0x00FC0000UL);
          st.lut_mslut0(0x6B6DBBDEUL);
          st.lut_mslut1(0x4AAAAAADUL);
          st.lut_mslut2(0x444924A5UL);
          st.lut_mslut3(0x20041044UL);
          st.lut_mslut4(0xBEFF8000UL);
          st.lut_mslut5(0x5ADB76F7UL);
          st.lut_mslut6(0x894A5555UL);
          st.lut_mslut7(0x00020844UL);
          st.lut_msutsel(0xFFFF8E56UL);
          break;
        case 4:
          SERIAL_EM("LaserCorrection [E]");
          st.lut_mslutstart(0x00FD0000UL);
          st.lut_mslut0(0xD5BAD554UL);
          st.lut_mslut1(0x92A4AAABUL);
          st.lut_mslut2(0x91356529UL);
          st.lut_mslut3(0x10248848UL);
          st.lut_mslut4(0xBFDC2200UL);
          st.lut_mslut5(0xB6DABBDBUL);
          st.lut_mslut6(0xA4A85552UL);
          st.lut_mslut7(0x00040089UL);
          st.lut_msutsel(0xFFFF9156UL);
          break;
        default:
          SERIAL_EM(" ERROR: Not a preset value. ");
      }
    }

    void tmc2130_set_wave(  TMC2130Stepper &st, const uint8_t amp, int16_t fac1000,
                            const int8_t xoff/*=0*/, const int8_t yoff/*=10*/, const uint8_t wavetype/*=0*/,
                            const bool config/*=0*/, const uint8_t addto/*=0*/
    ) {
      // TMC2130 wave compression algorithm
      // optimized for minimal memory requirements
      // wavetype: 0 (default) pow(Sin,1+x); 1 pow(Cycle,2+x)
      NOLESS(fac1000, TMC2130_WAVE_FAC1000_MIN);
      NOMORE(fac1000, TMC2130_WAVE_FAC1000_MAX);
      SERIAL_MSG(" tmc2130_set_wave: ");
      SERIAL_PS(wavetype == 1 ? PSTR("Cycle") : PSTR("Sin"));
      SERIAL_MSG("-Factor: ");
      const float fac = float((wavetype == 1 ? 2000 : 1000) + fac1000) / 1000;
      SERIAL_EV(fac);
      uint8_t vA = 0,                // Value of currentA
              va = 0,                // Previous vA
              w[4] = {1,1,1,1},      // W bits (MSLUTSEL)
              x[3] = {255,255,255};  // X segment bounds (MSLUTSEL)
      int8_t s = 0,                  // Current segment
             b,                      // Encoded bit value
             dA;                     // Delta value
      int i;                         // Microstep index
      uint32_t reg;                  // TMC2130 register
      tmc2130_wr_MSLUTSTART(st, 0, amp);
      // Set first W-Bit
      if (wavetype == 1)   // Cycle
        w[0] = uint8_t((amp - 1) * pow(1 - pow(float(xoff) / 256 - 1, 2), fac / 2) + yoff / 10);
      else   // Sin
        w[0] = uint8_t((amp - 1) * pow(sin((2 * PI * xoff) / 1024), fac) + yoff / 10);
      for (i = 0; i < 256; i++) {
        if (wavetype == 1)   // Cycle
          vA = uint8_t((amp - 1) * pow(1 - pow((float(i) + xoff) / 256 - 1, 2), fac / 2) + yoff / 10);
        else   // Sin
          vA = uint8_t((amp - 1) * pow(sin((2 * PI * (i + xoff)) / 1024), fac) + yoff / 10);
        dA = int8_t(vA - va);
        if (dA > w[0]) { w[0]++; break; }
        if (dA < w[0]) { /* w[0]--; */ break; }
        va = vA;
      }
      int8_t d1 = (int8_t)w[0], d0 = d1 - 1;
      // calc MSLUT
      va = 0;
      for (i = 0; i < 256 ; i++) {
        if (!(i & 0x1F)) reg = 0;
        if (wavetype == 1)   // Cycle
          vA = uint8_t((amp - 1) * pow(1 - pow((float(i) + xoff) / 256 - 1, 2), fac / 2) + yoff / 10);
        else   // Sin
          vA = uint8_t((amp - 1) * pow(sin((2 * PI * (i + xoff)) / 1024), fac) + yoff / 10); // + 0.5 for Round (compiler will truncate)

        dA = (int8_t)(vA - va); // calculate delta
        va = vA;
        b = -1;
        if (dA == d0)           // delta == delta0 => bit=0
          b = 0;
        else if (dA == d1)      // delta == delta1 => bit=1
          b = 1;
        else if (dA < d0) {     // delta < delta0 => switch wbit down
          b = 0;
          switch (dA) {
            case -1: d0 = -1; d1 = 0; w[s+1] = 0; break;
            case  0: d0 =  0; d1 = 1; w[s+1] = 1; break;
            case  1: d0 =  1; d1 = 2; w[s+1] = 2; break;
            default: b = -1; break;
          }
          if (b >= 0) x[s++] = i;
        }
        else if (dA > d1) { // delta > delta0 => switch wbit up
          b = 1;
          switch (dA) {
            case  1: d0 =  0; d1 = 1; w[s+1] = 1; break;
            case  2: d0 =  1; d1 = 2; w[s+1] = 2; break;
            case  3: d0 =  2; d1 = 3; w[s+1] = 3; break;
            default: b = -1; break;
          }
          if (b >= 0) x[s++] = i;
        }
        if (config == 1 && i == addto) {  // Additional Configuration
          if (b == 0) b = 1;
          else if (b == 1) b = 0;
        }
        if (b < 0) break; // delta out of range (<-1 or >3)
        if (s > 3) break; // segment out of range (> 3)
        if (b == 1) reg |= 0x80000000;

        if ((i & 0x1F) == 0x1F)
          tmc2130_wr_MSLUT(st, (uint8_t)(i >> 5), reg);
        else
          reg >>= 1;
      }
      tmc2130_wr_MSLUTSEL(st, x[0], x[1], x[2], w[0], w[1], w[2], w[3]);
      SERIAL_MV(" X = ", x[0]);
      SERIAL_MV(" - ", x[1]);
      SERIAL_MV(" - ", x[2]);
      SERIAL_MV(" W = ", w[0]);
      SERIAL_MV(" - ", w[1]);
      SERIAL_MV(" - ", w[2]);
      SERIAL_MV(" - ", w[3]);
      SERIL_EOL();
    }

  #endif // TMC2130_LINEARITY_CORRECTION

  void tmc_init_cs_pins() {
    #if PIN_EXISTS(X_CS)
      OUT_WRITE(X_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(Y_CS)
      OUT_WRITE(Y_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(Z_CS)
      OUT_WRITE(Z_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(X2_CS)
      OUT_WRITE(X2_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(Y2_CS)
      OUT_WRITE(Y2_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(Z2_CS)
      OUT_WRITE(Z2_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(Z3_CS)
      OUT_WRITE(Z3_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(E0_CS)
      OUT_WRITE(E0_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(E1_CS)
      OUT_WRITE(E1_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(E2_CS)
      OUT_WRITE(E2_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(E3_CS)
      OUT_WRITE(E3_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(E4_CS)
      OUT_WRITE(E4_CS_PIN, HIGH);
    #endif
    #if PIN_EXISTS(E5_CS)
      OUT_WRITE(E5_CS_PIN, HIGH);
    #endif
  }

#endif // HAVE_DRV(TMC2130)

//
// TMC2208 Driver objects and inits
//
#if HAVE_DRV(TMC2208)

  #if ENABLED(__AVR__)
    #include <SoftwareSerial.h>
    #include <HardwareSerial.h>
  #endif

  #include <TMC2208Stepper.h>

  #if TMC2208STEPPER_VERSION < 0x000101
    #error "DEPENDENCY ERROR: Update TMC2208Stepper library to 0.1.1 or newer."
  #endif

  #define _TMC2208_DEFINE_HARDWARE(ST) TMC2208Stepper stepper##ST(&ST##_HARDWARE_SERIAL)
  #define _TMC2208_DEFINE_SOFTWARE(ST) SoftwareSerial ST##_HARDWARE_SERIAL = SoftwareSerial(ST##_SERIAL_RX_PIN, ST##_SERIAL_TX_PIN); \
                                       TMC2208Stepper stepper##ST(&ST##_HARDWARE_SERIAL, ST##_SERIAL_RX_PIN > -1)

  // Stepper objects of TMC2208 steppers used
  #if X_HAS_DRV(TMC2208)
    #if ENABLED(X_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(X);
    #else
      _TMC2208_DEFINE_SOFTWARE(X);
    #endif
  #endif
  #if X2_HAS_DRV(TMC2208)
    #if ENABLED(X2_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(X2);
    #else
      _TMC2208_DEFINE_SOFTWARE(X2);
    #endif
  #endif
  #if Y_HAS_DRV(TMC2208)
    #if ENABLED(Y_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(Y);
    #else
      _TMC2208_DEFINE_SOFTWARE(Y);
    #endif
  #endif
  #if Y2_HAS_DRV(TMC2208)
    #if ENABLED(Y2_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(Y2);
    #else
      _TMC2208_DEFINE_SOFTWARE(Y2);
    #endif
  #endif
  #if Z_HAS_DRV(TMC2208)
    #if ENABLED(Z_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(Z);
    #else
      _TMC2208_DEFINE_SOFTWARE(Z);
    #endif
  #endif
  #if Z2_HAS_DRV(TMC2208)
    #if ENABLED(Z2_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(Z2);
    #else
      _TMC2208_DEFINE_SOFTWARE(Z2);
    #endif
  #endif
  #if Z3_HAS_DRV(TMC2208)
    #if ENABLED(Z3_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(Z3);
    #else
      _TMC2208_DEFINE_SOFTWARE(Z3);
    #endif
  #endif
  #if E0_HAS_DRV(TMC2208)
    #if ENABLED(E0_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(E0);
    #else
      _TMC2208_DEFINE_SOFTWARE(E0);
    #endif
  #endif
  #if E1_HAS_DRV(TMC2208)
    #if ENABLED(E1_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(E1);
    #else
      _TMC2208_DEFINE_SOFTWARE(E1);
    #endif
  #endif
  #if E2_HAS_DRV(TMC2208)
    #if ENABLED(E2_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(E2);
    #else
      _TMC2208_DEFINE_SOFTWARE(E2);
    #endif
  #endif
  #if E3_HAS_DRV(TMC2208)
    #if ENABLED(E3_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(E3);
    #else
      _TMC2208_DEFINE_SOFTWARE(E3);
    #endif
  #endif
  #if E4_HAS_DRV(TMC2208)
    #if ENABLED(E4_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(E4);
    #else
      _TMC2208_DEFINE_SOFTWARE(E4);
    #endif
  #endif
  #if E5_HAS_DRV(TMC2208)
    #if ENABLED(E5_HARDWARE_SERIAL)
      _TMC2208_DEFINE_HARDWARE(E5);
    #else
      _TMC2208_DEFINE_SOFTWARE(E5);
    #endif
  #endif

  void tmc2208_serial_begin() {
    #if X_HAS_DRV(TMC2208)
      X_HARDWARE_SERIAL.begin(115200);
    #endif
    #if X2_HAS_DRV(TMC2208)
      X2_HARDWARE_SERIAL.begin(115200);
    #endif
    #if Y_HAS_DRV(TMC2208)
      Y_HARDWARE_SERIAL.begin(115200);
    #endif
    #if Y2_HAS_DRV(TMC2208)
      Y2_HARDWARE_SERIAL.begin(115200);
    #endif
    #if Z_HAS_DRV(TMC2208)
      Z_HARDWARE_SERIAL.begin(115200);
    #endif
    #if Z2_HAS_DRV(TMC2208)
      Z2_HARDWARE_SERIAL.begin(115200);
    #endif
    #if Z3_HAS_DRV(TMC2208)
      Z3_HARDWARE_SERIAL.begin(115200);
    #endif
    #if E0_HAS_DRV(TMC2208)
      E0_HARDWARE_SERIAL.begin(115200);
    #endif
    #if E1_HAS_DRV(TMC2208)
      E1_HARDWARE_SERIAL.begin(115200);
    #endif
    #if E2_HAS_DRV(TMC2208)
      E2_HARDWARE_SERIAL.begin(115200);
    #endif
    #if E3_HAS_DRV(TMC2208)
      E3_HARDWARE_SERIAL.begin(115200);
    #endif
    #if E4_HAS_DRV(TMC2208)
      E4_HARDWARE_SERIAL.begin(115200);
    #endif
    #if E5_HAS_DRV(TMC2208)
      E5_HARDWARE_SERIAL.begin(115200);
    #endif
  }

  // Use internal reference voltage for current calculations. This is the default.
  // Following values from Trinamic's spreadsheet with values for a NEMA17 (42BYGHW609)
  void tmc2208_init(TMC2208Stepper &st, const uint16_t mA, const uint16_t microsteps, const uint32_t thrs, const float spmm) {
    st.pdn_disable(true); // Use UART
    st.mstep_reg_select(true); // Select microsteps with UART
    st.I_scale_analog(false);
    st.rms_current(mA, HOLD_MULTIPLIER, R_SENSE);
    st.microsteps(microsteps);
    st.blank_time(24);
    st.toff(5);
    st.intpol(INTERPOLATE);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current
    st.hysteresis_start(3);
    st.hysteresis_end(2);
    #if ENABLED(STEALTHCHOP)
      st.pwm_lim(12);
      st.pwm_reg(8);
      st.pwm_autograd(1);
      st.pwm_autoscale(1);
      st.pwm_freq(1);
      st.pwm_grad(14);
      st.pwm_ofs(36);
      st.en_spreadCycle(false);
      #if ENABLED(HYBRID_THRESHOLD)
        st.TPWMTHRS(12650000UL*microsteps/(256*thrs*spmm));
      #else
        UNUSED(thrs);
        UNUSED(spmm);
      #endif
    #else
      st.en_spreadCycle(true);
    #endif
    st.GSTAT(0b111); // Clear
    delay(200);
  }

  #define _TMC2208_INIT(ST, SPMM) tmc2208_init(stepper##ST, ST##_CURRENT, ST##_MICROSTEPS, ST##_HYBRID_THRESHOLD, SPMM)

  void tmc2208_init_to_defaults() {
    #if X_HAS_DRV(TMC2208)
      _TMC2208_INIT(X, mechanics.axis_steps_per_mm[X_AXIS]);
    #endif
    #if X2_HAS_DRV(TMC2208)
      _TMC2208_INIT(X2, mechanics.axis_steps_per_mm[X_AXIS]);
    #endif
    #if Y_HAS_DRV(TMC2208)
      _TMC2208_INIT(Y, mechanics.axis_steps_per_mm[Y_AXIS]);
    #endif
    #if Y2_HAS_DRV(TMC2208)
      _TMC2208_INIT(Y2, mechanics.axis_steps_per_mm[Y_AXIS]);
    #endif
    #if Z_HAS_DRV(TMC2208)
      _TMC2208_INIT(Z, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if Z2_HAS_DRV(TMC2208)
      _TMC2208_INIT(Z2, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if Z3_HAS_DRV(TMC2208)
      _TMC2208_INIT(Z3, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if E0_HAS_DRV(TMC2208)
      _TMC2208_INIT(E0, mechanics.axis_steps_per_mm[E_AXIS]);
    #endif
    #if E1_HAS_DRV(TMC2208)
      _TMC2208_INIT(E1, mechanics.axis_steps_per_mm[E_AXIS + 1]);
    #endif
    #if E2_HAS_DRV(TMC2208)
      _TMC2208_INIT(E2, mechanics.axis_steps_per_mm[E_AXIS + 2]);
    #endif
    #if E3_HAS_DRV(TMC2208)
      _TMC2208_INIT(E3, mechanics.axis_steps_per_mm[E_AXIS + 3]);
    #endif
    #if E4_HAS_DRV(TMC2208)
      _TMC2208_INIT(E4, mechanics.axis_steps_per_mm[E_AXIS + 4]);
    #endif
    #if E5_HAS_DRV(TMC2208)
      _TMC2208_INIT(E5, mechanics.axis_steps_per_mm[E_AXIS + 5]);
    #endif
  }
#endif // HAVE_DRV(TMC2208)

void restore_stepper_drivers() {
  #if X_IS_TRINAMIC
    stepperX.push();
  #endif
  #if X2_IS_TRINAMIC
    stepperX2.push();
  #endif
  #if Y_IS_TRINAMIC
    stepperY.push();
  #endif
  #if Y2_IS_TRINAMIC
    stepperY2.push();
  #endif
  #if Z_IS_TRINAMIC
    stepperZ.push();
  #endif
  #if Z2_IS_TRINAMIC
    stepperZ2.push();
  #endif
  #if Z3_IS_TRINAMIC
    stepperZ3.push();
  #endif
  #if E0_IS_TRINAMIC
    stepperE0.push();
  #endif
  #if E1_IS_TRINAMIC
    stepperE1.push();
  #endif
  #if E2_IS_TRINAMIC
    stepperE2.push();
  #endif
  #if E3_IS_TRINAMIC
    stepperE3.push();
  #endif
  #if E4_IS_TRINAMIC
    stepperE4.push();
  #endif
}

void reset_stepper_drivers() {
  #if HAVE_DRV(TMC26X)
    tmc26x_init_to_defaults();
  #endif
  #if HAVE_DRV(TMC2130)
    delay(100);
    tmc2130_init_to_defaults();
  #endif
  #if HAVE_DRV(TMC2208)
    delay(100);
    tmc2208_init_to_defaults();
  #endif
  #if HAVE_DRV(L6470)
    L6470_init_to_defaults();
  #endif
  #if ENABLED(TMC_ADV)
    TMC_ADV()
  #endif
  stepper.set_directions();
}

//
// L6470 Driver objects and inits
//
#if HAVE_DRV(L6470)

  #include <L6470.h>

  #define _L6470_DEFINE(ST) L6470 stepper##ST(ST##_ENABLE_PIN)

  // L6470 Stepper objects
  #if X_HAS_DRV(L6470)
    _L6470_DEFINE(X);
  #endif
  #if X2_HAS_DRV(L6470)
    _L6470_DEFINE(X2);
  #endif
  #if Y_HAS_DRV(L6470)
    _L6470_DEFINE(Y);
  #endif
  #if Y2_HAS_DRV(L6470)
    _L6470_DEFINE(Y2);
  #endif
  #if Z_HAS_DRV(L6470)
    _L6470_DEFINE(Z);
  #endif
  #if Z2_HAS_DRV(L6470)
    _L6470_DEFINE(Z2);
  #endif
  #if Z3_HAS_DRV(L6470)
    _L6470_DEFINE(Z3);
  #endif
  #if E0_HAS_DRV(L6470)
    _L6470_DEFINE(E0);
  #endif
  #if E1_HAS_DRV(L6470)
    _L6470_DEFINE(E1);
  #endif
  #if E2_HAS_DRV(L6470)
    _L6470_DEFINE(E2);
  #endif
  #if E3_HAS_DRV(L6470)
    _L6470_DEFINE(E3);
  #endif
  #if E4_HAS_DRV(L6470)
    _L6470_DEFINE(E4);
  #endif

  #define _L6470_INIT(A) do{ \
    stepper##A.init(); \
    stepper##A.softFree(); \
    stepper##A.setMicroSteps(A##_MICROSTEPS); \
    stepper##A.setOverCurrent(A##_OVERCURRENT); \
    stepper##A.setStallCurrent(A##_STALLCURRENT); \
  } while(0)

  void L6470_init_to_defaults() {
    #if X_HAS_DRV(L6470)
      _L6470_INIT(X);
    #endif
    #if X2_HAS_DRV(L6470)
      _L6470_INIT(X2);
    #endif
    #if Y_HAS_DRV(L6470)
      _L6470_INIT(Y);
    #endif
    #if Y2_HAS_DRV(L6470)
      _L6470_INIT(Y2);
    #endif
    #if Z_HAS_DRV(L6470)
      _L6470_INIT(Z);
    #endif
    #if Z2_HAS_DRV(L6470)
      _L6470_INIT(Z2);
    #endif
    #if Z3_HAS_DRV(L6470)
      _L6470_INIT(Z3);
    #endif
    #if E0_HAS_DRV(L6470)
      _L6470_INIT(E0);
    #endif
    #if E1_HAS_DRV(L6470)
      _L6470_INIT(E1);
    #endif
    #if E2_HAS_DRV(L6470)
      _L6470_INIT(E2);
    #endif
    #if E3_HAS_DRV(L6470)
      _L6470_INIT(E3);
    #endif
    #if E4_HAS_DRV(L6470)
      _L6470_INIT(E4);
    #endif
  }

#endif // HAVE_DRV(L6470)

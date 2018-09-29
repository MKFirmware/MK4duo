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
 * tmc.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if HAS_TRINAMIC

TMC_Stepper tmc;

// Public Parameters

#if X_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperX = NULL;
#endif
#if X2_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperX2 = NULL;
#endif
#if Y_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperY = NULL;
#endif
#if Y2_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperY2 = NULL;
#endif
#if Z_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperZ = NULL;
#endif
#if Z2_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperZ2 = NULL;
#endif
#if Z3_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperZ3 = NULL;
#endif
#if E0_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperE0 = NULL;
#endif
#if E1_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperE1 = NULL;
#endif
#if E2_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperE2 = NULL;
#endif
#if E3_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperE3 = NULL;
#endif
#if E4_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperE4 = NULL;
#endif
#if E5_IS_TRINAMIC
  TMC_TYPE* TMC_Stepper::stepperE5 = NULL;
#endif

// Private Parameters
bool TMC_Stepper::report_status = false;

// Public Function
void TMC_Stepper::init() {

  #if HAVE_DRV(TMC2130)

    #if ENABLED(SOFT_SPI_TMC2130)
      #define _TMC2130_DEFINE(ST) stepper##ST = new TMC2130Stepper(ST##_ENABLE_PIN, ST##_DIR_PIN, ST##_STEP_PIN, ST##_CS_PIN, SOFT_MOSI_PIN, SOFT_MISO_PIN, SOFT_SCK_PIN)
    #else
      #define _TMC2130_DEFINE(ST) stepper##ST = new TMC2130Stepper(ST##_ENABLE_PIN, ST##_DIR_PIN, ST##_STEP_PIN, ST##_CS_PIN)
    #endif

    // Stepper objects of TMC2130 steppers used
    #if X_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(X);
      config(stepperX, X_STEALTHCHOP, X_HOMING_SENSITIVITY);
    #endif
    #if X2_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(X2);
      config(stepperX2, X2_STEALTHCHOP, X_HOMING_SENSITIVITY);
    #endif
    #if Y_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(Y);
      config(stepperY, Y_STEALTHCHOP, Y_HOMING_SENSITIVITY);
    #endif
    #if Y2_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(Y2);
      config(stepperY2, Y2_STEALTHCHOP, Y_HOMING_SENSITIVITY);
    #endif
    #if Z_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(Z);
      config(stepperZ, Z_STEALTHCHOP, Z_HOMING_SENSITIVITY);
    #endif
    #if Z2_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(Z2);
      config(stepperZ2, Z2_STEALTHCHOP, Z_HOMING_SENSITIVITY);
    #endif
    #if Z3_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(Z3);
      config(stepperZ3, Z3_STEALTHCHOP, Z_HOMING_SENSITIVITY);
    #endif
    #if E0_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(E0);
      config(stepperE0, E0_STEALTHCHOP);
    #endif
    #if E1_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(E1);
      config(stepperE1, E1_STEALTHCHOP);
    #endif
    #if E2_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(E2);
      config(stepperE2, E2_STEALTHCHOP);
    #endif
    #if E3_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(E3);
      config(stepperE3, E3_STEALTHCHOP);
    #endif
    #if E4_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(E4);
      config(stepperE4, E4_STEALTHCHOP);
    #endif
    #if E5_HAS_DRV(TMC2130)
      _TMC2130_DEFINE(E5);
      config(stepperE5, E5_STEALTHCHOP);
    #endif

    TMC_ADV();

  #elif HAVE_DRV(TMC2208)

    #define _TMC2208_DEFINE_HARDWARE(ST)  TMC2208Stepper stepper##ST(&ST##_HARDWARE_SERIAL)
    #define _TMC2208_DEFINE_SOFTWARE(ST)  SoftwareSerial ST##_HARDWARE_SERIAL = SoftwareSerial(ST##_SERIAL_RX_PIN, ST##_SERIAL_TX_PIN); \
                                          TMC2208Stepper stepper##ST(&ST##_HARDWARE_SERIAL, ST##_SERIAL_RX_PIN > -1)

    #define _TMC2208_CONFIG(ST)           config(stepper##ST, ST##_STEALTHCHOP)

    // Stepper objects of TMC2208 steppers used
    #if X_HAS_DRV(TMC2208)
      #if ENABLED(X_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(X);
      #else
        _TMC2208_DEFINE_SOFTWARE(X);
      #endif
      X_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(X);
    #endif
    #if X2_HAS_DRV(TMC2208)
      #if ENABLED(X2_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(X2);
      #else
        _TMC2208_DEFINE_SOFTWARE(X2);
      #endif
      X2_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(X2);
    #endif
    #if Y_HAS_DRV(TMC2208)
      #if ENABLED(Y_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(Y);
      #else
        _TMC2208_DEFINE_SOFTWARE(Y);
      #endif
      Y_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(Y);
    #endif
    #if Y2_HAS_DRV(TMC2208)
      #if ENABLED(Y2_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(Y2);
      #else
        _TMC2208_DEFINE_SOFTWARE(Y2);
      #endif
      Y2_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(Y2);
    #endif
    #if Z_HAS_DRV(TMC2208)
      #if ENABLED(Z_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(Z);
      #else
        _TMC2208_DEFINE_SOFTWARE(Z);
      #endif
      Z_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(Z);
    #endif
    #if Z2_HAS_DRV(TMC2208)
      #if ENABLED(Z2_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(Z2);
      #else
        _TMC2208_DEFINE_SOFTWARE(Z2);
      #endif
      Z2_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(Z2);
    #endif
    #if Z3_HAS_DRV(TMC2208)
      #if ENABLED(Z3_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(Z3);
      #else
        _TMC2208_DEFINE_SOFTWARE(Z3);
      #endif
      Z3_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(Z3);
    #endif
    #if E0_HAS_DRV(TMC2208)
      #if ENABLED(E0_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(E0);
      #else
        _TMC2208_DEFINE_SOFTWARE(E0);
      #endif
      E0_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(E0);
    #endif
    #if E1_HAS_DRV(TMC2208)
      #if ENABLED(E1_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(E1);
      #else
        _TMC2208_DEFINE_SOFTWARE(E1);
      #endif
      E1_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(E1);
    #endif
    #if E2_HAS_DRV(TMC2208)
      #if ENABLED(E2_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(E2);
      #else
        _TMC2208_DEFINE_SOFTWARE(E2);
      #endif
      E2_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(E2);
    #endif
    #if E3_HAS_DRV(TMC2208)
      #if ENABLED(E3_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(E3);
      #else
        _TMC2208_DEFINE_SOFTWARE(E3);
      #endif
      E3_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(E3);
    #endif
    #if E4_HAS_DRV(TMC2208)
      #if ENABLED(E4_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(E4);
      #else
        _TMC2208_DEFINE_SOFTWARE(E4);
      #endif
      E4_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(E4);
    #endif
    #if E5_HAS_DRV(TMC2208)
      #if ENABLED(E5_HARDWARE_SERIAL)
        _TMC2208_DEFINE_HARDWARE(E5);
      #else
        _TMC2208_DEFINE_SOFTWARE(E5);
      #endif
      E5_HARDWARE_SERIAL.begin(115200);
      _TMC2208_CONFIG(E5);
    #endif

  #endif // HAVE_DRV(TMC2208)
}

// Use internal reference voltage for current calculations. This is the default.
// Following values from Trinamic's spreadsheet with values for a NEMA17 (42BYGHW609)
// https://www.trinamic.com/products/integrated-circuits/details/tmc2130/
void TMC_Stepper::current_init_to_defaults() {

  #define TMC_CURRENT_INIT(ST)  set_current(stepper##ST, ST##_CURRENT)

  #if X_IS_TRINAMIC
    TMC_CURRENT_INIT(X);
  #endif
  #if X2_IS_TRINAMIC
    TMC_CURRENT_INIT(X2);
  #endif
  #if Y_IS_TRINAMIC
    TMC_CURRENT_INIT(Y);
  #endif
  #if Y2_IS_TRINAMIC
    TMC_CURRENT_INIT(Y2);
  #endif
  #if Z_IS_TRINAMIC
    TMC_CURRENT_INIT(Z);
  #endif
  #if Z2_IS_TRINAMIC
    TMC_CURRENT_INIT(Z2);
  #endif
  #if Z3_IS_TRINAMIC
    TMC_CURRENT_INIT(Z3);
  #endif
  #if E0_IS_TRINAMIC
    TMC_CURRENT_INIT(E0);
  #endif
  #if E1_IS_TRINAMIC
    TMC_CURRENT_INIT(E1);
  #endif
  #if E2_IS_TRINAMIC
    TMC_CURRENT_INIT(E2);
  #endif
  #if E3_IS_TRINAMIC
    TMC_CURRENT_INIT(E3);
  #endif
  #if E4_IS_TRINAMIC
    TMC_CURRENT_INIT(E4);
  #endif
  #if E5_IS_TRINAMIC
    TMC_CURRENT_INIT(E5);
  #endif
}

void TMC_Stepper::microstep_init_to_defaults() {

  #define TMC_MICROSTEP_INIT(ST)  set_microstep(stepper##ST, ST##_MICROSTEPS)

  #if X_IS_TRINAMIC
    TMC_MICROSTEP_INIT(X);
  #endif
  #if X2_IS_TRINAMIC
    TMC_MICROSTEP_INIT(X2);
  #endif
  #if Y_IS_TRINAMIC
    TMC_MICROSTEP_INIT(Y);
  #endif
  #if Y2_IS_TRINAMIC
    TMC_MICROSTEP_INIT(Y2);
  #endif
  #if Z_IS_TRINAMIC
    TMC_MICROSTEP_INIT(Z);
  #endif
  #if Z2_IS_TRINAMIC
    TMC_MICROSTEP_INIT(Z2);
  #endif
  #if Z3_IS_TRINAMIC
    TMC_MICROSTEP_INIT(Z3);
  #endif
  #if E0_IS_TRINAMIC
    TMC_MICROSTEP_INIT(E0);
  #endif
  #if E1_IS_TRINAMIC
    TMC_MICROSTEP_INIT(E1);
  #endif
  #if E2_IS_TRINAMIC
    TMC_MICROSTEP_INIT(E2);
  #endif
  #if E3_IS_TRINAMIC
    TMC_MICROSTEP_INIT(E3);
  #endif
  #if E4_IS_TRINAMIC
    TMC_MICROSTEP_INIT(E4);
  #endif
  #if E5_IS_TRINAMIC
    TMC_MICROSTEP_INIT(E5);
  #endif
}

void TMC_Stepper::restore() {
  #if X_IS_TRINAMIC
    stepperX->push();
  #endif
  #if X2_IS_TRINAMIC
    stepperX2->push();
  #endif
  #if Y_IS_TRINAMIC
    stepperY->push();
  #endif
  #if Y2_IS_TRINAMIC
    stepperY2->push();
  #endif
  #if Z_IS_TRINAMIC
    stepperZ->push();
  #endif
  #if Z2_IS_TRINAMIC
    stepperZ2->push();
  #endif
  #if Z3_IS_TRINAMIC
    stepperZ3->push();
  #endif
  #if E0_IS_TRINAMIC
    stepperE0->push();
  #endif
  #if E1_IS_TRINAMIC
    stepperE1->push();
  #endif
  #if E2_IS_TRINAMIC
    stepperE2->push();
  #endif
  #if E3_IS_TRINAMIC
    stepperE3->push();
  #endif
  #if E4_IS_TRINAMIC
    stepperE4->push();
  #endif
}

#if HAVE_DRV(TMC2130) && ENABLED(SENSORLESS_HOMING)

  void TMC_Stepper::sensorless_homing(TMC2130Stepper* st, const uint8_t index, const uint32_t coolstep_sp_min, const bool enable/*=true*/) {
    static uint32_t coolstep_speed[3],
                    stealth_max_sp[3];
    static bool     stealth_state[3];

    while(!st->stst()); // Wait for motor stand-still

    if (enable) {
      coolstep_speed[index] = st->coolstep_min_speed();
      stealth_max_sp[index] = st->stealth_max_speed();
      stealth_state[index]  = st->stealthChop();
      st->stealth_max_speed(0);                 // Upper speed limit for stealthChop
      st->stealthChop(false);                   // Turn off stealthChop
      st->coolstep_min_speed(coolstep_sp_min);  // Minimum speed for StallGuard triggering
      st->sg_filter(false);                     // Turn off StallGuard filtering
    }
    else {
      st->coolstep_min_speed(coolstep_speed[index]);
      st->stealth_max_speed(stealth_max_sp[index]);
      st->stealthChop(stealth_state[index]);
    }
    st->diag1_stall(enable ? 1 : 0);
  }

#endif // SENSORLESS_HOMING

#if HAVE_DRV(TMC2130) && ENABLED(MSLUT_CALIBRATION)

  void TMC_Stepper::reset_wave(TMC_TYPE* st) {
    SERIAL_EM(" MSLUT RESET ");
    st->lut_mslutstart(0x00F70000UL);   // 0x00F70000 16187392
    st->ms_lookup_0(0xAAAAB554UL);      // 0xAAAAB554 2863314260
    st->ms_lookup_1(0x4A9554AAUL);      // 0x4A9554AA 1251300522
    st->ms_lookup_2(0x24492929UL);      // 0x24492929 608774441
    st->ms_lookup_3(0x10104222UL);      // 0x10104222 269500962
    st->ms_lookup_4(0xFBFFFFFFUL);      // 0xFBFFFFFF 4227858431
    st->ms_lookup_5(0xB5BB777DUL);      // 0xB5BB777D 3048961917
    st->ms_lookup_6(0x49295556UL);      // 0x49295556 1227445590
    st->ms_lookup_7(0x00404222UL);      // 0x00404222 4211234
    st->lut_msutsel(0xFFFF8056UL);      // 0xFFFF8056 4294934614
  }

  void TMC_Stepper::set_fixed_wave(TMC_TYPE* st, const uint8_t i) {
    SERIAL_MSG(" MSLUT Wave: ");
    switch (i) {
      case 1:
        SERIAL_EM("SoundCorrection [X]");
        st->lut_mslutstart(0x00FC0000UL);
        st->ms_lookup_0(0xAAAAB554UL);
        st->ms_lookup_1(0x52AAAAAAUL);
        st->ms_lookup_2(0x91252529UL);
        st->ms_lookup_3(0x10208848UL);
        st->ms_lookup_4(0xBFFC0200UL);
        st->ms_lookup_5(0xB6DBBBDFUL);
        st->ms_lookup_6(0x24A55556UL);
        st->ms_lookup_7(0x00041089UL);
        st->lut_msutsel(0xFFFF9156UL);
        break;
      case 2:
        SERIAL_EM("LaserCorrection [X]");
        st->lut_mslutstart(0x00FC0000UL);
        st->ms_lookup_0(0xD52ADB54UL);
        st->ms_lookup_1(0x52AAAAAAUL);
        st->ms_lookup_2(0x91252529UL);
        st->ms_lookup_3(0x10208848UL);
        st->ms_lookup_4(0xBFFC0200UL);
        st->ms_lookup_5(0xB6DBBBDFUL);
        st->ms_lookup_6(0x24A55556UL);
        st->ms_lookup_7(0x00040089UL);
        st->lut_msutsel(0xFFFF9156UL);
        break;
      case 3:
        SERIAL_EM("SoundCorrection [E]");
        st->lut_mslutstart(0x00FC0000UL);
        st->ms_lookup_0(0x6B6DBBDEUL);
        st->ms_lookup_1(0x4AAAAAADUL);
        st->ms_lookup_2(0x444924A5UL);
        st->ms_lookup_3(0x20041044UL);
        st->ms_lookup_4(0xBEFF8000UL);
        st->ms_lookup_5(0x5ADB76F7UL);
        st->ms_lookup_6(0x894A5555UL);
        st->ms_lookup_7(0x00020844UL);
        st->lut_msutsel(0xFFFF8E56UL);
        break;
      case 4:
        SERIAL_EM("LaserCorrection [E]");
        st->lut_mslutstart(0x00FD0000UL);
        st->ms_lookup_0(0xD5BAD554UL);
        st->ms_lookup_1(0x92A4AAABUL);
        st->ms_lookup_2(0x91356529UL);
        st->ms_lookup_3(0x10248848UL);
        st->ms_lookup_4(0xBFDC2200UL);
        st->ms_lookup_5(0xB6DABBDBUL);
        st->ms_lookup_6(0xA4A85552UL);
        st->ms_lookup_7(0x00040089UL);
        st->lut_msutsel(0xFFFF9156UL);
        break;
      default:
        SERIAL_EM(" ERROR: Not a preset value. ");
    }
  }

  void TMC_Stepper::set_wave( TMC_TYPE* st, const uint8_t amp, int16_t fac1000,
                              const int8_t xoff/*=0*/, const int8_t yoff/*=10*/, const uint8_t wavetype/*=0*/,
                              const bool config/*=0*/, const uint8_t addto/*=0*/
  ) {
    // TMC2130 wave compression algorithm
    // optimized for minimal memory requirements
    // wavetype: 0 (default) pow(Sin,1+x); 1 pow(Cycle,2+x)
    NOLESS(fac1000, TMC2130_WAVE_FAC1000_MIN);
    NOMORE(fac1000, TMC2130_WAVE_FAC1000_MAX);
    SERIAL_MSG(" set_wave: ");
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
    set_mslutstart(st, 0, amp);
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
        set_mslut(st, (uint8_t)(i >> 5), reg);
      else
        reg >>= 1;
    }
    set_mslutsel(st, x[0], x[1], x[2], w[0], w[1], w[2], w[3]);
    SERIAL_MV(" X = ", x[0]);
    SERIAL_MV(" - ", x[1]);
    SERIAL_MV(" - ", x[2]);
    SERIAL_MV(" W = ", w[0]);
    SERIAL_MV(" - ", w[1]);
    SERIAL_MV(" - ", w[2]);
    SERIAL_MV(" - ", w[3]);
    SERIAL_EOL();
  }

#endif // HAVE_DRV(TMC2130) && ENABLED(MSLUT_CALIBRATION)

#if ENABLED(MONITOR_DRIVER_STATUS)

  #define HAS_HW_COMMS(ST)  ST##_HAS_DRV(TMC2130) || (ST##_HAS_DRV(TMC2208) && ENABLED(ST##_HARDWARE_SERIAL))

  /**
   * Check for over temperature or short to ground error flags.
   * Report and log warning of overtemperature condition.
   * Reduce driver current in a persistent otpw condition.
   * Keep track of otpw counter so we don't reduce current on a single instance,
   * and so we don't repeatedly report warning before the condition is cleared.
   */
  void TMC_Stepper::monitor_driver() {
    static millis_t next_cOT = 0;
    if (ELAPSED(millis(), next_cOT)) {
      next_cOT = millis() + 500;
      #if HAS_HW_COMMS(X)
        static uint8_t x_otpw_cnt = 0;
        monitor_driver(stepperX, TMC_X, x_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Y)
        static uint8_t y_otpw_cnt = 0;
        monitor_driver(stepperY, TMC_Y, y_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Z)
        static uint8_t z_otpw_cnt = 0;
        monitor_driver(stepperZ, TMC_Z, z_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(X2)
        static uint8_t x2_otpw_cnt = 0;
        monitor_driver(stepperX2, TMC_X, x2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Y2)
        static uint8_t y2_otpw_cnt = 0;
        monitor_driver(stepperY2, TMC_Y, y2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Z2)
        static uint8_t z2_otpw_cnt = 0;
        monitor_driver(stepperZ2, TMC_Z, z2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Z3)
        static uint8_t z3_otpw_cnt = 0;
        monitor_driver(stepperZ3, TMC_Z, z3_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E0)
        static uint8_t e0_otpw_cnt = 0;
        monitor_driver(stepperE0, TMC_E0, e0_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E1)
        static uint8_t e1_otpw_cnt = 0;
        monitor_driver(stepperE1, TMC_E1, e1_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E2)
        static uint8_t e2_otpw_cnt = 0;
        monitor_driver(stepperE2, TMC_E2, e2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E3)
        static uint8_t e3_otpw_cnt = 0;
        monitor_driver(stepperE3, TMC_E3, e3_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E4)
        static uint8_t e4_otpw_cnt = 0;
        monitor_driver(stepperE4, TMC_E4, e4_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E5)
        static uint8_t e5_otpw_cnt = 0;
        monitor_driver(stepperE5, TMC_E5, e5_otpw_cnt);
      #endif

      if (report_status) SERIAL_EOL();
    }
  }

#endif // ENABLED(MONITOR_DRIVER_STATUS)

#if ENABLED(TMC_DEBUG)

  /**
   * M922 report functions
   */
  void TMC_Stepper::set_report_status(const bool status) {
    if ((report_status = status))
      SERIAL_EM("axis:pwm_scale |status_response|");
  }

  void TMC_Stepper::report_all() {
    #define TMC_REPORT(LABEL, ITEM) do{ SERIAL_MSG(LABEL);  debug_loop(ITEM); }while(0)
    #define DRV_REPORT(LABEL, ITEM) do{ SERIAL_MSG(LABEL); status_loop(ITEM); }while(0)
    TMC_REPORT("\t",                 TMC_CODES);
    TMC_REPORT("Enabled\t",          TMC_ENABLED);
    TMC_REPORT("Set current",        TMC_CURRENT);
    TMC_REPORT("RMS current",        TMC_RMS_CURRENT);
    TMC_REPORT("MAX current",        TMC_MAX_CURRENT);
    TMC_REPORT("Run current",        TMC_IRUN);
    TMC_REPORT("Hold current",       TMC_IHOLD);
    TMC_REPORT("CS actual\t",        TMC_CS_ACTUAL);
    TMC_REPORT("PWM scale\t",        TMC_PWM_SCALE);
    TMC_REPORT("vsense\t",           TMC_VSENSE);
    TMC_REPORT("stealthChop",        TMC_STEALTHCHOP);
    TMC_REPORT("msteps\t",           TMC_MICROSTEPS);
    TMC_REPORT("tstep\t",            TMC_TSTEP);
    TMC_REPORT("pwm\nthreshold\t",   TMC_TPWMTHRS);
    TMC_REPORT("[mm/s]\t",           TMC_TPWMTHRS_MMS);
    TMC_REPORT("OT prewarn",         TMC_OTPW);
    TMC_REPORT("OT prewarn has\n"
               "been triggered",     TMC_OTPW_TRIGGERED);
    TMC_REPORT("off time\t",         TMC_TOFF);
    TMC_REPORT("blank time",         TMC_TBL);
    TMC_REPORT("hysteresis\n-end\t", TMC_HEND);
    TMC_REPORT("-start\t",           TMC_HSTRT);
    TMC_REPORT("Stallguard thrs",    TMC_SGT);

    DRV_REPORT("DRVSTATUS",          TMC_DRV_CODES);
    #if HAVE_DRV(TMC2130)
      DRV_REPORT("stallguard\t",     TMC_STALLGUARD);
      DRV_REPORT("sg_result\t",      TMC_SG_RESULT);
      DRV_REPORT("fsactive\t",       TMC_FSACTIVE);
    #endif
    DRV_REPORT("stst\t",             TMC_STST);
    DRV_REPORT("olb\t",              TMC_OLB);
    DRV_REPORT("ola\t",              TMC_OLA);
    DRV_REPORT("s2gb\t",             TMC_S2GB);
    DRV_REPORT("s2ga\t",             TMC_S2GA);
    DRV_REPORT("otpw\t",             TMC_DRV_OTPW);
    DRV_REPORT("ot\t",               TMC_OT);
    #if HAVE_DRV(TMC2208)
      DRV_REPORT("157C\t",           TMC_T157);
      DRV_REPORT("150C\t",           TMC_T150);
      DRV_REPORT("143C\t",           TMC_T143);
      DRV_REPORT("120C\t",           TMC_T120);
      DRV_REPORT("s2vsa\t",          TMC_S2VSA);
      DRV_REPORT("s2vsb\t",          TMC_S2VSB);
    #endif
    DRV_REPORT("Driver registers:",  TMC_DRV_STATUS_HEX);
    SERIAL_EOL();
  }

#endif // ENABLED(TMC_DEBUG)

// Private Function
#if HAVE_DRV(TMC2130)

  void TMC_Stepper::config(TMC2130Stepper* st, const bool tmc_stealthchop/*=false*/, const int8_t tmc_sgt/*=0*/) {

    while(!st->stst()); // Wait for motor stand-still

    st->begin();
    st->interpolate(INTERPOLATE);
    st->stealth_amplitude(STEALTH_AMPL);
    st->stealth_gradient(STEALTH_GRAD);
    st->stealth_autoscale(STEALTH_AUTOSCALE);
    st->stealth_freq(STEALTH_FREQ);
    st->stealthChop(tmc_stealthchop);
    st->sgt(tmc_sgt);
    st->GSTAT(); // Clear GSTAT
  }

#elif HAVE_DRV(TMC2208)

  void TMC_Stepper::config(TMC2208Stepper* st, const bool tmc_stealthchop/*=false*/) {

    st->pdn_disable(true);      // Use UART
    st->mstep_reg_select(true); // Select microsteps with UART
    st->I_scale_analog(false);
    st->blank_time(24);
    st->toff(5);
    st->intpol(INTERPOLATE);
    st->TPOWERDOWN(128);        // ~2s until driver lowers to hold current
    st->hysteresis_start(3);
    st->hysteresis_end(2);

    if (tmc_stealthchop) {
      st->pwm_lim(12);
      st->pwm_reg(8);
      st->pwm_autograd(1);
      st->pwm_autoscale(1);
      st->pwm_freq(1);
      st->pwm_grad(14);
      st->pwm_ofs(36);
      st->en_spreadCycle(false);
    }
    else
      st->en_spreadCycle(true);

    st->GSTAT(0b111); // Clear
    delay(200);
  }

#endif

void TMC_Stepper::say_axis(const TMC_AxisEnum axis) {
  static const char ext_X[] PROGMEM = "X", ext_Y[] PROGMEM = "Y", ext_Z[] PROGMEM = "Z"
    #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(X_TWO_STEPPER_DRIVERS)
      , ext_X2[] PROGMEM = "X2"
    #endif
    #if ENABLED(Y_TWO_STEPPER_DRIVERS)
      , ext_Y2[] PROGMEM = "Y2"
    #endif
    #if ENABLED(Z_THREE_STEPPER_DRIVERS)
      , ext_Z2[] PROGMEM = "Z2", ext_Z3[] PROGMEM = "Z3"
    #elif ENABLED(Z_TWO_STEPPER_DRIVERS)
      , ext_Z2[] PROGMEM = "Z2"
    #endif
    #if DRIVER_EXTRUDERS > 0
      , ext_E0[] PROGMEM = "E0"
      #if DRIVER_EXTRUDERS > 1
        , ext_E1[] PROGMEM = "E1"
        #if DRIVER_EXTRUDERS > 2
          , ext_E2[] PROGMEM = "E2"
          #if DRIVER_EXTRUDERS > 3
            , ext_E3[] PROGMEM = "E3"
            #if DRIVER_EXTRUDERS > 4
              , ext_E4[] PROGMEM = "E4"
              #if DRIVER_EXTRUDERS > 5
                , ext_E5[] PROGMEM = "E5"
              #endif // DRIVER_EXTRUDERS > 5
            #endif // DRIVER_EXTRUDERS > 4
          #endif // DRIVER_EXTRUDERS > 3
        #endif // DRIVER_EXTRUDERS > 2
      #endif // DRIVER_EXTRUDERS > 1
    #endif // DRIVER_EXTRUDERS > 0
  ;

  static const char* const tmc_axes[] PROGMEM = {
    ext_X, ext_Y, ext_Z
    #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(X_TWO_STEPPER_DRIVERS)
      , ext_X2
    #endif
    #if ENABLED(Y_TWO_STEPPER_DRIVERS)
      , ext_Y2
    #endif
    #if ENABLED(Z_THREE_STEPPER_DRIVERS)
      , ext_Z2, ext_Z3
    #elif ENABLED(Z_TWO_STEPPER_DRIVERS)
      , ext_Z2
    #endif
    #if DRIVER_EXTRUDERS > 0
      , ext_E0
      #if DRIVER_EXTRUDERS > 1
        , ext_E1
        #if DRIVER_EXTRUDERS > 2
          , ext_E2
          #if DRIVER_EXTRUDERS > 3
            , ext_E3
            #if DRIVER_EXTRUDERS > 4
              , ext_E4
              #if DRIVER_EXTRUDERS > 5
                , ext_E5
              #endif // DRIVER_EXTRUDERS > 5
            #endif // DRIVER_EXTRUDERS > 4
          #endif // DRIVER_EXTRUDERS > 3
        #endif // DRIVER_EXTRUDERS > 2
      #endif // DRIVER_EXTRUDERS > 1
    #endif // DRIVER_EXTRUDERS > 0
  };

  SERIAL_PS((char*)pgm_read_ptr(&tmc_axes[axis]));

}

void TMC_Stepper::say_current(const TMC_AxisEnum axis, const uint16_t tmc_curr) {
  say_axis(axis);
  SERIAL_EMV(" driver current: ", tmc_curr);
}

void TMC_Stepper::say_microstep(const TMC_AxisEnum axis, const uint16_t tmc_ms) {
  say_axis(axis);
  SERIAL_EMV(" driver microstep: ", tmc_ms);
}

void TMC_Stepper::say_otpw(const TMC_AxisEnum axis, const bool tmc_otpw) {
  say_axis(axis);
  SERIAL_MSG(" temperature prewarn triggered: ");
  SERIAL_PS(tmc_otpw ? PSTR("true") : PSTR("false"));
  SERIAL_EOL();
}

void TMC_Stepper::say_otpw_cleared(const TMC_AxisEnum axis) {
  say_axis(axis);
  SERIAL_EM(" prewarn flag cleared");
}

void TMC_Stepper::say_pwmthrs(const TMC_AxisEnum axis, const uint32_t tmc_thrs) {
  say_axis(axis);
  SERIAL_EMV(" stealthChop max speed: ", tmc_thrs);
}

void TMC_Stepper::say_sgt(const TMC_AxisEnum axis, const int8_t tmc_sgt) {
  say_axis(axis);
  SERIAL_EMV(" homing sensitivity: ", tmc_sgt, DEC);
}

void TMC_Stepper::say_off_time(const TMC_AxisEnum axis, const uint8_t off_time) {
  say_axis(axis);
  SERIAL_EMV(" off_time: ", off_time);
}

void TMC_Stepper::say_fast_decay_time(const TMC_AxisEnum axis, const uint8_t fast_decay_time) {
  say_axis(axis);
  SERIAL_EMV(" fast_decay_time: ", fast_decay_time);
}

void TMC_Stepper::say_blank_time(const TMC_AxisEnum axis, const uint8_t blank_time) {
  say_axis(axis);
  SERIAL_EMV(" blank_time: ", blank_time);
}

void TMC_Stepper::say_hysteresis_end(const TMC_AxisEnum axis, const int8_t hysteresis_end) {
  say_axis(axis);
  SERIAL_EMV(" hysteresis_end: ", hysteresis_end);
}

void TMC_Stepper::say_hysteresis_start(const TMC_AxisEnum axis, const uint8_t hysteresis_start) {
  say_axis(axis);
  SERIAL_EMV(" hysteresis_start: ", hysteresis_start);
}

void TMC_Stepper::say_stealth_gradient(const TMC_AxisEnum axis, const uint8_t stealth_gradient) {
  say_axis(axis);
  SERIAL_EMV(" stealth_gradient: ", stealth_gradient);
}

void TMC_Stepper::say_stealth_amplitude(const TMC_AxisEnum axis, const uint8_t stealth_amplitude) {
  say_axis(axis);
  SERIAL_EMV(" stealth_amplitude: ", stealth_amplitude);
}

void TMC_Stepper::say_stealth_freq(const TMC_AxisEnum axis, const uint8_t stealth_freq) {
  say_axis(axis);
  SERIAL_EMV(" stealth_freq: ", stealth_freq);
}

void TMC_Stepper::say_stealth_autoscale(const TMC_AxisEnum axis, const bool stealth_autoscale) {
  say_axis(axis);
  SERIAL_EMV(" stealth_autoscale: ", stealth_autoscale);
}

void TMC_Stepper::say_disable_I_comparator(const TMC_AxisEnum axis, const bool disable_I_comparator) {
  say_axis(axis);
  SERIAL_EMV(" disable_I_comparator: ", disable_I_comparator);
}

void TMC_Stepper::set_mslutstart(TMC_TYPE* st, const uint8_t start_sin, const uint8_t start_sin90) {
  const uint32_t val = uint32_t(start_sin) | (uint32_t(start_sin90) << 16);
  SERIAL_EMV(" MSLUTSTART: ", val);
  st->lut_mslutstart(val);
}

void TMC_Stepper::set_mslutsel(TMC_TYPE* st, uint8_t x1, uint8_t x2, uint8_t x3, int8_t w0, int8_t w1, int8_t w2, int8_t w3) {
  const uint32_t val =  uint32_t(w0)
                     | (uint32_t(w1) <<  2) | (uint32_t(w2) <<  4) | (uint32_t(w3) <<  6)
                     | (uint32_t(x1) <<  8) | (uint32_t(x2) << 16) | (uint32_t(x3) << 24);
  SERIAL_EMV(" MSLUTSEL: ", val);
  st->lut_msutsel(val);
}

void TMC_Stepper::set_mslut(TMC_TYPE* st, const uint8_t i, const uint32_t val) {
  SERIAL_MV(" MSLUT", i);
  SERIAL_EMV(" : ", val);
  switch (i) {
    case 0: st->ms_lookup_0(val); break;
    case 1: st->ms_lookup_1(val); break;
    case 2: st->ms_lookup_2(val); break;
    case 3: st->ms_lookup_3(val); break;
    case 4: st->ms_lookup_4(val); break;
    case 5: st->ms_lookup_5(val); break;
    case 6: st->ms_lookup_6(val); break;
    case 7: st->ms_lookup_7(val); break;
    default: break;
  }
}

#if ENABLED(MONITOR_DRIVER_STATUS)

  #if HAVE_DRV(TMC2130)
    
    TMC_driver_data TMC_Stepper::get_driver_data(TMC2130Stepper* st) {
      constexpr uint32_t OTPW_bm = 0x4000000UL;
      constexpr uint8_t OTPW_bp = 26;
      constexpr uint32_t OT_bm = 0x2000000UL;
      constexpr uint8_t OT_bp = 25;
      constexpr uint8_t DRIVER_ERROR_bm = 0x2UL;
      constexpr uint8_t DRIVER_ERROR_bp = 1;
      TMC_driver_data data;
      data.drv_status = st->DRV_STATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_error = (st->status_response & DRIVER_ERROR_bm) >> DRIVER_ERROR_bp;
      return data;
    }

  #endif

  #if HAVE_DRV(TMC2208)

    static TMC_driver_data TMC_Stepper::get_driver_data(TMC2208Stepper* st) {
      constexpr uint32_t OTPW_bm = 0B1UL;
      constexpr uint8_t OTPW_bp = 0;
      constexpr uint32_t OT_bm = 0B10UL;
      constexpr uint8_t OT_bp = 1;
      TMC_driver_data data;
      data.drv_status = st->DRV_STATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_error = st->drv_err();
      return data;
    }

    uint8_t TMC_Stepper::get_status_response(TMC2208Stepper* st) {
      uint32_t drv_status = st->DRV_STATUS();
      uint8_t gstat = st->GSTAT();
      uint8_t response = 0;
      response |= (drv_status >> (31-3)) & 0B1000;
      response |= gstat & 0B11;
      return response;
    }

  #endif

  void TMC_Stepper::monitor_driver(TMC_TYPE* st, const TMC_AxisEnum axis, uint8_t &otpw_cnt) {

    TMC_driver_data data = get_driver_data(st);

    #if ENABLED(STOP_ON_ERROR)
      if (data.is_error) {
        SERIAL_EOL();
        say_axis(axis);
        SERIAL_MSG(" driver error detected:");
        if (data.is_ot) SERIAL_EM("overtemperature");
        if (st->s2ga()) SERIAL_EM("short to ground (coil A)");
        if (st->s2gb()) SERIAL_EM("short to ground (coil B)");
        #if ENABLED(TMC_DEBUG)
          tmc_report_all();
        #endif
        printer.kill(PSTR("Driver error"));
      }
    #endif

    // Report if a warning was triggered
    if (data.is_otpw && otpw_cnt == 0) {
      char timestamp[10];
      duration_t elapsed = print_job_counter.duration();
      (void)elapsed.toDigital(timestamp, true);
      SERIAL_EOL();
      SERIAL_TXT(timestamp);
      SERIAL_MSG(": ");
      say_axis(axis);
      SERIAL_MSG(" driver overtemperature warning! (");
      SERIAL_VAL(st->getCurrent());
      SERIAL_EM("mA)");
    }

    #if CURRENT_STEP_DOWN > 0
      // Decrease current if is_otpw is true and driver is enabled and there's been more than 4 warnings
      if (data.is_otpw && st->isEnabled() && otpw_cnt > 4) {
        st->setCurrent(st->getCurrent() - CURRENT_STEP_DOWN, R_SENSE, HOLD_MULTIPLIER);
        #if ENABLED(REPORT_CURRENT_CHANGE)
          say_axis(axis);
          SERIAL_EMV(" current decreased to ", st->getCurrent());
        #endif
      }
    #endif

    if (data.is_otpw) {
      otpw_cnt++;
      st->flag_otpw = true;
    }
    else if (otpw_cnt > 0) otpw_cnt = 0;

    if (report_status) {
      const uint32_t pwm_scale = get_pwm_scale(st);
      say_axis(axis);
      SERIAL_MV(":", pwm_scale);
      SERIAL_MSG(" |0b"); SERIAL_VAL(get_status_response(st), BIN);
      SERIAL_MSG("| ");
      if (data.is_error) SERIAL_CHR('E');
      else if (data.is_ot) SERIAL_CHR('O');
      else if (data.is_otpw) SERIAL_CHR('W');
      else if (otpw_cnt > 0) SERIAL_VAL(otpw_cnt, DEC);
      else if (st->flag_otpw) SERIAL_CHR('F');
      SERIAL_CHR('\t');
    }
  }

#endif // MONITOR_DRIVER_STATUS

#if ENABLED(TMC_DEBUG)

  void TMC_Stepper::drv_status_print_hex(const TMC_AxisEnum axis, const uint32_t drv_status) {
    say_axis(axis);
    SERIAL_MSG(" = 0x");
    for (int B = 24; B >= 8; B -= 8){
      SERIAL_VAL((drv_status >> (B + 4)) & 0xF);
      SERIAL_VAL((drv_status >> B) & 0xF);
      SERIAL_CHR(':');
    }
    SERIAL_VAL((drv_status >> 4) & 0xF);
    SERIAL_VAL(drv_status & 0xF);
    SERIAL_EOL();
  }

  #if HAVE_DRV(TMC2130)

    void TMC_Stepper::status(TMC2130Stepper* st, const TMC_debug_enum i) {
      switch(i) {
        case TMC_PWM_SCALE: SERIAL_VAL(st->PWM_SCALE(), DEC); break;
        case TMC_TSTEP: SERIAL_TXT(st->TSTEP()); break;
        case TMC_SGT: SERIAL_VAL(st->sgt(), DEC); break;
        case TMC_STEALTHCHOP: SERIAL_PS(st->stealthChop() ? PSTR("true") : PSTR("false")); break;
        default: break;
      }
    }

    void TMC_Stepper::parse_drv_status(TMC2130Stepper* st, const TMC_drv_status_enum i) {
      switch(i) {
        case TMC_STALLGUARD: if (st->stallguard()) SERIAL_CHR('X'); break;
        case TMC_SG_RESULT:  SERIAL_VAL(st->sg_result(), DEC);   break;
        case TMC_FSACTIVE:   if (st->fsactive())   SERIAL_CHR('X'); break;
        default: break;
      }
    }

  #elif HAVE_DRV(TMC2208)

    void TMC_Stepper::status(TMC2208Stepper* st, const TMC_debug_enum i) {
      switch(i) {
        case TMC_TSTEP: { uint32_t data = 0; st->TSTEP(&data); SERIAL_VAL(data); break; }
        case TMC_PWM_SCALE: SERIAL_VAL(st->pwm_scale_sum(), DEC); break;
        case TMC_STEALTHCHOP: SERIAL_PS(st->stealth() ? PSTR("true") : PSTR("false")); break;
        case TMC_S2VSA: if (st->s2vsa()) SERIAL_CHR('X'); break;
        case TMC_S2VSB: if (st->s2vsb()) SERIAL_CHR('X'); break;
        default: break;
      }
    }

    void TMC_Stepper::parse_drv_status(TMC2208Stepper* st, const TMC_drv_status_enum i) {
      switch(i) {
        case TMC_T157: if (st->t157()) SERIAL_CHR('X'); break;
        case TMC_T150: if (st->t150()) SERIAL_CHR('X'); break;
        case TMC_T143: if (st->t143()) SERIAL_CHR('X'); break;
        case TMC_T120: if (st->t120()) SERIAL_CHR('X'); break;
        default: break;
      }
    }

  #endif // HAVE_DRV(TMC2208)

  void TMC_Stepper::status(TMC_TYPE* st, const TMC_AxisEnum axis, const TMC_debug_enum i, const float tmc_spmm) {
    SERIAL_CHR('\t');
    switch(i) {
      case TMC_CODES: say_axis(axis); break;
      case TMC_ENABLED: SERIAL_PS(st->isEnabled() ? PSTR("true") : PSTR("false")); break;
      case TMC_CURRENT: SERIAL_VAL(st->getCurrent()); break;
      case TMC_RMS_CURRENT: SERIAL_VAL(st->rms_current()); break;
      case TMC_MAX_CURRENT: SERIAL_VAL((float)st->rms_current() * 1.41, 0); break;
      case TMC_IRUN:
        SERIAL_VAL(st->irun(), DEC);
        SERIAL_MSG("/31");
        break;
      case TMC_IHOLD:
        SERIAL_VAL(st->ihold(), DEC);
        SERIAL_MSG("/31");
        break;
      case TMC_CS_ACTUAL:
        SERIAL_VAL(st->cs_actual(), DEC);
        SERIAL_MSG("/31");
        break;

      case TMC_VSENSE: SERIAL_PS(st->vsense() ? PSTR("1=.18") : PSTR("0=.325")); break;

      case TMC_MICROSTEPS: SERIAL_VAL(st->microsteps()); break;
      case TMC_TPWMTHRS: {
          uint32_t tpwmthrs_val = st->TPWMTHRS();
          SERIAL_VAL(tpwmthrs_val);
        }
        break;
      case TMC_TPWMTHRS_MMS: {
          uint32_t tpwmthrs_val = st->TPWMTHRS();
            if (tpwmthrs_val)
              SERIAL_VAL(thrs(st->microsteps(), tpwmthrs_val, tmc_spmm));
            else
              SERIAL_CHR('-');
          }
        break;
      case TMC_OTPW: SERIAL_PS(st->otpw() ? PSTR("true") : PSTR("false")); break;
      case TMC_OTPW_TRIGGERED: SERIAL_PS(st->getOTPW() ? PSTR("true") : PSTR("false")); break;
      case TMC_TOFF: SERIAL_VAL(st->off_time(), DEC); break;
      case TMC_TBL: SERIAL_VAL(st->blank_time(), DEC); break;
      case TMC_HEND: SERIAL_VAL(st->hysteresis_end(), DEC); break;
      case TMC_HSTRT: SERIAL_VAL(st->hysteresis_start(), DEC); break;
      default: status(st, i); break;
    }
  }

  void TMC_Stepper::parse_drv_status(TMC_TYPE* st, const TMC_AxisEnum axis, const TMC_drv_status_enum i) {
    SERIAL_CHR('\t');
    switch(i) {
      case TMC_DRV_CODES:     say_axis(axis);                    break;
      case TMC_STST:          if (st->stst())         SERIAL_CHR('X'); break;
      case TMC_OLB:           if (st->olb())          SERIAL_CHR('X'); break;
      case TMC_OLA:           if (st->ola())          SERIAL_CHR('X'); break;
      case TMC_S2GB:          if (st->s2gb())         SERIAL_CHR('X'); break;
      case TMC_S2GA:          if (st->s2ga())         SERIAL_CHR('X'); break;
      case TMC_DRV_OTPW:      if (st->otpw())         SERIAL_CHR('X'); break;
      case TMC_OT:            if (st->ot())           SERIAL_CHR('X'); break;
      case TMC_DRV_CS_ACTUAL: SERIAL_VAL(st->cs_actual(), DEC);        break;
      case TMC_DRV_STATUS_HEX:drv_status_print_hex(axis, st->DRV_STATUS()); break;
      default: parse_drv_status(st, i); break;
    }
  }

  void TMC_Stepper::debug_loop(const TMC_debug_enum i) {
    #if X_IS_TRINAMIC
      status(stepperX, TMC_X, i, mechanics.axis_steps_per_mm[X_AXIS]);
    #endif
    #if X2_IS_TRINAMIC
      status(stepperX2, TMC_X2, i, mechanics.axis_steps_per_mm[X_AXIS]);
    #endif

    #if Y_IS_TRINAMIC
      status(stepperY, TMC_Y, i, mechanics.axis_steps_per_mm[Y_AXIS]);
    #endif
    #if Y2_IS_TRINAMIC
      status(stepperY2, TMC_Y2, i, mechanics.axis_steps_per_mm[Y_AXIS]);
    #endif

    #if Z_IS_TRINAMIC
      status(stepperZ, TMC_Z, i, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if Z2_IS_TRINAMIC
      status(stepperZ2, TMC_Z2, i, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if Z3_IS_TRINAMIC
      status(stepperZ3, TMC_Z3, i, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif

    #if E0_IS_TRINAMIC
      status(stepperE0, TMC_E0, i, mechanics.axis_steps_per_mm[E_AXIS]);
    #endif
    #if E1_IS_TRINAMIC
      status(stepperE1, TMC_E1, i, mechanics.axis_steps_per_mm[E_AXIS+1]);
    #endif
    #if E2_IS_TRINAMIC
      status(stepperE2, TMC_E2, i, mechanics.axis_steps_per_mm[E_AXIS+2]);
    #endif
    #if E3_IS_TRINAMIC
      status(stepperE3, TMC_E3, i, mechanics.axis_steps_per_mm[E_AXIS+3]);
    #endif
    #if E4_IS_TRINAMIC
      status(stepperE4, TMC_E4, i, mechanics.axis_steps_per_mm[E_AXIS+4]);
    #endif
    #if E5_IS_TRINAMIC
      status(stepperE5, TMC_E5, i, mechanics.axis_steps_per_mm[E_AXIS+5]);
    #endif

    SERIAL_EOL();
  }

  void TMC_Stepper::status_loop(const TMC_drv_status_enum i) {
    #if X_IS_TRINAMIC
      parse_drv_status(stepperX, TMC_X, i);
    #endif
    #if X2_IS_TRINAMIC
      parse_drv_status(stepperX2, TMC_X2, i);
    #endif

    #if Y_IS_TRINAMIC
      parse_drv_status(stepperY, TMC_Y, i);
    #endif
    #if Y2_IS_TRINAMIC
      parse_drv_status(stepperY2, TMC_Y2, i);
    #endif

    #if Z_IS_TRINAMIC
      parse_drv_status(stepperZ, TMC_Z, i);
    #endif
    #if Z2_IS_TRINAMIC
      parse_drv_status(stepperZ2, TMC_Z2, i);
    #endif
    #if Z3_IS_TRINAMIC
      parse_drv_status(stepperZ3, TMC_Z3, i);
    #endif

    #if E0_IS_TRINAMIC
      parse_drv_status(stepperE0, TMC_E0, i);
    #endif
    #if E1_IS_TRINAMIC
      parse_drv_status(stepperE1, TMC_E1, i);
    #endif
    #if E2_IS_TRINAMIC
      parse_drv_status(stepperE2, TMC_E2, i);
    #endif
    #if E3_IS_TRINAMIC
      parse_drv_status(stepperE3, TMC_E3, i);
    #endif
    #if E4_IS_TRINAMIC
      parse_drv_status(stepperE4, TMC_E4, i);
    #endif
    #if E5_IS_TRINAMIC
      parse_drv_status(stepperE5, TMC_E5, i);
    #endif

    SERIAL_EOL();
  }

#endif // TMC_DEBUG

#endif // HAS_TRINAMIC
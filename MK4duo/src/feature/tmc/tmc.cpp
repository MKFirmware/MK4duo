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

#if HAVE_DRV(TMC2208)
  #include <HardwareSerial.h>
#endif

#if AXIS_HAS_TMC(X)
  MKTMC* stepperX = NULL;
#endif
#if AXIS_HAS_TMC(X2)
  MKTMC* stepperX2 = NULL;
#endif
#if AXIS_HAS_TMC(Y)
  MKTMC* stepperY = NULL;
#endif
#if AXIS_HAS_TMC(Y2)
  MKTMC* stepperY2 = NULL;
#endif
#if AXIS_HAS_TMC(Z)
  MKTMC* stepperZ = NULL;
#endif
#if AXIS_HAS_TMC(Z2)
  MKTMC* stepperZ2 = NULL;
#endif
#if AXIS_HAS_TMC(Z3)
  MKTMC* stepperZ3 = NULL;
#endif
#if AXIS_HAS_TMC(E0)
  MKTMC* stepperE0 = NULL;
#endif
#if AXIS_HAS_TMC(E1)
  MKTMC* stepperE1 = NULL;
#endif
#if AXIS_HAS_TMC(E2)
  MKTMC* stepperE2 = NULL;
#endif
#if AXIS_HAS_TMC(E3)
  MKTMC* stepperE3 = NULL;
#endif
#if AXIS_HAS_TMC(E4)
  MKTMC* stepperE4 = NULL;
#endif
#if AXIS_HAS_TMC(E5)
  MKTMC* stepperE5 = NULL;
#endif

TMC_Stepper tmc;

/** Public Parameters */

/** Private Parameters */
bool TMC_Stepper::report_status = false;

/** Public Function */
void TMC_Stepper::init() {

  #if TMC_HAS_SPI
    init_cs_pins();
  #endif

  #if HAVE_DRV(TMC2660)

    #if ENABLED(SOFT_SPI_TMC2660)
      #define _TMC2660_DEFINE(ST, L)  stepper##ST = new MKTMC(L, ST##_CS_PIN, R_SENSE, TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK)
      #define TMC2660_DEFINE(ST)      _TMC2660_DEFINE(ST, TMC_##ST##_LABEL)
    #else
      #define _TMC2660_DEFINE(ST, L)  stepper##ST = new MKTMC(L, ST##_CS_PIN, R_SENSE)
      #define TMC2660_DEFINE(ST)      _TMC2660_DEFINE(ST, TMC_##ST##_LABEL)
    #endif

    #if DISABLED(SOFT_SPI_TMC2130)
      SPI.begin();
    #endif

    // Stepper objects of TMC2660 steppers used
    #if X_HAS_DRV(TMC2660)
      TMC2660_DEFINE(X);
      config(stepperX, X_STALL_SENSITIVITY);
    #endif
    #if X2_HAS_DRV(TMC2660)
      TMC2660_DEFINE(X2);
      config(stepperX2, X_STALL_SENSITIVITY);
    #endif
    #if Y_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Y);
      config(stepperY, Y_STALL_SENSITIVITY);
    #endif
    #if Y2_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Y2);
      config(stepperY2, Y_STALL_SENSITIVITY);
    #endif
    #if Z_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Z);
      config(stepperZ, Z_STALL_SENSITIVITY);
    #endif
    #if Z2_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Z2);
      config(stepperZ2, Z_STALL_SENSITIVITY);
    #endif
    #if Z3_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Z3);
      config(stepperZ3, Z_STALL_SENSITIVITY);
    #endif
    #if E0_HAS_DRV(TMC2660)
      TMC2660_DEFINE(E0);
      config(stepperE0);
    #endif
    #if E1_HAS_DRV(TMC2660)
      TMC2660_DEFINE(E1);
      config(stepperE1);
    #endif
    #if E2_HAS_DRV(TMC2660)
      TMC2660_DEFINE(E2);
      config(stepperE2);
    #endif
    #if E3_HAS_DRV(TMC2660)
      TMC2660_DEFINE(E3);
      config(stepperE3);
    #endif
    #if E4_HAS_DRV(TMC2660)
      TMC2660_DEFINE(E4);
      config(stepperE4);
    #endif
    #if E5_HAS_DRV(TMC2660)
      TMC2660_DEFINE(E5);
      config(stepperE5);
    #endif

    TMC_ADV();

  #elif HAVE_DRV(TMC2130)

    #if ENABLED(SOFT_SPI_TMC2130)
      #define _TMC2130_DEFINE(ST, L)  stepper##ST = new MKTMC(L, ST##_CS_PIN, R_SENSE, SOFT_MOSI_PIN, SOFT_MISO_PIN, SOFT_SCK_PIN)
      #define TMC2130_DEFINE(ST)      _TMC2130_DEFINE(ST, TMC_##ST##_LABEL)
    #else
      #define _TMC2130_DEFINE(ST, L)  stepper##ST = new MKTMC(L, ST##_CS_PIN, R_SENSE)
      #define TMC2130_DEFINE(ST)      _TMC2130_DEFINE(ST, TMC_##ST##_LABEL)
    #endif

    #if DISABLED(SOFT_SPI_TMC2130)
      SPI.begin();
    #endif

    // Stepper objects of TMC2130 steppers used
    #if X_HAS_DRV(TMC2130)
      TMC2130_DEFINE(X);
      config(stepperX, X_STEALTHCHOP, X_STALL_SENSITIVITY);
    #endif
    #if X2_HAS_DRV(TMC2130)
      TMC2130_DEFINE(X2);
      config(stepperX2, X_STEALTHCHOP, X_STALL_SENSITIVITY);
    #endif
    #if Y_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Y);
      config(stepperY, Y_STEALTHCHOP, Y_STALL_SENSITIVITY);
    #endif
    #if Y2_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Y2);
      config(stepperY2, Y_STEALTHCHOP, Y_STALL_SENSITIVITY);
    #endif
    #if Z_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Z);
      config(stepperZ, Z_STEALTHCHOP, Z_STALL_SENSITIVITY);
    #endif
    #if Z2_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Z2);
      config(stepperZ2, Z_STEALTHCHOP, Z_STALL_SENSITIVITY);
    #endif
    #if Z3_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Z3);
      config(stepperZ3, Z_STEALTHCHOP, Z_STALL_SENSITIVITY);
    #endif
    #if E0_HAS_DRV(TMC2130)
      TMC2130_DEFINE(E0);
      config(stepperE0, E0_STEALTHCHOP);
    #endif
    #if E1_HAS_DRV(TMC2130)
      TMC2130_DEFINE(E1);
      config(stepperE1, E1_STEALTHCHOP);
    #endif
    #if E2_HAS_DRV(TMC2130)
      TMC2130_DEFINE(E2);
      config(stepperE2, E2_STEALTHCHOP);
    #endif
    #if E3_HAS_DRV(TMC2130)
      TMC2130_DEFINE(E3);
      config(stepperE3, E3_STEALTHCHOP);
    #endif
    #if E4_HAS_DRV(TMC2130)
      TMC2130_DEFINE(E4);
      config(stepperE4, E4_STEALTHCHOP);
    #endif
    #if E5_HAS_DRV(TMC2130)
      TMC2130_DEFINE(E5);
      config(stepperE5, E5_STEALTHCHOP);
    #endif

    TMC_ADV();

  #elif HAVE_DRV(TMC2208)

    #define _TMC2208_DEFINE_HARDWARE(ST, L) stepper##ST = new MKTMC(L, ST##_HARDWARE_SERIAL, R_SENSE)
    #define TMC2208_DEFINE_HARDWARE(ST)     _TMC2208_DEFINE_HARDWARE(ST, TMC_##ST##_LABEL)

    #define _TMC2208_DEFINE_SOFTWARE(ST, L) stepper##ST = new MKTMC(L, ST##_SERIAL_RX_PIN, ST##_SERIAL_TX_PIN, R_SENSE, ST##_SERIAL_RX_PIN > -1)
    #define TMC2208_DEFINE_SOFTWARE(ST)     _TMC2208_DEFINE_SOFTWARE(ST, TMC_##ST##_LABEL)

    // Stepper objects of TMC2208 steppers used
    #if X_HAS_DRV(TMC2208)
      #if ENABLED(X_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(X);
        X_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(X);
        stepperX->beginSerial(115200);
      #endif
      config(stepperX, X_STEALTHCHOP);
    #endif
    #if X2_HAS_DRV(TMC2208)
      #if ENABLED(X2_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(X2);
        X2_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(X2);
        stepperX2->beginSerial(115200);
      #endif
      config(stepperX2, X_STEALTHCHOP);
    #endif
    #if Y_HAS_DRV(TMC2208)
      #if ENABLED(Y_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(Y);
        Y_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(Y);
        stepperY->beginSerial(115200);
      #endif
      config(stepperY, Y_STEALTHCHOP);
    #endif
    #if Y2_HAS_DRV(TMC2208)
      #if ENABLED(Y2_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(Y2);
        Y2_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(Y2);
        stepperY2->beginSerial(115200);
      #endif
      config(stepperY2, Y_STEALTHCHOP);
    #endif
    #if Z_HAS_DRV(TMC2208)
      #if ENABLED(Z_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(Z);
        Z_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(Z);
        stepperZ->beginSerial(115200);
      #endif
      config(stepperZ, Z_STEALTHCHOP);
    #endif
    #if Z2_HAS_DRV(TMC2208)
      #if ENABLED(Z2_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(Z2);
        Z2_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(Z2);
        stepperZ2->beginSerial(115200);
      #endif
      config(stepperZ2, Z_STEALTHCHOP);
    #endif
    #if Z3_HAS_DRV(TMC2208)
      #if ENABLED(Z3_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(Z3);
        Z3_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(Z3);
        stepperZ3->beginSerial(115200);
      #endif
      config(stepperZ3, Z_STEALTHCHOP);
    #endif
    #if E0_HAS_DRV(TMC2208)
      #if ENABLED(E0_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(E0);
        E0_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(E0);
        stepperE0->beginSerial(115200);
      #endif
      config(stepperE0, E0_STEALTHCHOP);
    #endif
    #if E1_HAS_DRV(TMC2208)
      #if ENABLED(E1_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(E1);
        E1_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(E1);
        stepperE1->beginSerial(115200);
      #endif
      config(stepperE1, E1_STEALTHCHOP);
    #endif
    #if E2_HAS_DRV(TMC2208)
      #if ENABLED(E2_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(E2);
        E2_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(E2);
        stepperE2->beginSerial(115200);
      #endif
      config(stepperE2, E2_STEALTHCHOP);
    #endif
    #if E3_HAS_DRV(TMC2208)
      #if ENABLED(E3_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(E3);
        E3_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(E3);
        stepperE3->beginSerial(115200);
      #endif
      config(stepperE3, E3_STEALTHCHOP);
    #endif
    #if E4_HAS_DRV(TMC2208)
      #if ENABLED(E4_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(E4);
        E4_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(E4);
        stepperE4->beginSerial(115200);
      #endif
      config(stepperE4, E4_STEALTHCHOP);
    #endif
    #if E5_HAS_DRV(TMC2208)
      #if ENABLED(E5_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(E5);
        E5_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(E5);
        stepperE5->beginSerial(115200);
      #endif
      config(stepperE5, E5_STEALTHCHOP);
    #endif

  #endif // HAVE_DRV(TMC2208)

}

// Use internal reference voltage for current calculations. This is the default.
// Following values from Trinamic's spreadsheet with values for a NEMA17 (42BYGHW609)
// https://www.trinamic.com/products/integrated-circuits/details/tmc2130/
void TMC_Stepper::current_init_to_defaults() {
  constexpr uint16_t  tmc_stepper_current[TMC_AXIS]   = { X_CURRENT, Y_CURRENT, Z_CURRENT, X_CURRENT, Y_CURRENT, Z_CURRENT, Z_CURRENT,
                                                          E0_CURRENT, E1_CURRENT, E2_CURRENT, E3_CURRENT, E4_CURRENT, E5_CURRENT };

  LOOP_TMC() {
    MKTMC* st = tmc.driver_by_index(t);
    if (st) st->rms_current(tmc_stepper_current[t]);
  }
}

void TMC_Stepper::microstep_init_to_defaults() {
  constexpr uint16_t tmc_stepper_microstep[TMC_AXIS]  = { X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS, X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS, Z_MICROSTEPS,
                                                          E0_MICROSTEPS, E1_MICROSTEPS, E2_MICROSTEPS, E3_MICROSTEPS, E4_MICROSTEPS, E5_MICROSTEPS };

  LOOP_TMC() {
    MKTMC* st = tmc.driver_by_index(t);
    if (st) st->microsteps(tmc_stepper_microstep[t]);
  }
}

void TMC_Stepper::hybrid_threshold_init_to_defaults() {
  constexpr uint32_t  tmc_hybrid_threshold[TMC_AXIS]  = { X_HYBRID_THRESHOLD, Y_HYBRID_THRESHOLD, Z_HYBRID_THRESHOLD,
                                                          X_HYBRID_THRESHOLD, Y_HYBRID_THRESHOLD, Z_HYBRID_THRESHOLD, Z_HYBRID_THRESHOLD,
                                                          E0_HYBRID_THRESHOLD, E1_HYBRID_THRESHOLD, E2_HYBRID_THRESHOLD,
                                                          E3_HYBRID_THRESHOLD, E4_HYBRID_THRESHOLD, E5_HYBRID_THRESHOLD };

  LOOP_TMC() {
    MKTMC* st = tmc.driver_by_index(t);
    if (st)
      set_pwmthrs(st, tmc_hybrid_threshold[t], mechanics.data.axis_steps_per_mm[st->id]);
  }
}

void TMC_Stepper::restore() {
  LOOP_TMC() {
    MKTMC* st = tmc.driver_by_index(t);
    if (st) st->push();
  }
}

void TMC_Stepper::test_connection(const bool test_x, const bool test_y, const bool test_z, const bool test_e) {
  uint8_t axis_connection = 0;

  if (test_x) {
    #if AXIS_HAS_TMC(X)
      axis_connection += test_connection(stepperX);
    #endif
    #if AXIS_HAS_TMC(X2)
      axis_connection += test_connection(stepperX2);
    #endif
  }

  if (test_y) {
    #if AXIS_HAS_TMC(Y)
      axis_connection += test_connection(stepperY);
    #endif
    #if AXIS_HAS_TMC(Y2)
      axis_connection += test_connection(stepperY2);
    #endif
  }

  if (test_z) {
    #if AXIS_HAS_TMC(Z)
      axis_connection += test_connection(stepperZ);
    #endif
    #if AXIS_HAS_TMC(Z2)
      axis_connection += test_connection(stepperZ2);
    #endif
    #if AXIS_HAS_TMC(Z3)
      axis_connection += test_connection(stepperZ3);
    #endif
  }

  if (test_e) {
    #if AXIS_HAS_TMC(E0)
      axis_connection += test_connection(stepperE0);
    #endif
    #if AXIS_HAS_TMC(E1)
      axis_connection += test_connection(stepperE1);
    #endif
    #if AXIS_HAS_TMC(E2)
      axis_connection += test_connection(stepperE2);
    #endif
    #if AXIS_HAS_TMC(E3)
      axis_connection += test_connection(stepperE3);
    #endif
    #if AXIS_HAS_TMC(E4)
      axis_connection += test_connection(stepperE4);
    #endif
    #if AXIS_HAS_TMC(E5)
      axis_connection += test_connection(stepperE5);
    #endif
  }

  if (axis_connection) lcdui.set_status_P(PSTR("TMC CONNECTION ERROR"));
}

#if ENABLED(MONITOR_DRIVER_STATUS)

  #define HAS_HW_COMMS(ST)  ST##_HAS_DRV(TMC2130) || ST##_HAS_DRV(TMC2660) || (ST##_HAS_DRV(TMC2208) && ENABLED(ST##_HARDWARE_SERIAL))

  void TMC_Stepper::monitor_driver() {
    static millis_t next_poll = 0;
    const millis_t ms = millis();
    if (ELAPSED(ms, next_poll)) {
      next_poll = ms + 1000UL;
      LOOP_TMC() {
        MKTMC* st = tmc.driver_by_index(t);
        if (st) monitor_driver(st);
      }

      #if ENABLED(TMC_DEBUG)
        if (report_status) SERIAL_EOL();
      #endif

    }
  }

#endif // ENABLED(MONITOR_DRIVER_STATUS)

#if HAS_SENSORLESS

  bool TMC_Stepper::enable_stallguard(MKTMC* st) {
    bool old_stealthChop = st->en_pwm_mode();

    st->TCOOLTHRS(0xFFFFF);
    st->en_pwm_mode(false);
    st->diag1_stall(true);

    return old_stealthChop;
  }

  void TMC_Stepper::disable_stallguard(MKTMC* st, const bool enable) {
    st->TCOOLTHRS(0);
    st->en_pwm_mode(enable);
    st->diag1_stall(false);
  }

#endif

#if ENABLED(TMC_DEBUG)

  /**
   * M922 report functions
   */
  #if ENABLED(MONITOR_DRIVER_STATUS)
    void TMC_Stepper::set_report_status(const bool status) {
      if ((report_status = status))
        SERIAL_EM("axis:pwm_scale |status_response|");
    }
  #endif

  void TMC_Stepper::report_all(bool print_x, const bool print_y, const bool print_z, const bool print_e) {
    #define TMC_REPORT(LABEL, ITEM) do{ SERIAL_SM(ECHO, LABEL);  debug_loop(ITEM, print_x, print_y, print_z, print_e); }while(0)
    #define DRV_REPORT(LABEL, ITEM) do{ SERIAL_SM(ECHO, LABEL); status_loop(ITEM, print_x, print_y, print_z, print_e); }while(0)
    TMC_REPORT("\t",                  TMC_CODES);
    TMC_REPORT("Enabled\t",           TMC_ENABLED);
    TMC_REPORT("Set current",         TMC_CURRENT);
    TMC_REPORT("RMS current",         TMC_RMS_CURRENT);
    TMC_REPORT("MAX current",         TMC_MAX_CURRENT);
    TMC_REPORT("Run current",         TMC_IRUN);
    TMC_REPORT("Hold current",        TMC_IHOLD);
    #if HAVE_DRV(TMC2160) || HAVE_DRV(TMC5160)
      TMC_REPORT("Global scaler",     TMC_GLOBAL_SCALER);
    #endif
    TMC_REPORT("CS actual\t",         TMC_CS_ACTUAL);
    TMC_REPORT("PWM scale",           TMC_PWM_SCALE);
    TMC_REPORT("vsense\t",            TMC_VSENSE);
    TMC_REPORT("stealthChop",         TMC_STEALTHCHOP);
    TMC_REPORT("msteps\t",            TMC_MICROSTEPS);
    TMC_REPORT("tstep\t",             TMC_TSTEP);
    TMC_REPORT("pwm\threshold\t",     TMC_TPWMTHRS);
    TMC_REPORT("[mm/s]\t",            TMC_TPWMTHRS_MMS);
    TMC_REPORT("OT prewarn",          TMC_OTPW);
    #if ENABLED(MONITOR_DRIVER_STATUS)
      TMC_REPORT("OT prewarn has\n"
                 "been triggered",    TMC_OTPW_TRIGGERED);
    #endif
    TMC_REPORT("off time\t",          TMC_TOFF);
    TMC_REPORT("blank time",          TMC_TBL);
    TMC_REPORT("hysteresis\n-end\t",  TMC_HEND);
    TMC_REPORT("-start\t",            TMC_HSTRT);
    TMC_REPORT("Stallguard thrs",     TMC_SGT);

    DRV_REPORT("DRVSTATUS\t",         TMC_DRV_CODES);
    #if HAVE_DRV(TMC2130) || HAVE_DRV(TMC2160) || HAVE_DRV(TMC5130) || HAVE_DRV(TMC5160)
      DRV_REPORT("stallguard\t",      TMC_STALLGUARD);
      DRV_REPORT("sg_result\t",       TMC_SG_RESULT);
      DRV_REPORT("fsactive\t",        TMC_FSACTIVE);
    #endif
    DRV_REPORT("stst\t",              TMC_STST);
    DRV_REPORT("olb\t",               TMC_OLB);
    DRV_REPORT("ola\t",               TMC_OLA);
    DRV_REPORT("s2gb\t",              TMC_S2GB);
    DRV_REPORT("s2ga\t",              TMC_S2GA);
    DRV_REPORT("otpw\t",              TMC_DRV_OTPW);
    DRV_REPORT("ot\t",                TMC_OT);
    #if HAVE_DRV(TMC2208)
      DRV_REPORT("157C\t",            TMC_T157);
      DRV_REPORT("150C\t",            TMC_T150);
      DRV_REPORT("143C\t",            TMC_T143);
      DRV_REPORT("120C\t",            TMC_T120);
      DRV_REPORT("s2vsa\t",           TMC_S2VSA);
      DRV_REPORT("s2vsb\t",           TMC_S2VSB);
    #endif
    DRV_REPORT("Driver registers:",   TMC_DRV_STATUS_HEX);
    SERIAL_EOL();
  }

  void TMC_Stepper::get_registers(bool print_x, bool print_y, bool print_z, bool print_e) {
    #define _TMC_GET_REG(LABEL, ITEM) do{ SERIAL_MSG(LABEL); get_registers(ITEM, print_x, print_y, print_z, print_e); }while(0)
    #define TMC_GET_REG(NAME, TABS)   _TMC_GET_REG(STRINGIFY(NAME) TABS, TMC_GET_##NAME)
    _TMC_GET_REG("\t", TMC_AXIS_CODES);
    TMC_GET_REG(GCONF, "\t\t");
    TMC_GET_REG(IHOLD_IRUN, "\t");
    TMC_GET_REG(GSTAT, "\t\t");
    TMC_GET_REG(IOIN, "\t\t");
    TMC_GET_REG(TPOWERDOWN, "\t");
    TMC_GET_REG(TSTEP, "\t\t");
    TMC_GET_REG(TPWMTHRS, "\t");
    TMC_GET_REG(TCOOLTHRS, "\t");
    TMC_GET_REG(THIGH, "\t\t");
    TMC_GET_REG(CHOPCONF, "\t");
    TMC_GET_REG(COOLCONF, "\t");
    TMC_GET_REG(PWMCONF, "\t");
    TMC_GET_REG(PWM_SCALE, "\t");
    TMC_GET_REG(DRV_STATUS, "\t");
  }

#endif // ENABLED(TMC_DEBUG)

#if DISABLED(DISABLE_M503)

  void TMC_Stepper::print_M350() {
    SERIAL_LM(CFG, "Stepper driver microsteps");
    #if AXIS_HAS_TMC(X) || AXIS_HAS_TMC(Y) || AXIS_HAS_TMC(Z)
      SERIAL_SM(CFG, "  M350");
      #if AXIS_HAS_TMC(X)
        SERIAL_MV(" X", stepperX->microsteps());
      #endif
      #if AXIS_HAS_TMC(Y)
        SERIAL_MV(" Y", stepperY->microsteps());
      #endif
      #if AXIS_HAS_TMC(Z)
        SERIAL_MV(" Z", stepperZ->microsteps());
      #endif
      SERIAL_EOL();
    #endif
    #if AXIS_HAS_TMC(E0)
      SERIAL_LMV(CFG, "  M350 T0 E", stepperE0->microsteps());
    #endif
    #if AXIS_HAS_TMC(E1)
      SERIAL_LMV(CFG, "  M350 T1 E", stepperE1->microsteps());
    #endif
    #if AXIS_HAS_TMC(E2)
      SERIAL_LMV(CFG, "  M350 T2 E", stepperE2->microsteps());
    #endif
    #if AXIS_HAS_TMC(E3)
      SERIAL_LMV(CFG, "  M350 T3 E", stepperE3->microsteps());
    #endif
    #if AXIS_HAS_TMC(E4)
      SERIAL_LMV(CFG, "  M350 T4 E", stepperE4->microsteps());
    #endif
    #if AXIS_HAS_TMC(E5)
      SERIAL_LMV(CFG, "  M350 T5 E", stepperE5->microsteps());
    #endif
  }

  void TMC_Stepper::print_M906() {
    SERIAL_LM(CFG, "Stepper driver current (mA)");
    #if AXIS_HAS_TMC(X) || AXIS_HAS_TMC(Y) || AXIS_HAS_TMC(Z)
      SERIAL_SM(CFG, "  M906");
      #if AXIS_HAS_TMC(X)
        SERIAL_MV(" X", stepperX->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(Y)
        SERIAL_MV(" Y", stepperY->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(Z)
        SERIAL_MV(" Z", stepperZ->getMilliamps());
      #endif
      SERIAL_EOL();
    #endif
    #if AXIS_HAS_TMC(E0)
      SERIAL_LMV(CFG, "  M906 T0 E", stepperE0->getMilliamps());
    #endif
    #if AXIS_HAS_TMC(E1)
      SERIAL_LMV(CFG, "  M906 T1 E", stepperE1->getMilliamps());
    #endif
    #if AXIS_HAS_TMC(E2)
      SERIAL_LMV(CFG, "  M906 T2 E", stepperE2->getMilliamps());
    #endif
    #if AXIS_HAS_TMC(E3)
      SERIAL_LMV(CFG, "  M906 T3 E", stepperE3->getMilliamps());
    #endif
    #if AXIS_HAS_TMC(E4)
      SERIAL_LMV(CFG, "  M906 T4 E", stepperE4->getMilliamps());
    #endif
    #if AXIS_HAS_TMC(E5)
      SERIAL_LMV(CFG, "  M906 T5 E", stepperE5->getMilliamps());
    #endif
  }

  void TMC_Stepper::print_M913() {

    #if ENABLED(HYBRID_THRESHOLD)

      #define TMC_GET_PWMTHRS(ST) tmc_thrs(stepper##ST->microsteps(), stepper##ST->TPWMTHRS(), mechanics.data.axis_steps_per_mm[ST##_AXIS])

      SERIAL_LM(CFG, "Stepper driver Hybrid Threshold");
      #if AXIS_HAS_TMC(X) || AXIS_HAS_TMC(Y) || AXIS_HAS_TMC(Z)
        SERIAL_SM(CFG, "  M913");
        #if AXIS_HAS_TMC(X)
          SERIAL_MV(" X", TMC_GET_PWMTHRS(X));
        #endif
        #if AXIS_HAS_TMC(Y)
          SERIAL_MV(" Y", TMC_GET_PWMTHRS(Y));
        #endif
        #if AXIS_HAS_TMC(Z)
          SERIAL_MV(" Z", TMC_GET_PWMTHRS(Z));
        #endif
        SERIAL_EOL();
      #endif
      #if AXIS_HAS_TMC(E0)
        SERIAL_LMV(CFG, "  M913 T0 E", TMC_GET_PWMTHRS(E0));
      #endif
      #if AXIS_HAS_TMC(E1)
        SERIAL_LMV(CFG, "  M913 T1 E", TMC_GET_PWMTHRS(E1));
      #endif
      #if AXIS_HAS_TMC(E2)
        SERIAL_LMV(CFG, "  M913 T2 E", TMC_GET_PWMTHRS(E2));
      #endif
      #if AXIS_HAS_TMC(E3)
        SERIAL_LMV(CFG, "  M913 T3 E", TMC_GET_PWMTHRS(E3));
      #endif
      #if AXIS_HAS_TMC(E4)
        SERIAL_LMV(CFG, "  M913 T4 E", TMC_GET_PWMTHRS(E4));
      #endif
      #if AXIS_HAS_TMC(E5)
        SERIAL_LMV(CFG, "  M913 T5 E", TMC_GET_PWMTHRS(E5));
      #endif

    #endif // HYBRID_THRESHOLD

  }

  void TMC_Stepper::print_M914() {
    #if HAS_SENSORLESS
      SERIAL_LM(CFG, "Stepper driver StallGuard threshold:");
      #if X_HAS_SENSORLESS || Y_HAS_SENSORLESS || Z_HAS_SENSORLESS
        SERIAL_SM(CFG, "  M914");
        #if X_HAS_SENSORLESS
          SERIAL_MV(" X", stepperX->sgt());
        #endif
        #if Y_HAS_SENSORLESS
          SERIAL_MV(" Y", stepperY->sgt());
        #endif
        #if Z_HAS_SENSORLESS
          SERIAL_MV(" Z", stepperZ->sgt());
        #endif
        SERIAL_EOL();
      #endif
    #endif // HAS_SENSORLESS
  }

  void TMC_Stepper::print_M940() {
    #if TMC_HAS_STEALTHCHOP
      SERIAL_LM(CFG, "Stepper driver StealthChop:");
      SERIAL_SM(CFG, "  M940");
      #if AXIS_HAS_STEALTHCHOP(X)
        SERIAL_MV(" X", stepperX->get_stealthChop_status());
      #endif
      #if AXIS_HAS_STEALTHCHOP(Y)
        SERIAL_MV(" Y", stepperY->get_stealthChop_status());
      #endif
      #if AXIS_HAS_STEALTHCHOP(Z)
        SERIAL_MV(" Z", stepperZ->get_stealthChop_status());
      #endif
      #if AXIS_HAS_STEALTHCHOP(E0)
        SERIAL_MV(" E", stepperE0->get_stealthChop_status());
      #endif
      SERIAL_EOL();
    #endif // HAS_STEALTHCHOP
  }

#endif // DISABLED(DISABLE_M503)

MKTMC* TMC_Stepper::driver_by_index(const uint8_t index) {

  switch(index) {
    #if AXIS_HAS_TMC(X)
      case 0: return stepperX; break;
    #endif
    #if AXIS_HAS_TMC(Y)
      case 1: return stepperY; break;
    #endif
    #if AXIS_HAS_TMC(Z)
      case 2: return stepperZ; break;
    #endif
    #if AXIS_HAS_TMC(X2)
      case 3: return stepperX2; break;
    #endif
    #if AXIS_HAS_TMC(Y2)
      case 4: return stepperY2; break;
    #endif
    #if AXIS_HAS_TMC(Z2)
      case 5: return stepperZ2; break;
    #endif
    #if AXIS_HAS_TMC(Z3)
      case 6: return stepperZ3; break;
    #endif
    #if AXIS_HAS_TMC(E0)
      case 7: return stepperE0; break;
    #endif
    #if AXIS_HAS_TMC(E1)
      case 8: return stepperE1; break;
    #endif
    #if AXIS_HAS_TMC(E2)
      case 9: return stepperE2; break;
    #endif
    #if AXIS_HAS_TMC(E3)
      case 10: return stepperE3; break;
    #endif
    #if AXIS_HAS_TMC(E4)
      case 11: return stepperE4; break;
    #endif
    #if AXIS_HAS_TMC(E5)
      case 12: return stepperE5; break;
    #endif
    default: return NULL; break;
  }

}

/** Private Function */
bool TMC_Stepper::test_connection(MKTMC* st) {
  SERIAL_MSG("Testing ");
  st->printLabel();
  SERIAL_MSG(" connection... ");
  const uint8_t test_result = st->test_connection();

  if (test_result > 0) SERIAL_MSG("Error: All ");

  const char *stat;
  switch (test_result) {
    default:
    case 0: stat = PSTR("OK"); break;
    case 1: stat = PSTR("HIGH"); break;
    case 2: stat = PSTR("LOW"); break;
  }
  SERIAL_PGM(stat);
  SERIAL_EOL();

  return test_result;
}

#if TMC_HAS_SPI

  void TMC_Stepper::init_cs_pins() {
    #if AXIS_HAS_SPI(X)
      OUT_WRITE(X_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(Y)
      OUT_WRITE(Y_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(Z)
      OUT_WRITE(Z_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(X2)
      OUT_WRITE(X2_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(Y2)
      OUT_WRITE(Y2_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(Z2)
      OUT_WRITE(Z2_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(Z3)
      OUT_WRITE(Z3_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(E0)
      OUT_WRITE(E0_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(E1)
      OUT_WRITE(E1_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(E2)
      OUT_WRITE(E2_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(E3)
      OUT_WRITE(E3_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(E4)
      OUT_WRITE(E4_CS_PIN, HIGH);
    #endif
    #if AXIS_HAS_SPI(E5)
      OUT_WRITE(E5_CS_PIN, HIGH);
    #endif
  }

#endif // TMC_HAS_SPI

#if HAVE_DRV(TMC2660)

  void TMC_Stepper::config(MKTMC* st, const int8_t sgt/*=0*/) {

    st->begin();

    TMC2660_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 1;
    chopconf.toff = chopper_timing.toff;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    st->CHOPCONF(chopconf.sr);
 
    st->intpol(INTERPOLATE);

    #if HAS_SENSORLESS
      st->sgt(sgt);
      homing_thrs = sgt;
    #endif

    st->diss2g(true); // Disable short to ground protection. Too many false readings?

  }

#elif HAVE_DRV(TMC2130)
  
  void TMC_Stepper::config(MKTMC* st, const bool stealth/*=false*/, const int8_t sgt/*=0*/) {

    st->begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 1;
    chopconf.toff = chopper_timing.toff;
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    st->CHOPCONF(chopconf.sr);

    st->iholddelay(10);
    st->TPOWERDOWN(128);

    st->en_pwm_mode(stealth);
    st->stealthChop_enabled = stealth;

    PWMCONF_t pwmconf{0};
    pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_grad = 5;
    pwmconf.pwm_ampl = 180;
    st->PWMCONF(pwmconf.sr);

    #if HAS_SENSORLESS
      st->sgt(sgt);
      st->homing_thrs = sgt;
    #endif

    st->GSTAT(); // Clear GSTAT

  }

#elif HAVE_DRV(TMC2208)

  void TMC_Stepper::config(MKTMC* st, const bool stealth/*=false*/) {

    TMC2208_n::GCONF_t gconf{0};
    gconf.pdn_disable = true; // Use UART
    gconf.mstep_reg_select = true; // Select microsteps with UART
    gconf.i_scale_analog = false;
    gconf.en_spreadcycle = !stealth;
    st->GCONF(gconf.sr);
    st->stealthChop_enabled = stealth;

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01; // blank_time = 24
    chopconf.toff = chopper_timing.toff;
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    st->CHOPCONF(chopconf.sr);

    st->iholddelay(10);
    st->TPOWERDOWN(128);

    TMC2208_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    st->PWMCONF(pwmconf.sr);

    st->GSTAT(0b111); // Clear
    delay(200);
  }

#endif

#if ENABLED(MONITOR_DRIVER_STATUS)

  #if HAVE_DRV(TMC2660)

    TMC_driver_data TMC_Stepper::get_driver_data(MKTMC* st) {
      constexpr uint32_t OTPW_bm = 0x4UL;
      constexpr uint8_t OTPW_bp = 2;
      constexpr uint32_t OT_bm = 0x2UL;
      constexpr uint8_t OT_bp = 1;
      TMC_driver_data data;
      data.drv_status = st->DRVSTATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      return data;
    }

  #elif HAVE_DRV(TMC2130) || HAVE_DRV(TMC2160) || HAVE_DRV(TMC5130) || HAVE_DRV(TMC5160)

    TMC_driver_data TMC_Stepper::get_driver_data(MKTMC* st) {
      constexpr uint32_t OTPW_bm = 0x4000000UL;
      constexpr uint8_t OTPW_bp = 26;
      constexpr uint32_t OT_bm = 0x2000000UL;
      constexpr uint8_t OT_bp = 25;
      constexpr uint8_t S2GA_bp = 27;
      constexpr uint8_t S2GB_bp = 28;
      TMC_driver_data data;
      data.drv_status = st->DRV_STATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_s2ga = (data.drv_status >> S2GA_bp) & 0b1;
      data.is_s2gb = (data.drv_status >> S2GB_bp) & 0b1;
      return data;
    }

  #elif HAVE_DRV(TMC2208)

    #if ENABLED(TMC_DEBUG)
      uint8_t TMC_Stepper::get_status_response(MKTMC* st, uint32_t drv_status) {
        uint8_t gstat = st->GSTAT();
        uint8_t response = 0;
        response |= (drv_status >> (31 - 3)) & 0b1000;
        response |= gstat & 0b11;
        return response;
      }
    #endif

    TMC_driver_data TMC_Stepper::get_driver_data(MKTMC* st) {
      constexpr uint32_t OTPW_bm = 0b1ul;
      constexpr uint8_t OTPW_bp = 0;
      constexpr uint32_t OT_bm = 0b10ul;
      constexpr uint8_t OT_bp = 1;
      constexpr uint8_t S2GA_bp = 2;
      constexpr uint8_t S2GB_bp = 3;
      TMC_driver_data data;
      data.drv_status = st.DRV_STATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_s2ga = (data.drv_status >> S2GA_bp) & 0b1;
      data.is_s2gb = (data.drv_status >> S2GB_bp) & 0b1;
      return data;
    }

  #endif

  void TMC_Stepper::monitor_driver(MKTMC* st) {

    TMC_driver_data data = get_driver_data(st);
    if ((data.drv_status == 0xFFFFFFFF) || (data.drv_status == 0x0)) return;

    if (data.is_ot /* | data.s2ga | data.s2gb*/) st->error_count++;
    else if (st->error_count > 0) st->error_count--;

    #if ENABLED(STOP_ON_ERROR)
      if (st->error_count >= 10) {
        SERIAL_EOL();
        st->printLabel();
        SERIAL_MSG(" driver error detected: 0x");
        if (data.is_ot) SERIAL_EM("overtemperature");
        if (data.is_s2ga) SERIAL_EM("short to ground (coil A)");
        if (data.is_s2gb) SERIAL_EM("short to ground (coil B)");
        #if ENABLED(TMC_DEBUG)
          report_all(true, true, true, true);
        #endif
        printer.kill(PSTR("Driver error"));
      }
    #endif

    // Report if a warning was triggered
    if (data.is_otpw && st->otpw_count == 0) {
      char timestamp[14];
      duration_t elapsed = print_job_counter.duration();
      (void)elapsed.toDigital(timestamp, true);
      SERIAL_EOL();
      SERIAL_TXT(timestamp);
      SERIAL_MSG(": ");
      st->printLabel();
      SERIAL_MSG(" driver overtemperature warning! (");
      SERIAL_VAL(st->getMilliamps());
      SERIAL_EM("mA)");
    }

    #if CURRENT_STEP_DOWN > 0
      // Decrease current if is_otpw is true and driver is enabled and there's been more than 4 warnings
      if (data.is_otpw && st->otpw_count > 4) {
        uint16_t I_rms = st->getMilliamps();
        if (st->isEnabled() && I_rms > 100) {
          st->rms_current(I_rms - (CURRENT_STEP_DOWN));
          #if ENABLED(REPORT_CURRENT_CHANGE)
            st->printLabel();
            SERIAL_EMV(" current decreased to ", st->getMilliamps());
          #endif
        }
      }
    #endif

    if (data.is_otpw) {
      st->otpw_count++;
      st->flag_otpw = true;
    }
    else if (st->otpw_count > 0) st->otpw_count = 0;

    #if ENABLED(TMC_DEBUG)
      if (report_status) {
        const uint32_t pwm_scale = get_pwm_scale(st);
        st->printLabel();
        SERIAL_MV(":", pwm_scale);
        SERIAL_MSG(" |0b"); SERIAL_VAL(get_status_response(st, data.drv_status), BIN);
        SERIAL_MSG("| ");
        if (st->error_count) SERIAL_CHR('E');
        else if (data.is_ot) SERIAL_CHR('O');
        else if (data.is_otpw) SERIAL_CHR('W');
        else if (st->otpw_count > 0) SERIAL_VAL(st->otpw_count);
        else if (st->flag_otpw) SERIAL_CHR('F');
        SERIAL_CHR('\t');
      }
    #endif
  }

#endif // MONITOR_DRIVER_STATUS

#if ENABLED(TMC_DEBUG)

  #define PRINT_TMC_REGISTER(REG_CASE) case TMC_GET_##REG_CASE: print_hex_long(st->REG_CASE(), ':'); break

  #if HAVE_DRV(TMC2660)

    void TMC_Stepper::status(MKTMC* st, const TMCdebugEnum i) {}
    void TMC_Stepper::parse_type_drv_status(MKTMC* st, const TMCdrvStatusEnum i) {}
    void TMC_Stepper::get_ic_registers(MKTMC* st, const TMCgetRegistersEnum i) {}

    void TMC_Stepper::get_registers(MKTMC* st, const TMCgetRegistersEnum i) {
      switch (i) {
        case TMC_AXIS_CODES: SERIAL_CHR('\t'); st->printLabel(); break;
        PRINT_TMC_REGISTER(DRVCONF);
        PRINT_TMC_REGISTER(DRVCTRL);
        PRINT_TMC_REGISTER(CHOPCONF);
        PRINT_TMC_REGISTER(DRVSTATUS);
        PRINT_TMC_REGISTER(SGCSCONF);
        PRINT_TMC_REGISTER(SMARTEN);
        default: SERIAL_CHR('\t'); break;
      }
      SERIAL_CHR('\t');
    }

  #elif HAVE_DRV(TMC2130)

    void TMC_Stepper::status(MKTMC* st, const TMCdebugEnum i) {
      switch(i) {
        case TMC_PWM_SCALE: SERIAL_VAL(st->PWM_SCALE()); break;
        case TMC_SGT: SERIAL_VAL(st->sgt()); break;
        case TMC_STEALTHCHOP: SERIAL_PGM(st->en_pwm_mode() ? PSTR("true") : PSTR("false")); break;
        default: break;
      }
    }

    void TMC_Stepper::parse_type_drv_status(MKTMC* st, const TMCdrvStatusEnum i) {
      switch(i) {
        case TMC_STALLGUARD: if (st->stallguard()) SERIAL_CHR('X');  break;
        case TMC_SG_RESULT:  SERIAL_VAL(st->sg_result());            break;
        case TMC_FSACTIVE:   if (st->fsactive())   SERIAL_CHR('X'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_VAL(st->cs_actual());  break;
        default: break;
      }
    }

    void TMC_Stepper::get_ic_registers(MKTMC* st, const TMCgetRegistersEnum i) {
      switch (i) {
        PRINT_TMC_REGISTER(TCOOLTHRS);
        PRINT_TMC_REGISTER(THIGH);
        PRINT_TMC_REGISTER(COOLCONF);
        default: SERIAL_CHR('\t'); break;
      }
    }

    void TMC_Stepper::get_registers(MKTMC* st, const TMCgetRegistersEnum i) {
      switch (i) {
        case TMC_AXIS_CODES: SERIAL_CHR('\t'); st->printLabel(); break;
        PRINT_TMC_REGISTER(GCONF);
        PRINT_TMC_REGISTER(IHOLD_IRUN);
        PRINT_TMC_REGISTER(GSTAT);
        PRINT_TMC_REGISTER(IOIN);
        PRINT_TMC_REGISTER(TPOWERDOWN);
        PRINT_TMC_REGISTER(TSTEP);
        PRINT_TMC_REGISTER(TPWMTHRS);
        PRINT_TMC_REGISTER(CHOPCONF);
        PRINT_TMC_REGISTER(PWMCONF);
        PRINT_TMC_REGISTER(PWM_SCALE);
        PRINT_TMC_REGISTER(DRV_STATUS);
        default: get_ic_registers(st, i); break;
      }
      SERIAL_CHR('\t');
    }

  #elif HAVE_DRV(TMC2208)

    void TMC_Stepper::status(MKTMC* st, const TMCdebugEnum i) {
      switch(i) {
        case TMC_PWM_SCALE: SERIAL_VAL(st->pwm_scale_sum()); break;
        case TMC_STEALTHCHOP: SERIAL_PGM(st->stealth() ? PSTR("true") : PSTR("false")); break;
        case TMC_S2VSA: if (st->s2vsa()) SERIAL_CHR('X'); break;
        case TMC_S2VSB: if (st->s2vsb()) SERIAL_CHR('X'); break;
        default: break;
      }
    }

    void TMC_Stepper::parse_type_drv_status(MKTMC* st, const TMCdrvStatusEnum i) {
      switch(i) {
        case TMC_T157: if (st->t157()) SERIAL_CHR('X'); break;
        case TMC_T150: if (st->t150()) SERIAL_CHR('X'); break;
        case TMC_T143: if (st->t143()) SERIAL_CHR('X'); break;
        case TMC_T120: if (st->t120()) SERIAL_CHR('X'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_VAL(st->cs_actual()); break;
        default: break;
      }
    }

    void TMC_Stepper::get_ic_registers(MKTMC* st, const TMCgetRegistersEnum i) {
      SERIAL_CHR('\t');
    }

    void TMC_Stepper::get_registers(MKTMC* st, const TMCgetRegistersEnum i) {
      switch (i) {
        case TMC_AXIS_CODES: SERIAL_CHR('\t'); st->printLabel(); break;
        PRINT_TMC_REGISTER(GCONF);
        PRINT_TMC_REGISTER(IHOLD_IRUN);
        PRINT_TMC_REGISTER(GSTAT);
        PRINT_TMC_REGISTER(IOIN);
        PRINT_TMC_REGISTER(TPOWERDOWN);
        PRINT_TMC_REGISTER(TSTEP);
        PRINT_TMC_REGISTER(TPWMTHRS);
        PRINT_TMC_REGISTER(CHOPCONF);
        PRINT_TMC_REGISTER(PWMCONF);
        PRINT_TMC_REGISTER(PWM_SCALE);
        PRINT_TMC_REGISTER(DRV_STATUS);
        default: get_ic_registers(st, i); break;
      }
      SERIAL_CHR('\t');
    }

  #endif // HAVE_DRV(TMC2208)

  #if HAVE_DRV(TMC2660)
  
    void TMC_Stepper::status(MKTMC* st, const TMCdebugEnum i, const float spmm) {
      SERIAL_CHR('\t');
      switch(i) {
        case TMC_CODES: st->printLabel(); break;
        case TMC_ENABLED: SERIAL_PGM(st->isEnabled() ? PSTR("true") : PSTR("false")); break;
        case TMC_CURRENT: SERIAL_VAL(st->getMilliamps()); break;
        case TMC_RMS_CURRENT: SERIAL_VAL(st->rms_current()); break;
        case TMC_MAX_CURRENT: SERIAL_VAL((float)st->rms_current() * 1.41, 0); break;
        case TMC_IRUN:
          SERIAL_VAL(st->cs());
          SERIAL_MSG("/31");
          break;
        case TMC_VSENSE: print_vsense(st); break;
        case TMC_MICROSTEPS: SERIAL_VAL(st->microsteps()); break;
        case TMC_SGT: SERIAL_VAL(st->sgt()); break;
        case TMC_TOFF: SERIAL_VAL(st->toff()); break;
        case TMC_TBL: SERIAL_VAL(st->blank_time()); break;
        case TMC_HEND: SERIAL_VAL(st->hysteresis_end()); break;
        case TMC_HSTRT: SERIAL_VAL(st->hysteresis_start()); break;
        default: break;
      }
    }

  #else

    void TMC_Stepper::status(MKTMC* st, const TMCdebugEnum i, const float spmm) {
      SERIAL_CHR('\t');
      switch(i) {
        case TMC_CODES: st->printLabel(); break;
        case TMC_ENABLED: SERIAL_PGM(st->isEnabled() ? PSTR("true") : PSTR("false")); break;
        case TMC_CURRENT: SERIAL_VAL(st->getMilliamps()); break;
        case TMC_RMS_CURRENT: SERIAL_VAL(st->rms_current()); break;
        case TMC_MAX_CURRENT: SERIAL_VAL((float)st->rms_current() * 1.41, 0); break;
        case TMC_IRUN:
          SERIAL_VAL(st->irun());
          SERIAL_MSG("/31");
          break;
        case TMC_IHOLD:
          SERIAL_VAL(st->ihold());
          SERIAL_MSG("/31");
          break;
        case TMC_CS_ACTUAL:
          SERIAL_VAL(st->cs_actual());
          SERIAL_MSG("/31");
          break;
        case TMC_VSENSE: print_vsense(st); break;
        case TMC_MICROSTEPS: SERIAL_VAL(st->microsteps()); break;
        case TMC_TSTEP: {
            uint32_t tstep_value = st->TSTEP();
            if (tstep_value == 0xFFFFF) SERIAL_MSG("max");
            else SERIAL_VAL(tstep_value);
          }
          break;
        case TMC_TPWMTHRS: {
            uint32_t tpwmthrs_val = st->TPWMTHRS();
            SERIAL_VAL(tpwmthrs_val);
          }
          break;
        case TMC_TPWMTHRS_MMS: {
            uint32_t tpwmthrs_val = st->TPWMTHRS();
              if (tpwmthrs_val)
                SERIAL_VAL(tmc_thrs(st->microsteps(), tpwmthrs_val, spmm));
              else
                SERIAL_CHR('-');
            }
          break;
        case TMC_OTPW: SERIAL_PGM(st->otpw() ? PSTR("true") : PSTR("false")); break;
        #if ENABLED(MONITOR_DRIVER_STATUS)
          case TMC_OTPW_TRIGGERED: SERIAL_PGM(st->getOTPW() ? PSTR("true") : PSTR("false")); break;
        #endif
        case TMC_TOFF: SERIAL_VAL(st->toff()); break;
        case TMC_TBL: SERIAL_VAL(st->blank_time()); break;
        case TMC_HEND: SERIAL_VAL(st->hysteresis_end()); break;
        case TMC_HSTRT: SERIAL_VAL(st->hysteresis_start()); break;
        default: status(st, i); break;
      }
    }

  #endif

  void TMC_Stepper::parse_drv_status(MKTMC* st, const TMCdrvStatusEnum i) {
    SERIAL_CHR('\t');
    switch (i) {
      case TMC_DRV_CODES:     st->printLabel();                     break;
      case TMC_STST:          if (st->stst())     SERIAL_CHR('X');  break;
      case TMC_OLB:           if (st->olb())      SERIAL_CHR('X');  break;
      case TMC_OLA:           if (st->ola())      SERIAL_CHR('X');  break;
      case TMC_S2GB:          if (st->s2gb())     SERIAL_CHR('X');  break;
      case TMC_S2GA:          if (st->s2ga())     SERIAL_CHR('X');  break;
      case TMC_DRV_OTPW:      if (st->otpw())     SERIAL_CHR('X');  break;
      case TMC_OT:            if (st->ot())       SERIAL_CHR('X');  break;
      case TMC_DRV_STATUS_HEX: {
        const uint32_t drv_status = st->DRV_STATUS();
        SERIAL_CHR('\t');
        st->printLabel();
        SERIAL_CHR('\t');
        print_hex_long(drv_status, ':');
        if (drv_status == 0xFFFFFFFF || drv_status == 0) SERIAL_MSG("\t Bad response!");
        SERIAL_EOL();
        break;
      }
      default: parse_type_drv_status(st, i); break;
    }
  }

  void TMC_Stepper::debug_loop(const TMCdebugEnum i, const bool print_x, const bool print_y, const bool print_z, const bool print_e) {

    if (print_x) {
      #if AXIS_HAS_TMC(X)
        status(stepperX, i, mechanics.data.axis_steps_per_mm[X_AXIS]);
      #endif
      #if AXIS_HAS_TMC(X2)
        status(stepperX2, i, mechanics.data.axis_steps_per_mm[X_AXIS]);
      #endif
    }

    if (print_y) {
      #if AXIS_HAS_TMC(Y)
        status(stepperY, i, mechanics.data.axis_steps_per_mm[Y_AXIS]);
      #endif
      #if AXIS_HAS_TMC(Y2)
        status(stepperY2, i, mechanics.data.axis_steps_per_mm[Y_AXIS]);
      #endif
    }

    if (print_z) {
      #if AXIS_HAS_TMC(Z)
        status(stepperZ, i, mechanics.data.axis_steps_per_mm[Z_AXIS]);
      #endif
      #if AXIS_HAS_TMC(Z2)
        status(stepperZ2,i, mechanics.data.axis_steps_per_mm[Z_AXIS]);
      #endif
      #if AXIS_HAS_TMC(Z3)
        status(stepperZ3, i, mechanics.data.axis_steps_per_mm[Z_AXIS]);
      #endif
    }

    if (print_e) {
      #if AXIS_HAS_TMC(E0)
        status(stepperE0, i, mechanics.data.axis_steps_per_mm[E_AXIS_N(0)]);
      #endif
      #if AXIS_HAS_TMC(E1)
        status(stepperE1, i, mechanics.data.axis_steps_per_mm[E_AXIS_N(1)]);
      #endif
      #if AXIS_HAS_TMC(E2)
        status(stepperE2, i, mechanics.data.axis_steps_per_mm[E_AXIS_N(2)]);
      #endif
      #if AXIS_HAS_TMC(E3)
        status(stepperE3, i, mechanics.data.axis_steps_per_mm[E_AXIS_N(3)]);
      #endif
      #if AXIS_HAS_TMC(E4)
        status(stepperE4, i, mechanics.data.axis_steps_per_mm[E_AXIS_N(4)]);
      #endif
      #if AXIS_HAS_TMC(E5)
        status(stepperE5, i, mechanics.data.axis_steps_per_mm[E_AXIS_N(5)]);
      #endif
    }

    SERIAL_EOL();
  }

  void TMC_Stepper::status_loop(const TMCdrvStatusEnum i, const bool print_x, const bool print_y, const bool print_z, const bool print_e) {
    if (print_x) {
      #if AXIS_HAS_TMC(X)
        parse_drv_status(stepperX, i);
      #endif
      #if AXIS_HAS_TMC(X2)
        parse_drv_status(stepperX2, i);
      #endif
    }

    if (print_y) {
      #if AXIS_HAS_TMC(Y)
        parse_drv_status(stepperY, i);
      #endif
      #if AXIS_HAS_TMC(Y2)
        parse_drv_status(stepperY2, i);
      #endif
    }

    if (print_z) {
      #if AXIS_HAS_TMC(Z)
        parse_drv_status(stepperZ, i);
      #endif
      #if AXIS_HAS_TMC(Z2)
        parse_drv_status(stepperZ2, i);
      #endif
      #if AXIS_HAS_TMC(Z3)
        parse_drv_status(stepperZ3, i);
      #endif
    }

    if (print_e) {
      #if AXIS_HAS_TMC(E0)
        parse_drv_status(stepperE0, i);
      #endif
      #if AXIS_HAS_TMC(E1)
        parse_drv_status(stepperE1, i);
      #endif
      #if AXIS_HAS_TMC(E2)
        parse_drv_status(stepperE2, i);
      #endif
      #if AXIS_HAS_TMC(E3)
        parse_drv_status(stepperE3, i);
      #endif
      #if AXIS_HAS_TMC(E4)
        parse_drv_status(stepperE4, i);
      #endif
      #if AXIS_HAS_TMC(E5)
        parse_drv_status(stepperE5, i);
      #endif
    }

    SERIAL_EOL();
  }

  void TMC_Stepper::get_registers(const TMCgetRegistersEnum i, const bool print_x, const bool print_y, const bool print_z, const bool print_e) {

    if (print_x) {
      #if AXIS_HAS_TMC(X)
        get_registers(stepperX, i);
      #endif
      #if AXIS_HAS_TMC(X2)
        get_registers(stepperX2, i);
      #endif
    }

    if (print_y) {
      #if AXIS_HAS_TMC(Y)
        get_registers(stepperY, i);
      #endif
      #if AXIS_HAS_TMC(Y2)
        get_registers(stepperY2, i);
      #endif
    }

    if (print_z) {
      #if AXIS_HAS_TMC(Z)
        get_registers(stepperZ, i);
      #endif
      #if AXIS_HAS_TMC(Z2)
        get_registers(stepperZ2, i);
      #endif
      #if AXIS_HAS_TMC(Z3)
        get_registers(stepperZ3, i);
      #endif
    }

    if (print_e) {
      #if AXIS_HAS_TMC(E0)
        get_registers(stepperE0, i);
      #endif
      #if AXIS_HAS_TMC(E1)
        get_registers(stepperE1, i);
      #endif
      #if AXIS_HAS_TMC(E2)
        get_registers(stepperE2, i);
      #endif
      #if AXIS_HAS_TMC(E3)
        get_registers(stepperE3, i);
      #endif
      #if AXIS_HAS_TMC(E4)
        get_registers(stepperE4, i);
      #endif
      #if AXIS_HAS_TMC(E5)
        get_registers(stepperE5, i);
      #endif
    }

    SERIAL_EOL();
  }

#endif // TMC_DEBUG

#endif // HAS_TRINAMIC

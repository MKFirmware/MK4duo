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

// Public Parameters

// Private Parameters
bool TMC_Stepper::report_status = false;

// Public Function
void TMC_Stepper::init() {

  #if HAVE_DRV(TMC2660)

    #if ENABLED(SOFT_SPI_TMC2660)
      #define _TMC2660_DEFINE(ST, L)  stepper##ST = new MKTMC(L, ST##_CS_PIN, R_SENSE, TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK)
      #define TMC2660_DEFINE(ST)      _TMC2660_DEFINE(ST, TMC_##ST##_LABEL)
    #else
      #define _TMC2660_DEFINE(ST, L)  stepper##ST = new MKTMC(L, ST##_CS_PIN, R_SENSE)
      #define TMC2660_DEFINE(ST)      _TMC2660_DEFINE(ST, TMC_##ST##_LABEL)
    #endif

    #if DISABLED(SOFT_SPI_TMC2130)
      HAL::spiBegin();
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
      #define _TMC2130_DEFINE(ST, L)  stepper##ST = new MKTMC(L, ST##_CS_PIN, R_SENSE, TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK)
      #define TMC2130_DEFINE(ST)      _TMC2130_DEFINE(ST, TMC_##ST##_LABEL)
    #else
      #define _TMC2130_DEFINE(ST, L)  stepper##ST = new MKTMC(L, ST##_CS_PIN, R_SENSE)
      #define TMC2130_DEFINE(ST)      _TMC2130_DEFINE(ST, TMC_##ST##_LABEL)
    #endif

    #if DISABLED(SOFT_SPI_TMC2130)
      HAL::spiBegin();
    #endif

    // Stepper objects of TMC2130 steppers used
    #if X_HAS_DRV(TMC2130)
      TMC2130_DEFINE(X);
      config(stepperX, X_STEALTHCHOP, X_STALL_SENSITIVITY);
    #endif
    #if X2_HAS_DRV(TMC2130)
      TMC2130_DEFINE(X2);
      config(stepperX2, X2_STEALTHCHOP, X_STALL_SENSITIVITY);
    #endif
    #if Y_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Y);
      config(stepperY, Y_STEALTHCHOP, Y_STALL_SENSITIVITY);
    #endif
    #if Y2_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Y2);
      config(stepperY2, Y2_STEALTHCHOP, Y_STALL_SENSITIVITY);
    #endif
    #if Z_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Z);
      config(stepperZ, Z_STEALTHCHOP, Z_STALL_SENSITIVITY);
    #endif
    #if Z2_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Z2);
      config(stepperZ2, Z2_STEALTHCHOP, Z_STALL_SENSITIVITY);
    #endif
    #if Z3_HAS_DRV(TMC2130)
      TMC2130_DEFINE(Z3);
      config(stepperZ3, Z3_STEALTHCHOP, Z_STALL_SENSITIVITY);
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
      config(stepperX2, X2_STEALTHCHOP);
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
      config(stepperY2, Y2_STEALTHCHOP);
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
      config(stepperZ2, Z2_STEALTHCHOP);
    #endif
    #if Z3_HAS_DRV(TMC2208)
      #if ENABLED(Z3_HARDWARE_SERIAL)
        TMC2208_DEFINE_HARDWARE(Z3);
        Z3_HARDWARE_SERIAL.begin(115200);
      #else
        TMC2208_DEFINE_SOFTWARE(Z3);
        stepperZ3->beginSerial(115200);
      #endif
      config(stepperZ3, Z3_STEALTHCHOP);
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

  #define TMC_CURRENT_INIT(ST)  set_current(stepper##ST, ST##_CURRENT)

  #if AXIS_HAS_TMC(X)
    TMC_CURRENT_INIT(X);
  #endif
  #if AXIS_HAS_TMC(X2)
    TMC_CURRENT_INIT(X2);
  #endif
  #if AXIS_HAS_TMC(Y)
    TMC_CURRENT_INIT(Y);
  #endif
  #if AXIS_HAS_TMC(Y2)
    TMC_CURRENT_INIT(Y2);
  #endif
  #if AXIS_HAS_TMC(Z)
    TMC_CURRENT_INIT(Z);
  #endif
  #if AXIS_HAS_TMC(Z2)
    TMC_CURRENT_INIT(Z2);
  #endif
  #if AXIS_HAS_TMC(Z3)
    TMC_CURRENT_INIT(Z3);
  #endif
  #if AXIS_HAS_TMC(E0)
    TMC_CURRENT_INIT(E0);
  #endif
  #if AXIS_HAS_TMC(E1)
    TMC_CURRENT_INIT(E1);
  #endif
  #if AXIS_HAS_TMC(E2)
    TMC_CURRENT_INIT(E2);
  #endif
  #if AXIS_HAS_TMC(E3)
    TMC_CURRENT_INIT(E3);
  #endif
  #if AXIS_HAS_TMC(E4)
    TMC_CURRENT_INIT(E4);
  #endif
  #if AXIS_HAS_TMC(E5)
    TMC_CURRENT_INIT(E5);
  #endif
}

void TMC_Stepper::microstep_init_to_defaults() {

  #define TMC_MICROSTEP_INIT(ST)  set_microstep(stepper##ST, ST##_MICROSTEPS)

  #if AXIS_HAS_TMC(X)
    TMC_MICROSTEP_INIT(X);
  #endif
  #if AXIS_HAS_TMC(X2)
    TMC_MICROSTEP_INIT(X2);
  #endif
  #if AXIS_HAS_TMC(Y)
    TMC_MICROSTEP_INIT(Y);
  #endif
  #if AXIS_HAS_TMC(Y2)
    TMC_MICROSTEP_INIT(Y2);
  #endif
  #if AXIS_HAS_TMC(Z)
    TMC_MICROSTEP_INIT(Z);
  #endif
  #if AXIS_HAS_TMC(Z2)
    TMC_MICROSTEP_INIT(Z2);
  #endif
  #if AXIS_HAS_TMC(Z3)
    TMC_MICROSTEP_INIT(Z3);
  #endif
  #if AXIS_HAS_TMC(E0)
    TMC_MICROSTEP_INIT(E0);
  #endif
  #if AXIS_HAS_TMC(E1)
    TMC_MICROSTEP_INIT(E1);
  #endif
  #if AXIS_HAS_TMC(E2)
    TMC_MICROSTEP_INIT(E2);
  #endif
  #if AXIS_HAS_TMC(E3)
    TMC_MICROSTEP_INIT(E3);
  #endif
  #if AXIS_HAS_TMC(E4)
    TMC_MICROSTEP_INIT(E4);
  #endif
  #if AXIS_HAS_TMC(E5)
    TMC_MICROSTEP_INIT(E5);
  #endif
}

void TMC_Stepper::restore() {
  #if AXIS_HAS_TMC(X)
    stepperX->push();
  #endif
  #if AXIS_HAS_TMC(X2)
    stepperX2->push();
  #endif
  #if AXIS_HAS_TMC(Y)
    stepperY->push();
  #endif
  #if AXIS_HAS_TMC(Y2)
    stepperY2->push();
  #endif
  #if AXIS_HAS_TMC(Z)
    stepperZ->push();
  #endif
  #if AXIS_HAS_TMC(Z2)
    stepperZ2->push();
  #endif
  #if AXIS_HAS_TMC(Z3)
    stepperZ3->push();
  #endif
  #if AXIS_HAS_TMC(E0)
    stepperE0->push();
  #endif
  #if AXIS_HAS_TMC(E1)
    stepperE1->push();
  #endif
  #if AXIS_HAS_TMC(E2)
    stepperE2->push();
  #endif
  #if AXIS_HAS_TMC(E3)
    stepperE3->push();
  #endif
  #if AXIS_HAS_TMC(E4)
    stepperE4->push();
  #endif
}

#if ENABLED(MONITOR_DRIVER_STATUS)

  #define HAS_HW_COMMS(ST)  ST##_HAS_DRV(TMC2130) || ST##_HAS_DRV(TMC2660) || (ST##_HAS_DRV(TMC2208) && ENABLED(ST##_HARDWARE_SERIAL))

  void TMC_Stepper::monitor_driver() {
    static millis_t next_cOT = 0;
    if (ELAPSED(millis(), next_cOT)) {
      next_cOT = millis() + 500;
      #if HAS_HW_COMMS(X)
        monitor_driver(stepperX);
      #endif
      #if HAS_HW_COMMS(Y)
        monitor_driver(stepperY);
      #endif
      #if HAS_HW_COMMS(Z)
        monitor_driver(stepperZ);
      #endif
      #if HAS_HW_COMMS(X2)
        monitor_driver(stepperX2);
      #endif
      #if HAS_HW_COMMS(Y2)
        monitor_driver(stepperY2);
      #endif
      #if HAS_HW_COMMS(Z2)
        monitor_driver(stepperZ2);
      #endif
      #if HAS_HW_COMMS(Z3)
        monitor_driver(stepperZ3);
      #endif
      #if HAS_HW_COMMS(E0)
        monitor_driver(stepperE0);
      #endif
      #if HAS_HW_COMMS(E1)
        monitor_driver(stepperE1);
      #endif
      #if HAS_HW_COMMS(E2)
        monitor_driver(stepperE2);
      #endif
      #if HAS_HW_COMMS(E3)
        monitor_driver(stepperE3);
      #endif
      #if HAS_HW_COMMS(E4)
        monitor_driver(stepperE4);
      #endif
      #if HAS_HW_COMMS(E5)
        monitor_driver(stepperE5);
      #endif

      #if ENABLED(TMC_DEBUG)
        if (report_status) SERIAL_EOL();
      #endif

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
    #define TMC_REPORT(LABEL, ITEM) do{ SERIAL_SM(ECHO, LABEL);  debug_loop(ITEM);  }while(0)
    #define DRV_REPORT(LABEL, ITEM) do{ SERIAL_SM(ECHO, LABEL);  status_loop(ITEM); }while(0)
    TMC_REPORT("\t\t\t",              TMC_CODES);
    TMC_REPORT("Enabled\t\t",         TMC_ENABLED);
    TMC_REPORT("Set current\t",       TMC_CURRENT);
    TMC_REPORT("RMS current\t",       TMC_RMS_CURRENT);
    TMC_REPORT("MAX current\t",       TMC_MAX_CURRENT);
    TMC_REPORT("Run current\t",       TMC_IRUN);
    TMC_REPORT("Hold current\t",      TMC_IHOLD);
    TMC_REPORT("CS actual\t",         TMC_CS_ACTUAL);
    TMC_REPORT("PWM scale\t",         TMC_PWM_SCALE);
    TMC_REPORT("vsense\t\t",          TMC_VSENSE);
    TMC_REPORT("stealthChop\t",       TMC_STEALTHCHOP);
    TMC_REPORT("msteps\t\t",          TMC_MICROSTEPS);
    TMC_REPORT("tstep\t\t",           TMC_TSTEP);
    TMC_REPORT("pwm threshold\t",     TMC_TPWMTHRS);
    TMC_REPORT("[mm/s]\t\t",          TMC_TPWMTHRS_MMS);
    TMC_REPORT("OT prewarn\t",        TMC_OTPW);
    #if ENABLED(MONITOR_DRIVER_STATUS)
      TMC_REPORT("OT prewarn has",    TMC_NULL);
      TMC_REPORT("been triggered\t",  TMC_OTPW_TRIGGERED);
    #endif
    TMC_REPORT("off time\t",          TMC_TOFF);
    TMC_REPORT("blank time\t",        TMC_TBL);
    TMC_REPORT("hysteresis\t",        TMC_NULL);
    TMC_REPORT("-end\t\t",            TMC_HEND);
    TMC_REPORT("-start\t\t",          TMC_HSTRT);
    TMC_REPORT("Stallguard thrs",     TMC_SGT);

    DRV_REPORT("DRVSTATUS\t",         TMC_DRV_CODES);
    #if HAVE_DRV(TMC2130)
      DRV_REPORT("stallguard\t\t",    TMC_STALLGUARD);
      DRV_REPORT("sg_result\t",       TMC_SG_RESULT);
      DRV_REPORT("fsactive\t\t",      TMC_FSACTIVE);
    #endif
    DRV_REPORT("stst\t\t",            TMC_STST);
    DRV_REPORT("olb\t\t",             TMC_OLB);
    DRV_REPORT("ola\t\t",             TMC_OLA);
    DRV_REPORT("s2gb\t\t",            TMC_S2GB);
    DRV_REPORT("s2ga\t\t",            TMC_S2GA);
    DRV_REPORT("otpw\t\t",            TMC_DRV_OTPW);
    DRV_REPORT("ot\t\t",              TMC_OT);
    #if HAVE_DRV(TMC2208)
      DRV_REPORT("157C\t\t",          TMC_T157);
      DRV_REPORT("150C\t\t",          TMC_T150);
      DRV_REPORT("143C\t\t",          TMC_T143);
      DRV_REPORT("120C\t\t",          TMC_T120);
      DRV_REPORT("s2vsa\t\t",         TMC_S2VSA);
      DRV_REPORT("s2vsb\t\t",         TMC_S2VSB);
    #endif
    DRV_REPORT("Driver registers:",   TMC_DRV_STATUS_HEX);
    SERIAL_EOL();
  }

#endif // ENABLED(TMC_DEBUG)

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

// Private Function
#if HAVE_DRV(TMC2660)

  void TMC_Stepper::config(MKTMC* st, const int8_t tmc_sgt/*=0*/) {

    st->begin();
    st->blank_time(24);
    st->toff(5); // Only enables the driver if used with stealthChop
    st->intpol(INTERPOLATE);
    st->sgt(tmc_sgt);

  }

#elif HAVE_DRV(TMC2130)
  
  void TMC_Stepper::config(MKTMC* st, const bool tmc_stealthchop/*=false*/, const int8_t tmc_sgt/*=0*/) {

    st->begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 1;
    chopconf.toff = 3;
    chopconf.intpol = INTERPOLATE;
    chopconf.hstrt = 2;
    chopconf.hend = 5;
    st->CHOPCONF(chopconf.sr);

    st->iholddelay(10);
    st->TPOWERDOWN(128);

    st->en_pwm_mode(tmc_stealthchop);
    PWMCONF_t pwmconf{0};
    pwmconf.pwm_freq = STEALTH_FREQ;
    pwmconf.pwm_autoscale = STEALTH_AUTOSCALE;
    pwmconf.pwm_grad = STEALTH_GRAD;
    pwmconf.pwm_ampl = STEALTH_AMPL;
    st->PWMCONF(pwmconf.sr);

    st->sgt(tmc_sgt);
    st->GSTAT(); // Clear GSTAT

    // Test connection
    if (st->test_connection() < 2)
      SERIAL_STR(ECHO);
    else
      SERIAL_STR(ER);
    SERIAL_MSG("stepper");
    st->printLabel();
    SERIAL_PS(st->test_connection() < 2 ? PSTR(" connect!") : PSTR(" not connect!"));
    SERIAL_EOL();

  }

#elif HAVE_DRV(TMC2208)

  void TMC_Stepper::config(MKTMC* st, const bool tmc_stealthchop/*=false*/) {

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

#if ENABLED(MONITOR_DRIVER_STATUS)

  #if HAVE_DRV(TMC2660)

    TMC_driver_data TMC_Stepper::get_driver_data(MKTMC* st) {
      constexpr uint32_t OTPW_bm = 0x4UL;
      constexpr uint8_t OTPW_bp = 2;
      constexpr uint32_t OT_bm = 0x2UL;
      constexpr uint8_t OT_bp = 1;
      constexpr uint8_t DRIVER_ERROR_bm = 0x6;
      TMC_driver_data data;
      data.drv_status = st->DRVSTATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_error = data.drv_status & DRIVER_ERROR_bm;
      return data;
    }

  #elif HAVE_DRV(TMC2130)

    #if ENABLED(TMC_DEBUG)
      uint8_t TMC_Stepper::get_status_response(MKTMC* st) {
        return st->status_response & 0xF;
      }
    #endif

    TMC_driver_data TMC_Stepper::get_driver_data(MKTMC* st) {
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

  #elif HAVE_DRV(TMC2208)

    #if ENABLED(TMC_DEBUG)
      uint8_t TMC_Stepper::get_status_response(MKTMC* st) {
        uint32_t drv_status = st->DRV_STATUS();
        uint8_t gstat = st->GSTAT();
        uint8_t response = 0;
        response |= (drv_status >> (31-3)) & 0B1000;
        response |= gstat & 0B11;
        return response;
      }
    #endif

    TMC_driver_data TMC_Stepper::get_driver_data(MKTMC* st) {
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

  #endif

  void TMC_Stepper::monitor_driver(MKTMC* st) {

    TMC_driver_data data = get_driver_data(st);

    #if ENABLED(STOP_ON_ERROR)
      if (data.is_error) {
        SERIAL_EOL();
        st->printLabel();
        SERIAL_MSG(" driver error detected:");
        if (data.is_ot) SERIAL_EM("overtemperature");
        if (st->s2ga()) SERIAL_EM("short to ground (coil A)");
        if (st->s2gb()) SERIAL_EM("short to ground (coil B)");
        #if ENABLED(TMC_DEBUG)
          report_all();
        #endif
        printer.kill(PSTR("Driver error"));
      }
    #endif

    // Report if a warning was triggered
    if (data.is_otpw && st->otpw_count == 0) {
      char timestamp[10];
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
      if (data.is_otpw && st->isEnabled() && st->otpw_count > 4) {
        st->rms_current(st->getMilliamps() - (CURRENT_STEP_DOWN));
        #if ENABLED(REPORT_CURRENT_CHANGE)
          st->printLabel();
          SERIAL_EMV(" current decreased to ", st->getMilliamps());
        #endif
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
        SERIAL_MSG(" |0b"); SERIAL_VAL(get_status_response(st), BIN);
        SERIAL_MSG("| ");
        if (data.is_error) SERIAL_CHR('E');
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

  void TMC_Stepper::drv_status_print_hex(const uint32_t drv_status) {
    for (int B = 24; B >= 8; B -= 8){
      SERIAL_VAL((drv_status >> (B + 4)) & 0xF, HEX);
      SERIAL_VAL((drv_status >> B) & 0xF, HEX);
      SERIAL_CHR(':');
    }
    SERIAL_VAL((drv_status >> 4) & 0xF, HEX);
    SERIAL_VAL(drv_status & 0xF, HEX);
    SERIAL_EOL();
  }

  #if HAVE_DRV(TMC2660)

    void TMC_Stepper::status(MKTMC* st, const TMC_debug_enum i) {}
    void TMC_Stepper::parse_type_drv_status(MKTMC* st, const TMC_drv_status_enum i) {}

  #elif HAVE_DRV(TMC2130)

    void TMC_Stepper::status(MKTMC* st, const TMC_debug_enum i) {
      switch(i) {
        case TMC_PWM_SCALE: SERIAL_VAL(st->PWM_SCALE()); break;
        case TMC_SGT: SERIAL_VAL(st->sgt()); break;
        case TMC_STEALTHCHOP: SERIAL_PS(st->en_pwm_mode() ? PSTR("true") : PSTR("false")); break;
        default: break;
      }
    }

    void TMC_Stepper::parse_type_drv_status(MKTMC* st, const TMC_drv_status_enum i) {
      switch(i) {
        case TMC_STALLGUARD: if (st->stallguard()) SERIAL_CHR('X');  break;
        case TMC_SG_RESULT:  SERIAL_VAL(st->sg_result());            break;
        case TMC_FSACTIVE:   if (st->fsactive())   SERIAL_CHR('X'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_VAL(st->cs_actual());  break;
        default: break;
      }
    }

  #elif HAVE_DRV(TMC2208)

    void TMC_Stepper::status(MKTMC* st, const TMC_debug_enum i) {
      switch(i) {
        case TMC_TSTEP: { uint32_t data = 0; st->TSTEP(&data); SERIAL_VAL(data); break; }
        case TMC_PWM_SCALE: SERIAL_VAL(st->pwm_scale_sum()); break;
        case TMC_STEALTHCHOP: SERIAL_PS(st->stealth() ? PSTR("true") : PSTR("false")); break;
        case TMC_S2VSA: if (st->s2vsa()) SERIAL_CHR('X'); break;
        case TMC_S2VSB: if (st->s2vsb()) SERIAL_CHR('X'); break;
        default: break;
      }
    }

    void TMC_Stepper::parse_type_drv_status(MKTMC* st, const TMC_drv_status_enum i) {
      switch(i) {
        case TMC_T157: if (st->t157()) SERIAL_CHR('X'); break;
        case TMC_T150: if (st->t150()) SERIAL_CHR('X'); break;
        case TMC_T143: if (st->t143()) SERIAL_CHR('X'); break;
        case TMC_T120: if (st->t120()) SERIAL_CHR('X'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_VAL(st->cs_actual()); break;
        default: break;
      }
    }

  #endif // HAVE_DRV(TMC2208)

  #if HAVE_DRV(TMC2660)
  
    void TMC_Stepper::status(MKTMC* st, const TMC_debug_enum i, const float tmc_spmm) {
      SERIAL_CHR('\t');
      switch(i) {
        case TMC_CODES: st->printLabel(); break;
        case TMC_ENABLED: SERIAL_PS(st->isEnabled() ? PSTR("true") : PSTR("false")); break;
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

    void TMC_Stepper::status(MKTMC* st, const TMC_debug_enum i, const float tmc_spmm) {
      SERIAL_CHR('\t');
      switch(i) {
        case TMC_CODES: st->printLabel(); break;
        case TMC_ENABLED: SERIAL_PS(st->isEnabled() ? PSTR("true") : PSTR("false")); break;
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
        case TMC_TSTEP: SERIAL_VAL(st->TSTEP()); break;
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
        #if ENABLED(MONITOR_DRIVER_STATUS)
          case TMC_OTPW_TRIGGERED: SERIAL_PS(st->getOTPW() ? PSTR("true") : PSTR("false")); break;
        #endif
        case TMC_TOFF: SERIAL_VAL(st->toff()); break;
        case TMC_TBL: SERIAL_VAL(st->blank_time()); break;
        case TMC_HEND: SERIAL_VAL(st->hysteresis_end()); break;
        case TMC_HSTRT: SERIAL_VAL(st->hysteresis_start()); break;
        default: status(st, i); break;
      }
    }

  #endif

  void TMC_Stepper::parse_drv_status(MKTMC* st, const TMC_drv_status_enum i) {
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
      case TMC_DRV_STATUS_HEX:
        st->printLabel();
        SERIAL_MSG("\t0x");
        drv_status_print_hex(st->DRV_STATUS());
        break;
      default: parse_type_drv_status(st, i); break;
    }
  }

  void TMC_Stepper::debug_loop(const TMC_debug_enum i) {
    #if AXIS_HAS_TMC(X)
      status(stepperX, i, mechanics.data.axis_steps_per_mm[X_AXIS]);
    #endif
    #if AXIS_HAS_TMC(X2)
      status(stepperX2, i, mechanics.data.axis_steps_per_mm[X_AXIS]);
    #endif

    #if AXIS_HAS_TMC(Y)
      status(stepperY, i, mechanics.data.axis_steps_per_mm[Y_AXIS]);
    #endif
    #if AXIS_HAS_TMC(Y2)
      status(stepperY2, i, mechanics.data.axis_steps_per_mm[Y_AXIS]);
    #endif

    #if AXIS_HAS_TMC(Z)
      status(stepperZ, i, mechanics.data.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if AXIS_HAS_TMC(Z2)
      status(stepperZ2,i, mechanics.data.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if AXIS_HAS_TMC(Z3)
      status(stepperZ3, i, mechanics.data.axis_steps_per_mm[Z_AXIS]);
    #endif

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

    SERIAL_EOL();
  }

  void TMC_Stepper::status_loop(const TMC_drv_status_enum i) {
    #if AXIS_HAS_TMC(X)
      parse_drv_status(stepperX, i);
    #endif
    #if AXIS_HAS_TMC(X2)
      parse_drv_status(stepperX2, i);
    #endif

    #if AXIS_HAS_TMC(Y)
      parse_drv_status(stepperY, i);
    #endif
    #if AXIS_HAS_TMC(Y2)
      parse_drv_status(stepperY2, i);
    #endif

    #if AXIS_HAS_TMC(Z)
      parse_drv_status(stepperZ, i);
    #endif
    #if AXIS_HAS_TMC(Z2)
      parse_drv_status(stepperZ2, i);
    #endif
    #if AXIS_HAS_TMC(Z3)
      parse_drv_status(stepperZ3, i);
    #endif

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

    SERIAL_EOL();
  }

#endif // TMC_DEBUG

#endif // HAS_TRINAMIC

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

/**
 * tmc.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../../MK4duo.h"
#include "sanitycheck.h"

#if HAS_TRINAMIC

#if HAVE_DRV(TMC2208)
  #include <HardwareSerial.h>
#endif

TMC_Manager tmcManager;

/** Public Parameters */
#if HAS_SENSORLESS && ENABLED(IMPROVE_HOMING_RELIABILITY)
  millis_l TMC_Manager::sg_guard_period = 0;
#endif

/** Private Parameters */
uint16_t TMC_Manager::report_status_interval = 0;

/** Public Function */
void TMC_Manager::init_cs_pins() {
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

void TMC_Manager::create_tmc() {

  #if HAVE_DRV(TMC2208)

    #define TMC_HW_DEFINE(ST,A)   do{ if (!driver[ST##_DRV]->tmc) {                                                     \
                                    driver[ST##_DRV]->tmc = new MKTMC(A##_DRV, &ST##_HARDWARE_SERIAL, float(R_SENSE));  \
                                    ST##_HARDWARE_SERIAL.begin(115200);                                                 \
                                  } }while(0)

    #define TMC_HW_DEFINE_E(ST,A) do{ if (!driver.e[ST##_DRV]->tmc) {                                                     \
                                    driver.e[ST##_DRV]->tmc = new MKTMC(A##_DRV, &ST##_HARDWARE_SERIAL, float(R_SENSE));  \
                                    ST##_HARDWARE_SERIAL.begin(115200);                                                   \
                                  } }while(0)

    #define TMC_SW_DEFINE(ST,A)   do{ if (!driver[ST##_DRV]->tmc) {                                                                                                   \
                                    driver[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_SERIAL_RX_PIN, ST##_SERIAL_TX_PIN, float(R_SENSE), ST##_SERIAL_RX_PIN > NoPin);   \
                                    driver[ST##_DRV]->tmc->beginSerial(115200);                                                                                       \
                                  } }while(0)

    #define TMC_SW_DEFINE_E(ST,A) do{ if (!driver.e[ST##_DRV]->tmc) {                                                                                                 \
                                    driver.e[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_SERIAL_RX_PIN, ST##_SERIAL_TX_PIN, float(R_SENSE), ST##_SERIAL_RX_PIN > NoPin); \
                                    driver.e[ST##_DRV]->tmc->beginSerial(115200);                                                                                     \
                                  } }while(0)

    // Stepper objects of TMC2208 steppers used
    #if X_HAS_DRV(TMC2208)
      #if ENABLED(X_HARDWARE_SERIAL)
        TMC_HW_DEFINE(X,X);
      #else
        TMC_SW_DEFINE(X,X);
      #endif
      config(driver.x, X_STEALTHCHOP);
    #endif
    #if X2_HAS_DRV(TMC2208)
      #if ENABLED(X2_HARDWARE_SERIAL)
        TMC_HW_DEFINE(X2,X);
      #else
        TMC_SW_DEFINE(X2,X);
      #endif
      config(driver.x2, X_STEALTHCHOP);
    #endif
    #if Y_HAS_DRV(TMC2208)
      #if ENABLED(Y_HARDWARE_SERIAL)
        TMC_HW_DEFINE(Y,Y);
      #else
        TMC_SW_DEFINE(Y,Y);
      #endif
      config(driver.y, Y_STEALTHCHOP);
    #endif
    #if Y2_HAS_DRV(TMC2208)
      #if ENABLED(Y2_HARDWARE_SERIAL)
        TMC_HW_DEFINE(Y2,Y);
      #else
        TMC_SW_DEFINE(Y2,Y);
      #endif
      config(driver.y2, Y_STEALTHCHOP);
    #endif
    #if Z_HAS_DRV(TMC2208)
      #if ENABLED(Z_HARDWARE_SERIAL)
        TMC_HW_DEFINE(Z,Z);
      #else
        TMC_SW_DEFINE(Z,Z);
      #endif
      config(driver.z, Z_STEALTHCHOP);
    #endif
    #if Z2_HAS_DRV(TMC2208)
      #if ENABLED(Z2_HARDWARE_SERIAL)
        TMC_HW_DEFINE(Z2,Z);
      #else
        TMC_SW_DEFINE(Z2,Z);
      #endif
      config(driver.z2, Z_STEALTHCHOP);
    #endif
    #if Z3_HAS_DRV(TMC2208)
      #if ENABLED(Z3_HARDWARE_SERIAL)
        TMC_HW_DEFINE(Z3,Z);
      #else
        TMC_SW_DEFINE(Z3,Z);
      #endif
      config(driver.z3, Z_STEALTHCHOP);
    #endif
    #if E0_HAS_DRV(TMC2208)
      #if ENABLED(E0_HARDWARE_SERIAL)
        TMC_HW_DEFINE_E(E0,E0);
      #else
        TMC_SW_DEFINE_E(E0,E0);
      #endif
      config(driver.e[0], E0_STEALTHCHOP);
    #endif
    #if E1_HAS_DRV(TMC2208)
      #if ENABLED(E1_HARDWARE_SERIAL)
        TMC_HW_DEFINE_E(E1,E1);
      #else
        TMC_SW_DEFINE_E(E1,E1);
      #endif
      config(driver.e[1], E1_STEALTHCHOP);
    #endif
    #if E2_HAS_DRV(TMC2208)
      #if ENABLED(E2_HARDWARE_SERIAL)
        TMC_HW_DEFINE_E(E2,E2);
      #else
        TMC_SW_DEFINE_E(E2,E2);
      #endif
      config(driver.e[2], E2_STEALTHCHOP);
    #endif
    #if E3_HAS_DRV(TMC2208)
      #if ENABLED(E3_HARDWARE_SERIAL)
        TMC_HW_DEFINE_E(E3,E3);
      #else
        TMC_SW_DEFINE_E(E3,E3);
      #endif
      config(driver.e[3], E3_STEALTHCHOP);
    #endif
    #if E4_HAS_DRV(TMC2208)
      #if ENABLED(E4_HARDWARE_SERIAL)
        TMC_HW_DEFINE_E(E4,E4);
      #else
        TMC_SW_DEFINE_E(E4,E4);
      #endif
      config(driver.e[4], E4_STEALTHCHOP);
    #endif
    #if E5_HAS_DRV(TMC2208)
      #if ENABLED(E5_HARDWARE_SERIAL)
        TMC_HW_DEFINE_E(E5,E5);
      #else
        TMC_SW_DEFINE_E(E5,E5);
      #endif
      config(driver.e[5], E5_STEALTHCHOP);
    #endif

  #elif HAVE_DRV(TMC2660)

    #if ENABLED(TMC_USE_SW_SPI)
      #define TMC2660_DEFINE(ST,A)    do{ if (!driver[ST##_DRV]->tmc)   driver[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_CS_PIN, float(R_SENSE), TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK);    }while(0)
      #define TMC2660_DEFINE_E(ST,A)  do{ if (!driver.e[ST##_DRV]->tmc) driver.e[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_CS_PIN, float(R_SENSE), TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK);  }while(0)
    #else
      #define TMC2660_DEFINE(ST,A)    do{ if (!driver[ST##_DRV]->tmc)   driver[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_CS_PIN, float(R_SENSE));    }while(0)
      #define TMC2660_DEFINE_E(ST,A)  do{ if (!driver.e[ST##_DRV]->tmc) driver.e[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_CS_PIN, float(R_SENSE));  }while(0)
    #endif

    // Stepper objects of TMC2660 steppers used
    #if X_HAS_DRV(TMC2660)
      TMC2660_DEFINE(X,X);
      config(driver.x);
    #endif
    #if X2_HAS_DRV(TMC2660)
      TMC2660_DEFINE(X2,X);
      config(driver.x2);
    #endif
    #if Y_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Y,Y);
      config(driver.y);
    #endif
    #if Y2_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Y2,Y);
      config(driver.y2);
    #endif
    #if Z_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Z,Z);
      config(driver.z);
    #endif
    #if Z2_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Z2,Z);
      config(driver.z2);
    #endif
    #if Z3_HAS_DRV(TMC2660)
      TMC2660_DEFINE(Z3,Z);
      config(driver.z3);
    #endif
    #if E0_HAS_DRV(TMC2660)
      TMC2660_DEFINE_E(E0,E0);
      config(driver.e[0]);
    #endif
    #if E1_HAS_DRV(TMC2660)
      TMC2660_DEFINE_E(E1,E1);
      config(driver.e[1]);
    #endif
    #if E2_HAS_DRV(TMC2660)
      TMC2660_DEFINE_E(E2,E2);
      config(driver.e[2]);
    #endif
    #if E3_HAS_DRV(TMC2660)
      TMC2660_DEFINE_E(E3,E3);
      config(driver.e[3]);
    #endif
    #if E4_HAS_DRV(TMC2660)
      TMC2660_DEFINE_E(E4,E4);
      config(driver.e[4]);
    #endif
    #if E5_HAS_DRV(TMC2660)
      TMC2660_DEFINE_E(E5,E5);
      config(driver.e[5]);
    #endif

  #elif HAS_TMCX1XX

    #if ENABLED(TMC_USE_SW_SPI)
      #define TMC_MODEL_DEFINE(ST,A)    do{ if (!driver[ST##_DRV]->tmc)   driver[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_CS_PIN, float(R_SENSE), TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK);    }while(0)
      #define TMC_MODEL_DEFINE_E(ST,A)  do{ if (!driver.e[ST##_DRV]->tmc) driver.e[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_CS_PIN, float(R_SENSE), TMC_SW_MOSI, TMC_SW_MISO, TMC_SW_SCK);  }while(0)
    #else
      #define TMC_MODEL_DEFINE(ST,A)    do{ if (!driver[ST##_DRV]->tmc)   driver[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_CS_PIN, float(R_SENSE));    }while(0)
      #define TMC_MODEL_DEFINE_E(ST,A)  do{ if (!driver.e[ST##_DRV]->tmc) driver.e[ST##_DRV]->tmc = new MKTMC(A##_DRV, ST##_CS_PIN, float(R_SENSE));  }while(0)
    #endif

    // Stepper objects of TMC2130 steppers used
    #if X_HAS_DRV(TMC2130) || X_HAS_DRV(TMC2160) || X_HAS_DRV(TMC5130) || X_HAS_DRV(TMC5160) || X_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE(X,X);
      config(driver.x, X_STEALTHCHOP);
    #endif
    #if X2_HAS_DRV(TMC2130) || X2_HAS_DRV(TMC2160) || X2_HAS_DRV(TMC5130) || X2_HAS_DRV(TMC5160) || X2_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE(X2,X);
      config(driver.x2, X_STEALTHCHOP);
    #endif
    #if Y_HAS_DRV(TMC2130) || Y_HAS_DRV(TMC2160) || Y_HAS_DRV(TMC5130) || Y_HAS_DRV(TMC5160) || Y_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE(Y,Y);
      config(driver.y, Y_STEALTHCHOP);
    #endif
    #if Y2_HAS_DRV(TMC2130) || Y2_HAS_DRV(TMC2160) || Y2_HAS_DRV(TMC5130) || Y2_HAS_DRV(TMC5160) || Y2_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE(Y2,Y);
      config(driver.y2, Y_STEALTHCHOP);
    #endif
    #if Z_HAS_DRV(TMC2130) || Z_HAS_DRV(TMC2160) || Z_HAS_DRV(TMC5130) || Z_HAS_DRV(TMC5160) || Z_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE(Z,Z);
      config(driver.z, Z_STEALTHCHOP);
    #endif
    #if Z2_HAS_DRV(TMC2130) || Z2_HAS_DRV(TMC2160) || Z2_HAS_DRV(TMC5130) || Z2_HAS_DRV(TMC5160) || Z2_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE(Z2,Z);
      config(driver.z2, Z_STEALTHCHOP);
    #endif
    #if Z3_HAS_DRV(TMC2130) || Z3_HAS_DRV(TMC2160) || Z3_HAS_DRV(TMC5130) || Z3_HAS_DRV(TMC5160) || Z3_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE(Z3,Z);
      config(driver.z3, Z_STEALTHCHOP);
    #endif
    #if E0_HAS_DRV(TMC2130) || E0_HAS_DRV(TMC2160) || E0_HAS_DRV(TMC5130) || E0_HAS_DRV(TMC5160) || E0_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE_E(E0,E0);
      config(driver.e[0], E0_STEALTHCHOP);
    #endif
    #if E1_HAS_DRV(TMC2130) || E1_HAS_DRV(TMC2160) || E1_HAS_DRV(TMC5130) || E1_HAS_DRV(TMC5160) || E1_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE_E(E1,E1);
      config(driver.e[1], E1_STEALTHCHOP);
    #endif
    #if E2_HAS_DRV(TMC2130) || E2_HAS_DRV(TMC2160) || E2_HAS_DRV(TMC5130) || E2_HAS_DRV(TMC5160) || E2_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE_E(E2,E2);
      config(driver.e[2], E2_STEALTHCHOP);
    #endif
    #if E3_HAS_DRV(TMC2130) || E3_HAS_DRV(TMC2160) || E3_HAS_DRV(TMC5130) || E3_HAS_DRV(TMC5160) || E3_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE_E(E3,E3);
      config(driver.e[3], E3_STEALTHCHOP);
    #endif
    #if E4_HAS_DRV(TMC2130) || E4_HAS_DRV(TMC2160) || E4_HAS_DRV(TMC5130) || E4_HAS_DRV(TMC5160) || E4_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE_E(E4,E4);
      config(driver.e[4], E4_STEALTHCHOP);
    #endif
    #if E5_HAS_DRV(TMC2130) || E5_HAS_DRV(TMC2160) || E5_HAS_DRV(TMC5130) || E5_HAS_DRV(TMC5160) || E4_HAS_DRV(TMC5161)
      TMC_MODEL_DEFINE_E(E5,E5);
      config(driver.e[5], E5_STEALTHCHOP);
    #endif

  #endif

  #if HAS_SENSORLESS
    #if X_HAS_SENSORLESS
      #if AXIS_HAS_STALLGUARD(X)
        driver.x->tmc->homing_threshold(X_STALL_SENSITIVITY);
      #endif
      #if AXIS_HAS_STALLGUARD(X2)
        driver.x2->tmc->homing_threshold(X_STALL_SENSITIVITY);
      #endif
    #endif
    #if Y_HAS_SENSORLESS
      #if AXIS_HAS_STALLGUARD(Y)
        driver.y->tmc->homing_threshold(Y_STALL_SENSITIVITY);
      #endif
      #if AXIS_HAS_STALLGUARD(Y2)
        driver.y2->tmc->homing_threshold(Y_STALL_SENSITIVITY);
      #endif
    #endif
    #if Z_HAS_SENSORLESS
      #if AXIS_HAS_STALLGUARD(Z)
        driver.z->tmc->homing_threshold(Z_STALL_SENSITIVITY);
      #endif
      #if AXIS_HAS_STALLGUARD(Z2)
        driver.z2->tmc->homing_threshold(Z_STALL_SENSITIVITY);
      #endif
      #if AXIS_HAS_STALLGUARD(Z3)
        driver.z3->tmc->homing_threshold(Z_STALL_SENSITIVITY);
      #endif
    #endif
  #endif

  TMC_ADV();

}

// Use internal reference voltage for current calculations. This is the default.
// Following values from Trinamic's spreadsheet with values for a NEMA17 (42BYGHW609)
// https://www.trinamic.com/products/integrated-circuits/details/tmc2130/
void TMC_Manager::factory_parameters() {

  constexpr uint16_t  tmc_stepper_current[]     = { X_CURRENT, Y_CURRENT, Z_CURRENT,
                                                    X_CURRENT, Y_CURRENT, Z_CURRENT, Z_CURRENT },
                      tmc_stepper_current_e[]   = { E0_CURRENT, E1_CURRENT, E2_CURRENT, E3_CURRENT, E4_CURRENT, E5_CURRENT },
                      tmc_stepper_microstep[]   = { X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS,
                                                    X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS, Z_MICROSTEPS },
                      tmc_stepper_microstep_e[] = { E0_MICROSTEPS, E1_MICROSTEPS, E2_MICROSTEPS, E3_MICROSTEPS, E4_MICROSTEPS, E5_MICROSTEPS };

  constexpr uint32_t  tmc_hybrid_threshold[]    = { X_HYBRID_THRESHOLD, Y_HYBRID_THRESHOLD, Z_HYBRID_THRESHOLD,
                                                    X_HYBRID_THRESHOLD, Y_HYBRID_THRESHOLD, Z_HYBRID_THRESHOLD, Z_HYBRID_THRESHOLD },
                      tmc_hybrid_threshold_e[]  = { E0_HYBRID_THRESHOLD, E1_HYBRID_THRESHOLD, E2_HYBRID_THRESHOLD,
                                                    E3_HYBRID_THRESHOLD, E4_HYBRID_THRESHOLD, E5_HYBRID_THRESHOLD };

  constexpr bool      tmc_stealth_enabled[]     = { X_STEALTHCHOP, Y_STEALTHCHOP, Z_STEALTHCHOP,
                                                    X_STEALTHCHOP, Y_STEALTHCHOP, Z_STEALTHCHOP, Z_STEALTHCHOP },
                      tmc_stealth_enabled_e[]   = { E0_STEALTHCHOP, E1_STEALTHCHOP, E2_STEALTHCHOP,
                                                    E3_STEALTHCHOP, E4_STEALTHCHOP, E5_STEALTHCHOP };

  LOOP_DRV_ALL_XYZ() {
    Driver* drv = driver[d];
    if (drv && drv->tmc) {
      drv->tmc->rms_current(tmc_stepper_current[d]);
      drv->tmc->microsteps(tmc_stepper_microstep[d]);
      #if ENABLED(HYBRID_THRESHOLD)
        drv->tmc->set_pwm_thrs(tmc_hybrid_threshold[d]);
      #endif
      #if TMC_HAS_STEALTHCHOP
        drv->tmc->stealthChop_enabled = tmc_stealth_enabled[d];
        drv->tmc->refresh_stepping_mode();
      #endif
    }
  }

  LOOP_DRV_EXT() {
    Driver* drv = driver.e[d];
    if (drv && drv->tmc) {
      drv->tmc->rms_current(tmc_stepper_current_e[d]);
      drv->tmc->microsteps(tmc_stepper_microstep_e[d]);
      #if ENABLED(HYBRID_THRESHOLD)
        drv->tmc->set_pwm_thrs_e(tmc_hybrid_threshold_e[d]);
      #endif
      #if TMC_HAS_STEALTHCHOP
        drv->tmc->stealthChop_enabled = tmc_stealth_enabled_e[d];
        drv->tmc->refresh_stepping_mode();
      #endif
    }
  }

}

void TMC_Manager::restore() {
  LOOP_DRV() if (driver[d] && driver[d]->tmc) driver[d]->tmc->push();
}

void TMC_Manager::test_connection(const bool test_x, const bool test_y, const bool test_z, const bool test_e) {
  uint8_t axis_connection = 0;

  if (test_x) {
    #if AXIS_HAS_TMC(X)
      axis_connection += test_connection(driver.x);
    #endif
    #if AXIS_HAS_TMC(X2)
      axis_connection += test_connection(driver.x2);
    #endif
  }

  if (test_y) {
    #if AXIS_HAS_TMC(Y)
      axis_connection += test_connection(driver.y);
    #endif
    #if AXIS_HAS_TMC(Y2)
      axis_connection += test_connection(driver.y2);
    #endif
  }

  if (test_z) {
    #if AXIS_HAS_TMC(Z)
      axis_connection += test_connection(driver.z);
    #endif
    #if AXIS_HAS_TMC(Z2)
      axis_connection += test_connection(driver.z2);
    #endif
    #if AXIS_HAS_TMC(Z3)
      axis_connection += test_connection(driver.z3);
    #endif
  }

  if (test_e) {
    #if AXIS_HAS_TMC(E0)
      axis_connection += test_connection(driver.e[0]);
    #endif
    #if AXIS_HAS_TMC(E1)
      axis_connection += test_connection(driver.e[1]);
    #endif
    #if AXIS_HAS_TMC(E2)
      axis_connection += test_connection(driver.e[2]);
    #endif
    #if AXIS_HAS_TMC(E3)
      axis_connection += test_connection(driver.e[3]);
    #endif
    #if AXIS_HAS_TMC(E4)
      axis_connection += test_connection(driver.e[4]);
    #endif
    #if AXIS_HAS_TMC(E5)
      axis_connection += test_connection(driver.e[5]);
    #endif
  }

  if (axis_connection) lcdui.set_status_P(PSTR("TMC CONNECTION ERROR"));
}

#if ENABLED(MONITOR_DRIVER_STATUS)

  void TMC_Manager::monitor_drivers() {

    static short_timer_t next_poll_timer(millis());

    // Poll TMC drivers at the configured interval
    bool need_update_error_counters = next_poll_timer.expired(MONITOR_DRIVER_STATUS_INTERVAL_MS);

    #if ENABLED(TMC_DEBUG)
      static short_timer_t next_debug_reporting_timer(millis());
      bool need_debug_reporting = next_debug_reporting_timer.expired(report_status_interval);
    #else
      constexpr bool need_debug_reporting = false;
    #endif

    if (need_update_error_counters || need_debug_reporting) {
      LOOP_DRV_ALL_XYZ()
        if (driver[d] && driver[d]->tmc) monitor_driver(driver[d], need_update_error_counters, need_debug_reporting);
      LOOP_DRV_EXT()
        if (driver.e[d] && driver.e[d]->tmc) monitor_driver(driver.e[d], need_update_error_counters, need_debug_reporting);
      #if ENABLED(TMC_DEBUG)
        if (need_debug_reporting) SERIAL_EOL();
      #endif
    }

  }

#endif // ENABLED(MONITOR_DRIVER_STATUS)

#if HAS_SENSORLESS

  bool TMC_Manager::enable_stallguard(Driver* drv) {
    const bool old_stealthChop = drv->tmc->en_pwm_mode();
    drv->tmc->TCOOLTHRS(0xFFFFF);
    drv->tmc->en_pwm_mode(false);
    drv->tmc->diag1_stall(true);
    return old_stealthChop;
  }

  void TMC_Manager::disable_stallguard(Driver* drv, const bool enable) {
    drv->tmc->TCOOLTHRS(0);
    drv->tmc->en_pwm_mode(enable);
    drv->tmc->diag1_stall(false);
  }

#endif

#if ENABLED(TMC_DEBUG)

  /**
   * M922 [S<0|1>] [Pnnn] Enable periodic status reports
   */
  #if ENABLED(MONITOR_DRIVER_STATUS)
    void TMC_Manager::set_report_interval(const uint16_t update_interval) {
      if ((report_status_interval = update_interval)) {
        SERIAL_EM("axis:pwm_scale"
        #if TMC_HAS_STEALTHCHOP
          "/current_scale"
        #endif
        #if TMC_HAS_STALLGUARD
            "/mech_load"
          #endif
          "|flags|warncount"
        );
      }
    }
  #endif

  /**
   * M922 report functions
   */
  void TMC_Manager::report_all(bool print_x, const bool print_y, const bool print_z, const bool print_e) {
    #define TMC_REPORT(LABEL, ITEM) do{ SERIAL_SM(ECHO, LABEL);  debug_loop(ITEM, print_x, print_y, print_z, print_e); }while(0)
    #define DRV_REPORT(LABEL, ITEM) do{ SERIAL_SM(ECHO, LABEL); status_loop(ITEM, print_x, print_y, print_z, print_e); }while(0)
    TMC_REPORT("\t\t",                TMC_CODES);
    TMC_REPORT("Enabled\t",           TMC_ENABLED);
    TMC_REPORT("Set current",         TMC_CURRENT);
    TMC_REPORT("RMS current",         TMC_RMS_CURRENT);
    TMC_REPORT("MAX current",         TMC_MAX_CURRENT);
    TMC_REPORT("Run current",         TMC_IRUN);
    TMC_REPORT("Hold current",        TMC_IHOLD);
    #if HAVE_DRV(TMC2160) || HAVE_DRV(TMC5160) || HAVE_DRV(TMC5161)
      TMC_REPORT("Global scaler",     TMC_GLOBAL_SCALER);
    #endif
    TMC_REPORT("CS actual",           TMC_CS_ACTUAL);
    TMC_REPORT("PWM scale",           TMC_PWM_SCALE);
    TMC_REPORT("vsense\t",            TMC_VSENSE);
    TMC_REPORT("stealthChop",         TMC_STEALTHCHOP);
    TMC_REPORT("msteps\t",            TMC_MICROSTEPS);
    TMC_REPORT("tstep\t",             TMC_TSTEP);
    TMC_REPORT("threshold",           TMC_TPWMTHRS);
    TMC_REPORT("[mm/s]\t",            TMC_TPWMTHRS_MMS);
    TMC_REPORT("OT prewarn",          TMC_OTPW);
    #if ENABLED(MONITOR_DRIVER_STATUS)
      TMC_REPORT("OT prewarn has\n"
                 "been triggered",    TMC_OTPW_TRIGGERED);
    #endif
    TMC_REPORT("off time",            TMC_TOFF);
    TMC_REPORT("blank time",          TMC_TBL);
    TMC_REPORT("hyst-end",            TMC_HEND);
    TMC_REPORT("hyst-start",          TMC_HSTRT);
    TMC_REPORT("Stallguard",          TMC_SGT);

    DRV_REPORT("DRVSTATUS",           TMC_DRV_CODES);
    #if HAS_TMCX1XX
      DRV_REPORT("stallguard",        TMC_STALLGUARD);
      DRV_REPORT("sg_result",         TMC_SG_RESULT);
      DRV_REPORT("fsactive",          TMC_FSACTIVE);
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
    DRV_REPORT("Driver registers:\n", TMC_DRV_STATUS_HEX);
    SERIAL_EOL();
  }

  void TMC_Manager::get_registers(bool print_x, bool print_y, bool print_z, bool print_e) {
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

  void TMC_Manager::print_M350() {

    SERIAL_LM(CFG, "Stepper driver microsteps");

    SERIAL_SM(CFG, "  M350");
    LOOP_DRV_XYZ() {
      if (driver[d] && driver[d]->tmc) {
        SERIAL_MSG(" ");
        driver[d]->printLabel();
        SERIAL_VAL(driver[d]->tmc->getMicrosteps());
      }
    }
    SERIAL_EOL();

    LOOP_DRV_EXT() {
      if (driver.e[d] && driver.e[d]->tmc) {
        SERIAL_SM(CFG, "  M350 ");
        driver.e[d]->printLabel();
        SERIAL_EMV(" E", driver.e[d]->tmc->getMicrosteps());
      }
    }

  }

  void TMC_Manager::print_M906() {

    SERIAL_LM(CFG, "Stepper driver current (mA)");

    SERIAL_SM(CFG, "  M906");
    LOOP_DRV_XYZ() {
      if (driver[d] && driver[d]->tmc) {
        SERIAL_MSG(" ");
        driver[d]->printLabel();
        SERIAL_VAL(driver[d]->tmc->getMilliamps());
      }
    }
    SERIAL_EOL();

    LOOP_DRV_EXT() {
      if (driver.e[d] && driver.e[d]->tmc) {
        SERIAL_SM(CFG, "  M906 ");
        driver.e[d]->printLabel();
        SERIAL_EMV(" E", driver.e[d]->tmc->getMilliamps());
      }
    }

  }

  void TMC_Manager::print_M913() {

    #if ENABLED(HYBRID_THRESHOLD)

      SERIAL_LM(CFG, "Stepper driver Hybrid Threshold");

      SERIAL_SM(CFG, "  M913");
      LOOP_DRV_XYZ() {
        if (driver[d] && driver[d]->tmc) {
          SERIAL_MSG(" ");
          driver[d]->printLabel();
          SERIAL_VAL(driver[d]->tmc->get_pwm_thrs());
        }
      }
      SERIAL_EOL();

      LOOP_DRV_EXT() {
        if (driver.e[d] && driver.e[d]->tmc) {
          SERIAL_SM(CFG, "  M913 ");
          driver.e[d]->printLabel();
          SERIAL_EMV(" E", driver.e[d]->tmc->get_pwm_thrs_e());
        }
      }

    #endif // HYBRID_THRESHOLD

  }

  void TMC_Manager::print_M914() {
    #if HAS_SENSORLESS
      SERIAL_LM(CFG, "Stepper driver StallGuard threshold:");
      #if X_HAS_SENSORLESS || Y_HAS_SENSORLESS || Z_HAS_SENSORLESS
        SERIAL_SM(CFG, "  M914");
        #if X_HAS_SENSORLESS
          SERIAL_CHR(' ');
          driver.x->printLabel();
          SERIAL_VAL(driver.x->tmc->homing_threshold());
        #endif
        #if Y_HAS_SENSORLESS
          SERIAL_CHR(' ');
          driver.y->printLabel();
          SERIAL_VAL(driver.y->tmc->homing_threshold());
        #endif
        #if Z_HAS_SENSORLESS
          SERIAL_CHR(' ');
          driver.z->printLabel();
          SERIAL_VAL(driver.z->tmc->homing_threshold());
        #endif
        SERIAL_EOL();
      #endif
    #endif // HAS_SENSORLESS
  }

  void TMC_Manager::print_M940() {
    #if TMC_HAS_STEALTHCHOP
      SERIAL_LM(CFG, "Stepper driver StealthChop:");
      SERIAL_SM(CFG, "  M940");
      #if AXIS_HAS_STEALTHCHOP(X)
        SERIAL_CHR(' ');
        driver.x->printLabel();
        SERIAL_VAL(driver.x->tmc->get_stealthChop_status());
      #endif
      #if AXIS_HAS_STEALTHCHOP(Y)
        SERIAL_CHR(' ');
        driver.y->printLabel();
        SERIAL_VAL(driver.y->tmc->get_stealthChop_status());
      #endif
      #if AXIS_HAS_STEALTHCHOP(Z)
        SERIAL_CHR(' ');
        driver.z->printLabel();
        SERIAL_VAL(driver.z->tmc->get_stealthChop_status());
      #endif
      #if AXIS_HAS_STEALTHCHOP(E0)
        SERIAL_MV(" E", driver.e[0]->tmc->get_stealthChop_status());
      #endif
      SERIAL_EOL();
    #endif // TMC_HAS_STEALTHCHOP
  }

#endif // DISABLED(DISABLE_M503)

#if ENABLED(MONITOR_DRIVER_STATUS)

  void TMC_Manager::report_otpw(Driver* drv) {
    drv->printLabel();
    SERIAL_ELOGIC(" temperature prewarn triggered:", drv->tmc->getOTPW());
  }

  void TMC_Manager::clear_otpw(Driver* drv) {
    drv->tmc->clear_otpw();
    drv->printLabel();
    SERIAL_EM(" prewarn flag cleared");
  }

#endif

void TMC_Manager::get_off_time(Driver* drv) {
  drv->printLabel();
  SERIAL_EMV(" off_time:", drv->tmc->toff());
}

void TMC_Manager::set_off_time(Driver* drv, const uint8_t off_time_val) {
  drv->tmc->toff(off_time_val);
}

#if HAVE_DRV(TMC2130)

  void TMC_Manager::get_blank_time(Driver* drv) {
    drv->printLabel();
    SERIAL_EMV(" blank_time: ", drv->tmc->blank_time());
  }

  void TMC_Manager::set_blank_time(Driver* drv, const uint8_t blank_time_val) {
    drv->tmc->blank_time(blank_time_val);
  }

  void TMC_Manager::get_hysteresis_end(Driver* drv) {
    drv->printLabel();
    SERIAL_EMV(" hysteresis_end: ", drv->tmc->hysteresis_end());
  }

  void TMC_Manager::set_hysteresis_end(Driver* drv, const int8_t hysteresis_end_val) {
    drv->tmc->hysteresis_end(hysteresis_end_val);
  }

  void TMC_Manager::get_hysteresis_start(Driver* drv) {
    drv->printLabel();
    SERIAL_EMV(" hysteresis_start: ", drv->tmc->hysteresis_start());
  }

  void TMC_Manager::set_hysteresis_start(Driver* drv, const uint8_t hysteresis_start_val) {
    drv->tmc->hysteresis_start(hysteresis_start_val);
  }

  void TMC_Manager::get_disable_I_comparator(Driver* drv) {
    drv->printLabel();
    SERIAL_EMV(" disable_I_comparator: ", drv->tmc->disfdcc());
  }

  void TMC_Manager::set_disable_I_comparator(Driver* drv, const bool onoff) {
    drv->tmc->disfdcc(onoff);
  }

  void TMC_Manager::get_stealth_gradient(Driver* drv) {
    drv->printLabel();
    SERIAL_EMV(" stealth_gradient: ", drv->tmc->pwm_grad());
  }

  void TMC_Manager::set_stealth_gradient(Driver* drv, const uint8_t stealth_gradient_val) {
    drv->tmc->pwm_grad(stealth_gradient_val);
  }

  void TMC_Manager::get_stealth_amplitude(Driver* drv) {
    drv->printLabel();
    SERIAL_EMV(" stealth_amplitude: ", drv->tmc->pwm_ampl());
  }

  void TMC_Manager::set_stealth_amplitude(Driver* drv, const uint8_t stealth_amplitude_val) {
    drv->tmc->pwm_ampl(stealth_amplitude_val);
  }

  void TMC_Manager::get_stealth_freq(Driver* drv) {
    drv->printLabel();
    SERIAL_EMV(" stealth_freq: ", drv->tmc->pwm_freq());
  }

  void TMC_Manager::set_stealth_freq(Driver* drv, const uint8_t stealth_freq_val) {
    drv->tmc->pwm_freq(stealth_freq_val);
  }

  void TMC_Manager::get_stealth_autoscale(Driver* drv) {
    drv->printLabel();
    SERIAL_EMV(" stealth_autoscale: ", drv->tmc->pwm_autoscale());
  }

  void TMC_Manager::set_stealth_autoscale(Driver* drv, const bool onoff) {
    drv->tmc->pwm_autoscale(onoff);
  }

#endif // HAVE_DRV(TMC2130)

/** Private Function */
bool TMC_Manager::test_connection(Driver* drv) {
  if (drv->tmc) {
    SERIAL_MSG("Testing ");
    drv->printLabel();
    SERIAL_MSG(" connection... ");
    const uint8_t test_result = drv->tmc->test_connection();

    if (test_result > 0) SERIAL_MSG("Error: All ");

    const char *stat;
    switch (test_result) {
      default:
      case 0: stat = PSTR("OK"); break;
      case 1: stat = PSTR("HIGH"); break;
      case 2: stat = PSTR("LOW"); break;
    }
    SERIAL_STR(stat);
    SERIAL_EOL();

    return test_result;
  }
  return false;
}

// Stepper config for type
#if HAVE_DRV(TMC2130)
  
  void TMC_Manager::config(Driver* drv, const bool stealth/*=false*/) {

    drv->tmc->begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 1;
    chopconf.toff = chopper_timing.toff;
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    #if ENABLED(SQUARE_WAVE_STEPPING)
      chopconf.dedge = true;
    #endif
    drv->tmc->CHOPCONF(chopconf.sr);

    drv->tmc->iholddelay(10);
    drv->tmc->TPOWERDOWN(128);

    drv->tmc->diag1_onstate(false);

    drv->tmc->en_pwm_mode(stealth);
    drv->tmc->stealthChop_enabled = stealth;

    PWMCONF_t pwmconf{0};
    pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_grad = 5;
    pwmconf.pwm_ampl = 180;
    drv->tmc->PWMCONF(pwmconf.sr);

    drv->tmc->GSTAT(); // Clear GSTAT

  }

#elif HAVE_DRV(TMC2160)

  void TMC_Manager::config(Driver* drv, const bool stealth/*=false*/) {

    drv->tmc->begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 1;
    chopconf.toff = chopper_timing.toff;
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    #if ENABLED(SQUARE_WAVE_STEPPING)
      chopconf.dedge = true;
    #endif
    drv->tmc->CHOPCONF(chopconf.sr);

    drv->tmc->iholddelay(10);
    drv->tmc->TPOWERDOWN(128);
    drv->tmc->TCOOLTHRS(0xFFFFF);

    drv->tmc->en_pwm_mode(stealth);
    drv->tmc->stealthChop_enabled = stealth;

    TMC2160_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    drv->tmc->PWMCONF(pwmconf.sr);

    drv->tmc->GSTAT(); // Clear GSTAT

  }

#elif HAVE_DRV(TMC2208)

  void TMC_Manager::config(Driver* drv, const bool stealth/*=false*/) {

    TMC2208_n::GCONF_t gconf{0};
    gconf.pdn_disable = true;       // Use UART
    gconf.mstep_reg_select = true;  // Select microsteps with UART
    gconf.i_scale_analog = false;
    gconf.en_spreadcycle = !stealth;
    drv->tmc->GCONF(gconf.sr);
    drv->tmc->stealthChop_enabled = stealth;

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01; // blank_time = 24
    chopconf.toff = chopper_timing.toff;
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    #if ENABLED(SQUARE_WAVE_STEPPING)
      chopconf.dedge = true;
    #endif
    drv->tmc->CHOPCONF(chopconf.sr);

    drv->tmc->iholddelay(10);
    drv->tmc->TPOWERDOWN(128);

    TMC2208_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    drv->tmc->PWMCONF(pwmconf.sr);

    drv->tmc->GSTAT(0b111); // Clear
    delay(200);
  }

#elif HAVE_DRV(TMC2660)

  void TMC_Manager::config(Driver* drv, const bool) {

    drv->tmc->begin();

    TMC2660_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 1;
    chopconf.toff = chopper_timing.toff;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    drv->tmc->CHOPCONF(chopconf.sr);
 
    drv->tmc->sdoff(0);
    #if ENABLED(SQUARE_WAVE_STEPPING)
      drv->tmc->dedge(true);
    #endif
    drv->tmc->intpol(INTERPOLATE);
    drv->tmc->diss2g(true); // Disable short to ground protection. Too many false readings?

    #if ENABLED(TMC_DEBUG)
      drv->tmc->rdsel(0b01);
    #endif

  }

#elif HAVE_DRV(TMC5130)

  void TMC_Manager::config(Driver* drv, const bool stealth/*=false*/) {

    drv->tmc->begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 1;
    chopconf.toff = chopper_timing.toff;
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    #if ENABLED(SQUARE_WAVE_STEPPING)
      chopconf.dedge = true;
    #endif
    drv->tmc->CHOPCONF(chopconf.sr);

    drv->tmc->iholddelay(10);
    drv->tmc->TPOWERDOWN(128);

    drv->tmc->en_pwm_mode(stealth);
    drv->tmc->stealthChop_enabled = stealth;

    PWMCONF_t pwmconf{0};
    pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_grad = 5;
    pwmconf.pwm_ampl = 180;
    drv->tmc->PWMCONF(pwmconf.sr);

    drv->tmc->GSTAT(); // Clear GSTAT

  }

#elif HAVE_DRV(TMC5160) || HAVE_DRV(TMC5161)
  
  void TMC_Manager::config(Driver* drv, const bool stealth/*=false*/) {

    drv->tmc->begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 1;
    chopconf.toff = chopper_timing.toff;
    chopconf.intpol = INTERPOLATE;
    chopconf.hend = chopper_timing.hend + 3;
    chopconf.hstrt = chopper_timing.hstrt - 1;
    #if ENABLED(SQUARE_WAVE_STEPPING)
      chopconf.dedge = true;
    #endif
    drv->tmc->CHOPCONF(chopconf.sr);

    drv->tmc->iholddelay(10);
    drv->tmc->TPOWERDOWN(128);

    drv->tmc->en_pwm_mode(stealth);
    drv->tmc->stealthChop_enabled = stealth;

    TMC2160_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    drv->tmc->PWMCONF(pwmconf.sr);

    drv->tmc->GSTAT(); // Clear GSTAT

  }

#endif

#if ENABLED(MONITOR_DRIVER_STATUS)

  #if ENABLED(TMC_DEBUG)
    #if HAVE_DRV(TMC2208)
      uint32_t TMC_Manager::get_pwm_scale(Driver* drv) { return drv->tmc->pwm_scale_sum(); }
    #elif HAVE_DRV(TMC2660)
      uint32_t TMC_Manager::get_pwm_scale(Driver* drv) { UNUSED(drv); return 0; }
    #elif HAS_TMCX1XX
      uint32_t TMC_Manager::get_pwm_scale(Driver* drv) { return drv->tmc->PWM_SCALE(); }
    #endif
  #endif

  #if HAVE_DRV(TMC2208)

    TMC_driver_data TMC_Manager::get_driver_data(Driver* drv) {
      constexpr uint8_t OTPW_bp = 0, OT_bp = 1;
      constexpr uint8_t S2G_bm = 0b11110;
      TMC_driver_data data;
      const auto ds = data.drv_status = drv->tmc->DRV_STATUS();
      data.is_otpw = TEST(ds, OTPW_bp);
      data.is_ot = TEST(ds, OT_bp);
      data.is_s2g = !!(ds & S2G_bm);
      #if ENABLED(TMC_DEBUG)
        constexpr uint32_t CS_ACTUAL_bm = 0x1F0000;
        constexpr uint8_t STEALTH_bp = 30, STST_bp = 31;
        #ifdef __AVR__
          // 8-bit optimization saves up to 12 bytes of PROGMEM per axis
          uint8_t spart = ds >> 16;
          data.cs_actual = spart & (CS_ACTUAL_bm >> 16);
          spart = ds >> 24;
          data.is_stealth = TEST(spart, STEALTH_bp - 24);
          data.is_standstill = TEST(spart, STST_bp - 24);
        #else
          constexpr uint8_t CS_ACTUAL_sb = 16;
          data.cs_actual = (ds & CS_ACTUAL_bm) >> CS_ACTUAL_sb;
          data.is_stealth = TEST(ds, STEALTH_bp);
          data.is_standstill = TEST(ds, STST_bp);
        #endif
        data.sg_result_reasonable = false;
      #endif
      return data;
    }

  #elif HAVE_DRV(TMC2660)

    TMC_driver_data TMC_Manager::get_driver_data(Driver* drv) {
      constexpr uint8_t OT_bp = 1, OTPW_bp = 2;
      constexpr uint8_t S2G_bm = 0b11000;
      TMC_driver_data data;
      const auto ds = data.drv_status = drv->tmc->DRVSTATUS();
      uint8_t spart = ds & 0xFF;
      data.is_otpw = TEST(spart, OTPW_bp);
      data.is_ot = TEST(spart, OT_bp);
      data.is_s2g = !!(ds & S2G_bm);
      #if ENABLED(TMC_DEBUG)
        constexpr uint8_t STALL_GUARD_bp = 0;
        constexpr uint8_t STST_bp = 7, SG_RESULT_sp = 10;
        constexpr uint32_t SG_RESULT_bm = 0xFFC00;
        data.is_stall = TEST(spart, STALL_GUARD_bp);
        data.is_standstill = TEST(spart, STST_bp);
        data.sg_result = (ds & SG_RESULT_bm) >> SG_RESULT_sp;
        data.sg_result_reasonable = true;
      #endif
      return data;
    }

  #elif HAS_TMCX1XX

    TMC_driver_data TMC_Manager::get_driver_data(Driver* drv) {
      constexpr uint8_t OT_bp = 25, OTPW_bp = 26;
      constexpr uint32_t S2G_bm = 0x18000000;
      #if ENABLED(TMC_DEBUG)
        constexpr uint16_t SG_RESULT_bm = 0x3FF;
        constexpr uint8_t STEALTH_bp = 14;
        constexpr uint32_t CS_ACTUAL_bm = 0x1F0000;
        constexpr uint8_t STALL_GUARD_bp = 24;
        constexpr uint8_t STST_bp = 31;
      #endif
      TMC_driver_data data;
      const auto ds = data.drv_status = drv->tmc->DRV_STATUS();
      #ifdef __AVR__
        // 8-bit optimization saves up to 70 bytes of PROGMEM per axis
        uint8_t spart;
        #if ENABLED(TMC_DEBUG)
          data.sg_result = ds & SG_RESULT_bm;
          spart = ds >> 8;
          data.is_stealth = TEST(spart, STEALTH_bp - 8);
          spart = ds >> 16;
          data.cs_actual = spart & (CS_ACTUAL_bm >> 16);
        #endif
        spart = ds >> 24;
        data.is_ot = TEST(spart, OT_bp - 24);
        data.is_otpw = TEST(spart, OTPW_bp - 24);
        data.is_s2g = !!(spart & (S2G_bm >> 24));
        #if ENABLED(TMC_DEBUG)
          data.is_stall = TEST(spart, STALL_GUARD_bp - 24);
          data.is_standstill = TEST(spart, STST_bp - 24);
          data.sg_result_reasonable = !data.is_standstill; // sg_result has no reasonable meaning while standstill
        #endif

      #else // !__AVR__

        data.is_ot = TEST(ds, OT_bp);
        data.is_otpw = TEST(ds, OTPW_bp);
        data.is_s2g = !!(ds & S2G_bm);
        #if ENABLED(TMC_DEBUG)
          constexpr uint8_t CS_ACTUAL_sb = 16;
          data.sg_result = ds & SG_RESULT_bm;
          data.is_stealth = TEST(ds, STEALTH_bp);
          data.cs_actual = (ds & CS_ACTUAL_bm) >> CS_ACTUAL_sb;
          data.is_stall = TEST(ds, STALL_GUARD_bp);
          data.is_standstill = TEST(ds, STST_bp);
          data.sg_result_reasonable = !data.is_standstill; // sg_result has no reasonable meaning while standstill
        #endif

      #endif // !__AVR__

      return data;
    }

  #endif

  void TMC_Manager::monitor_driver(Driver* drv, const bool need_update_error_counters, const bool need_debug_reporting) {

    TMC_driver_data data = get_driver_data(drv);
    if ((data.drv_status == 0xFFFFFFFF) || (data.drv_status == 0x0)) return;

    if (need_update_error_counters) {
      if (data.is_ot /* | data.s2ga | data.s2gb*/) drv->tmc->error_count++;
      else if (drv->tmc->error_count > 0) drv->tmc->error_count--;

      #if ENABLED(STOP_ON_ERROR)
        if (drv->tmc->error_count >= 10) {
          SERIAL_EOL();
          drv->printLabel();
          SERIAL_MSG(" driver error detected: 0x");
          SERIAL_EV(data.drv_status, HEX);
          if (data.is_ot) SERIAL_EM("overtemperature");
          if (data.is_s2g) SERIAL_EM("coil short circuit");
          #if ENABLED(TMC_DEBUG)
            report_all(true, true, true, true);
          #endif
          printer.kill(PSTR("Driver error"));
        }
      #endif

      // Report if a warning was triggered
      if (data.is_otpw && drv->tmc->otpw_count == 0) {
        char timestamp[14];
        duration_t elapsed = print_job_counter.duration();
        (void)elapsed.toDigital(timestamp, true);
        SERIAL_EOL();
        SERIAL_TXT(timestamp);
        SERIAL_MSG(": ");
        drv->printLabel();
        SERIAL_MSG(" driver overtemperature warning! (");
        SERIAL_VAL(drv->tmc->rms_current());
        SERIAL_EM("mA)");
      }

      #if CURRENT_STEP_DOWN > 0
        // Decrease current if is_otpw is true and driver is enabled and there's been more than 4 warnings
        if (data.is_otpw && drv->tmc->otpw_count > 4) {
          uint16_t I_rms = drv->tmc->rms_current();
          if (drv->tmc->isEnabled() && I_rms > 100) {
            drv->tmc->rms_current(I_rms - (CURRENT_STEP_DOWN));
            #if ENABLED(REPORT_CURRENT_CHANGE)
              drv->printLabel();
              SERIAL_EMV(" current decreased to ", drv->tmc->rms_current());
            #endif
          }
        }
      #endif

      if (data.is_otpw) {
        drv->tmc->otpw_count++;
        drv->tmc->flag_otpw = true;
      }
      else if (drv->tmc->otpw_count > 0) drv->tmc->otpw_count = 0;
    }

    #if ENABLED(TMC_DEBUG)
      if (need_debug_reporting) {
        const uint32_t pwm_scale = get_pwm_scale(drv);
        drv->printLabel();
        SERIAL_MV(":", pwm_scale);
        #if ENABLED(TMC_DEBUG)
          #if HAS_TMCX1XX || HAVE_DRV(TMC2208)
            SERIAL_MV("/", data.cs_actual);
          #endif
          #if TMC_HAS_STALLGUARD
            SERIAL_CHR('/');
            if (data.sg_result_reasonable)
              SERIAL_VAL(data.sg_result);
            else
              SERIAL_CHR('-');
          #endif
        #endif
        SERIAL_CHR('|');
        if (drv->tmc->error_count)    SERIAL_CHR('E');  // Error
        if (data.is_ot)               SERIAL_CHR('O');  // Over-temperature
        if (data.is_otpw)             SERIAL_CHR('W');  // over-temperature pre-Warning
        #if ENABLED(TMC_DEBUG)
          if (data.is_stall)          SERIAL_CHR('G');  // stallGuard
          if (data.is_stealth)        SERIAL_CHR('T');  // stealthChop
          if (data.is_standstill)     SERIAL_CHR('I');  // standstIll
        #endif
        if (drv->tmc->flag_otpw)      SERIAL_CHR('F');  // otpw Flag
        SERIAL_CHR('|');
        if (drv->tmc->otpw_count > 0) SERIAL_VAL(drv->tmc->otpw_count);
        SERIAL_CHR('\t');
      }
    #endif
  }

#endif // MONITOR_DRIVER_STATUS

#if ENABLED(TMC_DEBUG)

  #if HAVE_DRV(TMC2160) || HAVE_DRV(TMC5160) || HAVE_DRV(TMC5161)
    void TMC_Manager::print_vsense(Driver* drv) { UNUSED(drv); }
  #else
    void TMC_Manager::print_vsense(Driver* drv) { SERIAL_STR(drv->tmc->vsense() ? PSTR("1=.18") : PSTR("0=.325")); }
  #endif

  #define PRINT_TMC_REGISTER(REG_CASE) case TMC_GET_##REG_CASE: print_hex_long(drv->tmc->REG_CASE(), ':'); break

  #if HAVE_DRV(TMC2208)

    void TMC_Manager::status(Driver* drv, const TMCdebugEnum i) {
      switch (i) {
        case TMC_PWM_SCALE: SERIAL_VAL(drv->tmc->pwm_scale_sum()); break;
        case TMC_STEALTHCHOP: SERIAL_LOGIC("", drv->tmc->stealth()); break;
        case TMC_S2VSA: if (drv->tmc->s2vsa()) SERIAL_CHR('X'); break;
        case TMC_S2VSB: if (drv->tmc->s2vsb()) SERIAL_CHR('X'); break;
        default: break;
      }
    }

    void TMC_Manager::parse_type_drv_status(Driver* drv, const TMCdrvStatusEnum i) {
      switch (i) {
        case TMC_T157: if (drv->tmc->t157()) SERIAL_CHR('X'); break;
        case TMC_T150: if (drv->tmc->t150()) SERIAL_CHR('X'); break;
        case TMC_T143: if (drv->tmc->t143()) SERIAL_CHR('X'); break;
        case TMC_T120: if (drv->tmc->t120()) SERIAL_CHR('X'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_VAL(drv->tmc->cs_actual()); break;
        default: break;
      }
    }

    void TMC_Manager::get_ic_registers(Driver* drv, const TMCgetRegistersEnum i) {
      UNUSED(drv);
      UNUSED(i);
      SERIAL_CHR('\t');
    }

    void TMC_Manager::get_registers(Driver* drv, const TMCgetRegistersEnum i) {
      switch (i) {
        case TMC_AXIS_CODES: SERIAL_CHR('\t'); drv->printLabel(); break;
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
        default: get_ic_registers(drv, i); break;
      }
      SERIAL_CHR('\t');
    }

  #elif HAVE_DRV(TMC2660)

    void TMC_Manager::status(Driver* drv, const TMCdebugEnum i)                    { UNUSED(drv); UNUSED(i); }
    void TMC_Manager::parse_type_drv_status(Driver* drv, const TMCdrvStatusEnum i) { UNUSED(drv); UNUSED(i); }
    void TMC_Manager::get_ic_registers(Driver* drv, const TMCgetRegistersEnum i)   { UNUSED(drv); UNUSED(i); }

    void TMC_Manager::get_registers(Driver* drv, const TMCgetRegistersEnum i) {
      switch (i) {
        case TMC_AXIS_CODES: SERIAL_CHR('\t'); drv->printLabel(); break;
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

  #elif HAS_TMCX1XX

    void TMC_Manager::status(Driver* drv, const TMCdebugEnum i) {
      switch (i) {
        case TMC_PWM_SCALE: SERIAL_VAL(drv->tmc->PWM_SCALE()); break;
        case TMC_SGT: SERIAL_VAL(drv->tmc->sgt()); break;
        case TMC_STEALTHCHOP: SERIAL_LOGIC("", drv->tmc->en_pwm_mode()); break;
        default: break;
      }
    }

    void TMC_Manager::parse_type_drv_status(Driver* drv, const TMCdrvStatusEnum i) {
      switch (i) {
        case TMC_STALLGUARD: if (drv->tmc->stallguard()) SERIAL_CHR('X'); break;
        case TMC_SG_RESULT:  SERIAL_VAL(drv->tmc->sg_result());           break;
        case TMC_FSACTIVE:   if (drv->tmc->fsactive())   SERIAL_CHR('X'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_VAL(drv->tmc->cs_actual());        break;
        default: break;
      }
    }

    void TMC_Manager::get_ic_registers(Driver* drv, const TMCgetRegistersEnum i) {
      UNUSED(drv);
      switch (i) {
        PRINT_TMC_REGISTER(TCOOLTHRS);
        PRINT_TMC_REGISTER(THIGH);
        PRINT_TMC_REGISTER(COOLCONF);
        default: SERIAL_CHR('\t'); break;
      }
    }

    void TMC_Manager::get_registers(Driver* drv, const TMCgetRegistersEnum i) {
      switch (i) {
        case TMC_AXIS_CODES: SERIAL_CHR('\t'); drv->printLabel(); break;
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
        default: get_ic_registers(drv, i); break;
      }
      SERIAL_CHR('\t');
    }

  #endif // HAS_TMCX1XX

  #if HAVE_DRV(TMC2660)
  
    void TMC_Manager::status(Driver* drv, const TMCdebugEnum i, const float spmm) {
      SERIAL_CHR('\t');
      switch (i) {
        case TMC_CODES: drv->printLabel(); break;
        case TMC_ENABLED: SERIAL_LOGIC("", drv->tmc->isEnabled()); break;
        case TMC_CURRENT: SERIAL_VAL(drv->tmc->getMilliamps()); break;
        case TMC_RMS_CURRENT: SERIAL_VAL(drv->tmc->rms_current()); break;
        case TMC_MAX_CURRENT: SERIAL_VAL((float)drv->tmc->rms_current() * 1.41, 0); break;
        case TMC_IRUN:
          SERIAL_VAL(drv->tmc->cs(), DEC);
          SERIAL_MSG("/31");
          break;
        case TMC_VSENSE: SERIAL_STR(drv->tmc->vsense() ? PSTR("1=.165") : PSTR("0=.310")); break;
        case TMC_MICROSTEPS: SERIAL_VAL(drv->tmc->getMicrosteps()); break;
        //case TMC_OTPW: SERIAL_LOGIC("", drv->tmc->otpw()); break;
        //case TMC_OTPW_TRIGGERED: SERIAL_LOGIC("", drv->tmc->getOTPW()); break;
        case TMC_SGT: SERIAL_VAL(drv->tmc->sgt(), DEC); break;
        case TMC_TOFF: SERIAL_VAL(drv->tmc->toff(), DEC); break;
        case TMC_TBL: SERIAL_VAL(drv->tmc->blank_time(), DEC); break;
        case TMC_HEND: SERIAL_VAL(drv->tmc->hysteresis_end(), DEC); break;
        case TMC_HSTRT: SERIAL_VAL(drv->tmc->hysteresis_start(), DEC); break;
        default: break;
      }
    }

  #else

    void TMC_Manager::status(Driver* drv, const TMCdebugEnum i, const float spmm) {
      SERIAL_CHR('\t');
      switch (i) {
        case TMC_CODES: drv->printLabel(); break;
        case TMC_ENABLED: SERIAL_LOGIC("", drv->tmc->isEnabled()); break;
        case TMC_CURRENT: SERIAL_VAL(drv->tmc->getMilliamps()); break;
        case TMC_RMS_CURRENT: SERIAL_VAL(drv->tmc->rms_current()); break;
        case TMC_MAX_CURRENT: SERIAL_VAL((float)drv->tmc->rms_current() * 1.41, 0); break;
        case TMC_IRUN:
          SERIAL_VAL(drv->tmc->irun());
          SERIAL_MSG("/31");
          break;
        case TMC_IHOLD:
          SERIAL_VAL(drv->tmc->ihold());
          SERIAL_MSG("/31");
          break;
        case TMC_CS_ACTUAL:
          SERIAL_VAL(drv->tmc->cs_actual());
          SERIAL_MSG("/31");
          break;
        case TMC_VSENSE: print_vsense(drv); break;
        case TMC_MICROSTEPS: SERIAL_VAL(drv->tmc->getMicrosteps()); break;
        case TMC_TSTEP: {
          uint32_t tstep_value = drv->tmc->TSTEP();
          if (tstep_value == 0xFFFFF) SERIAL_MSG("max");
          else SERIAL_VAL(tstep_value);
        } break;
        #if ENABLED(HYBRID_THRESHOLD)
          case TMC_TPWMTHRS: SERIAL_VAL(uint32_t(drv->tmc->TPWMTHRS())); break;
          case TMC_TPWMTHRS_MMS: {
            const uint32_t tpwmthrs_val = drv->tmc->get_pwm_thrs();
            if (tpwmthrs_val) SERIAL_VAL(tpwmthrs_val); else SERIAL_CHR('-');
          } break;
        #endif
        case TMC_OTPW: SERIAL_LOGIC("", drv->tmc->otpw()); break;
        #if ENABLED(MONITOR_DRIVER_STATUS)
          case TMC_OTPW_TRIGGERED: SERIAL_LOGIC("", drv->tmc->getOTPW()); break;
        #endif
        case TMC_TOFF: SERIAL_VAL(drv->tmc->toff()); break;
        case TMC_TBL: SERIAL_VAL(drv->tmc->blank_time()); break;
        case TMC_HEND: SERIAL_VAL(drv->tmc->hysteresis_end()); break;
        case TMC_HSTRT: SERIAL_VAL(drv->tmc->hysteresis_start()); break;
        default: status(drv, i); break;
      }
    }

  #endif

  void TMC_Manager::parse_drv_status(Driver* drv, const TMCdrvStatusEnum i) {
    SERIAL_CHR('\t');
    switch (i) {
      case TMC_DRV_CODES:     drv->printLabel();                          break;
      case TMC_STST:          if (drv->tmc->stst())     SERIAL_CHR('X');  break;
      case TMC_OLB:           if (drv->tmc->olb())      SERIAL_CHR('X');  break;
      case TMC_OLA:           if (drv->tmc->ola())      SERIAL_CHR('X');  break;
      case TMC_S2GB:          if (drv->tmc->s2gb())     SERIAL_CHR('X');  break;
      case TMC_S2GA:          if (drv->tmc->s2ga())     SERIAL_CHR('X');  break;
      case TMC_DRV_OTPW:      if (drv->tmc->otpw())     SERIAL_CHR('X');  break;
      case TMC_OT:            if (drv->tmc->ot())       SERIAL_CHR('X');  break;
      case TMC_DRV_STATUS_HEX: {
        const uint32_t drv_status = drv->tmc->DRV_STATUS();
        SERIAL_SM(ECHO, "\t\t");
        drv->printLabel();
        SERIAL_CHR('\t');
        print_hex_long(drv_status, ':');
        if (drv_status == 0xFFFFFFFF || drv_status == 0) SERIAL_MSG("\t Bad response!");
        SERIAL_EOL();
        break;
      }
      default: parse_type_drv_status(drv, i); break;
    }
  }

  void TMC_Manager::debug_loop(const TMCdebugEnum i, const bool print_x, const bool print_y, const bool print_z, const bool print_e) {

    if (print_x) {
      #if AXIS_HAS_TMC(X)
        status(driver.x, i, mechanics.data.axis_steps_per_mm.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        status(driver.x2, i, mechanics.data.axis_steps_per_mm.x);
      #endif
    }

    if (print_y) {
      #if AXIS_HAS_TMC(Y)
        status(driver.y, i, mechanics.data.axis_steps_per_mm.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        status(driver.y2, i, mechanics.data.axis_steps_per_mm.y);
      #endif
    }

    if (print_z) {
      #if AXIS_HAS_TMC(Z)
        status(driver.z, i, mechanics.data.axis_steps_per_mm.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        status(driver.z2,i, mechanics.data.axis_steps_per_mm.z);
      #endif
      #if AXIS_HAS_TMC(Z3)
        status(driver.z3, i, mechanics.data.axis_steps_per_mm.z);
      #endif
    }

    if (print_e) {
      LOOP_EXTRUDER() if (driver.e[e] && driver.e[e]->tmc) status(driver.e[e], i, extruders[e]->data.axis_steps_per_mm);
    }

    SERIAL_EOL();
  }

  void TMC_Manager::status_loop(const TMCdrvStatusEnum i, const bool print_x, const bool print_y, const bool print_z, const bool print_e) {
    if (print_x) {
      #if AXIS_HAS_TMC(X)
        parse_drv_status(driver.x, i);
      #endif
      #if AXIS_HAS_TMC(X2)
        parse_drv_status(driver.x2, i);
      #endif
    }

    if (print_y) {
      #if AXIS_HAS_TMC(Y)
        parse_drv_status(driver.y, i);
      #endif
      #if AXIS_HAS_TMC(Y2)
        parse_drv_status(driver.y2, i);
      #endif
    }

    if (print_z) {
      #if AXIS_HAS_TMC(Z)
        parse_drv_status(driver.z, i);
      #endif
      #if AXIS_HAS_TMC(Z2)
        parse_drv_status(driver.z2, i);
      #endif
      #if AXIS_HAS_TMC(Z3)
        parse_drv_status(driver.z3, i);
      #endif
    }

    if (print_e) {
      LOOP_EXTRUDER() if (driver.e[e] && driver.e[e]->tmc) parse_drv_status(driver.e[e], i);
    }

    SERIAL_EOL();
  }

  void TMC_Manager::get_registers(const TMCgetRegistersEnum i, const bool print_x, const bool print_y, const bool print_z, const bool print_e) {

    if (print_x) {
      #if AXIS_HAS_TMC(X)
        get_registers(driver.x, i);
      #endif
      #if AXIS_HAS_TMC(X2)
        get_registers(driver.x2, i);
      #endif
    }

    if (print_y) {
      #if AXIS_HAS_TMC(Y)
        get_registers(driver.y, i);
      #endif
      #if AXIS_HAS_TMC(Y2)
        get_registers(driver.y2, i);
      #endif
    }

    if (print_z) {
      #if AXIS_HAS_TMC(Z)
        get_registers(driver.z, i);
      #endif
      #if AXIS_HAS_TMC(Z2)
        get_registers(driver.z2, i);
      #endif
      #if AXIS_HAS_TMC(Z3)
        get_registers(driver.z3, i);
      #endif
    }

    if (print_e) {
      #if AXIS_HAS_TMC(E0)
        get_registers(driver.e[0], i);
      #endif
      #if AXIS_HAS_TMC(E1)
        get_registers(driver.e[1], i);
      #endif
      #if AXIS_HAS_TMC(E2)
        get_registers(driver.e[2], i);
      #endif
      #if AXIS_HAS_TMC(E3)
        get_registers(driver.e[3], i);
      #endif
      #if AXIS_HAS_TMC(E4)
        get_registers(driver.e[4], i);
      #endif
      #if AXIS_HAS_TMC(E5)
        get_registers(driver.e[5], i);
      #endif
    }

    SERIAL_EOL();
  }

#endif // TMC_DEBUG

#endif // HAS_TRINAMIC

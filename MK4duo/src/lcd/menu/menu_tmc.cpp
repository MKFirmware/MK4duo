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

//
// TMC Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && HAS_TRINAMIC

#define TMC_EDIT_STORED_I_RMS(ST)   EDIT_ITEM_P(uint16_4, PSTR(MSG_HOST_##ST), &driver[ST##_DRV]->tmc->val_mA, 100, 3000, []{ driver[ST##_DRV]->tmc->refresh_stepper_current(); })
#define TMC_EDIT_STORED_I_RMS_E(I)  EDIT_ITEM_N_P(uint16_4, I, PSTR(MSG_HOST_E), &driver.e[I]->tmc->val_mA, 100, 3000,  []{ driver.e[MenuItemBase::itemIndex]->tmc->refresh_stepper_current(); })

void menu_tmc_current() {
  START_MENU();
  BACK_ITEM(MSG_TMC_DRIVERS);
  #if AXIS_HAS_TMC(X)
    TMC_EDIT_STORED_I_RMS(X);
  #endif
  #if AXIS_HAS_TMC(Y)
    TMC_EDIT_STORED_I_RMS(Y);
  #endif
  #if AXIS_HAS_TMC(Z)
    TMC_EDIT_STORED_I_RMS(Z);
  #endif
  #if AXIS_HAS_TMC(X2)
    TMC_EDIT_STORED_I_RMS(X2);
  #endif
  #if AXIS_HAS_TMC(Y2)
    TMC_EDIT_STORED_I_RMS(Y2);
  #endif
  #if AXIS_HAS_TMC(Z2)
    TMC_EDIT_STORED_I_RMS(Z2);
  #endif
  #if AXIS_HAS_TMC(Z3)
    TMC_EDIT_STORED_I_RMS(Z3);
  #endif
  LOOP_DRV_EXT() if (driver.e[d] && driver.e[d]->tmc) TMC_EDIT_STORED_I_RMS_E(d);
  END_MENU();
}

#define TMC_EDIT_STORED_MICROSTEPS(ST)  EDIT_ITEM_P(microstep, PSTR(MSG_HOST_##ST), &driver[ST##_DRV]->tmc->val_ms, 16, 128,  []() { driver[ST##_DRV]->tmc->refresh_stepper_microstep();  })
#define TMC_EDIT_STORED_MICROSTEPS_E(I) EDIT_ITEM_N_P(microstep, I, PSTR(MSG_HOST_E), &driver.e[I]->tmc->val_ms, 16, 128,     []() { driver.e[MenuItemBase::itemIndex]->tmc->refresh_stepper_microstep();       })

void menu_tmc_microstep() {
  START_MENU();
  BACK_ITEM(MSG_TMC_DRIVERS);
  #if AXIS_HAS_TMC(X)
    TMC_EDIT_STORED_MICROSTEPS(X);
  #endif
  #if AXIS_HAS_TMC(Y)
    TMC_EDIT_STORED_MICROSTEPS(Y);
  #endif
  #if AXIS_HAS_TMC(Z)
    TMC_EDIT_STORED_MICROSTEPS(Z);
  #endif
  #if AXIS_HAS_TMC(X2)
    TMC_EDIT_STORED_MICROSTEPS(X2);
  #endif
  #if AXIS_HAS_TMC(Y2)
    TMC_EDIT_STORED_MICROSTEPS(Y2);
  #endif
  #if AXIS_HAS_TMC(Z2)
    TMC_EDIT_STORED_MICROSTEPS(Z2);
  #endif
  #if AXIS_HAS_TMC(Z3)
    TMC_EDIT_STORED_MICROSTEPS(Z3);
  #endif
  LOOP_DRV_EXT() if (driver.e[d] && driver.e[d]->tmc) TMC_EDIT_STORED_MICROSTEPS_E(d);
  END_MENU();
}

#if ENABLED(HYBRID_THRESHOLD)

  #define TMC_EDIT_STORED_HYBRID_THRS(ST)   EDIT_ITEM_P(uint8, PSTR(MSG_HOST_##ST), &driver[ST##_DRV]->tmc->hybrid_thrs, 0, 255,  []() { driver[ST##_DRV]->tmc->refresh_hybrid_thrs(); })
  #define TMC_EDIT_STORED_HYBRID_THRS_E(I)  EDIT_ITEM_N_P(uint8, I, PSTR(MSG_HOST_E), &driver.e[I]->tmc->hybrid_thrs, 0, 255,     []() { driver.e[MenuItemBase::itemIndex]->tmc->refresh_hybrid_thrs(); })

  void menu_tmc_hybrid_thrs() {
    START_MENU();
    BACK_ITEM(MSG_TMC_DRIVERS);
    #if AXIS_HAS_TMC(X)
      TMC_EDIT_STORED_HYBRID_THRS(X);
    #endif
    #if AXIS_HAS_TMC(Y)
      TMC_EDIT_STORED_HYBRID_THRS(Y);
    #endif
    #if AXIS_HAS_TMC(Z)
      TMC_EDIT_STORED_HYBRID_THRS(Z);
    #endif
    #if AXIS_HAS_TMC(X2)
      TMC_EDIT_STORED_HYBRID_THRS(X2);
    #endif
    #if AXIS_HAS_TMC(Y2)
      TMC_EDIT_STORED_HYBRID_THRS(Y2);
    #endif
    #if AXIS_HAS_TMC(Z2)
      TMC_EDIT_STORED_HYBRID_THRS(Z2);
    #endif
    #if AXIS_HAS_TMC(Z3)
      TMC_EDIT_STORED_HYBRID_THRS(Z3);
    #endif
    #if AXIS_HAS_TMC(E0)
      TMC_EDIT_STORED_HYBRID_THRS_E(0);
    #endif
    #if AXIS_HAS_TMC(E1)
      TMC_EDIT_STORED_HYBRID_THRS_E(1);
    #endif
    #if AXIS_HAS_TMC(E2)
      TMC_EDIT_STORED_HYBRID_THRS_E(2);
    #endif
    #if AXIS_HAS_TMC(E3)
      TMC_EDIT_STORED_HYBRID_THRS_E(3);
    #endif
    #if AXIS_HAS_TMC(E4)
      TMC_EDIT_STORED_HYBRID_THRS_E(4);
    #endif
    #if AXIS_HAS_TMC(E5)
      TMC_EDIT_STORED_HYBRID_THRS_E(5);
    #endif
    END_MENU();
  }

#endif

#if HAS_SENSORLESS

  #define TMC_EDIT_STORED_SGT(ST) EDIT_ITEM_P(int4, PSTR(MSG_HOST_##ST), &driver[ST##_DRV]->tmc->homing_thrs, -64, 63, []() { driver[ST##_DRV]->tmc->refresh_homing_thrs(); })

  void menu_tmc_homing_thrs() {
    START_MENU();
    BACK_ITEM(MSG_TMC_DRIVERS);
    #if X_HAS_SENSORLESS
      TMC_EDIT_STORED_SGT(X);
    #endif
    #if Y_HAS_SENSORLESS
      TMC_EDIT_STORED_SGT(Y);
    #endif
    #if Z_HAS_SENSORLESS
      TMC_EDIT_STORED_SGT(Z);
    #endif
    END_MENU();
  }

#endif

#if TMC_HAS_STEALTHCHOP

  #define TMC_EDIT_STEP_MODE(ST)    EDIT_ITEM_P(bool, PSTR(MSG_HOST_##ST), &driver[ST##_DRV]->tmc->stealthChop_enabled, []() { driver[ST##_DRV]->tmc->refresh_stepping_mode(); })
  #define TMC_EDIT_STEP_MODE_E(I) EDIT_ITEM_N_P(bool, I, PSTR(MSG_HOST_E), &driver.e[I]->tmc->stealthChop_enabled,      []() { driver.e[MenuItemBase::itemIndex]->tmc->refresh_stepping_mode(); })

  void menu_tmc_step_mode() {
    START_MENU();
    STATIC_ITEM(MSG_TMC_STEALTH_ENABLED);
    BACK_ITEM(MSG_TMC_DRIVERS);
    #if AXIS_HAS_STEALTHCHOP(X)
      TMC_EDIT_STEP_MODE(X);
    #endif
    #if AXIS_HAS_STEALTHCHOP(Y)
      TMC_EDIT_STEP_MODE(Y);
    #endif
    #if AXIS_HAS_STEALTHCHOP(Z)
      TMC_EDIT_STEP_MODE(Z);
    #endif
    #if AXIS_HAS_STEALTHCHOP(X2)
      TMC_EDIT_STEP_MODE(X2);
    #endif
    #if AXIS_HAS_STEALTHCHOP(Y2)
      TMC_EDIT_STEP_MODE(Y2);
    #endif
    #if AXIS_HAS_STEALTHCHOP(Z2)
      TMC_EDIT_STEP_MODE(Z2);
    #endif
    #if AXIS_HAS_STEALTHCHOP(Z3)
      TMC_EDIT_STEP_MODE(Z3);
    #endif
    LOOP_DRV_EXT() if (driver.e[d] && driver.e[d]->tmc) TMC_EDIT_STEP_MODE_E(d);
    END_MENU();
  }

#endif

void menu_tmc() {
  START_MENU();
  BACK_ITEM(MSG_CONTROL);
  SUBMENU(MSG_TMC_CURRENT, menu_tmc_current);
  SUBMENU(MSG_TMC_MICROSTEP, menu_tmc_microstep);
  #if ENABLED(HYBRID_THRESHOLD)
    SUBMENU(MSG_TMC_HYBRID_THRS, menu_tmc_hybrid_thrs);
  #endif
  #if HAS_SENSORLESS
    SUBMENU(MSG_TMC_HOMING_THRS, menu_tmc_homing_thrs);
  #endif
  #if TMC_HAS_STEALTHCHOP
    SUBMENU(MSG_TMC_STEPPING_MODE, menu_tmc_step_mode);
  #endif
  END_MENU();
}

#endif // HAS_TRINAMIC

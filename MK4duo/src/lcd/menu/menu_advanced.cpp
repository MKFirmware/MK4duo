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
// Advanced Settings Menus
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

char cmd[20];

void menu_tmc();

#if ENABLED(VOLUMETRIC_EXTRUSION) || ENABLED(ADVANCED_PAUSE_FEATURE)
  //
  // Advanced Settings > Filament
  //
  void menu_advanced_filament() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    #if ENABLED(LIN_ADVANCE)
      LOOP_EXTRUDER()
        EDIT_ITEM_N(float52, e, MSG_ADVANCE_K, &extruders[e]->data.advance_K, 0, 10);
    #endif

    #if ENABLED(VOLUMETRIC_EXTRUSION)

      editable.state = toolManager.isVolumetric();
      EDIT_ITEM(bool, MSG_VOLUMETRIC_ENABLED, &editable.state, []{
        toolManager.setVolumetric(editable.state);
        toolManager.calculate_volumetric_multipliers;
      });

      if (toolManager.isVolumetric()) {
        LOOP_EXTRUDER()
          EDIT_ITEM_FAST_N(float43, e, MSG_FILAMENT_DIAM, &extruders[e]->data.filament_size, 1.5f, 3.5f, toolManager.calculate_volumetric_multipliers);
      }

    #endif // ENABLED(VOLUMETRIC_EXTRUSION)

    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      constexpr float extrude_maxlength =
        #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
          EXTRUDE_MAXLENGTH
        #else
          999
        #endif
      ;

      LOOP_EXTRUDER() {
        EDIT_ITEM_FAST_N(float3, e, MSG_FILAMENT_LOAD,    &extruders[e]->data.load_length,   0, extrude_maxlength);
        EDIT_ITEM_FAST_N(float3, e, MSG_FILAMENT_UNLOAD,  &extruders[e]->data.unload_length, 0, extrude_maxlength);
      }
    #endif

    #if FILAMENT_RUNOUT_DISTANCE_MM > 0
      editable.decimal = filamentrunout.runout_distance();
      EDIT_ITEM(float3, MSG_RUNOUT_DISTANCE_MM, &editable.decimal, 1, 30, []{
        filamentrunout.set_runout_distance(editable.decimal);
      });
    #endif

    END_MENU();

  }

#endif // ENABLED(VOLUMETRIC_EXTRUSION) || ENABLED(ADVANCED_PAUSE_FEATURE)

//
// Advanced Settings > Temperature helpers
//

#if ENABLED(PID_AUTOTUNE_MENU)

  #if HAS_HOTENDS
    int16_t autotune_temp[MAX_HOTEND]           = { 200 };
  #endif
  #if HAS_BEDS
    int16_t autotune_temp_bed[MAX_BED]          = { 60 };
  #endif
  #if HAS_CHAMBERS
    int16_t autotune_temp_chamber[MAX_CHAMBER]  = { 60 };
  #endif

#endif //PID_AUTOTUNE_MENU

//
// Advanced Settings > Temperature
//
void menu_advanced_temperature() {

  START_MENU();
  BACK_ITEM(MSG_ADVANCED_SETTINGS);

  // Autotemp, Min, Max, Fact
  #if ENABLED(AUTOTEMP)
    editable.state = planner.flag.autotemp_enabled;
    EDIT_ITEM(bool, MSG_AUTOTEMP, &editable.state, []{
      planner.flag.autotemp_enabled = editable.state;
    });
    EDIT_ITEM(float3, MSG_MIN, &planner.autotemp_min, 0, hotends[0]->data.temp.max - 10);
    EDIT_ITEM(float3, MSG_MAX, &planner.autotemp_max, 0, hotends[0]->data.temp.max - 10);
    EDIT_ITEM(float52, MSG_FACTOR, &planner.autotemp_factor, 0, 1);
  #endif

  // PID Hotend
  #if HAS_HOTENDS
    LOOP_HOTEND() {
      if (hotends[h]->isUsePid()) {
        EDIT_ITEM_N(float52, h, MSG_PID_P, &hotends[h]->data.pid.Kp, 1, 9990);
        EDIT_ITEM_N(float52, h, MSG_PID_I, &hotends[h]->data.pid.Ki, 0.01f, 9990);
        EDIT_ITEM_N(float52, h, MSG_PID_D, &hotends[h]->data.pid.Kd, 1, 9990);
        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          EDIT_ITEM_N(float3, h, MSG_PID_C, &hotends[h]->data.pid.Kc, 1, 9990);
        #endif
        #if ENABLED(PID_AUTOTUNE_MENU)
          EDIT_ITEM_FAST_N(int3, h, MSG_PID_AUTOTUNE, &autotune_temp[h], 150, hotends[h]->data.temp.max - 10, []{
            sprintf_P(cmd, PSTR("M303 U1 H%d S%d"), int(MenuItemBase::itemIndex), autotune_temp[MenuItemBase::itemIndex]);
            lcd_enqueue_one_now(cmd);
          });
        #endif
      }
    }
  #endif

  // PID Bed
  #if HAS_BEDS
    LOOP_BED() {
      if (beds[h]->isUsePid()) {
        EDIT_ITEM_N(float52, h, MSG_BED_PID_P, &beds[h]->data.pid.Kp, 1, 9990);
        EDIT_ITEM_N(float52, h, MSG_BED_PID_I, &beds[h]->data.pid.Ki, 0.01f, 9990);
        EDIT_ITEM_N(float52, h, MSG_BED_PID_D, &beds[h]->data.pid.Kd, 1, 9990);
        #if ENABLED(PID_AUTOTUNE_MENU)
          EDIT_ITEM_FAST_N(int3, h, MSG_PID_BED_AUTOTUNE, &autotune_temp_bed[h], 30, beds[h]->data.temp.max - 10, []{
            sprintf_P(cmd, PSTR("M303 U1 H-1 T%i S%i"), int(MenuItemBase::itemIndex), autotune_temp_bed[MenuItemBase::itemIndex]);
            lcd_enqueue_one_now(cmd);
          });
        #endif
      }
    }
  #endif

  // PID Chamber
  #if HAS_CHAMBERS
    LOOP_CHAMBER() {
      if (chambers[h]->isUsePid()) {
        EDIT_ITEM_N(float52, h, MSG_CHAMBER_PID_P, &chambers[h]->data.pid.Kp, 1, 9990);
        EDIT_ITEM_N(float52, h, MSG_CHAMBER_PID_I, &chambers[h]->data.pid.Ki, 0.01f, 9990);
        EDIT_ITEM_N(float52, h, MSG_CHAMBER_PID_D, &chambers[h]->data.pid.Kd, 1, 9990);
        #if ENABLED(PID_AUTOTUNE_MENU)
          EDIT_ITEM_FAST_N(int3, h, MSG_PID_CHAMBER_AUTOTUNE, &autotune_temp_chamber[h], 30, chambers[h]->data.temp.max - 10, []{
            sprintf_P(cmd, PSTR("M303 U1 H-2 T%i S%i"), int(MenuItemBase::itemIndex), autotune_temp_chamber[MenuItemBase::itemIndex]);
            lcd_enqueue_one_now(cmd);
          });
        #endif
      }
    }
  #endif

  END_MENU();

}

#if DISABLED(SLIM_LCD_MENUS)

  // M203 / M205 Velocity options
  void menu_advanced_velocity() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    // M203 Max Feedrate
    #if MECH(DELTA)
      EDIT_ITEM_FAST(float3, MSG_VMAX_A, &mechanics.data.max_feedrate_mm_s.x, 1, 9999, []{
        mechanics.data.max_feedrate_mm_s.y = mechanics.data.max_feedrate_mm_s.z = mechanics.data.max_feedrate_mm_s.x;
      });
    #else
      EDIT_ITEM_FAST(float3, MSG_VMAX_A, &mechanics.data.max_feedrate_mm_s.x, 1, 9999);
      EDIT_ITEM_FAST(float3, MSG_VMAX_B, &mechanics.data.max_feedrate_mm_s.y, 1, 9999);
      EDIT_ITEM_FAST(float3, MSG_VMAX_C, &mechanics.data.max_feedrate_mm_s.z, 1, 9999);
    #endif
    LOOP_EXTRUDER()
      EDIT_ITEM_FAST_N(float3, e, MSG_VMAX_E, &extruders[e]->data.max_feedrate_mm_s, 1, 9999);

    // M205 S Min Feedrate
    EDIT_ITEM_FAST(float3, MSG_VMIN, &mechanics.data.min_feedrate_mm_s, 0, 999);

    // M205 T Min Travel Feedrate
    EDIT_ITEM_FAST(float3, MSG_VTRAV_MIN, &mechanics.data.min_travel_feedrate_mm_s, 0, 999);

    END_MENU();
  }

  // M201 / M204 Accelerations
  void menu_advanced_acceleration() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    // M204 P Acceleration
    EDIT_ITEM_FAST(float5_25, MSG_ACC, &mechanics.data.acceleration, 25, 99000);

    // M204 R Retract Acceleration
    LOOP_EXTRUDER()
      EDIT_ITEM_FAST_N(float5, e, MSG_A_RETRACT, &extruders[e]->data.retract_acceleration, 100, 99000);

    // M204 T Travel Acceleration
    EDIT_ITEM_FAST(float5_25, MSG_A_TRAVEL, &mechanics.data.travel_acceleration, 25, 99000);

    // M201 settings
    #if MECH(DELTA)
      EDIT_ITEM_FAST(long5_25, MSG_AMAX_A, &mechanics.data.max_acceleration_mm_per_s2.x, 100, 99000, []{
        mechanics.data.max_acceleration_mm_per_s2.y = mechanics.data.max_acceleration_mm_per_s2.z = mechanics.data.max_acceleration_mm_per_s2.x;
        planner.reset_acceleration_rates();
      });
    #else
      EDIT_ITEM_FAST(long5_25, MSG_AMAX_A, &mechanics.data.max_acceleration_mm_per_s2.x, 100, 99000, planner.reset_acceleration_rates);
      EDIT_ITEM_FAST(long5_25, MSG_AMAX_B, &mechanics.data.max_acceleration_mm_per_s2.y, 100, 99000, planner.reset_acceleration_rates);
      EDIT_ITEM_FAST(long5_25, MSG_AMAX_C, &mechanics.data.max_acceleration_mm_per_s2.z,  10, 99000, planner.reset_acceleration_rates);
    #endif

    LOOP_EXTRUDER()
      EDIT_ITEM_FAST_N(long5_25, e, MSG_AMAX_E, &extruders[e]->data.max_acceleration_mm_per_s2, 100, 99000, []{
        if (MenuItemBase::itemIndex == toolManager.extruder.active) planner.reset_acceleration_rates();
      });

    END_MENU();
  }

  // M205 Jerk
  void menu_advanced_jerk() {
    START_MENU();
    BACK_ITEM(MSG_MOTION);

    #if ENABLED(JUNCTION_DEVIATION)
      #if ENABLED(LIN_ADVANCE)
        EDIT_ITEM(float43, MSG_JUNCTION_MM, &mechanics.data.junction_deviation_mm, 0.01f, 0.3f, mechanics.recalculate_max_e_jerk);
      #else
        EDIT_ITEM(float43, MSG_JUNCTION_MM, &mechanics.data.junction_deviation_mm, 0.01f, 0.3f);
      #endif
    #endif

    #if HAS_CLASSIC_JERK

      #if MECH(DELTA)
        EDIT_ITEM(float52sign, MSG_JERK, &mechanics.data.max_jerk.x, 0.1f, 990, []{
          mechanics.data.max_jerk.y = mechanics.data.max_jerk.z = mechanics.data.max_jerk.x;
        });
      #else
        EDIT_ITEM_FAST(float52sign, MSG_VA_JERK, &mechanics.data.max_jerk.x, 0.1f, 990);
        EDIT_ITEM_FAST(float52sign, MSG_VB_JERK, &mechanics.data.max_jerk.y, 0.1f, 990);
        EDIT_ITEM_FAST(float52sign, MSG_VC_JERK, &mechanics.data.max_jerk.z, 0.1f, 990);
      #endif

      #if HAS_CLASSIC_E_JERK
        LOOP_EXTRUDER()
          EDIT_ITEM_FAST_N(float52sign, e, MSG_VE_JERK, &extruders[e]->data.max_jerk, 0.1f, 990);
      #endif // DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)

    #endif // AS_CLASSIC_JERK

    END_MENU();
  }

  // M92 Steps-per-mm
  void menu_advanced_steps_per_mm() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    #if MECH(DELTA)
      EDIT_ITEM_FAST(float51, MSG_STEPS_PER_MM, &mechanics.data.axis_steps_per_mm.x, 5, 9999, []{
        mechanics.data.axis_steps_per_mm.y = mechanics.data.axis_steps_per_mm.z = mechanics.data.axis_steps_per_mm.x;
        planner.refresh_positioning();
      });
    #else
      EDIT_ITEM_FAST(float51, MSG_A_STEPS, &mechanics.data.axis_steps_per_mm.x, 5, 9999, planner.refresh_positioning);
      EDIT_ITEM_FAST(float51, MSG_B_STEPS, &mechanics.data.axis_steps_per_mm.y, 5, 9999, planner.refresh_positioning);
      EDIT_ITEM_FAST(float51, MSG_C_STEPS, &mechanics.data.axis_steps_per_mm.z, 5, 9999, planner.refresh_positioning);
    #endif

    LOOP_EXTRUDER()
      EDIT_ITEM_FAST_N(float51, e, MSG_E_STEPS, &extruders[e]->data.axis_steps_per_mm, 5, 9999, []{
        if (MenuItemBase::itemIndex == toolManager.extruder.active)
          planner.refresh_positioning();
        else
          extruders[MenuItemBase::itemIndex]->steps_to_mm = RECIPROCAL(extruders[MenuItemBase::itemIndex]->data.axis_steps_per_mm);
      });

    END_MENU();
  }

  // M851 - Z Probe Offsets
  #if HAS_BED_PROBE
    void menu_probe_offsets() {
      START_MENU();
      BACK_ITEM(MSG_ADVANCED_SETTINGS);
      EDIT_ITEM(float51sign, MSG_ZPROBE_XOFFSET, &probe.data.offset.x, -50, 50);
      EDIT_ITEM(float51sign, MSG_ZPROBE_YOFFSET, &probe.data.offset.y, -50, 50);
      EDIT_ITEM(LCD_Z_OFFSET_TYPE, MSG_ZPROBE_ZOFFSET, &probe.data.offset.z, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX);
      END_MENU();
    }
  #endif

#endif // !SLIM_LCD_MENUS

void menu_advanced_settings() {

  START_MENU();
  BACK_ITEM(MSG_CONFIGURATION);

  #if DISABLED(SLIM_LCD_MENUS)

    if (printer.mode == PRINTER_MODE_FFF) {
      #if HAS_LINEAR_EXTRUDER
        editable.uint8 = toolManager.extruder.total;
        EDIT_ITEM(uint8, MSG_MAX_EXTRUDERS, &editable.uint8, 0, MAX_EXTRUDER, []{
          toolManager.change_number_extruder(editable.uint8);
        });
      #endif
      SUBMENU(MSG_TEMPERATURE, menu_advanced_temperature);
      #if ENABLED(VOLUMETRIC_EXTRUSION) || ENABLED(ADVANCED_PAUSE_FEATURE)
        SUBMENU(MSG_FILAMENT, menu_advanced_filament);
      #elif ENABLED(LIN_ADVANCE)
        LOOP_EXTRUDER()
          EDIT_ITEM_N(float52, e, MSG_ADVANCE_K, &extruders[e]->data.advance_K, 0, 999);
      #endif
    }
      
    #if ENABLED(WORKSPACE_OFFSETS)
      //
      // Set Home Offsets
      //
      ACTION_ITEM(MSG_SET_HOME_OFFSETS, []{ commands.inject_P(PSTR("M428")); lcdui.return_to_status(); });
    #endif

    // M203 / M205 - Feedrate items
    SUBMENU(MSG_VELOCITY, menu_advanced_velocity);

    // M201 - Acceleration items
    SUBMENU(MSG_ACCELERATION, menu_advanced_acceleration);

    // M205 - Junction Deviation or Max Jerk
    #if ENABLED(JUNCTION_DEVIATION)
      SUBMENU(MSG_JUNCTION_DEVIATION, menu_advanced_jerk);
    #else
      SUBMENU(MSG_JERK, menu_advanced_jerk);
    #endif

    if (!printer.isPrinting()) {
      // M92 - Steps Per mm
      SUBMENU(MSG_STEPS_PER_MM, menu_advanced_steps_per_mm);

      #if HAS_BED_PROBE
        // M851 - Z Probe Offsets
        SUBMENU(MSG_ZPROBE_OFFSETS, menu_probe_offsets);
      #endif
    }

    #if HAS_TRINAMIC
      SUBMENU(MSG_TMC_DRIVERS, menu_tmc);
    #endif

  #endif // !SLIM_LCD_MENUS

  // M540 S - Abort on endstop hit when SD printing
  #if ENABLED(SD_ABORT_ON_ENDSTOP_HIT)
    EDIT_ITEM(bool, MSG_ENDSTOP_ABORT, &planner.flag.abort_on_endstop_hit);
  #endif

  #if ENABLED(EEPROM_SETTINGS) && DISABLED(SLIM_LCD_MENUS)
    CONFIRM_ITEM(MSG_INIT_EEPROM,
      MSG_BUTTON_INIT, MSG_BUTTON_CANCEL,
      []{ sound.feedback(eeprom.Init()); },
      lcdui.goto_previous_screen,
      GET_TEXT(MSG_INIT_EEPROM), (PGM_P)nullptr, PSTR("?")
    );
  #endif

  END_MENU();
}

#endif // HAS_LCD_MENU

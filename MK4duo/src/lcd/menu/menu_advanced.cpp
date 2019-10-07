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

//
// Advanced Settings Menus
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

void menu_tmc();

uint8_t lcd_extruder_total;

#if ENABLED(WORKSPACE_OFFSETS)
  //
  // Set the home offset based on the current_position
  //
  void lcd_set_home_offsets() {
    commands.inject_P(PSTR("M428"));
    lcdui.return_to_status();
  }
#endif

#if FILAMENT_RUNOUT_DISTANCE_MM > 0
  float lcd_runout_distance_mm;
#endif

#if ENABLED(VOLUMETRIC_EXTRUSION)
  bool lcd_volumetric_enabled;
#endif

#if ENABLED(VOLUMETRIC_EXTRUSION) || ENABLED(ADVANCED_PAUSE_FEATURE)
  //
  // Advanced Settings > Filament
  //
  void menu_advanced_filament() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    #if ENABLED(LIN_ADVANCE)
      EDIT_ITEM_INDEX(float52, MSG_ADVANCE_K, NO_INDEX, &planner.extruder_advance_K, 0, 999);
    #endif

    #if ENABLED(VOLUMETRIC_EXTRUSION)

      EDIT_ITEM_INDEX(bool, MSG_VOLUMETRIC_ENABLED, NO_INDEX, &lcd_volumetric_enabled, []{
        printer.setVolumetric(lcd_volumetric_enabled);
        tools.calculate_volumetric_multipliers;
      });

      if (printer.isVolumetric()) {
        LOOP_EXTRUDER()
          EDIT_ITEM_FAST_INDEX(float43, MSG_FILAMENT_DIAM, e, &tools.data.filament_size[e], 1.5f, 3.5f, tools.calculate_volumetric_multipliers);
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
        EDIT_ITEM_FAST_INDEX(float3, MSG_FILAMENT_UNLOAD, e, &advancedpause.data[e].unload_length, 0, extrude_maxlength);
        EDIT_ITEM_FAST_INDEX(float3, MSG_FILAMENT_LOAD, e, &advancedpause.data[e].load_length, 0, extrude_maxlength);
      }
    #endif

    #if FILAMENT_RUNOUT_DISTANCE_MM > 0
      EDIT_ITEM_INDEX(float3, MSG_RUNOUT_DISTANCE_MM, NO_INDEX, &lcd_runout_distance_mm, 1, 30, []{
        filamentrunout.set_runout_distance(lcd_runout_distance_mm);
      });
    #endif

    END_MENU();

  }

#endif // ENABLED(VOLUMETRIC_EXTRUSION) || ENABLED(ADVANCED_PAUSE_FEATURE)

//
// Advanced Settings > Temperature helpers
//

#if ENABLED(PID_AUTOTUNE_MENU)

  #if MAX_HOTEND > 0
    int16_t autotune_temp[MAX_HOTEND]           = { 200 };
  #endif
  #if MAX_BED > 0
    int16_t autotune_temp_bed[MAX_BED]          = { 60 };
  #endif
  #if MAX_CHAMBER > 0
    int16_t autotune_temp_chamber[MAX_CHAMBER]  = { 60 };
  #endif

  #if MAX_HOTEND > 0
    void _lcd_autotune_hotend() {
      char cmd[20];
      sprintf_P(cmd, PSTR("M303 U1 H%i S%i"), menu_edit_index, autotune_temp[menu_edit_index]);
      lcd_enqueue_one_now(cmd);
    }
    void updatePID_hotend() { hotends[menu_edit_index]->data.pid.update(); }
  #endif

  #if MAX_BED > 0
    void _lcd_autotune_bed() {
      char cmd[20];
      sprintf_P(cmd, PSTR("M303 U1 H-1 T%i S%i"), menu_edit_index, autotune_temp_bed[menu_edit_index]);
      lcd_enqueue_one_now(cmd);
    }
    void updatePID_bed() { beds[menu_edit_index]->data.pid.update(); }
  #endif

  #if MAX_CHAMBER > 0
    void _lcd_autotune_chamber() {
      char cmd[20];
      sprintf_P(cmd, PSTR("M303 U1 H-2 T%i S%i"), menu_edit_index, autotune_temp_chamber[menu_edit_index]);
      lcd_enqueue_one_now(cmd);
    }
    void updatePID_chamber() { chambers[menu_edit_index]->data.pid.update(); }
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
    EDIT_ITEM_INDEX(bool, MSG_AUTOTEMP, NO_INDEX, &planner.autotemp_enabled);
    EDIT_ITEM_INDEX(float3, MSG_MIN, NO_INDEX, &planner.autotemp_min, 0, hotends[0]->data.temp.max - 10);
    EDIT_ITEM_INDEX(float3, MSG_MAX, NO_INDEX, &planner.autotemp_max, 0, hotends[0]->data.temp.max - 10);
    EDIT_ITEM_INDEX(float52, MSG_FACTOR, NO_INDEX, &planner.autotemp_factor, 0, 1);
  #endif

  // PID Hotend
  #if MAX_HOTEND > 0
    LOOP_HOTEND() {
      if (hotends[h]->isUsePid()) {
        EDIT_ITEM_INDEX(float52, MSG_PID_P, h, &hotends[h]->data.pid.Kp, 1, 9990);
        EDIT_ITEM_INDEX(float52, MSG_PID_I, h, &hotends[h]->data.pid.Ki, 0.01f, 9990, updatePID_hotend);
        EDIT_ITEM_INDEX(float52, MSG_PID_D, h, &hotends[h]->data.pid.Kd, 1, 9990);
        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          EDIT_ITEM_INDEX(float3, MSG_PID_C, h, &hotends[h]->data.pid.Kc, 1, 9990);
        #endif
        EDIT_ITEM_FAST_INDEX(int3, MSG_PID_AUTOTUNE, h, &autotune_temp[h], 150, hotends[h]->data.temp.max - 10, _lcd_autotune_hotend);
      }
    }
  #endif

  // PID Bed
  #if MAX_BED > 0
    LOOP_BED() {
      if (beds[h]->isUsePid()) {
        EDIT_ITEM_INDEX(float52, "Bed " MSG_PID_P, h, &beds[h]->data.pid.Kp, 1, 9990);
        EDIT_ITEM_INDEX(float52, "Bed " MSG_PID_I, h, &beds[h]->data.pid.Ki, 0.01f, 9990, updatePID_bed);
        EDIT_ITEM_INDEX(float52, "Bed " MSG_PID_D, h, &beds[h]->data.pid.Kd, 1, 9990);
        EDIT_ITEM_FAST_INDEX(int3, "Bed " MSG_PID_AUTOTUNE, h, &autotune_temp_bed[h], 30, beds[h]->data.temp.max - 10, _lcd_autotune_bed);
      }
    }
  #endif

  // PID Chamber
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER() {
      if (chambers[h]->isUsePid()) {
        EDIT_ITEM_INDEX(float52, "Chamber " MSG_PID_P, h, &chambers[h]->data.pid.Kp, 1, 9990);
        EDIT_ITEM_INDEX(float52, "Chamber " MSG_PID_I, h, &chambers[h]->data.pid.Ki, 0.01f, 9990, updatePID_chamber);
        EDIT_ITEM_INDEX(float52, "Chamber " MSG_PID_D, h, &chambers[h]->data.pid.Kd, 1, 9990);
        EDIT_ITEM_FAST_INDEX(int3, "Chamber " MSG_PID_AUTOTUNE, h, &autotune_temp_chamber[h], 30, chambers[h]->data.temp.max - 10, _lcd_autotune_chamber);
      }
    }
  #endif

  END_MENU();

}

#if DISABLED(SLIM_LCD_MENUS)

  void _reset_acceleration_rates() {
    #if MECH(DELTA)
      mechanics.data.max_acceleration_mm_per_s2.y = mechanics.data.max_acceleration_mm_per_s2.z = mechanics.data.max_acceleration_mm_per_s2.x;
    #endif
    planner.reset_acceleration_rates();
  }
  void _reset_e_acceleration_rate() { if (menu_edit_index == tools.data.extruder.active) _reset_acceleration_rates(); }

  void _mechanics_refresh_positioning() {
    #if MECH(DELTA)
      mechanics.data.axis_steps_per_mm.y = mechanics.data.axis_steps_per_mm.z = mechanics.data.axis_steps_per_mm.x;
    #endif
    planner.refresh_positioning();
  }
  void _mechanics_refresh_e_positioning() {
    if (menu_edit_index == tools.data.extruder.active)
      _mechanics_refresh_positioning();
    else
      mechanics.steps_to_mm.e[menu_edit_index] = RECIPROCAL(mechanics.data.axis_steps_per_mm.e[menu_edit_index]);
  }

  #if MECH(DELTA)
    void _mechanics_set_feedrate() {
      mechanics.data.max_feedrate_mm_s.y = mechanics.data.max_feedrate_mm_s.z = mechanics.data.max_feedrate_mm_s.x;
    }
  #endif

  // M203 / M205 Velocity options
  void menu_advanced_velocity() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    // M203 Max Feedrate
    #if MECH(DELTA)
      EDIT_ITEM_FAST_INDEX(float3, MSG_VMAX, NO_INDEX, &mechanics.data.max_feedrate_mm_s.x, 1, 9999, _mechanics_set_feedrate);
    #else
      EDIT_ITEM_FAST_INDEX(float3, MSG_VMAX MSG_X, NO_INDEX, &mechanics.data.max_feedrate_mm_s.x, 1, 9999);
      EDIT_ITEM_FAST_INDEX(float3, MSG_VMAX MSG_Y, NO_INDEX, &mechanics.data.max_feedrate_mm_s.y, 1, 9999);
      EDIT_ITEM_FAST_INDEX(float3, MSG_VMAX MSG_Z, NO_INDEX, &mechanics.data.max_feedrate_mm_s.z, 1, 9999);
    #endif
    LOOP_EXTRUDER()
      EDIT_ITEM_FAST_INDEX(float3, MSG_VMAX MSG_E, e, &mechanics.data.max_feedrate_mm_s.e[e], 1, 9999);

    // M205 S Min Feedrate
    EDIT_ITEM_FAST_INDEX(float3, MSG_VMIN, NO_INDEX, &mechanics.data.min_feedrate_mm_s, 0, 999);

    // M205 T Min Travel Feedrate
    EDIT_ITEM_FAST_INDEX(float3, MSG_VTRAV_MIN, NO_INDEX, &mechanics.data.min_travel_feedrate_mm_s, 0, 999);

    END_MENU();
  }

  // M201 / M204 Accelerations
  void menu_advanced_acceleration() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    // M204 P Acceleration
    EDIT_ITEM_FAST_INDEX(float5_25, MSG_ACC, NO_INDEX, &mechanics.data.acceleration, 25, 99000);

    // M204 R Retract Acceleration
    LOOP_EXTRUDER()
      EDIT_ITEM_FAST_INDEX(float5, MSG_A_RETRACT MSG_E, e, &mechanics.data.retract_acceleration[e], 100, 99000);

    // M204 T Travel Acceleration
    EDIT_ITEM_FAST_INDEX(float5_25, MSG_A_TRAVEL, NO_INDEX, &mechanics.data.travel_acceleration, 25, 99000);

    // M201 settings
    #if MECH(DELTA)
      EDIT_ITEM_FAST_INDEX(long5_25, MSG_AMAX, NO_INDEX, &mechanics.data.max_acceleration_mm_per_s2.x, 100, 99000, _reset_acceleration_rates);
    #else
      EDIT_ITEM_FAST_INDEX(long5_25, MSG_AMAX MSG_X, NO_INDEX, &mechanics.data.max_acceleration_mm_per_s2.x, 100, 99000, _reset_acceleration_rates);
      EDIT_ITEM_FAST_INDEX(long5_25, MSG_AMAX MSG_Y, NO_INDEX, &mechanics.data.max_acceleration_mm_per_s2.y, 100, 99000, _reset_acceleration_rates);
      EDIT_ITEM_FAST_INDEX(long5_25, MSG_AMAX MSG_Z, NO_INDEX, &mechanics.data.max_acceleration_mm_per_s2.z, 10, 99000, _reset_acceleration_rates);
    #endif

    LOOP_EXTRUDER()
      EDIT_ITEM_FAST_INDEX(long5_25, MSG_AMAX MSG_E, e, &mechanics.data.max_acceleration_mm_per_s2.e[e], 100, 99000, _reset_e_acceleration_rate);
    
    END_MENU();
  }

  #if HAS_CLASSIC_JERK
    void _mechanics_set_jerk() {
      mechanics.data.max_jerk.y = mechanics.data.max_jerk.z = mechanics.data.max_jerk.x;
    }
  #endif

  // M205 Jerk
  void menu_advanced_jerk() {
    START_MENU();
    BACK_ITEM(MSG_MOTION);

    #if ENABLED(JUNCTION_DEVIATION)
      #if ENABLED(LIN_ADVANCE)
        EDIT_ITEM_INDEX(float43, MSG_JUNCTION_MM, NO_INDEX, &mechanics.data.junction_deviation_mm, 0.01f, 0.3f, mechanics.recalculate_max_e_jerk);
      #else
        EDIT_ITEM_INDEX(float43, MSG_JUNCTION_MM, NO_INDEX, &mechanics.data.junction_deviation_mm, 0.01f, 0.3f);
      #endif
    #endif

    #if HAS_CLASSIC_JERK

      #if MECH(DELTA)
        EDIT_ITEM_INDEX(float3, MSG_JERK, NO_INDEX, &mechanics.data.max_jerk.x, 1, 990, _mechanics_set_jerk);
      #else
        EDIT_ITEM_FAST_INDEX(float3, MSG_VA_JERK, NO_INDEX, &mechanics.data.max_jerk.x, 1, 990);
        EDIT_ITEM_FAST_INDEX(float3, MSG_VB_JERK, NO_INDEX, &mechanics.data.max_jerk.y, 1, 990);
        EDIT_ITEM_FAST_INDEX(float52sign, MSG_VC_JERK, NO_INDEX, &mechanics.data.max_jerk.z, 0.1, 990);
      #endif

      #if DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
        LOOP_EXTRUDER()
          EDIT_ITEM_FAST_INDEX(float3, MSG_VE_JERK MSG_E, e, &mechanics.data.max_jerk.e[e], 1, 990);
      #endif // DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)

    #endif // AS_CLASSIC_JERK

    END_MENU();
  }

  // M92 Steps-per-mm
  void menu_advanced_steps_per_mm() {
    START_MENU();
    BACK_ITEM(MSG_ADVANCED_SETTINGS);

    #if MECH(DELTA)
      EDIT_ITEM_FAST_INDEX(float51, MSG_STEPS_PER_MM, NO_INDEX, &mechanics.data.axis_steps_per_mm.x, 5, 9999, _mechanics_refresh_positioning);
    #else
      EDIT_ITEM_FAST_INDEX(float51, MSG_ASTEPS, NO_INDEX, &mechanics.data.axis_steps_per_mm.x, 5, 9999, _mechanics_refresh_positioning);
      EDIT_ITEM_FAST_INDEX(float51, MSG_BSTEPS, NO_INDEX, &mechanics.data.axis_steps_per_mm.y, 5, 9999, _mechanics_refresh_positioning);
      EDIT_ITEM_FAST_INDEX(float51, MSG_CSTEPS, NO_INDEX, &mechanics.data.axis_steps_per_mm.z, 5, 9999, _mechanics_refresh_positioning);
    #endif

    LOOP_EXTRUDER()
      EDIT_ITEM_FAST_INDEX(float51, MSG_ESTEPS, e, &mechanics.data.axis_steps_per_mm.e[e], 5, 9999, _mechanics_refresh_e_positioning);

    END_MENU();
  }

  #if ENABLED(EEPROM_SETTINGS)

    static void lcd_init_eeprom_confirm() {
      do_select_screen(
        PSTR(MSG_BUTTON_INIT), PSTR(MSG_BUTTON_CANCEL),
        []{ sound.feedback(eeprom.Init()); },
        lcdui.goto_previous_screen,
        PSTR(MSG_INIT_EEPROM), nullptr, PSTR("?")
      );
    }

  #endif

#endif // !SLIM_LCD_MENUS

void menu_advanced_settings() {

  lcd_extruder_total = tools.data.extruder.total;

  #if ENABLED(VOLUMETRIC_EXTRUSION)
    lcd_volumetric_enabled = printer.isVolumetric();
  #endif

  #if FILAMENT_RUNOUT_DISTANCE_MM > 0
    lcd_runout_distance_mm = filamentrunout.runout_distance();
  #endif

  START_MENU();
  BACK_ITEM(MSG_CONFIGURATION);

  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    SUBMENU(MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
  #elif HAS_BED_PROBE
    EDIT_ITEM_INDEX(float52, MSG_ZPROBE_ZOFFSET, NO_INDEX, &probe.data.offset.z, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX);
  #endif

  #if DISABLED(SLIM_LCD_MENUS)

    if (printer.mode == PRINTER_MODE_FFF) {
      #if HAS_LINEAR_EXTRUDER
        EDIT_ITEM_INDEX(uint8, MSG_MAX_EXTRUDERS, NO_INDEX, &lcd_extruder_total, 0, MAX_EXTRUDER, []{
          tools.change_number_extruder(lcd_extruder_total);
        });
      #endif
      SUBMENU(MSG_TEMPERATURE, menu_advanced_temperature);
      #if ENABLED(VOLUMETRIC_EXTRUSION) || ENABLED(ADVANCED_PAUSE_FEATURE)
        SUBMENU(MSG_FILAMENT, menu_advanced_filament);
      #elif ENABLED(LIN_ADVANCE)
        EDIT_ITEM_INDEX(float52, MSG_ADVANCE_K, NO_INDEX, &planner.extruder_advance_K, 0, 999);
      #endif
    }
      
    #if ENABLED(WORKSPACE_OFFSETS)
      //
      // Set Home Offsets
      //
      ACTION_ITEM(MSG_SET_HOME_OFFSETS, lcd_set_home_offsets);
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
    }

    #if HAS_TRINAMIC
      SUBMENU(MSG_TMC_DRIVERS, menu_tmc);
    #endif

  #endif // !SLIM_LCD_MENUS

  // M540 S - Abort on endstop hit when SD printing
  #if ENABLED(SD_ABORT_ON_ENDSTOP_HIT)
    EDIT_ITEM_INDEX(bool, MSG_ENDSTOP_ABORT, NO_INDEX, &planner.abort_on_endstop_hit);
  #endif

  #if ENABLED(EEPROM_SETTINGS) && DISABLED(SLIM_LCD_MENUS)
    SUBMENU(MSG_INIT_EEPROM, lcd_init_eeprom_confirm);
  #endif

  END_MENU();
}

#endif // HAS_LCD_MENU

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

//
// Advanced Settings Menus
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

#if ENABLED(WORKSPACE_OFFSETS)
  //
  // Set the home offset based on the current_position
  //
  void _lcd_set_home_offsets() {
    commands.enqueue_and_echo_P(PSTR("M428"));
    lcdui.return_to_status();
  }
#endif

#if ENABLED(VOLUMETRIC_EXTRUSION)
  bool lcd_volumetric_enabled = printer.isVolumetric();
  void lcd_set_volumetric() {
    printer.setVolumetric(lcd_volumetric_enabled);
    tools.calculate_volumetric_multipliers;
  }
#endif

#if ENABLED(VOLUMETRIC_EXTRUSION) || ENABLED(ADVANCED_PAUSE_FEATURE)
  //
  // Advanced Settings > Filament
  //
  void menu_advanced_filament() {
    START_MENU();
    MENU_BACK(MSG_ADVANCED_SETTINGS);

    #if ENABLED(LIN_ADVANCE)
      MENU_ITEM_EDIT(float3, MSG_ADVANCE_K, &planner.extruder_advance_K, 0, 999);
    #endif

    #if ENABLED(VOLUMETRIC_EXTRUSION)

      lcd_volumetric_enabled = printer.isVolumetric();
      MENU_ITEM_EDIT_CALLBACK(bool, MSG_VOLUMETRIC_ENABLED, &lcd_volumetric_enabled, lcd_set_volumetric);

      if (printer.isVolumetric()) {
        #if EXTRUDERS == 1
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM, &tools.filament_size[0], 1.5f, 3.5f, tools.calculate_volumetric_multipliers);
        #else // EXTRUDERS > 1
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM, &tools.filament_size[tools.active_extruder], 1.5f, 3.5f, tools.calculate_volumetric_multipliers);
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E1, &tools.filament_size[0], 1.5f, 3.5f, tools.calculate_volumetric_multipliers);
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E2, &tools.filament_size[1], 1.5f, 3.5f, tools.calculate_volumetric_multipliers);
          #if EXTRUDERS > 2
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E3, &tools.filament_size[2], 1.5f, 3.5f, tools.calculate_volumetric_multipliers);
            #if EXTRUDERS > 3
              MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E4, &tools.filament_size[3], 1.5f, 3.5f, tools.calculate_volumetric_multipliers);
              #if EXTRUDERS > 4
                MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E5, &tools.filament_size[4], 1.5f, 3.5f, tools.calculate_volumetric_multipliers);
                #if EXTRUDERS > 5
                  MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E6, &tools.filament_size[5], 1.5f, 3.5f, tools.calculate_volumetric_multipliers);
                #endif // EXTRUDERS > 5
              #endif // EXTRUDERS > 4
            #endif // EXTRUDERS > 3
          #endif // EXTRUDERS > 2
        #endif // EXTRUDERS > 1
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

      #if EXTRUDERS == 1
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_UNLOAD, &advancedpause.data[0].unload_length, 0, extrude_maxlength);
      #else // EXTRUDERS > 1
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_UNLOAD, &advancedpause.data[tools.active_extruder].unload_length, 0, extrude_maxlength);
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_UNLOAD MSG_DIAM_E1, &advancedpause.data[0].unload_length, 0, extrude_maxlength);
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_UNLOAD MSG_DIAM_E2, &advancedpause.data[1].unload_length, 0, extrude_maxlength);
        #if EXTRUDERS > 2
          MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_UNLOAD MSG_DIAM_E3, &advancedpause.data[2].unload_length, 0, extrude_maxlength);
          #if EXTRUDERS > 3
            MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_UNLOAD MSG_DIAM_E4, &advancedpause.data[3].unload_length, 0, extrude_maxlength);
            #if EXTRUDERS > 4
              MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_UNLOAD MSG_DIAM_E5, &advancedpause.data[4].unload_length, 0, extrude_maxlength);
              #if EXTRUDERS > 5
                MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_UNLOAD MSG_DIAM_E6, &advancedpause.data[5].unload_length, 0, extrude_maxlength);
              #endif // EXTRUDERS > 5
            #endif // EXTRUDERS > 4
          #endif // EXTRUDERS > 3
        #endif // EXTRUDERS > 2
      #endif // EXTRUDERS > 1

      #if EXTRUDERS == 1
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_LOAD, &advancedpause.data[0].load_length, 0, extrude_maxlength);
      #else // EXTRUDERS > 1
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_LOAD, &advancedpause.data[tools.active_extruder].load_length, 0, extrude_maxlength);
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_LOAD MSG_DIAM_E1, &advancedpause.data[0].load_length, 0, extrude_maxlength);
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_LOAD MSG_DIAM_E2, &advancedpause.data[1].load_length, 0, extrude_maxlength);
        #if EXTRUDERS > 2
          MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_LOAD MSG_DIAM_E3, &advancedpause.data[2].load_length, 0, extrude_maxlength);
          #if EXTRUDERS > 3
            MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_LOAD MSG_DIAM_E4, &advancedpause.data[3].load_length, 0, extrude_maxlength);
            #if EXTRUDERS > 4
              MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_LOAD MSG_DIAM_E5, &advancedpause.data[4].load_length, 0, extrude_maxlength);
              #if EXTRUDERS > 5
                MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_FILAMENT_LOAD MSG_DIAM_E6, &advancedpause.data[5].load_length, 0, extrude_maxlength);
              #endif // EXTRUDERS > 5
            #endif // EXTRUDERS > 4
          #endif // EXTRUDERS > 3
        #endif // EXTRUDERS > 2
      #endif // EXTRUDERS > 1
    #endif

    END_MENU();

  }

#endif // ENABLED(VOLUMETRIC_EXTRUSION) || ENABLED(ADVANCED_PAUSE_FEATURE)

//
// Advanced Settings > Temperature helpers
//

#if ENABLED(PID_AUTOTUNE_MENU)

  int16_t autotune_temp[HOTENDS] = ARRAY_BY_HOTENDS(200);
  int16_t autotune_temp_bed = 60;

  void _lcd_autotune(int16_t h) {
    char cmd[30];

    sprintf_P(cmd, PSTR("M303 U1 H%i S%i"), h,
      #if HAS_HEATER_BED
        h < 0 ? autotune_temp_bed : autotune_temp[h]
      #else
        autotune_temp[h]
      #endif
    );
    lcd_enqueue_command(cmd);
  }

#endif //PID_AUTOTUNE_MENU

#define _DEFINE_PIDTEMP_BASE_FUNCS(N) \
  void updatePID_H ## N() { heaters[N].pid.update(); } \

#if ENABLED(PID_AUTOTUNE_MENU)
  #if HAS_HEATER_BED
    #define DEFINE_PIDBED_FUNCS() \
      _DEFINE_PIDTEMP_BASE_FUNCS(BED_INDEX); \
      void lcd_autotune_callback_BED() { _lcd_autotune(-1); }
  #endif

  #define DEFINE_PIDTEMP_FUNCS(N) \
    _DEFINE_PIDTEMP_BASE_FUNCS(N); \
    void lcd_autotune_callback_H ## N() { _lcd_autotune(N); } typedef void _pid_##N##_void
#else
  #if HAS_HEATER_BED
    #define DEFINE_PIDBED_FUNCS() _DEFINE_PIDTEMP_BASE_FUNCS(BED_INDEX)
  #endif

  #define DEFINE_PIDTEMP_FUNCS(N) _DEFINE_PIDTEMP_BASE_FUNCS(N) typedef void _pid_##N##_void
#endif

DEFINE_PIDTEMP_FUNCS(0);
#if HOTENDS > 1
  DEFINE_PIDTEMP_FUNCS(1);
  #if HOTENDS > 2
    DEFINE_PIDTEMP_FUNCS(2);
    #if HOTENDS > 3
      DEFINE_PIDTEMP_FUNCS(3);
    #endif // HOTENDS > 3
  #endif // HOTENDS > 2
#endif // HOTENDS > 1

#if HAS_HEATER_BED
  DEFINE_PIDBED_FUNCS();
#endif

//
// Advanced Settings > Temperature
//
void menu_advanced_temperature() {
  START_MENU();
  MENU_BACK(MSG_ADVANCED_SETTINGS);
  //
  // Autotemp, Min, Max, Fact
  //
  #if ENABLED(AUTOTEMP) && (HAS_TEMP_0)
    MENU_ITEM_EDIT(bool, MSG_AUTOTEMP, &planner.autotemp_enabled);
    MENU_ITEM_EDIT(float3, MSG_MIN, &planner.autotemp_min, 0, heaters[0].data.maxtemp - 15);
    MENU_ITEM_EDIT(float3, MSG_MAX, &planner.autotemp_max, 0, heaters[0].data.maxtemp - 15);
    MENU_ITEM_EDIT(float52, MSG_FACTOR, &planner.autotemp_factor, 0, 1);
  #endif

  //
  // PID-P, PID-I, PID-D, PID-C, PID Autotune
  // PID-P H1, PID-I H1, PID-D H1, PID-C H1, PID Autotune H1
  // PID-P H2, PID-I H2, PID-D H2, PID-C H2, PID Autotune H2
  // PID-P H3, PID-I H3, PID-D H3, PID-C H3, PID Autotune H3
  // PID-P H4, PID-I H4, PID-D H4, PID-C H4, PID Autotune H4
  //
  #define _PID_BASE_MENU_ITEMS(HLABEL, hindex) \
    MENU_ITEM_EDIT(float52, MSG_PID_P HLABEL, &heaters[hindex].pid.Kp, 1, 9990); \
    MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_I HLABEL, &heaters[hindex].pid.Ki, 0.01f, 9990, updatePID_H ## hindex); \
    MENU_ITEM_EDIT(float52, MSG_PID_D HLABEL, &heaters[hindex].pid.Kd, 1, 9990)

  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    #define _PID_MENU_ITEMS(HLABEL, hindex) \
      _PID_BASE_MENU_ITEMS(HLABEL, hindex); \
      MENU_ITEM_EDIT(float3, MSG_PID_C HLABEL, &heaters[hindex].pid.Kc, 1, 9990)
  #else
    #define _PID_MENU_ITEMS(HLABEL, hindex) _PID_BASE_MENU_ITEMS(HLABEL, hindex)
  #endif

  #if ENABLED(PID_AUTOTUNE_MENU)
    #define PID_MENU_ITEMS(HLABEL, hindex) \
      _PID_MENU_ITEMS(HLABEL, hindex); \
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_PID_AUTOTUNE HLABEL, &autotune_temp[hindex], 150, heaters[hindex].data.maxtemp - 15, lcd_autotune_callback_H ## hindex)

    #if HAS_TEMP_BED
      #define PID_BED_MENU_ITEMS() \
        _PID_BASE_MENU_ITEMS(" BED", BED_INDEX); \
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_PID_AUTOTUNE " BED", &autotune_temp_bed, 30, heaters[BED_INDEX].data.maxtemp - 15, lcd_autotune_callback_BED)
    #endif
  #else
    #define PID_MENU_ITEMS(HLABEL, hindex)  _PID_MENU_ITEMS(HLABEL, hindex)
    #define PID_BED_MENU_ITEMS()            _PID_BASE_MENU_ITEMS("BED", BED_INDEX)
  #endif

  if (heaters[0].isUsePid()) { PID_MENU_ITEMS("", 0); }
  #if HOTENDS > 1
    if (heaters[1].isUsePid()) { PID_MENU_ITEMS(MSG_H1, 1); }
    #if HOTENDS > 2
      if (heaters[0].isUsePid()) { PID_MENU_ITEMS(MSG_H2, 2); }
      #if HOTENDS > 3
        if (heaters[0].isUsePid()) { PID_MENU_ITEMS(MSG_H3, 3); }
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1

  #if HAS_TEMP_BED
    if (heaters[BED_INDEX].isUsePid()) { PID_BED_MENU_ITEMS(); }
  #endif

  END_MENU();

}

#if DISABLED(SLIM_LCD_MENUS)

  void _reset_acceleration_rates() {
    #if MECH(DELTA)
      mechanics.data.max_acceleration_mm_per_s2[Y_AXIS] = mechanics.data.max_acceleration_mm_per_s2[Z_AXIS] = mechanics.data.max_acceleration_mm_per_s2[X_AXIS];
    #endif
    planner.reset_acceleration_rates();
  }
  #if EXTRUDERS > 1
    void _reset_e_acceleration_rate(const uint8_t e) { if (e == tools.active_extruder) _reset_acceleration_rates(); }
    void _reset_e0_acceleration_rate() { _reset_e_acceleration_rate(0); }
    void _reset_e1_acceleration_rate() { _reset_e_acceleration_rate(1); }
    #if EXTRUDERS > 2
      void _reset_e2_acceleration_rate() { _reset_e_acceleration_rate(2); }
      #if EXTRUDERS > 3
        void _reset_e3_acceleration_rate() { _reset_e_acceleration_rate(3); }
        #if EXTRUDERS > 4
          void _reset_e4_acceleration_rate() { _reset_e_acceleration_rate(4); }
          #if EXTRUDERS > 5
            void _reset_e5_acceleration_rate() { _reset_e_acceleration_rate(5); }
          #endif // EXTRUDERS > 5
        #endif // EXTRUDERS > 4
      #endif // EXTRUDERS > 3
    #endif // EXTRUDERS > 2
  #endif // EXTRUDERS > 1

  void _mechanics_refresh_positioning() {
    #if MECH(DELTA)
      mechanics.data.axis_steps_per_mm[Y_AXIS] = mechanics.data.axis_steps_per_mm[Z_AXIS] = mechanics.data.axis_steps_per_mm[X_AXIS];
    #endif
    planner.refresh_positioning();
  }
  #if EXTRUDERS > 1
    void _mechanics_refresh_e_positioning(const uint8_t e) {
      if (e == tools.active_extruder)
        _mechanics_refresh_positioning();
      else
        mechanics.steps_to_mm[E_AXIS + e] = RECIPROCAL(mechanics.data.axis_steps_per_mm[E_AXIS + e]);
    }
    void _mechanics_refresh_e0_positioning() { _mechanics_refresh_e_positioning(0); }
    void _mechanics_refresh_e1_positioning() { _mechanics_refresh_e_positioning(1); }
    #if EXTRUDERS > 2
      void _mechanics_refresh_e2_positioning() { _mechanics_refresh_e_positioning(2); }
      #if EXTRUDERS > 3
        void _mechanics_refresh_e3_positioning() { _mechanics_refresh_e_positioning(3); }
        #if EXTRUDERS > 4
          void _mechanics_refresh_e4_positioning() { _mechanics_refresh_e_positioning(4); }
          #if EXTRUDERS > 5
            void _mechanics_refresh_e5_positioning() { _mechanics_refresh_e_positioning(5); }
          #endif // EXTRUDERS > 5
        #endif // EXTRUDERS > 4
      #endif // EXTRUDERS > 3
    #endif // EXTRUDERS > 2
  #endif // EXTRUDERS > 1

  #if MECH(DELTA)
    void _mechanics_set_feedrate() {
      mechanics.data.max_feedrate_mm_s[Y_AXIS] = mechanics.data.max_feedrate_mm_s[Z_AXIS] = mechanics.data.max_feedrate_mm_s[X_AXIS];
    }
  #endif

  // M203 / M205 Velocity options
  void menu_advanced_velocity() {
    START_MENU();
    MENU_BACK(MSG_ADVANCED_SETTINGS);

    // M203 Max Feedrate
    #if MECH(DELTA)
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float3, MSG_VMAX, &mechanics.data.max_feedrate_mm_s[X_AXIS], 1, 999, _mechanics_set_feedrate);
    #else
      MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_X, &mechanics.data.max_feedrate_mm_s[X_AXIS], 1, 999);
      MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_Y, &mechanics.data.max_feedrate_mm_s[Y_AXIS], 1, 999);
      MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_Z, &mechanics.data.max_feedrate_mm_s[Z_AXIS], 1, 999);
    #endif

    #if EXTRUDERS > 1
      MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_E, &mechanics.data.max_feedrate_mm_s[E_AXIS + tools.active_extruder], 1, 999);
      MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_E1, &mechanics.data.max_feedrate_mm_s[E_AXIS], 1, 999);
      MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_E2, &mechanics.data.max_feedrate_mm_s[E_AXIS + 1], 1, 999);
      #if EXTRUDERS > 2
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_E3, &mechanics.data.max_feedrate_mm_s[E_AXIS + 2], 1, 999);
        #if EXTRUDERS > 3
          MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_E4, &mechanics.data.max_feedrate_mm_s[E_AXIS + 3], 1, 999);
          #if EXTRUDERS > 4
            MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_E5, &mechanics.data.max_feedrate_mm_s[E_AXIS + 4], 1, 999);
            #if EXTRUDERS > 5
              MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_E6, &mechanics.data.max_feedrate_mm_s[E_AXIS + 5], 1, 999);
            #endif // EXTRUDERS > 5
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #else
      MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMAX MSG_E, &mechanics.data.max_feedrate_mm_s[E_AXIS], 1, 999);
    #endif

    // M205 S Min Feedrate
    MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VMIN, &mechanics.data.min_feedrate_mm_s, 0, 999);

    // M205 T Min Travel Feedrate
    MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VTRAV_MIN, &mechanics.data.min_travel_feedrate_mm_s, 0, 999);

    END_MENU();
  }

  // M201 / M204 Accelerations
  void menu_advanced_acceleration() {
    START_MENU();
    MENU_BACK(MSG_ADVANCED_SETTINGS);

    // M204 P Acceleration
    MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_ACC, &mechanics.data.acceleration, 10, 99000);

    // M204 R Retract Acceleration
    #if EXTRUDERS > 1
      MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E, &mechanics.data.retract_acceleration[tools.active_extruder], 100, 99000);
      MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E1, &mechanics.data.retract_acceleration[0], 100, 99000);
      MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E2, &mechanics.data.retract_acceleration[1], 100, 99000);
      #if EXTRUDERS > 2
        MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E3, &mechanics.data.retract_acceleration[2], 100, 99000);
        #if EXTRUDERS > 3
          MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E4, &mechanics.data.retract_acceleration[3], 100, 99000);
          #if EXTRUDERS > 4
            MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E5, &mechanics.data.retract_acceleration[4], 100, 99000);
            #if EXTRUDERS > 5
              MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E6, &mechanics.data.retract_acceleration[5], 100, 99000);
            #endif // EXTRUDERS > 5
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #else
      MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E, &mechanics.data.retract_acceleration[0], 100, 99000);
    #endif

    // M204 T Travel Acceleration
    MENU_MULTIPLIER_ITEM_EDIT(float5, MSG_A_TRAVEL, &mechanics.data.travel_acceleration, 100, 99000);

    // M201 settings
    #if MECH(DELTA)
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX, &mechanics.data.max_acceleration_mm_per_s2[X_AXIS], 100, 99000, _reset_acceleration_rates);
    #else
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_X, &mechanics.data.max_acceleration_mm_per_s2[X_AXIS], 100, 99000, _reset_acceleration_rates);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Y, &mechanics.data.max_acceleration_mm_per_s2[Y_AXIS], 100, 99000, _reset_acceleration_rates);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Z, &mechanics.data.max_acceleration_mm_per_s2[Z_AXIS], 10, 99000, _reset_acceleration_rates);
    #endif

    #if EXTRUDERS > 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &mechanics.data.max_acceleration_mm_per_s2[E_AXIS + tools.active_extruder], 100, 99000, _reset_acceleration_rates);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E1, &mechanics.data.max_acceleration_mm_per_s2[E_AXIS], 100, 99000, _reset_e0_acceleration_rate);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E2, &mechanics.data.max_acceleration_mm_per_s2[E_AXIS + 1], 100, 99000, _reset_e1_acceleration_rate);
      #if EXTRUDERS > 2
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E3, &mechanics.data.max_acceleration_mm_per_s2[E_AXIS + 2], 100, 99000, _reset_e2_acceleration_rate);
        #if EXTRUDERS > 3
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E4, &mechanics.data.max_acceleration_mm_per_s2[E_AXIS + 3], 100, 99000, _reset_e3_acceleration_rate);
          #if EXTRUDERS > 4
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E5, &mechanics.data.max_acceleration_mm_per_s2[E_AXIS + 4], 100, 99000, _reset_e4_acceleration_rate);
            #if EXTRUDERS > 5
              MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E6, &mechanics.data.max_acceleration_mm_per_s2[E_AXIS + 5], 100, 99000, _reset_e5_acceleration_rate);
            #endif // EXTRUDERS > 5
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #else
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &mechanics.data.max_acceleration_mm_per_s2[E_AXIS], 100, 99000, _reset_acceleration_rates);
    #endif

    END_MENU();
  }

  #if HAS_CLASSIC_JERK
    void _mechanics_set_jerk() {
      mechanics.data.max_jerk[Y_AXIS] = mechanics.data.max_jerk[Z_AXIS] = mechanics.data.max_jerk[X_AXIS];
    }
  #endif

  // M205 Jerk
  void menu_advanced_jerk() {
    START_MENU();
    MENU_BACK(MSG_MOTION);

    #if ENABLED(JUNCTION_DEVIATION)
      #if ENABLED(LIN_ADVANCE)
        MENU_ITEM_EDIT_CALLBACK(float43, MSG_JUNCTION_MM, &mechanics.data.junction_deviation_mm, 0.01f, 0.3f, mechanics.recalculate_max_e_jerk);
      #else
        MENU_ITEM_EDIT(float43, MSG_JUNCTION_MM, &mechanics.data.junction_deviation_mm, 0.01f, 0.3f);
      #endif
    #endif

    #if HAS_CLASSIC_JERK
      #if MECH(DELTA)
        MENU_ITEM_EDIT_CALLBACK(float3, MSG_JERK, &mechanics.data.max_jerk[X_AXIS], 1, 990, _mechanics_set_jerk);
      #else
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VA_JERK, &mechanics.data.max_jerk[X_AXIS], 1, 990);
        MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VB_JERK, &mechanics.data.max_jerk[Y_AXIS], 1, 990);
        MENU_MULTIPLIER_ITEM_EDIT(float52sign, MSG_VC_JERK, &mechanics.data.max_jerk[Z_AXIS], 0.1, 990);
      #endif

      #if DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
        #if EXTRUDERS > 1
          MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VE_JERK MSG_E, &mechanics.data.max_jerk[E_AXIS + tools.active_extruder], 1, 990);
          MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VE_JERK MSG_E1, &mechanics.data.max_jerk[E_AXIS], 1, 990);
          MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VE_JERK MSG_E2, &mechanics.data.max_jerk[E_AXIS + 1], 1, 990);
          #if EXTRUDERS > 2
            MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VE_JERK MSG_E3, &mechanics.data.max_jerk[E_AXIS + 2], 1, 990);
            #if EXTRUDERS > 3
              MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VE_JERK MSG_E4, &mechanics.data.max_jerk[E_AXIS + 3], 1, 990);
              #if EXTRUDERS > 4
                MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VE_JERK MSG_E5, &mechanics.data.max_jerk[E_AXIS + 4], 1, 990);
                #if EXTRUDERS > 5
                  MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VE_JERK MSG_E6, &mechanics.data.max_jerk[E_AXIS + 5], 1, 990);
                #endif // EXTRUDERS > 5
              #endif // EXTRUDERS > 4
            #endif // EXTRUDERS > 3
          #endif // EXTRUDERS > 2
        #else
          MENU_MULTIPLIER_ITEM_EDIT(float3, MSG_VE_JERK, &mechanics.data.max_jerk[E_AXIS], 1, 990);
        #endif
      #endif // DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
    #endif // AS_CLASSIC_JERK

    END_MENU();
  }

  // M92 Steps-per-mm
  void menu_advanced_steps_per_mm() {
    START_MENU();
    MENU_BACK(MSG_ADVANCED_SETTINGS);

    #if MECH(DELTA)
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_STEPS_PER_MM, &mechanics.data.axis_steps_per_mm[X_AXIS], 5, 9999, _mechanics_refresh_positioning);
    #else
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_ASTEPS, &mechanics.data.axis_steps_per_mm[X_AXIS], 5, 9999, _mechanics_refresh_positioning);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_BSTEPS, &mechanics.data.axis_steps_per_mm[Y_AXIS], 5, 9999, _mechanics_refresh_positioning);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_CSTEPS, &mechanics.data.axis_steps_per_mm[Z_AXIS], 5, 9999, _mechanics_refresh_positioning);
    #endif

    #if EXTRUDERS > 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_ESTEPS, &mechanics.data.axis_steps_per_mm[E_AXIS + tools.active_extruder], 5, 9999, _mechanics_refresh_positioning);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E1STEPS, &mechanics.data.axis_steps_per_mm[E_AXIS], 5, 9999, _mechanics_refresh_e0_positioning);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E2STEPS, &mechanics.data.axis_steps_per_mm[E_AXIS + 1], 5, 9999, _mechanics_refresh_e1_positioning);
      #if EXTRUDERS > 2
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E3STEPS, &mechanics.data.axis_steps_per_mm[E_AXIS + 2], 5, 9999, _mechanics_refresh_e2_positioning);
        #if EXTRUDERS > 3
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E4STEPS, &mechanics.data.axis_steps_per_mm[E_AXIS + 3], 5, 9999, _mechanics_refresh_e3_positioning);
          #if EXTRUDERS > 4
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E5STEPS, &mechanics.data.axis_steps_per_mm[E_AXIS + 4], 5, 9999, _mechanics_refresh_e4_positioning);
            #if EXTRUDERS > 5
              MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E6STEPS, &mechanics.data.axis_steps_per_mm[E_AXIS + 5], 5, 9999, _mechanics_refresh_e4_positioning);
            #endif // EXTRUDERS > 5
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #else
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_ESTEPS, &mechanics.data.axis_steps_per_mm[E_AXIS], 5, 9999, _mechanics_refresh_positioning);
    #endif

    END_MENU();
  }

  #if ENABLED(EEPROM_SETTINGS)

    static void lcd_init_eeprom() {
      sound.feedback(eeprom.Init());
      lcdui.goto_previous_screen();
    }

    static void lcd_init_eeprom_confirm() {
      START_MENU();
      MENU_BACK(MSG_ADVANCED_SETTINGS);
      MENU_ITEM(function, MSG_INIT_EEPROM, lcd_init_eeprom);
      END_MENU();
    }

  #endif

#endif // !SLIM_LCD_MENUS

void menu_advanced_settings() {
  START_MENU();
  MENU_BACK(MSG_CONFIGURATION);

  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    MENU_ITEM(submenu, MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
  #elif HAS_BED_PROBE
    MENU_ITEM_EDIT(float52, MSG_ZPROBE_ZOFFSET, &probe.data.offset[Z_AXIS], Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX);
  #endif

  #if DISABLED(SLIM_LCD_MENUS)

    #if ENABLED(WORKSPACE_OFFSETS)
      //
      // Set Home Offsets
      //
      MENU_ITEM(function, MSG_SET_HOME_OFFSETS, _lcd_set_home_offsets);
    #endif

    // M203 / M205 - Feedrate items
    MENU_ITEM(submenu, MSG_VELOCITY, menu_advanced_velocity);

    // M201 - Acceleration items
    MENU_ITEM(submenu, MSG_ACCELERATION, menu_advanced_acceleration);

    // M205 - Junction Deviation or Max Jerk
    #if ENABLED(JUNCTION_DEVIATION)
      MENU_ITEM(submenu, MSG_JUNCTION_DEVIATION, menu_advanced_jerk);
    #else
      MENU_ITEM(submenu, MSG_JERK, menu_advanced_jerk);
    #endif

    if (!printer.isPrinting()) {
      // M92 - Steps Per mm
      MENU_ITEM(submenu, MSG_STEPS_PER_MM, menu_advanced_steps_per_mm);
    }

  #endif // !SLIM_LCD_MENUS

  if (printer.mode == PRINTER_MODE_FFF) {
    MENU_ITEM(submenu, MSG_TEMPERATURE, menu_advanced_temperature);
    #if ENABLED(VOLUMETRIC_EXTRUSION) || ENABLED(ADVANCED_PAUSE_FEATURE)
      MENU_ITEM(submenu, MSG_FILAMENT, menu_advanced_filament);
    #elif ENABLED(LIN_ADVANCE)
      MENU_ITEM_EDIT(float52, MSG_ADVANCE_K, &planner.extruder_advance_K, 0, 999);
    #endif
  }

  // M540 S - Abort on endstop hit when SD printing
  #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
    MENU_ITEM_EDIT(bool, MSG_ENDSTOP_ABORT, &planner.abort_on_endstop_hit);
  #endif

  //
  // BLTouch Self-Test and Reset
  //
  #if ENABLED(BLTOUCH)
    MENU_ITEM(gcode, MSG_BLTOUCH_SELFTEST, PSTR("M280 P" STRINGIFY(Z_PROBE_SERVO_NR) " S" STRINGIFY(BLTOUCH_SELFTEST)));
    if (!endstops.isProbeEnabled() && bltouch.test())
      MENU_ITEM(gcode, MSG_BLTOUCH_RESET, PSTR("M280 P" STRINGIFY(Z_PROBE_SERVO_NR) " S" STRINGIFY(BLTOUCH_RESET)));
  #endif

  #if ENABLED(EEPROM_SETTINGS) && DISABLED(SLIM_LCD_MENUS)
    MENU_ITEM(submenu, MSG_INIT_EEPROM, lcd_init_eeprom_confirm);
  #endif

  END_MENU();
}

#endif // HAS_LCD_MENU

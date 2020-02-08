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
#pragma once

/**
 * toolmanager.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "extruder/extruder.h"

struct tool_data_t {
  uint8_t total       : 4;
  uint8_t active      : 4;
  uint8_t previous    : 4;
  uint8_t target      : 4;
  bool    volumetric  : 1;
  bool    LA_test     : 1;
};

class ToolManager {

  public: /** Constructor */

    ToolManager() {};

  public: /** Public Parameters */

    static tool_data_t extruder;

    #if ENABLED(VOLUMETRIC_EXTRUSION)
      static float  volumetric_area_nominal;              // Nominal cross-sectional area
    #endif

    #if ENABLED(IDLE_OOZING_PREVENT)
      static bool IDLE_OOZING_enabled;
    #endif

  private: /** Private Parameters */

    #if ENABLED(IDLE_OOZING_PREVENT)
      static bool IDLE_OOZING_retracted[MAX_EXTRUDER];
    #endif

  public: /** Public Function */

    /**
     * Create Object Extruder
     */
    static void create_object();

    /**
     * Initialize Factory parameters
     */
    static void factory_parameters();

    /**
     * Change number extruder
     */
    static void change_number_extruder(const uint8_t e);

    /**
     * Get Active hotend on active extruder
     */
    static uint8_t active_hotend();

    /**
     * Get Target hotend on active extruder
     */
    static uint8_t target_hotend();

    /**
     * Change tools
     */
    static void change(const uint8_t new_tool, bool no_move=false);

    static void print_M563();

    #if ENABLED(LIN_ADVANCE)
      static void setup_test_linadvance();
      static void test_linadvance();
      static void print_M900();

      FORCE_INLINE static void setTestLinAdvance(const bool onoff) { extruder.LA_test = onoff; }
      FORCE_INLINE static bool IsTestLinAdvance() { return extruder.LA_test; }
    #endif

    #if ENABLED(VOLUMETRIC_EXTRUSION)
      static void print_M200();
      static void calculate_volumetric_multipliers();

      FORCE_INLINE static void setVolumetric(const bool onoff) { extruder.volumetric = onoff; }
      FORCE_INLINE static bool isVolumetric() { return extruder.volumetric; }
    #endif

    #if ENABLED(IDLE_OOZING_PREVENT)
      static void IDLE_OOZING_retract(const bool retracting);
    #endif

    #if ENABLED(EXT_SOLENOID)
      static void enable_solenoid(const uint8_t e);
      static void enable_solenoid_on_active_extruder();
      static void disable_all_solenoids();
    #endif

  private: /** Private Function */

    /**
     * Initialize Extruder Factory parameters
     */
    static void extruder_factory_parameters(const uint8_t e);

    static void invalid_extruder_error();

    static void fast_line_to_current(const AxisEnum fr_axis);

    #if ENABLED(VOLUMETRIC_EXTRUSION)
      static float calculate_volumetric_multiplier(const float diameter);
    #endif

    #if HAS_MKMULTI_TOOLS
      static void MK_multi_tool_change();
    #endif

    #if HAS_DONDOLO
      static void move_extruder_servo();
    #endif

    #if ENABLED(DUAL_X_CARRIAGE)
      static void dualx_tool_change(bool &no_move);
    #endif

};

extern ToolManager toolManager;

/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define G28

#if ENABLED(DEBUG_LEVELING_FEATURE)

  void log_machine_info() {
    SERIAL_MSG("Machine Type: ");
    #if IS_DELTA
      SERIAL_EM("Delta");
    #elif IS_SCARA
      SERIAL_EM("SCARA");
    #elif IS_CORE
      SERIAL_EM("Core");
    #else
      SERIAL_EM("Cartesian");
    #endif

    SERIAL_MSG("Probe: ");
    #if ENABLED(PROBE_MANUALLY)
      SERIAL_EM("PROBE_MANUALLY");
    #elif ENABLED(Z_PROBE_FIX_MOUNTED)
      SERIAL_EM("Z_PROBE_FIX_MOUNTED");
    #elif ENABLED(BLTOUCH)
      SERIAL_EM("BLTOUCH");
    #elif ENABLED(Z_PROBE_SLED)
      SERIAL_EM("Z_PROBE_SLED");
    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      SERIAL_EM("ALLEN KEY");
    #elif HAS_Z_SERVO_PROBE
      SERIAL_EM("SERVO PROBE");
    #else
      SERIAL_EM("NONE");
    #endif

    #if HAS_BED_PROBE
      SERIAL_MV("Probe Offset X:", X_PROBE_OFFSET_FROM_NOZZLE);
      SERIAL_MV(" Y:", Y_PROBE_OFFSET_FROM_NOZZLE);
      SERIAL_MV(" Z:", probe.z_offset);
      #if X_PROBE_OFFSET_FROM_NOZZLE > 0
        SERIAL_MSG(" (Right");
      #elif X_PROBE_OFFSET_FROM_NOZZLE < 0
        SERIAL_MSG(" (Left");
      #elif Y_PROBE_OFFSET_FROM_NOZZLE != 0
        SERIAL_MSG(" (Middle");
      #else
        SERIAL_MSG(" (Aligned With");
      #endif
      #if Y_PROBE_OFFSET_FROM_NOZZLE > 0
        SERIAL_MSG("-Back");
      #elif Y_PROBE_OFFSET_FROM_NOZZLE < 0
        SERIAL_MSG("-Front");
      #elif X_PROBE_OFFSET_FROM_NOZZLE != 0
        SERIAL_MSG("-Center");
      #endif
      if (probe.z_offset < 0)
        SERIAL_MSG(" & Below");
      else if (probe.z_offset > 0)
        SERIAL_MSG(" & Above");
      else
        SERIAL_MSG(" & Same Z as");
      SERIAL_EM(" Nozzle)");
    #endif

    #if HAS_ABL
      SERIAL_MSG("Auto Bed Leveling: ");
      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        SERIAL_MSG("LINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        SERIAL_MSG("BILINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_3POINT)
        SERIAL_MSG("3POINT");
      #endif
      if (bedlevel.leveling_is_active()) {
        SERIAL_EM(" (enabled)");
        #if ABL_PLANAR
          const float diff[XYZ] = {
            mechanics.get_axis_position_mm(X_AXIS) - mechanics.current_position[X_AXIS],
            mechanics.get_axis_position_mm(Y_AXIS) - mechanics.current_position[Y_AXIS],
            mechanics.get_axis_position_mm(Z_AXIS) - mechanics.current_position[Z_AXIS]
          };
          SERIAL_MSG("ABL Adjustment X");
          if (diff[X_AXIS] > 0) SERIAL_CHR('+');
          SERIAL_VAL(diff[X_AXIS]);
          SERIAL_MSG(" Y");
          if (diff[Y_AXIS] > 0) SERIAL_CHR('+');
          SERIAL_VAL(diff[Y_AXIS]);
          SERIAL_MSG(" Z");
          if (diff[Z_AXIS] > 0) SERIAL_CHR('+');
          SERIAL_VAL(diff[Z_AXIS]);
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          SERIAL_MV("ABL Adjustment Z", bedlevel.bilinear_z_offset(mechanics.current_position));
        #endif
      }
      else
        SERIAL_MSG(" (disabled)");

      SERIAL_EOL();

    #elif ENABLED(MESH_BED_LEVELING)

      SERIAL_MSG("Mesh Bed Leveling");
      if (bedlevel.leveling_is_active()) {
        float lz = mechanics.current_position[Z_AXIS];
        bedlevel.apply_leveling(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], lz);
        SERIAL_EM(" (enabled)");
        SERIAL_MV("MBL Adjustment Z", lz);
      }
      else
        SERIAL_MSG(" (disabled)");

      SERIAL_EOL();

    #endif

  }

#endif // DEBUG_LEVELING_FEATURE

#if ENABLED(PROBE_MANUALLY)
  bool g29_in_progress = false;
#else
  constexpr bool g29_in_progress = false;
#endif

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *  B   Return to back point
 *
 */
inline void gcode_G28(const bool always_home_all=false) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_EM(">>> gcode_G28");
      log_machine_info();
    }
  #endif

  #if HAS_POWER_SWITCH
    if (!powerManager.powersupply_on) powerManager.power_on(); // Power On if power is off
  #endif

  // Wait for planner moves to finish!
  stepper.synchronize();

  // Cancel the active G29 session
  #if ENABLED(PROBE_MANUALLY)
    g29_in_progress = false;
    #if HAS_NEXTION_MANUAL_BED
      LcdBedLevelOff();
    #endif
  #endif

  // Disable the leveling matrix before homing
  #if HAS_LEVELING
    bedlevel.set_bed_leveling_enabled(false);
  #endif

  // Always home with tool 0 active
  #if HOTENDS > 1
    const uint8_t old_tool_index = printer.active_extruder;
    printer.tool_change(0, 0, true);
  #endif

  #if ENABLED(CNC_WORKSPACE_PLANES)
    workspace_plane = PLANE_XY;
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    mechanics.hotend_duplication_enabled = false;
  #endif

  printer.setup_for_endstop_or_probe_move();
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_EM("> endstops.enable(true)");
  #endif
  endstops.enable(true); // Enable endstops for next homing move

  bool come_back = parser.seen('B');
  float lastpos[NUM_AXIS];
  float old_feedrate_mm_s;
  if (come_back) {
    old_feedrate_mm_s = mechanics.feedrate_mm_s;
    COPY_ARRAY(lastpos, mechanics.current_position);
  }

  mechanics.Home(always_home_all);

  #if ENABLED(NPR2)
    if ((home_all) || (parser.seen('E'))) {
      mechanics.set_destination_to_current();
      mechanics.destination[E_AXIS] = -200;
      printer.active_driver = printer.active_extruder = 1;
      planner.buffer_line_kinematic(mechanics.destination, COLOR_HOMERATE, printer.active_extruder);
      stepper.synchronize();
      old_color = 99;
      printer.active_driver = printer.active_extruder = 0;
      mechanics.current_position[E_AXIS] = 0;
      mechanics.sync_plan_position_e();
    }
  #endif

  endstops.not_homing();

  #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
    // move to a height where we can use the full xy-area
    mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
  #endif

  if (come_back) {
    mechanics.feedrate_mm_s = mechanics.homing_feedrate_mm_s[X_AXIS];
    COPY_ARRAY(mechanics.destination, lastpos);
    mechanics.prepare_move_to_destination();
    mechanics.feedrate_mm_s = old_feedrate_mm_s;
  }

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
    #if MECH(DELTA)
      gfx_clear((X_MAX_POS) * 2, (Y_MAX_POS) * 2, Z_MAX_POS);
      gfx_cursor_to(mechanics.current_position[X_AXIS] + (X_MAX_POS), mechanics.current_position[Y_AXIS] + (Y_MAX_POS), mechanics.current_position[Z_AXIS]);
    #else
      gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
      gfx_cursor_to(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS]);
    #endif
  #endif

  printer.clean_up_after_endstop_or_probe_move();

  stepper.synchronize();

  // Restore the active tool after homing
  #if HOTENDS > 1
    printer.tool_change(old_tool_index, 0, true);
  #endif

  lcd_refresh();

  mechanics.report_current_position();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G28");
  #endif

} // G28

void home_all_axes() { gcode_G28(true); }

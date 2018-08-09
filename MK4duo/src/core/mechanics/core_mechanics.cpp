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
 * core_mechanics.cpp
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "core_mechanics.h"

#if IS_CORE

  Core_Mechanics mechanics;

  /** Public Parameters */
  const float Core_Mechanics::base_max_pos[XYZ]   = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS },
              Core_Mechanics::base_min_pos[XYZ]   = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
              Core_Mechanics::base_home_pos[XYZ]  = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS },
              Core_Mechanics::max_length[XYZ]     = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH };

  /** Public Function */
  void Core_Mechanics::factory_parameters() {

    static const float    tmp1[] PROGMEM  = DEFAULT_AXIS_STEPS_PER_UNIT,
                          tmp2[] PROGMEM  = DEFAULT_MAX_FEEDRATE;
    static const uint32_t tmp3[] PROGMEM  = DEFAULT_MAX_ACCELERATION,
                          tmp4[] PROGMEM  = DEFAULT_RETRACT_ACCELERATION;

    LOOP_XYZE_N(i) {
      axis_steps_per_mm[i]          = pgm_read_float(&tmp1[i < COUNT(tmp1) ? i : COUNT(tmp1) - 1]);
      max_feedrate_mm_s[i]          = pgm_read_float(&tmp2[i < COUNT(tmp2) ? i : COUNT(tmp2) - 1]);
      max_acceleration_mm_per_s2[i] = pgm_read_dword_near(&tmp3[i < COUNT(tmp3) ? i : COUNT(tmp3) - 1]);
    }

    for (uint8_t i = 0; i < EXTRUDERS; i++)
      retract_acceleration[i] = pgm_read_dword_near(&tmp4[i < COUNT(tmp4) ? i : COUNT(tmp4) - 1]);

    acceleration              = DEFAULT_ACCELERATION;
    travel_acceleration       = DEFAULT_TRAVEL_ACCELERATION;
    min_feedrate_mm_s         = DEFAULT_MINIMUMFEEDRATE;
    min_segment_time_us       = DEFAULT_MINSEGMENTTIME;
    min_travel_feedrate_mm_s  = DEFAULT_MINTRAVELFEEDRATE;

    #if ENABLED(JUNCTION_DEVIATION)
      junction_deviation_mm = JUNCTION_DEVIATION_MM;
    #else
      static const float tmp5[] PROGMEM = DEFAULT_EJERK;
      max_jerk[X_AXIS]  = DEFAULT_XJERK;
      max_jerk[Y_AXIS]  = DEFAULT_YJERK;
      max_jerk[Z_AXIS]  = DEFAULT_ZJERK;
      for (uint8_t i = 0; i < EXTRUDERS; i++)
        max_jerk[E_AXIS + i] = pgm_read_float(&tmp5[i < COUNT(tmp5) ? i : COUNT(tmp5) - 1]);
    #endif

    #if ENABLED(WORKSPACE_OFFSETS)
      ZERO(mechanics.home_offset);
    #endif

  }

  void Core_Mechanics::sync_plan_position_mech_specific() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("sync_plan_position_mech_specific", current_position);
    #endif
    sync_plan_position();
  }

  /**
   * Home Core
   */
  void Core_Mechanics::home(const bool homeX/*=false*/, const bool homeY/*=false*/, const bool homeZ/*=false*/) {

    if (printer.debugSimulation()) {
      LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        mechanics.Nextion_gfx_clear();
      #endif
      return;
    }

    #if HAS_POWER_SWITCH
      powerManager.power_on(); // Power On if power is off
    #endif

    // Wait for planner moves to finish!
    planner.synchronize();

    // Cancel the active G29 session
    #if HAS_LEVELING && ENABLED(PROBE_MANUALLY)
      bedlevel.g29_in_progress = false;
      #if HAS_NEXTION_MANUAL_BED
        Nextion_ProbeOff();
      #endif
    #endif

    // Disable the leveling matrix before homing
    #if HAS_LEVELING
      const bool leveling_was_active = bedlevel.leveling_active;
      bedlevel.set_bed_leveling_enabled(false);
    #endif

    #if ENABLED(CNC_WORKSPACE_PLANES)
      workspace_plane = PLANE_XY;
    #endif

    // Always home with tool 0 active
    #if HOTENDS > 1
      const uint8_t old_tool_index = tools.active_extruder;
      tools.change(0, 0, true);
    #endif

    printer.setup_for_endstop_or_probe_move();
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("> endstops.setEnabled(true)");
    #endif
    endstops.setEnabled(true); // Enable endstops for next homing move

    bool come_back = parser.boolval('B');
    float lastpos[NUM_AXIS];
    float old_feedrate_mm_s;
    if (come_back) {
      old_feedrate_mm_s = feedrate_mm_s;
      COPY_ARRAY(lastpos, current_position);
    }

    const bool home_all = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    set_destination_to_current();

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first
      if (home_all || homeZ) homeaxis(Z_AXIS);
    #endif

    const float z_homing_height = printer.isZHomed() ? MIN_Z_HEIGHT_FOR_HOMING : 0;

    if (z_homing_height && (home_all || homeX || homeY)) {
      // Raise Z before homing any other axes and z is not already high enough (never lower z)
      destination[Z_AXIS] = z_homing_height;
      if (destination[Z_AXIS] > current_position[Z_AXIS]) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling())
            SERIAL_EMV("Raise Z (before homing) to ", destination[Z_AXIS]);
        #endif
        do_blocking_move_to_z(destination[Z_AXIS]);
      }
    }

    #if ENABLED(QUICK_HOME)
      if (home_all || (homeX && homeY)) quick_home_xy();
    #endif

    #if ENABLED(HOME_Y_BEFORE_X)
      // Home Y (before X)
      if (home_all || homeY) homeaxis(Y_AXIS);
    #endif

    // Home X
    if (home_all || homeX) homeaxis(X_AXIS);

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y (after X)
      if (home_all || homeY) homeaxis(Y_AXIS);
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      if (home_all || homeZ) {
        #if ENABLED(Z_SAFE_HOMING)
          home_z_safely();
        #else
          homeaxis(Z_AXIS);
        #endif // !Z_SAFE_HOMING

        #if HOMING_Z_WITH_PROBE && Z_PROBE_AFTER_PROBING > 0
          probe.move_z_after_probing();
        #endif

      } // home_all || homeZ

    #elif ENABLED(DOUBLE_Z_HOMING)
      if (home_all || homeZ) double_home_z();
    #endif

    sync_plan_position();
    endstops.setNotHoming();

    if (come_back) {
      feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
      COPY_ARRAY(destination, lastpos);
      prepare_move_to_destination();
      feedrate_mm_s = old_feedrate_mm_s;
    }

    #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
      mechanics.Nextion_gfx_clear();
    #endif

    #if HAS_LEVELING
      bedlevel.set_bed_leveling_enabled(leveling_was_active);
    #endif

    printer.clean_up_after_endstop_or_probe_move();

    planner.synchronize();

    // Restore the active tool after homing
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif

    lcd_refresh();

    mechanics.report_current_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("<<< G28");
    #endif

  }

  /**
   * Prepare a linear move in a Cartesian setup.
   *
   * When a mesh-based leveling system is active, moves are segmented
   * according to the configuration of the leveling system.
   *
   * Returns true if current_position[] was set to destination[]
   */
  bool Core_Mechanics::prepare_move_to_destination_mech_specific() {

    #if ENABLED(LASER) && ENABLED(LASER_FIRE_E)
      if (current_position[E_AXIS] < destination[E_AXIS] && ((current_position[X_AXIS] != destination [X_AXIS]) || (current_position[Y_AXIS] != destination [Y_AXIS])))
        laser.status = LASER_ON;
      else
        laser.status = LASER_OFF;
    #endif

    #if HAS_MESH
      if (bedlevel.leveling_active) {
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          ubl.line_to_destination_cartesian(MMS_SCALED(feedrate_mm_s), tools.active_extruder);
          return true;
        #else
          /**
           * For MBL and ABL-BILINEAR only segment moves when X or Y are involved.
           * Otherwise fall through to do a direct single move.
           */
          if (current_position[X_AXIS] != destination[X_AXIS] || current_position[Y_AXIS] != destination[Y_AXIS]) {
            #if ENABLED(MESH_BED_LEVELING)
              mbl.line_to_destination(MMS_SCALED(feedrate_mm_s));
            #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
              abl.bilinear_line_to_destination(MMS_SCALED(feedrate_mm_s));
            #endif
            return true;
          }
        #endif
      }
    #endif // HAS_MESH

    line_to_destination(MMS_SCALED(feedrate_mm_s));
    return false;
  }

  void Core_Mechanics::homeaxis(const AxisEnum axis) {

    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV(">>> homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    // Homing Z towards the bed? Deploy the Z probe or endstop.
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS && DEPLOY_PROBE()) return;
    #endif

    // Set flags for X, Y, Z motor locking
    #if ENABLED(X_TWO_ENDSTOPS)
      if (axis == X_AXIS) stepper.set_homing_dual_axis(true);
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      if (axis == Y_AXIS) stepper.set_homing_dual_axis(true);
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      if (axis == Z_AXIS) stepper.set_homing_dual_axis(true);
    #endif

    // Fast move towards endstop until triggered
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("Home 1 Fast:");
    #endif

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      // BLTOUCH needs to be deployed every time
      if (axis == Z_AXIS && probe.set_bltouch_deployed(true)) return;
    #endif

    mechanics.do_homing_move(axis, 1.5f * max_length[axis] * home_dir[axis]);

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      // BLTOUCH needs to be deployed every time
      if (axis == Z_AXIS) probe.set_bltouch_deployed(false);
    #endif

    // When homing Z with probe respect probe clearance
    const float bump = home_dir[axis] * (
      #if HOMING_Z_WITH_PROBE
        (axis == Z_AXIS) ? MAX(Z_PROBE_BETWEEN_HEIGHT, home_bump_mm[Z_AXIS]) :
      #endif
      home_bump_mm[axis]
    );

    // If a second homing move is configured...
    if (bump) {
      // Move away from the endstop by the axis HOME_BUMP_MM
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("Move Away:");
      #endif
      mechanics.do_homing_move(axis, -bump
        #if HOMING_Z_WITH_PROBE
          , axis == Z_AXIS ? MMM_TO_MMS(Z_PROBE_SPEED_FAST) : 0.0
        #endif
      );

      // Slow move towards endstop until triggered
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("Home 2 Slow:");
      #endif

      #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
        // BLTOUCH needs to be deployed every time
        if (axis == Z_AXIS && probe.set_bltouch_deployed(true)) return;
      #endif

      mechanics.do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));

      #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
        // BLTOUCH needs to be deployed every time
        if (axis == Z_AXIS) probe.set_bltouch_deployed(false);
      #endif
    }

    #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
      const bool pos_dir = home_dir[axis] > 0;
      #if ENABLED(X_TWO_ENDSTOPS)
        if (axis == X_AXIS) {
          const float adj = ABS(endstops.x_endstop_adj);
          if (adj) {
            if (pos_dir ? (endstops.x_endstop_adj > 0) : (endstops.x_endstop_adj < 0)) stepper.set_x_lock(true); else stepper.set_x2_lock(true);
            mechanics.do_homing_move(axis, pos_dir ? -adj : adj);
            stepper.set_x_lock(false);
            stepper.set_x2_lock(false);
          }
        }
      #endif
      #if ENABLED(Y_TWO_ENDSTOPS)
        if (axis == Y_AXIS) {
          const float adj = ABS(endstops.y_endstop_adj);
          if (adj) {
            if (pos_dir ? (endstops.y_endstop_adj > 0) : (endstops.y_endstop_adj < 0)) stepper.set_y_lock(true); else stepper.set_y2_lock(true);
            mechanics.do_homing_move(axis, pos_dir ? -adj : adj);
            stepper.set_y_lock(false);
            stepper.set_y2_lock(false);
          }
        }
      #endif
      #if ENABLED(Z_TWO_ENDSTOPS)
        if (axis == Z_AXIS) {
          const float adj = ABS(endstops.z_endstop_adj);
          if (adj) {
            if (pos_dir ? (endstops.z_endstop_adj > 0) : (endstops.z_endstop_adj < 0)) stepper.set_z_lock(true); else stepper.set_z2_lock(true);
            mechanics.do_homing_move(axis, pos_dir ? -adj : adj);
            stepper.set_z_lock(false);
            stepper.set_z2_lock(false);
          }
        }
      #endif
      stepper.set_homing_dual_axis(false);
    #endif

    // For cartesian machines,
    // set the axis to its home position
    set_axis_is_at_home(axis);
    sync_plan_position();

    destination[axis] = current_position[axis];

    // Put away the Z probe
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS && STOW_PROBE()) return;
    #endif

    // Clear retracted status if homing the Z axis
    #if ENABLED(FWRETRACT)
      if (axis == Z_AXIS) fwretract.hop_amount = 0.0;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')');
        SERIAL_EOL();
      }
    #endif
  }

  #if ENABLED(QUICK_HOME)

    void Core_Mechanics::quick_home_xy() {

      // Pretend the current position is 0,0
      current_position[X_AXIS] = current_position[Y_AXIS] = 0;
      sync_plan_position();

      const float mlx = max_length[X_AXIS],
                  mly = max_length[Y_AXIS],
                  mlratio = mlx > mly ? mly / mlx : mlx / mly,
                  fr_mm_s = MIN(homing_feedrate_mm_s[X_AXIS], homing_feedrate_mm_s[Y_AXIS]) * SQRT(sq(mlratio) + 1.0);

      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(X_AXIS);
        sensorless_homing_per_axis(Y_AXIS);
      #endif

      do_blocking_move_to_xy(1.5 * mlx * home_dir[X_AXIS], 1.5 * mly * home_dir[Y_AXIS], fr_mm_s);

      endstops.validate_homing_move();

      current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;

      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(X_AXIS, false);
        sensorless_homing_per_axis(Y_AXIS, false);
      #endif
    }

  #endif // QUICK_HOME

  #if ENABLED(Z_SAFE_HOMING)

    void Core_Mechanics::home_z_safely() {

      // Disallow Z homing if X or Y are unknown
      if (!printer.isXHomed() || !printer.isYHomed()) {
        LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
        SERIAL_LM(ECHO, MSG_ERR_Z_HOMING);
        return;
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("Z_SAFE_HOMING >>>");
      #endif

      sync_plan_position();

      /**
       * Move the Z probe (or just the nozzle) to the safe homing point
       */
      destination[X_AXIS] = Z_SAFE_HOMING_X_POINT;
      destination[Y_AXIS] = Z_SAFE_HOMING_Y_POINT;
      destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

      #if HOMING_Z_WITH_PROBE
        destination[X_AXIS] -= probe.offset[X_AXIS];
        destination[Y_AXIS] -= probe.offset[Y_AXIS];
      #endif

      if (mechanics.position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) {

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling()) DEBUG_POS("Z_SAFE_HOMING", destination);
        #endif

        #if ENABLED(SENSORLESS_HOMING)
          printer.safe_delay(500);
        #endif

        do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);
        homeaxis(Z_AXIS);
      }
      else {
        LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
        SERIAL_LM(ECHO, MSG_ZPROBE_OUT);
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("<<< Z_SAFE_HOMING");
      #endif
    }

  #endif // Z_SAFE_HOMING

  #if ENABLED(DOUBLE_Z_HOMING)

    void Core_Mechanics::double_home_z() {

      // Disallow Z homing if X or Y are unknown
      if (!printer.isXHomed() || !printer.isYHomed()) {
        LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
        SERIAL_LM(ECHO, MSG_ERR_Z_HOMING);
        return;
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("DOUBLE_Z_HOMING >>>");
      #endif

      sync_plan_position();

      /**
       * Move the Z probe (or just the nozzle) to the safe homing point
       */
      destination[X_AXIS] = DOUBLE_Z_HOMING_X_POINT;
      destination[Y_AXIS] = DOUBLE_Z_HOMING_Y_POINT;
      destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

      #if HAS_BED_PROBE
        destination[X_AXIS] -= probe.offset[X_AXIS];
        destination[Y_AXIS] -= probe.offset[Y_AXIS];
      #endif

      if (mechanics.position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) {

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling()) DEBUG_POS("DOUBLE_Z_HOMING", destination);
        #endif

        const float newzero = probe_pt(destination[X_AXIS], destination[Y_AXIS], true, 1) - (2 * probe.offset[Z_AXIS]);
        current_position[Z_AXIS] -= newzero;
        destination[Z_AXIS] = current_position[Z_AXIS];
        endstops.soft_endstop_max[Z_AXIS] = base_max_pos(Z_AXIS) - newzero;

        sync_plan_position();
        do_blocking_move_to_z(MIN_Z_HEIGHT_FOR_HOMING);
      }
      else {
        LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
        SERIAL_LM(ECHO, MSG_ZPROBE_OUT);
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("<<< DOUBLE_Z_HOMING");
      #endif
    }

  #endif

  /**
   * Set an axis' current position to its home position (after homing).
   *
   * For Cartesian this applies one-to-one when an
   * individual axis has been homed.
   *
   * Callers must sync the planner position after calling this!
   */
  void Core_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    printer.setAxisHomed(axis, true);

    #if ENABLED(WORKSPACE_OFFSETS)
      position_shift[axis] = 0;
      endstops.update_software_endstops(axis);
    #endif

    current_position[axis] = base_home_pos[axis];

    /**
     * Z Probe Z Homing? Account for the probe's Z offset.
     */
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS) {
        current_position[Z_AXIS] -= probe.offset[Z_AXIS];

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling()) {
            SERIAL_EM("*** Z HOMED WITH PROBE ***");
            SERIAL_EMV("zprobe_zoffset = ", probe.offset[Z_AXIS]);
          }
        #endif
      }
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        #if ENABLED(WORKSPACE_OFFSETS)
          SERIAL_MV("> home_offset[", axis_codes[axis]);
          SERIAL_EMV("] = ", home_offset[axis]);
        #endif
        DEBUG_POS("", current_position);
        SERIAL_MV("<<< set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  #if DISABLED(DISABLE_M503)

    void Core_Mechanics::print_parameters() {

      SERIAL_LM(CFG, "Steps per unit:");
      SERIAL_SMV(CFG, "  M92 X", LINEAR_UNIT(axis_steps_per_mm[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(axis_steps_per_mm[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(axis_steps_per_mm[Z_AXIS]), 3);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(axis_steps_per_mm[E_AXIS]), 3);
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M92 T", (int)e);
          SERIAL_EMV(" E", VOLUMETRIC_UNIT(axis_steps_per_mm[E_AXIS + e]), 3);
        }
      #endif // EXTRUDERS > 1

      SERIAL_LM(CFG, "Maximum feedrates (units/s):");
      SERIAL_SMV(CFG, "  M203 X", LINEAR_UNIT(max_feedrate_mm_s[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(max_feedrate_mm_s[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(max_feedrate_mm_s[Z_AXIS]), 3);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(max_feedrate_mm_s[E_AXIS]), 3);
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M203 T", (int)e);
          SERIAL_EMV(" E", VOLUMETRIC_UNIT(max_feedrate_mm_s[E_AXIS + e]), 3);
        }
      #endif // EXTRUDERS > 1

      SERIAL_LM(CFG, "Maximum Acceleration (units/s2):");
      SERIAL_SMV(CFG, "  M201 X", LINEAR_UNIT(max_acceleration_mm_per_s2[X_AXIS]));
      SERIAL_MV(" Y", LINEAR_UNIT(max_acceleration_mm_per_s2[Y_AXIS]));
      SERIAL_MV(" Z", LINEAR_UNIT(max_acceleration_mm_per_s2[Z_AXIS]));
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(max_acceleration_mm_per_s2[E_AXIS]));
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M201 T", (int)e);
          SERIAL_EMV(" E", VOLUMETRIC_UNIT(max_acceleration_mm_per_s2[E_AXIS + e]));
        }
      #endif // EXTRUDERS > 1

      SERIAL_LM(CFG, "Acceleration (units/s2): P<print_accel> V<travel_accel> T* R<retract_accel>:");
      SERIAL_SMV(CFG,"  M204 P", LINEAR_UNIT(acceleration), 3);
      SERIAL_MV(" V", LINEAR_UNIT(travel_acceleration), 3);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 R", LINEAR_UNIT(retract_acceleration[0]), 3);
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M204 T", (int)e);
          SERIAL_EMV(" R", LINEAR_UNIT(retract_acceleration[e]), 3);
        }
      #endif

      SERIAL_LM(CFG, "Advanced variables: B<min_segment_time_us> S<min_feedrate> V<min_travel_feedrate>:");
      SERIAL_SMV(CFG, " M205 B", min_segment_time_us);
      SERIAL_MV(" S", LINEAR_UNIT(min_feedrate_mm_s), 3);
      SERIAL_EMV(" V", LINEAR_UNIT(min_travel_feedrate_mm_s), 3);

      #if ENABLED(JUNCTION_DEVIATION)
        SERIAL_LM(CFG, "Junction Deviation: J<Junction deviation mm>:");
        SERIAL_LMV(CFG, "  M205 J", junction_deviation_mm, 3);
      #else
        SERIAL_LM(CFG, "Jerk: X<max_xy_jerk> Z<max_z_jerk> T* E<max_e_jerk>:");
        SERIAL_SMV(CFG, " M205 X", LINEAR_UNIT(max_jerk[X_AXIS]), 3);
        SERIAL_MV(" Y", LINEAR_UNIT(max_jerk[Y_AXIS]), 3);
        SERIAL_MV(" Z", LINEAR_UNIT(max_jerk[Z_AXIS]), 3);
        #if EXTRUDERS == 1
          SERIAL_MV(" T0 E", LINEAR_UNIT(max_jerk[E_AXIS]), 3);
        #endif
        SERIAL_EOL();
        #if (EXTRUDERS > 1)
          LOOP_EXTRUDER() {
            SERIAL_SMV(CFG, "  M205 T", (int)e);
            SERIAL_EMV(" E" , LINEAR_UNIT(max_jerk[E_AXIS + e]), 3);
          }
        #endif
      #endif

      #if ENABLED(WORKSPACE_OFFSETS)
        SERIAL_LM(CFG, "Home offset:");
        SERIAL_SMV(CFG, "  M206 X", LINEAR_UNIT(home_offset[X_AXIS]), 3);
        SERIAL_MV(" Y", LINEAR_UNIT(home_offset[Y_AXIS]), 3);
        SERIAL_EMV(" Z", LINEAR_UNIT(home_offset[Z_AXIS]), 3);
      #endif

    }

  #endif // DISABLED(DISABLE_M503)

#endif // IS_CORE

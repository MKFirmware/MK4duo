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
 * scara_mechanics.cpp
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "scara_mechanics.h"

#if IS_SCARA

  Scara_Mechanics mechanics;

  /** Public Parameters */
  const float Scara_Mechanics::base_max_pos[XYZ]    = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS },
              Scara_Mechanics::base_min_pos[XYZ]    = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
              Scara_Mechanics::base_home_pos[XYZ]   = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS },
              Scara_Mechanics::max_length[XYZ]      = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH },
              Scara_Mechanics::L1                   = SCARA_LINKAGE_1,
              Scara_Mechanics::L2                   = SCARA_LINKAGE_2,
              Scara_Mechanics::L1_2                 = sq(float(L1)),
              Scara_Mechanics::L1_2_2               = 2.0f * L1_2,
              Scara_Mechanics::L2_2                 = sq(float(L2));

  float Scara_Mechanics::delta[ABC]                 = { 0.0 },
        Scara_Mechanics::delta_segments_per_second  = SCARA_SEGMENTS_PER_SECOND;

  /** Public Function */
  void Scara_Mechanics::factory_parameters() {

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

    delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND;

    #if ENABLED(WORKSPACE_OFFSETS)
      ZERO(mechanics.home_offset);
    #endif

  }

  void Scara_Mechanics::sync_plan_position_mech_specific() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("sync_plan_position_mech_specific", current_position);
    #endif
    planner.set_position_mm_kinematic(current_position);
  }

  // Return true if the given point is within the printable area
  bool Scara_Mechanics::position_is_reachable(const float &rx, const float &ry) {
    #if MIDDLE_DEAD_ZONE_R > 0
      const float R2 = HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y);
      return R2 >= sq(float(MIDDLE_DEAD_ZONE_R)) && R2 <= sq(L1 + L2);
    #else
      return HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y) <= sq(L1 + L2);
    #endif
  }
  // Return true if the both nozzle and the probe can reach the given point.
  bool Scara_Mechanics::position_is_reachable_by_probe(const float &rx, const float &ry) {
    return position_is_reachable(rx, ry)
        && position_is_reachable(rx - probe.offset[X_AXIS], ry - probe.offset[Y_AXIS]);
  }

  /**
   * Get the stepper positions in the cartesian_position[] array.
   * Forward kinematics are applied for SCARA.
   *
   * The result is in the current coordinate space with
   * leveling applied. The coordinates need to be run through
   * unapply_leveling to obtain the "ideal" coordinates
   * suitable for current_position, etc.
   */
  void Scara_Mechanics::get_cartesian_from_steppers() {
    InverseTransform(
      planner.get_axis_position_mm(A_AXIS),
      planner.get_axis_position_mm(B_AXIS),
      cartesian_position
    );
    cartesian_position[Z_AXIS] = planner.get_axis_position_mm(Z_AXIS);
  }

  #if DISABLED(AUTO_BED_LEVELING_UBL)

    /**
     * Prepare a linear move in a SCARA setup.
     *
     * This calls planner.buffer_line several times, adding
     * small incremental moves for SCARA.
     */
     bool Scara_Mechanics::prepare_move_to_destination_mech_specific() {

      // Get the top feedrate of the move in the XY plane
      const float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

      // Get the cartesian distances moved in XYZE
      const float difference[XYZE] = {
        destination[X_AXIS] - current_position[X_AXIS],
        destination[Y_AXIS] - current_position[Y_AXIS],
        destination[Z_AXIS] - current_position[Z_AXIS],
        destination[E_AXIS] - current_position[E_AXIS]
      };

      // If the move is only in Z/E don't split up the move
      if (!difference[X_AXIS] && !difference[Y_AXIS]) {
        planner.buffer_line_kinematic(destination, _feedrate_mm_s, tools.active_extruder);
        return false; // caller will update current_position
      }

      // Fail if attempting move outside printable radius
      if (endstops.isSoftEndstop() && !mechanics.position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) return true;

      // Get the linear distance in XYZ
      float cartesian_mm = SQRT(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));

      // If the move is very short, check the E move distance
      if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(difference[E_AXIS]);

      // No E move either? Game over.
      if (UNEAR_ZERO(cartesian_mm)) return true;

      // Minimum number of seconds to move the given distance
      const float seconds = cartesian_mm / _feedrate_mm_s;

      // The number of segments-per-second times the duration
      // gives the number of segments
      uint16_t segments = delta_segments_per_second * seconds;

      // For SCARA minimum segment size is 0.5mm
      NOMORE(segments, cartesian_mm * 2);

      // At least one segment is required
      NOLESS(segments, 1U);

      // The approximate length of each segment
      const float inv_segments = 1.0f / float(segments),
                  segment_distance[XYZE] = {
                    difference[X_AXIS] * inv_segments,
                    difference[Y_AXIS] * inv_segments,
                    difference[Z_AXIS] * inv_segments,
                    difference[E_AXIS] * inv_segments
                  };

      #if DISABLED(SCARA_FEEDRATE_SCALING)
        const float cartesian_segment_mm = cartesian_mm * inv_segments;
      #endif

      //SERIAL_MV("mm=", cartesian_mm);
      //SERIAL_MV(" seconds=", seconds);
      //SERIAL_MV(" segments=", segments);
      #if DISABLED(SCARA_FEEDRATE_SCALING)
        //SERIAL_MV(" segment_mm=", cartesian_segment_mm);
      #endif
      //SERIAL_EOL();

      #if ENABLED(SCARA_FEEDRATE_SCALING)
        // SCARA needs to scale the feed rate from mm/s to degrees/s
        // i.e., Complete the angular vector in the given time.
        const float segment_length = cartesian_mm * inv_segments,
                    inv_segment_length = 1.0f / segment_length, // 1/mm/segs
                    inverse_secs = inv_segment_length * _feedrate_mm_s;

        float oldA = planner.position_float[A_AXIS],
              oldB = planner.position_float[B_AXIS];
      #endif

      // Get the current position as starting point
      float raw[XYZE];
      COPY_ARRAY(raw, current_position);

      // Calculate and execute the segments
      while (--segments) {

        printer.check_periodical_actions();

        LOOP_XYZE(i) raw[i] += segment_distance[i];
        Transform(raw);

        // Adjust Z if bed leveling is enabled
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          if (bedlevel.leveling_active) {
            const float zadj = abl.bilinear_z_offset(raw);
            delta[A_AXIS] += zadj;
            delta[B_AXIS] += zadj;
            delta[C_AXIS] += zadj;
          }
        #endif

        #if ENABLED(SCARA_FEEDRATE_SCALING)
          // For SCARA scale the feed rate from mm/s to degrees/s
          // i.e., Complete the angular vector in the given time.
          if (!planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], raw[Z_AXIS], raw[E_AXIS], HYPOT(delta[A_AXIS] - oldA, delta[B_AXIS] - oldB) * inverse_secs, tools.active_extruder))
            break;
          oldA = delta[A_AXIS];
          oldB = delta[B_AXIS];
        #else
          if (!planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], _feedrate_mm_s, tools.active_extruder, cartesian_segment_mm))
            break;
        #endif

      }

      // Ensure last segment arrives at target location.
      #if ENABLED(SCARA_FEEDRATE_SCALING)
        // For SCARA scale the feed rate from mm/s to degrees/s
        // With segments > 1 length is 1 segment, otherwise total length
        Transform(destination);

        // Adjust Z if bed leveling is enabled
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          if (bedlevel.leveling_active) {
            const float zadj = abl.bilinear_z_offset(raw);
            delta[A_AXIS] += zadj;
            delta[B_AXIS] += zadj;
            delta[C_AXIS] += zadj;
          }
        #endif
        const float diff2 = HYPOT2(delta[A_AXIS] - oldA, delta[B_AXIS] - oldB);
        if (diff2)
          planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], destination[Z_AXIS], destination[E_AXIS], SQRT(diff2) * inverse_secs, tools.active_extruder);
      #else
        planner.buffer_line_kinematic(destination, _feedrate_mm_s, tools.active_extruder);
      #endif

      return false;
    }

  #endif // DISABLED(AUTO_BED_LEVELING_UBL)

  /**
   *  Plan a move to (X, Y, Z) and set the current_position
   *  The final current_position may not be the one that was requested
   */
  void Scara_Mechanics::do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s /*=0.0*/) {

    const float old_feedrate_mm_s = feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, rx, ry, rz);
    #endif

    const float z_feedrate = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];

    if (!position_is_reachable(rx, ry)) return;

    set_destination_to_current();

    // If Z needs to raise, do it before moving XY
    if (destination[Z_AXIS] < rz) {
      destination[Z_AXIS] = rz;
      prepare_uninterpolated_move_to_destination(z_feedrate);
    }

    destination[X_AXIS] = rx;
    destination[Y_AXIS] = ry;
    prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S);

    // If Z needs to lower, do it after moving XY
    if (destination[Z_AXIS] > rz) {
      destination[Z_AXIS] = rz;
      prepare_uninterpolated_move_to_destination(z_feedrate);
    }

    feedrate_mm_s = old_feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_ECHOLNPGM("<<< do_blocking_move_to");
    #endif

    planner.synchronize();

  }

  /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void Scara_Mechanics::prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);
    #endif

    if ( current_position[X_AXIS] == destination[X_AXIS]
      && current_position[Y_AXIS] == destination[Y_AXIS]
      && current_position[Z_AXIS] == destination[Z_AXIS]
      && current_position[E_AXIS] == destination[E_AXIS]
    ) return;

    planner.buffer_line_kinematic(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), tools.active_extruder);

    set_current_to_destination();
  }

  bool Scara_Mechanics::move_to_cal(uint8_t delta_a, uint8_t delta_b) {
    if (printer.isRunning()) {
      InverseTransform(delta_a, delta_b, cartesian_position);
      destination[X_AXIS] = cartesian_position[X_AXIS];
      destination[Y_AXIS] = cartesian_position[Y_AXIS];
      destination[Z_AXIS] = current_position[Z_AXIS];
      prepare_move_to_destination();
      return true;
    }
    return false;
  }

  /**
   * Morgan SCARA InverseTransform. Results in cartesian[].
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void Scara_Mechanics::InverseTransform(const float Ha, const float Hb, float cartesian[XYZ]) {

    const float a_sin = sin(RADIANS(Ha)) * L1,
                a_cos = cos(RADIANS(Ha)) * L1,
                b_sin = sin(RADIANS(Hb)) * L2,
                b_cos = cos(RADIANS(Hb)) * L2;

    cartesian[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  //theta
    cartesian[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  //theta+phi

  }

  /**
   * Morgan SCARA Transform. Results in delta[].
   *
   * See http://forums.reprap.org/read.php?185,283327
   *
   * Maths and first version by QHARLEY.
   */
  void Scara_Mechanics::Transform(const float raw[XYZ]) {

    static float C2, S2, SK1, SK2, THETA, PSI;

    float sx = raw[X_AXIS] - SCARA_OFFSET_X,  // Translate SCARA to standard X Y
          sy = raw[Y_AXIS] - SCARA_OFFSET_Y;  // With scaling factor.

    if (L1 == L2)
      C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
    else
      C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0f * L1 * L2);

    S2 = SQRT(1 - sq(C2));

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    THETA = ATAN2(SK1, SK2) - ATAN2(sx, sy);

    // Angle of Arm2
    PSI = ATAN2(S2, C2);

    delta[A_AXIS] = DEGREES(THETA);        // theta is support arm angle
    delta[B_AXIS] = DEGREES(THETA + PSI);  // equal to sub arm angle (inverted motor)
    delta[C_AXIS] = raw[Z_AXIS];

  }

  void Scara_Mechanics::homeaxis(const AxisEnum axis) {

    // Only Z homing (with probe) is permitted
    if (axis != Z_AXIS) { BUZZ(100, 880); return; }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV(">>> homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    // Homing Z towards the bed? Deploy the Z probe or endstop.
    #if HOMING_Z_WITH_PROBE
      if (probe.set_deployed(true)) return;
    #endif

    // Fast move towards endstop until triggered
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("Home 1 Fast:");
    #endif

    // Fast move towards endstop until triggered
    mechanics.do_homing_move(axis, 1.5f * max_length[axis] * home_dir[axis]);

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
      mechanics.do_homing_move(axis, -bump);

      // Slow move towards endstop until triggered
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("Home 2 Slow:");
      #endif
      mechanics.do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
    }

    set_axis_is_at_home(axis);
    sync_plan_position_mech_specific();

    // Put away the Z probe
    #if HOMING_Z_WITH_PROBE
      if (probe.set_deployed(false)) return;
    #endif

    // Clear retracted status if homing the Z axis
    #if ENABLED(FWRETRACT)
      fwretract.hop_amount = 0.0;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  /**
   * Home Scara
   */
  void Scara_Mechanics::home() {

    if (printer.debugSimulation()) {
      LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        mechanics.Nextion_gfx_clear();
      #endif
      return true;
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
        Nextion_ProbeOn();
      #endif
    #endif

    // Disable the leveling matrix before homing
    #if HAS_LEVELING
      const bool leveling_was_active = bedlevel.leveling_active;
      bedlevel.set_bed_leveling_enabled(false);
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
      old_feedrate_mm_s = mechanics.feedrate_mm_s;
      COPY_ARRAY(lastpos, mechanics.current_position);
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS(">>> home_scara", current_position);
    #endif

    const bool  homeA = parser.seen('X'),
                homeB = parser.seen('Y'),
                homeZ = parser.seen('Z');

    const bool home_all = (!homeA && !homeB && !homeZ) || (homeA && homeB && homeZ);

    // Home A
    if (home_all || homeA) {
      homeaxis(A_AXIS);
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) DEBUG_POS("> homeA", current_position);
      #endif
    }

    // Home B
    if (home_all || homeB) {
      homeaxis(B_AXIS);
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) DEBUG_POS("> homeB", current_position);
      #endif
    }
    
    // Home Z
    if (home_all || homeZ) {
      homeaxis(Z_AXIS);
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) DEBUG_POS("> homeZ", current_position);
      #endif
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("<<< home_scara", current_position);
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
      Nextion_gfx_clear();
    #endif

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      bedlevel.set_bed_leveling_enabled(ubl_state_at_entry);
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
      if (printer.debugLeveling()) SERIAL_EM("<<< gcode_G28");
    #endif
  }

  /**
   * Set an axis' current position to its home position (after homing).
   *
   * SCARA should wait until all XY homing is done before setting the XY
   * current_position to home, because neither X nor Y is at home until
   * both are at home. Z can however be homed individually.
   *
   * Callers must sync the planner position after calling this!
   */ 
  void Scara_Mechanics::set_axis_is_at_home(const AxisEnum axis) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_ECHOPAIR(">>> set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHAR(')');
        SERIAL_EOL();
      }
    #endif

    printer.setAxisHomed(axis, true);

    #if ENABLED(MORGAN_SCARA)

      /**
       * Morgan SCARA homes XY at the same time
       */
      if (axis == X_AXIS || axis == Y_AXIS) {

        float homeposition[XYZ];
        LOOP_XYZ(i) homeposition[i] = base_home_pos[(AxisEnum)i];

        /**
         * Get Home position SCARA arm angles using inverse kinematics,
         * and calculate homing offset using forward kinematics
         */
        Transform(homeposition);
        InverseTransform(delta[A_AXIS], delta[B_AXIS], cartesian_position);

        // SERIAL_ECHOPAIR("Cartesian X:", cartes[X_AXIS]);
        // SERIAL_ECHOLNPAIR(" Y:", cartes[Y_AXIS]);

        current_position[axis] = cartesian_position[axis];

        /**
         * SCARA home positions are based on configuration since the actual
         * limits are determined by the inverse kinematic transform.
         */
        endstops.soft_endstop_min[axis] = base_min_pos[axis]; // + (cartes[axis] - base_home_pos(axis));
        endstops.soft_endstop_max[axis] = base_max_pos[axis]; // + (cartes[axis] - base_home_pos(axis));
      }
      else
    #endif
    {
      current_position[axis] = base_home_pos[axis];
    }

    /**
     * Z Probe Z Homing? Account for the probe's Z offset.
     */
    #if HAS_BED_PROBE && Z_HOME_DIR < 0
      if (axis == Z_AXIS) {
        #if HOMING_Z_WITH_PROBE

          current_position[Z_AXIS] -= probe.offset[Z_AXIS];

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (printer.debugLeveling()) {
              SERIAL_EM("*** Z HOMED WITH PROBE ***");
              SERIAL_EMV("> zprobe_zoffset = ", probe.offset[Z_AXIS]);
            }
          #endif

        #elif ENABLED(DEBUG_LEVELING_FEATURE)

          if (printer.debugLeveling()) SERIAL_EM("*** Z HOMED TO ENDSTOP ***");

        #endif
      }
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        DEBUG_POS("", current_position);
        SERIAL_MT("<<< set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')');
        SERIAL_EOL();
      }
    #endif

  }

  // Report detail current position to host
  void Scara_Mechanics::report_current_position_detail() {

    SERIAL_MSG("\nLogical:");
    const float logical[XYZ] = {
      LOGICAL_X_POSITION(current_position[X_AXIS]),
      LOGICAL_Y_POSITION(current_position[Y_AXIS]),
      LOGICAL_Z_POSITION(current_position[Z_AXIS])
    };
    report_xyz(logical);

    float leveled[XYZ] = { current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] };

    #if PLANNER_LEVELING

      SERIAL_MSG("Leveled:");
      bedlevel.apply_leveling(leveled);
      report_xyz(leveled);

      SERIAL_MSG("UnLevel:");
      float unleveled[XYZ] = { leveled[X_AXIS], leveled[Y_AXIS], leveled[Z_AXIS] };
      bedlevel.unapply_leveling(unleveled);
      report_xyz(unleveled);

    #endif

    SERIAL_MSG("ScaraK: ");
    Transform(leveled);
    report_xyz(delta);

    planner.synchronize();

    SERIAL_MSG("Stepper:");
    LOOP_XYZE(i) {
      SERIAL_CHR(' ');
      SERIAL_CHR(axis_codes[i]);
      SERIAL_CHR(':');
      SERIAL_VAL(stepper.position((AxisEnum)i));
      SERIAL_MSG("    ");
    }
    SERIAL_EOL();

    const float deg[XYZ] = {
      planner.get_axis_position_degrees(A_AXIS),
      planner.get_axis_position_degrees(B_AXIS)
    };
    SERIAL_MSG("Degrees:");
    report_xyze(deg, 2);

    SERIAL_MSG("FromStp:");
    get_cartesian_from_steppers();  // writes cartesian_position[XYZ] (with forward kinematics)
    const float from_steppers[XYZE] = { cartesian_position[X_AXIS], cartesian_position[Y_AXIS], cartesian_position[Z_AXIS], planner.get_axis_position_mm(E_AXIS) };
    report_xyze(from_steppers);

    const float diff[XYZE] = {
      from_steppers[X_AXIS] - leveled[X_AXIS],
      from_steppers[Y_AXIS] - leveled[Y_AXIS],
      from_steppers[Z_AXIS] - leveled[Z_AXIS],
      from_steppers[E_AXIS] - current_position[E_AXIS]
    };

    SERIAL_MSG("Differ: ");
    report_xyze(diff);

  }

  void Scara_Mechanics::do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s/*=0.0*/) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV(">>> do_homing_move(", axis_codes[axis]);
        SERIAL_MV(", ", distance);
        SERIAL_MSG(", ");
        if (fr_mm_s)
          SERIAL_VAL(fr_mm_s);
        else {
          SERIAL_MV(" [", homing_feedrate_mm_s[axis]);
          SERIAL_CHR(']');
        }
        SERIAL_CHR(')');
        SERIAL_EOL();
      }
    #endif

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      const bool deploy_bltouch = (axis == Z_AXIS && distance < 0.0);
      if (deploy_bltouch) probe.set_bltouch_deployed(true);
    #endif

    #if QUIET_PROBING
      if (axis == Z_AXIS) probe.probing_pause(true);
    #endif

    // Tell the planner we're at Z=0
    current_position[axis] = 0;

    sync_plan_position_mech_specific();
    current_position[axis] = distance;
    Transform(current_position);
    planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], tools.active_extruder);

    planner.synchronize();

    #if QUIET_PROBING
      if (axis == Z_AXIS) probe.probing_pause(false);
    #endif

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      if (deploy_bltouch) probe.set_bltouch_deployed(false);
    #endif

    endstops.validate_homing_move();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV("<<< do_homing_move(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  #if DISABLED(DISABLE_M503)

    void Scara_Mechanics::print_parameters() {

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

    }

  #endif // DISABLED(DISABLE_M503)

#endif // IS_SCARA

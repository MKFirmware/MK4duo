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

  void Scara_Mechanics::Init() {
    // TODO!!!
  }

  /**
   * Report current position to host
   */
  void Scara_Mechanics::report_current_position() {
    SERIAL_MV( "X:", current_position[X_AXIS], 2);
    SERIAL_MV(" Y:", current_position[Y_AXIS], 2);
    SERIAL_MV(" Z:", current_position[Z_AXIS], 3);
    SERIAL_EMV(" E:", current_position[E_AXIS], 4);

    stepper.report_positions();

    SERIAL_MV("SCARA Theta:", planner.get_axis_position_degrees(A_AXIS));
    SERIAL_EMV("   Psi+Theta:", planner.get_axis_position_degrees(B_AXIS));
  }

  void Scara_Mechanics::report_current_position_detail() {

    planner.synchronize();

    SERIAL_MSG("\nLogical:");
    report_xyze(current_position);

    SERIAL_MSG("Raw:    ");
    const float raw[XYZ] = { NATIVE_X_POSITION(current_position[X_AXIS]), NATIVE_Y_POSITION(current_position[Y_AXIS]), NATIVE_Z_POSITION(current_position[Z_AXIS]) };
    report_xyz(raw);

    float leveled[XYZ] = { current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] };

    #if HAS_LEVELING

      SERIAL_MSG("Leveled:");
      bedlevel.apply_leveling(leveled);
      report_xyz(leveled);

      SERIAL_MSG("UnLevel:");
      float unleveled[XYZ] = { leveled[X_AXIS], leveled[Y_AXIS], leveled[Z_AXIS] };
      bedlevel.unapply_leveling(unleveled);
      report_xyz(unleveled);

    #endif

    SERIAL_MSG("ScaraK: ");
    inverse_kinematics_SCARA(leveled);  // writes delta[]
    report_xyz(delta);

    SERIAL_MSG("Stepper:");
    const long step_count[XYZE] = { stepper.position(X_AXIS), stepper.position(Y_AXIS), stepper.position(Z_AXIS), stepper.position(E_AXIS) };
    report_xyze((float*)step_count, 4, 0);

    SERIAL_MSG("Degrees:");
    const float deg[XYZ] = { planner.get_axis_position_degrees(A_AXIS), planner.get_axis_position_degrees(B_AXIS) };
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

  /**
   * Get the stepper positions in the cartes[] array.
   * Forward kinematics are applied for DELTA and SCARA.
   *
   * The result is in the current coordinate space with
   * leveling applied. The coordinates need to be run through
   * unapply_leveling to obtain the "ideal" coordinates
   * suitable for current_position, etc.
   */
  void Scara_Mechanics::get_cartesian_from_steppers() {
    forward_kinematics_SCARA( planner.get_axis_position_degrees(A_AXIS), planner.get_axis_position_degrees(B_AXIS) );
    cartesian_position[Z_AXIS] = planner.get_axis_position_mm(Z_AXIS);
  }

   /**
   * Prepare a linear move in a SCARA setup.
   *
   * This calls planner.buffer_line several times, adding
   * small incremental moves for SCARA.
   */
   bool Scara_Mechanics::prepare_move_to_destination_mech_specific() {

    // Get the top feedrate of the move in the XY plane
    const float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

    // If the move is only in Z/E don't split up the move
    if (destination[X_AXIS] == current_position[X_AXIS] && destination[Y_AXIS] == current_position[Y_AXIS]) {
      planner.buffer_line_kinematic(destination, _feedrate_mm_s, tools.active_extruder);
      return false;
    }

    // Fail if attempting move outside printable radius
    if (!position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) return true;

    // Get the cartesian distances moved in XYZE
    const float difference[XYZE] = {
      destination[X_AXIS] - current_position[X_AXIS],
      destination[Y_AXIS] - current_position[Y_AXIS],
      destination[Z_AXIS] - current_position[Z_AXIS],
      destination[E_AXIS] - current_position[E_AXIS]
    };

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

    // For SCARA minimum segment size is 0.25mm
    NOMORE(segments, cartesian_mm * 4);

    // At least one segment is required
    NOLESS(segments, 1U);

    // The approximate length of each segment
    const float inv_segments = RECIPROCAL(segments),
                segment_distance[XYZE] = {
                  difference[X_AXIS] * inv_segments,
                  difference[Y_AXIS] * inv_segments,
                  difference[Z_AXIS] * inv_segments,
                  difference[E_AXIS] * inv_segments
                };

    // SERIAL_ECHOPAIR("mm=", cartesian_mm);
    // SERIAL_ECHOPAIR(" seconds=", seconds);
    // SERIAL_ECHOLNPAIR(" segments=", segments);

    #if ENABLED(SCARA_FEEDRATE_SCALING)
      // SCARA needs to scale the feed rate from mm/s to degrees/s
      const float inv_segment_length = MIN(10.0, float(segments) / cartesian_mm), // 1/mm/segs
                  feed_factor = inv_segment_length * _feedrate_mm_s;
      float oldA = planner.get_axis_position_degrees(A_AXIS),
            oldB = planner.get_axis_position_degrees(B_AXIS);
    #endif

    // Get the current position as starting point
    float raw[XYZE];
    COPY(raw, current_position);

    // Drop one segment so the last move is to the exact target.
    // If there's only 1 segment, loops will be skipped entirely.
    --segments;

    // Calculate and execute the segments
    for (uint16_t s = segments + 1; --s;) {
      LOOP_XYZE(i) raw[i] += segment_distance[i];
      inverse_kinematics(raw);

      // Adjust Z if bed leveling is enabled
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (bedlevel.leveling_active)
          delta[Z_AXIS] += abl.bilinear_z_offset(raw);
      #endif

      #if ENABLED(SCARA_FEEDRATE_SCALING)
        // For SCARA scale the feed rate from mm/s to degrees/s
        // Use ratio between the length of the move and the larger angle change
        const float adiff = ABS(delta[A_AXIS] - oldA),
                    bdiff = ABS(delta[B_AXIS] - oldB);
        planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], MAX(adiff, bdiff) * feed_factor, tools.active_extruder);
        oldA = delta[A_AXIS];
        oldB = delta[B_AXIS];
      #else
        planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], _feedrate_mm_s, tools.active_extruder);
      #endif
    }

    // Since segment_distance is only approximate,
    // the final move must be to the exact destination.

    #if ENABLED(SCARA_FEEDRATE_SCALING)
      // For SCARA scale the feed rate from mm/s to degrees/s
      // With segments > 1 length is 1 segment, otherwise total length
      inverse_kinematics(rtarget);
      ADJUST_DELTA(rtarget);
      const float adiff = ABS(delta[A_AXIS] - oldA),
                  bdiff = ABS(delta[B_AXIS] - oldB);
      planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], MAX(adiff, bdiff) * feed_factor, tools.active_extruder);
    #else
      planner.buffer_line_kinematic(rtarget, _feedrate_mm_s, tools.active_extruder);
    #endif

    return false;
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

    const int axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        (axis == X_AXIS) ? x_home_dir(tools.tools.active_extruder) :
      #endif
      home_dir[axis];

    // Homing Z towards the bed? Deploy the Z probe or endstop.
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS && probe.set_deployed(true)) return;
    #endif

    // Set a flag for Z motor locking
    #if ENABLED(Z_TWO_ENDSTOPS)
      printer.setHoming(true);
    #endif

    // Fast move towards endstop until triggered
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("Home 1 Fast:");
    #endif

    // Fast move towards endstop until triggered
    do_homing_move(axis, 1.5 * max_length[axis] * axis_home_dir);

    // When homing Z with probe respect probe clearance
    const float bump = axis_home_dir * (
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
      do_homing_move(axis, -bump);

      // Slow move towards endstop until triggered
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("Home 2 Slow:");
      #endif
      do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
    }

    #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
      const bool pos_dir = axis_home_dir > 0;
      #if ENABLED(X_TWO_ENDSTOPS)
        if (axis == X_AXIS) {
          const bool lock_x1 = pos_dir ? (endstops.x_endstop_adj > 0) : (endstops.x_endstop_adj < 0);
          float adj = ABS(endstops.x_endstop_adj);
          if (pos_dir) adj = -adj;
          if (lock_x1) stepper.set_x_lock(true); else stepper.set_x2_lock(true);
          do_homing_move(axis, adj);
          if (lock_x1) stepper.set_x_lock(false); else stepper.set_x2_lock(false);
          stepper.set_homing_flag_x(false);
        }
      #endif
      #if ENABLED(Y_TWO_ENDSTOPS)
        if (axis == Y_AXIS) {
          const bool lock_y1 = pos_dir ? (endstops.y_endstop_adj > 0) : (endstops.y_endstop_adj < 0);
          float adj = ABS(endstops.y_endstop_adj);
          if (pos_dir) adj = -adj;
          if (lock_y1) stepper.set_y_lock(true); else stepper.set_y2_lock(true);
          do_homing_move(axis, adj);
          if (lock_y1) stepper.set_y_lock(false); else stepper.set_y2_lock(false);
          stepper.set_homing_flag_y(false);
        }
      #endif
      #if ENABLED(Z_TWO_ENDSTOPS)
        if (axis == Z_AXIS) {
          const bool lock_z1 = pos_dir ? (endstops.z_endstop_adj > 0) : (endstops.z_endstop_adj < 0);
          float adj = ABS(endstops.z_endstop_adj);
          if (pos_dir) adj = -adj;
          if (lock_z1) stepper.set_z_lock(true); else stepper.set_z2_lock(true);
          do_homing_move(axis, adj);
          if (lock_z1) stepper.set_z_lock(false); else stepper.set_z2_lock(false);
          stepper.set_homing_flag_z(false);
        }
      #endif
    #endif

    set_axis_is_at_home(axis);
    sync_plan_position_mech_specific();

    // Put away the Z probe
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS && probe.set_deployed(false)) return;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  void Scara_Mechanics::do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s/*=0.0*/) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV(">>> do_homing_move(", axis_codes[axis]);
        SERIAL_MV(", ", distance);
        SERIAL_MV(", ", fr_mm_s);
        SERIAL_CHR(')'); SERIAL_EOL();
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
    inverse_kinematics(current_position);
    planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s(axis), tools.tools.active_extruder);

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

  void Scara_Mechanics::set_position_mm_kinematic(const float position[NUM_AXIS]) {
    #if HAS_LEVELING
      float lpos[XYZ] = { position[X_AXIS], position[Y_AXIS], position[Z_AXIS] };
      bedlevel.apply_leveling(lpos);
    #else
      const float * const lpos = position;
    #endif
    
    inverse_kinematics(lpos);
    _set_position_mm(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], position[E_AXIS]);
  }

  void Scara_Mechanics::sync_plan_position_mech_specific() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("sync_plan_position_mech_specific", current_position);
    #endif
    set_position_mm_kinematic(current_position);
  }

  /**
   *  Plan a move to (X, Y, Z) and set the current_position
   *  The final current_position may not be the one that was requested
   */
  void Scara_Mechanics::do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s/*=0.0*/) {
    const float old_feedrate_mm_s = feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, lx, ly, lz);
    #endif

    if (!position_is_reachable(lx, ly)) return;

    set_destination_to_current();

    // If Z needs to raise, do it before moving XY
    if (destination[Z_AXIS] < lz) {
      destination[Z_AXIS] = lz;
      prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS]);
    }

    destination[X_AXIS] = lx;
    destination[Y_AXIS] = ly;
    prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S);

    // If Z needs to lower, do it after moving XY
    if (destination[Z_AXIS] > lz) {
      destination[Z_AXIS] = lz;
      prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS]);
    }

    planner.synchronize();

    feedrate_mm_s = old_feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_ECHOLNPGM("<<< do_blocking_move_to");
    #endif
  }

  bool Scara_Mechanics::position_is_reachable(const float &rx, const float &ry) {
    #if MIDDLE_DEAD_ZONE_R > 0
      const float R2 = HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y);
      return R2 >= sq(float(MIDDLE_DEAD_ZONE_R)) && R2 <= sq(L1 + L2);
    #else
      return HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y) <= sq(L1 + L2);
    #endif
  }

  bool Scara_Mechanics::position_is_reachable_by_probe(const float &rx, const float &ry) {
    // Both the nozzle and the probe must be able to reach the point.
    // This won't work on SCARA since the probe offset rotates with the arm.
    // TODO: fix this
    return position_is_reachable(rx, ry)
        && position_is_reachable(rx - X_PROBE_OFFSET_FROM_EXTRUDER, ry - Y_PROBE_OFFSET_FROM_EXTRUDER);
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
        LOOP_XYZ(i) homeposition[i] = LOGICAL_POSITION(base_home_pos((AxisEnum)i), i);

        // SERIAL_ECHOPAIR("homeposition X:", homeposition[X_AXIS]);
        // SERIAL_ECHOLNPAIR(" Y:", homeposition[Y_AXIS]);

        /**
         * Get Home position SCARA arm angles using inverse kinematics,
         * and calculate homing offset using forward kinematics
         */
        inverse_kinematics(homeposition);
        forward_kinematics_SCARA(delta[A_AXIS], delta[B_AXIS]);

        // SERIAL_ECHOPAIR("Cartesian X:", cartes[X_AXIS]);
        // SERIAL_ECHOLNPAIR(" Y:", cartes[Y_AXIS]);

        current_position[axis] = LOGICAL_POSITION(cartes[axis], axis);

        /**
         * SCARA home positions are based on configuration since the actual
         * limits are determined by the inverse kinematic transform.
         */
        soft_endstop_min[axis] = base_min_pos(axis); // + (cartes[axis] - base_home_pos(axis));
        soft_endstop_max[axis] = base_max_pos(axis); // + (cartes[axis] - base_home_pos(axis));
      }
      else
    #endif
    {
      current_position[axis] = LOGICAL_POSITION(base_home_pos(axis), axis);
    }

    /**
     * Z Probe Z Homing? Account for the probe's Z offset.
     */
    #if HAS_BED_PROBE && Z_HOME_DIR < 0
      if (axis == Z_AXIS) {
        #if HOMING_Z_WITH_PROBE

          current_position[Z_AXIS] -= zprobe_zoffset;

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (printer.debugLeveling()) {
              SERIAL_ECHOLNPGM("*** Z HOMED WITH PROBE (Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN) ***");
              SERIAL_ECHOLNPAIR("> zprobe_zoffset = ", zprobe_zoffset);
            }
          #endif

        #elif ENABLED(DEBUG_LEVELING_FEATURE)

          if (printer.debugLeveling()) SERIAL_ECHOLNPGM("*** Z HOMED TO ENDSTOP (Z_MIN_PROBE_ENDSTOP) ***");

        #endif
      }
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        #if HAS_HOME_OFFSET
          SERIAL_ECHOPAIR("> home_offset[", axis_codes[axis]);
          SERIAL_ECHOLNPAIR("] = ", home_offset[axis]);
        #endif
        DEBUG_POS("", current_position);
        SERIAL_ECHOPAIR("<<< set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHAR(')');
        SERIAL_EOL();
      }
    #endif
  }

#if ENABLED(MORGAN_SCARA)

  bool Scara_Mechanics::move_to_cal(uint8_t delta_a, uint8_t delta_b) {
    if (printer.isRunning()) {
      forward_kinematics_SCARA(delta_a, delta_b);
      destination[X_AXIS] = cartesian_position[X_AXIS];
      destination[Y_AXIS] = cartesian_position[Y_AXIS];
      destination[Z_AXIS] = current_position[Z_AXIS];
      prepare_move_to_destination();
      return true;
    }
    return false;
  }

  /**
   * Morgan SCARA Forward Kinematics. Results in cartes[].
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void Scara_Mechanics::forward_kinematics_SCARA(const float &a, const float &b) {

    float a_sin = sin(RADIANS(a)) * L1,
          a_cos = cos(RADIANS(a)) * L1,
          b_sin = sin(RADIANS(b)) * L2,
          b_cos = cos(RADIANS(b)) * L2;

    cartes[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  //theta
    cartes[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  //theta+phi

    /*
      SERIAL_ECHOPAIR("SCARA FK Angle a=", a);
      SERIAL_ECHOPAIR(" b=", b);
      SERIAL_ECHOPAIR(" a_sin=", a_sin);
      SERIAL_ECHOPAIR(" a_cos=", a_cos);
      SERIAL_ECHOPAIR(" b_sin=", b_sin);
      SERIAL_ECHOLNPAIR(" b_cos=", b_cos);
      SERIAL_ECHOPAIR(" cartes[X_AXIS]=", cartes[X_AXIS]);
      SERIAL_ECHOLNPAIR(" cartes[Y_AXIS]=", cartes[Y_AXIS]);
    //*/
  }

  /**
   * Morgan SCARA Inverse Kinematics. Results in delta[].
   *
   * See http://forums.reprap.org/read.php?185,283327
   *
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void Scara_Mechanics::inverse_kinematics_SCARA(const float logical[XYZ]) {

    static float C2, S2, SK1, SK2, THETA, PSI;

    float sx = NATIVE_X_POSITION(logical[X_AXIS]) - SCARA_OFFSET_X,  // Translate SCARA to standard X Y
          sy = NATIVE_Y_POSITION(logical[Y_AXIS]) - SCARA_OFFSET_Y;  // With scaling factor.

    if (L1 == L2)
      C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
    else
      C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0 * L1 * L2);

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
    delta[C_AXIS] = logical[Z_AXIS];

    /*
      DEBUG_POS("SCARA IK", logical);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOPAIR("  SCARA (x,y) ", sx);
      SERIAL_ECHOPAIR(",", sy);
      SERIAL_ECHOPAIR(" C2=", C2);
      SERIAL_ECHOPAIR(" S2=", S2);
      SERIAL_ECHOPAIR(" Theta=", THETA);
      SERIAL_ECHOLNPAIR(" Phi=", PHI);
    //*/
  }

#endif // MORGAN_SCARA

#endif // IS_SCARA

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
 * delta_mechanics.cpp
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../../../MK4duo.h"
#include "delta_mechanics.h"

#if IS_DELTA

  Delta_Mechanics mechanics;

  void Delta_Mechanics::init() {
    delta_diagonal_rod              = (float)DELTA_DIAGONAL_ROD;
    delta_radius                    = (float)DELTA_RADIUS;
    delta_segments_per_second       = (float)DELTA_SEGMENTS_PER_SECOND;
    delta_print_radius              = (float)DELTA_PRINTABLE_RADIUS;
    delta_probe_radius              = (float)DELTA_PROBEABLE_RADIUS;
    delta_height                    = (float)DELTA_HEIGHT;
    delta_endstop_adj[A_AXIS]       = (float)TOWER_A_ENDSTOP_ADJ;
    delta_endstop_adj[B_AXIS]       = (float)TOWER_B_ENDSTOP_ADJ;
    delta_endstop_adj[C_AXIS]       = (float)TOWER_C_ENDSTOP_ADJ;
    delta_tower_angle_adj[A_AXIS]   = (float)TOWER_A_ANGLE_ADJ;
    delta_tower_angle_adj[B_AXIS]   = (float)TOWER_B_ANGLE_ADJ;
    delta_tower_angle_adj[C_AXIS]   = (float)TOWER_C_ANGLE_ADJ;
    delta_tower_radius_adj[A_AXIS]  = (float)TOWER_A_RADIUS_ADJ;
    delta_tower_radius_adj[B_AXIS]  = (float)TOWER_B_RADIUS_ADJ;
    delta_tower_radius_adj[C_AXIS]  = (float)TOWER_C_RADIUS_ADJ;
    delta_diagonal_rod_adj[A_AXIS]  = (float)TOWER_A_DIAGROD_ADJ;
    delta_diagonal_rod_adj[B_AXIS]  = (float)TOWER_B_DIAGROD_ADJ;
    delta_diagonal_rod_adj[C_AXIS]  = (float)TOWER_C_DIAGROD_ADJ;
    delta_clip_start_height         = (float)DELTA_HEIGHT;

    recalc_delta_settings();
  }

  // Return true if the given point is within the printable area
  bool Delta_Mechanics::position_is_reachable(const float &rx, const float &ry) {
    return HYPOT2(rx, ry) <= sq(delta_print_radius);
  }
  // Return true if the both nozzle and the probe can reach the given point.
  bool Delta_Mechanics::position_is_reachable_by_probe(const float &rx, const float &ry) {
    return position_is_reachable(rx, ry)
        && position_is_reachable(rx - probe.offset[X_AXIS], ry - probe.offset[Y_AXIS]);
  }

  void Delta_Mechanics::set_position_mm(ARG_X, ARG_Y, ARG_Z, const float &e) {
    _set_position_mm(rx, ry, rz, e);
  }

  void Delta_Mechanics::set_position_mm(const float position[NUM_AXIS]) {
    #if PLANNER_LEVELING
      float lpos[XYZ] = { position[X_AXIS], position[Y_AXIS], position[Z_AXIS] };
      bedlevel.apply_leveling(lpos);
    #else
      const float * const lpos = position;
    #endif
    Transform(lpos);
    _set_position_mm(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], position[E_AXIS]);
  }

  /**
   * Get the stepper positions in the cartesian_position[] array.
   * Forward kinematics are applied for DELTA.
   *
   * The result is in the current coordinate space with
   * leveling applied. The coordinates need to be run through
   * unapply_leveling to obtain the "ideal" coordinates
   * suitable for current_position, etc.
   */
  void Delta_Mechanics::get_cartesian_from_steppers() {
    InverseTransform(
      stepper.get_axis_position_mm(A_AXIS),
      stepper.get_axis_position_mm(B_AXIS),
      stepper.get_axis_position_mm(C_AXIS),
      cartesian_position
    );
  }

  #if DISABLED(AUTO_BED_LEVELING_UBL)

    /**
     * Prepare a linear move in a DELTA setup.
     *
     * This calls buffer_line several times, adding
     * small incremental moves for DELTA.
     */
    bool Delta_Mechanics::prepare_move_to_destination_mech_specific() {

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
      if (endstops.isSoftEndstop() && !position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) return true;

      // Get the linear distance in XYZ
      float cartesian_mm = SQRT(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));

      // If the move is very short, check the E move distance
      if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = FABS(difference[E_AXIS]);

      // No E move either? Game over.
      if (UNEAR_ZERO(cartesian_mm)) return true;

      // Minimum number of seconds to move the given distance
      const float seconds = cartesian_mm / _feedrate_mm_s;

      // The number of segments-per-second times the duration
      // gives the number of segments we should produce
      uint16_t segments = delta_segments_per_second * seconds;

      // At least one segment is required
      NOLESS(segments, 1);

      // The approximate length of each segment
      const float inv_segments = 1.0 / float(segments),
                  cartesian_segment_mm = cartesian_mm * inv_segments,
                  segment_distance[XYZE] = {
                    difference[X_AXIS] * inv_segments,
                    difference[Y_AXIS] * inv_segments,
                    difference[Z_AXIS] * inv_segments,
                    difference[E_AXIS] * inv_segments
                  };

      //SERIAL_MV("mm=", cartesian_mm);
      //SERIAL_MV(" seconds=", seconds);
      //SERIAL_EMV(" segments=", segments);
      //SERIAL_EMV(" segment_mm=", cartesian_segment_mm);

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

        planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], _feedrate_mm_s, tools.active_extruder, cartesian_segment_mm);

      }

      planner.buffer_line_kinematic(destination, _feedrate_mm_s, tools.active_extruder, cartesian_segment_mm);

      return false; // caller will update current_position
    }

  #endif // DISABLED(AUTO_BED_LEVELING_UBL)

  /**
   *  Plan a move to (X, Y, Z) and set the current_position
   *  The final current_position may not be the one that was requested
   */
  void Delta_Mechanics::do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s /*=0.0*/) {
    const float old_feedrate_mm_s = feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, rx, ry, rz);
    #endif

    const float z_feedrate = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];

    if (!position_is_reachable(rx, ry)) return;

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

    set_destination_to_current();          // sync destination at the start

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("set_destination_to_current", destination);
    #endif

    // when in the danger zone
    if (current_position[C_AXIS] > delta_clip_start_height) {
      if (rz > delta_clip_start_height) {   // staying in the danger zone
        destination[A_AXIS] = rx;           // move directly (uninterpolated)
        destination[B_AXIS] = ry;
        destination[C_AXIS] = rz;
        prepare_uninterpolated_move_to_destination(); // set_current_to_destination
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling()) DEBUG_POS("danger zone move", current_position);
        #endif
        return;
      }
      destination[C_AXIS] = delta_clip_start_height;
      prepare_uninterpolated_move_to_destination(); // set_current_to_destination
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) DEBUG_POS("zone border move", current_position);
      #endif
    }

    if (rz > current_position[C_AXIS]) {    // raising?
      destination[C_AXIS] = rz;
      prepare_uninterpolated_move_to_destination(z_feedrate);   // set_current_to_destination
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) DEBUG_POS("z raise move", current_position);
      #endif
    }

    destination[A_AXIS] = rx;
    destination[B_AXIS] = ry;
    prepare_move_to_destination();         // set_current_to_destination
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("xy move", current_position);
    #endif

    if (rz < current_position[C_AXIS]) {    // lowering?
      destination[C_AXIS] = rz;
      prepare_uninterpolated_move_to_destination(z_feedrate);   // set_current_to_destination
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) DEBUG_POS("z lower move", current_position);
      #endif
    }

    stepper.synchronize();

    feedrate_mm_s = old_feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("<<< do_blocking_move_to");
    #endif
  }

  /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void Delta_Mechanics::prepare_uninterpolated_move_to_destination(const float fr_mm_s/*=0.0*/) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);
    #endif

    commands.refresh_cmd_timeout();

    #if UBL_DELTA
      // ubl segmented line will do z-only moves in single segment
      ubl.prepare_segmented_line_to(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s));
    #else
      if ( current_position[A_AXIS] == destination[A_AXIS]
        && current_position[B_AXIS] == destination[B_AXIS]
        && current_position[C_AXIS] == destination[C_AXIS]
        && current_position[E_AXIS] == destination[E_AXIS]
      ) return;

      planner.buffer_line_kinematic(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), tools.active_extruder);
    #endif

    set_current_to_destination();
  }

  /**
   * Delta InverseTransform
   *
   * See the Wikipedia article "Trilateration"
   * https://en.wikipedia.org/wiki/Trilateration
   *
   * Establish a new coordinate system in the plane of the
   * three carriage points. This system has its origin at
   * tower1, with tower2 on the X axis. Tower3 is in the X-Y
   * plane with a Z component of zero.
   * We will define unit vectors in this coordinate system
   * in our original coordinate system. Then when we calculate
   * the Xnew, Ynew and Znew values, we can translate back into
   * the original system by moving along those unit vectors
   * by the corresponding values.
   *
   * Variable names matched to Mk4duo, c-version, and avoid the
   * use of any vector library.
   *
   * by Andreas Hardtung 2016-06-07
   * based on a Java function from "Delta Robot Mechanics V3"
   * by Steve Graves
   *
   * The result is stored in the cartesian[] array.
   */
  void Delta_Mechanics::InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[XYZ]) {

    const float Fa = coreFa + sq(Ha);
    const float Fb = coreFb + sq(Hb);
    const float Fc = coreFc + sq(Hc);

    // Setup PQRSU such that x = -(S - uz)/P, y = (P - Rz)/Q
    const float P = (Xbc * Fa) + (Xca * Fb) + (Xab * Fc);
    const float S = (Ybc * Fa) + (Yca * Fb) + (Yab * Fc);

    const float R = 2 * ((Xbc * Ha) + (Xca * Hb) + (Xab * Hc));
    const float U = 2 * ((Ybc * Ha) + (Yca * Hb) + (Yab * Hc));

    const float R2 = sq(R), U2 = sq(U);

    const float A = U2 + R2 + Q2;
    const float minusHalfB = S * U + P * R + Ha * Q2 + towerX[A_AXIS] * U * Q - towerY[A_AXIS] * R * Q;
    const float C = sq(S + towerX[A_AXIS] * Q) + sq(P - towerY[A_AXIS] * Q) + (sq(Ha) - D2) * Q2;

    const float z = (minusHalfB - SQRT(sq(minusHalfB) - A * C)) / A;

    cartesian[X_AXIS] = (U * z - S) / Q;
    cartesian[Y_AXIS] = (P - R * z) / Q;
    cartesian[Z_AXIS] = z;
  }

  void Delta_Mechanics::recalc_delta_settings() {

    // Get a minimum radius for clamping
    endstops.soft_endstop_radius_2 = sq(delta_print_radius);

    delta_diagonal_rod_2[A_AXIS] = sq(delta_diagonal_rod + delta_diagonal_rod_adj[A_AXIS]);
    delta_diagonal_rod_2[B_AXIS] = sq(delta_diagonal_rod + delta_diagonal_rod_adj[B_AXIS]);
    delta_diagonal_rod_2[C_AXIS] = sq(delta_diagonal_rod + delta_diagonal_rod_adj[C_AXIS]);

    // Effective X/Y positions of the three vertical towers.
    towerX[A_AXIS] = COS(RADIANS(210 + delta_tower_angle_adj[A_AXIS])) * (delta_radius + delta_tower_radius_adj[A_AXIS]); // front left tower
    towerY[A_AXIS] = SIN(RADIANS(210 + delta_tower_angle_adj[A_AXIS])) * (delta_radius + delta_tower_radius_adj[A_AXIS]);
    towerX[B_AXIS] = COS(RADIANS(330 + delta_tower_angle_adj[B_AXIS])) * (delta_radius + delta_tower_radius_adj[B_AXIS]); // front right tower
    towerY[B_AXIS] = SIN(RADIANS(330 + delta_tower_angle_adj[B_AXIS])) * (delta_radius + delta_tower_radius_adj[B_AXIS]);
    towerX[C_AXIS] = COS(RADIANS( 90 + delta_tower_angle_adj[C_AXIS])) * (delta_radius + delta_tower_radius_adj[C_AXIS]); // back middle tower
    towerY[C_AXIS] = SIN(RADIANS( 90 + delta_tower_angle_adj[C_AXIS])) * (delta_radius + delta_tower_radius_adj[C_AXIS]);

    Xbc = towerX[C_AXIS] - towerX[B_AXIS];
    Xca = towerX[A_AXIS] - towerX[C_AXIS];
    Xab = towerX[B_AXIS] - towerX[A_AXIS];
    Ybc = towerY[C_AXIS] - towerY[B_AXIS];
    Yca = towerY[A_AXIS] - towerY[C_AXIS];
    Yab = towerY[B_AXIS] - towerY[A_AXIS];
    coreFa = HYPOT2(towerX[A_AXIS], towerY[A_AXIS]);
    coreFb = HYPOT2(towerX[B_AXIS], towerY[B_AXIS]);
    coreFc = HYPOT2(towerX[C_AXIS], towerY[C_AXIS]);
    Q = 2 * (Xca * Yab - Xab * Yca);
    Q2 = sq(Q);
    D2 = sq(delta_diagonal_rod);

    NOMORE(delta_probe_radius, delta_print_radius);

    printer.unsetHomedAll();

    Set_clip_start_height();

  }

  #if ENABLED(DELTA_FAST_SQRT) && ENABLED(__AVR__)

    /**
     * Fast inverse SQRT from Quake III Arena
     * See: https://en.wikipedia.org/wiki/Fast_inverse_square_root
     */
    float Delta_Mechanics::Q_rsqrt(float number) {
      long i;
      float x2, y;
      const float threehalfs = 1.5f;
      x2 = number * 0.5f;
      y  = number;
      i  = * ( long * ) &y;                         // evil floating point bit level hacking
      i  = 0x5F3759DF - ( i >> 1 );
      y  = * ( float * ) &i;
      y  = y * ( threehalfs - ( x2 * y * y ) );     // 1st iteration
      // y  = y * ( threehalfs - ( x2 * y * y ) );  // 2nd iteration, this can be removed
      return y;
    }

    #define _SQRT(n) (1.0f / Q_rsqrt(n))

  #else

    #define _SQRT(n) SQRT(n)

  #endif

  /**
   * Delta Transform
   *
   * Calculate the tower positions for a given machine
   * position, storing the result in the delta[] array.
   *
   * This is an expensive calculation, requiring 3 square
   * roots per segmented linear move, and strains the limits
   * of a Mega2560 with a Graphical Display.
   */
  void Delta_Mechanics::Transform(const float raw[]) {
    delta[A_AXIS] = raw[Z_AXIS] + _SQRT(delta_diagonal_rod_2[A_AXIS] - HYPOT2(towerX[A_AXIS] - raw[X_AXIS], towerY[A_AXIS] - raw[Y_AXIS]));
    delta[B_AXIS] = raw[Z_AXIS] + _SQRT(delta_diagonal_rod_2[B_AXIS] - HYPOT2(towerX[B_AXIS] - raw[X_AXIS], towerY[B_AXIS] - raw[Y_AXIS]));
    delta[C_AXIS] = raw[Z_AXIS] + _SQRT(delta_diagonal_rod_2[C_AXIS] - HYPOT2(towerX[C_AXIS] - raw[X_AXIS], towerY[C_AXIS] - raw[Y_AXIS]));
  }

  void Delta_Mechanics::Transform_buffer_segment(const float raw[], const float fr) {
    Transform(raw);
    planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], fr, tools.active_extruder);
  }

  void Delta_Mechanics::Set_clip_start_height() {
    float cartesian[XYZ] = { 0, 0, 0 };
    Transform(cartesian);
    float distance = delta[A_AXIS];
    cartesian[Y_AXIS] = delta_print_radius;
    Transform(cartesian);
    delta_clip_start_height = delta_height - FABS(distance - delta[A_AXIS]);
  }

  void Delta_Mechanics::homeaxis(const AxisEnum axis) {

    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV(">>> homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }

      if (printer.debugLeveling()) SERIAL_EM("Home 1 Fast:");
    #endif
    
    // Fast move towards endstop until triggered
    do_homing_move(axis, 1.5 * delta_height);

    // When homing Z with probe respect probe clearance
    const float bump = home_bump_mm[axis];

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

    // Delta has already moved all three towers up in G28
    // so here it re-homes each tower in turn.
    // Delta homing treats the axes as normal linear axes.

    // retrace by the amount specified in delta_endstop_adj + additional 0.1mm in order to have minimum steps
    if (delta_endstop_adj[axis] < 0) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("delta_endstop_adj:");
      #endif
      do_homing_move(axis, delta_endstop_adj[axis] - 0.1);
    }

    // Clear retracted status if homing the Z axis
    #if ENABLED(FWRETRACT)
      if (axis == Z_AXIS)
        for (uint8_t i = 0; i < EXTRUDERS; i++) fwretract.retracted[i] = false;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  /**
   * Home Delta
   */
  bool Delta_Mechanics::home(const bool always_home_all) {

    UNUSED(always_home_all);

    if (printer.debugSimulation()) {
      LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        Nextion_gfx_clear();
      #endif
      return true;
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_EM(">>> gcode_G28");
        log_machine_info();
      }
    #endif

    #if HAS_POWER_SWITCH
      if (!powerManager.lastPowerOn) powerManager.power_on(); // Power On if power is off
    #endif

    // Wait for planner moves to finish!
    stepper.synchronize();

    // Cancel the active G29 session
    #if HAS_LEVELING && ENABLED(PROBE_MANUALLY)
      bedlevel.g29_in_progress = false;
      #if HAS_NEXTION_MANUAL_BED
        Nextion_ProbeOn();
      #endif
    #endif

    // Disable the leveling matrix before homing
    #if HAS_LEVELING
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        const bool ubl_state_at_entry = bedlevel.leveling_active;
      #endif
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
      if (printer.debugLeveling()) DEBUG_POS(">>> home_delta", current_position);
    #endif

    // Disable stealthChop if used. Enable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      sensorless_homing_per_axis(A_AXIS);
      sensorless_homing_per_axis(B_AXIS);
      sensorless_homing_per_axis(C_AXIS);
    #endif

    // Init the current position of all carriages to 0,0,0
    ZERO(current_position);
    set_position_mm(current_position[A_AXIS], current_position[B_AXIS], current_position[C_AXIS], current_position[E_AXIS]);

    // Move all carriages together linearly until an endstop is hit.
    current_position[A_AXIS] = current_position[B_AXIS] = current_position[C_AXIS] = delta_height + 10;
    feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
    line_to_current_position();
    stepper.synchronize();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      sensorless_homing_per_axis(A_AXIS, false);
      sensorless_homing_per_axis(B_AXIS, false);
      sensorless_homing_per_axis(C_AXIS, false);
    #endif

    // If an endstop was not hit, then damage can occur if homing is continued.
    // This can occur if the delta height is
    // not set correctly.
    if (!(TEST(endstops.hit_bits, X_MAX) ||
          TEST(endstops.hit_bits, Y_MAX) ||
          TEST(endstops.hit_bits, Z_MAX))) {
      LCD_MESSAGEPGM(MSG_ERR_HOMING_FAILED);
      SERIAL_LM(ER, MSG_ERR_HOMING_FAILED);
      return false;
    }

    endstops.hit_on_purpose(); // clear endstop hit flags

    // At least one carriage has reached the top.
    // Now re-home each carriage separately.
    homeaxis(A_AXIS);
    homeaxis(B_AXIS);
    homeaxis(C_AXIS);

    // Set all carriages to their home positions
    // Do this here all at once for Delta, because
    // XYZ isn't ABC. Applying this per-tower would
    // give the impression that they are the same.
    LOOP_XYZ(i) set_axis_is_at_home((AxisEnum)i);

    sync_plan_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("<<< home_delta", current_position);
    #endif

    endstops.setNotHoming();

    #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
      // move to a height where we can use the full xy-area
      do_blocking_move_to_z(delta_clip_start_height);
    #endif

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

    stepper.synchronize();

    // Restore the active tool after homing
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif

    lcd_refresh();

    report_current_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("<<< gcode_G28");
    #endif

    return true;
  }

  void Delta_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    printer.setAxisHomed(axis, true);

    current_position[axis] = (axis == C_AXIS ? delta_height : 0.0);

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        DEBUG_POS("", current_position);
        SERIAL_MV("<<< set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  // Report detail current position to host
  void Delta_Mechanics::report_current_position_detail() {

    stepper.synchronize();

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

    SERIAL_MSG("DeltaK: ");
    Transform(leveled);
    report_xyz(delta);

    SERIAL_MSG("Stepper:");
    LOOP_XYZE(i) {
      SERIAL_CHR(' ');
      SERIAL_CHR(axis_codes[i]);
      SERIAL_CHR(':');
      SERIAL_VAL(stepper.position((AxisEnum)i));
      SERIAL_MSG("    ");
    }
    SERIAL_EOL();

    SERIAL_MSG("FromStp:");
    get_cartesian_from_steppers();
    const float from_steppers[XYZE] = { cartesian_position[X_AXIS], cartesian_position[Y_AXIS], cartesian_position[Z_AXIS], stepper.get_axis_position_mm(E_AXIS) };
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

  #if ENABLED(DELTA_AUTO_CALIBRATION_1)

    // Compute the derivative of height with respect to a parameter at the specified motor endpoints.
    // 'deriv' indicates the parameter as follows:
    // 0, 1, 2 = X, Y, Z tower endstop adjustments
    // 3 = delta radius
    // 4 = X tower correction
    // 5 = Y tower correction
    // 6 = delta_diagonal_rod rod length
    // 7, 8 = X tilt, Y tilt. We scale these by the printable radius to get sensible values in the range -1..1
    float Delta_Mechanics::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) {
      const float perturb = 0.2;      // perturbation amount in mm or degrees
      Delta_Mechanics hiParams(*this), loParams(*this);

      switch(deriv) {
        case 0:
        case 1:
        case 2:
          // Endstop corrections
          break;

        case 3:
          hiParams.delta_radius += perturb;
          loParams.delta_radius -= perturb;
          break;

        case 4:
          hiParams.delta_tower_angle_adj[A_AXIS] += perturb;
          loParams.delta_tower_angle_adj[A_AXIS] -= perturb;
          break;

        case 5:
          hiParams.delta_tower_angle_adj[B_AXIS] += perturb;
          loParams.delta_tower_angle_adj[B_AXIS] -= perturb;
          break;

        case 6:
          hiParams.delta_diagonal_rod += perturb;
          loParams.delta_diagonal_rod -= perturb;
          break;
      }

      hiParams.recalc_delta_settings();
      loParams.recalc_delta_settings();

      float newPos[ABC];

      hiParams.InverseTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc, newPos);
      const float zHi = newPos[C_AXIS];

      loParams.InverseTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc, newPos);
      const float zLo = newPos[C_AXIS];

      return ((float)zHi - (float)zLo) / (2 * perturb);
    }
  
  #endif // ENABLED(DELTA_AUTO_CALIBRATION_1)

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)

    void Delta_Mechanics::Nextion_gfx_clear() {
      gfx_clear(delta_print_radius * 2, delta_print_radius * 2, delta_height);
      gfx_cursor_to(current_position[X_AXIS] + delta_print_radius, current_position[Y_AXIS] + delta_print_radius, current_position[Z_AXIS]);
    }

  #endif

  #if ENABLED(SENSORLESS_HOMING)

    /**
     * Set sensorless homing if the axis has it.
     */
    void Delta_Mechanics::sensorless_homing_per_axis(const AxisEnum axis, const bool enable/*=true*/) {
      switch (axis) {
        case X_AXIS: tmc_sensorless_homing(stepperX, enable); break;
        case Y_AXIS: tmc_sensorless_homing(stepperY, enable); break;
        case Z_AXIS: tmc_sensorless_homing(stepperZ, enable); break;
      }
    }

  #endif // SENSORLESS_HOMING

#endif // IS_DELTA

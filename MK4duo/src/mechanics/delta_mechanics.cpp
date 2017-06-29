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
 * delta_mechanics.cpp
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../base.h"

#if IS_DELTA

  Delta_Mechanics Mechanics;

  void Delta_Mechanics::Init() {
    delta_diagonal_rod              = DELTA_DIAGONAL_ROD;
    delta_radius                    = DEFAULT_DELTA_RADIUS;
    delta_segments_per_second       = DELTA_SEGMENTS_PER_SECOND;
    delta_print_radius              = DELTA_PRINTABLE_RADIUS;
    delta_probe_radius              = DELTA_PRINTABLE_RADIUS - max(abs(X_PROBE_OFFSET_FROM_NOZZLE), abs(Y_PROBE_OFFSET_FROM_NOZZLE));
    delta_height                    = DELTA_HEIGHT;
    delta_endstop_adj[A_AXIS]       = TOWER_A_ENDSTOP_ADJ;
    delta_endstop_adj[B_AXIS]       = TOWER_B_ENDSTOP_ADJ;
    delta_endstop_adj[C_AXIS]       = TOWER_C_ENDSTOP_ADJ;
    delta_tower_radius_adj[A_AXIS]  = TOWER_A_RADIUS_ADJ;
    delta_tower_radius_adj[B_AXIS]  = TOWER_B_RADIUS_ADJ;
    delta_tower_radius_adj[C_AXIS]  = TOWER_C_RADIUS_ADJ;
    delta_tower_pos_adj[A_AXIS]     = TOWER_A_POSITION_ADJ;
    delta_tower_pos_adj[B_AXIS]     = TOWER_B_POSITION_ADJ;
    delta_tower_pos_adj[C_AXIS]     = TOWER_C_POSITION_ADJ;
    delta_diagonal_rod_adj[A_AXIS]  = TOWER_A_DIAGROD_ADJ;
    delta_diagonal_rod_adj[B_AXIS]  = TOWER_B_DIAGROD_ADJ;
    delta_diagonal_rod_adj[C_AXIS]  = TOWER_C_DIAGROD_ADJ;
    delta_clip_start_height         = Z_MAX_POS;

    recalc_delta_settings();
  }

  /**
   * Get an axis position according to stepper position(s)
   */
  float Delta_Mechanics::get_axis_position_mm(AxisEnum axis) {
    return stepper.position(axis) * steps_to_mm[axis];
  }

  /**
   * Directly set the planner XYZ position (and stepper positions)
   * converting mm into steps.
   */
  void Delta_Mechanics::set_position_mm(const float &a, const float &b, const float &c, const float &e) {

    planner.position[A_AXIS] = LROUND(a * axis_steps_per_mm[A_AXIS]),
    planner.position[B_AXIS] = LROUND(b * axis_steps_per_mm[B_AXIS]),
    planner.position[C_AXIS] = LROUND(c * axis_steps_per_mm[C_AXIS]),
    planner.position[E_AXIS] = LROUND(e * axis_steps_per_mm[E_INDEX]);

    #if ENABLED(LIN_ADVANCE)
      planner.position_float[A_AXIS] = a;
      planner.position_float[B_AXIS] = b;
      planner.position_float[C_AXIS] = c;
      planner.position_float[E_AXIS] = e;
    #endif

    stepper.set_position(planner.position[A_AXIS], planner.position[B_AXIS], planner.position[C_AXIS], planner.position[E_AXIS]);
    planner.zero_previous_nominal_speed();
    planner.zero_previous_speed();

  }

  /**
   * Setters for planner position (also setting stepper position).
   */
  void Delta_Mechanics::set_position_mm(const AxisEnum axis, const float &v) {

    #if EXTRUDERS > 1
      const uint8_t axis_index = axis + (axis == E_AXIS ? active_extruder : 0);
    #else
      const uint8_t axis_index = axis;
    #endif

    planner.position[axis] = LROUND(v * axis_steps_per_mm[axis_index]);

    #if ENABLED(LIN_ADVANCE)
      planner.position_float[axis] = v;
    #endif

    stepper.set_position(axis, v);
    planner.zero_previous_speed(axis);

  }

  void Delta_Mechanics::set_position_mm_kinematic(const float position[NUM_AXIS]) {
    #if HAS_LEVELING
      float lpos[XYZ] = { position[X_AXIS], position[Y_AXIS], position[Z_AXIS] };
      bedlevel.apply_leveling(lpos);
    #else
      const float * const lpos = position;
    #endif
    Transform(position);
    set_position_mm(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], position[E_AXIS]);
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
      get_axis_position_mm(A_AXIS),
      get_axis_position_mm(B_AXIS),
      get_axis_position_mm(C_AXIS),
      cartesian_position
    );
    cartesian_position[X_AXIS] += LOGICAL_X_POSITION(0);
    cartesian_position[Y_AXIS] += LOGICAL_Y_POSITION(0);
    cartesian_position[Z_AXIS] += LOGICAL_Z_POSITION(0);
  }

  /**
   * Set the current_position for an axis based on
   * the stepper positions, removing any leveling that
   * may have been applied.
   */
  void Delta_Mechanics::set_current_from_steppers_for_axis(const AxisEnum axis) {
    get_cartesian_from_steppers();
    #if HAS_LEVELING
      bedlevel.unapply_leveling(cartesian_position);
    #endif
    if (axis == ALL_AXES)
      COPY_ARRAY(current_position, cartesian_position);
    else
      current_position[axis] = cartesian_position[axis];
  }

  /**
   * Prepare a linear move in a DELTA setup.
   *
   * This calls buffer_line several times, adding
   * small incremental moves for DELTA.
   */
  void Delta_Mechanics::prepare_move_to_destination() {

    endstops.clamp_to_software_endstops(destination);
    refresh_cmd_timeout();

    #if ENABLED(PREVENT_COLD_EXTRUSION)
      if (!DEBUGGING(DRYRUN)) {
        if (destination[E_AXIS] != current_position[E_AXIS]) {
          if (thermalManager.tooColdToExtrude(active_extruder))
            current_position[E_AXIS] = destination[E_AXIS];
          #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
            if (destination[E_AXIS] - current_position[E_AXIS] > EXTRUDE_MAXLENGTH) {
              current_position[E_AXIS] = destination[E_AXIS];
              SERIAL_LM(ER, MSG_ERR_LONG_EXTRUDE_STOP);
            }
          #endif
        }
      }
    #endif

    // Get the top feedrate of the move in the XY plane
    const float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

    // If the move is only in Z/E don't split up the move
    if (destination[A_AXIS] == current_position[A_AXIS] && destination[B_AXIS] == current_position[B_AXIS]) {
      planner.buffer_line_kinematic(destination, _feedrate_mm_s, active_extruder, active_driver);
      set_current_to_destination();
      return;
    }

    // Fail if attempting move outside printable radius
    if (!position_is_reachable_xy(destination[A_AXIS], destination[B_AXIS])) return;

    // Get the cartesian distances moved in XYZE
    float difference[NUM_AXIS];
    LOOP_XYZE(i) difference[i] = destination[i] - current_position[i];

    // Get the linear distance in XYZ
    float cartesian_mm = SQRT(sq(difference[A_AXIS]) + sq(difference[B_AXIS]) + sq(difference[C_AXIS]));

    // If the move is very short, check the E move distance
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = abs(difference[E_AXIS]);

    // No E move either? Game over.
    if (UNEAR_ZERO(cartesian_mm)) return;

    // Minimum number of seconds to move the given distance
    float seconds = cartesian_mm / _feedrate_mm_s;

    // The number of segments-per-second times the duration
    // gives the number of segments we should produce
    uint16_t segments = delta_segments_per_second * seconds;

    // At least one segment is required
    NOLESS(segments, 1);

    // The approximate length of each segment
    const float inv_segments = 1.0 / float(segments),
                segment_distance[XYZE] = {
                  difference[A_AXIS] * inv_segments,
                  difference[B_AXIS] * inv_segments,
                  difference[C_AXIS] * inv_segments,
                  difference[E_AXIS] * inv_segments
                };

    //SERIAL_MV("mm=", cartesian_mm);
    //SERIAL_MV(" seconds=", seconds);
    //SERIAL_EMV(" segments=", segments);

    // Get the logical current position as starting point
    float logical[XYZE];
    COPY_ARRAY(logical, current_position);

    // Drop one segment so the last move is to the exact target.
    // If there's only 1 segment, loops will be skipped entirely.
    --segments;

    // Calculate and execute the segments
    for (uint16_t s = segments + 1; --s;) {
      LOOP_XYZE(i) logical[i] += segment_distance[i];
      Transform(logical);

      // Adjust Z if bed leveling is enabled
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (bedlevel.abl_enabled) {
          const float zadj = bedlevel.bilinear_z_offset(logical);
          delta[A_AXIS] += zadj;
          delta[B_AXIS] += zadj;
          delta[C_AXIS] += zadj;
        }
      #endif

      planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], _feedrate_mm_s, active_extruder, active_driver);

    }

    planner.buffer_line_kinematic(destination, _feedrate_mm_s, active_extruder, active_driver);

    set_current_to_destination();
  }

  /**
   * line_to_current_position
   * Move the planner to the current position from wherever it last moved
   * (or from wherever it has been told it is located).
   */
  void Delta_Mechanics::line_to_current_position() {
    planner.buffer_line(current_position[A_AXIS], current_position[B_AXIS], current_position[C_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder, active_driver);
  }

  /**
   * line_to_destination
   * Move the planner to the position stored in the destination array, which is
   * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
   */
  void Delta_Mechanics::line_to_destination(float fr_mm_s) {
    planner.buffer_line(destination[A_AXIS], destination[B_AXIS], destination[C_AXIS], destination[E_AXIS], fr_mm_s, active_extruder, active_driver);
  }
  void Delta_Mechanics::line_to_destination() { line_to_destination(feedrate_mm_s); }

  /**
   *  Plan a move to (X, Y, Z) and set the current_position
   *  The final current_position may not be the one that was requested
   */
  void Delta_Mechanics::do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s /*=0.0*/) {
    const float old_feedrate_mm_s = feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, lx, ly, lz);
    #endif

    if (!position_is_reachable_xy(lx, ly)) return;

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

    set_destination_to_current();          // sync destination at the start

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("set_destination_to_current", destination);
    #endif

    // when in the danger zone
    if (current_position[C_AXIS] > delta_clip_start_height) {
      if (lz > delta_clip_start_height) {   // staying in the danger zone
        destination[A_AXIS] = lx;           // move directly (uninterpolated)
        destination[B_AXIS] = ly;
        destination[C_AXIS] = lz;
        prepare_uninterpolated_move_to_destination(); // set_current_to_destination
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("danger zone move", current_position);
        #endif
        return;
      }
      else {
        destination[C_AXIS] = delta_clip_start_height;
        prepare_uninterpolated_move_to_destination(); // set_current_to_destination
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("zone border move", current_position);
        #endif
      }
    }

    if (lz > current_position[C_AXIS]) {    // raising?
      destination[C_AXIS] = lz;
      prepare_uninterpolated_move_to_destination();   // set_current_to_destination
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("z raise move", current_position);
      #endif
    }

    destination[A_AXIS] = lx;
    destination[B_AXIS] = ly;
    prepare_move_to_destination();         // set_current_to_destination
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("xy move", current_position);
    #endif

    if (lz < current_position[C_AXIS]) {    // lowering?
      destination[C_AXIS] = lz;
      prepare_uninterpolated_move_to_destination();   // set_current_to_destination
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("z lower move", current_position);
      #endif
    }

    stepper.synchronize();

    feedrate_mm_s = old_feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< do_blocking_move_to");
    #endif
  }
  void Delta_Mechanics::do_blocking_move_to(const float logical[XYZ], const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(logical[A_AXIS], logical[B_AXIS], logical[C_AXIS], fr_mm_s);
  }
  void Delta_Mechanics::do_blocking_move_to_x(const float &lx, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(lx, current_position[B_AXIS], current_position[C_AXIS], fr_mm_s);
  }
  void Delta_Mechanics::do_blocking_move_to_z(const float &lz, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(current_position[A_AXIS], current_position[B_AXIS], lz, fr_mm_s);
  }
  void Delta_Mechanics::do_blocking_move_to_xy(const float &lx, const float &ly, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(lx, ly, current_position[C_AXIS], fr_mm_s);
  }

  void Delta_Mechanics::sync_plan_position() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position_kinematic", current_position);
    #endif
    set_position_mm_kinematic(current_position);
  }
  void Delta_Mechanics::sync_plan_position_e() {
    set_e_position_mm(current_position[E_AXIS]);
  }

  /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void Delta_Mechanics::prepare_uninterpolated_move_to_destination(const float fr_mm_s/*=0.0*/) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);
    #endif

    refresh_cmd_timeout();

    if ( current_position[A_AXIS] == destination[A_AXIS]
      && current_position[B_AXIS] == destination[B_AXIS]
      && current_position[C_AXIS] == destination[C_AXIS]
      && current_position[E_AXIS] == destination[E_AXIS]
    ) return;

    planner.buffer_line_kinematic(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), active_extruder, active_driver);

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
  void Delta_Mechanics::InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[ABC]) {

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

    const float z = (minusHalfB - sqrtf(sq(minusHalfB) - A * C)) / A;

    cartesian[A_AXIS] = (U * z - S) / Q;
    cartesian[B_AXIS] = (P - R * z) / Q;
    cartesian[C_AXIS] = z;
  }

  void Delta_Mechanics::recalc_delta_settings() {

    LOOP_XY(i) {
      endstops.soft_endstop_min[i] = -delta_print_radius;
      endstops.soft_endstop_max[i] = delta_print_radius;
    }
    endstops.soft_endstop_max[C_AXIS]  = delta_height;
    delta_probe_radius = delta_print_radius - max(abs(X_PROBE_OFFSET_FROM_NOZZLE), abs(Y_PROBE_OFFSET_FROM_NOZZLE));

    delta_diagonal_rod_2[A_AXIS] = sq(delta_diagonal_rod + delta_diagonal_rod_adj[A_AXIS]);
    delta_diagonal_rod_2[B_AXIS] = sq(delta_diagonal_rod + delta_diagonal_rod_adj[B_AXIS]);
    delta_diagonal_rod_2[C_AXIS] = sq(delta_diagonal_rod + delta_diagonal_rod_adj[C_AXIS]);

    // Effective X/Y positions of the three vertical towers.
    towerX[A_AXIS] = -((delta_radius + delta_tower_pos_adj[A_AXIS]) * cos(RADIANS(30 + delta_tower_radius_adj[A_AXIS]))); // front left tower
    towerY[A_AXIS] = -((delta_radius + delta_tower_pos_adj[A_AXIS]) * sin(RADIANS(30 + delta_tower_radius_adj[A_AXIS]))); 
    towerX[B_AXIS] = +((delta_radius + delta_tower_pos_adj[B_AXIS]) * cos(RADIANS(30 - delta_tower_radius_adj[B_AXIS]))); // front right tower
    towerY[B_AXIS] = -((delta_radius + delta_tower_pos_adj[B_AXIS]) * sin(RADIANS(30 - delta_tower_radius_adj[B_AXIS]))); 
    towerX[C_AXIS] = -((delta_radius + delta_tower_pos_adj[C_AXIS]) * sin(RADIANS(     delta_tower_radius_adj[C_AXIS]))); // back middle tower
    towerY[C_AXIS] = +((delta_radius + delta_tower_pos_adj[C_AXIS]) * cos(RADIANS(     delta_tower_radius_adj[C_AXIS]))); 

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

    const float tempHeight = delta_diagonal_rod;		// any sensible height will do here, probably even zero
    float cartesian[ABC];
    InverseTransform(tempHeight, tempHeight, tempHeight, cartesian);
    homed_Height = delta_height + tempHeight - cartesian[C_AXIS];
    printRadiusSquared = sq(delta_print_radius);

    Set_clip_start_height();

  }

  #if ENABLED(DELTA_FAST_SQRT) && DISABLED(MATH_USE_HAL)

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
   * Delta Inverse Mechanics
   *
   * Calculate the tower positions for a given logical
   * position, storing the result in the delta[] array.
   *
   * This is an expensive calculation, requiring 3 square
   * roots per segmented linear move, and strains the limits
   * of a Mega2560 with a Graphical Display.
   */
  void Delta_Mechanics::Transform(const float logical[XYZ]) {
    const float raw[XYZ] = {  RAW_X_POSITION(logical[A_AXIS]),
                              RAW_Y_POSITION(logical[B_AXIS]),
                              RAW_Z_POSITION(logical[C_AXIS])
    };

    delta[A_AXIS] = raw[C_AXIS] + _SQRT(delta_diagonal_rod_2[A_AXIS] - HYPOT2(towerX[A_AXIS] - raw[A_AXIS], towerY[A_AXIS] - raw[B_AXIS]));
    delta[B_AXIS] = raw[C_AXIS] + _SQRT(delta_diagonal_rod_2[B_AXIS] - HYPOT2(towerX[B_AXIS] - raw[A_AXIS], towerY[B_AXIS] - raw[B_AXIS]));
    delta[C_AXIS] = raw[C_AXIS] + _SQRT(delta_diagonal_rod_2[C_AXIS] - HYPOT2(towerX[C_AXIS] - raw[A_AXIS], towerY[C_AXIS] - raw[B_AXIS]));
  }

  void Delta_Mechanics::Transform_segment_raw(const float rx, const float ry, const float rz, const float le, const float fr) {
    const float delta_A = rz + _SQRT(delta_diagonal_rod_2[A_AXIS] - HYPOT2(towerX[A_AXIS] - rx, towerY[A_AXIS] - ry ));
    const float delta_B = rz + _SQRT(delta_diagonal_rod_2[B_AXIS] - HYPOT2(towerX[B_AXIS] - rx, towerY[B_AXIS] - ry ));
    const float delta_C = rz + _SQRT(delta_diagonal_rod_2[C_AXIS] - HYPOT2(towerX[C_AXIS] - rx, towerY[C_AXIS] - ry ));

    planner._buffer_line(delta_A, delta_B, delta_C, le, fr, active_extruder, active_driver);
  }

  void Delta_Mechanics::Set_clip_start_height() {
    float cartesian[XYZ] = {
      LOGICAL_X_POSITION(0),
      LOGICAL_Y_POSITION(0),
      LOGICAL_Z_POSITION(0)
    };
    Transform(cartesian);
    float distance = delta[A_AXIS];
    cartesian[Y_AXIS] = LOGICAL_Y_POSITION(delta_print_radius);
    Transform(cartesian);
    delta_clip_start_height = delta_height - FABS(distance - delta[A_AXIS]);
  }

  // Recalculate the steps/s^2 acceleration rates, based on the mm/s^2
  void Delta_Mechanics::reset_acceleration_rates() {
    #if EXTRUDERS > 1
      #define HIGHEST_CONDITION (i < E_AXIS || i == E_INDEX)
    #else
      #define HIGHEST_CONDITION true
    #endif
    uint32_t highest_rate = 1;
    LOOP_XYZE_N(i) {
      max_acceleration_steps_per_s2[i] = max_acceleration_mm_per_s2[i] * axis_steps_per_mm[i];
      if (HIGHEST_CONDITION) NOLESS(highest_rate, max_acceleration_steps_per_s2[i]);
    }
    planner.cutoff_long = 4294967295UL / highest_rate;
  }

  // Recalculate position, steps_to_mm if axis_steps_per_mm changes!
  void Delta_Mechanics::refresh_positioning() {
    LOOP_XYZE_N(i) steps_to_mm[i] = 1.0 / axis_steps_per_mm[i];
    set_position_mm_kinematic(current_position);
    reset_acceleration_rates();
  }

  void Delta_Mechanics::do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s/*=0.0*/) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV(">>> do_homing_move(", axis_codes[axis]);
        SERIAL_MV(", ", distance);
        SERIAL_MV(", ", fr_mm_s);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    // Tell the planner we're at Z=0
    current_position[axis] = 0;

    set_position_mm(current_position[A_AXIS], current_position[B_AXIS], current_position[C_AXIS], current_position[E_AXIS]);
    current_position[axis] = distance;
    planner.buffer_line(current_position[A_AXIS], current_position[B_AXIS], current_position[C_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], active_extruder, active_driver);

    stepper.synchronize();

    endstops.hit_on_purpose();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("<<< do_homing_move(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  void Delta_Mechanics::homeaxis(const AxisEnum axis) {

    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV(">>> homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }

      if (DEBUGGING(LEVELING)) SERIAL_EM("Home 1 Fast:");
    #endif
    
    // Fast move towards endstop until triggered
    do_homing_move(axis, 1.5 * delta_height);

    // When homing Z with probe respect probe clearance
    const float bump = home_bump_mm[axis];

    // If a second homing move is configured...
    if (bump) {
      // Move away from the endstop by the axis HOME_BUMP_MM
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("Move Away:");
      #endif
      do_homing_move(axis, -bump);

      // Slow move towards endstop until triggered
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("Home 2 Slow:");
      #endif
      do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
    }

    // Delta has already moved all three towers up in G28
    // so here it re-homes each tower in turn.
    // Delta homing treats the axes as normal linear axes.

    // retrace by the amount specified in delta_endstop_adj + additional 0.1mm in order to have minimum steps
    if (delta_endstop_adj[axis] < 0) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("delta_endstop_adj:");
      #endif
      do_homing_move(axis, delta_endstop_adj[axis] - 0.1);
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  /**
   * Home Delta
   */
  void Delta_Mechanics::Home(const bool always_home_all) {

    UNUSED(always_home_all);

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> home_delta", current_position);
    #endif

    // Init the current position of all carriages to 0,0,0
    ZERO(current_position);
    set_position_mm(current_position[A_AXIS], current_position[B_AXIS], current_position[C_AXIS], current_position[E_AXIS]);

    // Move all carriages together linearly until an endstop is hit.
    current_position[A_AXIS] = current_position[B_AXIS] = current_position[C_AXIS] = delta_height + 10;
    feedrate_mm_s = homing_feedrate_mm_s[A_AXIS];
    line_to_current_position();
    stepper.synchronize();
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
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< home_delta", current_position);
    #endif
  }

  void Delta_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    axis_known_position[axis] = axis_homed[axis] = true;

    #if ENABLED(WORKSPACE_OFFSETS)
      position_shift[axis] = 0;
      endstops.update_software_endstops(axis);
    #endif

    current_position[axis] = (axis == C_AXIS ? delta_height : 0.0);

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
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

  float Delta_Mechanics::get_homing_bump_feedrate(const AxisEnum axis) {
    const uint8_t homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
    uint8_t hbd = homing_bump_divisor[axis];
    if (hbd < 1) {
      hbd = 10;
      SERIAL_LM(ER, "Warning: Homing Bump Divisor < 1");
    }
    return homing_feedrate_mm_s[axis] / hbd;
  }

  bool Delta_Mechanics::axis_unhomed_error(const bool x/*=true*/, const bool y/*=true*/, const bool z/*=true*/) {
    const bool  xx = x && !axis_homed[A_AXIS],
                yy = y && !axis_homed[B_AXIS],
                zz = z && !axis_homed[C_AXIS];

    if (xx || yy || zz) {
      SERIAL_SM(ECHO, MSG_HOME " ");
      if (xx) SERIAL_MSG(MSG_X);
      if (yy) SERIAL_MSG(MSG_Y);
      if (zz) SERIAL_MSG(MSG_Z);
      SERIAL_EM(" " MSG_FIRST);

      #if ENABLED(ULTRA_LCD)
        lcd_status_printf_P(0, PSTR(MSG_HOME " %s%s%s " MSG_FIRST), xx ? MSG_X : "", yy ? MSG_Y : "", zz ? MSG_Z : "");
      #endif
      return true;
    }
    return false;
  }
  bool Delta_Mechanics::position_is_reachable_raw_xy(const float &rx, const float &ry) {
    return HYPOT2(rx, ry) <= printRadiusSquared;
  }
  bool Delta_Mechanics::position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry) {
    // both the nozzle and the probe must be able to reach the point
    return  position_is_reachable_raw_xy(rx, ry)
        &&  position_is_reachable_raw_xy(rx - X_PROBE_OFFSET_FROM_NOZZLE, ry - Y_PROBE_OFFSET_FROM_NOZZLE);
  }
  bool Delta_Mechanics::position_is_reachable_by_probe_xy(const float &lx, const float &ly) {
    return position_is_reachable_by_probe_raw_xy(RAW_X_POSITION(lx), RAW_Y_POSITION(ly));
  }
  bool Delta_Mechanics::position_is_reachable_xy(const float &lx, const float &ly) {
    return position_is_reachable_raw_xy(RAW_X_POSITION(lx), RAW_Y_POSITION(ly));
  }

  #if ENABLED(DELTA_AUTO_CALIBRATION_1)

    /**
     * Delta AutoCalibration Algorithm of Minor Squares based on DC42 RepRapFirmware 7 points
     * Usage:
     *    G33 <Fn> <Pn> <Q>
     *      F = Num Factors 3 or 4 or 6 or 7
     *        The input vector contains the following parameters in this order:
     *          X, Y and Z endstop adjustments
     *          Delta radius
     *          X tower position adjustment and Y tower position adjustment
     *          Diagonal rod length adjustment
     *      P = Num probe points 7 or 10
     *      Q = Debugging
     */
    void Delta_Mechanics::auto_calibration() {

      // G33 Q is also available if debugging
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        const bool query = parser.seen('Q');
        const uint8_t old_debug_flags = mk_debug_flags;
        if (query) mk_debug_flags |= DEBUG_LEVELING;
        if (DEBUGGING(LEVELING)) {
          DEBUG_POS(">>> gcode_G33", current_position);
          log_machine_info();
        }
        mk_debug_flags = old_debug_flags;
        #if DISABLED(PROBE_MANUALLY)
          if (query) return;
        #endif
      #endif

      // Define local vars 'static' for manual probing, 'auto' otherwise
      #if ENABLED(PROBE_MANUALLY)
        #define ABL_VAR static
      #else
        #define ABL_VAR
      #endif

      const uint8_t   MaxCalibrationPoints = 10;

      ABL_VAR uint8_t probe_index,
                      numFactors,
                      numPoints;

      ABL_VAR float   xBedProbePoints[MaxCalibrationPoints],
                      yBedProbePoints[MaxCalibrationPoints],
                      zBedProbePoints[MaxCalibrationPoints],
                      initialSumOfSquares,
                      expectedRmsError;
      ABL_VAR char    rply[50];

      #if HAS_SOFTWARE_ENDSTOPS
        ABL_VAR bool enable_soft_endstops = true;
      #endif

      const bool stow = parser.seen('S') ? parser.value_bool() : true;

      /**
       * On the initial G33 fetch command parameters.
       */
      if (!g33_in_progress) {

        numFactors = parser.seen('F') ? constrain(parser.value_int(), 3, 7) : 7;
        numPoints  = parser.seen('P') ? constrain(parser.value_int(), 7, 10) : 7;

        stepper.synchronize();
        #if HAS_LEVELING
          bedlevel.reset_bed_level(); // After calibration bed-level data is no longer valid
        #endif
        #if HOTENDS > 1
          const uint8_t old_tool_index = active_extruder;
          tool_change(0, 0, true);
        #endif
        setup_for_endstop_or_probe_move();

        endstops.enable(true);
        Home();
        endstops.not_homing();

        do_blocking_move_to_z(_Z_PROBE_DEPLOY_HEIGHT, homing_feedrate_mm_s[Z_AXIS]);
        stepper.synchronize();  // wait until the machine is idle

        SERIAL_MV("Starting Auto Calibration ", numPoints);
        SERIAL_MV(" points and ", numFactors);
        SERIAL_EM(" Factors");
        LCD_MESSAGEPGM(MSG_DELTA_AUTO_CALIBRATE);
        probe_index = 0;
        #if HAS_NEXTION_MANUAL_BED
          LcdBedLevelOn();
        #endif
      }

      #if ENABLED(PROBE_MANUALLY)

        // Query G33 status
        if (parser.seen('Q')) {
          if (!g33_in_progress)
            SERIAL_EM("Manual G30 idle");
          else {
            SERIAL_MV("Manual G30 point ", probe_index + 1);
            SERIAL_EMV(" of ", numPoints);
          }
          return;
        }

        // Fall through to probe the first point
        g33_in_progress = true;

        if (probe_index == 0) {
          // For the initial G30 save software endstop state
          #if HAS_SOFTWARE_ENDSTOPS
            enable_soft_endstops = endstops.soft_endstops_enabled;
          #endif
        }
        else {
          // Save the previous Z before going to the next point
          zBedProbePoints[probe_index - 1] = current_position[Z_AXIS];
        }

        // Is there a next point to move to?
        if (probe_index < 6) {
          xBedProbePoints[probe_index] = delta_print_radius * sin((2 * M_PI * probe_index) / 6);
          yBedProbePoints[probe_index] = delta_print_radius * cos((2 * M_PI * probe_index) / 6);
        }
        if (numPoints >= 10) {
          if (probe_index >= 6 && probe_index < 9) {
            xBedProbePoints[probe_index] = (delta_print_radius / 2) * sin((2 * M_PI * (probe_index - 6)) / 3);
            yBedProbePoints[probe_index] = (delta_print_radius / 2) * cos((2 * M_PI * (probe_index - 6)) / 3);
          }
          else if (probe_index >= 9) {
            xBedProbePoints[9] = 0.0;
            yBedProbePoints[9] = 0.0;
          }
        }
        else {
          if (probe_index >= 6) {
            xBedProbePoints[6] = 0.0;
            yBedProbePoints[6] = 0.0;
          }
        }

        // Is there a next point to move to?
        if (probe_index < numPoints) {
          _manual_goto_xy(xBedProbePoints[probe_index], yBedProbePoints[probe_index]); // Can be used here too!
          ++probe_index;
          #if HAS_SOFTWARE_ENDSTOPS
            // Disable software endstops to allow manual adjustment
            // If G29 is not completed, they will not be re-enabled
            endstops.soft_endstops_enabled = false;
          #endif
          return;
        }
        else {
          // Then calibration is done!
          // G33 finishing code goes here

          // After recording the last point, activate abl
          SERIAL_EM("Calibration probing done.");
          g33_in_progress = false;

          // Re-enable software endstops, if needed
          #if HAS_SOFTWARE_ENDSTOPS
            endstops.soft_endstops_enabled = enable_soft_endstops;
          #endif
        }

      #else

        for (probe_index = 0; probe_index < 6; probe_index++) {
          xBedProbePoints[probe_index] = delta_probe_radius * sin((2 * M_PI * probe_index) / 6);
          yBedProbePoints[probe_index] = delta_probe_radius * cos((2 * M_PI * probe_index) / 6);
          zBedProbePoints[probe_index] = probe.check_pt(xBedProbePoints[probe_index], yBedProbePoints[probe_index], false, 4);
        }
        if (numPoints >= 10) {
          for (probe_index = 6; probe_index < 9; probe_index++) {
            xBedProbePoints[probe_index] = (delta_probe_radius / 2) * sin((2 * M_PI * (probe_index - 6)) / 3);
            yBedProbePoints[probe_index] = (delta_probe_radius / 2) * cos((2 * M_PI * (probe_index - 6)) / 3);
            zBedProbePoints[probe_index] = probe.check_pt(xBedProbePoints[probe_index], yBedProbePoints[probe_index], false, 4);
          }
          xBedProbePoints[9] = 0.0;
          yBedProbePoints[9] = 0.0;
          zBedProbePoints[9] = probe.check_pt(0.0, 0.0, true, 4);
        }
        else {
          xBedProbePoints[6] = 0.0;
          yBedProbePoints[6] = 0.0;
          zBedProbePoints[6] = probe.check_pt(0.0, 0.0, true, 4);
        }

      #endif

      // convert delta_endstop_adj;
      Convert_endstop_adj();

      float probeMotorPositions[MaxCalibrationPoints][ABC],
            corrections[MaxCalibrationPoints];

      initialSumOfSquares = 0.0;

      // Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
      for (uint8_t i = 0; i < numPoints; ++i) {
        corrections[i] = 0.0;
        float machinePos[ABC];
        float xp = xBedProbePoints[i], yp = yBedProbePoints[i];

        xp -= X_PROBE_OFFSET_FROM_NOZZLE;
        yp -= Y_PROBE_OFFSET_FROM_NOZZLE;
        machinePos[A_AXIS] = xp;
        machinePos[B_AXIS] = yp;
        machinePos[C_AXIS] = 0.0;

        Transform(machinePos);

        for (uint8_t axis = 0; axis < ABC; axis++)
          probeMotorPositions[i][axis] = delta[axis];

        initialSumOfSquares += sq(zBedProbePoints[i]);
      }

      // Do 1 or more Newton-Raphson iterations
      uint8_t iteration = 0;

      do {
        iteration++;

        float derivativeMatrix[MaxCalibrationPoints][numFactors],
              normalMatrix[numFactors][numFactors + 1];

        for (uint8_t i = 0; i < numPoints; i++) {
          for (uint8_t j = 0; j < numFactors; j++) {
            derivativeMatrix[i][j] =
              ComputeDerivative(j, probeMotorPositions[i][A_AXIS], probeMotorPositions[i][B_AXIS], probeMotorPositions[i][C_AXIS]);
          }
        }

        for (uint8_t i = 0; i < numFactors; i++) {
          for (uint8_t j = 0; j < numFactors; j++) {
            float temp = derivativeMatrix[0][i] * derivativeMatrix[0][j];
            for (uint8_t k = 1; k < numPoints; k++) {
              temp += derivativeMatrix[k][i] * derivativeMatrix[k][j];
            }
            normalMatrix[i][j] = temp;
          }
          float temp = derivativeMatrix[0][i] * -(zBedProbePoints[0] + corrections[0]);
          for (uint8_t k = 1; k < numPoints; k++) {
            temp += derivativeMatrix[k][i] * -(zBedProbePoints[k] + corrections[k]);
          }
          normalMatrix[i][numFactors] = temp;
        }

        // Perform Gauss-Jordan elimination on a N x (N+1) matrix.
        // Returns a pointer to the solution vector.
        for (uint8_t i = 0; i < numFactors; i++) {
          // Swap the rows around for stable Gauss-Jordan elimination
          float vmax = abs(normalMatrix[i][i]);
          for (uint8_t j = i + 1; j < numFactors; j++) {
            const float rmax = abs(normalMatrix[j][i]);
            if (rmax > vmax) {
              // Swap 2 rows of a matrix
              if (i != j) {
                for (uint8_t k = 0; k < numFactors + 1; k++) {
                  const float temp = normalMatrix[i][k];
                  normalMatrix[i][k] = normalMatrix[j][k];
                  normalMatrix[j][k] = temp;
                }
              }
              vmax = rmax;
            }
          }

          // Use row i to eliminate the ith element from previous and subsequent rows
          float v = normalMatrix[i][i];
          for (uint8_t j = 0; j < i; j++)	{
            float factor = normalMatrix[j][i] / v;
            normalMatrix[j][i] = 0.0;
            for (uint8_t k = i + 1; k <= numFactors; k++) {
              normalMatrix[j][k] -= normalMatrix[i][k] * factor;
            }
          }

          for (uint8_t j = i + 1; j < numFactors; j++) {
            float factor = normalMatrix[j][i] / v;
            normalMatrix[j][i] = 0.0;
            for (uint8_t k = i + 1; k <= numFactors; k++) {
              normalMatrix[j][k] -= normalMatrix[i][k] * factor;
            }
          }
        }

        float solution[numFactors];
        for (uint8_t i = 0; i < numFactors; i++)
          solution[i] = normalMatrix[i][numFactors] / normalMatrix[i][i];
        Adjust(numFactors, solution);

        // Calculate the expected probe heights using the new parameters
        float expectedResiduals[MaxCalibrationPoints];
        float sumOfSquares = 0.0;

        for (int8_t i = 0; i < numPoints; i++) {
          LOOP_XYZ(axis) probeMotorPositions[i][axis] += solution[axis];
          float newPosition[ABC];
          InverseTransform(probeMotorPositions[i][A_AXIS], probeMotorPositions[i][B_AXIS], probeMotorPositions[i][C_AXIS], newPosition);
          corrections[i] = newPosition[Z_AXIS];
          expectedResiduals[i] = zBedProbePoints[i] + newPosition[Z_AXIS];
          sumOfSquares += sq(expectedResiduals[i]);
        }

        expectedRmsError = SQRT(sumOfSquares / numPoints);

      } while (iteration < 2);

      // convert delta_endstop_adj;
      Convert_endstop_adj();

      SERIAL_MV("Calibrated ", numFactors);
      SERIAL_MV(" factors using ", numPoints);
      SERIAL_MV(" points, deviation before ", SQRT(initialSumOfSquares / numPoints), 4);
      SERIAL_MV(" after ", expectedRmsError, 4);
      SERIAL_EOL();

      recalc_delta_settings();

      SERIAL_MV("Endstops X", delta_endstop_adj[A_AXIS], 3);
      SERIAL_MV(" Y", delta_endstop_adj[B_AXIS], 3);
      SERIAL_MV(" Z", delta_endstop_adj[C_AXIS], 3);
      SERIAL_MV(" height ", endstops.soft_endstop_max[C_AXIS], 3);
      SERIAL_MV(" diagonal rod ", delta_diagonal_rod, 3);
      SERIAL_MV(" delta radius ", delta_radius, 3);
      SERIAL_MV(" Towers radius correction A", delta_tower_radius_adj[A_AXIS], 2);
      SERIAL_MV(" B", delta_tower_radius_adj[B_AXIS], 2);
      SERIAL_MV(" C", delta_tower_radius_adj[C_AXIS], 2);
      SERIAL_EOL();

      endstops.enable(true);
      Home();
      endstops.not_homing();

      #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
        do_blocking_move_to_z(delta_clip_start_height);
      #endif
      clean_up_after_endstop_or_probe_move();
      #if HOTENDS > 1
        tool_change(old_tool_index, 0, true);
      #endif

      #if HAS_NEXTION_MANUAL_BED
        LcdBedLevelOff();
      #endif
    }

    // Compute the derivative of height with respect to a parameter at the specified motor endpoints.
    // 'deriv' indicates the parameter as follows:
    // 0, 1, 2 = X, Y, Z tower endstop adjustments
    // 3 = delta radius
    // 4 = X tower correction
    // 5 = Y tower correction
    // 6 = delta_diagonal_rod rod length
    // 7, 8 = X tilt, Y tilt. We scale these by the printable radius to get sensible values in the range -1..1
    float Delta_Mechanics::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) {
      const float perturb = 0.2;			// perturbation amount in mm or degrees
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
          hiParams.delta_tower_radius_adj[A_AXIS] += perturb;
          loParams.delta_tower_radius_adj[A_AXIS] -= perturb;
          break;

        case 5:
          hiParams.delta_tower_radius_adj[B_AXIS] += perturb;
          loParams.delta_tower_radius_adj[B_AXIS] -= perturb;
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

    // Perform 3, 4, 6, 7 - factor adjustment.
    // The input vector contains the following parameters in this order:
    //  X, Y and Z endstop adjustments
    //  Delta radius
    //  X tower position adjustment
    //  Y tower position adjustment
    //  Diagonal rod length adjustment
    void Delta_Mechanics::Adjust(const uint8_t numFactors, const float v[]) {

      const float oldHeightA = homed_Height + delta_endstop_adj[A_AXIS];

      // Update endstop adjustments
      delta_endstop_adj[A_AXIS] += v[0];
      delta_endstop_adj[B_AXIS] += v[1];
      delta_endstop_adj[C_AXIS] += v[2];
      NormaliseEndstopAdjustments();

      if (numFactors >= 4) {
        delta_radius += v[3];

        if (numFactors >= 6) {
          delta_tower_radius_adj[A_AXIS] += v[4];
          delta_tower_radius_adj[B_AXIS] += v[5];

          if (numFactors == 7) delta_diagonal_rod += v[6];

        }
      }

      recalc_delta_settings();
      const float heightError = homed_Height + delta_endstop_adj[A_AXIS] - oldHeightA - v[0];
      delta_height -= heightError;
      homed_Height -= heightError;

    }

    // Convert delta_endstop_adj
    void Delta_Mechanics::Convert_endstop_adj() {
      LOOP_XYZ(i) delta_endstop_adj[i] *= -1;
    }

    // Normalize Endstop
    void Delta_Mechanics::NormaliseEndstopAdjustments() {
      const float min_endstop = MIN3(delta_endstop_adj[A_AXIS], delta_endstop_adj[B_AXIS], delta_endstop_adj[C_AXIS]);
      LOOP_XYZ(i) delta_endstop_adj[i] -= min_endstop;
      delta_height += min_endstop;
      homed_Height += min_endstop;
    }

  #elif ENABLED(DELTA_AUTO_CALIBRATION_2)

    /**
     * Delta AutoCalibration Algorithm based on Thinkyhead Marlin
     *       Calibrate height, endstops, delta radius, and tower angles.
     *
     * Parameters:
     *
     *   Pn Number of probe points:
     *
     *      P1     Probe center and set height only.
     *      P2     Probe center and towers. Set height, endstops, and delta radius.
     *      P3     Probe all positions: center, towers and opposite towers. Set all.
     *      P4-P7  Probe all positions at different locations and average them.
     *
     *   T   Don't calibrate tower angle corrections
     *
     *   Cn.nn Calibration precision; when omitted calibrates to maximum precision
     *
     *   Vn Verbose level:
     *
     *      V0  Dry-run mode. Report settings and probe results. No calibration.
     *      V1  Report settings
     *      V2  Report settings and probe results
     *
     *   E   Engage the probe for each point
     */

    void print_signed_float(const char * const prefix, const float &f) {
      SERIAL_MSG("  ");
      SERIAL_PS(prefix);
      SERIAL_CHR(':');
      if (f >= 0) SERIAL_CHR('+');
      SERIAL_VAL(f, 2);
    }

    void Delta_Mechanics::auto_calibration() {

      const int8_t probe_points = parser.intval('P', 4);
      if (!WITHIN(probe_points, 1, 7)) {
        SERIAL_EM("?(P)oints is implausible (1-7).");
        return;
      }

      const int8_t verbose_level = parser.byteval('V', 1);
      if (!WITHIN(verbose_level, 0, 2)) {
        SERIAL_EM("?(V)erbose Level is implausible (0-2).");
        return;
      }

      const float calibration_precision = parser.floatval('C');
      if (calibration_precision < 0) {
        SERIAL_EM("?(C)alibration precision is implausible (>0).");
        return;
      }

      const bool  towers_set            = !parser.noboolval('T'),
                  stow_after_each       = parser.boolval('E'),
                  _1p_calibration       = probe_points == 1,
                  _4p_calibration       = probe_points == 2,
                  _4p_towers_points     = _4p_calibration && towers_set,
                  _4p_opposite_points   = _4p_calibration && !towers_set,
                  _7p_calibration       = probe_points >= 3,
                  _7p_half_circle       = probe_points == 3,
                  _7p_double_circle     = probe_points == 5,
                  _7p_triple_circle     = probe_points == 6,
                  _7p_quadruple_circle  = probe_points == 7,
                  _7p_multi_circle      = _7p_double_circle || _7p_triple_circle || _7p_quadruple_circle,
                  _7p_intermed_points   = _7p_calibration && !_7p_half_circle;

      const static char save_message[] PROGMEM = "Save with M500 and/or copy to configuration_delta.h";
      int8_t iterations = 0;
      float test_precision,
            zero_std_dev = (verbose_level ? 999.0 : 0.0), // 0.0 in dry-run mode : forced end
            zero_std_dev_old = zero_std_dev,
            zero_std_dev_min = zero_std_dev,
            e_old[XYZ] = {
              delta_endstop_adj[A_AXIS],
              delta_endstop_adj[B_AXIS],
              delta_endstop_adj[C_AXIS]
            },
            dr_old = delta_radius,
            zh_old = delta_height,
            alpha_old = delta_tower_radius_adj[A_AXIS],
            beta_old = delta_tower_radius_adj[B_AXIS];

      SERIAL_EM("G33 Auto Calibrate");

      stepper.synchronize();
      #if HAS_LEVELING
        bedlevel.reset_bed_level(); // After calibration bed-level data is no longer valid
      #endif
      #if HOTENDS > 1
        const uint8_t old_tool_index = active_extruder;
        tool_change(0, 0, true);
      #endif
      setup_for_endstop_or_probe_move();
      endstops.enable(true);
      Home();
      endstops.not_homing();
      probe.set_deployed(true);

      // print settings

      SERIAL_MSG("Checking... AC");
      if (verbose_level == 0) SERIAL_MSG(" (DRY-RUN)");
      SERIAL_EOL();
      LCD_MESSAGEPGM(MSG_DELTA_CHECKING);

      SERIAL_MV(".Height:", delta_height, 2);
      if (!_1p_calibration) {
        print_signed_float(PSTR("  Ex"), delta_endstop_adj[A_AXIS]);
        print_signed_float(PSTR("Ey"), delta_endstop_adj[B_AXIS]);
        print_signed_float(PSTR("Ez"), delta_endstop_adj[C_AXIS]);
        SERIAL_MV("    Radius:", delta_radius, 2);
      }
      SERIAL_EOL();
      if (_7p_calibration && towers_set) {
        SERIAL_MSG(".Tower angle:   ");
        print_signed_float(PSTR("Tx"), delta_tower_radius_adj[A_AXIS]);
        print_signed_float(PSTR("Ty"), delta_tower_radius_adj[B_AXIS]);
        print_signed_float(PSTR("Tz"), delta_tower_radius_adj[C_AXIS]);
        SERIAL_EOL();
      }

      do {

        float z_at_pt[13] = { 0 };

        test_precision = zero_std_dev_old != 999.0 ? (zero_std_dev + zero_std_dev_old) / 2 : zero_std_dev;

        iterations++;

        if (!_7p_half_circle && !_7p_triple_circle) { // probe the center
          z_at_pt[0] += probe.check_pt(0.0, 0.0, stow_after_each, 1);
        }
        if (_7p_calibration) { // probe extra center points
          for (int8_t axis = _7p_multi_circle ? 11 : 9; axis > 0; axis -= _7p_multi_circle ? 2 : 4) {
            const float a = RADIANS(180 + 30 * axis), r = delta_probe_radius * 0.1;
            z_at_pt[0] += probe.check_pt(cos(a) * r, sin(a) * r, stow_after_each, 1);
          }
          z_at_pt[0] /= float(_7p_double_circle ? 7 : probe_points);
        }
        if (!_1p_calibration) {  // probe the radius
          bool zig_zag = true;
          const uint8_t start = _4p_opposite_points ? 3 : 1,
                         step = _4p_calibration ? 4 : _7p_half_circle ? 2 : 1;
          for (uint8_t axis = start; axis < 13; axis += step) {
            const float zigadd = (zig_zag ? 0.5 : 0.0),
                        offset_circles =  _7p_quadruple_circle ? zigadd + 1.0 :
                                          _7p_triple_circle    ? zigadd + 0.5 :
                                          _7p_double_circle    ? zigadd : 0;
            for (float circles = -offset_circles ; circles <= offset_circles; circles++) {
              const float a = RADIANS(180 + 30 * axis),
                          r = delta_probe_radius * (1 + circles * (zig_zag ? 0.1 : -0.1));
              z_at_pt[axis] += probe.check_pt(cos(a) * r, sin(a) * r, stow_after_each, 1);
            }
            zig_zag = !zig_zag;
            z_at_pt[axis] /= (2 * offset_circles + 1);
          }
        }
        if (_7p_intermed_points) { // average intermediates to tower and opposites
          for (uint8_t axis = 1; axis < 13; axis += 2)
            z_at_pt[axis] = (z_at_pt[axis] + (z_at_pt[axis + 1] + z_at_pt[(axis + 10) % 12 + 1]) / 2.0) / 2.0;
        }

        float S1  = z_at_pt[0],
              S2  = sq(z_at_pt[0]);
        int16_t N = 1;
        if (!_1p_calibration) { // std dev from zero plane
          for (uint8_t axis = (_4p_opposite_points ? 3 : 1); axis < 13; axis += (_4p_calibration ? 4 : 2)) {
            S1 += z_at_pt[axis];
            S2 += sq(z_at_pt[axis]);
            N++;
          }
        }
        zero_std_dev_old = zero_std_dev;
        NOMORE(zero_std_dev_min, zero_std_dev);
        zero_std_dev = round(SQRT(S2 / N) * 1000.0) / 1000.0 + 0.00001;

        // Solve matrices

        if (zero_std_dev < test_precision && zero_std_dev > calibration_precision) {
          if (zero_std_dev < zero_std_dev_min) {
            COPY_ARRAY(e_old, delta_endstop_adj);
            dr_old = delta_radius;
            zh_old = delta_height;
            alpha_old = delta_tower_radius_adj[A_AXIS];
            beta_old = delta_tower_radius_adj[B_AXIS];
          }

          float e_delta[XYZ] = { 0.0 }, r_delta = 0.0, t_alpha = 0.0, t_beta = 0.0;
          const float r_diff = delta_radius - delta_probe_radius,
                      h_factor = 1.00 + r_diff * 0.001,                          // 1.02 for r_diff = 20mm
                      r_factor = -(1.75 + 0.005 * r_diff + 0.001 * sq(r_diff)),  // 2.25 for r_diff = 20mm
                      a_factor = 100.0 / delta_probe_radius;               // 1.25 for cal_rd = 80mm

          #define ZP(N,I) ((N) * z_at_pt[I])
          #define Z1000(I) ZP(1.00, I)
          #define Z1050(I) ZP(h_factor, I)
          #define Z0700(I) ZP(h_factor * 2.0 / 3.00, I)
          #define Z0350(I) ZP(h_factor / 3.00, I)
          #define Z0175(I) ZP(h_factor / 6.00, I)
          #define Z2250(I) ZP(r_factor, I)
          #define Z0750(I) ZP(r_factor / 3.00, I)
          #define Z0375(I) ZP(r_factor / 6.00, I)
          #define Z0444(I) ZP(a_factor * 4.0 / 9.0, I)
          #define Z0888(I) ZP(a_factor * 8.0 / 9.0, I)

          switch (probe_points) {
            case 1:
              test_precision = 0.00;
              LOOP_XYZ(i) e_delta[i] = Z1000(0);
              break;

            case 2:
              if (towers_set) {
                e_delta[X_AXIS] = Z1050(0) + Z0700(1) - Z0350(5) - Z0350(9);
                e_delta[Y_AXIS] = Z1050(0) - Z0350(1) + Z0700(5) - Z0350(9);
                e_delta[Z_AXIS] = Z1050(0) - Z0350(1) - Z0350(5) + Z0700(9);
                r_delta         = Z2250(0) - Z0750(1) - Z0750(5) - Z0750(9);
              }
              else {
                e_delta[X_AXIS] = Z1050(0) - Z0700(7) + Z0350(11) + Z0350(3);
                e_delta[Y_AXIS] = Z1050(0) + Z0350(7) - Z0700(11) + Z0350(3);
                e_delta[Z_AXIS] = Z1050(0) + Z0350(7) + Z0350(11) - Z0700(3);
                r_delta         = Z2250(0) - Z0750(7) - Z0750(11) - Z0750(3);
              }
              break;

            default:
              e_delta[X_AXIS] = Z1050(0) + Z0350(1) - Z0175(5) - Z0175(9) - Z0350(7) + Z0175(11) + Z0175(3);
              e_delta[Y_AXIS] = Z1050(0) - Z0175(1) + Z0350(5) - Z0175(9) + Z0175(7) - Z0350(11) + Z0175(3);
              e_delta[Z_AXIS] = Z1050(0) - Z0175(1) - Z0175(5) + Z0350(9) + Z0175(7) + Z0175(11) - Z0350(3);
              r_delta         = Z2250(0) - Z0375(1) - Z0375(5) - Z0375(9) - Z0375(7) - Z0375(11) - Z0375(3);
              
              if (towers_set) {
                t_alpha = Z0444(1) - Z0888(5) + Z0444(9) + Z0444(7) - Z0888(11) + Z0444(3);
                t_beta  = Z0888(1) - Z0444(5) - Z0444(9) + Z0888(7) - Z0444(11) - Z0444(3);
              }
              break;
          }

          LOOP_XYZ(axis) delta_endstop_adj[axis] += e_delta[axis];
          delta_radius += r_delta;
          delta_tower_radius_adj[A_AXIS] += t_alpha;
          delta_tower_radius_adj[B_AXIS] += t_beta;

          // adjust delta_height and endstops by the max amount
          const float z_temp = MAX3(delta_endstop_adj[A_AXIS], delta_endstop_adj[B_AXIS], delta_endstop_adj[C_AXIS]);
          delta_height -= z_temp;
          LOOP_XYZ(i) delta_endstop_adj[i] -= z_temp;

          recalc_delta_settings();
        }
        else if(zero_std_dev >= test_precision) {   // step one back
          COPY_ARRAY(delta_endstop_adj, e_old);
          delta_radius = dr_old;
          delta_height = zh_old;
          delta_tower_radius_adj[A_AXIS] = alpha_old;
          delta_tower_radius_adj[B_AXIS] = beta_old;

          recalc_delta_settings();
        }

        // print report
        if (verbose_level != 1) {
          SERIAL_MSG(".    ");
          print_signed_float(PSTR("c"), z_at_pt[0]);
          if (_4p_towers_points || _7p_calibration) {
            print_signed_float(PSTR("   x"), z_at_pt[1]);
            print_signed_float(PSTR(" y"), z_at_pt[5]);
            print_signed_float(PSTR(" z"), z_at_pt[9]);
          }
          if (!_4p_opposite_points) SERIAL_EOL();
          if ((_4p_opposite_points) || _7p_calibration) {
            if (_7p_calibration) {
              SERIAL_CHR('.');
              SERIAL_SP(13);
            }
            print_signed_float(PSTR("  yz"), z_at_pt[7]);
            print_signed_float(PSTR("zx"), z_at_pt[11]);
            print_signed_float(PSTR("xy"), z_at_pt[3]);
            SERIAL_EOL();
          }
        }
        if (test_precision != 0.0) {
          if (zero_std_dev >= test_precision || zero_std_dev <= calibration_precision) {  // end iterations
            SERIAL_MSG("Calibration OK");
            SERIAL_SP(36);
            if (zero_std_dev >= test_precision)
              SERIAL_MSG("rolling back.");
            else
              SERIAL_MV("std dev:", zero_std_dev, 3);
            SERIAL_EOL();
            LCD_MESSAGEPGM(MSG_DELTA_AUTO_CALIBRATE_OK);
          }
          else {
            char mess[15] = "No convergence";
            if (iterations < 31)
              sprintf_P(mess, PSTR("Iteration:%02i"), (int)iterations);
            SERIAL_TXT(mess);
            SERIAL_SP(38);
            SERIAL_EMV("std dev:", zero_std_dev, 3);
            lcd_setstatus(mess);
          }
          SERIAL_MV(".Height:", delta_height, 2);
          if (!_1p_calibration) {
            print_signed_float(PSTR("  Ex"), delta_endstop_adj[A_AXIS]);
            print_signed_float(PSTR("Ey"), delta_endstop_adj[B_AXIS]);
            print_signed_float(PSTR("Ez"), delta_endstop_adj[C_AXIS]);
            SERIAL_MV("    Radius:", delta_radius, 2);
          }
          SERIAL_EOL();
          if (_7p_calibration && towers_set) {
            SERIAL_MSG(".Tower angle :  ");
            print_signed_float(PSTR("Tx"), delta_tower_radius_adj[A_AXIS]);
            print_signed_float(PSTR("Ty"), delta_tower_radius_adj[B_AXIS]);
            print_signed_float(PSTR("Tz"), delta_tower_radius_adj[C_AXIS]);
            SERIAL_EOL();
          }
          if (zero_std_dev >= test_precision || zero_std_dev <= calibration_precision) {
            SERIAL_PS(save_message);
            SERIAL_EOL();
          }
        }
        else {
          if (verbose_level == 0) {
            SERIAL_MSG("End DRY-RUN");
            SERIAL_SP(39);
            SERIAL_EMV("std dev:", zero_std_dev, 3);
          }
          else {
            SERIAL_MSG("Calibration OK");
            LCD_MESSAGEPGM(MSG_DELTA_AUTO_CALIBRATE_OK);
            SERIAL_PS(save_message);
            SERIAL_EOL();
          }
        }

        stepper.synchronize();

        // Homing
        endstops.enable(true);
        Home();
        endstops.not_homing();

      } while (zero_std_dev < test_precision && zero_std_dev > calibration_precision && iterations < 31);

      #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
        do_blocking_move_to_z(delta_clip_start_height);
      #endif
      probe.set_deployed(false);
      clean_up_after_endstop_or_probe_move();
      #if HOTENDS > 1
        tool_change(old_tool_index, 0, true);
      #endif
    }

  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)

    void Delta_Mechanics::print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z) {
      SERIAL_PS(prefix);
      SERIAL_CHR('(');
      SERIAL_VAL(x);
      SERIAL_MV(", ", y);
      SERIAL_MV(", ", z);
      SERIAL_CHR(")");

      if (suffix) SERIAL_PS(suffix);
      else SERIAL_EOL();
    }

    void Delta_Mechanics::print_xyz(const char* prefix, const char* suffix, const float xyz[]) {
      print_xyz(prefix, suffix, xyz[A_AXIS], xyz[B_AXIS], xyz[C_AXIS]);
    }

  #endif

#endif

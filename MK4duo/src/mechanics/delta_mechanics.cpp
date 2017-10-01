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

#include "../../base.h"
#include "delta_mechanics.h"

#if IS_DELTA

  Delta_Mechanics mechanics;

  void Delta_Mechanics::Init() {
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

  void Delta_Mechanics::set_position_mm(ARG_X, ARG_Y, ARG_Z, const float &e) {
    _set_position_mm(lx, ly, lz, e);
  }

  bool Delta_Mechanics::position_is_reachable_raw_xy(const float &rx, const float &ry) {
    return HYPOT2(rx, ry) <= sq(delta_print_radius);
  }

  bool Delta_Mechanics::position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry) {
    // Both the nozzle and the probe must be able to reach the point.
    return position_is_reachable_raw_xy(rx, ry)
        && position_is_reachable_raw_xy(rx - probe.offset[X_AXIS], ry - probe.offset[Y_AXIS]);
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
      get_axis_position_mm(A_AXIS),
      get_axis_position_mm(B_AXIS),
      get_axis_position_mm(C_AXIS),
      cartesian_position
    );
    cartesian_position[X_AXIS] += LOGICAL_X_POSITION(0);
    cartesian_position[Y_AXIS] += LOGICAL_Y_POSITION(0);
    cartesian_position[Z_AXIS] += LOGICAL_Z_POSITION(0);
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

      // If the move is only in Z/E don't split up the move
      if (destination[X_AXIS] == current_position[X_AXIS] && destination[Y_AXIS] == current_position[B_AXIS]) {
        planner.buffer_line_kinematic(destination, _feedrate_mm_s, tools.active_extruder);
        set_current_to_destination();
        return false;
      }

      // Fail if attempting move outside printable radius
      if (!position_is_reachable_xy(destination[X_AXIS], destination[Y_AXIS])) return true;

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
      if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = abs(difference[E_AXIS]);

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
                  segment_distance[XYZE] = {
                    difference[X_AXIS] * inv_segments,
                    difference[Y_AXIS] * inv_segments,
                    difference[Z_AXIS] * inv_segments,
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

        planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], _feedrate_mm_s, tools.active_extruder);

      }

      planner.buffer_line_kinematic(destination, _feedrate_mm_s, tools.active_extruder);

      set_current_to_destination();
      return false;
    }

  #endif // DISABLED(AUTO_BED_LEVELING_UBL)

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

  void Delta_Mechanics::manual_goto_xy(const float &x, const float &y) {

    current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS) + Z_PROBE_DEPLOY_HEIGHT;
    planner.buffer_line_kinematic(current_position, homing_feedrate_mm_s[Z_AXIS], tools.active_extruder);

    current_position[X_AXIS] = LOGICAL_X_POSITION(x);
    current_position[Y_AXIS] = LOGICAL_Y_POSITION(y);
    planner.buffer_line_kinematic(current_position, MMM_TO_MMS(XY_PROBE_SPEED), tools.active_extruder);

    current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS) + Z_PROBE_BETWEEN_HEIGHT; // just slightly over the bed
    planner.buffer_line_kinematic(current_position, MMM_TO_MMS(Z_PROBE_SPEED_SLOW), tools.active_extruder);

    #if ENABLED(PROBE_MANUALLY) && ENABLED(LCD_BED_LEVELING) && ENABLED(ULTRA_LCD)
      lcd_wait_for_move = false;
    #endif

  }

  /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void Delta_Mechanics::prepare_uninterpolated_move_to_destination(const float fr_mm_s/*=0.0*/) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);
    #endif

    commands.refresh_cmd_timeout();

    if ( current_position[A_AXIS] == destination[A_AXIS]
      && current_position[B_AXIS] == destination[B_AXIS]
      && current_position[C_AXIS] == destination[C_AXIS]
      && current_position[E_AXIS] == destination[E_AXIS]
    ) return;

    planner.buffer_line_kinematic(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), tools.active_extruder);

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

    LOOP_XYZ(axis) {
      endstops.soft_endstop_min[axis] = (axis == C_AXIS ? 0 : -delta_print_radius);
      endstops.soft_endstop_max[axis] = (axis == C_AXIS ? delta_height : delta_print_radius);
    }

    delta_diagonal_rod_2[A_AXIS] = sq(delta_diagonal_rod + delta_diagonal_rod_adj[A_AXIS]);
    delta_diagonal_rod_2[B_AXIS] = sq(delta_diagonal_rod + delta_diagonal_rod_adj[B_AXIS]);
    delta_diagonal_rod_2[C_AXIS] = sq(delta_diagonal_rod + delta_diagonal_rod_adj[C_AXIS]);

    // Effective X/Y positions of the three vertical towers.
    towerX[A_AXIS] = COS(RADIANS(210.f + delta_tower_angle_adj[A_AXIS])) * (delta_radius + delta_tower_radius_adj[A_AXIS]); // front left tower
    towerY[A_AXIS] = SIN(RADIANS(210.f + delta_tower_angle_adj[A_AXIS])) * (delta_radius + delta_tower_radius_adj[A_AXIS]);
    towerX[B_AXIS] = COS(RADIANS(330.f + delta_tower_angle_adj[B_AXIS])) * (delta_radius + delta_tower_radius_adj[B_AXIS]); // front right tower
    towerY[B_AXIS] = SIN(RADIANS(330.f + delta_tower_angle_adj[B_AXIS])) * (delta_radius + delta_tower_radius_adj[B_AXIS]);
    towerX[C_AXIS] = COS(RADIANS( 90.f + delta_tower_angle_adj[C_AXIS])) * (delta_radius + delta_tower_radius_adj[C_AXIS]); // back middle tower
    towerY[C_AXIS] = SIN(RADIANS( 90.f + delta_tower_angle_adj[C_AXIS])) * (delta_radius + delta_tower_radius_adj[C_AXIS]);

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

    NOMORE(delta_probe_radius, delta_print_radius);

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
   * Delta Transform
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

    planner._buffer_line(delta_A, delta_B, delta_C, le, fr, tools.active_extruder);
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
  bool Delta_Mechanics::Home(const bool always_home_all) {

    UNUSED(always_home_all);

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
      bedlevel.g29_in_progress = false;
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
      const uint8_t old_tool_index = tools.active_extruder;
      tools.change(0, 0, true);
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

    // If an endstop was not hit, then damage can occur if homing is continued.
    // This can occur if the delta height is
    // not set correctly.
    if (!(TEST(endstops.endstop_hit_bits, X_MAX) ||
          TEST(endstops.endstop_hit_bits, Y_MAX) ||
          TEST(endstops.endstop_hit_bits, Z_MAX))) {
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
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< home_delta", current_position);
    #endif

    #if ENABLED(NPR2)
      if ((home_all) || (parser.seen('E'))) {
        set_destination_to_current();
        destination[E_AXIS] = -200;
        tools.active_driver = tools.active_extruder = 1;
        planner.buffer_line_kinematic(destination, COLOR_HOMERATE, tools.active_extruder);
        stepper.synchronize();
        printer.old_color = 99;
        tools.active_driver = tools.active_extruder = 0;
        current_position[E_AXIS] = 0;
        sync_plan_position_e();
      }
    #endif

    endstops.not_homing();

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
      gfx_clear(delta_print_radius * 2, delta_print_radius * 2, delta_height);
      gfx_cursor_to(current_position[X_AXIS] + delta_print_radius, current_position[Y_AXIS] + delta_print_radius, current_position[Z_AXIS]);
    #endif

    printer.clean_up_after_endstop_or_probe_move();

    stepper.synchronize();

    // Restore the active tool after homing
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif

    lcd_refresh();

    //report_current_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G28");
    #endif

    return true;
  }

  void Delta_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    axis_known_position[axis] = axis_homed[axis] = true;

    current_position[axis] = (axis == C_AXIS ? delta_height : 0.0);

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS("", current_position);
        SERIAL_MV("<<< set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  void Delta_Mechanics::Calibration_cleanup(
    #if HOTENDS > 1
      const uint8_t old_tool_index
    #endif
  ) {
    #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
      do_blocking_move_to_z(delta_clip_start_height);
    #endif
    STOW_PROBE();
    printer.clean_up_after_endstop_or_probe_move();
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif
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
     */
    void Delta_Mechanics::auto_calibration() {

      const uint8_t   MaxCalibrationPoints = 10;

      uint8_t iteration = 0;

      float   xBedProbePoints[MaxCalibrationPoints],
              yBedProbePoints[MaxCalibrationPoints],
              zBedProbePoints[MaxCalibrationPoints],
              initialSumOfSquares,
              expectedRmsError;

      char    rply[50];

      const uint8_t numFactors = parser.intval('F', 7);
      if (!WITHIN(numFactors, 3, 7)) {
        SERIAL_EM("?(F)actors is implausible (3 to 7).");
        return;
      }

      const uint8_t probe_points  = parser.intval('P', 7);
      if (!WITHIN(probe_points, 1, 7)) {
        SERIAL_EM("?(P)oints is implausible (1 to 7).");
        return;
      }

      SERIAL_MV("Starting Auto Calibration ", probe_points);
      SERIAL_MV(" points and ", numFactors);
      SERIAL_EM(" Factors");
      LCD_MESSAGEPGM(MSG_DELTA_AUTO_CALIBRATE);

      stepper.synchronize();

      #if HAS_LEVELING
        bedlevel.reset(); // After calibration bed-level data is no longer valid
      #endif

      #if HOTENDS > 1
        const uint8_t old_tool_index = tools.active_extruder;
        tools.change(0, 0, true);
        #define CALIBRATION_CLEANUP() Calibration_cleanup(old_tool_index)
      #else
        #define CALIBRATION_CLEANUP() Calibration_cleanup()
      #endif

      printer.setup_for_endstop_or_probe_move();
      endstops.enable(true);
      if (!Home()) return;
      endstops.not_homing();
      DEPLOY_PROBE();

      const float dx = (probe.offset[X_AXIS]),
                  dy = (probe.offset[Y_AXIS]);

      for (uint8_t probe_index = 0; probe_index < 6; probe_index++) {
        xBedProbePoints[probe_index] = delta_probe_radius * SIN((2 * M_PI * probe_index) / 6);
        yBedProbePoints[probe_index] = delta_probe_radius * COS((2 * M_PI * probe_index) / 6);
        zBedProbePoints[probe_index] = probe.check_pt(xBedProbePoints[probe_index] + probe.offset[X_AXIS], yBedProbePoints[probe_index] + probe.offset[Y_AXIS], false, 4);
        if (isnan(zBedProbePoints[probe_index])) return CALIBRATION_CLEANUP();
      }
      if (probe_points >= 10) {
        for (uint8_t probe_index = 6; probe_index < 9; probe_index++) {
          xBedProbePoints[probe_index] = (delta_probe_radius / 2) * SIN((2 * M_PI * (probe_index - 6)) / 3);
          yBedProbePoints[probe_index] = (delta_probe_radius / 2) * COS((2 * M_PI * (probe_index - 6)) / 3);
          zBedProbePoints[probe_index] = probe.check_pt(xBedProbePoints[probe_index] + probe.offset[X_AXIS], yBedProbePoints[probe_index] + probe.offset[Y_AXIS], false, 4);
          if (isnan(zBedProbePoints[probe_index])) return CALIBRATION_CLEANUP();
        }
        xBedProbePoints[9] = 0.0;
        yBedProbePoints[9] = 0.0;
        zBedProbePoints[9] = probe.check_pt(probe.offset[X_AXIS], probe.offset[Y_AXIS], true, 4, false);
        if (isnan(zBedProbePoints[9])) return CALIBRATION_CLEANUP();
      }
      else {
        xBedProbePoints[6] = 0.0;
        yBedProbePoints[6] = 0.0;
        zBedProbePoints[6] = probe.check_pt(probe.offset[X_AXIS], probe.offset[Y_AXIS], true, 4, false);
        if (isnan(zBedProbePoints[6])) return CALIBRATION_CLEANUP();
      }

      // convert delta_endstop_adj;
      Convert_endstop_adj();

      float probeMotorPositions[MaxCalibrationPoints][ABC],
            corrections[MaxCalibrationPoints];

      initialSumOfSquares = 0.0;

      // Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
      for (uint8_t i = 0; i < probe_points; ++i) {
        corrections[i] = 0.0;
        float machinePos[ABC];

        machinePos[A_AXIS] = xBedProbePoints[i];
        machinePos[B_AXIS] = yBedProbePoints[i];
        machinePos[C_AXIS] = 0.0;

        Transform(machinePos);

        for (uint8_t axis = 0; axis < ABC; axis++)
          probeMotorPositions[i][axis] = delta[axis];

        initialSumOfSquares += sq(zBedProbePoints[i]);
      }

      // Do 1 or more Newton-Raphson iterations

      do {
        iteration++;

        float derivativeMatrix[MaxCalibrationPoints][numFactors],
              normalMatrix[numFactors][numFactors + 1];

        for (uint8_t i = 0; i < probe_points; i++) {
          for (uint8_t j = 0; j < numFactors; j++) {
            derivativeMatrix[i][j] =
              ComputeDerivative(j, probeMotorPositions[i][A_AXIS], probeMotorPositions[i][B_AXIS], probeMotorPositions[i][C_AXIS]);
          }
        }

        for (uint8_t i = 0; i < numFactors; i++) {
          for (uint8_t j = 0; j < numFactors; j++) {
            float temp = derivativeMatrix[0][i] * derivativeMatrix[0][j];
            for (uint8_t k = 1; k < probe_points; k++) {
              temp += derivativeMatrix[k][i] * derivativeMatrix[k][j];
            }
            normalMatrix[i][j] = temp;
          }
          float temp = derivativeMatrix[0][i] * -(zBedProbePoints[0] + corrections[0]);
          for (uint8_t k = 1; k < probe_points; k++) {
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

        for (int8_t i = 0; i < probe_points; i++) {
          LOOP_XYZ(axis) probeMotorPositions[i][axis] += solution[axis];
          float newPosition[ABC];
          InverseTransform(probeMotorPositions[i][A_AXIS], probeMotorPositions[i][B_AXIS], probeMotorPositions[i][C_AXIS], newPosition);
          corrections[i] = newPosition[Z_AXIS];
          expectedResiduals[i] = zBedProbePoints[i] + newPosition[Z_AXIS];
          sumOfSquares += sq(expectedResiduals[i]);
        }

        expectedRmsError = SQRT(sumOfSquares / probe_points);

      } while (iteration < 2);

      // convert delta_endstop_adj;
      Convert_endstop_adj();

      SERIAL_MV("Calibrated ", numFactors);
      SERIAL_MV(" factors using ", probe_points);
      SERIAL_MV(" points, deviation before ", SQRT(initialSumOfSquares / probe_points), 4);
      SERIAL_MV(" after ", expectedRmsError, 4);
      SERIAL_EOL();

      recalc_delta_settings();

      SERIAL_MV("Endstops X", delta_endstop_adj[A_AXIS], 3);
      SERIAL_MV(" Y", delta_endstop_adj[B_AXIS], 3);
      SERIAL_MV(" Z", delta_endstop_adj[C_AXIS], 3);
      SERIAL_MV(" height ", delta_height, 3);
      SERIAL_MV(" diagonal rod ", delta_diagonal_rod, 3);
      SERIAL_MV(" delta radius ", delta_radius, 3);
      SERIAL_MV(" Towers angle correction I", delta_tower_angle_adj[A_AXIS], 2);
      SERIAL_MV(" J", delta_tower_angle_adj[B_AXIS], 2);
      SERIAL_MV(" K", delta_tower_angle_adj[C_AXIS], 2);
      SERIAL_EOL();

      endstops.enable(true);
      if (!Home()) return;
      endstops.not_homing();

      CALIBRATION_CLEANUP();

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
     *      P0     No probe. Normalize only.
     *      P1     Probe center and set height only.
     *      P2     Probe center and towers. Set height, endstops, and delta radius.
     *      P3     Probe all positions: center, towers and opposite towers. Set all.
     *      P4-P7  Probe all positions at different locations and average them.
     *
     *   T0  Don't calibrate tower angle corrections
     *
     *   Cn.nn Calibration precision; when omitted calibrates to maximum precision
     *
     *   Fn  Force to run at least n iterations and takes the best result
     *
     *   Vn Verbose level:
     *
     *      V0  Dry-run mode. Report settings and probe results. No calibration.
     *      V1  Report settings
     *      V2  Report settings and probe results
     *
     *   E   Engage the probe for each point
     */
    void Delta_Mechanics::auto_calibration() {

      const int8_t probe_points = parser.intval('P', 4);
      if (!WITHIN(probe_points, 0, 7)) {
        SERIAL_EM("?(P)oints is implausible (0-7).");
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

      const int8_t force_iterations = parser.intval('F', 0);
      if (!WITHIN(force_iterations, 0, 30)) {
        SERIAL_EM("?(F)orce iteration is implausible (0-30).");
        return;
      }

      const bool  towers_set            = parser.boolval('T', true),
                  stow_after_each       = parser.boolval('E'),
                  _0p_calibration       = probe_points == 0,
                  _1p_calibration       = probe_points == 1,
                  _4p_calibration       = probe_points == 2,
                  _4p_towers_points     = _4p_calibration && towers_set,
                  _4p_opposite_points   = _4p_calibration && !towers_set,
                  _7p_calibration       = probe_points >= 3 || _0p_calibration,
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
            e_old[ABC] = {
              delta_endstop_adj[A_AXIS],
              delta_endstop_adj[B_AXIS],
              delta_endstop_adj[C_AXIS]
            },
            dr_old = delta_radius,
            dh_old = delta_height,
            ta_old[ABC] = {
              delta_tower_angle_adj[A_AXIS],
              delta_tower_angle_adj[B_AXIS],
              delta_tower_angle_adj[C_AXIS]
            };

      if (!_1p_calibration && !_0p_calibration) {  // test if the outer radius is reachable
        const float circles = (_7p_quadruple_circle ? 1.5 :
                               _7p_triple_circle    ? 1.0 :
                               _7p_double_circle    ? 0.5 : 0),
                    r = (1 + circles * 0.1) * delta_probe_radius;
        for (uint8_t axis = 1; axis < 13; ++axis) {
          const float a = RADIANS(180 + 30 * axis);
          if (!position_is_reachable_by_probe_xy(COS(a) * r + probe.offset[X_AXIS], SIN(a) * r + probe.offset[Y_AXIS])) {
            SERIAL_EM("?(M666 P)robe radius is implausible.");
            return;
          }
        }
      }

      SERIAL_EM("G33 Auto Calibrate");

      stepper.synchronize();

      #if HAS_LEVELING
        bedlevel.reset(); // After calibration bed-level data is no longer valid
      #endif

      #if HOTENDS > 1
        const uint8_t old_tool_index = tools.active_extruder;
        tools.change(0, 0, true);
        #define CALIBRATION_CLEANUP() Calibration_cleanup(old_tool_index)
      #else
        #define CALIBRATION_CLEANUP() Calibration_cleanup()
      #endif

      printer.setup_for_endstop_or_probe_move();
      endstops.enable(true);
      if (!_0p_calibration) {
        if (!Home()) return;
        endstops.not_homing();
        DEPLOY_PROBE();
      }

      // print settings

      SERIAL_MSG(MSG_DELTA_CHECKING);
      if (verbose_level == 0) SERIAL_MSG(" (DRY-RUN)");
      SERIAL_EOL();
      LCD_MESSAGEPGM(MSG_DELTA_CHECKING);

      print_G33_settings(!_1p_calibration, _7p_calibration && towers_set);

      #if DISABLED(PROBE_MANUALLY)
        if (!_0p_calibration) {
          const float measured_z = probe.check_pt(probe.offset[X_AXIS], probe.offset[Y_AXIS], stow_after_each, 1, false); // 1st probe to set height
          if (isnan(measured_z)) return CALIBRATION_CLEANUP();
          delta_height -= measured_z;
        }
      #endif

      do {

        float z_at_pt[13] = { 0.0 };

        test_precision = _0p_calibration ? 0.00 : zero_std_dev_old != 999.0 ? (zero_std_dev + zero_std_dev_old) / 2 : zero_std_dev;

        iterations++;

        // Probe the points

        if (!_0p_calibration) {
          if (!_7p_half_circle && !_7p_triple_circle) {   // probe center (P1=1, 2=1, 3=0, 4=1, 5=1, 6=0, 7=1)
            z_at_pt[0] += probe.check_pt(probe.offset[X_AXIS], probe.offset[Y_AXIS], stow_after_each, 1, false);
            if (isnan(z_at_pt[0])) return CALIBRATION_CLEANUP();
          }
          if (_7p_calibration) {                          // probe extra center points (P1=0, 2=0, 3=3, 4=3, 5=6, 6=6, 7=6)
            for (int8_t axis = _7p_multi_circle ? 11 : 9; axis > 0; axis -= _7p_multi_circle ? 2 : 4) {
              const float a = RADIANS(180 + 30 * axis), r = delta_probe_radius * 0.1;
              z_at_pt[0] += probe.check_pt(COS(a) * r + probe.offset[X_AXIS], SIN(a) * r + probe.offset[Y_AXIS], stow_after_each, 1);
              if (isnan(z_at_pt[0])) return CALIBRATION_CLEANUP();
            }
            z_at_pt[0] /= float(_7p_double_circle ? 7 : probe_points);
          }
          if (!_1p_calibration) {                         // probe the radius points (P1=0, 2=3, 3=6, 4=12, 5=18, 6=30, 7=42)
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
                z_at_pt[axis] += probe.check_pt(COS(a) * r + probe.offset[X_AXIS], SIN(a) * r + probe.offset[Y_AXIS], stow_after_each, 1);
                if (isnan(z_at_pt[axis])) return CALIBRATION_CLEANUP();
              }
              zig_zag = !zig_zag;
              z_at_pt[axis] /= (2 * offset_circles + 1);
            }
          }
          if (_7p_intermed_points) // average intermediates to tower and opposites
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
        zero_std_dev = round(SQRT(S2 / N) * 1000.0) / 1000.0 + 0.00001;

        // Solve matrices

        if ((zero_std_dev < test_precision && zero_std_dev > calibration_precision) || iterations <= force_iterations) {
          if (zero_std_dev < zero_std_dev_min) {
            COPY_ARRAY(e_old, delta_endstop_adj);
            dr_old = delta_radius;
            dh_old = delta_height;
            COPY_ARRAY(ta_old, delta_tower_angle_adj);
          }

          float e_delta[ABC] = { 0.0 }, r_delta = 0.0, t_delta[ABC] = { 0.0 };
          const float r_diff = delta_radius - delta_probe_radius,
                      h_factor = (1.00 + r_diff * 0.001) / 6.0,                       // 1.02 / 6 for r_diff = 20mm
                      r_factor = -(1.75 + 0.005 * r_diff + 0.001 * sq(r_diff)) / 6.0, // 2.25 / 6 for r_diff = 20mm
                      a_factor = 66.66 / delta_probe_radius;                          // 0.83     for cal_rd = 80mm

          #define ZP(N,I) ((N) * z_at_pt[I])
          #define Z6(I) ZP(6, I)
          #define Z4(I) ZP(4, I)
          #define Z2(I) ZP(2, I)
          #define Z1(I) ZP(1, I)

          #if ENABLED(PROBE_MANUALLY)
            test_precision = 0.00; // forced end
          #endif

          switch (probe_points) {
            case 1:
              test_precision = 0.00; // forced end
              LOOP_XYZ(i) e_delta[i] = Z1(0);
              break;

            case 2:
              if (towers_set) {
                e_delta[A_AXIS] = (Z6(0) + Z4(1) - Z2(5) - Z2(9)) * h_factor;
                e_delta[B_AXIS] = (Z6(0) - Z2(1) + Z4(5) - Z2(9)) * h_factor;
                e_delta[C_AXIS] = (Z6(0) - Z2(1) - Z2(5) + Z4(9)) * h_factor;
                r_delta         = (Z6(0) - Z2(1) - Z2(5) - Z2(9)) * r_factor;
              }
              else {
                e_delta[A_AXIS] = (Z6(0) - Z4(7) + Z2(11) + Z2(3)) * h_factor;
                e_delta[B_AXIS] = (Z6(0) + Z2(7) - Z4(11) + Z2(3)) * h_factor;
                e_delta[C_AXIS] = (Z6(0) + Z2(7) + Z2(11) - Z4(3)) * h_factor;
                r_delta         = (Z6(0) - Z2(7) - Z2(11) - Z2(3)) * r_factor;
              }
              break;

            default:
              e_delta[A_AXIS] = (Z6(0) + Z2(1) - Z1(5) - Z1(9) - Z2(7) + Z1(11) + Z1(3)) * h_factor;
              e_delta[B_AXIS] = (Z6(0) - Z1(1) + Z2(5) - Z1(9) + Z1(7) - Z2(11) + Z1(3)) * h_factor;
              e_delta[C_AXIS] = (Z6(0) - Z1(1) - Z1(5) + Z2(9) + Z1(7) + Z1(11) - Z2(3)) * h_factor;
              r_delta         = (Z6(0) - Z1(1) - Z1(5) - Z1(9) - Z1(7) - Z1(11) - Z1(3)) * r_factor;

              if (towers_set) {
                t_delta[A_AXIS] = (            - Z1(5) + Z1(9)         - Z1(11) + Z1(3)) * a_factor;
                t_delta[B_AXIS] = (      Z1(1)         - Z1(9) + Z1(7)          - Z1(3)) * a_factor;
                t_delta[C_AXIS] = (    - Z1(1) + Z1(5)         - Z1(7) + Z1(11)        ) * a_factor;
              }
              break;
          }

          LOOP_XYZ(axis) delta_endstop_adj[axis] += e_delta[axis];
          delta_radius += r_delta;
          LOOP_XYZ(axis) delta_tower_angle_adj[axis] += t_delta[axis];
        }
        else if (zero_std_dev >= test_precision) {   // step one back
          COPY_ARRAY(delta_endstop_adj, e_old);
          delta_radius = dr_old;
          delta_height = dh_old;
          COPY_ARRAY(delta_tower_angle_adj, ta_old);
        }
        if (verbose_level != 0) {                                    // !dry run
          // normalise angles to least squares
          float a_sum = 0.0;
          LOOP_XYZ(axis) a_sum += delta_tower_angle_adj[axis];
          LOOP_XYZ(axis) delta_tower_angle_adj[axis] -= a_sum / 3.0;

          // adjust delta_height and endstops by the max amount
          const float z_temp = MAX3(delta_endstop_adj[A_AXIS], delta_endstop_adj[B_AXIS], delta_endstop_adj[C_AXIS]);
          delta_height -= z_temp;
          LOOP_XYZ(i) delta_endstop_adj[i] -= z_temp;
        }
        recalc_delta_settings();
        NOMORE(zero_std_dev_min, zero_std_dev);

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
        if (verbose_level != 0) {                                    // !dry run
          if ((zero_std_dev >= test_precision || zero_std_dev <= calibration_precision) && iterations > force_iterations) {  // end iterations
            SERIAL_MSG("Calibration OK");
            SERIAL_SP(36);
            #if DISABLED(PROBE_MANUALLY)
              if (zero_std_dev >= test_precision && !_1p_calibration)
                SERIAL_MSG("rolling back.");
              else
            #endif
              SERIAL_MV("std dev:", zero_std_dev, 3);
            SERIAL_EOL();
            char mess[21];
            sprintf_P(mess, PSTR("Calibration sd:"), "");
            if (zero_std_dev_min < 1)
              sprintf_P(&mess[15], PSTR("0.%03i"), (int)round(zero_std_dev_min * 1000.0));
            else
              sprintf_P(&mess[15], PSTR("%03i.x"), (int)round(zero_std_dev_min));
            lcd_setstatus(mess);
            print_G33_settings(!_1p_calibration, _7p_calibration && towers_set);
            SERIAL_PS(save_message);
            SERIAL_EOL();
          }
          else {                                                     // !end iterations
            char mess[15];
            if (iterations < 31)
              sprintf_P(mess, PSTR("Iteration : %02i"), (int)iterations);
            else
              sprintf_P(mess, PSTR("No convergence"), "");
            SERIAL_TXT(mess);
            SERIAL_SP(36);
            SERIAL_EMV("std dev:", zero_std_dev, 3);
            lcd_setstatus(mess);
            print_G33_settings(!_1p_calibration, _7p_calibration && towers_set);
          }
        }
        else {
          const char *enddryrun = PSTR("End DRY-RUN");
          SERIAL_PS(enddryrun);
          SERIAL_SP(39);
          SERIAL_EMV("std dev:", zero_std_dev, 3);

          char mess[21];
          sprintf_P(mess, enddryrun, "");
          sprintf_P(&mess[11], PSTR(" sd:"), "");
          if (zero_std_dev < 1)
            sprintf_P(&mess[15], PSTR("0.%03i"), (int)round(zero_std_dev * 1000.0));
          else
            sprintf_P(&mess[15], PSTR("%03i.x"), (int)round(zero_std_dev));
          lcd_setstatus(mess);
        }

        endstops.enable(true);
        if (!Home()) return;
        endstops.not_homing();

      } while ((zero_std_dev < test_precision && zero_std_dev > calibration_precision && iterations < 31) || iterations <= force_iterations);

      CALIBRATION_CLEANUP();
    }

  #endif

  void Delta_Mechanics::print_signed_float(const char * const prefix, const float &f) {
    SERIAL_MSG("  ");
    SERIAL_PS(prefix);
    SERIAL_CHR(':');
    if (f >= 0) SERIAL_CHR('+');
    SERIAL_VAL(f, 2);
  }

  void Delta_Mechanics::print_G33_settings(const bool end_stops, const bool tower_angles) {
    SERIAL_MV(".Height:", delta_height, 2);
    if (end_stops) {
      print_signed_float(PSTR("  Ex"), delta_endstop_adj[A_AXIS]);
      print_signed_float(PSTR("Ey"), delta_endstop_adj[B_AXIS]);
      print_signed_float(PSTR("Ez"), delta_endstop_adj[C_AXIS]);
      SERIAL_MV("    Radius:", delta_radius, 2);
    }
    SERIAL_EOL();
    if (tower_angles) {
      SERIAL_MSG(".Tower angle :  ");
      print_signed_float(PSTR("Tx"), delta_tower_angle_adj[A_AXIS]);
      print_signed_float(PSTR("Ty"), delta_tower_angle_adj[B_AXIS]);
      print_signed_float(PSTR("Tz"), delta_tower_angle_adj[C_AXIS]);
      SERIAL_EOL();
    }
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
        delta_tower_angle_adj[A_AXIS] += v[4];
        delta_tower_angle_adj[B_AXIS] += v[5];

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

  // Report detail current position to host
  void Delta_Mechanics::report_current_position_detail() {

    stepper.synchronize();

    SERIAL_MSG("\nLogical:");
    report_xyze(current_position);

    SERIAL_MSG("Raw:    ");
    const float raw[XYZ] = { RAW_X_POSITION(current_position[X_AXIS]), RAW_Y_POSITION(current_position[Y_AXIS]), RAW_Z_POSITION(current_position[Z_AXIS]) };
    report_xyz(raw);

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
    Transform(leveled);  // writes delta[]
    report_xyz(delta);

    SERIAL_MSG("Stepper:");
    const long step_count[XYZE] = { stepper.position(X_AXIS), stepper.position(Y_AXIS), stepper.position(Z_AXIS), stepper.position(E_AXIS) };
    report_xyze((float*)step_count, 4, 0);

    SERIAL_MSG("FromStp:");
    get_cartesian_from_steppers();  // writes cartesian_position[XYZ] (with forward kinematics)
    const float from_steppers[XYZE] = { cartesian_position[X_AXIS], cartesian_position[Y_AXIS], cartesian_position[Z_AXIS], get_axis_position_mm(E_AXIS) };
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

#endif // IS_DELTA

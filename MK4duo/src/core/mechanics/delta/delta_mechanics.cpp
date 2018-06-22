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

  /** Public Parameters */
  float Delta_Mechanics::delta[ABC]                     = { 0.0 },
        Delta_Mechanics::delta_diagonal_rod             = DELTA_DIAGONAL_ROD,
        Delta_Mechanics::delta_radius                   = DELTA_RADIUS,
        Delta_Mechanics::delta_segments_per_second      = DELTA_SEGMENTS_PER_SECOND,
        Delta_Mechanics::delta_print_radius             = DELTA_PRINTABLE_RADIUS,
        Delta_Mechanics::delta_probe_radius             = DELTA_PROBEABLE_RADIUS,
        Delta_Mechanics::delta_height                   = DELTA_HEIGHT,
        Delta_Mechanics::delta_clip_start_height        = DELTA_HEIGHT,
        Delta_Mechanics::delta_endstop_adj[ABC]         = { TOWER_A_ENDSTOP_ADJ, TOWER_B_ENDSTOP_ADJ, TOWER_C_ENDSTOP_ADJ },
        Delta_Mechanics::delta_tower_angle_adj[ABC]     = { TOWER_A_ANGLE_ADJ, TOWER_B_ANGLE_ADJ, TOWER_C_ANGLE_ADJ },
        Delta_Mechanics::delta_tower_radius_adj[ABC]    = { TOWER_A_RADIUS_ADJ, TOWER_B_RADIUS_ADJ, TOWER_C_RADIUS_ADJ },
        Delta_Mechanics::delta_diagonal_rod_adj[ABC]    = { TOWER_A_DIAGROD_ADJ, TOWER_B_DIAGROD_ADJ, TOWER_C_DIAGROD_ADJ };

  /** Private Parameters */
  float Delta_Mechanics::delta_diagonal_rod_2[ABC] = { 0.0 },  // Diagonal rod 2
        Delta_Mechanics::towerX[ABC]               = { 0.0 },  // The X coordinate of each tower
        Delta_Mechanics::towerY[ABC]               = { 0.0 },  // The Y coordinate of each tower
        Delta_Mechanics::Xbc                       = 0.0,
        Delta_Mechanics::Xca                       = 0.0,
        Delta_Mechanics::Xab                       = 0.0,
        Delta_Mechanics::Ybc                       = 0.0,
        Delta_Mechanics::Yca                       = 0.0,
        Delta_Mechanics::Yab                       = 0.0,
        Delta_Mechanics::coreFa                    = 0.0,
        Delta_Mechanics::coreFb                    = 0.0,
        Delta_Mechanics::coreFc                    = 0.0,
        Delta_Mechanics::Q                         = 0.0,
        Delta_Mechanics::Q2                        = 0.0,
        Delta_Mechanics::D2                        = 0.0;

  void Delta_Mechanics::init() {
    delta_diagonal_rod              = DELTA_DIAGONAL_ROD;
    delta_radius                    = DELTA_RADIUS;
    delta_segments_per_second       = DELTA_SEGMENTS_PER_SECOND;
    delta_print_radius              = DELTA_PRINTABLE_RADIUS;
    delta_probe_radius              = DELTA_PROBEABLE_RADIUS;
    delta_height                    = DELTA_HEIGHT;
    delta_clip_start_height         = DELTA_HEIGHT;
    delta_endstop_adj[A_AXIS]       = TOWER_A_ENDSTOP_ADJ;
    delta_endstop_adj[B_AXIS]       = TOWER_B_ENDSTOP_ADJ;
    delta_endstop_adj[C_AXIS]       = TOWER_C_ENDSTOP_ADJ;
    delta_tower_angle_adj[A_AXIS]   = TOWER_A_ANGLE_ADJ;
    delta_tower_angle_adj[B_AXIS]   = TOWER_B_ANGLE_ADJ;
    delta_tower_angle_adj[C_AXIS]   = TOWER_C_ANGLE_ADJ;
    delta_tower_radius_adj[A_AXIS]  = TOWER_A_RADIUS_ADJ;
    delta_tower_radius_adj[B_AXIS]  = TOWER_B_RADIUS_ADJ;
    delta_tower_radius_adj[C_AXIS]  = TOWER_C_RADIUS_ADJ;
    delta_diagonal_rod_adj[A_AXIS]  = TOWER_A_DIAGROD_ADJ;
    delta_diagonal_rod_adj[B_AXIS]  = TOWER_B_DIAGROD_ADJ;
    delta_diagonal_rod_adj[C_AXIS]  = TOWER_C_DIAGROD_ADJ;
    recalc_delta_settings();
  }

  void Delta_Mechanics::sync_plan_position_mech_specific() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("sync_plan_position_mech_specific", current_position);
    #endif
    planner.set_position_mm_kinematic(current_position);
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
      planner.get_axis_position_mm(A_AXIS),
      planner.get_axis_position_mm(B_AXIS),
      planner.get_axis_position_mm(C_AXIS),
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
      // gives the number of segments we should produce
      uint16_t segments = delta_segments_per_second * seconds;

      // At least one segment is required
      NOLESS(segments, 1U);

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

        if (!planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], _feedrate_mm_s, tools.active_extruder, cartesian_segment_mm))
          break;

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

    if (!mechanics.position_is_reachable(rx, ry)) return;

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

    feedrate_mm_s = old_feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("<<< do_blocking_move_to");
    #endif

    planner.synchronize();

  }

  /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void Delta_Mechanics::prepare_uninterpolated_move_to_destination(const float fr_mm_s/*=0.0*/) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);
    #endif

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
    delta_clip_start_height = delta_height - ABS(distance - delta[A_AXIS]);
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
    mechanics.do_homing_move(axis, 1.5 * delta_height);

    // When homing Z with probe respect probe clearance
    const float bump = home_bump_mm[axis];

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

    // Delta has already moved all three towers up in G28
    // so here it re-homes each tower in turn.
    // Delta homing treats the axes as normal linear axes.

    // retrace by the amount specified in delta_endstop_adj + additional 0.1mm in order to have minimum steps
    if (delta_endstop_adj[axis] < 0) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("delta_endstop_adj:");
      #endif
      mechanics.do_homing_move(axis, delta_endstop_adj[axis] - 0.1);
    }

    // Clear z_lift if homing the Z axis
    #if ENABLED(FWRETRACT)
      if (axis == Z_AXIS) fwretract.hop_amount = 0.0;
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
  bool Delta_Mechanics::home() {

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

    #if ENABLED(BLTOUCH)
      probe.bltouch_command(BLTOUCH_RESET);
      probe.set_bltouch_deployed(false);
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

    // Init the current position of all carriages to 0,0,0
    ZERO(current_position);
    mechanics.sync_plan_position();

    // Disable stealthChop if used. Enable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      mechanics.sensorless_homing();
    #endif

    // Move all carriages together linearly until an endstop is hit.
    current_position[A_AXIS] = current_position[B_AXIS] = current_position[C_AXIS] = delta_height + 10;
    feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
    mechanics.line_to_current_position();
    planner.synchronize();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      sensorless_homing(false);
    #endif

    // If an endstop was not hit, then damage can occur if homing is continued.
    // This can occur if the delta height not set correctly.
    if (!(endstops.trigger_state() && (_BV(X_MAX) | _BV(Y_MAX) | _BV(Z_MAX)))) {
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

    sync_plan_position_mech_specific();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) DEBUG_POS("<<< home_delta", current_position);
    #endif

    endstops.setNotHoming();

    #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
      // move to a height where we can use the full xy-area
      mechanics.do_blocking_move_to_z(delta_clip_start_height);
    #endif

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

    SERIAL_MSG("FromStp:");
    get_cartesian_from_steppers();
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
      float zHi, zLo, newPos[ABC];

      switch(deriv) {
        case 0:
        case 1:
        case 2:
          // Endstop corrections
          InverseTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc, newPos);
          zHi = newPos[C_AXIS];
          InverseTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc, newPos);
          zLo = newPos[C_AXIS];
          break;

        case 3: {
          const float old_delta_radius = delta_radius;

          // Calc High parameters
          delta_radius += perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zHi = newPos[C_AXIS];

          // Reset Delta Radius
          delta_radius = old_delta_radius;

          // Calc Low parameters
          delta_radius -= perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zLo = newPos[C_AXIS];

          // Reset Delta Radius
          delta_radius = old_delta_radius;
          break;
        }

        case 4: {
          const float old_delta_tower_angle_adj = delta_tower_angle_adj[A_AXIS];

          // Calc High parameters
          delta_tower_angle_adj[A_AXIS] += perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zHi = newPos[C_AXIS];

          // Reset Delta tower Alpha angle adj 
          delta_tower_angle_adj[A_AXIS] = old_delta_tower_angle_adj;

          // Calc Low parameters
          delta_tower_angle_adj[A_AXIS] -= perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zLo = newPos[C_AXIS];

          // Reset Delta tower Alpha angle adj 
          delta_tower_angle_adj[A_AXIS] = old_delta_tower_angle_adj;
          break;
        }

        case 5: {
          const float old_delta_tower_angle_adj = delta_tower_angle_adj[B_AXIS];

          // Calc High parameters
          delta_tower_angle_adj[B_AXIS] += perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zHi = newPos[C_AXIS];

          // Reset Delta tower Beta angle adj 
          delta_tower_angle_adj[B_AXIS] = old_delta_tower_angle_adj;

          // Calc Low parameters
          delta_tower_angle_adj[B_AXIS] -= perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zLo = newPos[C_AXIS];

          // Reset Delta tower Beta angle adj 
          delta_tower_angle_adj[B_AXIS] = old_delta_tower_angle_adj;
          break;
        }

        case 6: {
          const float old_delta_diagonal_rod = delta_diagonal_rod;

          // Calc High parameters
          delta_diagonal_rod += perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zHi = newPos[C_AXIS];

          // Reset Delta Diagonal Rod
          delta_diagonal_rod = old_delta_diagonal_rod;

          // Calc Low parameters
          delta_diagonal_rod -= perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zLo = newPos[C_AXIS];

          // Reset Delta Diagonal Rod
          delta_diagonal_rod = old_delta_diagonal_rod;
          break;
        }
      }

      recalc_delta_settings();

      return (zHi - zLo) / (2 * perturb);
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
     * Set sensorless homing.
     */
    void Delta_Mechanics::sensorless_homing(const bool on/*=true*/) {
      sensorless_homing_per_axis(A_AXIS, on);
      sensorless_homing_per_axis(B_AXIS, on);
      sensorless_homing_per_axis(C_AXIS, on);
    }

  #endif // SENSORLESS_HOMING

#endif // IS_DELTA

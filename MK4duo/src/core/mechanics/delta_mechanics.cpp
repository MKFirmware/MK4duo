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

#include "../../../MK4duo.h"

#if IS_DELTA

  Delta_Mechanics mechanics;

  #if ENABLED(DELTA_FAST_SQRT) && ENABLED(__AVR__)
    #define _SQRT(n) (1.0f / Q_rsqrt(n))
  #else
    #define _SQRT(n) SQRT(n)
  #endif

  /** Public Parameters */
  mechanics_data_t Delta_Mechanics::data;

  float Delta_Mechanics::delta[ABC]                 = { 0.0 },
        Delta_Mechanics::delta_clip_start_height    = 0;

  /** Private Parameters */
  float Delta_Mechanics::delta_diagonal_rod_2[ABC]  = { 0.0 },  // Diagonal rod 2
        Delta_Mechanics::towerX[ABC]                = { 0.0 },  // The X coordinate of each tower
        Delta_Mechanics::towerY[ABC]                = { 0.0 },  // The Y coordinate of each tower
        Delta_Mechanics::Xbc                        = 0.0,
        Delta_Mechanics::Xca                        = 0.0,
        Delta_Mechanics::Xab                        = 0.0,
        Delta_Mechanics::Ybc                        = 0.0,
        Delta_Mechanics::Yca                        = 0.0,
        Delta_Mechanics::Yab                        = 0.0,
        Delta_Mechanics::coreFa                     = 0.0,
        Delta_Mechanics::coreFb                     = 0.0,
        Delta_Mechanics::coreFc                     = 0.0,
        Delta_Mechanics::Q                          = 0.0,
        Delta_Mechanics::Q2                         = 0.0,
        Delta_Mechanics::D2                         = 0.0;

  /** Public Function */
  void Delta_Mechanics::factory_parameters() {

    static const float    tmp_step[]          PROGMEM = DEFAULT_AXIS_STEPS_PER_UNIT,
                          tmp_maxfeedrate[]   PROGMEM = DEFAULT_MAX_FEEDRATE;

    static const uint32_t tmp_maxacc[]        PROGMEM = DEFAULT_MAX_ACCELERATION,
                          tmp_retractacc[]    PROGMEM = DEFAULT_RETRACT_ACCELERATION;

    LOOP_XYZE_N(i) {
      data.axis_steps_per_mm[i]           = pgm_read_float(&tmp_step[i < COUNT(tmp_step) ? i : COUNT(tmp_step) - 1]);
      data.max_feedrate_mm_s[i]           = pgm_read_float(&tmp_maxfeedrate[i < COUNT(tmp_maxfeedrate) ? i : COUNT(tmp_maxfeedrate) - 1]);
      data.max_acceleration_mm_per_s2[i]  = pgm_read_dword_near(&tmp_maxacc[i < COUNT(tmp_maxacc) ? i : COUNT(tmp_maxacc) - 1]);
    }

    LOOP_EXTRUDER()
      data.retract_acceleration[e]  = pgm_read_dword_near(&tmp_retractacc[e < COUNT(tmp_retractacc) ? e : COUNT(tmp_retractacc) - 1]);

    data.acceleration               = DEFAULT_ACCELERATION;
    data.travel_acceleration        = DEFAULT_TRAVEL_ACCELERATION;
    data.min_feedrate_mm_s          = DEFAULT_MIN_FEEDRATE;
    data.min_segment_time_us        = DEFAULT_MIN_SEGMENT_TIME;
    data.min_travel_feedrate_mm_s   = DEFAULT_MIN_TRAVEL_FEEDRATE;

    #if ENABLED(JUNCTION_DEVIATION)
      data.junction_deviation_mm = float(JUNCTION_DEVIATION_MM);
    #endif

    static const float tmp_ejerk[] PROGMEM = DEFAULT_EJERK;
    data.max_jerk[X_AXIS]  = DEFAULT_XJERK;
    data.max_jerk[Y_AXIS]  = DEFAULT_YJERK;
    data.max_jerk[Z_AXIS]  = DEFAULT_ZJERK;
    #if DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
      LOOP_EXTRUDER()
        data.max_jerk[E_AXIS + e] = pgm_read_float(&tmp_ejerk[e < COUNT(tmp_ejerk) ? e : COUNT(tmp_ejerk) - 1]);
    #endif

    data.diagonal_rod             = DELTA_DIAGONAL_ROD;
    data.radius                   = DELTA_RADIUS;
    data.segments_per_second      = DELTA_SEGMENTS_PER_SECOND;
    data.print_radius             = DELTA_PRINTABLE_RADIUS;
    data.probe_radius             = DELTA_PROBEABLE_RADIUS;
    data.height                   = DELTA_HEIGHT;
    data.endstop_adj[A_AXIS]      = TOWER_A_ENDSTOP_ADJ;
    data.endstop_adj[B_AXIS]      = TOWER_B_ENDSTOP_ADJ;
    data.endstop_adj[C_AXIS]      = TOWER_C_ENDSTOP_ADJ;
    data.tower_angle_adj[A_AXIS]  = TOWER_A_ANGLE_ADJ;
    data.tower_angle_adj[B_AXIS]  = TOWER_B_ANGLE_ADJ;
    data.tower_angle_adj[C_AXIS]  = TOWER_C_ANGLE_ADJ;
    data.tower_radius_adj[A_AXIS] = TOWER_A_RADIUS_ADJ;
    data.tower_radius_adj[B_AXIS] = TOWER_B_RADIUS_ADJ;
    data.tower_radius_adj[C_AXIS] = TOWER_C_RADIUS_ADJ;
    data.diagonal_rod_adj[A_AXIS] = TOWER_A_DIAGROD_ADJ;
    data.diagonal_rod_adj[B_AXIS] = TOWER_B_DIAGROD_ADJ;
    data.diagonal_rod_adj[C_AXIS] = TOWER_C_DIAGROD_ADJ;

    delta_clip_start_height       = DELTA_HEIGHT;

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
        planner.buffer_line(destination, _feedrate_mm_s, tools.active_extruder);
        return false; // caller will update current_position
      }

      // Fail if attempting move outside printable radius
      if (endstops.isSoftEndstop() && !position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) return true;

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
      uint16_t segments = data.segments_per_second * seconds;

      // At least one segment is required
      NOLESS(segments, 1U);

      // The approximate length of each segment
      const float inv_segments = 1.0f / float(segments),
                  segment_distance[XYZE] = {
                    difference[X_AXIS] * inv_segments,
                    difference[Y_AXIS] * inv_segments,
                    difference[Z_AXIS] * inv_segments,
                    difference[E_AXIS] * inv_segments
                  },
                  cartesian_segment_mm = cartesian_mm * inv_segments;

      /*
      SERIAL_MV("mm=", cartesian_mm);
      SERIAL_MV(" seconds=", seconds);
      SERIAL_MV(" segments=", segments);
      SERIAL_MV(" segment_mm=", cartesian_segment_mm);
      SERIAL_EOL();
      //*/

      // Get the current position as starting point
      float raw[XYZE];
      COPY_ARRAY(raw, current_position);

      // Calculate and execute the segments
      while (--segments) {

        printer.check_periodical_actions();

        LOOP_XYZE(i) raw[i] += segment_distance[i];

        if (!planner.buffer_line(raw, _feedrate_mm_s, tools.active_extruder, cartesian_segment_mm))
          break;

      }

      planner.buffer_line(destination, _feedrate_mm_s, tools.active_extruder, cartesian_segment_mm);

      return false; // caller will update current_position

    }

  #endif // DISABLED(AUTO_BED_LEVELING_UBL)

  /**
   *  Plan a move to (X, Y, Z) and set the current_position
   *  The final current_position may not be the one that was requested
   */
  void Delta_Mechanics::do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s /*=0.0*/) {
    const float old_feedrate_mm_s = feedrate_mm_s;

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) Com::print_xyz(PSTR(">>> do_blocking_move_to"), NULL, rx, ry, rz);
    #endif

    const float z_feedrate = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];

    if (!position_is_reachable(rx, ry)) return;

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

    set_destination_to_current();          // sync destination at the start

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) DEBUG_POS("set_destination_to_current", destination);
    #endif

    // when in the danger zone
    if (current_position[C_AXIS] > delta_clip_start_height) {
      if (rz > delta_clip_start_height) {   // staying in the danger zone
        destination[A_AXIS] = rx;           // move directly (uninterpolated)
        destination[B_AXIS] = ry;
        destination[C_AXIS] = rz;
        prepare_uninterpolated_move_to_destination(); // set_current_to_destination
        #if ENABLED(DEBUG_FEATURE)
          if (printer.debugFeature()) DEBUG_POS("danger zone move", current_position);
        #endif
        return;
      }
      destination[C_AXIS] = delta_clip_start_height;
      prepare_uninterpolated_move_to_destination(); // set_current_to_destination
      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) DEBUG_POS("zone border move", current_position);
      #endif
    }

    if (rz > current_position[C_AXIS]) {    // raising?
      destination[C_AXIS] = rz;
      prepare_uninterpolated_move_to_destination(z_feedrate);   // set_current_to_destination
      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) DEBUG_POS("z raise move", current_position);
      #endif
    }

    destination[A_AXIS] = rx;
    destination[B_AXIS] = ry;
    prepare_move_to_destination();         // set_current_to_destination
    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) DEBUG_POS("xy move", current_position);
    #endif

    if (rz < current_position[C_AXIS]) {    // lowering?
      destination[C_AXIS] = rz;
      prepare_uninterpolated_move_to_destination(z_feedrate);   // set_current_to_destination
      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) DEBUG_POS("z lower move", current_position);
      #endif
    }

    feedrate_mm_s = old_feedrate_mm_s;

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) SERIAL_EM("<<< do_blocking_move_to");
    #endif

    planner.synchronize();

  }
  void Delta_Mechanics::do_blocking_move_to_x(const float &rx, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(rx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
  }
  void Delta_Mechanics::do_blocking_move_to_z(const float &rz, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], rz, fr_mm_s);
  }
  void Delta_Mechanics::do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(rx, ry, current_position[Z_AXIS], fr_mm_s);
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
   * by Andreas Hardtung 07-06-2016
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
  void Delta_Mechanics::Transform(const float (&raw)[XYZ]) {

    #if HOTENDS > 1
      // Delta hotend offsets must be applied in Cartesian space
      const float pos[XYZ] = {
        raw[X_AXIS] - tools.hotend_offset[X_AXIS][tools.active_extruder],
        raw[Y_AXIS] - tools.hotend_offset[Y_AXIS][tools.active_extruder],
        raw[Z_AXIS]
      };
      delta[A_AXIS] = pos[Z_AXIS] + _SQRT(delta_diagonal_rod_2[A_AXIS] - HYPOT2(towerX[A_AXIS] - pos[X_AXIS], towerY[A_AXIS] - pos[Y_AXIS]));
      delta[B_AXIS] = pos[Z_AXIS] + _SQRT(delta_diagonal_rod_2[B_AXIS] - HYPOT2(towerX[B_AXIS] - pos[X_AXIS], towerY[B_AXIS] - pos[Y_AXIS]));
      delta[C_AXIS] = pos[Z_AXIS] + _SQRT(delta_diagonal_rod_2[C_AXIS] - HYPOT2(towerX[C_AXIS] - pos[X_AXIS], towerY[C_AXIS] - pos[Y_AXIS]));
    #else
      delta[A_AXIS] = raw[Z_AXIS] + _SQRT(delta_diagonal_rod_2[A_AXIS] - HYPOT2(towerX[A_AXIS] - raw[X_AXIS], towerY[A_AXIS] - raw[Y_AXIS]));
      delta[B_AXIS] = raw[Z_AXIS] + _SQRT(delta_diagonal_rod_2[B_AXIS] - HYPOT2(towerX[B_AXIS] - raw[X_AXIS], towerY[B_AXIS] - raw[Y_AXIS]));
      delta[C_AXIS] = raw[Z_AXIS] + _SQRT(delta_diagonal_rod_2[C_AXIS] - HYPOT2(towerX[C_AXIS] - raw[X_AXIS], towerY[C_AXIS] - raw[Y_AXIS]));
    #endif

  }

  void Delta_Mechanics::Transform(const float (&raw)[XYZE]) {
    const float raw_xyz[XYZ] = { raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS] };
    Transform(raw_xyz);
  }

  void Delta_Mechanics::recalc_delta_settings() {

    // Get a minimum radius for clamping
    endstops.soft_endstop_radius_2 = sq(data.print_radius);

    delta_diagonal_rod_2[A_AXIS] = sq(data.diagonal_rod + data.diagonal_rod_adj[A_AXIS]);
    delta_diagonal_rod_2[B_AXIS] = sq(data.diagonal_rod + data.diagonal_rod_adj[B_AXIS]);
    delta_diagonal_rod_2[C_AXIS] = sq(data.diagonal_rod + data.diagonal_rod_adj[C_AXIS]);

    // Effective X/Y positions of the three vertical towers.
    towerX[A_AXIS] = COS(RADIANS(210 + data.tower_angle_adj[A_AXIS])) * (data.radius + data.tower_radius_adj[A_AXIS]); // front left tower
    towerY[A_AXIS] = SIN(RADIANS(210 + data.tower_angle_adj[A_AXIS])) * (data.radius + data.tower_radius_adj[A_AXIS]);
    towerX[B_AXIS] = COS(RADIANS(330 + data.tower_angle_adj[B_AXIS])) * (data.radius + data.tower_radius_adj[B_AXIS]); // front right tower
    towerY[B_AXIS] = SIN(RADIANS(330 + data.tower_angle_adj[B_AXIS])) * (data.radius + data.tower_radius_adj[B_AXIS]);
    towerX[C_AXIS] = COS(RADIANS( 90 + data.tower_angle_adj[C_AXIS])) * (data.radius + data.tower_radius_adj[C_AXIS]); // back middle tower
    towerY[C_AXIS] = SIN(RADIANS( 90 + data.tower_angle_adj[C_AXIS])) * (data.radius + data.tower_radius_adj[C_AXIS]);

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
    D2 = sq(data.diagonal_rod);

    NOMORE(data.probe_radius, data.print_radius);

    printer.unsetHomedAll();

    Set_clip_start_height();

  }

  /**
   * Home Delta
   */
  void Delta_Mechanics::home(const bool report_position/*=true*/) {

    if (printer.debugSimulation()) {
      LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        Nextion_gfx_clear();
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
    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) SERIAL_EM("> endstops.setEnabled(true)");
    #endif
    endstops.setEnabled(true); // Enable endstops for next homing move

    bool come_back = parser.boolval('B');
    float lastpos[NUM_AXIS];
    float old_feedrate_mm_s;
    if (come_back) {
      old_feedrate_mm_s = feedrate_mm_s;
      COPY_ARRAY(lastpos, current_position);
    }

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) DEBUG_POS(">>> home", current_position);
    #endif

    // Init the current position of all carriages to 0,0,0
    ZERO(current_position);
    ZERO(destination);
    sync_plan_position();

    // Disable stealthChop if used. Enable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      sensorless_homing();
    #endif

    // Move all carriages together linearly until an endstop is hit.
    destination[Z_AXIS] = data.height + 10;
    planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], homing_feedrate_mm_s[X_AXIS], tools.active_extruder);
    planner.synchronize();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      sensorless_homing(false);
    #endif

    endstops.validate_homing_move();

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

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) DEBUG_POS("<<< home", current_position);
    #endif

    endstops.setNotHoming();

    #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
      // move to a height where we can use the full xy-area
      do_blocking_move_to_z(delta_clip_start_height, homing_feedrate_mm_s[X_AXIS]);
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

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) SERIAL_EM("<<< G28");
    #endif

    if (report_position) report_current_position();
  }

  /**
   * Home an individual linear axis
   */
  void Delta_Mechanics::do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s/*=0.0*/) {

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
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

    // Only do some things when moving towards an endstop
    const bool is_home_dir = distance > 0;

    if (is_home_dir) {
      // Disable stealthChop if used. Enable diag1 pin on driver.
      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(axis);
      #endif
    }

    float target[ABCE] = { planner.get_axis_position_mm(A_AXIS), planner.get_axis_position_mm(B_AXIS), planner.get_axis_position_mm(C_AXIS), planner.get_axis_position_mm(E_AXIS) };
    target[axis] = 0;
    planner.set_machine_position_mm(target);
    target[axis] = distance;

    #if ENABLED(JUNCTION_DEVIATION)
      const float delta_mm_cart[XYZE] = {0, 0, 0, 0};
    #endif

    // Set delta axes directly
    planner.buffer_segment(target
      #if ENABLED(JUNCTION_DEVIATION)
        , delta_mm_cart
      #endif
      , fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], tools.active_extruder
    );

    planner.synchronize();

    if (is_home_dir) {
      endstops.validate_homing_move();

      // Re-enable stealthChop if used. Disable diag1 pin on driver.
      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(axis, false);
      #endif
    }

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MV("<<< do_homing_move(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

  }

  /**
   * Set an axis' current position to its home position (after homing).
   *
   * DELTA should wait until all homing is done before setting the XYZ
   * current_position to home, because homing is a single operation.
   * In the case where the axis positions are already known and previously
   * homed, DELTA could home to X or Y individually by moving either one
   * to the center. However, homing Z always homes XY and Z.
   *
   * Callers must sync the planner position after calling this!
   */
  void Delta_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    printer.setAxisHomed(axis, true);

    current_position[axis] = (axis == C_AXIS ? data.height : 0.0);

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        DEBUG_POS("", current_position);
        SERIAL_MV("<<< set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  // Return true if the given point is within the printable area
  bool Delta_Mechanics::position_is_reachable(const float &rx, const float &ry) {
    return HYPOT2(rx, ry) <= sq(data.print_radius);
  }
  // Return true if the both nozzle and the probe can reach the given point.
  bool Delta_Mechanics::position_is_reachable_by_probe(const float &rx, const float &ry) {
    return position_is_reachable(rx, ry)
        && position_is_reachable(rx - probe.offset[X_AXIS], ry - probe.offset[Y_AXIS]);
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

  #if ENABLED(ARC_SUPPORT)

    #if N_ARC_CORRECTION < 1
      #undef N_ARC_CORRECTION
      #define N_ARC_CORRECTION 1
    #endif

    /**
     * Plan an arc in 2 dimensions
     *
     * The arc is approximated by generating many small linear segments.
     * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
     * Arcs should only be made relatively large (over 5mm), as larger arcs with
     * larger segments will tend to be more efficient. Your slicer should have
     * options for G2/G3 arc generation. In future these options may be GCode tunable.
     */
    void Delta_Mechanics::plan_arc(
      const float (&cart)[XYZE],  // Destination position
      const float (&offset)[2],   // Center of rotation relative to current_position
      const uint8_t clockwise     // Clockwise?
    ) {

      // Radius vector from center to current location
      float r_P = -offset[0], r_Q = -offset[1];

      const float radius = HYPOT(r_P, r_Q),
                  center_P = current_position[X_AXIS] - r_P,
                  center_Q = current_position[Y_AXIS] - r_Q,
                  rt_X = cart[X_AXIS] - center_P,
                  rt_Y = cart[Y_AXIS] - center_Q,
                  linear_travel = cart[Z_AXIS] - current_position[Z_AXIS],
                  extruder_travel = cart[E_AXIS] - current_position[E_AXIS];

      // CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
      float angular_travel = ATAN2(r_P * rt_Y - r_Q * rt_X, r_P * rt_X + r_Q * rt_Y);
      if (angular_travel < 0) angular_travel += RADIANS(360);
      if (clockwise) angular_travel -= RADIANS(360);

      // Make a circle if the angular rotation is 0
      if (angular_travel == 0 && current_position[X_AXIS] == cart[X_AXIS] && current_position[Y_AXIS] == cart[Y_AXIS])
        angular_travel += RADIANS(360);

      const float flat_mm = radius * angular_travel,
                  mm_of_travel = linear_travel ? HYPOT(flat_mm, linear_travel) : ABS(flat_mm);
      if (mm_of_travel < 0.001f) return;

      uint16_t segments = FLOOR(mm_of_travel / (MM_PER_ARC_SEGMENT));
      if (segments == 0) segments = 1;

      /**
       * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
       *     r_T = [cos(phi) -sin(phi);
       *            sin(phi)  cos(phi] * r ;
       *
       * For arc generation, the center of the circle is the axis of rotation and the radius vector is
       * defined from the circle center to the initial position. Each line segment is formed by successive
       * vector rotations. This requires only two cos() and sin() computations to form the rotation
       * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
       * all double numbers are single precision on the Arduino. (True double precision will not have
       * round off issues for CNC applications.) Single precision error can accumulate to be greater than
       * tool precision in some cases. Therefore, arc path correction is implemented.
       *
       * Small angle approximation may be used to reduce computation overhead further. This approximation
       * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
       * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
       * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
       * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
       * issue for CNC machines with the single precision Arduino calculations.
       *
       * This approximation also allows plan_arc to immediately insert a line segment into the planner
       * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
       * This is important when there are successive arc motions.
       */
      // Vector rotation matrix values
      float raw[XYZE];
      const float theta_per_segment = angular_travel / segments,
                  linear_per_segment = linear_travel / segments,
                  extruder_per_segment = extruder_travel / segments,
                  sin_T = theta_per_segment,
                  cos_T = 1 - 0.5f * sq(theta_per_segment); // Small angle approximation

      // Initialize the linear axis
      raw[Z_AXIS] = current_position[Z_AXIS];

      // Initialize the extruder axis
      raw[E_AXIS] = current_position[E_AXIS];

      const float fr_mm_s = MMS_SCALED(feedrate_mm_s);

      millis_t next_idle_ms = millis() + 200UL;

      #if ENABLED(DELTA_FEEDRATE_SCALING)
        // DELTA needs to scale the feed rate from mm/s to degrees/s
        const float inv_segment_length = 1.0f / float(MM_PER_ARC_SEGMENT),
                    inverse_secs = inv_segment_length * fr_mm_s;
        float oldA = planner.position_float[A_AXIS],
              oldB = planner.position_float[B_AXIS],
              oldC = planner.position_float[C_AXIS];
      #endif

      #if N_ARC_CORRECTION > 1
        int8_t arc_recalc_count = N_ARC_CORRECTION;
      #endif

      for (uint16_t i = 1; i < segments; i++) { // Iterate (segments-1) times

        printer.check_periodical_actions();
        if (ELAPSED(millis(), next_idle_ms)) {
          next_idle_ms = millis() + 200UL;
          printer.idle();
        }

        #if N_ARC_CORRECTION > 1
          if (--arc_recalc_count) {
            // Apply vector rotation matrix to previous r_P / 1
            const float r_new_Y = r_P * sin_T + r_Q * cos_T;
            r_P = r_P * cos_T - r_Q * sin_T;
            r_Q = r_new_Y;
          }
          else
        #endif
        {
          #if N_ARC_CORRECTION > 1
            arc_recalc_count = N_ARC_CORRECTION;
          #endif

          // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
          // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
          // To reduce stuttering, the sin and cos could be computed at different times.
          // For now, compute both at the same time.
          const float cos_Ti = cos(i * theta_per_segment),
                      sin_Ti = sin(i * theta_per_segment);
          r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti;
          r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti;
        }

        // Update raw location
        raw[X_AXIS] = center_P + r_P;
        raw[Y_AXIS] = center_Q + r_Q;
        raw[Z_AXIS] += linear_per_segment;
        raw[E_AXIS] += extruder_per_segment;

        endstops.clamp_to_software(raw);

        #if ENABLED(DELTA_FEEDRATE_SCALING)
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
        #endif

        #if ENABLED(DELTA_FEEDRATE_SCALING)
          // For DELTA scale the feed rate from Effector mm/s to Carriage mm/s
          // i.e., Complete the linear vector in the given time.
          if (!planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], SQRT(sq(delta[A_AXIS] - oldA) + sq(delta[B_AXIS] - oldB) + sq(delta[C_AXIS] - oldC)) * inverse_secs, tools.active_extruder, MM_PER_ARC_SEGMENT))
            break;
          oldA = delta[A_AXIS]; oldB = delta[B_AXIS]; oldC = delta[C_AXIS];
        #elif HAS_UBL_AND_CURVES
          float pos[XYZ] = { raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS] };
          bedlevel.apply_leveling(pos);
          if (!planner.buffer_segment(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], raw[E_AXIS], fr_mm_s, tools.active_extruder, MM_PER_ARC_SEGMENT))
            break;
        #else
          if (!planner.buffer_line(raw, fr_mm_s, tools.active_extruder))
            break;
        #endif
      }

      // Ensure last segment arrives at target location.
      #if ENABLED(DELTA_FEEDRATE_SCALING)
        Transform(cart);
        // Adjust Z if bed leveling is enabled
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          if (bedlevel.leveling_active) {
            const float zadj = abl.bilinear_z_offset(cart);
            delta[A_AXIS] += zadj;
            delta[B_AXIS] += zadj;
            delta[C_AXIS] += zadj;
          }
        #endif
      #endif

      #if ENABLED(DELTA_FEEDRATE_SCALING)
        const float diff2 = sq(delta[A_AXIS] - oldA) + sq(delta[B_AXIS] - oldB) + sq(delta[C_AXIS] - oldC);
        if (diff2)
          planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], cart[E_AXIS], SQRT(diff2) * inverse_secs, tools.active_extruder, MM_PER_ARC_SEGMENT);
      #elif HAS_UBL_AND_CURVES
        float pos[XYZ] = { cart[X_AXIS], cart[Y_AXIS], cart[Z_AXIS] };
        bedlevel.apply_leveling(pos);
        planner.buffer_segment(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], cart[E_AXIS], fr_mm_s, tools.active_extruder, MM_PER_ARC_SEGMENT);
      #else
        planner.buffer_line(cart, fr_mm_s, tools.active_extruder);
      #endif

      COPY_ARRAY(current_position, cart);

    }

  #endif // ENABLED(ARC_SUPPORT)

  #if ENABLED(DELTA_AUTO_CALIBRATION_1)

    // Compute the derivative of height with respect to a parameter at the specified motor endpoints.
    // 'deriv' indicates the parameter as follows:
    // 0, 1, 2 = X, Y, Z tower endstop adjustments
    // 3 = delta radius
    // 4 = X tower correction
    // 5 = Y tower correction
    // 6 = data.diagonal_rod rod length
    // 7, 8 = X tilt, Y tilt. We scale these by the printable radius to get sensible values in the range -1..1
    float Delta_Mechanics::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) {
      constexpr float perturb = 0.2;      // perturbation amount in mm or degrees
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
          const float old_delta_radius = data.radius;

          // Calc High parameters
          data.radius += perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zHi = newPos[C_AXIS];

          // Reset Delta Radius
          data.radius = old_delta_radius;

          // Calc Low parameters
          data.radius -= perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zLo = newPos[C_AXIS];

          // Reset Delta Radius
          data.radius = old_delta_radius;
          break;
        }

        case 4: {
          const float old_delta_tower_angle_adj = data.tower_angle_adj[A_AXIS];

          // Calc High parameters
          data.tower_angle_adj[A_AXIS] += perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zHi = newPos[C_AXIS];

          // Reset Delta tower Alpha angle adj 
          data.tower_angle_adj[A_AXIS] = old_delta_tower_angle_adj;

          // Calc Low parameters
          data.tower_angle_adj[A_AXIS] -= perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zLo = newPos[C_AXIS];

          // Reset Delta tower Alpha angle adj 
          data.tower_angle_adj[A_AXIS] = old_delta_tower_angle_adj;
          break;
        }

        case 5: {
          const float old_delta_tower_angle_adj = data.tower_angle_adj[B_AXIS];

          // Calc High parameters
          data.tower_angle_adj[B_AXIS] += perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zHi = newPos[C_AXIS];

          // Reset Delta tower Beta angle adj 
          data.tower_angle_adj[B_AXIS] = old_delta_tower_angle_adj;

          // Calc Low parameters
          data.tower_angle_adj[B_AXIS] -= perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zLo = newPos[C_AXIS];

          // Reset Delta tower Beta angle adj 
          data.tower_angle_adj[B_AXIS] = old_delta_tower_angle_adj;
          break;
        }

        case 6: {
          const float old_delta_diagonal_rod = data.diagonal_rod;

          // Calc High parameters
          data.diagonal_rod += perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zHi = newPos[C_AXIS];

          // Reset Delta Diagonal Rod
          data.diagonal_rod = old_delta_diagonal_rod;

          // Calc Low parameters
          data.diagonal_rod -= perturb;
          recalc_delta_settings();
          InverseTransform(ha, hb, hc, newPos);
          zLo = newPos[C_AXIS];

          // Reset Delta Diagonal Rod
          data.diagonal_rod = old_delta_diagonal_rod;
          break;
        }
      }

      recalc_delta_settings();

      return (zHi - zLo) / (2.0f * perturb);
    }

  #endif // ENABLED(DELTA_AUTO_CALIBRATION_1)

  #if DISABLED(DISABLE_M503)

    void Delta_Mechanics::print_parameters() {

      SERIAL_LM(CFG, "Steps per unit:");
      SERIAL_SMV(CFG, "  M92 X", LINEAR_UNIT(data.axis_steps_per_mm[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(data.axis_steps_per_mm[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(data.axis_steps_per_mm[Z_AXIS]), 3);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(data.axis_steps_per_mm[E_AXIS]), 3);
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M92 T", (int)e);
          SERIAL_EMV(" E", VOLUMETRIC_UNIT(data.axis_steps_per_mm[E_AXIS + e]), 3);
        }
      #endif // EXTRUDERS > 1

      SERIAL_LM(CFG, "Maximum feedrates (units/s):");
      SERIAL_SMV(CFG, "  M203 X", LINEAR_UNIT(data.max_feedrate_mm_s[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(data.max_feedrate_mm_s[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(data.max_feedrate_mm_s[Z_AXIS]), 3);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(data.max_feedrate_mm_s[E_AXIS]), 3);
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M203 T", (int)e);
          SERIAL_EMV(" E", VOLUMETRIC_UNIT(data.max_feedrate_mm_s[E_AXIS + e]), 3);
        }
      #endif // EXTRUDERS > 1

      SERIAL_LM(CFG, "Maximum Acceleration (units/s2):");
      SERIAL_SMV(CFG, "  M201 X", LINEAR_UNIT(data.max_acceleration_mm_per_s2[X_AXIS]));
      SERIAL_MV(" Y", LINEAR_UNIT(data.max_acceleration_mm_per_s2[Y_AXIS]));
      SERIAL_MV(" Z", LINEAR_UNIT(data.max_acceleration_mm_per_s2[Z_AXIS]));
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(data.max_acceleration_mm_per_s2[E_AXIS]));
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M201 T", (int)e);
          SERIAL_EMV(" E", VOLUMETRIC_UNIT(data.max_acceleration_mm_per_s2[E_AXIS + e]));
        }
      #endif // EXTRUDERS > 1

      SERIAL_LM(CFG, "Acceleration (units/s2): P<DEFAULT_ACCELERATION> V<DEFAULT_TRAVEL_ACCELERATION> T* R<DEFAULT_RETRACT_ACCELERATION>:");
      SERIAL_SMV(CFG,"  M204 P", LINEAR_UNIT(data.acceleration), 3);
      SERIAL_MV(" V", LINEAR_UNIT(data.travel_acceleration), 3);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 R", LINEAR_UNIT(data.retract_acceleration[0]), 3);
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M204 T", (int)e);
          SERIAL_EMV(" R", LINEAR_UNIT(data.retract_acceleration[e]), 3);
        }
      #endif

      SERIAL_LM(CFG, "Advanced: B<DEFAULT_MIN_SEGMENT_TIME> S<DEFAULT_MIN_FEEDRATE> V<DEFAULT_MIN_TRAVEL_FEEDRATE>");
      SERIAL_SMV(CFG, "  M205 B", data.min_segment_time_us);
      SERIAL_MV(" S", LINEAR_UNIT(data.min_feedrate_mm_s), 3);
      SERIAL_EMV(" V", LINEAR_UNIT(data.min_travel_feedrate_mm_s), 3);

      #if ENABLED(JUNCTION_DEVIATION)
        SERIAL_LM(CFG, "Junction Deviation: J<JUNCTION_DEVIATION_MM>");
        SERIAL_LMV(CFG, "  M205 J", data.junction_deviation_mm, 3);
      #endif

      SERIAL_SM(CFG, "Jerk: X<DEFAULT_XJERK>");
      #if DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
        SERIAL_MSG(" T* E<DEFAULT_EJERK>");
      #endif
      SERIAL_EOL();

      SERIAL_SMV(CFG, "  M205 X", LINEAR_UNIT(data.max_jerk[X_AXIS]), 3);

      #if DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
        #if EXTRUDERS == 1
          SERIAL_MV(" T0 E", LINEAR_UNIT(data.max_jerk[E_AXIS]), 3);
        #endif
        SERIAL_EOL();
        #if (EXTRUDERS > 1)
          LOOP_EXTRUDER() {
            SERIAL_SMV(CFG, "  M205 T", (int)e);
            SERIAL_EMV(" E" , LINEAR_UNIT(data.max_jerk[E_AXIS + e]), 3);
          }
        #endif
      #endif

      SERIAL_LM(CFG, "Endstop adjustment:");
      SERIAL_SM(CFG, "  M666");
      SERIAL_MV(" X", LINEAR_UNIT(data.endstop_adj[A_AXIS]));
      SERIAL_MV(" Y", LINEAR_UNIT(data.endstop_adj[B_AXIS]));
      SERIAL_MV(" Z", LINEAR_UNIT(data.endstop_adj[C_AXIS]));
      SERIAL_EOL();

      SERIAL_LM(CFG, "Delta Geometry adjustment: ABC<TOWER_*_DIAGROD_ADJ> IJK<TOWER_*_ANGLE_ADJ> UVW<TOWER_*_RADIUS_ADJ>");
      SERIAL_SM(CFG, "  M666");
      SERIAL_MV(" A", LINEAR_UNIT(data.diagonal_rod_adj[0]), 3);
      SERIAL_MV(" B", LINEAR_UNIT(data.diagonal_rod_adj[1]), 3);
      SERIAL_MV(" C", LINEAR_UNIT(data.diagonal_rod_adj[2]), 3);
      SERIAL_MV(" I", data.tower_angle_adj[0], 3);
      SERIAL_MV(" J", data.tower_angle_adj[1], 3);
      SERIAL_MV(" K", data.tower_angle_adj[2], 3);
      SERIAL_MV(" U", LINEAR_UNIT(data.tower_radius_adj[0]), 3);
      SERIAL_MV(" V", LINEAR_UNIT(data.tower_radius_adj[1]), 3);
      SERIAL_MV(" W", LINEAR_UNIT(data.tower_radius_adj[2]), 3);
      SERIAL_EOL();
      SERIAL_LM(CFG, "Delta Geometry adjustment: R<DELTA_RADIUS> D<DELTA_DIAGONAL_ROD> S<DELTA_SEGMENTS_PER_SECOND>");
      SERIAL_SM(CFG, "  M666");
      SERIAL_MV(" R", LINEAR_UNIT(data.radius));
      SERIAL_MV(" D", LINEAR_UNIT(data.diagonal_rod));
      SERIAL_MV(" S", data.segments_per_second);
      SERIAL_EOL();
      SERIAL_LM(CFG, "Delta Geometry adjustment: O<DELTA_PRINTABLE_RADIUS> P<DELTA_PROBEABLE_RADIUS> H<DELTA_HEIGHT>");
      SERIAL_SM(CFG, "  M666");
      SERIAL_MV(" O", LINEAR_UNIT(data.print_radius));
      SERIAL_MV(" P", LINEAR_UNIT(data.probe_radius));
      SERIAL_MV(" H", LINEAR_UNIT(data.height), 3);
      SERIAL_EOL();

    }

  #endif // DISABLED(DISABLE_M503)

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)

    void Delta_Mechanics::Nextion_gfx_clear() {
      gfx_clear(data.print_radius * 2, data.print_radius * 2, data.height);
      gfx_cursor_to(current_position[X_AXIS] + data.print_radius, current_position[Y_AXIS] + data.print_radius, current_position[Z_AXIS]);
    }

  #endif

  /** Private Function */
  void Delta_Mechanics::homeaxis(const AxisEnum axis) {

    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MV(">>> homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }

      if (printer.debugFeature()) SERIAL_EM("Home 1 Fast:");
    #endif

    // Fast move towards endstop until triggered
    do_homing_move(axis, 1.5 * data.height);

    // When homing Z with probe respect probe clearance
    const float bump = home_bump_mm[axis];

    // If a second homing move is configured...
    if (bump) {
      // Move away from the endstop by the axis HOME_BUMP_MM
      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM("Move Away:");
      #endif
      do_homing_move(axis, -bump);

      // Slow move towards endstop until triggered
      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM("Home 2 Slow:");
      #endif
      do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
    }

    // Delta has already moved all three towers up in G28
    // so here it re-homes each tower in turn.
    // Delta homing treats the axes as normal linear axes.

    // retrace by the amount specified in data.endstop_adj + additional 0.1mm in order to have minimum steps
    if (data.endstop_adj[axis] < 0) {
      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM("data.endstop_adj:");
      #endif
      do_homing_move(axis, data.endstop_adj[axis] - 0.1);
    }

    // Clear z_lift if homing the Z axis
    #if ENABLED(FWRETRACT)
      if (axis == Z_AXIS) fwretract.current_hop = 0.0;
    #endif

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void Delta_Mechanics::prepare_uninterpolated_move_to_destination(const float fr_mm_s/*=0.0*/) {
    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);
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

      planner.buffer_line(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), tools.active_extruder);
    #endif

    set_current_to_destination();
  }

  void Delta_Mechanics::Set_clip_start_height() {
    float cartesian[XYZ] = { 0, 0, 0 };
    Transform(cartesian);
    float distance = delta[A_AXIS];
    cartesian[Y_AXIS] = data.print_radius;
    Transform(cartesian);
    delta_clip_start_height = data.height - ABS(distance - delta[A_AXIS]);
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

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
    delta_radius              = DEFAULT_DELTA_RADIUS;
    diagonal_rod              = DELTA_DIAGONAL_ROD;
    segments_per_second       = DELTA_SEGMENTS_PER_SECOND;
    print_radius              = DELTA_PRINTABLE_RADIUS;
    probe_radius              = DELTA_PRINTABLE_RADIUS - max(abs(X_PROBE_OFFSET_FROM_NOZZLE), abs(Y_PROBE_OFFSET_FROM_NOZZLE));
    delta_height              = DELTA_HEIGHT;
    endstop_adj[A_AXIS]       = TOWER_A_ENDSTOP_ADJ;
    endstop_adj[B_AXIS]       = TOWER_B_ENDSTOP_ADJ;
    endstop_adj[C_AXIS]       = TOWER_C_ENDSTOP_ADJ;
    tower_radius_adj[A_AXIS]  = TOWER_A_RADIUS_ADJ;
    tower_radius_adj[B_AXIS]  = TOWER_B_RADIUS_ADJ;
    tower_radius_adj[C_AXIS]  = TOWER_C_RADIUS_ADJ;
    tower_pos_adj[A_AXIS]     = TOWER_A_POSITION_ADJ;
    tower_pos_adj[B_AXIS]     = TOWER_B_POSITION_ADJ;
    tower_pos_adj[C_AXIS]     = TOWER_C_POSITION_ADJ;
    diagonal_rod_adj[A_AXIS]  = TOWER_A_DIAGROD_ADJ;
    diagonal_rod_adj[B_AXIS]  = TOWER_B_DIAGROD_ADJ;
    diagonal_rod_adj[C_AXIS]  = TOWER_C_DIAGROD_ADJ;
    clip_start_height         = Z_MAX_POS;

    Recalc();
  }

  /**
   * Prepare a single move and get ready for the next one.
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
    if (destination[X_AXIS] == current_position[X_AXIS] && destination[Y_AXIS] == current_position[Y_AXIS]) {
      planner.buffer_line_kinematic(destination, _feedrate_mm_s, active_extruder, active_driver);
      set_current_to_destination();
      return;
    }

    // Fail if attempting move outside printable radius
    if (!position_is_reachable_xy(destination[X_AXIS], destination[Y_AXIS])) return;

    // Get the cartesian distances moved in XYZE
    float difference[NUM_AXIS];
    LOOP_XYZE(i) difference[i] = destination[i] - current_position[i];

    // Get the linear distance in XYZ
    float cartesian_mm = SQRT(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));

    // If the move is very short, check the E move distance
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = abs(difference[E_AXIS]);

    // No E move either? Game over.
    if (UNEAR_ZERO(cartesian_mm)) return;

    // Minimum number of seconds to move the given distance
    float seconds = cartesian_mm / _feedrate_mm_s;

    // The number of segments-per-second times the duration
    // gives the number of segments we should produce
    uint16_t segments = segments_per_second * seconds;

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
        if (planner.abl_enabled) {
          const float zadj = bilinear_z_offset(logical);
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
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder, active_driver);
  }

  /**
   * line_to_destination
   * Move the planner to the position stored in the destination array, which is
   * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
   */
  void Delta_Mechanics::line_to_destination(float fr_mm_s) {
    planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder, active_driver);
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
    if (current_position[Z_AXIS] > clip_start_height) {
      if (lz > clip_start_height) {   // staying in the danger zone
        destination[X_AXIS] = lx;           // move directly (uninterpolated)
        destination[Y_AXIS] = ly;
        destination[Z_AXIS] = lz;
        prepare_uninterpolated_move_to_destination(); // set_current_to_destination
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("danger zone move", current_position);
        #endif
        return;
      }
      else {
        destination[Z_AXIS] = clip_start_height;
        prepare_uninterpolated_move_to_destination(); // set_current_to_destination
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("zone border move", current_position);
        #endif
      }
    }

    if (lz > current_position[Z_AXIS]) {    // raising?
      destination[Z_AXIS] = lz;
      prepare_uninterpolated_move_to_destination();   // set_current_to_destination
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("z raise move", current_position);
      #endif
    }

    destination[X_AXIS] = lx;
    destination[Y_AXIS] = ly;
    prepare_move_to_destination();         // set_current_to_destination
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("xy move", current_position);
    #endif

    if (lz < current_position[Z_AXIS]) {    // lowering?
      destination[Z_AXIS] = lz;
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
    do_blocking_move_to(logical[X_AXIS], logical[Y_AXIS], logical[Z_AXIS], fr_mm_s);
  }
  void Delta_Mechanics::do_blocking_move_to_x(const float &lx, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(lx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
  }
  void Delta_Mechanics::do_blocking_move_to_z(const float &lz, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], lz, fr_mm_s);
  }
  void Delta_Mechanics::do_blocking_move_to_xy(const float &lx, const float &ly, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(lx, ly, current_position[Z_AXIS], fr_mm_s);
  }

  void Delta_Mechanics::sync_plan_position() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position_kinematic", current_position);
    #endif
    planner.set_position_mm_kinematic(current_position);
  }

  void Delta_Mechanics::sync_plan_position_e() {
    planner.set_e_position_mm(current_position[E_AXIS]);
  }

  /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void Delta_Mechanics::prepare_uninterpolated_move_to_destination(const float fr_mm_s/*=0.0*/) {
    if ( current_position[X_AXIS] == destination[X_AXIS]
      && current_position[Y_AXIS] == destination[Y_AXIS]
      && current_position[Z_AXIS] == destination[Z_AXIS]
      && current_position[E_AXIS] == destination[E_AXIS]
    ) return;

    refresh_cmd_timeout();
    planner.buffer_line_kinematic(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), active_extruder, active_driver);
    set_current_to_destination();
  }

  #if ENABLED(DELTA_AUTO_CALIBRATION_1)

    // Compute the derivative of height with respect to a parameter at the specified motor endpoints.
    // 'deriv' indicates the parameter as follows:
    // 0, 1, 2 = X, Y, Z tower endstop adjustments
    // 3 = delta radius
    // 4 = X tower correction
    // 5 = Y tower correction
    // 6 = diagonal_rod rod length
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
          hiParams.tower_radius_adj[A_AXIS] += perturb;
          loParams.tower_radius_adj[A_AXIS] -= perturb;
          break;

        case 5:
          hiParams.tower_radius_adj[B_AXIS] += perturb;
          loParams.tower_radius_adj[B_AXIS] -= perturb;
          break;

        case 6:
          hiParams.diagonal_rod += perturb;
          loParams.diagonal_rod -= perturb;
          break;
      }

      hiParams.Recalc();
      loParams.Recalc();

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

      const float oldHeightA = homed_Height + endstop_adj[A_AXIS];

      // Update endstop adjustments
      endstop_adj[A_AXIS] += v[0];
      endstop_adj[B_AXIS] += v[1];
      endstop_adj[C_AXIS] += v[2];
      NormaliseEndstopAdjustments();

      if (numFactors >= 4) {
        delta_radius += v[3];

        if (numFactors >= 6) {
          tower_radius_adj[A_AXIS] += v[4];
          tower_radius_adj[B_AXIS] += v[5];

          if (numFactors == 7) diagonal_rod += v[6];

        }
      }

      Recalc();
      const float heightError = homed_Height + endstop_adj[A_AXIS] - oldHeightA - v[0];
      delta_height -= heightError;
      homed_Height -= heightError;

    }

    // Convert endstop_adj
    void Delta_Mechanics::Convert_endstop_adj() {
      LOOP_XYZ(i) endstop_adj[i] *= -1;
    }

    // Normalize Endstop
    void Delta_Mechanics::NormaliseEndstopAdjustments() {
      const float min_endstop = MIN3(endstop_adj[A_AXIS], endstop_adj[B_AXIS], endstop_adj[C_AXIS]);
      LOOP_XYZ(i) endstop_adj[i] -= min_endstop;
      delta_height += min_endstop;
      homed_Height += min_endstop;
    }

  #endif // DELTA_AUTO_CALIBRATION_1

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

    cartesian[X_AXIS] = (U * z - S) / Q;
    cartesian[Y_AXIS] = (P - R * z) / Q;
    cartesian[Z_AXIS] = z;
  }

  void Delta_Mechanics::Recalc() {

    LOOP_XY(i) {
      endstops.soft_endstop_min[i] = -print_radius;
      endstops.soft_endstop_max[i] = print_radius;
    }
    endstops.soft_endstop_max[Z_AXIS]  = delta_height;
    probe_radius = print_radius - max(abs(X_PROBE_OFFSET_FROM_NOZZLE), abs(Y_PROBE_OFFSET_FROM_NOZZLE));

    delta_diagonal_rod_2[A_AXIS] = sq(diagonal_rod + diagonal_rod_adj[A_AXIS]);
    delta_diagonal_rod_2[B_AXIS] = sq(diagonal_rod + diagonal_rod_adj[B_AXIS]);
    delta_diagonal_rod_2[C_AXIS] = sq(diagonal_rod + diagonal_rod_adj[C_AXIS]);

    // Effective X/Y positions of the three vertical towers.
    towerX[A_AXIS] = -((delta_radius + tower_pos_adj[A_AXIS]) * cos(RADIANS(30 + tower_radius_adj[A_AXIS]))); // front left tower
    towerY[A_AXIS] = -((delta_radius + tower_pos_adj[A_AXIS]) * sin(RADIANS(30 + tower_radius_adj[A_AXIS]))); 
    towerX[B_AXIS] = +((delta_radius + tower_pos_adj[B_AXIS]) * cos(RADIANS(30 - tower_radius_adj[B_AXIS]))); // front right tower
    towerY[B_AXIS] = -((delta_radius + tower_pos_adj[B_AXIS]) * sin(RADIANS(30 - tower_radius_adj[B_AXIS]))); 
    towerX[C_AXIS] = -((delta_radius + tower_pos_adj[C_AXIS]) * sin(RADIANS(     tower_radius_adj[C_AXIS]))); // back middle tower
    towerY[C_AXIS] = +((delta_radius + tower_pos_adj[C_AXIS]) * cos(RADIANS(     tower_radius_adj[C_AXIS]))); 

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
    D2 = sq(diagonal_rod);

    const float tempHeight = diagonal_rod;		// any sensible height will do here, probably even zero
    float cartesian[ABC];
    InverseTransform(tempHeight, tempHeight, tempHeight, cartesian);
    homed_Height = delta_height + tempHeight - cartesian[Z_AXIS];
    printRadiusSquared = sq(print_radius);

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

    delta[A_AXIS] = logical[Z_AXIS] + _SQRT(delta_diagonal_rod_2[A_AXIS] - sq(logical[A_AXIS] - towerX[A_AXIS]) - sq(logical[B_AXIS] - towerY[A_AXIS]));
    delta[B_AXIS] = logical[Z_AXIS] + _SQRT(delta_diagonal_rod_2[B_AXIS] - sq(logical[A_AXIS] - towerX[B_AXIS]) - sq(logical[B_AXIS] - towerY[B_AXIS]));
    delta[C_AXIS] = logical[Z_AXIS] + _SQRT(delta_diagonal_rod_2[C_AXIS] - sq(logical[A_AXIS] - towerX[C_AXIS]) - sq(logical[B_AXIS] - towerY[C_AXIS]));

    /*
    SERIAL_MV("cartesian X:", logical[X_AXIS]);
    SERIAL_MV(" Y:", logical[Y_AXIS]);
    SERIAL_EMV(" Z:", logical[Z_AXIS]);
    SERIAL_MV("delta A:", delta[A_AXIS]);
    SERIAL_MV(" B:", delta[B_AXIS]);
    SERIAL_EMV(" C:", delta[C_AXIS]);
    */
  }

  void Delta_Mechanics::Set_clip_start_height() {
    float cartesian[XYZ] = { 0, 0, 0 };
    Transform(cartesian);
    float distance = delta[A_AXIS];
    cartesian[Y_AXIS] = print_radius;
    Transform(cartesian);
    clip_start_height = endstops.soft_endstop_max[Z_AXIS] - abs(distance - delta[A_AXIS]);
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

    planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    current_position[axis] = distance;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], active_extruder, active_driver);

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

    // retrace by the amount specified in endstop_adj + additional 0.1mm in order to have minimum steps
    if (endstop_adj[axis] < 0) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("endstop_adj:");
      #endif
      do_homing_move(axis, endstop_adj[axis] - 0.1);
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
    planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

    // Move all carriages together linearly until an endstop is hit.
    current_position[X_AXIS] = current_position[Y_AXIS] = current_position[Z_AXIS] = delta_height + 10;
    feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
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

    current_position[axis] = (axis == Z_AXIS ? delta_height : 0.0);

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
    const bool  xx = x && !axis_homed[X_AXIS],
                yy = y && !axis_homed[Y_AXIS],
                zz = z && !axis_homed[Z_AXIS];

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
      print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
    }

  #endif

#endif

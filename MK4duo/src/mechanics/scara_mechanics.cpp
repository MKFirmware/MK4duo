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

#include "../../base.h"
#include "scara_mechanics.h"

#if IS_SCARA

  Scara_Mechanics mechanics;

  void Scara_Mechanics::Init() {
    // TODO!!!
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
      planner.buffer_line_kinematic(destination, _feedrate_mm_s, active_extruder);
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
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = FABS(difference[E_AXIS]);

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
    NOLESS(segments, 1);

    // The approximate length of each segment
    const float inv_segments = 1.0 / float(segments),
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
      const float inv_segment_length = min(10.0, float(segments) / cartesian_mm), // 1/mm/segs
                  feed_factor = inv_segment_length * _feedrate_mm_s;
      float oldA = stepper.get_axis_position_degrees(A_AXIS),
            oldB = stepper.get_axis_position_degrees(B_AXIS);
    #endif

    // Get the logical current position as starting point
    float logical[XYZE];
    COPY(logical, current_position);

    // Drop one segment so the last move is to the exact target.
    // If there's only 1 segment, loops will be skipped entirely.
    --segments;

    // Calculate and execute the segments
    for (uint16_t s = segments + 1; --s;) {
      LOOP_XYZE(i) logical[i] += segment_distance[i];
      inverse_kinematics(logical);

      ADJUST_DELTA(logical); // Adjust Z if bed leveling is enabled

      #if ENABLED(SCARA_FEEDRATE_SCALING)
        // For SCARA scale the feed rate from mm/s to degrees/s
        // Use ratio between the length of the move and the larger angle change
        const float adiff = abs(delta[A_AXIS] - oldA),
                    bdiff = abs(delta[B_AXIS] - oldB);
        planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], max(adiff, bdiff) * feed_factor, active_extruder);
        oldA = delta[A_AXIS];
        oldB = delta[B_AXIS];
      #else
        planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], _feedrate_mm_s, active_extruder);
      #endif
    }

    // Since segment_distance is only approximate,
    // the final move must be to the exact destination.

    #if ENABLED(SCARA_FEEDRATE_SCALING)
      // For SCARA scale the feed rate from mm/s to degrees/s
      // With segments > 1 length is 1 segment, otherwise total length
      inverse_kinematics(ltarget);
      ADJUST_DELTA(ltarget);
      const float adiff = abs(delta[A_AXIS] - oldA),
                  bdiff = abs(delta[B_AXIS] - oldB);
      planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], max(adiff, bdiff) * feed_factor, active_extruder);
    #else
      planner.buffer_line_kinematic(ltarget, _feedrate_mm_s, active_extruder);
    #endif

    return false;
  }

  bool Scara_Mechanics::position_is_reachable_raw_xy(const float &rx, const float &ry) {
    #if MIDDLE_DEAD_ZONE_R > 0
      const float R2 = HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y);
      return R2 >= sq(float(MIDDLE_DEAD_ZONE_R)) && R2 <= sq(L1 + L2);
    #else
      return HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y) <= sq(L1 + L2);
    #endif
  }

  bool Scara_Mechanics::position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry) {
    // Both the nozzle and the probe must be able to reach the point.
    // This won't work on SCARA since the probe offset rotates with the arm.
    // TODO: fix this
    return position_is_reachable_raw_xy(rx, ry)
        && position_is_reachable_raw_xy(rx - X_PROBE_OFFSET_FROM_EXTRUDER, ry - Y_PROBE_OFFSET_FROM_EXTRUDER);
  }

 /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void Scara_Mechanics::prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);
    #endif

    commands.refresh_cmd_timeout();

    if ( current_position[X_AXIS] == destination[X_AXIS]
      && current_position[Y_AXIS] == destination[Y_AXIS]
      && current_position[Z_AXIS] == destination[Z_AXIS]
      && current_position[E_AXIS] == destination[E_AXIS]
    ) return;

    planner.buffer_line_kinematic(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), active_extruder);

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
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR(">>> set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHAR(')');
        SERIAL_EOL();
      }
    #endif

    axis_known_position[axis] = axis_homed[axis] = true;

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
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOLNPGM("*** Z HOMED WITH PROBE (Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN) ***");
              SERIAL_ECHOLNPAIR("> zprobe_zoffset = ", zprobe_zoffset);
            }
          #endif

        #elif ENABLED(DEBUG_LEVELING_FEATURE)

          if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("*** Z HOMED TO ENDSTOP (Z_MIN_PROBE_ENDSTOP) ***");

        #endif
      }
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
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
    if (printer.IsRunning()) {
      forward_kinematics_SCARA(delta_a, delta_b);
      destination[X_AXIS] = LOGICAL_X_POSITION(cartesian_position[X_AXIS]);
      destination[Y_AXIS] = LOGICAL_Y_POSITION(cartesian_position[Y_AXIS]);
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
  void Scara_Mechanics::inverse_kinematics(const float logical[XYZ]) {

    static float C2, S2, SK1, SK2, THETA, PSI;

    float sx = RAW_X_POSITION(logical[X_AXIS]) - SCARA_OFFSET_X,  // Translate SCARA to standard X Y
          sy = RAW_Y_POSITION(logical[Y_AXIS]) - SCARA_OFFSET_Y;  // With scaling factor.

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

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
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/simen/grbl/tree
 *
 * It has preliminary support for Matthew Roberts advance algorithm
 *  - http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "../base.h"

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  void gcode_M100();
  #if ENABLED(M100_FREE_MEMORY_DUMPER)
    void M100_dump_routine(const char * const title, const char *start, const char *end);
  #endif
#endif

#if ENABLED(G38_PROBE_TARGET)
  bool G38_move         = false,
       G38_endstop_hit  = false;
#endif

#if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
  bool flow_firstread = false;
#endif

#if ENABLED(CNCROUTER)
  uint8_t active_cnc_tool = 0;
  #define CNC_M6_TOOL_ID 255
#endif

#if HEATER_USES_AD595
  float ad595_offset[HOTENDS] = ARRAY_BY_HOTENDS(TEMP_SENSOR_AD595_OFFSET),
        ad595_gain[HOTENDS]   = ARRAY_BY_HOTENDS(TEMP_SENSOR_AD595_GAIN);
#endif

#if ENABLED(NPR2)
  uint8_t old_color = 99;
#endif

#if ENABLED(RFID_MODULE)
  unsigned long Spool_ID[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0);
  bool  RFID_ON = false,
        Spool_must_read[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(false),
        Spool_must_write[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if ENABLED(FWRETRACT)

  bool  autoretract_enabled           = false,
        retracted[EXTRUDERS]          = { false },
        retracted_swap[EXTRUDERS]     = { false };

  float retract_length                = RETRACT_LENGTH,
        retract_length_swap           = RETRACT_LENGTH_SWAP,
        retract_feedrate_mm_s         = RETRACT_FEEDRATE,
        retract_zlift                 = RETRACT_ZLIFT,
        retract_recover_length        = RETRACT_RECOVER_LENGTH,
        retract_recover_length_swap   = RETRACT_RECOVER_LENGTH_SWAP,
        retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE;

#endif // FWRETRACT

#if MECH(DELTA)

  #if ENABLED(DELTA_AUTO_CALIBRATION_3)

    float ac_prec,
          bed_level_c,
          bed_level_x,
          bed_level_y,
          bed_level_z,
          bed_level_ox,
          bed_level_oy,
          bed_level_oz,
          adj_t1_Radius = 0,
          adj_t2_Radius = 0,
          adj_t3_Radius = 0,
          adj_diagrod_length();

    int fix_tower_errors();
    bool adj_deltaradius();

    void  adj_tower_delta(uint8_t tower);
    void  adj_tower_radius(uint8_t tower);
    void  calibration_report();
    void  bed_probe_all();
    void  adj_endstops();

  #endif // DELTA_AUTO_CALIBRATION_3

#endif

#if IS_SCARA
  // Float constants for SCARA calculations
  const float L1 = SCARA_LINKAGE_1, L2 = SCARA_LINKAGE_2,
              L1_2 = sq(float(L1)), L1_2_2 = 2.0 * L1_2,
              L2_2 = sq(float(L2));

  float scara_segments_per_second = SCARA_SEGMENTS_PER_SECOND,
        delta[ABC];
  static void plan_direct_stepper_move(const float target[XYZE]);
#endif

#if MECH(MAKERARM_SCARA)

  #define LEFT_ARM false
  #define RIGHT_ARM true

  bool arm_orientation = LEFT_ARM;

  bool set_head_index = true;
  float head_offsets[4] = { 0.0 };
  int quadrant_limit = 1;

  float dest_fin[3] = { 0.0 },
        previous_extruder_pos = 0,
        final_extruder_pos,
        layer_height = 0.0;

  bool z_layer_height_check = true;

  float z_offset_for_eq = 0,
        z_for_bed_eq[3] = { 0 },
        z_points_qd1[3] = { 0 },
        z_points_qd2[3] = { 0 },
        read_z = 0.0;

  bool G92_called = false,
       offset_toggle_x = true,
       offset_toggle_y = true;

#endif

#if ENABLED(COLOR_MIXING_EXTRUDER)
  float mixing_factor[MIXING_STEPPERS]; // Reciprocal of mix proportion. 0.0 = off, otherwise >= 1.0
  #if MIXING_VIRTUAL_TOOLS  > 1
    float mixing_virtual_tool_mix[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];
  #endif
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  unsigned long axis_last_activity = 0;
  bool IDLE_OOZING_enabled = true;
  bool IDLE_OOZING_retracted[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  float power_consumption_meas = 0.0;
  unsigned long power_consumption_hour,
                startpower  = 0;
                stoppower   = 0;
#endif

#if ENABLED(NPR2)
  static float  color_position[] = COLOR_STEP,
                color_step_moltiplicator = (DRIVER_MICROSTEP / MOTOR_ANGLE) * CARTER_MOLTIPLICATOR;
#endif // NPR2

#if ENABLED(EASY_LOAD)
  bool allow_lengthy_extrude_once; // for load/unload
#endif

#if HAS_CHDK
  millis_t chdkHigh = 0;
  bool chdkActive = false;
#endif

#if ENABLED(PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
  int lpq_len = 20;
#endif

#if ENABLED(CNC_WORKSPACE_PLANES)
  static WorkspacePlane workspace_plane = PLANE_XY;
#endif

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

#if HAS_BED_PROBE

  #if MECH(MAKERARM_SCARA)

    /**
     * Get the arm-end position based on the probe position
     * If the position is unreachable return vector_3 0,0,0
     */
    vector_3 probe_point_to_end_point(const float &x, const float &y) {

      // Simply can't reach the given point
      if (HYPOT2(x, y) > sq(L1 + L2 + Y_PROBE_OFFSET_FROM_NOZZLE))
        return vector_3();

      float pos[XYZ] = { x, y, 0 };

      // Get the angles for placing the probe at x, y
      inverse_kinematics(pos, Y_PROBE_OFFSET_FROM_NOZZLE);

      // Get the arm-end XY based on the given angles
      forward_kinematics_SCARA(mechanics.delta[A_AXIS], mechanics.delta[B_AXIS]);
      float tx = LOGICAL_X_POSITION(mechanics.cartesian_position[X_AXIS]),
            ty = LOGICAL_Y_POSITION(mechanics.cartesian_position[Y_AXIS]);

      return vector_3(tx, ty, 0);
    }

    /**
     * Get the probe position based on the arm-end position
     * If the position is unreachable return vector_3 0,0,0
     */
    vector_3 end_point_to_probe_point(const float logical[XYZ]) {

      // Simply can't reach the given point
      if (HYPOT2(logical[X_AXIS], logical[Y_AXIS]) > sq(L1 + L2))
        return vector_3();

      // Get the angles for placing the arm-end at x, y
      inverse_kinematics(logical);

      // Get the probe XY based on the sum of the angles
      float ab = RADIANS(mechanics.delta[A_AXIS] + mechanics.delta[B_AXIS] + 90.0);
      return vector_3(
        logical[X_AXIS] + sin(ab) * X_PROBE_OFFSET_FROM_NOZZLE,
        logical[Y_AXIS] - cos(ab) * Y_PROBE_OFFSET_FROM_NOZZLE,
        0
      );
    }

  #endif // MAKERARM_SCARA

#endif // HAS_BED_PROBE

#if ENABLED(FLOWMETER_SENSOR)

  void print_flowratestate() {
    float readval = get_flowrate();

    #if ENABLED(MINFLOW_PROTECTION)
      if(readval > MINFLOW_PROTECTION)
        flow_firstread = true;
    #endif

    SERIAL_MV(" FLOW: ", readval);
    SERIAL_MSG(" l/min ");
  }

#endif

#if ENABLED(CNCROUTER) && ENABLED(FAST_PWM_CNCROUTER)

  void print_cncspeed() {
    SERIAL_MV(" CNC speed: ", getCNCSpeed());
    SERIAL_MSG(" rpm ");
  }

#endif

#if HAS_DONDOLO

  inline void move_extruder_servo(const uint8_t e) {
    const int angles[2] = { DONDOLO_SERVOPOS_E0, DONDOLO_SERVOPOS_E1 };
    MOVE_SERVO(DONDOLO_SERVO_INDEX, angles[e]);

    #if (DONDOLO_SERVO_DELAY > 0)
      printer.safe_delay(DONDOLO_SERVO_DELAY);
    #endif
  }

#endif

/**
 * Function for DELTA
 */
#if MECH(DELTA)

  #if ENABLED(DELTA_AUTO_CALIBRATION_3)

    void bed_probe_all() {
      // Initial throwaway probe.. used to stabilize probe
      bed_level_c = probe.check_pt(0.0, 0.0);

      // Probe all bed positions & store carriage positions
      bed_level_z = probe.check_pt(0.0, mechanics.delta_probe_radius);
      bed_level_oy = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
      bed_level_x = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
      bed_level_oz = probe.check_pt(0.0, -mechanics.delta_probe_radius);
      bed_level_y = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
      bed_level_ox = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
      bed_level_c = probe.check_pt(0.0, 0.0);
    }

    void apply_endstop_adjustment(const float x_endstop, const float y_endstop, const float z_endstop) {
      mechanics.delta_endstop_adj[X_AXIS] += x_endstop;
      mechanics.delta_endstop_adj[Y_AXIS] += y_endstop;
      mechanics.delta_endstop_adj[Z_AXIS] += z_endstop;

      mechanics.Transform(mechanics.current_position);
      mechanics.set_position_mm(mechanics.delta[A_AXIS] - x_endstop , mechanics.delta[B_AXIS] - y_endstop, mechanics.delta[C_AXIS] - z_endstop, mechanics.current_position[E_AXIS]);
      stepper.synchronize();
    }

    void adj_endstops() {
      bool x_done = false;
      bool y_done = false;
      bool z_done = false;

      do {
        bed_level_z = probe.check_pt(0.0, mechanics.delta_probe_radius);
        bed_level_x = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
        bed_level_y = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);

        apply_endstop_adjustment(bed_level_x, bed_level_y, bed_level_z);

        SERIAL_MV("x:", bed_level_x, 4);
        SERIAL_MV(" (adj:", mechanics.delta_endstop_adj[0], 4);
        SERIAL_MV(") y:", bed_level_y, 4);
        SERIAL_MV(" (adj:", mechanics.delta_endstop_adj[1], 4);
        SERIAL_MV(") z:", bed_level_z, 4);
        SERIAL_MV(" (adj:", mechanics.delta_endstop_adj[2], 4);
        SERIAL_CHR(')'); SERIAL_EOL();

        if (FABS(bed_level_x) <= ac_prec) {
          x_done = true;
          SERIAL_MSG("X=OK ");
        }
        else {
          x_done = false;
          SERIAL_MSG("X=ERROR ");
        }

        if (FABS(bed_level_y) <= ac_prec) {
          y_done = true;
          SERIAL_MSG("Y=OK ");
        }
        else {
          y_done = false;
          SERIAL_MSG("Y=ERROR ");
        }

        if (FABS(bed_level_z) <= ac_prec) {
          z_done = true;
          SERIAL_EM("Z=OK");
        }
        else {
          z_done = false;
          SERIAL_EM("Z=ERROR");
        }
      } while (((x_done == false) or (y_done == false) or (z_done == false)));

      const float high_endstop = MAX3(mechanics.delta_endstop_adj[A_AXIS], mechanics.delta_endstop_adj[B_AXIS], mechanics.delta_endstop_adj[C_AXIS]);

      SERIAL_EMV("High endstop:", high_endstop, 4);

      if (high_endstop > 0) {
        SERIAL_EMV("Reducing Build height by ", high_endstop);
        LOOP_XYZ(i) mechanics.delta_endstop_adj[i] -= high_endstop;
        mechanics.delta_height -= high_endstop;
      }

      mechanics.recalc_delta_settings();
    }

    int fix_tower_errors() {
      bool t1_err, t2_err, t3_err,
              xy_equal, xz_equal, yz_equal;
      float saved_tower_radius_adj[ABC],
            high_diff,
            x_diff, y_diff, z_diff,
            low_opp, high_opp;
      uint8_t err_tower = 0;

      COPY_ARRAY(saved_tower_radius_adj, mechanics.delta_tower_radius_adj);

      x_diff = FABS(bed_level_x - bed_level_ox);
      y_diff = FABS(bed_level_y - bed_level_oy);
      z_diff = FABS(bed_level_z - bed_level_oz);
      high_diff = MAX3(x_diff, y_diff, z_diff);

      if (x_diff <= ac_prec) t1_err = false; else t1_err = true;
      if (y_diff <= ac_prec) t2_err = false; else t2_err = true;
      if (z_diff <= ac_prec) t3_err = false; else t3_err = true;

      SERIAL_MV("x_diff:", x_diff, 5);
      SERIAL_MV(" y_diff:", y_diff, 5);
      SERIAL_MV(" z_diff:", z_diff, 5);
      SERIAL_EMV(" high_diff:", high_diff, 5);

      // Are all errors equal? (within defined precision)
      xy_equal = false;
      xz_equal = false;
      yz_equal = false;
      if (FABS(x_diff - y_diff) <= ac_prec) xy_equal = true;
      if (FABS(x_diff - z_diff) <= ac_prec) xz_equal = true;
      if (FABS(y_diff - z_diff) <= ac_prec) yz_equal = true;

      SERIAL_MSG("xy_equal = ");
      if (xy_equal == true) SERIAL_EM("true"); else SERIAL_EM("false");
      SERIAL_MSG("xz_equal = ");
      if (xz_equal == true) SERIAL_EM("true"); else SERIAL_EM("false");
      SERIAL_MSG("yz_equal = ");
      if (yz_equal == true) SERIAL_EM("true"); else SERIAL_EM("false");

      low_opp   = MIN3(bed_level_ox, bed_level_oy, bed_level_oz);
      high_opp  = MAX3(bed_level_ox, bed_level_oy, bed_level_oz);

      SERIAL_EMV("Opp Range = ", high_opp - low_opp, 5);

      if (high_opp - low_opp  < ac_prec) {
        SERIAL_EM("Opposite Points within Limits - Adjustment not required");
        t1_err = false;
        t2_err = false;
        t3_err = false;
      }

      // All Towers have errors
      if ((t1_err == true) and (t2_err == true) and (t3_err == true)) {
        if ((xy_equal == false) or (xz_equal == false) or (yz_equal == false)) {
          // Errors not equal .. select the tower that needs to be adjusted
          if (high_diff == x_diff) err_tower = 1;
          if (high_diff == y_diff) err_tower = 2;
          if (high_diff == z_diff) err_tower = 3;
          SERIAL_MV("Tower ", err_tower);
          SERIAL_EM(" has largest error");
        }
        if ((xy_equal == true) and (xz_equal == true) and (yz_equal == true)) {
          SERIAL_EM("All Towers Errors Equal");
          t1_err = false;
          t2_err = false;
          t3_err = false;
        }
      }

      /*
      // Two tower errors
      if ((t1_err == true) and (t2_err == true) and (t3_err == false)) {
        if (high_diff == x_diff) err_tower = 1;
        else err_tower = 2;
      }
      else if ((t1_err == true) and (t2_err == false) and (t3_err == true)) {
        if (high_diff == x_diff) err_tower = 1;
        else err_tower = 3;
      }
      else if ((t1_err == false) and (t2_err == true) and (t3_err == true)) {
        if (high_diff == y_diff) err_tower = 2;
        else err_tower = 3;
      }
      */

      // Single tower error
      if ((t1_err == true) and (t2_err == false) and (t3_err == false)) err_tower = 1;
      if ((t1_err == false) and (t2_err == true) and (t3_err == false)) err_tower = 2;
      if ((t1_err == false) and (t2_err == false) and (t3_err == true)) err_tower = 3;

      SERIAL_MSG("t1:");
      if (t1_err == true) SERIAL_MSG("Err"); else SERIAL_MSG("OK");
      SERIAL_MSG(" t2:");
      if (t2_err == true) SERIAL_MSG("Err"); else SERIAL_MSG("OK");
      SERIAL_MSG(" t3:");
      if (t3_err == true) SERIAL_MSG("Err"); else SERIAL_MSG("OK");
      SERIAL_EOL();

      if (err_tower == 0)
        SERIAL_EM("Tower geometry OK");
      else {
        SERIAL_MV("Tower", int(err_tower));
        SERIAL_EM(" Error: Adjusting");
        adj_tower_radius(err_tower);
      }

      // Set return value to indicate if anything has been changed (0 = no change)
      int retval = 0;
      LOOP_XYZ(i) if (saved_tower_radius_adj[i] != mechanics.delta_tower_radius_adj[i]) retval++;
      return retval;
    }

    bool adj_deltaradius() {
      bool adj_done;
      int adj_attempts;
      float adj_dRadius, adjdone_vector;

      bed_level_c = probe.check_pt(0.0, 0.0);

      if (FABS(bed_level_c) <= ac_prec) {
        SERIAL_EM("Delta Radius OK");
        return false;
      }
      else {
        SERIAL_EM("Adjusting Delta Radius");
        SERIAL_EMV("Bed level center = ", bed_level_c);

        // set initial direction and magnitude for delta radius adjustment
        adj_attempts = 0;
        adj_dRadius = 0;
        adjdone_vector = 0.01;

        do {
          mechanics.delta_radius += adj_dRadius;
          mechanics.recalc_delta_settings();
          adj_done = false;

          adj_endstops();
          bed_level_c = probe.check_pt(0.0, 0.0);

          // Set inital adjustment value if it is currently 0
          if (adj_dRadius == 0) {
            if (bed_level_c > 0) adj_dRadius = -0.2;
            if (bed_level_c < 0) adj_dRadius = 0.2;
          }

          // Adjustment complete?
          if (FABS(bed_level_c) <= ac_prec) {
            //Done to within acprec .. but done within adjdone_vector?
            if (FABS(bed_level_c) <= adjdone_vector)
              adj_done = true;
            else {
              adj_attempts ++;
              if (adj_attempts > 3) {
                adjdone_vector += 0.01;
                adj_attempts = 0;
              }
            }
          }

          // Show progress
          SERIAL_MV(" c:", bed_level_c, 4);
          SERIAL_MV(" delta radius:", mechanics.delta_radius, 4);
          SERIAL_MV(" prec:", adjdone_vector, 3);
          SERIAL_MV(" tries:", adj_attempts);
          SERIAL_MSG(" done:");
          if (adj_done == true) SERIAL_EM("true");
          else SERIAL_EM("false");

          // Overshot target? .. reverse and scale down adjustment
          if (((bed_level_c < 0) and (adj_dRadius < 0)) or ((bed_level_c > 0) and (adj_dRadius > 0))) adj_dRadius = -(adj_dRadius / 2);

        } while (adj_done == false);

        return true;
      }
    }

    void adj_tower_radius(uint8_t tower) {
      bool adj_done;
      float adj_tRadius = 0.0,
            bed_level   = 0.0,
            bed_level_o = 0.0;

      do {
        mechanics.delta_tower_radius_adj[tower - 1] += adj_tRadius;
        mechanics.recalc_delta_settings();
        adj_done = false;

        if (tower == 1) {
          // Bedlevel_x
          bed_level = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
          // Bedlevel_ox
          bed_level_o = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        }
        if (tower == 2) {
          // Bedlevel_y
          bed_level = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
          // Bedlevel_oy
          bed_level_o = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        }
        if (tower == 3) {
          // Bedlevel_z
          bed_level = probe.check_pt(0.0, mechanics.delta_probe_radius);
          // Bedlevel_oz
          bed_level_o = probe.check_pt(0.0, -mechanics.delta_probe_radius);
        }

        // Set inital adjustment value if it is currently 0
        if (adj_tRadius == 0) {
          if (bed_level_o < bed_level) adj_tRadius = -1;
          if (bed_level_o > bed_level) adj_tRadius = 1;
        }

        // Overshot target? .. reverse and scale down adjustment
        if (((bed_level_o < bed_level) and (adj_tRadius > 0)) or ((bed_level_o > bed_level) and (adj_tRadius < 0))) adj_tRadius = -(adj_tRadius / 2);

        // Adjustment complete?
        if (FABS(bed_level_o) < bed_level + 0.015) adj_done = true;

        // Show progress
        SERIAL_MV("tower:", bed_level, 4);
        SERIAL_MV(" opptower:", bed_level_o, 4);
        SERIAL_MV(" tower radius adj:", mechanics.delta_tower_radius_adj[tower - 1], 4);
        SERIAL_MSG(" done:");
        if (adj_done == true) SERIAL_EM("true");
        else SERIAL_EM("false");

        if (adj_done == false) adj_endstops();

      } while (adj_done == false);
    }

    void adj_tower_delta(uint8_t tower) {
      float adj_val = 0;
      float adj_mag = 0.2;
      float adj_prv;

      do {
        mechanics.delta_tower_pos_adj[tower - 1] += adj_val;
        mechanics.recalc_delta_settings();

        if ((tower == 1) or (tower == 3)) bed_level_oy = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        if ((tower == 1) or (tower == 2)) bed_level_oz = probe.check_pt(0.0, -mechanics.delta_probe_radius);
        if ((tower == 2) or (tower == 3)) bed_level_ox = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);

        adj_prv = adj_val;
        adj_val = 0;

        if (tower == 1) {
          if (bed_level_oy < bed_level_oz) adj_val = adj_mag;
          if (bed_level_oy > bed_level_oz) adj_val = -adj_mag;
        }

        if (tower == 2) {
          if (bed_level_oz < bed_level_ox) adj_val = adj_mag;
          if (bed_level_oz > bed_level_ox) adj_val = -adj_mag;
        }

        if (tower == 3) {
          if (bed_level_ox < bed_level_oy) adj_val = adj_mag;
          if (bed_level_ox > bed_level_oy) adj_val = -adj_mag;
        }

        if ((adj_val > 0) and (adj_prv < 0)) {
          adj_mag = adj_mag / 2;
          adj_val = adj_mag;
        }

        if ((adj_val < 0) and (adj_prv > 0)) {
          adj_mag = adj_mag / 2;
          adj_val = -adj_mag;
        }

        // Show Adjustments made
        if (tower == 1) {
          SERIAL_MV("oy:", bed_level_oy, 4);
          SERIAL_MV(" oz:", bed_level_oz, 4);
        }

        if (tower == 2) {
          SERIAL_MV("ox:", bed_level_ox, 4);
          SERIAL_MV(" oz:", bed_level_oz, 4);
        }

        if (tower == 3) {
          SERIAL_MV("ox:", bed_level_ox, 4);
          SERIAL_MV(" oy:", bed_level_oy, 4);
        }

        SERIAL_EMV(" tower delta adj:", adj_val, 5);
      } while(adj_val != 0);
    }

    float adj_diagrod_length() {
      float adj_val = 0;
      float adj_mag = 0.2;
      float adj_prv, target;
      float prev_diag_rod = mechanics.delta_diagonal_rod;

      do {
        mechanics.delta_diagonal_rod += adj_val;
        mechanics.recalc_delta_settings();

        bed_level_oy = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        bed_level_oz = probe.check_pt(0.0, -mechanics.delta_probe_radius);
        bed_level_ox = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        bed_level_c = probe.check_pt(0.0, 0.0);

        target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;
        adj_prv = adj_val;
        adj_val = 0;

        if (bed_level_c - 0.01 < target) adj_val = -adj_mag;
        if (bed_level_c + 0.01 > target) adj_val = adj_mag;

        if (((adj_val > 0) and (adj_prv < 0)) or ((adj_val < 0) and (adj_prv > 0))) {
          adj_val = adj_val / 2;
          adj_mag = adj_mag / 2;
        }

        if ((bed_level_c - 0.01 < target) and (bed_level_c + 0.01 > target)) adj_val = 0;

        // If adj magnatude is very small.. quit adjusting
        if ((abs(adj_val) < 0.001) and (adj_val != 0)) adj_val = 0;

        SERIAL_MV("target:", target, 4);
        SERIAL_MV(" c:", bed_level_c, 4);
        SERIAL_EMV(" adj:", adj_val, 5);
      } while(adj_val != 0);

      return (mechanics.delta_diagonal_rod - prev_diag_rod);
    }

    void calibration_report() {
      // Display Report
      SERIAL_EM("| \tZ-Tower\t\t\tEndstop Offsets");

      SERIAL_MSG("| \t");
      if (bed_level_z >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_z, 4);
      SERIAL_MV("\t\t\tX:", mechanics.delta_endstop_adj[0], 4);
      SERIAL_MV(" Y:", mechanics.delta_endstop_adj[1], 4);
      SERIAL_EMV(" Z:", mechanics.delta_endstop_adj[2], 4);

      SERIAL_MSG("| ");
      if (bed_level_ox >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_ox, 4);
      SERIAL_MSG("\t");
      if (bed_level_oy >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_oy, 4);
      SERIAL_EM("\t\tTower Offsets");

      SERIAL_MSG("| \t");
      if (bed_level_c >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_c, 4);
      SERIAL_MV("\t\t\tA:", mechanics.delta_tower_radius_adj[0]);
      SERIAL_MV(" B:", mechanics.delta_tower_radius_adj[1]);
      SERIAL_EMV(" C:", mechanics.delta_tower_radius_adj[2]);

      SERIAL_MSG("| ");
      if (bed_level_x >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_x, 4);
      SERIAL_MSG("\t");
      if (bed_level_y >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_y, 4);
      SERIAL_MV("\t\tI:", mechanics.delta_tower_pos_adj[0]);
      SERIAL_MV(" J:", mechanics.delta_tower_pos_adj[1]);
      SERIAL_EMV(" K:", mechanics.delta_tower_pos_adj[2]);

      SERIAL_MSG("| \t");
      if (bed_level_oz >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_oz, 4);
      SERIAL_EMV("\t\t\tDelta Radius: ", mechanics.delta_radius, 4);

      SERIAL_EMV("| X-Tower\tY-Tower\t\tDiagonal Rod: ", mechanics.delta_diagonal_rod, 4);
      SERIAL_EOL();
    }

  #endif

#endif // DELTA

#if MECH(MORGAN_SCARA)

  /**
   * Morgan SCARA Forward mechanics. Results in cartesian_position[].
   * Maths and first version by QHARLEY.
   * Integrated and slightly restructured by Joachim Cerny.
   */
  void forward_kinematics_SCARA(const float &a, const float &b) {

    const float a_sin = sin(RADIANS(a)) * L1,
                a_cos = cos(RADIANS(a)) * L1,
                b_sin = sin(RADIANS(b)) * L2,
                b_cos = cos(RADIANS(b)) * L2;

    cartesian_position[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  //theta
    cartesian_position[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  //theta+phi

      //SERIAL_MV(" cartesian_position[X_AXIS]=", cartesian_position[X_AXIS]);
      //SERIAL_EMV(" cartesian_position[Y_AXIS]=", cartesian_position[Y_AXIS]);
  }

  /**
   * Morgan SCARA Inverse mechanics. Results in delta[].
   *
   * See http://forums.reprap.org/read.php?185,283327
   *
   * Maths and first version by QHARLEY.
   * Integrated and slightly restructured by Joachim Cerny.
   */
  void inverse_kinematics(const float logical[XYZ]) {

    static float C2, S2, SK1, SK2, THETA, PSI;

    const float sx = RAW_X_POSITION(logical[X_AXIS]) - SCARA_offset_x,  // Translate SCARA to standard X Y
                sy = RAW_Y_POSITION(logical[Y_AXIS]) - SCARA_offset_y;  // With scaling factor.

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
    SERIAL_MV("  SCARA (x,y) ", sx);
    SERIAL_MV(",", sy);
    SERIAL_MV(" C2=", C2);
    SERIAL_MV(" S2=", S2);
    SERIAL_MV(" Theta=", THETA);
    SERIAL_EMV(" Psi=", PSI);
    */
  }

#endif // MORGAN_SCARA

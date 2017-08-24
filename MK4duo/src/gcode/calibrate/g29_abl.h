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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

void out_of_range_error(const char* p_edge) {
  SERIAL_MSG("?Probe ");
  SERIAL_PS(p_edge);
  SERIAL_EM(" position out of range.");
}

#if HAS_ABL

  #define CODE_G29

  #if ABL_GRID
    #if ENABLED(PROBE_Y_FIRST)
      #define PR_OUTER_VAR xCount
      #define PR_OUTER_END abl_grid_points_x
      #define PR_INNER_VAR yCount
      #define PR_INNER_END abl_grid_points_y
    #else
      #define PR_OUTER_VAR yCount
      #define PR_OUTER_END abl_grid_points_y
      #define PR_INNER_VAR xCount
      #define PR_INNER_END abl_grid_points_x
    #endif
  #endif

  /**
   * G29: Detailed Z-Probe, probes the bed at 3 or more points.
   *      Will fail if the printer has not been homed with G28.
   *
   * Enhanced G29 Auto Bed Levelling Probe Routine
   *
   *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply
   *     or alter the bed level data. Useful to check the topology
   *     after a first run of G29.
   *
   *  J  Jettison current bed leveling data
   *
   *  V  Set the verbose level (0-4). Example: "G29 V3"
   *
   * Parameters With LINEAR leveling only:
   *
   *  P  Set the size of the grid that will be probed (P x P points).
   *     Example: "G29 P4"
   *
   *  X  Set the X size of the grid that will be probed (X x Y points).
   *     Example: "G29 X7 Y5"
   *
   *  Y  Set the Y size of the grid that will be probed (X x Y points).
   *
   *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.
   *     This is useful for manual bed leveling and finding flaws in the bed (to
   *     assist with part placement).
   *     Not supported by non-linear delta printer bed leveling.
   *
   * Parameters With LINEAR and BILINEAR leveling only:
   *
   *  S  Set the XY travel speed between probe points (in units/min)
   *
   *  F  Set the Front limit of the probing grid
   *  B  Set the Back limit of the probing grid
   *  L  Set the Left limit of the probing grid
   *  R  Set the Right limit of the probing grid
   *
   * Parameters with DEBUG_LEVELING_FEATURE only:
   *
   *  C  Make a totally fake grid with no actual probing.
   *     For use in testing when no probing is possible.
   *
   * Parameters with BILINEAR leveling only:
   *
   *  Z  Supply an additional Z probe offset
   *
   * Extra parameters with PROBE_MANUALLY:
   *
   *  To do manual probing simply repeat G29 until the procedure is complete.
   *  The first G29 accepts parameters. 'G29 Q' for status, 'G29 A' to abort.
   *
   *  Q  Query leveling and G29 state
   *
   *  A  Abort current leveling procedure
   *
   *  W  Write a mesh point. (Ignored during leveling.)
   *  X  Required X for mesh point
   *  Y  Required Y for mesh point
   *  Z  Required Z for mesh point
   *
   * Without PROBE_MANUALLY:
   *
   *  E  By default G29 will engage the Z probe, test the bed, then disengage.
   *     Include "E" to engage/disengage the Z probe for each sample.
   *     There's no extra effect if you have a fixed Z probe.
   *
   */
  inline void gcode_G29(void) {

    // G29 Q is also available if debugging
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      const bool query = parser.seen('Q');
      const uint8_t old_debug_flags = commands.mk_debug_flags;
      if (query) commands.mk_debug_flags |= DEBUG_LEVELING;
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS(">>> gcode_G29", mechanics.current_position);
        mechanics.log_machine_info();
      }
      commands.mk_debug_flags = old_debug_flags;
      #if DISABLED(PROBE_MANUALLY)
        if (query) return;
      #endif
    #endif

    #if ENABLED(PROBE_MANUALLY)
      const bool seenA = parser.seen('A'), seenQ = parser.seen('Q'), no_action = seenA || seenQ;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE) && DISABLED(PROBE_MANUALLY)
      const bool faux = parser.boolval('C');
    #elif ENABLED(PROBE_MANUALLY)
      const bool faux = no_action;
    #else
      bool constexpr faux = false;
    #endif

    #if MECH(DELTA)
      if (!bedlevel.g29_in_progress) {
        // Homing
        mechanics.Home(true);
        mechanics.do_blocking_move_to_z(_Z_PROBE_DEPLOY_HEIGHT, mechanics.homing_feedrate_mm_s[Z_AXIS]);
      }
    #else
      // Don't allow auto-levelling without homing first
      if (mechanics.axis_unhomed_error()) return;
    #endif

    // Define local vars 'static' for manual probing, 'auto' otherwise
    #if ENABLED(PROBE_MANUALLY)
      #define ABL_VAR static
    #else
      #define ABL_VAR
    #endif

    ABL_VAR int verbose_level;
    ABL_VAR float xProbe, yProbe, measured_z;
    ABL_VAR bool dryrun, abl_should_enable;

    #if ENABLED(PROBE_MANUALLY) || ENABLED(AUTO_BED_LEVELING_LINEAR)
      ABL_VAR int abl_probe_index;
    #endif

    #if HAS_SOFTWARE_ENDSTOPS && ENABLED(PROBE_MANUALLY)
      ABL_VAR bool enable_soft_endstops = true;
    #endif

    #if ABL_GRID

      #if ENABLED(PROBE_MANUALLY)
        ABL_VAR uint8_t PR_OUTER_VAR;
        ABL_VAR  int8_t PR_INNER_VAR;
      #endif

      ABL_VAR int left_probe_bed_position,
                  right_probe_bed_position,
                  front_probe_bed_position,
                  back_probe_bed_position;

      ABL_VAR float xGridSpacing = 0.0,
                    yGridSpacing = 0.0;

      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        ABL_VAR uint8_t abl_grid_points_x = GRID_MAX_POINTS_X,
                        abl_grid_points_y = GRID_MAX_POINTS_Y;
        ABL_VAR bool    do_topography_map;
      #else // Bilinear
        uint8_t constexpr abl_grid_points_x = GRID_MAX_POINTS_X,
                          abl_grid_points_y = GRID_MAX_POINTS_Y;
      #endif

      #if ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(PROBE_MANUALLY)
        #if ENABLED(AUTO_BED_LEVELING_LINEAR)
          ABL_VAR int abl2;
        #else // Bilinear
          int constexpr abl2 = GRID_MAX_POINTS;
        #endif
      #endif

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        ABL_VAR float zoffset;

      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

        ABL_VAR int indexIntoAB[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];

        ABL_VAR float eqnAMatrix[GRID_MAX_POINTS * 3], // "A" matrix of the linear system of equations
                      eqnBVector[GRID_MAX_POINTS],     // "B" vector of Z points
                      mean;
      #endif

    #elif ENABLED(AUTO_BED_LEVELING_3POINT)

      int constexpr abl2 = 3;

      // Probe at 3 arbitrary points
      ABL_VAR vector_3 points[3] = {
        vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, 0),
        vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, 0),
        vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, 0)
      };

    #endif // AUTO_BED_LEVELING_3POINT

    #if ENABLED(AUTO_BED_LEVELING_LINEAR)
      struct linear_fit_data lsf_results;
      incremental_LSF_reset(&lsf_results);
    #endif

    /**
     * On the initial G29 fetch command parameters.
     */
    if (!bedlevel.g29_in_progress) {

      #if ENABLED(PROBE_MANUALLY) || ENABLED(AUTO_BED_LEVELING_LINEAR)
        abl_probe_index = -1;
      #endif

      abl_should_enable = bedlevel.leveling_is_active();

      #if HAS_NEXTION_MANUAL_BED
        LcdBedLevelOn();
      #endif

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        if (parser.seen('W')) {
          if (!bedlevel.leveling_is_valid()) {
            SERIAL_LM(ER, "No bilinear grid");
            return;
          }

          const float z = parser.floatval('Z', RAW_CURRENT_POSITION(Z));
          if (!WITHIN(z, -10, 10)) {
            SERIAL_LM(ER, "Bad Z value");
            return;
          }

          const float x = parser.floatval('X', NAN),
                      y = parser.floatval('Y', NAN);
          int8_t      i = parser.byteval('I', -1),
                      j = parser.byteval('J', -1);

          if (!isnan(x) && !isnan(y)) {
            // Get nearest i / j from x / y
            i = (x - LOGICAL_X_POSITION(bedlevel.bilinear_start[X_AXIS]) + 0.5 * xGridSpacing) / xGridSpacing;
            j = (y - LOGICAL_Y_POSITION(bedlevel.bilinear_start[Y_AXIS]) + 0.5 * yGridSpacing) / yGridSpacing;
            i = constrain(i, 0, GRID_MAX_POINTS_X - 1);
            j = constrain(j, 0, GRID_MAX_POINTS_Y - 1);
          }
          if (WITHIN(i, 0, GRID_MAX_POINTS_X - 1) && WITHIN(j, 0, GRID_MAX_POINTS_Y)) {
            bedlevel.set_bed_leveling_enabled(false);
            bedlevel.z_values[i][j] = z;
            #if ENABLED(ABL_BILINEAR_SUBDIVISION)
              bedlevel.bed_level_virt_interpolate();
            #endif
            bedlevel.set_bed_leveling_enabled(abl_should_enable);
          }
          return;
        } // parser.seen('W')

      #endif

      #if HAS_LEVELING

        // Jettison bed leveling data
        if (parser.seen('J')) {
          bedlevel.reset_bed_level();
          return;
        }

      #endif

      verbose_level = parser.intval('V');
      if (!WITHIN(verbose_level, 0, 4)) {
        SERIAL_EM("?(V)erbose Level is implausible (0-4).");
        return;
      }

      dryrun = parser.boolval('D')
        #if ENABLED(PROBE_MANUALLY)
          || no_action
        #endif
      ;

      #if ENABLED(AUTO_BED_LEVELING_LINEAR)

        do_topography_map = verbose_level > 2 || parser.boolval('T');

        // X and Y specify points in each direction, overriding the default
        // These values may be saved with the completed mesh
        abl_grid_points_x = parser.intval('X', GRID_MAX_POINTS_X);
        abl_grid_points_y = parser.intval('Y', GRID_MAX_POINTS_Y);
        if (parser.seenval('P')) abl_grid_points_x = abl_grid_points_y = parser.value_int();

        if (abl_grid_points_x < 2 || abl_grid_points_y < 2) {
          SERIAL_EM("?Number of probe points is implausible (2 minimum).");
          return;
        }

        abl2 = abl_grid_points_x * abl_grid_points_y;

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        zoffset = parser.linearval('Z');

      #endif

      #if ABL_GRID

        bedlevel.xy_probe_feedrate_mm_s = MMM_TO_MMS(parser.seen('S') ? parser.value_linear_units() : XY_PROBE_SPEED);

        left_probe_bed_position   = (int)parser.linearval('L', LOGICAL_X_POSITION(LEFT_PROBE_BED_POSITION));
        right_probe_bed_position  = (int)parser.linearval('R', LOGICAL_X_POSITION(RIGHT_PROBE_BED_POSITION));
        front_probe_bed_position  = (int)parser.linearval('F', LOGICAL_Y_POSITION(FRONT_PROBE_BED_POSITION));
        back_probe_bed_position   = (int)parser.linearval('B', LOGICAL_Y_POSITION(BACK_PROBE_BED_POSITION));

        const bool left_out_l   = left_probe_bed_position < LOGICAL_X_POSITION(MIN_PROBE_X),
                   left_out     = left_out_l || left_probe_bed_position > right_probe_bed_position - (MIN_PROBE_EDGE),
                   right_out_r  = right_probe_bed_position > LOGICAL_X_POSITION(MAX_PROBE_X),
                   right_out    = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
                   front_out_f  = front_probe_bed_position < LOGICAL_Y_POSITION(MIN_PROBE_Y),
                   front_out    = front_out_f || front_probe_bed_position > back_probe_bed_position - (MIN_PROBE_EDGE),
                   back_out_b   = back_probe_bed_position > LOGICAL_Y_POSITION(MAX_PROBE_Y),
                   back_out     = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;

        if (left_out || right_out || front_out || back_out) {
          if (left_out) {
            out_of_range_error(PSTR("(L)eft"));
            left_probe_bed_position = left_out_l ? LOGICAL_X_POSITION(MIN_PROBE_X) : right_probe_bed_position - (MIN_PROBE_EDGE);
          }
          if (right_out) {
            out_of_range_error(PSTR("(R)ight"));
            right_probe_bed_position = right_out_r ? LOGICAL_Y_POSITION(MAX_PROBE_X) : left_probe_bed_position + MIN_PROBE_EDGE;
          }
          if (front_out) {
            out_of_range_error(PSTR("(F)ront"));
            front_probe_bed_position = front_out_f ? LOGICAL_Y_POSITION(MIN_PROBE_Y) : back_probe_bed_position - (MIN_PROBE_EDGE);
          }
          if (back_out) {
            out_of_range_error(PSTR("(B)ack"));
            back_probe_bed_position = back_out_b ? LOGICAL_Y_POSITION(MAX_PROBE_Y) : front_probe_bed_position + MIN_PROBE_EDGE;
          }
          return;
        }

        // probe at the points of a lattice grid
        xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (abl_grid_points_x - 1);
        yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (abl_grid_points_y - 1);

      #endif // ABL_GRID

      if (verbose_level > 0) {
        SERIAL_EM("G29 Auto Bed Leveling");
        if (dryrun) SERIAL_EM("Running in DRY-RUN mode");
      }

      stepper.synchronize();

      // Disable auto bed leveling during G29
      bedlevel.abl_enabled = false;

      if (!dryrun) {
        // Re-orient the current position without leveling
        // based on where the steppers are positioned.
        mechanics.set_current_from_steppers_for_axis(ALL_AXES);

        // Sync the planner to where the steppers stopped
        mechanics.sync_plan_position();
      }

      #if HAS_BED_PROBE
        // Deploy the probe. Probe will raise if needed.
        if (probe.set_deployed(true)) {
          bedlevel.abl_enabled = abl_should_enable;
          return;
        }
      #endif

      if (!faux) printer.setup_for_endstop_or_probe_move();

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        if ( xGridSpacing != bedlevel.bilinear_grid_spacing[X_AXIS]
          || yGridSpacing != bedlevel.bilinear_grid_spacing[Y_AXIS]
          || left_probe_bed_position != LOGICAL_X_POSITION(bedlevel.bilinear_start[X_AXIS])
          || front_probe_bed_position != LOGICAL_Y_POSITION(bedlevel.bilinear_start[Y_AXIS])
        ) {
          if (dryrun) {
            // Before reset bed level, re-enable to correct the position
            bedlevel.abl_enabled = abl_should_enable;
          }
          // Reset grid to 0.0 or "not probed". (Also disables ABL)
          bedlevel.reset_bed_level();

          // Initialize a grid with the given dimensions
          bedlevel.bilinear_grid_spacing[X_AXIS] = xGridSpacing;
          bedlevel.bilinear_grid_spacing[Y_AXIS] = yGridSpacing;
          bedlevel.bilinear_start[X_AXIS] = RAW_X_POSITION(left_probe_bed_position);
          bedlevel.bilinear_start[Y_AXIS] = RAW_Y_POSITION(front_probe_bed_position);

          // Can't re-enable (on error) until the new grid is written
          abl_should_enable = false;
        }

      #endif // AUTO_BED_LEVELING_BILINEAR

      #if ENABLED(AUTO_BED_LEVELING_3POINT)

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_EM("> 3-point Leveling");
        #endif

        // Probe at 3 arbitrary points
        points[0].z = points[1].z = points[2].z = 0;

      #endif // AUTO_BED_LEVELING_3POINT

    } // !bedlevel.g29_in_progress

    #if ENABLED(PROBE_MANUALLY)

      // For manual probing, get the next index to probe now.
      // On the first probe this will be incremented to 0.
      if (!no_action) {
        ++abl_probe_index;
        bedlevel.g29_in_progress = true;
      }

      // Abort current G29 procedure, go back to ABLStart
      if (seenA && bedlevel.g29_in_progress) {
        SERIAL_EM("Manual G29 aborted");
        #if HAS_SOFTWARE_ENDSTOPS
          endstops.soft_endstops_enabled = enable_soft_endstops;
        #endif
        bedlevel.abl_enabled = abl_should_enable;
        bedlevel.g29_in_progress = false;
        #if ENABLED(LCD_BED_LEVELING) && ENABLED(ULTRA_LCD)
          lcd_wait_for_move = false;
        #endif
      }

      // Query G29 status
      if (verbose_level || seenQ) {
        SERIAL_MSG("Manual G29 ");
        if (bedlevel.g29_in_progress) {
          SERIAL_MV("point ", min(abl_probe_index + 1, abl2));
          SERIAL_EMV(" of ", abl2);
        }
        else
          SERIAL_EM("idle");
      }

      if (no_action) return;

      if (abl_probe_index == 0) {
        // For the initial G29 save software endstop state
        #if HAS_SOFTWARE_ENDSTOPS
          enable_soft_endstops = endstops.soft_endstops_enabled;
        #endif
      }
      else {
        // For G29 after adjusting Z.
        // Save the previous Z before going to the next point
        measured_z = mechanics.current_position[Z_AXIS];

        #if ENABLED(AUTO_BED_LEVELING_LINEAR)

          mean += measured_z;
          eqnBVector[abl_probe_index] = measured_z;
          eqnAMatrix[abl_probe_index + 0 * abl2] = xProbe;
          eqnAMatrix[abl_probe_index + 1 * abl2] = yProbe;
          eqnAMatrix[abl_probe_index + 2 * abl2] = 1;

          incremental_LSF(&lsf_results, xProbe, yProbe, measured_z);

        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

          bedlevel.z_values[xCount][yCount] = measured_z + zoffset;

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_MV("Save X", xCount);
              SERIAL_MV(" Y", yCount);
              SERIAL_EMV(" Z", bedlevel.z_values[xCount][yCount]);
            }
          #endif

        #elif ENABLED(AUTO_BED_LEVELING_3POINT)

          points[abl_probe_index].z = measured_z;

        #endif
      }

      //
      // If there's another point to sample, move there with optional lift.
      //

      #if ABL_GRID

        // Skip any unreachable points
        while (abl_probe_index < abl2) {

          // Set xCount, yCount based on abl_probe_index, with zig-zag
          PR_OUTER_VAR = abl_probe_index / PR_INNER_END;
          PR_INNER_VAR = abl_probe_index - (PR_OUTER_VAR * PR_INNER_END);

          // Probe in reverse order for every other row/column
          bool zig = (PR_OUTER_VAR & 1); // != ((PR_OUTER_END) & 1);

          if (zig) PR_INNER_VAR = (PR_INNER_END - 1) - PR_INNER_VAR;

          const float xBase = xCount * xGridSpacing + left_probe_bed_position,
                      yBase = yCount * yGridSpacing + front_probe_bed_position;

          xProbe = FLOOR(xBase + (xBase < 0 ? 0 : 0.5));
          yProbe = FLOOR(yBase + (yBase < 0 ? 0 : 0.5));

          #if ENABLED(AUTO_BED_LEVELING_LINEAR)
            indexIntoAB[xCount][yCount] = abl_probe_index;
          #endif

          // Keep looping till a reachable point is found
          if (mechanics.position_is_reachable_xy(xProbe, yProbe)) break;
          ++abl_probe_index;
        }

        // Is there a next point to move to?
        if (abl_probe_index < abl2) {
          mechanics.manual_goto_xy(xProbe, yProbe); // Can be used here too!
          #if HAS_SOFTWARE_ENDSTOPS
            // Disable software endstops to allow manual adjustment
            // If G29 is not completed, they will not be re-enabled
            endstops.soft_endstops_enabled = false;
          #endif
          return;
        }
        else {

          // Leveling done! Fall through to G29 finishing code below

          SERIAL_EM("Grid probing done.");

          // Re-enable software endstops, if needed
          #if HAS_SOFTWARE_ENDSTOPS
            endstops.soft_endstops_enabled = enable_soft_endstops;
          #endif
        }

      #elif ENABLED(AUTO_BED_LEVELING_3POINT)

        // Probe at 3 arbitrary points
        if (abl_probe_index < 3) {
          xProbe = LOGICAL_X_POSITION(points[abl_probe_index].x);
          yProbe = LOGICAL_Y_POSITION(points[abl_probe_index].y);
          #if HAS_SOFTWARE_ENDSTOPS
            // Disable software endstops to allow manual adjustment
            // If G29 is not completed, they will not be re-enabled
            endstops.soft_endstops_enabled = false;
          #endif
          return;
        }
        else {

          SERIAL_EM("3-point probing done.");
          bedlevel.g29_in_progress = false;

          // Re-enable software endstops, if needed
          #if HAS_SOFTWARE_ENDSTOPS
            endstops.soft_endstops_enabled = enable_soft_endstops;
          #endif

          if (!dryrun) {
            vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
            if (planeNormal.z < 0) {
              planeNormal.x *= -1;
              planeNormal.y *= -1;
              planeNormal.z *= -1;
            }
            bedlevel.matrix = matrix_3x3::create_look_at(planeNormal);

            // Can't re-enable (on error) until the new grid is written
            abl_should_enable = false;
          }

        }

      #endif // AUTO_BED_LEVELING_3POINT

    #else // !PROBE_MANUALLY
      {
        const bool stow_probe_after_each = parser.seen('E');

        #if ABL_GRID

          bool zig = PR_OUTER_END & 1;  // Always end at RIGHT and BACK_PROBE_BED_POSITION

          for (uint8_t PR_OUTER_VAR = 0; PR_OUTER_VAR < PR_OUTER_END && !isnan(measured_z); PR_OUTER_VAR++) {

            int8_t inStart, inStop, inInc;

            if (zig) {
              inStart = 0;
              inStop = PR_INNER_END;
              inInc = 1;
            }
            else {
              inStart = PR_INNER_END - 1;
              inStop = -1;
              inInc = -1;
            }

            zig ^= true; // zag

            // Inner loop is Y with PROBE_Y_FIRST enabled
            for (int8_t PR_INNER_VAR = inStart; PR_INNER_VAR != inStop; PR_INNER_VAR += inInc) {

              float xBase = left_probe_bed_position + xGridSpacing * xCount,
                    yBase = front_probe_bed_position + yGridSpacing * yCount;

              xProbe = FLOOR(xBase + (xBase < 0 ? 0 : 0.5));
              yProbe = FLOOR(yBase + (yBase < 0 ? 0 : 0.5));

              #if ENABLED(AUTO_BED_LEVELING_LINEAR)
                indexIntoAB[xCount][yCount] = ++abl_probe_index; // 0...
              #endif

              #if IS_KINEMATIC
                // Avoid probing outside the round or hexagonal area
                if (!mechanics.position_is_reachable_by_probe_xy(xProbe, yProbe)) continue;
              #endif

              measured_z = faux ? 0.001 * random(-100, 101) : probe.check_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);

              if (isnan(measured_z)) {
                bedlevel.abl_enabled = abl_should_enable;
                break;
              }

              #if ENABLED(AUTO_BED_LEVELING_LINEAR)

                mean += measured_z;
                eqnBVector[abl_probe_index] = measured_z;
                eqnAMatrix[abl_probe_index + 0 * abl2] = xProbe;
                eqnAMatrix[abl_probe_index + 1 * abl2] = yProbe;
                eqnAMatrix[abl_probe_index + 2 * abl2] = 1;

                incremental_LSF(&lsf_results, xProbe, yProbe, measured_z);

              #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

                bedlevel.z_values[xCount][yCount] = measured_z + zoffset;

              #endif

              abl_should_enable = false;
              printer.idle();

            } // inner
          } // outer

        #elif ENABLED(AUTO_BED_LEVELING_3POINT)

          // Probe at 3 arbitrary points

          for (uint8_t i = 0; i < 3; ++i) {
            // Retain the last probe position
            xProbe = LOGICAL_X_POSITION(points[i].x);
            yProbe = LOGICAL_Y_POSITION(points[i].y);
            measured_z = faux ? 0.001 * random(-100, 101) : probe.check_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);
            if (isnan(measured_z)) {
              bedlevel.abl_enabled = abl_should_enable;
              break;
            }
            points[i].z = measured_z;
          }

          if (!dryrun&& !isnan(measured_z)) {
            vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
            if (planeNormal.z < 0) {
              planeNormal.x *= -1;
              planeNormal.y *= -1;
              planeNormal.z *= -1;
            }
            bedlevel.matrix = matrix_3x3::create_look_at(planeNormal);

            // Can't re-enable (on error) until the new grid is written
            abl_should_enable = false;
          }

        #endif // AUTO_BED_LEVELING_3POINT

        // Raise to _Z_PROBE_DEPLOY_HEIGHT. Stow the probe.
        if (probe.set_deployed(false)) {
          bedlevel.abl_enabled = abl_should_enable;
          measured_z = NAN;
        }
      }
    #endif // !PROBE_MANUALLY

    //
    // G29 Finishing Code
    //
    // Unless this is a dry run, auto bed leveling will
    // definitely be enabled after this point.
    //
    // If code above wants to continue leveling, it should
    // return or loop before this point.
    //

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> probing complete", mechanics.current_position);
    #endif

    #if ENABLED(PROBE_MANUALLY)
      bedlevel.g29_in_progress = false;
      #if ENABLED(LCD_BED_LEVELING) && ENABLED(ULTRA_LCD)
        lcd_wait_for_move = false;
      #endif
    #endif

    // Calculate leveling, print reports, correct the position
    if (!isnan(measured_z)) {

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        if (!dryrun) bedlevel.extrapolate_unprobed_bed_level();
        bedlevel.print_bilinear_leveling_grid();

        bedlevel.refresh_bed_level();

        #if ENABLED(ABL_BILINEAR_SUBDIVISION)
          bedlevel.print_bilinear_leveling_grid_virt();
        #endif

      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

        // For LINEAR leveling calculate matrix, print reports, correct the position

        /**
         * solve the plane equation ax + by + d = z
         * A is the matrix with rows [x y 1] for all the probed points
         * B is the vector of the Z positions
         * the normal vector to the plane is formed by the coefficients of the
         * plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
         * so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z
         */
        float plane_equation_coefficients[3];

        finish_incremental_LSF(&lsf_results);
        plane_equation_coefficients[0] = -lsf_results.A;  // We should be able to eliminate the '-' on these three lines and down below
        plane_equation_coefficients[1] = -lsf_results.B;  // but that is not yet tested.
        plane_equation_coefficients[2] = -lsf_results.D;

        mean /= abl2;

        if (verbose_level) {
          SERIAL_MV("Eqn coefficients: a: ", plane_equation_coefficients[0], 8);
          SERIAL_MV(" b: ", plane_equation_coefficients[1], 8);
          SERIAL_EMV(" d: ", plane_equation_coefficients[2], 8);
          if (verbose_level > 2)
            SERIAL_EMV("Mean of sampled points: ", mean, 8);
        }

        // Create the matrix but don't correct the position yet
        if (!dryrun) {
          bedlevel.matrix = matrix_3x3::create_look_at(
            vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1) // We can eleminate the '-' here and up above
          );
        }

        // Show the Topography map if enabled
        if (do_topography_map) {

          SERIAL_EM(" Bed Height Topography:");
          SERIAL_EM("   +--- BACK --+");
          SERIAL_EM("   |           |");
          SERIAL_EM(" L |    (+)    | R");
          SERIAL_EM(" E |           | I");
          SERIAL_EM(" F | (-) N (+) | G");
          SERIAL_EM(" T |           | H");
          SERIAL_EM("   |    (-)    | T");
          SERIAL_EM("   |           |");
          SERIAL_EM("   O-- FRONT --+");
          SERIAL_EM(" (0,0)");

          float min_diff = 999;

          for (int8_t yy = abl_grid_points_y - 1; yy >= 0; yy--) {
            for (uint8_t xx = 0; xx < abl_grid_points_x; xx++) {
              int ind = indexIntoAB[xx][yy];
              float diff = eqnBVector[ind] - mean,
                    x_tmp = eqnAMatrix[ind + 0 * abl2],
                    y_tmp = eqnAMatrix[ind + 1 * abl2],
                    z_tmp = 0;

              apply_rotation_xyz(bedlevel.matrix, x_tmp, y_tmp, z_tmp);

              NOMORE(min_diff, eqnBVector[ind] - z_tmp);

              if (diff >= 0.0)
                SERIAL_MSG(" +");   // Include + for column alignment
              else
                SERIAL_CHR(' ');
              SERIAL_VAL(diff, 5);
            } // xx
            SERIAL_EOL();
          } // yy
          SERIAL_EOL();

          if (verbose_level > 3) {
            SERIAL_EM("\nCorrected Bed Height vs. Bed Topology:");

            for (int8_t yy = abl_grid_points_y - 1; yy >= 0; yy--) {
              for (uint8_t xx = 0; xx < abl_grid_points_x; xx++) {
                int ind = indexIntoAB[xx][yy];
                float x_tmp = eqnAMatrix[ind + 0 * abl2],
                      y_tmp = eqnAMatrix[ind + 1 * abl2],
                      z_tmp = 0;

                apply_rotation_xyz(bedlevel.matrix, x_tmp, y_tmp, z_tmp);

                float diff = eqnBVector[ind] - z_tmp - min_diff;
                if (diff >= 0.0)
                  SERIAL_MSG(" +");   // Include + for column alignment
                else
                  SERIAL_CHR(' ');
                SERIAL_VAL(diff, 5);
              } // xx
              SERIAL_EOL();
            } // yy
            SERIAL_EOL();
          }
        } // do_topography_map

      #endif // AUTO_BED_LEVELING_LINEAR_GRID

      #if ABL_PLANAR

        // For LINEAR and 3POINT leveling correct the current position

        if (verbose_level > 0)
          bedlevel.matrix.debug("\n\nBed Level Correction Matrix:");

        if (!dryrun) {
          //
          // Correct the current XYZ position based on the tilted plane.
          //

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("G29 uncorrected XYZ", mechanics.current_position);
          #endif

          float converted[XYZ];
          COPY_ARRAY(converted, mechanics.current_position);

          bedlevel.abl_enabled = true;
          bedlevel.unapply_leveling(converted); // use conversion machinery
          bedlevel.abl_enabled = false;

          // Use the last measured distance to the bed, if possible
          if ( NEAR(mechanics.current_position[X_AXIS], xProbe - (probe.offset[X_AXIS]))
            && NEAR(mechanics.current_position[Y_AXIS], yProbe - (probe.offset[Y_AXIS]))
          ) {
            float simple_z = mechanics.current_position[Z_AXIS] - measured_z;
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                SERIAL_MV("Z from Probe:", simple_z);
                SERIAL_MV("  Matrix:", converted[Z_AXIS]);
                SERIAL_EMV("  Discrepancy:", simple_z - converted[Z_AXIS]);
              }
            #endif
            converted[Z_AXIS] = simple_z;
          }

          // The rotated XY and corrected Z are now current_position
          COPY_ARRAY(mechanics.current_position, converted);

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("G29 corrected XYZ", mechanics.current_position);
          #endif
        }

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        if (!dryrun) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) SERIAL_EMV("G29 uncorrected Z:", mechanics.current_position[Z_AXIS]);
          #endif

          // Unapply the offset because it is going to be immediately applied
          // and cause compensation movement in Z. (Just like bedlevel.unapply_leveling)
          mechanics.current_position[Z_AXIS] -= bedlevel.bilinear_z_offset(mechanics.current_position);

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) SERIAL_EMV(" corrected Z:", mechanics.current_position[Z_AXIS]);
          #endif
        }

      #endif // ABL_PLANAR

      #if ENABLED(Z_PROBE_END_SCRIPT)
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_MSG("Z Probe End Script: ");
            SERIAL_EM(Z_PROBE_END_SCRIPT);
          }
        #endif
        commands.enqueue_and_echo_commands_P(PSTR(Z_PROBE_END_SCRIPT));
        stepper.synchronize();
      #endif

      #if HAS_NEXTION_MANUAL_BED
        LcdBedLevelOff();
      #endif

      // Auto Bed Leveling is complete! Enable if possible.
      bedlevel.abl_enabled = dryrun ? abl_should_enable : true;
    } // !isnan(measured_z)

    // Restore state after probing
    if (!faux) printer.clean_up_after_endstop_or_probe_move();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G29");
    #endif

    mechanics.report_current_position();

    KEEPALIVE_STATE(IN_HANDLER);

    if (bedlevel.abl_enabled)
      mechanics.sync_plan_position();
  }

#endif // HAS_ABL

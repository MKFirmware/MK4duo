/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if OLD_ABL

#define CODE_G29

#if ABL_GRID
  #if ENABLED(PROBE_Y_FIRST)
    #define PR_OUTER_VAR meshCount.x
    #define PR_OUTER_END abl_grid_points.x
    #define PR_INNER_VAR meshCount.y
    #define PR_INNER_END abl_grid_points.y
  #else
    #define PR_OUTER_VAR meshCount.y
    #define PR_OUTER_END abl_grid_points.y
    #define PR_INNER_VAR meshCount.x
    #define PR_INNER_END abl_grid_points.x
  #endif
#endif

/**
 * G29: Detailed Z probe, probes the bed at 3 or more points.
 *      Will fail if the printer has not been homed with G28.
 *
 * Enhanced G29 Auto Bed Leveling Probe Routine
 *
 *  O  Auto-level only if needed
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
 * Extra parameters with BILINEAR only:
 *
 *  W  Write a mesh point. (If G29 is idle.)
 *  I  X index for mesh point
 *  J  Y index for mesh point
 *  X  X for mesh point, overrides I
 *  Y  Y for mesh point, overrides J
 *  Z  Z for mesh point. Otherwise, raw current Z.
 *
 * Without PROBE_MANUALLY:
 *
 *  E  By default G29 will engage the Z probe, test the bed, then disengage.
 *     Include "E" to engage/disengage the Z probe for each sample.
 *     There's no extra effect if you have a fixed Z probe.
 *
 */
inline void gcode_G29() {

  // G29 Q is also available if debugging
  const bool seenQ = parser.seen('Q');
  if (seenQ) {
    DEBUG_POS(">>> G29", mechanics.position);
    DEBUG_LOG_INFO();
  }

  #if DISABLED(PROBE_MANUALLY)
    if (seenQ) return;
  #endif

  #if HAS_PROBE_MANUALLY
    const bool seenA = parser.seen('A');
  #else
    constexpr bool seenA = false;
  #endif

  const bool no_action = seenA || seenQ;

  #if DISABLED(PROBE_MANUALLY)
    const bool faux = parser.boolval('C');
  #else
    const bool faux = no_action;
  #endif

  #if MECH(DELTA)
    if (!bedlevel.flag.g29_in_progress) {
      // Homing
      mechanics.home();
      mechanics.do_blocking_move_to_z(_Z_PROBE_DEPLOY_HEIGHT, mechanics.homing_feedrate_mm_s.z);
    }
  #else
    // Don't allow auto-leveling without homing first
    if (mechanics.axis_unhomed_error()) return;
  #endif

  // Auto-level only if needed
  if (!no_action && bedlevel.flag.leveling_active && parser.boolval('O')) {
    if (printer.debugFeature()) DEBUG_EM("> Auto-level not needed, skip\n<<< G29");
    return;
  }

  // Define local vars 'static' for manual probing, 'auto' otherwise
  #if HAS_PROBE_MANUALLY
    #define ABL_VAR static
  #else
    #define ABL_VAR
  #endif

  ABL_VAR int       verbose_level = 0;
  ABL_VAR xy_pos_t  probePos;
  ABL_VAR float     measured_z = 0;
  ABL_VAR bool      dryrun = false;

  #if HAS_PROBE_MANUALLY || ENABLED(AUTO_BED_LEVELING_LINEAR)
    ABL_VAR int abl_probe_index = 0;
  #endif

  #if HAS_SOFTWARE_ENDSTOPS && HAS_PROBE_MANUALLY
    ABL_VAR bool saved_soft_endstops_state = true;
  #endif

  #if ABL_GRID

    #if HAS_PROBE_MANUALLY
      ABL_VAR xy_int8_t meshCount;
    #endif

    ABL_VAR xy_float_t  probe_position_lf,
                        probe_position_rb,
                        gridSpacing = { 0, 0 };

    #if ENABLED(AUTO_BED_LEVELING_LINEAR)
      ABL_VAR bool        do_topography_map;
      ABL_VAR xy_uint8_t  abl_grid_points;
    #else // Bilinear
      constexpr xy_uint8_t abl_grid_points = { GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y };
    #endif

    #if ENABLED(AUTO_BED_LEVELING_LINEAR)
      ABL_VAR int abl_points = 0;
    #elif HAS_PROBE_MANUALLY
      constexpr int abl_points = GRID_MAX_POINTS;
    #endif

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

      ABL_VAR float zoffset = 0.0;

    #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

      ABL_VAR int indexIntoAB[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];

      ABL_VAR float eqnAMatrix[GRID_MAX_POINTS * 3], // "A" matrix of the linear system of equations
                    eqnBVector[GRID_MAX_POINTS],     // "B" vector of Z points
                    mean = 0.0;
    #endif

  #elif ENABLED(AUTO_BED_LEVELING_3POINT)

    #if HAS_PROBE_MANUALLY
      constexpr int abl_points = 3; // used to show total points
    #endif

    // Probe at 3 arbitrary points
    const float x_min = probe.min_x(),
                x_max = probe.max_x(),
                y_min = probe.min_y(),
                y_max = probe.max_y();

    ABL_VAR vector_3 points[3] = {
      { x_min, y_min, 0 },
      { x_max, y_min, 0 },
      { (x_max - x_min) / 2, y_max, 0 }
    };

  #endif // AUTO_BED_LEVELING_3POINT

  #if ENABLED(AUTO_BED_LEVELING_LINEAR)
    struct linear_fit_data lsf_results;
    incremental_LSF_reset(&lsf_results);
  #endif

  /**
   * On the initial G29 fetch command parameters.
   */
  if (!bedlevel.flag.g29_in_progress) {

    #if HOTENDS > 1
      if (toolManager.extruder.active != 0) toolManager.change(0);
    #endif

    #if HAS_PROBE_MANUALLY || ENABLED(AUTO_BED_LEVELING_LINEAR)
      abl_probe_index = -1;
    #endif

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

      const bool seen_w = parser.seen('W');
      if (seen_w) {
        if (!bedlevel.leveling_is_valid()) {
          SERIAL_LM(ER, "No bilinear grid");
          return;
        }

        const float rz = parser.seenval('Z') ? NATIVE_Z_POSITION(parser.value_linear_units()) : mechanics.position.z;
        if (!WITHIN(rz, -10, 10)) {
          SERIAL_LM(ER, "Bad Z value");
          return;
        }

        const float rx  = NATIVE_X_POSITION(parser.linearval('X', NAN)),
                    ry  = NATIVE_Y_POSITION(parser.linearval('Y', NAN));
        int8_t      i   = parser.byteval('I', -1),
                    j   = parser.byteval('J', -1);

        if (!isnan(rx) && !isnan(ry)) {
          // Get nearest i / j from x / y
          i = (rx - abl.data.bilinear_start.x + 0.5 * gridSpacing.x) / gridSpacing.x;
          j = (ry - abl.data.bilinear_start.y + 0.5 * gridSpacing.y) / gridSpacing.y;
          LIMIT(i, 0, GRID_MAX_POINTS_X - 1);
          LIMIT(j, 0, GRID_MAX_POINTS_Y - 1);
        }
        if (WITHIN(i, 0, GRID_MAX_POINTS_X - 1) && WITHIN(j, 0, GRID_MAX_POINTS_Y)) {
          bedlevel.set_bed_leveling_enabled(false);
          abl.data.z_values[i][j] = rz;
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            abl.virt_interpolate();
          #endif
          bedlevel.restore_bed_leveling_state();
          mechanics.report_position();
        }
        return;
      } // parser.seen('W')

    #else
      constexpr bool seen_w = false;
    #endif

    // Jettison bed leveling data
    if (!seen_w && parser.seen('J')) {
      bedlevel.reset();
      return;
    }

    verbose_level = parser.intval('V');
    if (!WITHIN(verbose_level, 0, 4)) {
      SERIAL_EM("?(V)erbose Level is implausible (0-4).");
      return;
    }

    dryrun = parser.boolval('D')
      #if HAS_PROBE_MANUALLY
        || no_action
      #endif
    ;

    #if ENABLED(AUTO_BED_LEVELING_LINEAR)

      do_topography_map = verbose_level > 2 || parser.boolval('T');

      // X and Y specify points in each direction, overriding the default
      // These values may be saved with the completed mesh
      abl_grid_points.set(
        parser.byteval('X', GRID_MAX_POINTS_X),
        parser.byteval('Y', GRID_MAX_POINTS_Y)
      );
      if (parser.seenval('P')) abl_grid_points.x = abl_grid_points.y = parser.value_int();

      if (!WITHIN(abl_grid_points.x, 2, GRID_MAX_POINTS_X)) {
        SERIAL_EM("?Probe points (X) is implausible (2-" STRINGIFY(GRID_MAX_POINTS_X) ").");
        return;
      }
      if (!WITHIN(abl_grid_points.y, 2, GRID_MAX_POINTS_Y)) {
        SERIAL_EM("?Probe points (Y) is implausible (2-" STRINGIFY(GRID_MAX_POINTS_Y) ").");
        return;
      }

      abl_points = abl_grid_points.x * abl_grid_points.y;
      mean = 0.0;

    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

      zoffset = parser.linearval('Z');

    #endif

    #if ABL_GRID

      bedlevel.xy_probe_feedrate_mm_s = MMM_TO_MMS(parser.linearval('S', XY_PROBE_SPEED));

      #if !MECH(DELTA)
        const float x_min = probe.min_x(), x_max = probe.max_x(),
                    y_min = probe.min_y(), y_max = probe.max_y();
      #endif

      probe_position_lf.set(
        parser.seenval('L') ? NATIVE_X_POSITION(parser.value_linear_units()) : LEFT_PROBE_BED_POSITION,
        parser.seenval('F') ? NATIVE_Y_POSITION(parser.value_linear_units()) : FRONT_PROBE_BED_POSITION
      );
      probe_position_rb.set(
        parser.seenval('R') ? NATIVE_X_POSITION(parser.value_linear_units()) : RIGHT_PROBE_BED_POSITION,
        parser.seenval('B') ? NATIVE_Y_POSITION(parser.value_linear_units()) : BACK_PROBE_BED_POSITION
      );

      if (
        #if IS_KINEMATIC
             !mechanics.position_is_reachable_by_probe(probe_position_lf.x, 0)
          || !mechanics.position_is_reachable_by_probe(probe_position_rb.x, 0)
          || !mechanics.position_is_reachable_by_probe(0, probe_position_lf.y)
          || !mechanics.position_is_reachable_by_probe(0, probe_position_rb.y)
        #else
             !mechanics.position_is_reachable_by_probe(probe_position_lf)
          || !mechanics.position_is_reachable_by_probe(probe_position_rb)
        #endif
      ) {
        SERIAL_EM("? (L,R,F,B) out of bounds.");
        return;
      }

      // probe at the points of a lattice grid
      gridSpacing.set((probe_position_rb.x - probe_position_lf.x) / (abl_grid_points.x - 1),
                      (probe_position_rb.y - probe_position_lf.y) / (abl_grid_points.y - 1));

    #endif // ABL_GRID

    if (verbose_level > 0) {
      SERIAL_MSG("G29 Auto Bed Leveling");
      if (dryrun) SERIAL_MSG(" (DRYRUN)");
      SERIAL_EOL();
    }

    planner.synchronize();

    // Disable auto bed leveling during G29.
    // Be formal so G29 can be done successively without G28.
    if (!no_action) bedlevel.set_bed_leveling_enabled(false);

    #if HAS_BED_PROBE
      // Deploy the probe. Probe will raise if needed.
      if (DEPLOY_PROBE()) {
        bedlevel.restore_bed_leveling_state();
        return;
      }
    #endif

    if (!faux) mechanics.setup_for_endstop_or_probe_move();

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

      #if HAS_PROBE_MANUALLY
        if (!no_action)
      #endif
      {
        if (gridSpacing != abl.data.bilinear_grid_spacing || probe_position_lf != abl.data.bilinear_start) {
          // Reset grid to 0.0 or "not probed". (Also disables ABL)
          bedlevel.reset();

          // Initialize a grid with the given dimensions
          abl.data.bilinear_grid_spacing  = gridSpacing;
          abl.data.bilinear_start         = probe_position_lf;

          // Can't re-enable (on error) until the new grid is written
          bedlevel.flag.leveling_previous = false;
        }
      }

    #endif // AUTO_BED_LEVELING_BILINEAR

    #if ENABLED(AUTO_BED_LEVELING_3POINT)

      if (printer.debugFeature()) DEBUG_EM("> 3-point Leveling");

      // Probe at 3 arbitrary points
      points[0].z = points[1].z = points[2].z = 0;

    #endif // AUTO_BED_LEVELING_3POINT

  } // !bedlevel.flag.g29_in_progress

  #if HAS_PROBE_MANUALLY

    // For manual probing, get the next index to probe now.
    // On the first probe this will be incremented to 0.
    if (!no_action) {
      ++abl_probe_index;
      bedlevel.flag.g29_in_progress = true;
    }

    // Abort current G29 procedure, go back to idle state
    if (seenA && bedlevel.flag.g29_in_progress) {
      SERIAL_EM("Manual G29 aborted");
      #if HAS_SOFTWARE_ENDSTOPS
        endstops.setSoftEndstop(saved_soft_endstops_state);
      #endif
      bedlevel.restore_bed_leveling_state();
      bedlevel.flag.g29_in_progress = false;
      #if ENABLED(LCD_BED_LEVELING) && HAS_LCD_MENU
        lcdui.wait_for_move = false;
      #endif
    }

    // Query G29 status
    if (verbose_level || seenQ) {
      SERIAL_MSG("Manual G29 ");
      if (bedlevel.flag.g29_in_progress) {
        SERIAL_MV("point ", MIN(abl_probe_index + 1, abl_points));
        SERIAL_EMV(" of ", abl_points);
      }
      else
        SERIAL_EM("idle");
    }

    if (no_action) return;

    if (abl_probe_index == 0) {
      // For the initial G29 S2 save software endstop state
      #if HAS_SOFTWARE_ENDSTOPS
        saved_soft_endstops_state = endstops.isSoftEndstop();
      #endif
      // Move close to the bed before the first point
      mechanics.do_blocking_move_to_z(0);
    }
    else {

      #if ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_3POINT)
        const uint16_t index = abl_probe_index - 1;
      #endif

      // For G29 after adjusting Z.
      // Save the previous Z before going to the next point
      measured_z = mechanics.position.z;

      #if ENABLED(AUTO_BED_LEVELING_LINEAR)

        mean += measured_z;
        eqnBVector[index] = measured_z;
        eqnAMatrix[index + 0 * abl_points] = probePos.x;
        eqnAMatrix[index + 1 * abl_points] = probePos.y;
        eqnAMatrix[index + 2 * abl_points] = 1;

        incremental_LSF(&lsf_results, probePos, measured_z);

      #elif ENABLED(AUTO_BED_LEVELING_3POINT)

        points[index].z = measured_z;

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        abl.data.z_values[meshCount.x][meshCount.y] = measured_z + zoffset;

        if (printer.debugFeature()) {
          DEBUG_MV("Save X", meshCount.x);
          DEBUG_MV(" Y", meshCount.y);
          DEBUG_EMV(" Z", measured_z + zoffset);
        }

      #endif
    }

    //
    // If there's another point to sample, move there with optional lift.
    //

    #if ABL_GRID

      // Skip any unreachable points
      while (abl_probe_index < abl_points) {

        // Set meshCount.x, meshCount.y based on abl_probe_index, with zig-zag
        PR_OUTER_VAR = abl_probe_index / PR_INNER_END;
        PR_INNER_VAR = abl_probe_index - (PR_OUTER_VAR * PR_INNER_END);

        // Probe in reverse order for every other row/column
        bool zig = (PR_OUTER_VAR & 1); // != ((PR_OUTER_END) & 1);

        if (zig) PR_INNER_VAR = (PR_INNER_END - 1) - PR_INNER_VAR;

        probePos = probe_position_lf + gridSpacing * meshCount.asFloat();

        #if ENABLED(AUTO_BED_LEVELING_LINEAR)
          indexIntoAB[meshCount.x][meshCount.y] = abl_probe_index;
        #endif

        // Keep looping till a reachable point is found
        if (mechanics.position_is_reachable(probePos)) break;
        ++abl_probe_index;
      }

      // Is there a next point to move to?
      if (abl_probe_index < abl_points) {
        bedlevel.manual_goto_xy(probePos); // Can be used here too!
        #if HAS_SOFTWARE_ENDSTOPS
          // Disable software endstops to allow manual adjustment
          // If G29 is not completed, they will not be re-enabled
          endstops.setSoftEndstop(false);
        #endif
        return;
      }
      else {

        // Leveling done! Fall through to G29 finishing code below

        SERIAL_EM("Grid probing done.");

        // Re-enable software endstops, if needed
        #if HAS_SOFTWARE_ENDSTOPS
          endstops.setSoftEndstop(saved_soft_endstops_state);
        #endif
      }

    #elif ENABLED(AUTO_BED_LEVELING_3POINT)

      // Probe at 3 arbitrary points
      if (abl_probe_index < abl_points) {
        probePos.x = points[abl_probe_index].x;
        probePos.y = points[abl_probe_index].y;
        bedlevel.manual_goto_xy(probePos.x, probePos.y);
        #if HAS_SOFTWARE_ENDSTOPS
          // Disable software endstops to allow manual adjustment
          // If G29 is not completed, they will not be re-enabled
          endstops.setSoftEndstop(false);
        #endif
        return;
      }
      else {

        SERIAL_EM("3-point probing done.");

        // Re-enable software endstops, if needed
        #if HAS_SOFTWARE_ENDSTOPS
          endstops.setSoftEndstop(saved_soft_endstops_state);
        #endif

        if (!dryrun) {
          vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
          if (planeNormal.z < 0) planeNormal *= -1;
          bedlevel.matrix = matrix_3x3::create_look_at(planeNormal);

          // Can't re-enable (on error) until the new grid is written
          bedlevel.flag.leveling_previous = false;
        }

      }

    #endif // AUTO_BED_LEVELING_3POINT

  #else // !PROBE_MANUALLY
  {
    const ProbePtRaiseEnum raise_after = parser.boolval('E') ? PROBE_PT_STOW : PROBE_PT_RAISE;

    measured_z = 0.0;

    #if ABL_GRID

      bool zig = PR_OUTER_END & 1;  // Always end at RIGHT and BACK_PROBE_BED_POSITION

      xy_int8_t meshCount;

      // Outer loop is X with PROBE_Y_FIRST enabled
      // Outer loop is Y with PROBE_Y_FIRST disabled
      for (PR_OUTER_VAR = 0; PR_OUTER_VAR < PR_OUTER_END && !isnan(measured_z); PR_OUTER_VAR++) {

        int8_t inStart, inStop, inInc;

        if (zig) { // away from origin
          inStart = 0;
          inStop  = PR_INNER_END;
          inInc   = 1;
        }
        else {     // towards origin
          inStart = PR_INNER_END - 1;
          inStop  = -1;
          inInc   = -1;
        }

        zig ^= true; // zag

        // An index to print current state
        uint8_t pt_index = (PR_OUTER_VAR) * (PR_INNER_END) + 1;

        // Inner loop is Y with PROBE_Y_FIRST enabled
        // Inner loop is X with PROBE_Y_FIRST disabled
        for (PR_INNER_VAR = inStart; PR_INNER_VAR != inStop; pt_index++, PR_INNER_VAR += inInc) {

          probePos = probe_position_lf + gridSpacing * meshCount.asFloat();

          #if ENABLED(AUTO_BED_LEVELING_LINEAR)
            indexIntoAB[meshCount.x][meshCount.y] = ++abl_probe_index; // 0...
          #endif

          #if IS_KINEMATIC
            // Avoid probing outside the round or hexagonal area
            if (!mechanics.position_is_reachable_by_probe(probePos)) continue;
          #endif

          if (verbose_level) {
            SERIAL_MV("Probing mesh point ", int(pt_index));
            SERIAL_MV("/", int(GRID_MAX_POINTS));
            SERIAL_EOL();
          }
          #if HAS_LCD
            lcdui.status_printf_P(0, PSTR(S_FMT " %i/%i"), GET_TEXT(MSG_PROBING_MESH), int(pt_index), int(GRID_MAX_POINTS));
          #endif

          measured_z = faux ? 0.001f * random(-100, 101) : probe.check_at_point(probePos, raise_after, verbose_level);

          if (isnan(measured_z)) {
            bedlevel.restore_bed_leveling_state();
            break;
          }

          #if ENABLED(AUTO_BED_LEVELING_LINEAR)

            mean += measured_z;
            eqnBVector[abl_probe_index] = measured_z;
            eqnAMatrix[abl_probe_index + 0 * abl_points] = probePos.x;
            eqnAMatrix[abl_probe_index + 1 * abl_points] = probePos.y;
            eqnAMatrix[abl_probe_index + 2 * abl_points] = 1;

            incremental_LSF(&lsf_results, probePos, measured_z);

          #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

            abl.data.z_values[meshCount.x][meshCount.y] = measured_z + zoffset;

          #endif

          bedlevel.flag.leveling_previous = false;
          printer.idle();

        } // inner
      } // outer

    #elif ENABLED(AUTO_BED_LEVELING_3POINT)

      // Probe at 3 arbitrary points

      for (uint8_t i = 0; i < 3; ++i) {
        // Retain the last probe position
        probePos.x = points[i].x;
        probePos.y = points[i].y;
        measured_z = faux ? 0.001 * random(-100, 101) : probe.check_at_point(probePos.x, probePos.y, raise_after, verbose_level);
        if (isnan(measured_z)) {
          bedlevel.restore_bed_leveling_state();
          break;
        }
        points[i].z = measured_z;
      }

      if (!dryrun && !isnan(measured_z)) {
        vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
        if (planeNormal.z < 0) planeNormal *= -1;
        bedlevel.matrix = matrix_3x3::create_look_at(planeNormal);

        // Can't re-enable (on error) until the new grid is written
        bedlevel.flag.leveling_previous = false;
      }

    #endif // AUTO_BED_LEVELING_3POINT

    // Raise to _Z_PROBE_DEPLOY_HEIGHT. Stow the probe.
    if (STOW_PROBE()) {
      bedlevel.restore_bed_leveling_state();
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

  if (printer.debugFeature()) DEBUG_POS("> probing complete", mechanics.position);

  #if HAS_PROBE_MANUALLY
    bedlevel.flag.g29_in_progress = false;
    #if ENABLED(LCD_BED_LEVELING) && HAS_LCD_MENU
      lcdui.wait_for_move = false;
    #endif
  #endif

  // Calculate leveling, print reports, correct the position
  if (!isnan(measured_z)) {
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

      if (!dryrun) abl.extrapolate_unprobed_bed_level();
      abl.print_bilinear_leveling_grid();

      abl.refresh_bed_level();

      #if ENABLED(ABL_BILINEAR_SUBDIVISION)
        abl.print_bilinear_leveling_grid_virt();
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
      struct { float a, b, d; } plane_equation_coefficients;

      finish_incremental_LSF(&lsf_results);
      plane_equation_coefficients.a = -lsf_results.A;  // We should be able to eliminate the '-' on these three lines and down below
      plane_equation_coefficients.b = -lsf_results.B;  // but that is not yet tested.
      plane_equation_coefficients.d = -lsf_results.D;

      mean /= abl_points;

      if (verbose_level) {
        SERIAL_MV("Eqn coefficients: a: ", plane_equation_coefficients.a, 8);
        SERIAL_MV(" b: ", plane_equation_coefficients.b, 8);
        SERIAL_MV(" d: ", plane_equation_coefficients.d, 8);
        SERIAL_EOL();
        if (verbose_level > 2)
          SERIAL_EMV("Mean of sampled points: ", mean, 8);
      }

      // Create the matrix but don't correct the position yet
      if (!dryrun) {
        bedlevel.matrix = matrix_3x3::create_look_at(
          vector_3(-plane_equation_coefficients.a, -plane_equation_coefficients.b, 1) // We can eleminate the '-' here and up above
        );
      }

      // Show the Topography map if enabled
      if (do_topography_map) {

        float min_diff = 999;

        auto print_topo_map = [&](PGM_P const title, const bool get_min) {
          SERIAL_STR(title);
          for (int8_t yy = abl_grid_points.y - 1; yy >= 0; yy--) {
            for (uint8_t xx = 0; xx < abl_grid_points.x; xx++) {
              const int ind = indexIntoAB[xx][yy];
              xyz_float_t tmp = { eqnAMatrix[ind + 0 * abl_points],
                                  eqnAMatrix[ind + 1 * abl_points], 0 };
              apply_rotation_xyz(bedlevel.matrix, tmp);
              if (get_min) NOMORE(min_diff, eqnBVector[ind] - tmp.z);
              const float subval = get_min ? mean : tmp.z + min_diff,
                            diff = eqnBVector[ind] - subval;
              SERIAL_CHR(' '); if (diff >= 0.0) SERIAL_CHR('+');   // Include + for column alignment
              SERIAL_VAL(diff, 5);
            } // xx
            SERIAL_EOL();
          } // yy
          SERIAL_EOL();
        };

        print_topo_map(PSTR("\nBed Height Topography:\n"
                               "   +--- BACK --+\n"
                               "   |           |\n"
                               " L |    (+)    | R\n"
                               " E |           | I\n"
                               " F | (-) N (+) | G\n"
                               " T |           | H\n"
                               "   |    (-)    | T\n"
                               "   |           |\n"
                               "   O-- FRONT --+\n"
                               " (0,0)\n"), true);

        if (verbose_level > 3)
          print_topo_map(PSTR("\nCorrected Bed Height vs. Bed Topology:\n"), false);

      } // do_topography_map

    #endif // AUTO_BED_LEVELING_LINEAR_GRID

    #if ABL_PLANAR

      // For LINEAR and 3POINT leveling correct the current position

      if (verbose_level > 0)
        bedlevel.matrix.debug(PSTR("\n\nBed Level Correction Matrix:"));

      if (!dryrun) {
        //
        // Correct the current XYZ position based on the tilted plane.
        //

        if (printer.debugFeature()) DEBUG_POS("G29 uncorrected XYZ", mechanics.position);

        xyze_pos_t converted = mechanics.position;
        bedlevel.force_unapply_leveling(converted);

        // Use the last measured distance to the bed, if possible
        if ( NEAR(mechanics.position.x, probePos.x - probe.data.offset.x)
          && NEAR(mechanics.position.y, probePos.y - probe.data.offset.y)
        ) {
          float simple_z = mechanics.position.z - measured_z;
          if (printer.debugFeature()) {
            DEBUG_MV("Probed Z", simple_z);
            DEBUG_MV("  Matrix Z", converted.z);
            DEBUG_EMV("  Discrepancy:", simple_z - converted.z);
          }
          converted.z = simple_z;
        }

        // The rotated XY and corrected Z are now position.x
        mechanics.position = converted;

        if (printer.debugFeature()) DEBUG_POS("G29 corrected XYZ", mechanics.position);

      }

    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

      if (!dryrun) {
        if (printer.debugFeature()) DEBUG_EMV("G29 uncorrected Z:", mechanics.position.z);

        // Unapply the offset because it is going to be immediately applied
        // and cause compensation movement in Z
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          const float fade_scaling_factor = bedlevel.fade_scaling_factor_for_z(mechanics.position.z);
        #else
          constexpr float fade_scaling_factor = 1.0f;
        #endif
        mechanics.position.z -= fade_scaling_factor * abl.bilinear_z_offset(mechanics.position);

        if (printer.debugFeature()) DEBUG_EMV(" corrected Z:", mechanics.position.z);
      }

    #endif // ABL_PLANAR

    // Auto Bed Leveling is complete! Enable if possible.
    bedlevel.flag.leveling_active = dryrun ? bedlevel.flag.leveling_previous : true;
  } // !isnan(measured_z)

  // Restore state after probing
  if (!faux) mechanics.clean_up_after_endstop_or_probe_move();

  // Sync the planner from the position
  if (bedlevel.flag.leveling_active) mechanics.sync_plan_position();

  #if HAS_BED_PROBE && Z_PROBE_AFTER_PROBING > 0
    probe.move_z_after_probing();
  #endif

  #if ENABLED(Z_PROBE_END_SCRIPT)
    if (printer.debugFeature()) {
      DEBUG_MSG("Z Probe End Script: ");
      DEBUG_EM(Z_PROBE_END_SCRIPT);
    }
    planner.synchronize();
    commands.process_now_P(PSTR(Z_PROBE_END_SCRIPT));
  #endif

  mechanics.report_position();

  if (printer.debugFeature()) DEBUG_EM("<<< G29");

}

#endif // OLD_ABL

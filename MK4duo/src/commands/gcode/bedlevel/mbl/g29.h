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

#if ENABLED(MESH_BED_LEVELING)

  #define CODE_G29

  // Save 130 bytes with non-duplication of PSTR
  inline void say_not_entered(const char c) {
    SERIAL_CHR(c);
    SERIAL_EM(" not entered.");
  }

  /**
   * G29: Mesh-based Z probe, probes a grid and produces a
   *      mesh to compensate for variable bed height
   *
   * Parameters With MESH_BED_LEVELING:
   *
   *  S0              Produce a mesh report
   *  S1              Start probing mesh points
   *  S2              Probe the next mesh point
   *  S3 In Jn Zn.nn  Manually modify a single point
   *  S4 Zn.nn        Set z offset. Positive away from bed, negative closer to bed.
   *  S5              Reset and disable mesh
   *
   * The S0 report the points as below
   *
   *  +----> X-axis  1-n
   *  |
   *  |
   *  v Y-axis  1-n
   *
   */
  inline void gcode_G29() {

    static int mbl_probe_index = -1;
    #if HAS_SOFTWARE_ENDSTOPS
      static bool enable_soft_endstops;
    #endif

    MeshLevelingStateEnum state = (MeshLevelingStateEnum)parser.byteval('S', (int8_t)MeshReport);
    if (!WITHIN(state, 0, 5)) {
      SERIAL_MSG("S out of range (0-5).");
      return;
    }

    int8_t ix, iy;

    switch (state) {
      case MeshReport:
        SERIAL_MSG("Mesh Bed Leveling ");
        if (bedlevel.leveling_is_valid()) {
          SERIAL_EONOFF("state: ", bedlevel.flag.leveling_active);
          mbl.report_mesh();
        }
        else
          SERIAL_EM("has no data.");
        break;

      case MeshStart:
        mbl.factory_parameters();
        mbl_probe_index = 0;
        if (!lcdui.wait_for_move) {
          commands.enqueue_now_P(PSTR("G28\nG29 S2"));
          return;
        }
        state = MeshNext;

      case MeshNext:
        if (mbl_probe_index < 0) {
          SERIAL_EM("Start mesh probing with \"G29 S1\" first.");
          return;
        }
        // For each G29 S2...
        if (mbl_probe_index == 0) {
          #if HAS_SOFTWARE_ENDSTOPS
            // For the initial G29 S2 save software endstop state
            enable_soft_endstops = endstops.isSoftEndstop();
          #endif
          // Move close to the bed before the first point
          mechanics.do_blocking_move_to_z(0);
        }
        else {
          // Save Z for the previous mesh position
          mbl.set_zigzag_z(mbl_probe_index - 1, mechanics.position.z);
          #if HAS_SOFTWARE_ENDSTOPS
            endstops.setSoftEndstop(enable_soft_endstops);
          #endif
        }
        // If there's another point to sample, move there with optional lift.
        if (mbl_probe_index < GRID_MAX_POINTS) {
          #if HAS_SOFTWARE_ENDSTOPS
            // Disable software endstops to allow manual adjustment
            // If G29 is not completed, they will not be re-enabled
            endstops.setSoftEndstop(false);
          #endif

          mbl.zigzag(mbl_probe_index++, ix, iy);
          bedlevel.manual_goto_xy({ mbl.data.index_to_xpos[ix], mbl.data.index_to_ypos[iy] });
        }
        else {
          // One last "return to the bed" (as originally coded) at completion
          mechanics.position.z = MANUAL_PROBE_HEIGHT;
          mechanics.line_to_position();
          planner.synchronize();

          // After recording the last point, activate the mbl and home
          mbl_probe_index = -1;
          SERIAL_EM("Mesh probing done.");
          sound.feedback();

          mechanics.home();
          bedlevel.set_bed_leveling_enabled(true);

          #if ENABLED(MESH_G28_REST_ORIGIN)
            mechanics.position.z = 0;
            mechanics.line_to_position(mechanics.homing_feedrate_mm_s.z);
            planner.synchronize();
          #endif

          #if ENABLED(LCD_BED_LEVELING)
            lcdui.wait_for_move = false;
          #endif
        }
        break;

      case MeshSet:
        if (parser.seenval('I')) {
          ix = parser.value_int();
          if (!WITHIN(ix, 0, GRID_MAX_POINTS_X - 1)) {
            SERIAL_MV("I out of range (0-", int(GRID_MAX_POINTS_X - 1));
            SERIAL_EM(")");
            return;
          }
        }
        else {
          say_not_entered('I');
          return;
        }

        if (parser.seenval('J')) {
          iy = parser.value_int();
          if (!WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1)) {
            SERIAL_MV("J out of range (0-", int(GRID_MAX_POINTS_Y - 1));
            SERIAL_EM(")");
            return;
          }
        }
        else {
          say_not_entered('J');
          return;
        }

        if (parser.seenval('Z')) {
          mbl.data.z_values[ix][iy] = parser.value_linear_units();
        }
        else {
          say_not_entered('Z');
          return;
        }

        break;

      case MeshSetZOffset:
        if (parser.seenval('Z')) {
          mbl.data.z_offset = parser.value_linear_units();
        }
        else {
          say_not_entered('Z');
          return;
        }
        break;

      case MeshReset:
        bedlevel.reset();
        break;

    } // switch (state)

    if (state == MeshNext) {
      SERIAL_MV("MBL G29 point ", MIN(mbl_probe_index, GRID_MAX_POINTS));
      SERIAL_EMV(" of ", int(GRID_MAX_POINTS));
    }

    mechanics.report_position();
  }

#endif // ENABLED(MESH_BED_LEVELING)

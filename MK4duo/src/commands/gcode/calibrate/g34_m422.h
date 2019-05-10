/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(Z_STEPPER_AUTO_ALIGN)

#define CODE_G34

float z_auto_align_xpos[Z_STEPPER_COUNT] = Z_STEPPER_ALIGN_X,
      z_auto_align_ypos[Z_STEPPER_COUNT] = Z_STEPPER_ALIGN_Y;

inline void set_all_z_lock(const bool lock) {
  stepper.set_z_lock(lock);
  stepper.set_z2_lock(lock);
  #if ENABLED(Z_THREE_STEPPER_DRIVERS)
    stepper.set_z3_lock(lock);
  #endif
}

/**
 * G34: Z-Stepper automatic alignment
 *
 * Parameters: I<iterations> T<accuracy> A<amplification>
 */
inline void gcode_G34(void) {

  if (printer.debugFeature()) {
    DEBUG_EM(">>> G34");
    DEBUG_LOG_INFO();
  }

  do { // break out on error

    if (!mechanics.isAxisHomed(X_AXIS) || !mechanics.isAxisHomed(Y_AXIS)) {
      if (printer.debugFeature()) DEBUG_EM("> XY homing required.");
      break;
    }

    const int8_t z_auto_align_iterations = parser.intval('I', Z_STEPPER_ALIGN_ITERATIONS);
    if (!WITHIN(z_auto_align_iterations, 1, 30)) {
      SERIAL_EM("?(I)teration out of bounds (1-30).");
      break;
    }

    const float z_auto_align_accuracy = parser.floatval('T', Z_STEPPER_ALIGN_ACC);
    if (!WITHIN(z_auto_align_accuracy, 0.01f, 1.0f)) {
      SERIAL_EM("?(T)arget accuracy out of bounds (0.01-1.0).");
      break;
    }

    const float z_auto_align_amplification = parser.floatval('A', Z_STEPPER_ALIGN_AMP);
    if (!WITHIN(ABS(z_auto_align_amplification), 0.5f, 2.0f)) {
      SERIAL_EM("?(A)mplification out of bounds (0.5-2.0).");
      break;
    }

    // Wait for planner moves to finish!
    planner.synchronize();

    // Disable the leveling matrix before auto-aligning
    #if HAS_LEVELING
      #if ENABLED(RESTORE_LEVELING_AFTER_G34)
        const bool leveling_was_active = bedlevel.flag.leveling_active;
      #endif
      bedlevel.set_bed_leveling_enabled(false);
    #endif

    #if ENABLED(CNC_WORKSPACE_PLANES)
      workspace_plane = PLANE_XY;
    #endif

    #if ENABLED(BLTOUCH)
      bltouch.init();
    #endif

    // Always home with tool 0 active
    #if HOTENDS > 1
      const uint8_t old_tool_index = tools.active_extruder;
      tools.change(0, 0, true);
    #endif

    // Before moving other axes raise Z, if needed. Never lower Z.
    if (mechanics.current_position[Z_AXIS] < Z_PROBE_BETWEEN_HEIGHT) {
      if (printer.debugFeature()) DEBUG_EMV("Raise Z (before moving to probe pos) to ", Z_PROBE_BETWEEN_HEIGHT);
      mechanics.do_blocking_move_to_z(Z_PROBE_BETWEEN_HEIGHT);
    }

    // Remember corrections to determine errors on each iteration
    float last_z_align_move[Z_STEPPER_COUNT] = ARRAY_N(Z_STEPPER_COUNT, 10000.0f, 10000.0f, 10000.0f),
          z_measured[Z_STEPPER_COUNT] = { 0 };
    bool err_break = false;
    for (uint8_t iteration = 0; iteration < z_auto_align_iterations; ++iteration) {
      if (printer.debugFeature()) DEBUG_EM("> probing all positions.");

      // Reset minimum value
      float z_measured_min = 100000.0f;
      // For each iteration go through all probe positions (one per Z-Stepper)
      for (uint8_t zstepper = 0; zstepper < Z_STEPPER_COUNT; ++zstepper) {
        // Probe a Z height for each stepper
        z_measured[zstepper] = probe.check_pt(z_auto_align_xpos[zstepper], z_auto_align_ypos[zstepper], PROBE_PT_RAISE, false);

        // Stop on error
        if (isnan(z_measured[zstepper])) {
          if (printer.debugFeature()) DEBUG_EM("> PROBING FAILED!");
          err_break = true;
          break;
        }

        if (printer.debugFeature()) {
          DEBUG_MV("> Z", int(zstepper + 1));
          DEBUG_EMV(" measured position is ", z_measured[zstepper]);
        }

        // Remember the maximum position to calculate the correction
        z_measured_min = MIN(z_measured_min, z_measured[zstepper]);
      }

      if (err_break) break;

      // Remember the current z position to return to
      float z_original_position = mechanics.current_position[Z_AXIS];

      // Iterations can stop early if all corrections are below required accuracy
      bool success_break = true;
      // Correct stepper offsets and re-iterate
      for (uint8_t zstepper = 0; zstepper < Z_STEPPER_COUNT; ++zstepper) {
        stepper.set_separate_multi_axis(true);
        set_all_z_lock(true); // Steppers will be enabled separately

        // Calculate current stepper move
        const float z_align_move = z_measured[zstepper] - z_measured_min,
                    z_align_abs = ABS(z_align_move);

        // Check for lost accuracy compared to last move
        if (last_z_align_move[zstepper] < z_align_abs - 1.0) {
          // Stop here
          if (printer.debugFeature()) DEBUG_EM("> detected decreasing accuracy.");
          err_break = true;
          break;
        }
        else
          last_z_align_move[zstepper] = z_align_abs;

        // Only stop early if all measured points achieve accuracy target
        if (z_align_abs > z_auto_align_accuracy) success_break = false;

        if (printer.debugFeature()) {
          DEBUG_MV("> Z", int(zstepper + 1));
          DEBUG_EMV(" corrected by ", z_align_move);
        }

        switch (zstepper) {
          case 0: stepper.set_z_lock(false); break;
          case 1: stepper.set_z2_lock(false); break;
          #if ENABLED(Z_THREE_STEPPER_DRIVERS)
            case 2: stepper.set_z3_lock(false); break;
          #endif
        }

        // This will lose home position and require re-homing
        mechanics.do_blocking_move_to_z(z_auto_align_amplification * z_align_move + mechanics.current_position[Z_AXIS]);
      }

      if (err_break) break;

      // Move Z back to previous position
      set_all_z_lock(true);
      mechanics.do_blocking_move_to_z(z_original_position);
      set_all_z_lock(false);

      stepper.set_separate_multi_axis(false);

      if (success_break) {
        if (printer.debugFeature()) DEBUG_EM("> achieved target accuracy.");
        break;
      }
    }

    if (err_break) break;

    // Restore the active tool after homing
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif

    #if HAS_LEVELING
      #if ENABLED(RESTORE_LEVELING_AFTER_G34)
        bedlevel.set_bed_leveling_enabled(leveling_was_active);
      #endif
    #endif

    // After this operation the z position needs correction
    mechanics.setAxisHomed(Z_AXIS, false);

    gcode_G28();

  } while(0);

  if (printer.debugFeature()) DEBUG_EM("<<< G34");
}

#define CODE_M422

/**
 * M422: Z-Stepper automatic alignment parameter selection
 */
inline void gcode_M422() {
  const int8_t zstepper = parser.intval('S') - 1;
  if (!WITHIN(zstepper, 0, Z_STEPPER_COUNT - 1)) {
    SERIAL_EM("?(S) Z-Stepper index invalid.");
    return;
  }

  const float x_pos = parser.floatval('X', z_auto_align_xpos[zstepper]);
  if (!WITHIN(x_pos, mechanics.data.base_pos[X_AXIS].min, mechanics.data.base_pos[X_AXIS].max)) {
    SERIAL_EM("?(X) out of bounds.");
    return;
  }

  const float y_pos = parser.floatval('Y', z_auto_align_ypos[zstepper]);
  if (!WITHIN(y_pos, mechanics.data.base_pos[Y_AXIS].min, mechanics.data.base_pos[Y_AXIS].max)) {
    SERIAL_EM("?(Y) out of bounds.");
    return;
  }

  z_auto_align_xpos[zstepper] = x_pos;
  z_auto_align_ypos[zstepper] = y_pos;
}

#endif // ENABLED(Z_STEPPER_AUTO_ALIGN)

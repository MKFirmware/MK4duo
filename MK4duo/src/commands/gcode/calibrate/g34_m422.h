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

    // Always home with tool 0 active
    #if HOTENDS > 1
      const uint8_t old_tool_index = tools.active_extruder;
      tools.change(0, 0, true);
    #endif

    #if ENABLED(BLTOUCH) && ENABLED(BLTOUCH_HIGH_SPEED_MODE)
      // In BLTOUCH HS mode, the probe travels in a deployed state.
      // Users of G34 might have a badly misaligned bed, so raise Z by the
      // length of the deployed pin (BLTOUCH stroke < 7mm)
      #define Z_BASIC_CLEARANCE Z_PROBE_BETWEEN_HEIGHT + 7.0f
    #else
      #define Z_BASIC_CLEARANCE Z_PROBE_BETWEEN_HEIGHT
    #endif

    #define G34_MAX_GRADE 5
    float z_probe = Z_BASIC_CLEARANCE + (G34_MAX_GRADE) * 0.01f * (
      #if ENABLED(Z_THREE_STEPPER_DRIVERS)
         SQRT(MAX(HYPOT2(z_auto_align_xpos[0] - z_auto_align_ypos[0], z_auto_align_xpos[1] - z_auto_align_ypos[1]),
                  HYPOT2(z_auto_align_xpos[1] - z_auto_align_ypos[1], z_auto_align_xpos[2] - z_auto_align_ypos[2]),
                  HYPOT2(z_auto_align_xpos[2] - z_auto_align_ypos[2], z_auto_align_xpos[0] - z_auto_align_ypos[0])))
      #else
         HYPOT(z_auto_align_xpos[0] - z_auto_align_ypos[0], z_auto_align_xpos[1] - z_auto_align_ypos[1])
      #endif
    );

    mechanics.home();

    // Move the Z coordinate realm towards the positive - dirty trick
    mechanics.current_position[Z_AXIS] -= z_probe * 0.5;

    float last_z_align_move[Z_STEPPER_COUNT] = ARRAY_N(Z_STEPPER_COUNT, 10000.0f, 10000.0f, 10000.0f),
          z_measured[Z_STEPPER_COUNT] = { 0 },
          z_maxdiff = 0.0f,
          amplification = z_auto_align_amplification;

    uint8_t iteration;
    bool err_break = false;
    for (iteration = 0; iteration < z_auto_align_iterations; ++iteration) {
      if (printer.debugFeature()) DEBUG_EM("> probing all positions.");

      SERIAL_EMV("\nITERATION: ", int(iteration + 1));

      // Initialize minimum value
      float z_measured_min = 100000.0f;
      // Probe all positions (one per Z-Stepper)
      for (uint8_t izstepper = 0; izstepper < Z_STEPPER_COUNT; ++izstepper) {
        // iteration odd/even --> downward / upward stepper sequence 
        const uint8_t zstepper = (iteration & 1) ? Z_STEPPER_COUNT - 1 - izstepper : izstepper;

        // Safe clearance even on an incline
        if (iteration == 0 || izstepper > 0) mechanics.do_blocking_move_to_z(z_probe);

        // Probe a Z height for each stepper
        if (isnan(probe.check_pt(z_auto_align_xpos[zstepper], z_auto_align_ypos[zstepper], PROBE_PT_RAISE, true))) {
          SERIAL_EM("Probing failed.");
          err_break = true;
          break;
        }

        // This is not the trigger Z value. It is the position of the probe after raising it.
        // It is higher than the trigger value by a constant value (not known here). This value
        // is more useful for determining the desired next iteration Z position for probing. It is
        // equally well suited for determining the misalignment, just like the trigger position would be.
        z_measured[zstepper] = mechanics.current_position[Z_AXIS];
        if (printer.debugFeature()) {
          DEBUG_MV("> Z", int(zstepper + 1));
          DEBUG_EMV(" measured position is ", z_measured[zstepper]);
        }

        // Remember the minimum measurement to calculate the correction later on
        z_measured_min = MIN(z_measured_min, z_measured[zstepper]);
      }

      if (err_break) break;

      // Adapt the next probe clearance height based on the new measurements.
      // Safe_height = lowest distance to bed (= highest measurement) plus highest measured misalignment.
      #if ENABLED(Z_THREE_STEPPER_DRIVERS)
        z_maxdiff = MAX(ABS(z_measured[0] - z_measured[1]), ABS(z_measured[1] - z_measured[2]), ABS(z_measured[2] - z_measured[0]));
        z_probe = Z_BASIC_CLEARANCE + MAX(z_measured[0], z_measured[1], z_measured[2]) + z_maxdiff;
      #else
        z_maxdiff = ABS(z_measured[0] - z_measured[1]);
        z_probe = Z_BASIC_CLEARANCE + MAX(z_measured[0], z_measured[1]) + z_maxdiff;
      #endif

      SERIAL_MV("\nDIFFERENCE Z1-Z2=", ABS(z_measured[0] - z_measured[1]));
      #if ENABLED(Z_TRIPLE_STEPPER_DRIVERS)
        SERIAL_MV(" Z2-Z3=", ABS(z_measured[1] - z_measured[2]));
        SERIAL_MV(" Z3-Z1=", ABS(z_measured[2] - z_measured[0]));
      #endif
      SERIAL_EOL();

      // The following correction actions are to be enabled for select Z-steppers only
      stepper.set_separate_multi_axis(true);

      bool success_break = true;
      // Correct stepper offsets and re-iterate
      for (uint8_t zstepper = 0; zstepper < Z_STEPPER_COUNT; ++zstepper) {
        // Calculate current stepper move
        const float z_align_move = z_measured[zstepper] - z_measured_min,
                    z_align_abs = ABS(z_align_move);

        // Optimize one iterations correction based on the first measurements
        if (z_align_abs > 0.0f) amplification = iteration == 1 ? MIN(last_z_align_move[zstepper] / z_align_abs, 2.0f) : z_auto_align_amplification;

        // Check for lost accuracy compared to last move
        if (last_z_align_move[zstepper] < z_align_abs - 1.0) {
          SERIAL_EM("Decreasing accuracy detected.");
          err_break = true;
          break;
        }

        // Remember the alignment for the next iteration
        last_z_align_move[zstepper] = z_align_abs;

        // Stop early if all measured points achieve accuracy target// Only stop early if all measured points achieve accuracy target
        if (z_align_abs > z_auto_align_accuracy) success_break = false;

        if (printer.debugFeature()) {
          DEBUG_MV("> Z", int(zstepper + 1));
          DEBUG_EMV(" corrected by ", z_align_move);
        }

        // Lock all steppers except one
        set_all_z_lock(true);
        switch (zstepper) {
          case 0: stepper.set_z_lock(false); break;
          case 1: stepper.set_z2_lock(false); break;
          #if ENABLED(Z_THREE_STEPPER_DRIVERS)
            case 2: stepper.set_z3_lock(false); break;
          #endif
        }

        // Do a move to correct part of the misalignment for the current stepper
        mechanics.do_blocking_move_to_z(amplification * z_align_move + mechanics.current_position[Z_AXIS]);
      }

      // Back to normal stepper operations
      set_all_z_lock(false);
      stepper.set_separate_multi_axis(false);

      if (err_break) break;
      
      if (success_break) {
        SERIAL_EM("Target accuracy achieved.");
        break;
      }
    }

    if (err_break) {
      SERIAL_EM("G34 aborted.");
      break;
    }

    SERIAL_MV("Did ", int(iteration + (iteration != z_auto_align_iterations)));
    SERIAL_EMV(" iterations of ", int(z_auto_align_iterations));
    SERIAL_EMV("Accuracy: ", z_maxdiff);

    // Restore the active tool after homing
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif

    #if HAS_LEVELING && ENABLED(RESTORE_LEVELING_AFTER_G34)
      bedlevel.set_bed_leveling_enabled(leveling_was_active);
    #endif

    // After this operation the z position needs correction
    mechanics.setAxisHomed(Z_AXIS, false);

    #if ENABLED(BLTOUCH) && ENABLED(BLTOUCH_HIGH_SPEED_MODE)
      // In BLTOUCH High Speed mode, the pin is still deployed at this point.
      // The upcoming G28 means travel, so it is better to stow the pin.
      bltouch.stow();
    #endif

    // Home after the alignment procedure
    mechanics.home();

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

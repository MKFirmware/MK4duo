/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
 * tools.cpp
 *
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if EXTRUDERS > 0

Tools tools;

/** Public Parameters */
tool_data_t Tools::data;

extruder_t  Tools::extruder;

int16_t Tools::flow_percentage[EXTRUDERS]       = ARRAY_BY_EXTRUDERS(100),
        Tools::density_percentage[EXTRUDERS]    = ARRAY_BY_EXTRUDERS(100);
float   Tools::e_factor[EXTRUDERS]              = ARRAY_BY_EXTRUDERS(1.0);

#if ENABLED(VOLUMETRIC_EXTRUSION)
  float Tools::filament_size[EXTRUDERS]         = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA),
        Tools::volumetric_area_nominal          = CIRCLE_AREA(float(DEFAULT_NOMINAL_FILAMENT_DIA) * 0.5f),
        Tools::volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0);
#endif

#if ENABLED(PID_ADD_EXTRUSION_RATE)
  int16_t Tools::lpq_len = 20;
#endif

/** Public Function */
void Tools::factory_parameters() {

  #if ENABLED(HOTEND_OFFSET_X) && ENABLED(HOTEND_OFFSET_Y) && ENABLED(HOTEND_OFFSET_Z)
    constexpr float HEoffset[XYZ][6] = {
      HOTEND_OFFSET_X,
      HOTEND_OFFSET_Y,
      HOTEND_OFFSET_Z
    };
  #else
    constexpr float HEoffset[XYZ][HOTENDS] = { 0.0f };
  #endif

  static_assert(
    HEoffset[X_AXIS][0] == 0 && HEoffset[Y_AXIS][0] == 0 && HEoffset[Z_AXIS][0] == 0,
    "Offsets for the first hotend must be 0.0."
  );
  LOOP_XYZ(i) {
    LOOP_HOTEND() data.hotend_offset[i][h] = HEoffset[i][h];
  }

  #if ENABLED(TOOL_CHANGE_FIL_SWAP)
    data.swap_length    = TOOL_CHANGE_FIL_SWAP_LENGTH;
    data.purge_lenght   = TOOL_CHANGE_FIL_SWAP_PURGE;
    data.purge_speed    = TOOL_CHANGE_FIL_SWAP_PURGE_SPEED;
    data.retract_speed  = TOOL_CHANGE_FIL_SWAP_RETRACT_SPEED;
  #endif

  #if ENABLED(NOZZLE_PARK_FEATURE)
    data.park_point = NOZZLE_PARK_POINT;
  #elif EXTRUDERS > 1
    data.park_point = { 0, 0, TOOL_CHANGE_Z_RAISE };
  #endif

}

void Tools::change(const uint8_t tmp_extruder, bool no_move/*=false*/) {

  #if ENABLED(COLOR_MIXING_EXTRUDER)

    UNUSED(no_move);

    if (tmp_extruder >= MIXING_VIRTUAL_TOOLS)
      return invalid_extruder_error(tmp_extruder);

    #if MIXING_VIRTUAL_TOOLS > 1
      // T0-Tnnn: Switch virtual tool by changing the index to the mix
      mixer.T(tmp_extruder);
    #endif

  #elif HAS_MMU2

    UNUSED(no_move);

    mmu2.tool_change(tmp_extruder);

  #elif EXTRUDERS < 2

    UNUSED(no_move);

    if (tmp_extruder) invalid_extruder_error(tmp_extruder);
    return;

  #else // EXTRUDERS > 1

    planner.synchronize();
  
    #if ENABLED(DUAL_X_CARRIAGE)  // Only T0 allowed if the Printer is in DXC_DUPLICATION_MODE or DXC_SCALED_DUPLICATION_MODE
      // Only T0 allowed in DXC_DUPLICATION_MODE
      if (tmp_extruder != 0 && mechanics.dxc_is_duplicating())
         return invalid_extruder_error(tmp_extruder);
    #endif

    #if HAS_LEVELING
      // Set current position to the physical position
      const bool leveling_was_active = bedlevel.flag.leveling_active;
      bedlevel.set_bed_leveling_enabled(false);
    #endif

    if (tmp_extruder >= EXTRUDERS)
      return invalid_extruder_error(tmp_extruder);

    if (!no_move && mechanics.axis_unhomed_error()) {
      no_move = true;
      if (printer.debugFeature()) DEBUG_EM("No move (not homed)");
    }

    #if ENABLED(DUAL_X_CARRIAGE)
      const bool idex_full_control = mechanics.dual_x_carriage_mode == DXC_FULL_CONTROL_MODE;
    #else
      constexpr bool idex_full_control = false;
    #endif

    const bool can_move_away = !no_move && !idex_full_control;

    #if ENABLED(TOOL_CHANGE_FIL_SWAP)
      const bool should_swap = can_move_away && data.swap_length;
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        const bool too_cold = !printer.debugDryrun() && (thermalManager.tooColdToExtrude(ACTIVE_HOTEND) || thermalManager.tooColdToExtrude(TARGET_HOTEND));
      #else
        constexpr bool too_cold = false;
      #endif
      if (should_swap) {
        if (too_cold)
          SERIAL_LM(ER, MSG_HOTEND_TOO_COLD);
        else {
          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            advancedpause.do_pause_e_move(-data.swap_length, MMM_TO_MMS(data.retract_speed));
          #else
            mechanics.current_position[E_AXIS] -= data.swap_length / planner.e_factor[extruder.active];
            planner.buffer_line(mechanics.current_position, MMM_TO_MMS(data.retract_speed), extruder.active);
            planner.synchronize();
          #endif
        }
      }
    #endif // TOOL_CHANGE_FIL_SWAP

    if (tmp_extruder != extruder.active) {

      REMEMBER(fr, mechanics.feedrate_mm_s, XY_PROBE_FEEDRATE_MM_S);

      #if HAS_SOFTWARE_ENDSTOPS
        #if HOTENDS > 1
          #define _EXT_ARGS , extruder.active, tmp_extruder
        #else
          #define _EXT_ARGS
        #endif
        endstops.update_software_endstops(X_AXIS _EXT_ARGS);
        #if DISABLED(DUAL_X_CARRIAGE)
          endstops.update_software_endstops(Y_AXIS _EXT_ARGS);
          endstops.update_software_endstops(Z_AXIS _EXT_ARGS);
        #endif
      #endif

      mechanics.set_destination_to_current();

      #if !HAS_DONDOLO
        if (can_move_away) {
          // Do a small lift to avoid the workpiece in the move back (below)
          mechanics.current_position[Z_AXIS] += data.park_point.z;
          #if HAS_SOFTWARE_ENDSTOPS
            NOMORE(mechanics.current_position[Z_AXIS], endstops.soft_endstop[Z_AXIS].max);
          #endif
          fast_line_to_current(Z_AXIS);
          #if ENABLED(TOOL_CHANGE_PARK)
            mechanics.current_position[X_AXIS] = data.park_point.x;
            mechanics.current_position[Y_AXIS] = data.park_point.y;
          #endif
          planner.buffer_line(mechanics.current_position, feedrate_mm_s, extruder.active);
          planner.synchronize();
        }
      #endif

      #if HOTENDS > 1
        #if ENABLED(DUAL_X_CARRIAGE)
          constexpr float x_diff = 0;
        #else
          const float x_diff = data.hotend_offset[X_AXIS][tmp_extruder] - data.hotend_offset[X_AXIS][extruder.active];
        #endif
        const float y_diff = data.hotend_offset[Y_AXIS][tmp_extruder] - data.hotend_offset[Y_AXIS][extruder.active],
                    z_diff = data.hotend_offset[Z_AXIS][tmp_extruder] - data.hotend_offset[Z_AXIS][extruder.active];
      #else
        constexpr float x_diff = 0, y_diff = 0, z_diff = 0;
      #endif

      #if ENABLED(DUAL_X_CARRIAGE)
        dualx_tool_change(tmp_extruder, no_move); // Can modify no_move
      #elif HAS_DONDOLO
        // Always raise by at least 1 to avoid workpiece
        mechanics.current_position[Z_AXIS] += MAX(-z_diff, 0.0) + data.park_point.z;
        #if HAS_SOFTWARE_ENDSTOPS
          NOMORE(mechanics.current_position[Z_AXIS], endstops.soft_endstop[Z_AXIS].max);
        #endif
        if (!no_move) fast_line_to_current(Z_AXIS);
        move_extruder_servo(tmp_extruder);
      #endif

      if (printer.debugFeature()) {
        DEBUG_MV("Offset Tool XYZ by { ", x_diff);
        DEBUG_MV(", ", y_diff);
        DEBUG_MV(", ", z_diff);
        DEBUG_EM(" }");
      }

      // The newly-selected extruder XY is actually at...
      mechanics.current_position[X_AXIS] += x_diff;
      mechanics.current_position[Y_AXIS] += y_diff;
      mechanics.current_position[Z_AXIS] += z_diff;

      #if HAS_MKMULTI_TOOLS
        MK_multi_tool_change(tmp_extruder);
      #else
        // Set the new active extruder
        extruder.previous = extruder.active;
        extruder.active = tmp_extruder;
      #endif

      // Tell the planner the new "current position"
      mechanics.sync_plan_position();

      #if ENABLED(DELTA)
        const bool safe_to_move = mechanics.current_position[Z_AXIS] < mechanics.delta_clip_start_height - 1;
      #else
        constexpr bool safe_to_move = true;
      #endif

      // Raise, move, and lower again
      if (safe_to_move && !no_move && printer.isRunning()) {

        #if ENABLED(TOOL_CHANGE_FIL_SWAP)
          if (should_swap && !too_cold) {
            #if ENABLED(ADVANCED_PAUSE_FEATURE)
              do_pause_e_move(data.swap_length + data.purge_lenght, MMM_TO_MMS(data.purge_speed));
            #else
              current_position[E_AXIS] += (data.swap_length + data.purge_lenght) / planner.e_factor[tmp_extruder];
              planner.buffer_line(mechanics.current_position, MMM_TO_MMS(data.purge_speed), tmp_extruder);
            #endif
            planner.synchronize();
            planner.set_e_position_mm((mechanics.destination[E_AXIS] = mechanics.current_position[E_AXIS] = mechanics.current_position[E_AXIS] - data.purge_lenght));
          }
        #endif

        // Prevent a move outside physical bounds
        endstops.apply_motion_limits(mechanics.destination);

        // Should the nozzle move back to the old position?
        if (can_move_away) {
          #if ENABLED(TOOL_CHANGE_NO_RETURN)
            // Just move back down
            if (printer.debugFeature()) DEBUG_LM(DEB, "Move back Z only");
            mechanics.do_blocking_move_to_z(mechanics.destination[Z_AXIS], mechanics.data.max_feedrate_mm_s[Z_AXIS]);
          #else
            // Move back to the original (or adjusted) position
            if (printer.debugFeature()) DEBUG_POS("Move back", mechanics.destination);
            mechanics.do_blocking_move_to(destination);
          #endif
        }
        else if (printer.debugFeature()) DEBUG_LM(DEB, "Move back skipped");

        #if ENABLED(DUAL_X_CARRIAGE)
          mechanics.active_extruder_parked = false;
        #endif
      }
      #if HAS_DONDOLO
        else {
          // Move back down. (Including when the new tool is higher.)
          mechanics.do_blocking_move_to_z(mechanics.destination[Z_AXIS], mechanics.data.max_feedrate_mm_s[Z_AXIS]);
        }
      #endif
    } // (tmp_extruder != extruder.active)

    planner.synchronize();

    #if ENABLED(EXT_SOLENOID)
      disable_all_solenoids();
      enable_solenoid_on_active_extruder();
    #endif

    #if HAS_LEVELING
      // Restore leveling to re-establish the logical position
      bedlevel.set_bed_leveling_enabled(leveling_was_active);
    #endif

    SERIAL_LMV(ECHO, MSG_ACTIVE_EXTRUDER, (int)extruder.active);

  #endif // EXTRUDERS > 1

}

#if ENABLED(NOZZLE_PARK_FEATURE) || EXTRUDERS > 1

  void Tools::print_M217() {
    #if ENABLED(TOOL_CHANGE_FIL_SWAP)
      SERIAL_LM(CFG, "Tool change: S<swap_lenght> E<purge_lenght> P<purge_speed> R<retract_speed>");
      SERIAL_SM(CFG, "  M217");
      SERIAL_MV(" S", LINEAR_UNIT(tools.data.swap_length));
      SERIAL_MV(" E", LINEAR_UNIT(tools.data.purge_lenght));
      SERIAL_MV(" P", LINEAR_UNIT(tools.data.purge_speed));
      SERIAL_MV(" R", LINEAR_UNIT(tools.data.retract_speed));
      SERIAL_EOL();
    #endif

    #if ENABLED(NOZZLE_PARK_FEATURE)
      SERIAL_LM(CFG, "Nozzle Park: X<point> Y<point> Z<point>");
      SERIAL_SM(CFG, "  M217");
      SERIAL_MV(" X", LINEAR_UNIT(tools.data.park_point.x));
      SERIAL_MV(" Y", LINEAR_UNIT(tools.data.park_point.y));
      SERIAL_MV(" Z", LINEAR_UNIT(tools.data.park_point.z));
      SERIAL_EOL();
    #else
      SERIAL_LM(CFG, "Z raise: Z<point>:");
      SERIAL_SM(CFG, "  M217");
      SERIAL_MV(" Z", LINEAR_UNIT(tools.data.park_point.z));
      SERIAL_EOL();
    #endif
  }

#endif // ENABLED(NOZZLE_PARK_FEATURE) || EXTRUDERS > 1

void Tools::print_M218(const uint8_t h) {
  SERIAL_LM(CFG, "Hotend offset (unit): H<Hotend> X<offset> Y<offset> Z<offset>:");
  SERIAL_SMV(CFG, "  M218 H", (int)h);
  SERIAL_MV(" X", LINEAR_UNIT(data.hotend_offset[X_AXIS][h]), 3);
  SERIAL_MV(" Y", LINEAR_UNIT(data.hotend_offset[Y_AXIS][h]), 3);
  SERIAL_MV(" Z", LINEAR_UNIT(data.hotend_offset[Z_AXIS][h]), 3);
  SERIAL_EOL();
}

#if ENABLED(EXT_SOLENOID)

  void Tools::enable_solenoid(const uint8_t e) {
    switch (e) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS_SOLENOID_1 && EXTRUDERS > 1
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_2 && EXTRUDERS > 2
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_3 && EXTRUDERS > 3
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_4 && EXTRUDERS > 4
          case 4:
            OUT_WRITE(SOL4_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_5 && EXTRUDERS > 5
          case 5:
            OUT_WRITE(SOL5_PIN, HIGH);
            break;
        #endif
      default:
        SERIAL_LM(ER, MSG_INVALID_SOLENOID);
        break;
    }
  }

  void Tools::enable_solenoid_on_active_extruder() { enable_solenoid(extruder.active); }

  void Tools::disable_all_solenoids() {
    OUT_WRITE(SOL0_PIN, LOW);
    #if HAS_SOLENOID_1 && EXTRUDERS > 1
      OUT_WRITE(SOL1_PIN, LOW);
    #endif
    #if HAS_SOLENOID_2 && EXTRUDERS > 2
      OUT_WRITE(SOL2_PIN, LOW);
    #endif
    #if HAS_SOLENOID_3 && EXTRUDERS > 3
      OUT_WRITE(SOL3_PIN, LOW);
    #endif
    #if HAS_SOLENOID_4 && EXTRUDERS > 4
      OUT_WRITE(SOL4_PIN, LOW);
    #endif
    #if HAS_SOLENOID_5 && EXTRUDERS > 5
      OUT_WRITE(SOL5_PIN, LOW);
    #endif
  }

#endif

/** Private Function */
void Tools::invalid_extruder_error(const uint8_t e) {
  SERIAL_SMV(ER, "T", (int)e);
  SERIAL_EM(" " MSG_INVALID_EXTRUDER);
}

void Tools::fast_line_to_current(const AxisEnum fr_axis) {
  planner.buffer_line(mechanics.current_position, mechanics.data.max_feedrate_mm_s[fr_axis], extruder.active);
}

#if ENABLED(VOLUMETRIC_EXTRUSION)

  /**
   * Get a volumetric multiplier from a filament diameter.
   * This is the reciprocal of the circular cross-section area.
   * Return 1.0 with volumetric off or a diameter of 0.0.
   */
  float Tools::calculate_volumetric_multiplier(const float diameter) {
    return (printer.isVolumetric() && diameter) ? RECIPROCAL(CIRCLE_AREA(diameter * 0.5)) : 1.0;
  }

  /**
   * Convert the filament sizes into volumetric multipliers.
   * The multiplier converts a given E value into a length.
   */
  void Tools::calculate_volumetric_multipliers() {
    for (uint8_t e = 0; e < EXTRUDERS; e++) {
      volumetric_multiplier[e] = calculate_volumetric_multiplier(filament_size[e]);
      refresh_e_factor(e);
    }
  }

#endif // ENABLED(VOLUMETRIC_EXTRUSION)

#if ENABLED(MKSE6)

  void Tools::MK_multi_tool_change(const uint8_t e) {

    planner.synchronize(); // Finish all movement

    const int angles[EXTRUDERS] = ARRAY_BY_EXTRUDERS_N (
      MKSE6_SERVOPOS_E0, MKSE6_SERVOPOS_E1,
      MKSE6_SERVOPOS_E2, MKSE6_SERVOPOS_E3,
      MKSE6_SERVOPOS_E4, MKSE6_SERVOPOS_E5
    );
    MOVE_SERVO(MKSE6_SERVO_INDEX, angles[e]);

    #if (MKSE6_SERVO_DELAY > 0)
      HAL::delayMilliseconds(MKSE6_SERVO_DELAY);
    #endif

    // Set the new active extruder
    extruder.previous = extruder.active;
    extruder.active = e;
  }

#elif ENABLED(MKR4)

  void Tools::MK_multi_tool_change(const uint8_t e) {

    planner.synchronize(); // Finish all movement
    stepper.disable_E();

    #if (EXTRUDERS == 4) && HAS_E0E2 && HAS_E1E3 && (DRIVER_EXTRUDERS == 2)

      switch (e) {
        case 0:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          WRITE_RELE(E1E3_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 1:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          WRITE_RELE(E1E3_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 2:
          WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
          WRITE_RELE(E1E3_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 3:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          WRITE_RELE(E1E3_CHOICE_PIN, HIGH);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
      }

    #elif (EXTRUDERS == 3) && HAS_E0E2 && (DRIVER_EXTRUDERS == 2)

      switch (e) {
        case 0:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 1:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 2:
          WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
      }

    #elif (EXTRUDERS == 2) && HAS_E0E1 && (DRIVER_EXTRUDERS == 1)

      switch (e) {
        case 0:
          WRITE_RELE(E0E1_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 1:
          WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
      }

    #endif // E0E1_CHOICE_PIN E0E2_CHOICE_PIN E1E3_CHOICE_PIN

    // Set the new active extruder
    extruder.previous = extruder.active;
    extruder.active = e;
  }

#elif ENABLED(MKR6) || ENABLED(MKR12)

  void Tools::MK_multi_tool_change(const uint8_t e) {

    planner.synchronize(); // Finish all movement
    stepper.disable_E();

    #if (EXTRUDERS == 2) && HAS_EX1 && (DRIVER_EXTRUDERS == 1)

      switch (e) {
        case 0:
          WRITE_RELE(EX1_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 1:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
      }

    #elif (EXTRUDERS == 3) && HAS_EX1 && HAS_EX2 && (DRIVER_EXTRUDERS == 1)

      switch (e) {
        case 0:
          WRITE_RELE(EX1_CHOICE_PIN, LOW);
          WRITE_RELE(EX2_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 1:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          WRITE_RELE(EX2_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 2:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          WRITE_RELE(EX2_CHOICE_PIN, HIGH);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
      }

    #elif (EXTRUDERS > 3) && HAS_EX1 && HAS_EX2

      uint8_t multiply = e, driver;

      for (driver = 0; driver < DRIVER_EXTRUDERS; driver++) {
        if (multiply < 3) break;
        multiply -= 3;
      }

      switch (multiply) {
        case 0:
          WRITE_RELE(EX1_CHOICE_PIN, LOW);
          WRITE_RELE(EX2_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 1:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          WRITE_RELE(EX2_CHOICE_PIN, LOW);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        case 2:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          WRITE_RELE(EX2_CHOICE_PIN, HIGH);
          HAL::delayMilliseconds(500); // 500 microseconds delay for relay
          stepper.enable_E();
          break;
        default:
          SERIAL_LM(ER, "More Driver Extruders");
          break;
      }

    #endif

    // Set the new active extruder
    extruder.previous = extruder.active;
    extruder.active = e;
  }

#endif

#if HAS_DONDOLO

  void Tools::move_extruder_servo(const uint8_t e) {
    planner.synchronize();
    MOVE_SERVO(DONDOLO_SERVO_INDEX, servo[DONDOLO_SERVO_INDEX].angle[e]);
    #if (DONDOLO_SERVO_DELAY > 0)
      HAL::delayMilliseconds(DONDOLO_SERVO_DELAY);
    #endif
  }

#endif

#if ENABLED(DUAL_X_CARRIAGE)

  void Tools::dualx_tool_change(const uint8_t tmp_extruder, bool &no_move) {

    if (printer.debugFeature()) {
      DEBUG_MSG("Dual X Carriage Mode ");
      switch (mechanics.dual_x_carriage_mode) {
        case DXC_FULL_CONTROL_MODE:       DEBUG_EM("DXC_FULL_CONTROL_MODE");        break;
        case DXC_AUTO_PARK_MODE:          DEBUG_EM("DXC_AUTO_PARK_MODE");           break;
        case DXC_DUPLICATION_MODE:        DEBUG_EM("DXC_DUPLICATION_MODE");         break;
        case DXC_SCALED_DUPLICATION_MODE: DEBUG_EM("DXC_SCALED_DUPLICATION_MODE");  break;
      }
    }

    const float xhome = mechanics.x_home_pos(extruder.active);
    if (mechanics.dual_x_carriage_mode == DXC_AUTO_PARK_MODE
        && printer.isRunning()
        && (mechanics.delayed_move_ms || mechanics.current_position[X_AXIS] != xhome)
    ) {
      float raised_z = mechanics.current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT;
      #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
        NOMORE(raised_z, endstops.soft_endstop[Z_AXIS].max);
      #endif
      if (printer.debugFeature()) {
        DEBUG_EMV("Raise to ", raised_z);
        DEBUG_EMV("MoveX to ", xhome);
        DEBUG_EMV("Lower to ", mechanics.current_position[Z_AXIS]);
      }
      // Park old head: 1) raise 2) move to park position 3) lower
      #define CUR_X mechanics.current_position[X_AXIS]
      #define CUR_Y mechanics.current_position[Y_AXIS]
      #define CUR_Z mechanics.current_position[Z_AXIS]
      #define CUR_E mechanics.current_position[E_AXIS]

      planner.buffer_line( CUR_X, CUR_Y, raised_z, CUR_E, mechanics.data.max_feedrate_mm_s[Z_AXIS], extruder.active);
      planner.buffer_line( xhome, CUR_Y, raised_z, CUR_E, mechanics.data.max_feedrate_mm_s[X_AXIS], extruder.active);
      planner.buffer_line( xhome, CUR_Y, CUR_Z,    CUR_E, mechanics.data.max_feedrate_mm_s[Z_AXIS], extruder.active);

      planner.synchronize();
    }

    // apply Y & Z extruder offset (x offset is already used in determining home pos)
    mechanics.current_position[Y_AXIS] -= data.hotend_offset[Y_AXIS][extruder.active] - data.hotend_offset[Y_AXIS][tmp_extruder];
    mechanics.current_position[Z_AXIS] -= data.hotend_offset[Z_AXIS][extruder.active] - data.hotend_offset[Z_AXIS][tmp_extruder];

    // Activate the new extruder
    extruder.active = tmp_extruder;

    // This function resets the max/min values - the current position may be overwritten below.
    mechanics.set_axis_is_at_home(X_AXIS);

    if (printer.debugFeature()) DEBUG_POS("New Extruder", mechanics.current_position);

    // Only when auto-parking are carriages safe to move
    if (mechanics.dual_x_carriage_mode != DXC_AUTO_PARK_MODE) no_move = true;

    switch (mechanics.dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
        // New current position is the position of the activated hotend
        mechanics.current_position[X_AXIS] = mechanics.inactive_extruder_x_pos;
        // Save the inactive hotend's position (from the old mechanics.current_position)
        mechanics.inactive_extruder_x_pos = mechanics.destination[X_AXIS];
        break;
      case DXC_AUTO_PARK_MODE:
        // record raised toolhead position for use by unpark
        COPY_ARRAY(mechanics.raised_parked_position, mechanics.current_position);
        mechanics.raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
        #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
          NOMORE(mechanics.raised_parked_position[Z_AXIS], endstops.soft_endstop[Z_AXIS].max);
        #endif
        mechanics.active_extruder_parked = true;
        mechanics.delayed_move_ms = 0;
        break;
    }

    if (printer.debugFeature()) {
      DEBUG_EMV("Active hotend parked: ", mechanics.active_extruder_parked ? "yes" : "no");
      DEBUG_POS("New hotend (parked)", mechanics.current_position);
    }

  }

#endif // DUAL_X_CARRIAGE

#endif // EXTRUDERS > 0

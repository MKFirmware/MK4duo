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
 * toolmanager.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

ToolManager toolManager;

/** Public Parameters */
tool_data_t ToolManager::extruder;

#if ENABLED(VOLUMETRIC_EXTRUSION)
  float ToolManager::volumetric_area_nominal = CIRCLE_AREA(float(DEFAULT_NOMINAL_FILAMENT_DIA) * 0.5f);
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  bool  ToolManager::IDLE_OOZING_enabled = true,
        ToolManager::IDLE_OOZING_retracted[MAX_EXTRUDER] = { false };
#endif

/** Public Function */
void ToolManager::create_object() {
  #if MAX_EXTRUDER > 0
    LOOP_EXTRUDER() {
      if (!extruders[e]) {
        extruders[e] = new Extruder();
        extruder_factory_parameters(e);
        SERIAL_LMV(ECHO, "Create E", int(e));
      }
    }
  #endif
}

void ToolManager::factory_parameters() {

  extruder.total  = EXTRUDERS;
  extruder.active = extruder.previous = extruder.target = 0;

  create_object();

  LOOP_EXTRUDER() if (extruders[e]) extruder_factory_parameters(e);

  #if ENABLED(VOLUMETRIC_EXTRUSION)
    setVolumetric(VOLUMETRIC_DEFAULT_ON);
  #endif

  #if ENABLED(IDLE_OOZING_PREVENT)
    IDLE_OOZING_enabled = true;
  #endif

}

void ToolManager::change_number_extruder(const uint8_t e) {

  if (extruder.total < e) {
    extruder.total = e;
    create_object();
  }
  else if (extruder.total > e) {
    for (uint8_t ee = e; ee < MAX_EXTRUDER; ee++) {
      if (extruders[ee]) {
        SERIAL_LMV(ECHO, "Delete extruder ", ee);
        delete (extruders[ee]);
        extruders[ee] = nullptr;
      }
    }
    extruder.total = e;
  }

}

uint8_t ToolManager::active_hotend() { return extruders[extruder.active]->get_hotend(); }

uint8_t ToolManager::target_hotend() { return extruders[extruder.target]->get_hotend(); }

void ToolManager::change(const uint8_t new_tool, bool no_move/*=false*/) {

  extruder.target = new_tool;

  #if ENABLED(COLOR_MIXING_EXTRUDER)

    UNUSED(no_move);

    if (extruder.target >= MIXING_VIRTUAL_TOOLS)
      return invalid_extruder_error();

    #if MIXING_VIRTUAL_TOOLS > 1
      // T0-Tnnn: Switch virtual tool by changing the index to the mix
      mixer.T(extruder.target);
    #endif

  #elif HAS_MMU2

    UNUSED(no_move);

    mmu2.tool_change(extruder.target);

  #elif MAX_EXTRUDER < 2

    UNUSED(no_move);

    if (extruder.target) invalid_extruder_error();
    return;

  #else // MAX_EXTRUDER > 1

    planner.synchronize();
  
    #if ENABLED(DUAL_X_CARRIAGE)  // Only T0 allowed if the Printer is in DXC_DUPLICATION_MODE or DXC_MIRRORED_MODE
      // Only T0 allowed in DXC_DUPLICATION_MODE
      if (extruder.target != 0 && mechanics.dxc_is_duplicating())
         return invalid_extruder_error();
    #endif

    if (extruder.target >= extruder.total)
      return invalid_extruder_error();

    if (!no_move && mechanics.axis_unhomed_error()) {
      no_move = true;
      if (printer.debugFeature()) DEBUG_EM("No move (not homed)");
    }

    #if HAS_LCD_MENU
      if (!no_move) lcdui.return_to_status();
    #endif

    #if ENABLED(DUAL_X_CARRIAGE)
      const bool idex_full_control = mechanics.dual_x_carriage_mode == DXC_FULL_CONTROL_MODE;
    #else
      constexpr bool idex_full_control = false;
    #endif

    const bool can_move_away = !no_move && !idex_full_control;

    #if ENABLED(TOOL_CHANGE_FIL_SWAP)
      const bool should_swap = can_move_away && extruders[extruder.active]->data.swap_length;
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        const bool too_cold = !printer.debugDryrun() && (tempManager.tooColdToExtrude(active_hotend()) || tempManager.tooColdToExtrude(target_hotend()));
      #else
        constexpr bool too_cold = false;
      #endif
      if (should_swap) {
        if (too_cold) {
          SERIAL_LM(ER, MSG_HOST_HOTEND_TOO_COLD);
          extruder.previous = extruder.active;
          extruder.active   = extruder.target;
          return;
        }
        else {
          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            advancedpause.do_pause_e_move(-extruders[extruder.active]->data.swap_length, MMM_TO_MMS(extruders[extruder.active]->data.retract_speed));
          #else
            mechanics.position.e -= extruders[extruder.active]->data.swap_length / extruders[extruder.active]->e_factor;
            planner.buffer_line(mechanics.position, MMM_TO_MMS(extruders[extruder.active]->data.retract_speed), extruder.active);
            planner.synchronize();
          #endif
        }
      }
    #endif // TOOL_CHANGE_FIL_SWAP

    #if HAS_LEVELING
      // Set current position to the physical position
      bedlevel.set_bed_leveling_enabled(false);
    #endif

    if (extruder.target != extruder.active) {

      if (parser.linearval('F') > 0)
        mechanics.feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());
      else
        REMEMBER(fr, mechanics.feedrate_mm_s, XY_PROBE_FEEDRATE_MM_S);

      #if HAS_SOFTWARE_ENDSTOPS
        endstops.update_software_endstops(X_AXIS);
        #if DISABLED(DUAL_X_CARRIAGE)
          endstops.update_software_endstops(Y_AXIS);
          endstops.update_software_endstops(Z_AXIS);
        #endif
      #endif

      mechanics.destination = mechanics.position;

      #if !HAS_DONDOLO
        // Do a small lift to avoid the workpiece in the move back (below)
        if (can_move_away) {
          #if ENABLED(TOOL_CHANGE_PARK)
            nozzle.park(2);
          #else
            // Do a small lift to avoid the workpiece in the move back (below)
            mechanics.position.z += nozzle.data.park_point.z;
            #if HAS_SOFTWARE_ENDSTOPS
              endstops.apply_motion_limits(mechanics.position);
            #endif
            fast_line_to_current(Z_AXIS);
            planner.synchronize();
          #endif
        }
      #endif

      xyz_pos_t diff = nozzle.data.hotend_offset[target_hotend()] - nozzle.data.hotend_offset[active_hotend()];
      #if ENABLED(DUAL_X_CARRIAGE)
        diff.x = 0;
      #endif

      #if ENABLED(DUAL_X_CARRIAGE)
        dualx_tool_change(no_move); // Can modify no_move
      #elif HAS_DONDOLO
        if (!no_move) {
          #if HAS_SOFTWARE_ENDSTOPS
            const float maxz = MIN(endstops.soft_endstop.max.z, mechanics.data.base_pos.max.z);
          #else
            constexpr float maxz = mechanics.data.base_pos.max.z;
          #endif

          // Check if Z has space to compensate at least z_offset, and if not, just abort now
          const float newz = mechanics.position.z + MAX(-diff.z, 0.0);
          if (newz > maxz) return;

          mechanics.position.z = MIN(newz + nozzle.data.park_point.z, maxz);
          fast_line_to_current(Z_AXIS);
        }
        move_extruder_servo();
      #elif HAS_MKMULTI_TOOLS
        MK_multi_tool_change();
      #endif

      #if DISABLED(DUAL_X_CARRIAGE)
        // Set the new active extruder
        extruder.previous = extruder.active;
        extruder.active   = extruder.target;
      #endif

      // The newly-selected extruder XYZ is actually at...
      if (printer.debugFeature()) {
        DEBUG_MV("Offset Tool XYZ by { ", diff.x);
        DEBUG_MV(", ", diff.y);
        DEBUG_MV(", ", diff.z);
        DEBUG_EM(" }");
      }
      mechanics.position += diff;

      // Tell the planner the new "current position"
      mechanics.sync_plan_position();

      #if MECH(DELTA)
        const bool safe_to_move = mechanics.position.z < mechanics.delta_clip_start_height - 1;
      #else
        constexpr bool safe_to_move = true;
      #endif

      // Return to position and lower again
      if (safe_to_move && !no_move && printer.isRunning()) {

        if (tempManager.heater.hotends == 1) {
          extruders[extruder.previous]->singlenozzle_temp = hotends[0]->deg_target();
          if (extruders[extruder.active]->singlenozzle_temp && extruders[extruder.active]->singlenozzle_temp != hotends[0]->deg_target()) {
            hotends[0]->set_target_temp(extruders[extruder.active]->singlenozzle_temp);
            #if HAS_LCD
              nozzle.set_heating_message();
            #endif
            hotends[0]->wait_for_target(true);
          }
        }

        #if ENABLED(TOOL_CHANGE_FIL_SWAP)
          if (should_swap && !too_cold) {
            #if ENABLED(ADVANCED_PAUSE_FEATURE)
              advancedpause.do_pause_e_move(extruders[extruder.active]->data.swap_length, MMM_TO_MMS(extruders[extruder.active]->data.prime_speed));
              advancedpause.do_pause_e_move(extruders[extruder.active]->data.purge_lenght, PAUSE_PARK_PURGE_FEEDRATE);
            #else
              mechanics.position.e += (extruders[extruder.active]->data.swap_length) / extruders[extruder.active]->e_factor;
              planner.buffer_line(mechanics.position, mechanics.data.max_feedrate_mm_s[E_AXIS], extruder.active);
              mechanics.position.e += (extruders[extruder.active]->data.purge_lenght) / extruders[extruder.active]->e_factor;
              planner.buffer_line(mechanics.position, MMM_TO_MMS(extruders[extruder.active]->data.prime_speed * 0.2f), extruder.active);
            #endif
            planner.synchronize();
            planner.set_e_position_mm((mechanics.destination.e = mechanics.position.e = mechanics.position.e - extruders[extruder.active]->data.purge_lenght));
          }
        #endif

        // Prevent a move outside physical bounds
        endstops.apply_motion_limits(mechanics.destination);

        // Should the nozzle move back to the old position?
        if (can_move_away) {
          #if ENABLED(TOOL_CHANGE_NO_RETURN)
            // Just move back down
            if (printer.debugFeature()) DEBUG_LM(DEB, "Move back Z only");
            mechanics.do_blocking_move_to_z(mechanics.destination.z, mechanics.data.max_feedrate_mm_s.z);
          #else
            // Move back to the original (or adjusted) position
            if (printer.debugFeature()) DEBUG_POS("Move back", mechanics.destination);
            mechanics.do_blocking_move_to(mechanics.destination);
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
          mechanics.do_blocking_move_to_z(mechanics.destination.z, mechanics.data.max_feedrate_mm_s.z);
        }
      #endif
    } // (extruder.target != extruder.active)

    planner.synchronize();

    #if ENABLED(EXT_SOLENOID)
      disable_all_solenoids();
      enable_solenoid_on_active_extruder();
    #endif

    #if HAS_LEVELING
      // Restore leveling to re-establish the logical position
      bedlevel.restore_bed_leveling_state();
    #endif

    SERIAL_LMV(ECHO, MSG_HOST_ACTIVE_EXTRUDER, (int)extruder.active);

  #endif // EXTRUDERS > 1

}

void ToolManager::print_M563() {
  SERIAL_LM(CFG, "Hotend assignment T<Tool> H<Hotend>");
  LOOP_EXTRUDER() {
    SERIAL_SMV(CFG, "  M563 T", (int)e);
    SERIAL_MV(" D", extruders[e]->data.driver);
    SERIAL_MV(" H", extruders[e]->data.hotend);
    SERIAL_EOL();
  }
}

#if ENABLED(LIN_ADVANCE)

  void ToolManager::setup_test_linadvance() {
    SERIAL_EONOFF(" Test Linear Advance", IsTestLinAdvance());
    if (IsTestLinAdvance()) {
      planner.synchronize();
      extruders[extruder.active]->data.advance_K = LIN_ADVANCE_K_START;
    }
  }

  void ToolManager::test_linadvance() {
    planner.synchronize();
    extruders[extruder.active]->data.advance_K += LIN_ADVANCE_K_FACTOR;
    SERIAL_SMV(ECHO, " Layer:", printer.currentLayer);
    SERIAL_EMV(" Lin Advance K:", extruders[extruder.active]->data.advance_K);
  }

  void ToolManager::print_M900() {
    SERIAL_LM(CFG, "Linear Advance T<Tool> K<factor>");
    LOOP_EXTRUDER() {
      SERIAL_SMV(CFG, "  M900 T", (int)e);
      SERIAL_EMV(" K", extruders[e]->data.advance_K);
    }
  }

#endif

#if ENABLED(VOLUMETRIC_EXTRUSION)

  void ToolManager::print_M200() {
    SERIAL_SM(CFG, "Filament settings");
    if (toolManager.isVolumetric())
      SERIAL_EOL();
    else
      SERIAL_EM(" Disabled");

    LOOP_EXTRUDER() {
      SERIAL_SMV(CFG, "  M200 T", (int)e);
      SERIAL_EMV(" D", extruders[e]->data.filament_size, 3);
    }
  }

  /**
   * Convert the filament sizes into volumetric multipliers.
   * The multiplier converts a given E value into a length.
   */
  void ToolManager::calculate_volumetric_multipliers() {
    LOOP_EXTRUDER() {
      extruders[e]->volumetric_multiplier = calculate_volumetric_multiplier(extruders[e]->data.filament_size);
      extruders[e]->refresh_e_factor();
    }
  }

  /**
   * Get a volumetric multiplier from a filament diameter.
   * This is the reciprocal of the circular cross-section area.
   * Return 1.0 with volumetric off or a diameter of 0.0.
   */
  float ToolManager::calculate_volumetric_multiplier(const float diameter) {
    return (toolManager.isVolumetric() && diameter) ? RECIPROCAL(CIRCLE_AREA(diameter * 0.5)) : 1.0;
  }

#endif

#if ENABLED(IDLE_OOZING_PREVENT)

  void ToolManager::IDLE_OOZING_retract(const bool retracting) {

    if (retracting && !IDLE_OOZING_retracted[extruder.active]) {

      float old_feedrate_mm_s = mechanics.feedrate_mm_s;

      mechanics.destination = mechanics.position;
      mechanics.position.e += IDLE_OOZING_LENGTH
        #if ENABLED(VOLUMETRIC_EXTRUSION)
          / volumetric_multiplier[extruder.active]
        #endif
      ;
      mechanics.feedrate_mm_s = IDLE_OOZING_FEEDRATE;
      planner.set_e_position_mm(mechanics.position.e);
      mechanics.prepare_move_to_destination();
      mechanics.feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[extruder.active] = true;
      //SERIAL_EM("-");
    }
    else if (!retracting && IDLE_OOZING_retracted[extruder.active]) {

      float old_feedrate_mm_s = mechanics.feedrate_mm_s;

      mechanics.destination = mechanics.position;
      mechanics.position.e -= (IDLE_OOZING_LENGTH + IDLE_OOZING_RECOVER_LENGTH)
        #if ENABLED(VOLUMETRIC_EXTRUSION)
          / volumetric_multiplier[extruder.active]
        #endif
      ;

      mechanics.feedrate_mm_s = IDLE_OOZING_RECOVER_FEEDRATE;
      planner.set_e_position_mm(mechanics.position.e);
      mechanics.prepare_move_to_destination();
      mechanics.feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[extruder.active] = false;
      //SERIAL_EM("+");
    }
  }

#endif

#if ENABLED(EXT_SOLENOID)

  void ToolManager::enable_solenoid(const uint8_t e) {
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
        SERIAL_LM(ER, MSG_HOST_INVALID_SOLENOID);
        break;
    }
  }

  void ToolManager::enable_solenoid_on_active_extruder() { enable_solenoid(extruder.active); }

  void ToolManager::disable_all_solenoids() {
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
void ToolManager::extruder_factory_parameters(const uint8_t e) {

  static const float  tmp_step[]        PROGMEM = DEFAULT_AXIS_STEPS_PER_UNIT_E,
                      tmp_maxfeedrate[] PROGMEM = DEFAULT_MAX_FEEDRATE_E;

  static const uint32_t tmp_maxacc[]    PROGMEM = DEFAULT_MAX_ACCELERATION_E,
                        tmp_retract[]   PROGMEM = DEFAULT_RETRACT_ACCELERATION;

  static const float    tmp_ejerk[]     PROGMEM = DEFAULT_EJERK;

  extruders[e]->data.axis_steps_per_mm          = pgm_read_float(&tmp_step[e < COUNT(tmp_step) ? e : COUNT(tmp_step) - 1]);
  extruders[e]->data.max_feedrate_mm_s          = pgm_read_float(&tmp_maxfeedrate[e < COUNT(tmp_maxfeedrate) ? e : COUNT(tmp_maxfeedrate) - 1]);
  extruders[e]->data.max_acceleration_mm_per_s2 = pgm_read_dword_near(&tmp_maxacc[e < COUNT(tmp_maxacc) ? e : COUNT(tmp_maxacc) - 1]);
  extruders[e]->data.retract_acceleration       = pgm_read_dword_near(&tmp_retract[e < COUNT(tmp_retract) ? e : COUNT(tmp_retract) - 1]);

  extruders[e]->flow_percentage     = 100;
  extruders[e]->density_percentage  = 100;
  extruders[e]->singlenozzle_temp   = 0;

  #if HAS_CLASSIC_E_JERK
    extruders[e]->data.max_jerk = pgm_read_float(&tmp_ejerk[e < COUNT(tmp_ejerk) ? e : COUNT(tmp_ejerk) - 1]);
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    extruders[e]->data.load_length   = PAUSE_PARK_FAST_LOAD_LENGTH;
    extruders[e]->data.unload_length = PAUSE_PARK_UNLOAD_LENGTH;
  #endif

  #if ENABLED(LIN_ADVANCE)
    extruders[e]->data.advance_K = LIN_ADVANCE_K;
    extruder.LA_test = false;
  #endif

  #if ENABLED(VOLUMETRIC_EXTRUSION)
    extruders[e]->volumetric_multiplier  = 1.0f;
    extruders[e]->data.filament_size     = DEFAULT_NOMINAL_FILAMENT_DIA;
    extruder.volumetric = false;
  #endif

  #if ENABLED(TOOL_CHANGE_FIL_SWAP)
    extruders[e]->data.swap_length    = TOOL_CHANGE_FIL_SWAP_LENGTH;
    extruders[e]->data.purge_lenght   = TOOL_CHANGE_FIL_SWAP_PURGE;
    extruders[e]->data.prime_speed    = TOOL_CHANGE_FIL_SWAP_PRIME_SPEED;
    extruders[e]->data.retract_speed  = TOOL_CHANGE_FIL_SWAP_RETRACT_SPEED;
  #endif

  if (extruder.total == stepper.data.drivers_e)
    extruders[e]->data.driver = e;
  else
    extruders[e]->data.driver = 0;

  if (extruder.total == tempManager.heater.hotends)
    extruders[e]->data.hotend = e;
  else
    extruders[e]->data.hotend = 0;

}

void ToolManager::invalid_extruder_error() {
  SERIAL_SMV(ER, "T", (int)extruder.target);
  SERIAL_EM(" " MSG_HOST_INVALID_EXTRUDER);
}

void ToolManager::fast_line_to_current(const AxisEnum fr_axis) {
  planner.buffer_line(mechanics.position, mechanics.data.max_feedrate_mm_s[fr_axis], extruder.active);
}

#if ENABLED(MKSE6)

  void ToolManager::MK_multi_tool_change() {

    planner.synchronize(); // Finish all movement

    const int angles[EXTRUDERS] = ARRAY_BY_EXTRUDERS_N (
      MKSE6_SERVOPOS_E0, MKSE6_SERVOPOS_E1,
      MKSE6_SERVOPOS_E2, MKSE6_SERVOPOS_E3,
      MKSE6_SERVOPOS_E4, MKSE6_SERVOPOS_E5
    );
    MOVE_SERVO(MKSE6_SERVO_INDEX, angles[extruder.target]);

    #if (MKSE6_SERVO_DELAY > 0)
      HAL::delayMilliseconds(MKSE6_SERVO_DELAY);
    #endif

  }

#elif ENABLED(MKR4)

  void ToolManager::MK_multi_tool_change() {

    planner.synchronize(); // Finish all movement
    stepper.disable_E();

    #if (EXTRUDERS == 4) && HAS_E0E2 && HAS_E1E3 && (DRIVER_EXTRUDERS == 2)

      switch (extruder.target) {
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

      switch (extruder.target) {
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

      switch (extruder.target) {
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

  }

#elif ENABLED(MKR6) || ENABLED(MKR12)

  void ToolManager::MK_multi_tool_change() {

    planner.synchronize(); // Finish all movement
    stepper.disable_E();

    #if (EXTRUDERS == 2) && HAS_EX1 && (DRIVER_EXTRUDERS == 1)

      switch (extruder.target) {
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

      switch (extruder.target) {
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

      uint8_t multiply = extruder.target, driver;

      LOOP_DRV_EXT() {
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

  }

#endif

#if HAS_DONDOLO

  void ToolManager::move_extruder_servo() {
    planner.synchronize();
    MOVE_SERVO(DONDOLO_SERVO_INDEX, servo[DONDOLO_SERVO_INDEX].angle[extruder.target]);
    #if (DONDOLO_SERVO_DELAY > 0)
      HAL::delayMilliseconds(DONDOLO_SERVO_DELAY);
    #endif
  }

#endif

#if ENABLED(DUAL_X_CARRIAGE)

  void ToolManager::dualx_tool_change(bool &no_move) {

    if (printer.debugFeature()) {
      DEBUG_MSG("Dual X Carriage Mode ");
      switch (mechanics.dual_x_carriage_mode) {
        case DXC_FULL_CONTROL_MODE: DEBUG_EM("FULL_CONTROL"); break;
        case DXC_AUTO_PARK_MODE:    DEBUG_EM("AUTO_PARK");    break;
        case DXC_DUPLICATION_MODE:  DEBUG_EM("DUPLICATION");  break;
        case DXC_MIRRORED_MODE:     DEBUG_EM("MIRRORED");     break;
      }
    }

    const float xhome = mechanics.x_home_pos(extruder.active);
    if (mechanics.dual_x_carriage_mode == DXC_AUTO_PARK_MODE
        && printer.isRunning()
        && (mechanics.delayed_move_timer.isRunning() || mechanics.position.x != xhome)
    ) {

      if (printer.debugFeature()) DEBUG_EMV("MoveX to ", xhome);

      // Park old head
      mechanics.position.x = xhome;
      planner.buffer_line(mechanics.position, mechanics.data.max_feedrate_mm_s.x, extruder.active);
      planner.synchronize();
    }

    // Activate the new extruder
    extruder.previous = extruder.active;
    extruder.active   = extruder.target;

    // This function resets the max/min values - the current position may be overwritten below.
    mechanics.set_axis_is_at_home(X_AXIS);

    if (printer.debugFeature()) DEBUG_POS("New Extruder", mechanics.position);

    switch (mechanics.dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
        // New current position is the position of the activated hotend
        mechanics.position.x = mechanics.inactive_extruder_x_pos;
        // Save the inactive hotend's position (from the old mechanics.position.x)
        mechanics.inactive_extruder_x_pos = mechanics.destination.x;
        break;
      case DXC_AUTO_PARK_MODE:
        // record raised toolhead position for use by unpark
        mechanics.raised_parked_position = mechanics.position;
        mechanics.active_extruder_parked = true;
        mechanics.delayed_move_timer.stop();
        break;
      default:
        break;
    }

    if (printer.debugFeature()) {
      DEBUG_EMV("Active hotend parked: ", mechanics.active_extruder_parked ? "yes" : "no");
      DEBUG_POS("New hotend (parked)", mechanics.position);
    }

  }

#endif // DUAL_X_CARRIAGE

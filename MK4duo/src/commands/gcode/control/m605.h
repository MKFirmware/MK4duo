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
 * mcode
 *
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(DUAL_X_CARRIAGE)

#define CODE_M605

/**
 * M605: Set dual x-carriage movement mode
 *
 *    M605    : Restore user specified DEFAULT_DUAL_X_CARRIAGE_MODE
 *    M605  S0: Full control mode. The slicer has full control over x-carriage movement
 *    M605  S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
 *    M605  S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
 *                            units x-offset and an optional differential extruder temperature of
 *                            mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
 *                            the first with a spacing of 100mm in the x direction and 2 degrees hotter.
 *    M605  S3: Enable Scaled Duplication mode. The second extruder will duplicate the first extruder's
 *              movement similar to the M605 S2 mode. However, the second extruder will be producing
 *              a mirror image of the first extruder. The initial x-offset and temperature differential are
 *              set with M605 S2 [Xnnn] [Rmmm] and then followed with a M605 S3 to start the mirrored movement.
 *    M605  W : DXC What? command.
 *
 *    Note: the X axis should be homed after changing dual x-carriage mode.
 */
inline void gcode_M605() {
  planner.synchronize();

  if (parser.seen('S')) {
    const DualXModeEnum previous_mode = mechanics.dual_x_carriage_mode;

    mechanics.dual_x_carriage_mode = (DualXModeEnum)parser.value_byte();
    mechanics.scaled_duplication_mode = false;

    if (mechanics.dual_x_carriage_mode == DXC_SCALED_DUPLICATION_MODE) {
      if (previous_mode != DXC_DUPLICATION_MODE) {
        SERIAL_MSG("Printer must be in DXC_DUPLICATION_MODE prior to \n");
        SERIAL_MSG("specifying DXC_SCALED_DUPLICATION_MODE.\n");
        mechanics.dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        return;
      }
      mechanics.scaled_duplication_mode = true;
      stepper.set_directions();
      float x_jog = mechanics.current_position.x - .1;
      for (uint8_t i = 2; --i;) {
        planner.buffer_line(x_jog, mechanics.current_position.y, mechanics.current_position.z, mechanics.current_position.e, mechanics.feedrate_mm_s, 0);
        x_jog += .1;
      }
      return;
    }

    switch (mechanics.dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      case DXC_DUPLICATION_MODE:
        if (parser.seen('X')) mechanics.duplicate_extruder_x_offset = MAX(parser.value_linear_units(), X2_MIN_POS - mechanics.x_home_pos(0));
        if (parser.seen('R')) mechanics.duplicate_extruder_temp_offset = parser.value_celsius_diff();
        if (toolManager.extruder.active != 0) toolManager.change(0);
        break;
      default:
        mechanics.dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    mechanics.active_extruder_parked = false;
    mechanics.extruder_duplication_enabled = false;
    stepper.set_directions();
    mechanics.delayed_move_timer.stop();
  }
  else if (!parser.seen('W'))  // if no S or W parameter, the DXC mode gets reset to the user's default
    mechanics.dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

  if (parser.seen('W')) {
    DEBUG_SM(DEB, "DXC mode: ");
    switch (mechanics.dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:       DEBUG_MSG("DXC_FULL_CONTROL_MODE");        break;
      case DXC_AUTO_PARK_MODE:          DEBUG_MSG("DXC_AUTO_PARK_MODE");           break;
      case DXC_DUPLICATION_MODE:        DEBUG_MSG("DXC_DUPLICATION_MODE");         break;
      case DXC_SCALED_DUPLICATION_MODE: DEBUG_MSG("DXC_SCALED_DUPLICATION_MODE");  break;
    }
    DEBUG_MV("\nActive Ext: ", int(toolManager.extruder.active));
    if (!mechanics.active_extruder_parked) DEBUG_MSG(" NOT ");
    DEBUG_EM(" parked.");
    DEBUG_MV("\nactive_extruder_x_pos: ", mechanics.current_position.x);
    DEBUG_MV("\ninactive_extruder_x_pos: ", mechanics.inactive_extruder_x_pos);
    DEBUG_MV("\nactive_extruder_x_pos: ", mechanics.current_position.x);
    DEBUG_MV("\ninactive_extruder_x_pos: ", mechanics.inactive_extruder_x_pos);
    DEBUG_MV("\nextruder_duplication_enabled: ", int(mechanics.extruder_duplication_enabled));
    DEBUG_MV("\nduplicate_extruder_x_offset: ", mechanics.duplicate_extruder_x_offset);
    DEBUG_MV("\nduplicate_extruder_temp_offset: ", mechanics.duplicate_extruder_temp_offset);
    DEBUG_MV("\ndelayed_move_time: ", mechanics.delayed_move_timer.started());
    DEBUG_MV("\nX1 Home X: ", mechanics.x_home_pos(0));
    DEBUG_MV("\nX1_MIN_POS=", int(X1_MIN_POS));
    DEBUG_MV("\nX1_MAX_POS=", int(X1_MAX_POS));
    DEBUG_MV("\nX1 Home X: ", mechanics.x_home_pos(1));
    DEBUG_MV("\nX2_MIN_POS=", int(X2_MIN_POS));
    DEBUG_MV("\nX2_MAX_POS=", int(X2_MAX_POS));
    DEBUG_MV("\nX2_HOME_DIR=", int(X2_HOME_DIR));
    DEBUG_MV("\nX2_HOME_POS=", int(X2_HOME_POS));
    DEBUG_EOL();
  }

}

#endif // ENABLED(DUAL_X_CARRIAGE)

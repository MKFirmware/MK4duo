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
 * mcode
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(DUAL_X_CARRIAGE)

#define CODE_M605

/**
 * M605: Set dual x-carriage movement mode
 *
 *   M605 S0 : (FULL_CONTROL) The slicer has full control over both X-carriages and can achieve optimal travel
 *             results as long as it supports dual X-carriages.
 *
 *   M605 S1 : (AUTO_PARK) The firmware automatically parks and unparks the X-carriages on tool-change so that
 *             additional slicer support is not required.
 *
 *   M605 S2 X R : (DUPLICATION) The firmware moves the second X-carriage and extruder in synchronization with
 *             the first X-carriage and extruder, to print 2 copies of the same object at the same time.
 *             Set the constant X-offset and temperature differential with M605 S2 X[offs] R[deg] and
 *             follow with "M605 S2" to initiate duplicated movement. For example, use "M605 S2 X100 R2" to
 *             make a copy 100mm to the right with E1 2° hotter than E0.
 *
 *   M605 S3 : (MIRRORED) Formbot/Vivedino-inspired mirrored mode in which the second extruder duplicates
 *             the movement of the first except the second extruder is reversed in the X axis.
 *             The temperature differential and initial X offset must be set with "M605 S2 X[offs] R[deg]",
 *             then followed by "M605 S3" to initiate mirrored movement.
 *
 *    M605 W  : IDEX What? command.
 *
 *    Note: the X axis should be homed after changing Dual X-carriage mode.
 */
inline void gcode_M605() {
  planner.synchronize();

  if (parser.seen('S')) {
    const DualXModeEnum previous_mode = mechanics.dual_x_carriage_mode;

    mechanics.dual_x_carriage_mode = (DualXModeEnum)parser.value_byte();
    mechanics.mirrored_duplication_mode = false;

    if (mechanics.dual_x_carriage_mode == DXC_MIRRORED_MODE) {
      if (previous_mode != DXC_DUPLICATION_MODE) {
        SERIAL_MSG("Printer must be in DXC_DUPLICATION_MODE prior to \n");
        SERIAL_MSG("specifying DXC_MIRRORED_MODE.\n");
        mechanics.dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        return;
      }
      mechanics.mirrored_duplication_mode = true;
      stepper.set_directions();
      float x_jog = mechanics.position.x - .1;
      for (uint8_t i = 2; --i;) {
        planner.buffer_line(x_jog, mechanics.position.y, mechanics.position.z, mechanics.position.e, mechanics.feedrate_mm_s, 0);
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

  if (printer.debugFeature() && parser.seen('W')) {
    DEBUG_SM(DEB, "DXC mode: ");
    switch (mechanics.dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE: DEBUG_MSG("DXC_FULL_CONTROL_MODE"); break;
      case DXC_AUTO_PARK_MODE:    DEBUG_MSG("DXC_AUTO_PARK_MODE");    break;
      case DXC_DUPLICATION_MODE:  DEBUG_MSG("DXC_DUPLICATION_MODE");  break;
      case DXC_MIRRORED_MODE:     DEBUG_MSG("DXC_MIRRORED_MODE");     break;
    }
    DEBUG_MV("\nActive Ext: ", int(toolManager.extruder.active));
    if (!mechanics.active_extruder_parked) DEBUG_MSG(" NOT ");
    DEBUG_EM(" parked.");
    DEBUG_MV("\nactive_extruder_x_pos: ", mechanics.position.x);
    DEBUG_MV("\ninactive_extruder_x_pos: ", mechanics.inactive_extruder_x_pos);
    DEBUG_MV("\nactive_extruder_x_pos: ", mechanics.position.x);
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

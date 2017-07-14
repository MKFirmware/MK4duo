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
 */

/**
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define G0
#define G1

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
void gcode_G0_G1(
  #if IS_SCARA
    bool fast_move = false
  #elif ENABLED(LASER)
    bool lfire = false
  #endif
) {
  if (printer.IsRunning()) {
    printer.get_destination_from_command(); // For X Y Z E F

    #if ENABLED(FWRETRACT)
      if (autoretract_enabled && !(parser.seen('X') || parser.seen('Y') || parser.seen('Z')) && parser.seen('E')) {
        const float echange = mechanics.destination[E_AXIS] - mechanics.current_position[E_AXIS];
        // Is this move an attempt to retract or recover?
        if ((echange < -(MIN_RETRACT) && !retracted[printer.active_extruder]) || (echange > MIN_RETRACT && retracted[printer.active_extruder])) {
          mechanics.current_position[E_AXIS] = mechanics.destination[E_AXIS]; // hide the slicer-generated retract/recover from calculations
          mechanics.sync_plan_position_e();  // AND from the planner
          fwretract(!retracted[printer.active_extruder]);
          return;
        }
      }
    #endif // FWRETRACT

    #if ENABLED(LASER) && ENABLED(LASER_FIRE_G1)
      if (lfire) {
        #if ENABLED(INTENSITY_IN_BYTE)
          if (parser.seenval('S')) laser.intensity = ((float)parser.value_byte() / 255.0) * 100.0;
        #else
          if (parser.seenval('S')) laser.intensity = parser.value_float();
        #endif
        if (parser.seen('L')) laser.duration = parser.value_ulong();
        if (parser.seen('P')) laser.ppm = parser.value_float();
        if (parser.seen('D')) laser.diagnostics = parser.value_bool();
        if (parser.seen('B')) laser.set_mode(parser.value_int());

        laser.status = LASER_ON;
      }
    #endif

    #if IS_SCARA
      fast_move ? prepare_uninterpolated_move_to_destination() : mechanics.prepare_move_to_destination();
    #else
      mechanics.prepare_move_to_destination();
    #endif

    #if ENABLED(LASER) && ENABLED(LASER_FIRE_G1)
      if (lfire) laser.status = LASER_OFF;
    #endif

  }
}

inline void gcode_G0(void) {
  #if IS_SCARA
    gcode_G0_G1(true);
  #elif ENABLED(LASER)
    gcode_G0_G1(false);
  #else
    gcode_G0_G1();
  #endif
}

inline void gcode_G1(void) {
  #if IS_SCARA
    gcode_G0_G1(false);
  #elif ENABLED(LASER)
    gcode_G0_G1(true);
  #else
    gcode_G0_G1();
  #endif
}

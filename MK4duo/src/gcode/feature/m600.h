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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  #define CODE_M600

  /**
   * M600: Pause Park and filament change
   *
   *  E[distance] - Retract the filament this far (negative value)
   *  Z[distance] - Move the Z axis by this distance
   *  X[position] - Move to this X position, with Y
   *  Y[position] - Move to this Y position, with X
   *  U[distance] - Retract distance for removal (negative value) (manual reload)
   *  L[distance] - Extrude distance for insertion (positive value) (manual reload)
   *  S[temp]     - New temperature for new filament
   *  B[count]    - Number of times to beep, -1 for indefinite (if equipped with a buzzer)
   *
   *  Default values are used for omitted arguments.
   *
   */
  inline void gcode_M600(void) {

    // Homing first
    if (mechanics.axis_unhomed_error()) mechanics.Home(true);

    // Initial retract before move to pause park position
    const float retract = parser.seen('E') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
        - (PAUSE_PARK_RETRACT_LENGTH)
      #endif
    ;

    // Second retract after cooldown hotend
    const float retract2 = 0.0
      #if ENABLED(PAUSE_PARK_RETRACT_2_LENGTH) && PAUSE_PARK_RETRACT_2_LENGTH > 0
        - (PAUSE_PARK_RETRACT_2_LENGTH)
      #endif
    ;

    // Lift Z axis
    const float z_lift = parser.linearval('Z', 0
      #if ENABLED(PAUSE_PARK_Z_ADD) && PAUSE_PARK_Z_ADD > 0
        + PAUSE_PARK_Z_ADD
      #endif
    );

    // Move XY axes to filament exchange position
    const float x_pos = parser.linearval('X', 0
      #if ENABLED(PAUSE_PARK_X_POS)
        + PAUSE_PARK_X_POS
      #endif
    );
    const float y_pos = parser.linearval('Y', 0
      #if ENABLED(PAUSE_PARK_Y_POS)
        + PAUSE_PARK_Y_POS
      #endif
    );

    // Unload filament
    const float unload_length = parser.seen('U') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_UNLOAD_LENGTH) && PAUSE_PARK_UNLOAD_LENGTH > 0
        - (PAUSE_PARK_UNLOAD_LENGTH)
      #endif
    ;

    // Load filament
    const float load_length = parser.seen('L') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_LOAD_LENGTH)
        + PAUSE_PARK_LOAD_LENGTH
      #endif
    ;

    int16_t temp = 0;
    if (parser.seenval('S')) temp = parser.value_celsius();

    const int beep_count = parser.intval('B',
      #if ENABLED(PAUSE_PARK_NUMBER_OF_ALERT_BEEPS)
        PAUSE_PARK_NUMBER_OF_ALERT_BEEPS
      #else
        -1
      #endif
    );

    const bool job_running = print_job_counter.isRunning();

    if (pause_print(retract, retract2, z_lift, x_pos, y_pos, unload_length, temp, beep_count, true)) {
      wait_for_filament_reload(beep_count);
      resume_print(load_length, PAUSE_PARK_EXTRUDE_LENGTH, beep_count);
    }

    // Resume the print job timer if it was running
    if (job_running) print_job_counter.start();

  }

#endif // ENABLED(ADVANCED_PAUSE_FEATURE)

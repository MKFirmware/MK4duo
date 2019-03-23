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
 * mcode
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(HYSTERESIS_FEATURE)

  #define CODE_M99

  /**
   * M99: Set Hysteresis value
   *  F[float] Enable/disable/fade-out hysteresis correction (0.0 to 1.0)
   *  X[float] Sets the hysteresis distance on X (0 to disable)
   *  Y[float] Sets the hysteresis distance on Y (0 to disable)
   *  Z[float] Sets the hysteresis distance on Z (0 to disable)
   *
   */
  inline void gcode_M99(void) {

    LOOP_XYZ(axis) {
      if (parser.seen(axis_codes[axis]))
        planner.hysteresis_mm[axis] = parser.value_float();
    }

    if (parser.seen('F'))
      planner.hysteresis_correction = MAX(0, MIN(1.0, parser.value_float()));

    if (planner.hysteresis_correction > 0)
      SERIAL_EM("Hysteresis correction is active:");
    else
      SERIAL_EM("Hysteresis correction is inactive:");

    SERIAL_MV(" Correction Amount/Fade-out: F", planner.hysteresis_correction);
    SERIAL_EM(" (F1.0 = full, F0.0 = none)");
    SERIAL_MSG("  Hysteresis Distance (mm): ");
    SERIAL_MV(" X", planner.hysteresis_mm[X_AXIS]);
    SERIAL_MV(" Y", planner.hysteresis_mm[Y_AXIS]);
    SERIAL_MV(" Z", planner.hysteresis_mm[Z_AXIS]);
    SERIAL_EOL();

  }

#endif // HYSTERESIS_FEATURE

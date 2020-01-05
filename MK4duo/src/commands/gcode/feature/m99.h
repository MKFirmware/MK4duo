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
inline void gcode_M99() {

  LOOP_XYZ(axis) {
    if (parser.seen(axis_codes[axis]))
      hysteresis.data.mm[axis] = parser.value_float();
  }

  if (parser.seen('F'))
    hysteresis.data.correction = MAX(0, MIN(1.0, parser.value_float()));

  SERIAL_MSG("Hysteresis correction is ");
  if (hysteresis.data.correction == 0) SERIAL_MSG("in");
  SERIAL_EM("active:");

  SERIAL_MV(" Correction Amount/Fade-out: F", hysteresis.data.correction);
  SERIAL_EM(" (F1.0 = full, F0.0 = none)");
  SERIAL_MSG("  Hysteresis Distance (mm): ");
  SERIAL_MV(" X", hysteresis.data.mm[X_AXIS]);
  SERIAL_MV(" Y", hysteresis.data.mm[Y_AXIS]);
  SERIAL_MV(" Z", hysteresis.data.mm[Z_AXIS]);
  SERIAL_EOL();

}

#endif // HYSTERESIS_FEATURE

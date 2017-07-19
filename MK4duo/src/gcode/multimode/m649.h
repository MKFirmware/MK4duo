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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(LASER)

  #define CODE_M649

  // M649 set laser options
  inline void gcode_M649(void) {
    // do this at the start so we can debug if needed!
    if (parser.seen('D') && IsRunning()) laser.diagnostics = parser.value_bool();

    // Wait for the rest
    // stepper.synchronize();
    if (parser.seen('S') && IsRunning()) {
      laser.intensity = parser.value_float();
      laser.rasterlaserpower =  laser.intensity;
    }

    if (IsRunning()) {
      if (parser.seen('L')) laser.duration = parser.value_ulong();
      if (parser.seen('P')) laser.ppm = parser.value_float();
      if (parser.seen('B')) laser.set_mode(parser.value_int());
      if (parser.seen('R')) laser.raster_mm_per_pulse = (parser.value_float());
    }

    if (parser.seen('F')) {
      float next_feedrate = parser.value_linear_units();
      if (next_feedrate > 0.0) mechanics.feedrate_mm_s = next_feedrate;
    }
  }

#endif // ENABLED(LASER)
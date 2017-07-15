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

#if ENABLED(LASER) && ENABLED(LASER_RASTER)

  #define CODE_G7

  inline void gcode_G7(void) {

    if (parser.seenval('L')) laser.raster_raw_length = parser.value_int();

    if (parser.seenval('$')) {
      laser.raster_direction = parser.value_int();
      mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] + (laser.raster_mm_per_pulse * laser.raster_aspect_ratio); // increment Y axis
    }

    if (parser.seenval('@')) {
      laser.raster_direction = parser.value_int();
      #if ENABLED(LASER_RASTER_MANUAL_Y_FEED)
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS]; // Dont increment X axis
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS]; // Dont increment Y axis
      #else
        switch(laser.raster_direction) {
          case 0:
          case 1:
          case 4:
            mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] + (laser.raster_mm_per_pulse * laser.raster_aspect_ratio); // increment Y axis
          break;
          case 2:
          case 3:
          case 5:
            mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] + (laser.raster_mm_per_pulse * laser.raster_aspect_ratio); // increment X axis
          break;
        }
      #endif
    }

    if (parser.seen('D')) laser.raster_num_pixels = base64_decode(laser.raster_data, parser.string_arg + 1, laser.raster_raw_length);

    switch (laser.raster_direction) {
      case 0: // Negative X
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] - (laser.raster_mm_per_pulse * laser.raster_num_pixels);
        if (laser.diagnostics) SERIAL_EM("Negative Horizontal Raster Line");
      break;
      case 1: // Positive X
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] + (laser.raster_mm_per_pulse * laser.raster_num_pixels);
        if (laser.diagnostics) SERIAL_EM("Positive Horizontal Raster Line");
      break;
      case 2: // Negative Vertical
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] - (laser.raster_mm_per_pulse * laser.raster_num_pixels);
        if (laser.diagnostics) SERIAL_EM("Negative Vertical Raster Line");
      break;
      case 3: // Positive Vertical
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] + (laser.raster_mm_per_pulse * laser.raster_num_pixels);
        if (laser.diagnostics) SERIAL_EM("Positive Vertical Raster Line");
      break;
      case 4: // Negative X Positive Y 45deg
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] - ((laser.raster_mm_per_pulse * laser.raster_num_pixels) * 0.707106);
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] + ((laser.raster_mm_per_pulse * laser.raster_num_pixels) * 0.707106);
        if (laser.diagnostics) SERIAL_EM("Negative X Positive Y 45deg Raster Line");
      break;
      case 5: // Positive X Negarite Y 45deg
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] + ((laser.raster_mm_per_pulse * laser.raster_num_pixels) * 0.707106);
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] - ((laser.raster_mm_per_pulse * laser.raster_num_pixels) * 0.707106);
        if (laser.diagnostics) SERIAL_EM("Positive X Negarite Y 45deg Raster Line");
      break;
      default:
        if (laser.diagnostics) SERIAL_EM("Unknown direction");
      break;
    }

    laser.ppm = 1 / laser.raster_mm_per_pulse; // number of pulses per millimetre
    laser.duration = (1000000 / mechanics.feedrate_mm_s) / laser.ppm; // (1 second in microseconds / (time to move 1mm in microseconds)) / (pulses per mm) = Duration of pulse, taking into account mechanics.feedrate_mm_s as speed and ppm

    laser.mode = RASTER;
    laser.status = LASER_ON;
    mechanics.prepare_move_to_destination();
  }

#endif

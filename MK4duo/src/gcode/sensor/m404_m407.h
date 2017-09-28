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

#if ENABLED(FILAMENT_SENSOR)

  #define CODE_M404
  #define CODE_M405
  #define CODE_M406
  #define CODE_M407

  /**
   * M404: Display or set (in current units) the nominal filament width (3mm, 1.75mm ) W<3.0>
   */
  inline void gcode_M404(void) {
    if (parser.seen('W')) {
      filament_width_nominal = parser.value_linear_units();
    }
    else {
      SERIAL_EMV("Filament dia (nominal mm):", filament_width_nominal);
    }
  }

  /**
   * M405: Turn on filament sensor for control
   */
  inline void gcode_M405(void) {
    // This is technically a linear measurement, but since it's quantized to centimeters and is a different unit than
    // everything else, it uses parser.value_int() instead of parser.value_linear_units().
    if (parser.seen('D')) meas_delay_cm = parser.value_byte();
    NOMORE(meas_delay_cm, MAX_MEASUREMENT_DELAY);

    if (filwidth_delay_index[1] == -1) { // Initialize the ring buffer if not done since startup
      const uint8_t temp_ratio = thermalManager.widthFil_to_size_ratio() - 100; // -100 to scale within a signed byte

      for (uint8_t i = 0; i < COUNT(measurement_delay); ++i)
        measurement_delay[i] = temp_ratio;

      filwidth_delay_index[0] = filwidth_delay_index[1] = 0;
    }

    filament_sensor = true;

    //SERIAL_MV("Filament dia (measured mm):", filament_width_meas);
    //SERIAL_EMV("Extrusion ratio(%):", tools.flow_percentage[tools.active_extruder]);
  }

  /**
   * M406: Turn off filament sensor for control
   */
  inline void gcode_M406(void) {
    filament_sensor = false;
    printer.calculate_volumetric_multipliers();   // Restore correct 'volumetric_multiplier' value
  }

  /**
   * M407: Get measured filament diameter on serial output
   */
  inline void gcode_M407(void) {
    SERIAL_EMV("Filament dia (measured mm):", filament_width_meas);
  }

#endif // ENABLED(FILAMENT_SENSOR)

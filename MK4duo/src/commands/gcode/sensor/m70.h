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

#if HAS_POWER_CONSUMPTION_SENSOR

  #define CODE_M70

  /**
   * M70 - Power consumption sensor calibration
   *
   * Z - Calibrate zero current offset
   * A - Isert readed DC Current value (Ampere)
   * W - Insert readed AC Wattage value (Watt)
   */
  inline void gcode_M70(void) {
    if (parser.seen('Z')) {
      SERIAL_EMV("Actual POWER_ZERO:", POWER_ZERO, 7);
      SERIAL_EMV("New POWER_ZERO:", powerManager.raw_analog2voltage(), 7);
      SERIAL_EM("Insert new calculated values into the FW and call \"M70 A\" for the next calibration step.");
    }
    else if (parser.seen('A')) {
      SERIAL_EMV("Actual POWER_ERROR:", POWER_ERROR, 7);
      SERIAL_EMV("New POWER_ERROR:", powerManager.analog2error(parser.value_float()), 7);
      SERIAL_EM("Insert new calculated values into the FW and call \"M70 W\" for the last calibration step.");
    }
    else if (parser.seen('W')) {
      SERIAL_EMV("Actual POWER_EFFICIENCY:", POWER_EFFICIENCY, 7);
      SERIAL_EMV("New POWER_EFFICIENCY:", powerManager.analog2efficiency(parser.value_float()), 7);
      SERIAL_EM("Insert new calculated values into the FW and then ACS712 it should be calibrated correctly.");
    }
  }

#endif // HAS_POWER_CONSUMPTION_SENSOR

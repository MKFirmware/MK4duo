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

#if HAS_EXT_ENCODER

  //#define CODE_M602
  #define CODE_M604

  /**
   * M602: Enable or disable Extruder Encoder
   */

  /**
   * M604: Set data Extruder Encoder
   *
   *  S[step] - Set Error Steps
   *
   */
  inline void gcode_M604(void) {
    GET_TARGET_EXTRUDER(604);
    stepper.synchronize();
    extruder.encErrorSteps[extruder.target] = parser.intval('S', ENC_ERROR_STEPS);
    SERIAL_EMV("Encoder Error Steps: ", extruder.encErrorSteps[extruder.target]);
  }

#endif // HAS_EXT_ENCODER

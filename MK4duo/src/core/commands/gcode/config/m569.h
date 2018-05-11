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

#define CODE_M569

/**
 * M569: Set Stepper direction
 */
inline void gcode_M569(void) {

  GET_TARGET_EXTRUDER(569);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      stepper.setStepDir((AxisEnum)a, parser.value_bool());
    }
  }

  SERIAL_EM("Reporting Stepper Direction");
  SERIAL_MT(" X dir:", stepper.isStepDir(X_AXIS) ? "true" : "false");
  SERIAL_MT(" Y dir:", stepper.isStepDir(Y_AXIS) ? "true" : "false");
  SERIAL_MV(" Z dir:", stepper.isStepDir(Z_AXIS) ? "true" : "false");

  #if EXTRUDERS == 1
    SERIAL_EMV(" E dir:", stepper.isStepDir(E_AXIS) ? "true" : "false");
  #endif

  #if EXTRUDERS > 1
    for (int8_t i = 0; i < EXTRUDERS; i++) {
      SERIAL_MV(" Extruder:", (int)i);
      SERIAL_MT(" dir:" , stepper.isStepDir((AxisEnum)(E_AXIS + i)) ? "true" : "false");
    }
  #endif

  SERIAL_EOL();
}

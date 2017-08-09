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

#define CODE_M115

/**
 * M115: Capabilities string
 */
inline void gcode_M115(void) {

  SERIAL_EM(MSG_M115_REPORT);

  #if ENABLED(EXTENDED_CAPABILITIES_REPORT)

    // EEPROM (M500, M501)
    #if ENABLED(EEPROM_SETTINGS)
      SERIAL_LM(CAP, "EEPROM:1");
    #else
      SERIAL_LM(CAP, "EEPROM:0");
    #endif

    // AUTOREPORT_TEMP (M155)
    #if ENABLED(AUTO_REPORT_TEMPERATURES)
      SERIAL_LM(CAP, "AUTOREPORT_TEMP:1");
    #else
      SERIAL_LM(CAP, "AUTOREPORT_TEMP:0");
    #endif

    // PROGRESS (M530 S L, M531 <file>, M532 X L)
    SERIAL_LM(CAP, "PROGRESS:1");

    // Print Job timer M75, M76, M77
    SERIAL_LM(CAP, "PRINT_JOB:1");

    // AUTOLEVEL (G29)
    #if HAS_ABL
      SERIAL_LM(CAP, "AUTOLEVEL:1");
    #else
      SERIAL_LM(CAP, "AUTOLEVEL:0");
    #endif

    // Z_PROBE (G30)
    #if HAS_BED_PROBE
      SERIAL_LM(CAP, "Z_PROBE:1");
    #else
      SERIAL_LM(CAP, "Z_PROBE:0");
    #endif

    // MESH_REPORT (M320 V, M420 V)
    #if HAS_LEVELING
      SERIAL_LM(CAP, "LEVELING_DATA:1");
    #else
      SERIAL_LM(CAP, "LEVELING_DATA:0");
    #endif

    // SOFTWARE_POWER (M80, M81)
    #if HAS_POWER_SWITCH
      SERIAL_LM(CAP, "SOFTWARE_POWER:1");
    #else
      SERIAL_LM(CAP, "SOFTWARE_POWER:0");
    #endif

    // CASE LIGHTS (M355)
    #if HAS_CASE_LIGHT
      SERIAL_LM(CAP, "TOGGLE_LIGHTS:1");
    #else
      SERIAL_LM(CAP, "TOGGLE_LIGHTS:0");
    #endif

    // EMERGENCY_PARSER (M108, M112, M410)
    #if ENABLED(EMERGENCY_PARSER)
      SERIAL_LM(CAP, "EMERGENCY_PARSER:1");
    #else
      SERIAL_LM(CAP, "EMERGENCY_PARSER:0");
    #endif

  #endif // EXTENDED_CAPABILITIES_REPORT
}

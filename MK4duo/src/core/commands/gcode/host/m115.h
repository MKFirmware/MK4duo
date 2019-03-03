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

#define CODE_M115

/**
 * M115: Capabilities string
 */
inline void gcode_M115(void) {

  SERIAL_EM(MSG_M115_REPORT);

  // SERIAL_XON_XOFF
  #if ENABLED(SERIAL_XON_XOFF)
    SERIAL_CAP("SERIAL_XON_XOFF:1");
  #else
    SERIAL_CAP("SERIAL_XON_XOFF:0");
  #endif

  // EEPROM (M500, M501)
  #if ENABLED(EEPROM_SETTINGS)
    SERIAL_CAP("EEPROM:1");
  #else
    SERIAL_CAP("EEPROM:0");
  #endif

  // Volumetric Extrusion (M200)
  #if ENABLED(VOLUMETRIC_EXTRUSION)
    SERIAL_CAP("VOLUMETRIC:1");
  #else
    SERIAL_CAP("VOLUMETRIC:0");
  #endif

  // AUTOREPORT_TEMP (M155)
  SERIAL_CAP("AUTOREPORT_TEMP:1");

  // PROGRESS (M530 S L, M531 <file>, M532 X L)
  SERIAL_CAP("PROGRESS:1");

  // Print Job timer M75, M76, M77
  SERIAL_CAP("PRINT_JOB:1");

  // Command pause stop
  SERIAL_CAP("PAUSESTOP:0");

  // Prompt support
  SERIAL_CAP("PROMPT_SUPPORT:0");

  // AUTOLEVEL (G29)
  #if HAS_ABL
    SERIAL_CAP("AUTOLEVEL:1");
  #else
    SERIAL_CAP("AUTOLEVEL:0");
  #endif

  // Z_PROBE (G30)
  #if HAS_BED_PROBE
    SERIAL_CAP("Z_PROBE:1");
  #else
    SERIAL_CAP("Z_PROBE:0");
  #endif

  // MESH_REPORT (M420 V)
  #if HAS_LEVELING
    SERIAL_CAP("LEVELING_DATA:1");
  #else
    SERIAL_CAP("LEVELING_DATA:0");
  #endif

  // SOFTWARE_POWER (M80, M81)
  #if HAS_POWER_SWITCH
    SERIAL_CAP("SOFTWARE_POWER:1");
  #else
    SERIAL_CAP("SOFTWARE_POWER:0");
  #endif

  // CASE LIGHTS (M355)
  #if HAS_CASE_LIGHT
    SERIAL_CAP("TOGGLE_LIGHTS:1");
  #else
    SERIAL_CAP("TOGGLE_LIGHTS:0");
  #endif

  // EMERGENCY_PARSER (M108, M112, M410, M876)
  #if ENABLED(EMERGENCY_PARSER)
    SERIAL_CAP("EMERGENCY_PARSER:1");
  #else
    SERIAL_CAP("EMERGENCY_PARSER:0");
  #endif

}

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

#define CODE_M115

#define SERIAL_CAP_OFF(msg) host_cap(PSTR(msg))
#define SERIAL_CAP_ON(msg)  host_cap(PSTR(msg), true)
#define SERIAL_CAP(msg,ena) host_cap(PSTR(msg), ena)

static void host_cap(PGM_P const pstr, const bool ena=false) {
  SERIAL_STR(CAP);
  SERIAL_STR(pstr);
  SERIAL_CHR(':');
  SERIAL_VAL(int(ena ? 1 : 0));
  SERIAL_EOL();
}

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {

  SERIAL_EM(STR_M115_REPORT);

  // PAREN_COMMENTS
  SERIAL_CAP_ON("PAREN_COMMENTS");

  // QUOTED_STRINGS
  SERIAL_CAP_ON("QUOTED_STRINGS");

  // SERIAL_XON_XOFF
  SERIAL_CAP("SERIAL_XON_XOFF", HAS_XON_XOFF);

  // EEPROM (M500, M501)
  SERIAL_CAP("EEPROM", HAS_EEPROM);

  // Volumetric Extrusion (M200)
  SERIAL_CAP("VOLUMETRIC", HAS_VOLUMETRIC_EXTRUSION);

  // AUTOREPORT_TEMP (M155)
  SERIAL_CAP_ON("AUTOREPORT_TEMP");

  // PROGRESS (M530 S L, M531 <file>, M532 X L)
  SERIAL_CAP_ON("PROGRESS");

  // BUILD_PERCENT (M73)
  SERIAL_CAP_ON("BUILD_PERCENT");

  // Print Job timer M75, M76, M77
  SERIAL_CAP_ON("PRINT_JOB");

  // Command pause stop
  SERIAL_CAP_ON("PAUSESTOP");

  // PROMPT SUPPORT (M876)
  SERIAL_CAP_ON("PROMPT_SUPPORT");

  // AUTOLEVEL (G29)
  SERIAL_CAP("AUTOLEVEL", HAS_AUTOLEVEL);

  // Z_PROBE (G30)
  SERIAL_CAP("Z_PROBE", HAS_BED_PROBE);

  // MESH_REPORT (M420 V)
  SERIAL_CAP("LEVELING_DATA", HAS_LEVELING);

  // SOFTWARE_POWER (M80, M81)
  SERIAL_CAP("SOFTWARE_POWER", HAS_POWER_SWITCH);

  // CASE LIGHTS (M355)
  SERIAL_CAP("TOGGLE_LIGHTS", HAS_CASE_LIGHT);

  // EMERGENCY_PARSER (M108, M112, M410, M876)
  SERIAL_CAP("EMERGENCY_PARSER", HAS_EMERGENCY_PARSER);

  // SDCARD (M20, M23, M24, etc.)
  SERIAL_CAP("SDCARD", HAS_SD_SUPPORT);

  // AUTOREPORT_SD_STATUS (M27 extension)
  SERIAL_CAP("AUTOREPORT_SD_STATUS", HAS_SD_SUPPORT);

  // THERMAL_PROTECTION
  SERIAL_CAP("THERMAL_PROTECTION", HAS_THERMAL_PROTECTION);

  // BABYSTEPPING (M290)
  SERIAL_CAP("BABYSTEPPING", HAS_BABYSTEPPING);

  // CHAMBER_TEMPERATURE (M141, M191)
  SERIAL_CAP("CHAMBER_TEMPERATURE", HAS_CHAMBERS);

}

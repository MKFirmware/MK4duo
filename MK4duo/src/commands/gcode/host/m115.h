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

  SERIAL_EM(MSG_HOST_M115_REPORT);

  // SERIAL_XON_XOFF
  #if ENABLED(SERIAL_XON_XOFF)
    SERIAL_CAP_ON("SERIAL_XON_XOFF");
  #else
    SERIAL_CAP_OFF("SERIAL_XON_XOFF");
  #endif

  // EEPROM (M500, M501)
  #if ENABLED(EEPROM_SETTINGS)
    SERIAL_CAP_ON("EEPROM");
  #else
    SERIAL_CAP_OFF("EEPROM");
  #endif

  // Volumetric Extrusion (M200)
  #if ENABLED(VOLUMETRIC_EXTRUSION)
    SERIAL_CAP_ON("VOLUMETRIC");
  #else
    SERIAL_CAP_OFF("VOLUMETRIC");
  #endif

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
  #if HAS_AUTOLEVEL
    SERIAL_CAP_ON("AUTOLEVEL");
  #else
    SERIAL_CAP_OFF("AUTOLEVEL");
  #endif

  // Z_PROBE (G30)
  #if HAS_BED_PROBE
    SERIAL_CAP_ON("Z_PROBE");
  #else
    SERIAL_CAP_OFF("Z_PROBE");
  #endif

  // MESH_REPORT (M420 V)
  #if HAS_LEVELING
    SERIAL_CAP_ON("LEVELING_DATA");
  #else
    SERIAL_CAP_OFF("LEVELING_DATA");
  #endif

  // SOFTWARE_POWER (M80, M81)
  #if HAS_POWER_SWITCH
    SERIAL_CAP_ON("SOFTWARE_POWER");
  #else
    SERIAL_CAP_OFF("SOFTWARE_POWER");
  #endif

  // CASE LIGHTS (M355)
  #if HAS_CASE_LIGHT
    SERIAL_CAP_ON("TOGGLE_LIGHTS");
  #else
    SERIAL_CAP_OFF("TOGGLE_LIGHTS");
  #endif

  // EMERGENCY_PARSER (M108, M112, M410, M876)
  #if ENABLED(EMERGENCY_PARSER)
    SERIAL_CAP_ON("EMERGENCY_PARSER");
  #else
    SERIAL_CAP_OFF("EMERGENCY_PARSER");
  #endif

  // CHAMBER_TEMPERATURE (M141, M191)
  #if HAS_CHAMBERS
    SERIAL_CAP_ON("CHAMBER_TEMPERATURE");
  #else
    SERIAL_CAP_OFF("CHAMBER_TEMPERATURE");
  #endif

}

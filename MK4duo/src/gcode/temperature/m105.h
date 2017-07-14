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

#define CODE_M105

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105(void) {

  GET_TARGET_HOTEND(105);

  #if HAS_TEMP_HOTEND || HAS_TEMP_BED || HAS_TEMP_CHAMBER || HAS_TEMP_COOLER || ENABLED(FLOWMETER_SENSOR) || (ENABLED(CNCROUTER) && ENABLED(FAST_PWM_CNCROUTER))
    SERIAL_STR(OK);
    #if HAS_TEMP_HOTEND || HAS_TEMP_BED
      thermalManager.print_heaterstates();
    #endif
    #if HAS_TEMP_CHAMBER
      thermalManager.print_chamberstate();
    #endif
    #if HAS_TEMP_COOLER
      thermalManager.print_coolerstate();
    #endif
    #if ENABLED(FLOWMETER_SENSOR)
      print_flowratestate();
    #endif
    #if ENABLED(CNCROUTER) && ENABLED(FAST_PWM_CNCROUTER)
      print_cncspeed();
    #endif
    #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
      thermalManager.print_MCUstate();
    #endif
  #else // HASNT(TEMP_0) && HASNT(TEMP_BED)
    SERIAL_LM(ER, MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_EOL();
}

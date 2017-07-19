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

#if HAS_MULTI_MODE

  /**
   * Shared function for Printer Mode GCodes
   */
  static void gcode_printer_mode(const int8_t new_mode) {
    const static char str_tooltype_0[] PROGMEM = "FFF";
    const static char str_tooltype_1[] PROGMEM = "Laser";
    const static char str_tooltype_2[] PROGMEM = "CNC";
    const static char* const tool_strings[] PROGMEM = { str_tooltype_0, str_tooltype_1, str_tooltype_2 };
    if (new_mode >= 0 && (PrinterMode)new_mode < PRINTER_MODE_COUNT) printer.mode = (PrinterMode)new_mode;
    SERIAL_SM(ECHO, "Printer-Mode: ");
    SERIAL_PS((char*)pgm_read_word(&(tool_strings[printer.mode])));
    SERIAL_CHR(' ');
    SERIAL_EV((int)(printer.mode == PRINTER_MODE_FFF ? extruder.active : 0));
  }

  #define CODE_M450
  #define CODE_M451

  /**
   * M450: Set and/or report current tool type
   *
   *  S<type> - The new tool type
   */
  inline void gcode_M450(void) {
    gcode_printer_mode(parser.seen('S') ? parser.value_byte() : -1);
  }

  /**
   * M451: Select FFF printer mode
   */
  inline void gcode_M451(void) { gcode_printer_mode(PRINTER_MODE_FFF); }

  #if ENABLED(LASER)

    #define CODE_M452

    /**
     * M452: Select Laser printer mode
     */
    inline void gcode_M452(void) { gcode_printer_mode(PRINTER_MODE_LASER); }
  #endif

  #if HAS_CNCROUTER

    #define CODE_M453

    /**
     * M453: Select CNC printer mode
     */
    inline void gcode_M453(void) { gcode_printer_mode(PRINTER_MODE_CNC); }
  #endif

#endif // HAS_MULTI_MODE

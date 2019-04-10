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
 * tcode.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

/**
 * T0-T<n>: Switch tool, usually switching extruders or CNC tools
 *
 * For Extruders:
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 *
 * For CNC no other parameters are expected
 *
 * For PRUSA_MMU2:
 *   T[n] Gcode to extrude at least 38.10 mm at feedrate 19.02 mm/s must follow immediately to load to extruder wheels.
 *   T?   Gcode to extrude shouldn't have to follow. Load to extruder wheels is done automatically.
 *   Tx   Same as T?, but nozzle doesn't have to be preheated. Tc requires a preheated nozzle to finish filament load.
 *   Tc   Load to nozzle after filament was prepared by Tc and nozzle is already heated.
 */
inline void gcode_T(const uint8_t tool_id) {

  if (printer.debugFeature()) {
    DEBUG_MV(">>> T(", tool_id);
    DEBUG_CHR(')');
    DEBUG_EOL();
    DEBUG_POS("BEFORE", mechanics.current_position);
  }

  #if ENABLED(CNCROUTER)
    
    bool wait = true;
    bool raise_z = false;

    if (printer.mode == PRINTER_MODE_CNC) {
      // Host manage wait on change, don't block
      if (parser.seen('W')) wait = false;
      // Host manage position, don't raise Z
      if (parser.seen('Z')) raise_z = false;

      cnc.tool_change(tool_id, wait, raise_z);
    }

  #endif

  #if HAS_MMU2
    if (parser.string_arg) {
      mmu2.tool_change(parser.string_arg);   // Special commands T?/Tx/Tc
      return;
    }
  #endif

  #if EXTRUDERS == 1 && ENABLED(ADVANCED_PAUSE_FEATURE)

    if (printer.mode == PRINTER_MODE_FFF && printer.isPrinting() && tools.previous_extruder != tool_id) {
      commands.process_now_P(PSTR("M600"));
      tools.previous_extruder = tool_id;
    }

  #elif (EXTRUDERS > 1 && HOTENDS == 1) || (ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1)

    if (printer.mode == PRINTER_MODE_FFF) tools.change(tool_id);

  #elif EXTRUDERS > 1 && HOTENDS > 1

    if (printer.mode == PRINTER_MODE_FFF) {
      tools.change(
        tool_id,
        MMM_TO_MMS(parser.linearval('F')),
        (tool_id == tools.active_extruder) || parser.boolval('S')
      );
    }

  #endif

  if (printer.debugFeature()) {
    DEBUG_POS("AFTER", mechanics.current_position);
    DEBUG_EM("<<< T()");
  }

}

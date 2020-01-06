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
 * tcode.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
    DEBUG_POS("BEFORE", mechanics.position);
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

  if (printer.mode == PRINTER_MODE_FFF) {

    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      if (printer.isPrinting() && toolManager.extruder.previous != tool_id && toolManager.extruder.total == 1) {
        commands.inject_P(PSTR("M600"));
        toolManager.extruder.previous = tool_id;
      }
      else
    #endif
      toolManager.change(tool_id, (tool_id == toolManager.extruder.active) || parser.boolval('S'));

  }

  if (printer.debugFeature()) {
    DEBUG_POS("AFTER", mechanics.position);
    DEBUG_MV("<<< T(", tool_id);
    DEBUG_CHR(')');
    DEBUG_EOL();
  }

}

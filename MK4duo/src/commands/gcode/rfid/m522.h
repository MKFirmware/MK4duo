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

#if ENABLED(RFID_MODULE)

#define CODE_M522

/**
 * M522: Read or Write on card. M522 T<extruders> R<read> or W<write> L<list>
 */
inline void gcode_M522() {

  if (commands.get_target_tool(522)) return;

  if (!printer.IsRfid()) return;

  if (parser.seen('R')) {
    SERIAL_EM("Put RFID on tag!");
    #if HAS_NEXTION_LCD
      rfid_setText("Put RFID on tag!");
    #endif
    rfid522.Spool_must_read[toolManager.extruder.target] = true;
  }
  if (parser.seen('W')) {
    if (rfid522.Spool_ID[toolManager.extruder.target] != 0) {
      SERIAL_EM("Put RFID on tag!");
      #if HAS_NEXTION_LCD
        rfid_setText("Put RFID on tag!");
      #endif
      rfid522.Spool_must_write[toolManager.extruder.target] = true;
    }
    else {
      SERIAL_LM(ER, "You have not read this Spool!");
      #if HAS_NEXTION_LCD
        rfid_setText("You have not read this Spool!", 64488);
      #endif
    }
  }

  if (parser.seen('L')) rfid522.print_info(toolManager.extruder.target);
}

#endif // RFID_MODULE

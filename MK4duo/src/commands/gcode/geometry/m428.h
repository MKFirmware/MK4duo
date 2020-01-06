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

#if ENABLED(WORKSPACE_OFFSETS)

  #define CODE_M428

  /**
   * M428: Set home_offset based on the distance between the
   *       position.x and the nearest "reference point."
   *       If an axis is past center its Endstop position
   *       is the reference-point. Otherwise it uses 0. This allows
   *       the Z offset to be set near the bed when using a max Endstop.
   *
   *       M428 can't be used more than 2cm away from 0 or an Endstop.
   *
   *       Use M206 to set these values directly.
   */
  inline void gcode_M428() {
    if (mechanics.axis_unhomed_error()) return;

    xyz_pos_t diff;
    LOOP_XYZ(i) {
      diff[i] = mechanics.axis_home_pos(AxisEnum(i)) - mechanics.position[i];
      if (WITHIN(diff[i], -20, 20) && mechanics.get_homedir((AxisEnum)i) > 0)
        diff[i] = -mechanics.position[i];
      if (!WITHIN(diff[i], -20, 20)) {
        SERIAL_LM(ER, MSG_HOST_ERR_M428_TOO_FAR);
        LCD_ALERTMESSAGEPGM_P(PSTR("Err: Too far!"));
        sound.feedback(false); // BUZZ(200, 40);
        return;
      }
    }

    LOOP_XYZ(i) mechanics.set_home_offset((AxisEnum)i, diff[i]);
    mechanics.report_position();
    LCD_MESSAGEPGM(MSG_OFFSETS_APPLIED);
    sound.feedback();
  }

#endif // ENABLED(WORKSPACE_OFFSETS)

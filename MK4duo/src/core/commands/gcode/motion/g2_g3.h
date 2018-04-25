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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 *
 * This command has two forms: IJ-form and R-form.
 *
 *  - I specifies an X offset. J specifies a Y offset.
 *    At least one of the IJ parameters is required.
 *    X and Y can be omitted to do a complete circle.
 *    The given XY is not error-checked. The arc ends
 *     based on the angle of the mechanics.destination.
 *    Mixing I or J with R will throw an error.
 *
 *  - R specifies the radius. X or Y is required.
 *    Omitting both X and Y will throw an error.
 *    X or Y must differ from the current XY.
 *    Mixing R with I or J will throw an error.
 *
 *  - P specifies the number of full circles to do
 *    before the specified arc move.
 *
 *  Examples:
 *
 *    G2 I10           ; CW circle centered at X+10
 *    G3 X20 Y12 R14   ; CCW circle with r=14 ending at X20 Y12
 */
#if ENABLED(ARC_SUPPORT) && !IS_SCARA

  #if N_ARC_CORRECTION < 1
    #undef N_ARC_CORRECTION
    #define N_ARC_CORRECTION 1
  #endif

  #define CODE_G2
  #define CODE_G3

  void gcode_G2_G3(bool clockwise) {

    if (printer.isRunning()) {

      #if ENABLED(SF_ARC_FIX)
        const bool relative_mode_backup = printer.isRelativeMode();
        printer.setRelativeMode(true);
      #endif

      commands.get_destination();

      #if ENABLED(LASER) && ENABLED(LASER_FIRE_G1)
        #if ENABLED(INTENSITY_IN_BYTE)
          if (parser.seenval('S')) laser.intensity = ((float)parser.value_byte() / 255.0) * 100.0;
        #else
          if (parser.seenval('S')) laser.intensity = parser.value_float();
        #endif
        if (parser.seenval('L')) laser.duration = parser.value_ulong();
        if (parser.seenval('P')) laser.ppm = parser.value_float();
        if (parser.seenval('D')) laser.diagnostics = parser.value_bool();
        if (parser.seenval('B')) laser.set_mode(parser.value_int());

        laser.status = LASER_ON;
      #endif

      #if ENABLED(SF_ARC_FIX)
        printer.setRelativeMode(relative_mode_backup);
      #endif

      float arc_offset[2] = { 0.0, 0.0 };
      if (parser.seenval('R')) {
        const float r = parser.value_linear_units(),
                    p1 = mechanics.current_position[X_AXIS], q1 = mechanics.current_position[Y_AXIS],
                    p2 = mechanics.destination[X_AXIS],      q2 = mechanics.destination[Y_AXIS];
        if (r && (p2 != p1 || q2 != q1)) {
          const float e = clockwise ^ (r < 0) ? -1 : 1,           // clockwise -1/1, counterclockwise 1/-1
                      dx = p2 - p1, dy = q2 - q1,                 // X and Y differences
                      d = HYPOT(dx, dy),                          // Linear distance between the points
                      h = SQRT(sq(r) - sq(d * 0.5)),              // Distance to the arc pivot-point
                      mx = (p1 + p2) * 0.5, my = (q1 + q2) * 0.5, // Point between the two points
                      sx = -dy / d, sy = dx / d,                  // Slope of the perpendicular bisector
                      cx = mx + e * h * sx, cy = my + e * h * sy; // Pivot-point of the arc
          arc_offset[0] = cx - p1;
          arc_offset[1] = cy - q1;
        }
      }
      else {
        if (parser.seenval('I')) arc_offset[0] = parser.value_linear_units();
        if (parser.seenval('J')) arc_offset[1] = parser.value_linear_units();
      }

      if (arc_offset[0] || arc_offset[1]) {

        #if ENABLED(ARC_P_CIRCLES)
          // P indicates number of circles to do
          int8_t circles_to_do = parser.byteval('P');
          if (!WITHIN(circles_to_do, 0, 100))
            SERIAL_LM(ER, MSG_ERR_ARC_ARGS);
          while (circles_to_do--)
            mechanics.plan_arc(mechanics.current_position, arc_offset, clockwise);
        #endif

        // Send an arc to the planner
        mechanics.plan_arc(mechanics.destination, arc_offset, clockwise);
        stepper.move_watch.start();
      }
      else {
        // Bad arguments
        SERIAL_LM(ER, MSG_ERR_ARC_ARGS);
      }

      #if ENABLED(LASER) && ENABLED(LASER_FIRE_G1)
        laser.status = LASER_OFF;
      #endif
    }
  }

  inline void gcode_G2(void) { gcode_G2_G3(true); }
  inline void gcode_G3(void) { gcode_G2_G3(false); }

#endif // ARC_SUPPORT && !IS_SCARA

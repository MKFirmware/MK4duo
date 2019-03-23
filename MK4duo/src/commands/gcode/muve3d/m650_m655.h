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

#if MECH(MUVE3D)

  static float  peel_distance   = 0,
                peel_speed      = 0,
                peel_pause      = 0,
                retract_speed   = 0,
                tilt_distance   = 0,
                layer_thickness = 0;
  static bool   tilted          = false;

  #define CODE_M650
  #define CODE_M651
  #define CODE_M653
  #define CODE_M654
  #define CODE_M655

  // M650: Set peel distance
  inline void gcode_M650(void) {

    planner.synchronize();

    peel_distance   = (parser.seen('D') ? parser.value_float() : 2.0);
    peel_speed      = (parser.seen('S') ? parser.value_float() : 2.0);
    retract_speed   = (parser.seen('R') ? parser.value_float() : 2.0);
    peel_pause      = (parser.seen('P') ? parser.value_float() : 0.0);
    tilt_distance   = (parser.seen('T') ? parser.value_float() : 20.0);
    layer_thickness = (parser.seen('H') ? parser.value_float() : 0.0);

    // Initialize tilted to false. The intent here is that you would send this command at the start of a print job, and
    // the platform would be level when you do. As such, we assume that you either hand-cranked it to level, or executed
    // an M654 command via manual GCode before running a new print job. If not, then the platform is currently tilted, and
    // your print job is going to go poorly.
    tilted = false;
  }

  // M651: Run peel move and return back to start.
  inline void gcode_M651(void) {

    if (peel_distance > 0) {
      planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS] + peel_distance, mechanics.destination[Z_AXIS], peel_speed, tools.active_extruder);
      planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS] + peel_distance, mechanics.destination[Z_AXIS] + peel_distance, peel_speed, tools.active_extruder);
      planner.synchronize();
      if (peel_pause > 0) printer.safe_delay(peel_pause);
    }

    planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS], mechanics.destination[Z_AXIS], retract_speed, tools.active_extruder);
    planner.synchronize();
  }

  // M653: Execute tilt move
  inline void gcode_M653(void) {
    // Double tilts are not allowed.
    if (!tilted) {
      planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS] + tilt_distance, mechanics.destination[Z_AXIS], retract_speed, tools.active_extruder);
      planner.synchronize();
    }
  }

  // M654 - execute untilt move
  inline void gcode_M654(void) {
    // Can only untilt if tilted
    if (tilted) {
       // To prevent subsequent commands from not knowing our
       // actual position, update the Z axis, then move to it.
       mechanics.destination[Z_AXIS] += tilt_distance;
       planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS], mechanics.destination[Z_AXIS], retract_speed, tools.active_extruder);
       // And save it away as our current position, because we're there.
       mechanics.set_current_to_destination();
       planner.synchronize();
       tilted = false;
    }
  }

  // M655: Send projector control commands via serial
  inline void gcode_M655(void) {

    // Viewsonic commands
    if (parser.seen('V')) {
      int tempVal = parser.value_int();

      switch (tempVal) {
        // Power Off
        case 0: {
          // 0614000400341101005E
          const byte off[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                              0x34, 0x11, 0x01, 0x00, 0x5E};
          DLPSerial.write(off, sizeof(off));
        }
        break;
        // Power On
        case 1: {
          // 0614000400341100005D
          const byte on[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                             0x34, 0x11, 0x00, 0x00, 0x5D};
          DLPSerial.write(on, sizeof(on));
        }
        break;
        // Factory Reset
        case 2: {
          // 0614000400341102005F
          const byte reset[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                0x34, 0x11, 0x02, 0x00, 0x5F};
          DLPSerial.write(reset, sizeof(reset));
        }
        break;
        // Splash Screen Black
        case 3: {
          // 061400040034110A0067
          const byte blackScreen[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                      0x34, 0x11, 0x0A, 0x00, 0x67};
          DLPSerial.write(blackScreen, sizeof(blackScreen));
        }
        break;
        // High Altitude On
        case 4: {
          // 061400040034110C016A
          const byte HAOn[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                               0x34, 0x11, 0x0C, 0x01, 0x6A};
          DLPSerial.write(HAOn, sizeof(HAOn));
        }
        break;
        // High Altitude Off
        case 5: {
          // 061400040034110C0069
          const byte HAOff[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                0x34, 0x11, 0x0C, 0x00, 0x69};
          DLPSerial.write(HAOff, sizeof(HAOff));
        }
        break;
        // Lamp Mode Normal
        case 6: {
          // 0614000400341110006D
          const byte lampNormal[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                     0x34, 0x11, 0x10, 0x00, 0x6D};
          DLPSerial.write(lampNormal, sizeof(lampNormal));
        }
        break;
        // Contrast Decrease
        case 7: {
          // 06140004003412020060
          const byte contDec[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                  0x34, 0x12, 0x02, 0x00, 0x60};
          DLPSerial.write(contDec, sizeof(contDec));
        }
        break;
        // Contrast Increase
        case 8: {
          // 06140004003412020161
          const byte contInc[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                  0x34, 0x12, 0x02, 0x01, 0x61};
          DLPSerial.write(contInc, sizeof(contInc));
        }
        break;
        // Brightness Decrease
        case 9: {
          // 06140004003412030061
          const byte brightDec[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                    0x34, 0x12, 0x03, 0x00, 0x61};
          DLPSerial.write(brightDec, sizeof(brightDec));
        }
        break;
        // Brightness Increase
        case 10: {
          // 06140004003412030162
          const byte brightInc[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                    0x34, 0x12, 0x03, 0x01, 0x62};
          DLPSerial.write(brightInc, sizeof(brightInc));
        }
        break;
      }
    }
  }

#endif // MECH(MUVE3D)

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

#if ENABLED(G38_PROBE_TARGET)

  #define CODE_G38

  static bool G38_run_probe() {

    bool G38_pass_fail = false;

    // Get direction of move and retract
    float retract_mm[XYZ];
    LOOP_XYZ(i) {
      float dist = mechanics.destination[i] - mechanics.current_position[i];
      retract_mm[i] = FABS(dist) < G38_MINIMUM_MOVE ? 0 : mechanics.home_bump_mm((AxisEnum)i) * (dist > 0 ? -1 : 1);
    }

    stepper.synchronize();  // wait until the machine is idle

    // Move until mechanics.destination reached or target hit
    endstops.setEnabled(true);
    printer.setG38Move(true);
    endstops.setG38EndstopHit(false);
    mechanics.prepare_move_to_destination();
    stepper.synchronize();
    printer.setG38Move(false);

    endstops.hit_on_purpose();
    mechanics.set_current_from_steppers_for_axis(ALL_AXES);
    mechanics.sync_plan_position_mech_specific();

    // Only do remaining moves if target was hit
    if (endstops.isG38EndstopHit()) {

      G38_pass_fail = true;

      // Move away by the retract distance
      mechanics.set_destination_to_current();
      LOOP_XYZ(i) mechanics.destination[i] += retract_mm[i];
      endstops.setEnabled(false);
      mechanics.prepare_move_to_destination();
      stepper.synchronize();

      mechanics.feedrate_mm_s /= 4;

      // Bump the target more slowly
      LOOP_XYZ(i) mechanics.destination[i] -= retract_mm[i] * 2;

      endstops.setEnabled(true);
      printer.setG38Move(true);
      mechanics.prepare_move_to_destination();
      stepper.synchronize();
      printer.setG38Move(false);

      mechanics.set_current_from_steppers_for_axis(ALL_AXES);
      mechanics.sync_plan_position_mech_specific();
    }

    endstops.hit_on_purpose();
    endstops.setNotHoming();
    return G38_pass_fail;
  }

  /**
   * G38.2 - probe toward workpiece, stop on contact, signal error if failure
   * G38.3 - probe toward workpiece, stop on contact
   *
   * Like G28 except uses Z min endstop for all axes
   */
  void gcode_G38_S(bool is_38_2) {
    // Get X Y Z E F
    gcode_get_destination();

    setup_for_endstop_or_probe_move();

    // If any axis has enough movement, do the move
    LOOP_XYZ(i)
      if (FABS(mechanics.destination[i] - mechanics.current_position[i]) >= G38_MINIMUM_MOVE) {
        if (!parser.seenval('F')) mechanics.feedrate_mm_s = mechanics.homing_feedrate_mm_s[i];
        // If G38.2 fails throw an error
        if (!G38_run_probe() && is_38_2) {
          SERIAL_LM(ER, "Failed to reach target");
        }
        break;
      }

    clean_up_after_endstop_or_probe_move();
  }

  inline void gcode_G38(void) {
    if (parser.subcode == 2 || parser.subcode == 3)
      gcode_G38_S(parser.subcode == 2);
  }

#endif // G38_PROBE_TARGET

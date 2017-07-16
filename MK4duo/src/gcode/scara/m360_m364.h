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

#if MECH(MORGAN_SCARA)

  bool SCARA_move_to_cal(uint8_t delta_a, uint8_t delta_b) {
    if (IsRunning()) {
      forward_kinematics_SCARA(delta_a, delta_b);
      mechanics.destination[X_AXIS] = LOGICAL_X_POSITION(mechanics.cartesian_position[X_AXIS]);
      mechanics.destination[Y_AXIS] = LOGICAL_Y_POSITION(mechanics.cartesian_position[Y_AXIS]);
      mechanics.destination[Z_AXIS] = mechanics.current_position[Z_AXIS];
      mechanics.prepare_move_to_destination();
      return true;
    }
    return false;
  }

  #define CODE_M360
  #define CODE_M361
  #define CODE_M362
  #define CODE_M363
  #define CODE_M364

  /**
   * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   */
  inline bool gcode_M360(void) {
    SERIAL_LM(ECHO, " Cal: Theta 0");
    return SCARA_move_to_cal(0, 120);
  }

  /**
   * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M361(void) {
    SERIAL_LM(ECHO, " Cal: Theta 90");
    return SCARA_move_to_cal(90, 130);
  }

  /**
   * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   */
  inline bool gcode_M362(void) {
    SERIAL_LM(ECHO, " Cal: Psi 0");
    return SCARA_move_to_cal(60, 180);
  }

  /**
   * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M363(void) {
    SERIAL_LM(ECHO, " Cal: Psi 90");
    return SCARA_move_to_cal(50, 90);
  }

  /**
   * M364: SCARA calibration: Move to cal-position PsiC (90 deg to Theta calibration position)
   */
  inline bool gcode_M364(void) {
    SERIAL_LM(ECHO, " Cal: Theta-Psi 90");
    return SCARA_move_to_cal(45, 135);
  }

#endif // MECH(MORGAN_SCARA)

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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define CODE_M569

/**
 * M569: Stepper driver control
 *
 *  X[bool]           - Set X direction
 *  Y[bool]           - Set Y direction
 *  Z[bool]           - Set Z direction
 *  T[tools] E[bool]  - Set Extruder direction
 *
 *  P[int]            - Set minimum pulse
 *  R[long]           - Set maximum rate
 *  D[long]           - Set Direction delay
 *
 */
inline void gcode_M569(void) {

  //#define DEBUG_PULSE_CYCLE

  if (commands.get_target_tool(569)) return;

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      stepper.setStepDir((AxisEnum)a, parser.value_bool());
    }
  }
  // Set actually direction
  reset_stepper_drivers();

  if (parser.seen('D')) stepper.direction_delay = parser.value_ulong();
  if (parser.seen('P')) stepper.minimum_pulse   = parser.value_byte();
  if (parser.seen('R')) stepper.maximum_rate    = parser.value_ulong();
  // Recalculate pulse cycle
  HAL_calc_pulse_cycle();

  SERIAL_EM("Reporting Stepper control");
  SERIAL_MT(" X dir:", stepper.isStepDir(X_AXIS) ? "true" : "false");
  SERIAL_MT(" Y dir:", stepper.isStepDir(Y_AXIS) ? "true" : "false");
  SERIAL_MV(" Z dir:", stepper.isStepDir(Z_AXIS) ? "true" : "false");

  #if DRIVER_EXTRUDERS == 1
    SERIAL_MV(" E dir:", stepper.isStepDir(E_AXIS) ? "true" : "false");
  #endif

  #if DRIVER_EXTRUDERS > 1
    LOOP_DRV_EXTRUDER() {
      #if HAS_MKMULTI_TOOLS
        SERIAL_MV(" Driver Extruder:", d);
      #else
        SERIAL_MV(" Extruder:", d);
      #endif
      SERIAL_MT(" dir:" , stepper.isStepDir((AxisEnum)(E_AXIS + d)) ? "true" : "false");
    }
  #endif

  SERIAL_MV(" Direction delay(ns):", stepper.direction_delay);
  SERIAL_MV(" Minimum pulse(us):", stepper.minimum_pulse);
  SERIAL_MV(" Maximum rate(Hz):", stepper.maximum_rate);

  SERIAL_EOL();

  #if ENABLED(DEBUG_PULSE_CYCLE)
    SERIAL_EMV("HAL_min_pulse_cycle:", HAL_min_pulse_cycle);
    SERIAL_EMV("HAL_min_pulse_tick:", HAL_min_pulse_tick);
    SERIAL_EMV("HAL_add_pulse_ticks:", HAL_add_pulse_ticks);
    SERIAL_EMV("HAL_min_isr_frequency:", HAL_min_isr_frequency);
    SERIAL_EMV("HAL_frequency_limit[0]:", HAL_frequency_limit[0]);
    SERIAL_EMV("HAL_frequency_limit[1]:", HAL_frequency_limit[1]);
    SERIAL_EMV("HAL_frequency_limit[2]:", HAL_frequency_limit[2]);
    SERIAL_EMV("HAL_frequency_limit[3]:", HAL_frequency_limit[3]);
    SERIAL_EMV("HAL_frequency_limit[4]:", HAL_frequency_limit[4]);
    SERIAL_EMV("HAL_frequency_limit[5]:", HAL_frequency_limit[5]);
    SERIAL_EMV("HAL_frequency_limit[6]:", HAL_frequency_limit[6]);
    SERIAL_EMV("HAL_frequency_limit[7]:", HAL_frequency_limit[7]);
  #endif

}

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

#if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)

  #define CODE_M906

  /**
   * M906: Set motor currents
   */
  inline void gcode_M906() {

    if (commands.get_target_tool(906)) return;

    LOOP_XYZE(i) {
      if (parser.seen(axis_codes[i])) {
        const uint8_t a = i + (i == E_AXIS ? toolManager.extruder.target : 0);
        driver[a]->data.ma = parser.value_ushort();
        externaldac.set_driver_current(a, driver[a]->data.ma);
      }
    }
  }

#elif HAS_TRINAMIC

  #define CODE_M906
  
  /**
   * M906: Set motor current in milliamps using axis codes X, Y, Z, E
   * Report driver currents when no axis specified
   */
  inline void gcode_M906() {

    if (commands.get_target_tool(906)) return;

    #if DISABLED(DISABLE_M503)
      // No arguments? Show M906 report.
      if (!parser.seen("XYZE")) {
        tmcManager.print_M906();
        return;
      }
    #endif

    LOOP_XYZE(i) if (uint16_t value = parser.intval(axis_codes[i])) {
      switch (i) {
        case X_AXIS:
          #if AXIS_HAS_TMC(X)
            driver.x->tmc->rms_current(value);
          #endif
          #if AXIS_HAS_TMC(X2)
            driver.x2->rms_current(value);
          #endif
          break;
        case Y_AXIS:
          #if AXIS_HAS_TMC(Y)
            driver.y->tmc->rms_current(value);
          #endif
          #if AXIS_HAS_TMC(Y2)
            driver.y2->tmc->rms_current(value);
          #endif
          break;
        case Z_AXIS:
          #if AXIS_HAS_TMC(Z)
            driver.z->tmc->rms_current(value);
          #endif
          #if AXIS_HAS_TMC(Z2)
            driver.z2->tmc->rms_current(value);
          #endif
          #if AXIS_HAS_TMC(Z3)
            driver.z3->tmc->rms_current(value);
          #endif
          break;
        case E_AXIS:
          Driver* drv = driver.e[extruders[toolManager.extruder.target]->get_driver()];
          if (drv && drv->tmc) drv->tmc->rms_current(value);
          break;
      }
    }

  }

#endif // HAS_TRINAMIC

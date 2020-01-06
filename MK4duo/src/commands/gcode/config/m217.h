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

#if ENABLED(NOZZLE_PARK_FEATURE) || MAX_EXTRUDER > 1

#define CODE_M217

/**
 * M217 - Set Park position and tool change parameters
 *
 *  T[tools]    Tools
 *  S[linear]   Swap length
 *  E[linear]   Purge length
 *  P[linear/m] Purge speed
 *  R[linear/m] Retract speed
 *  X[linear]   Park X (Requires NOZZLE_PARK_FEATURE)
 *  Y[linear]   Park Y (Requires NOZZLE_PARK_FEATURE)
 *  Z[linear]   Park Z Raise
 *
 */
inline void gcode_M217() {

  if (commands.get_target_tool(217)) return;

  #if ENABLED(TOOL_CHANGE_FIL_SWAP)
    const uint8_t t = toolManager.extruder.target;
    #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
      static constexpr float max_extrude = EXTRUDE_MAXLENGTH;
    #else
      static constexpr float max_extrude = 500;
    #endif
  #endif

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M217 report.
    if (parser.seen_any()) {
      #if ENABLED(TOOL_CHANGE_FIL_SWAP)
        extruders[t]->print_M217(t);
      #endif
      nozzle.print_M217();
      return;
    }
  #endif

  #if ENABLED(TOOL_CHANGE_FIL_SWAP)
    if (parser.seenval('S')) { const float    v = parser.value_linear_units();  extruders[t]->data.swap_length    = constrain(v, 0,  max_extrude);  }
    if (parser.seenval('E')) { const float    v = parser.value_linear_units();  extruders[t]->data.purge_lenght   = constrain(v, 0,  max_extrude);  }
    if (parser.seenval('P')) { const int16_t  v = parser.value_linear_units();  extruders[t]->data.prime_speed    = constrain(v, 10, 6000);         }
    if (parser.seenval('R')) { const int16_t  v = parser.value_linear_units();  extruders[t]->data.retract_speed  = constrain(v, 10, 6000);         }
  #endif

  if (parser.seenval('X')) nozzle.data.park_point.x = parser.value_linear_units();
  if (parser.seenval('Y')) nozzle.data.park_point.y = parser.value_linear_units();
  if (parser.seenval('Z')) nozzle.data.park_point.z = parser.value_linear_units();

}

#endif // ENABLED(NOZZLE_PARK_FEATURE) || MAX_EXTRUDER > 1
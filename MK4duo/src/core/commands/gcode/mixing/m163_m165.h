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

#if ENABLED(COLOR_MIXING_EXTRUDER)

  #define CODE_M163

  /**
   * M163: Set a single mix factor for a mixing extruder
   *       This is called "weight" by some systems.
   *
   *   S[index]   The channel index to set
   *   P[float]   The mix value
   *
   */
  inline void gcode_M163(void) {
    const int mix_index = parser.intval('S');
    if (mix_index < MIXING_STEPPERS)
      mixer.set_M163_collector(mix_index, MAX(parser.floatval('P'), 0.0));
  }

  #define CODE_M164

  /**
   * M164: Store the current mix factors as a virtual tools.
   *
   *   S[index]   The virtual tools to store
   *              If 'S' is omitted update the active virtual tool.
   *
   */
  inline void gcode_M164(void) {
    #if MIXING_VIRTUAL_TOOLS > 1
      const int tool_index = parser.intval('S', -1);
    #else
      constexpr int tool_index = 0;
    #endif
    if (WITHIN(tool_index, 0, MIXING_VIRTUAL_TOOLS - 1))
      mixer.normalize(tool_index);
    else
      mixer.normalize(mixer.get_current_v_tool());
  }

  #define CODE_M165

  /**
   * M165: Set multiple mix factors for a mixing extruder.
   *       Factors that are left out will be set to 0.
   *
   *   A[factor] Mix factor for extruder stepper 1
   *   B[factor] Mix factor for extruder stepper 2
   *   C[factor] Mix factor for extruder stepper 3
   *   D[factor] Mix factor for extruder stepper 4
   *   H[factor] Mix factor for extruder stepper 5
   *   I[factor] Mix factor for extruder stepper 6
   *
   */
  inline void gcode_M165(void) {
    // Get mixing parameters from the GCode
    // The total "must" be 1.0 (but it will be normalized)
    // If no mix factors are given, the old mix is preserved
    const char mixing_codes[] = { 'A', 'B'
      #if MIXING_STEPPERS > 2
        , 'C'
        #if MIXING_STEPPERS > 3
          , 'D'
          #if MIXING_STEPPERS > 4
            , 'H'
            #if MIXING_STEPPERS > 5
              , 'I'
            #endif // MIXING_STEPPERS > 5
          #endif // MIXING_STEPPERS > 4
        #endif // MIXING_STEPPERS > 3
      #endif // MIXING_STEPPERS > 2
    };
    uint8_t mix_bits = 0;
    MIXER_STEPPER_LOOP(i) {
      if (parser.seenval(mixing_codes[i])) {
        SBI(mix_bits, i);
        mixer.set_M163_collector(i, MAX(parser.value_float(), 0.0f));
      }
    }
    // If any mixing factors were included, clear the rest
    // If none were included, preserve the last mix
    if (mix_bits) {
      MIXER_STEPPER_LOOP(i)
        if (!TEST(mix_bits, i)) mixer.set_M163_collector(i, 0.0f);
      mixer.normalize(mixer.get_current_v_tool());
    }
  }

#endif  // COLOR_MIXING_EXTRUDER

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
#pragma once

/**
 * mixing.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(COLOR_MIXING_EXTRUDER)

#ifdef __AVR__
  #define MIXER_ACCU_SIGNED
  typedef uint8_t       mixer_color_t[MIXING_STEPPERS];;
  typedef int8_t        mixer_accu_t;
  #define COLOR_A_MASK 0x80
  #define COLOR_MASK 0x7F
#else
  typedef uint_fast16_t mixer_color_t[MIXING_STEPPERS];
  typedef uint_fast16_t mixer_accu_t;
  #define COLOR_A_MASK 0x8000
  #define COLOR_MASK 0x7FFF
#endif

#define MIXING_STEPPER_LOOP(VAR) \
  for (uint8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++)

typedef struct {
  bool enabled;               // This gradient is enabled
  mixer_color_t color;        // The current gradient color
  float start_z, end_z;       // Region for gradient
  int8_t start_pct, end_pct;  // Mix component for E0
} gradient_t;

class Mixer {

  public: /** Constructor */

    Mixer() {}

  public: /** Public Parameters */

    static int16_t mix[MIXING_STEPPERS];      // Current Mix in proportion to 100
                                              // For use by the LCD display

    static gradient_t gradient;

  private: /** Private Parameters */

    // Used up to Planner level
    static uint_fast8_t   selected_v_tool;
    static float          M163_collector[MIXING_STEPPERS];
    static mixer_color_t  color[MIXING_VIRTUAL_TOOLS];

    // Used in Stepper
    static int_fast8_t    runner;
    static mixer_color_t  s_color;
    static mixer_accu_t   accu[MIXING_STEPPERS];

  public: /** Public Function */

    static void init(void); // Populate colors at boot time

    // Used up to Planner level
    static void normalize(const uint8_t tool_index);
    FORCE_INLINE static uint8_t get_current_v_tool(void) { return selected_v_tool; }
    FORCE_INLINE static void T(const uint_fast8_t c) { selected_v_tool = c; }
    FORCE_INLINE static void set_M163_collector(const uint8_t c, const float f) { M163_collector[c] = f; }

    // Used when dealing with blocks
    FORCE_INLINE static void populate_block(mixer_color_t b_color) {
      if (gradient.enabled) {
        MIXING_STEPPER_LOOP(i) b_color[i] = i < 2 ? gradient.color[i] : 0;
        return;
      }
      uint_fast8_t j = get_current_v_tool();
      MIXING_STEPPER_LOOP(i) b_color[i] = color[j][i];
    }
    FORCE_INLINE static void stepper_setup(mixer_color_t b_color) { MIXING_STEPPER_LOOP(i) s_color[i] = b_color[i]; }

    static inline void update_gradient_from_mix() {
      // Scale each component to the largest one in terms of COLOR_A_MASK
      // So the largest component will be COLOR_A_MASK and the other will be in proportion to it
      const float inverse_max = RECIPROCAL(MAX(mix[0], mix[1]));
      gradient.color[0] = RECIPROCAL(mix[0] * 0.01) * COLOR_A_MASK * inverse_max;
      gradient.color[1] = RECIPROCAL(mix[1] * 0.01) * COLOR_A_MASK * inverse_max;
    }

    // Used in Stepper
    FORCE_INLINE static uint8_t get_stepper(void) { return runner; }
    FORCE_INLINE static uint8_t get_next_stepper(void) {
      for (;;) {
        if (--runner < 0) runner = MIXING_STEPPERS - 1;
        accu[runner] += s_color[runner];
        if (
          #ifdef MIXER_ACCU_SIGNED
            accu[runner] < 0
          #else
            accu[runner] & COLOR_A_MASK
          #endif
        ) {
          accu[runner] &= COLOR_MASK;
          return runner;
        }
      }
    }

};

extern Mixer mixer;

#endif // ENABLED(COLOR_MIXING_EXTRUDER)

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

//#define MIXING_DEBUG

#ifdef __AVR__
  #define MIXER_ACCU_SIGNED
  #define COLOR_A_MASK  0x80
  #define COLOR_MASK    0x7F
#else
  #define COLOR_A_MASK  0x8000
  #define COLOR_MASK    0x7FFF
#endif

#define GRADIENT_MIX 2

#define MIXING_STEPPER_LOOP(VAR) \
  for (uint8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++)

#define MIXING_GRADIENT_LOOP(VAR) \
  for (uint8_t VAR = 0; VAR < GRADIENT_MIX; VAR++)

typedef struct {
  bool          enabled;                  // This gradient is enabled
  mixer_color_t color[GRADIENT_MIX];      // The current gradient color
  float         start_z, end_z;           // Region for gradient
  int8_t        start_vtool, end_vtool;   // Start and end virtual tools
  uint8_t       start_mix[GRADIENT_MIX],  // Start and end mixes from those tools
                end_mix[GRADIENT_MIX];
} gradient_t;

class Mixer {

  public: /** Constructor */

    Mixer() {}

  public: /** Public Parameters */

    static int16_t mix[MIXING_STEPPERS];  // For use by the LCD display

    static gradient_t gradient;

  private: /** Private Parameters */

    // Used up to Planner level
    static uint_fast8_t   selected_v_tool;
    static float          M163_collector[MIXING_STEPPERS];
    static mixer_color_t  color[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];

    // Used in Stepper
    static int_fast8_t    runner;
    static mixer_color_t  s_color[MIXING_STEPPERS];
    static mixer_accu_t   accu[MIXING_STEPPERS];

  public: /** Public Function */

    static void init(); // Populate colors at boot time

    // Used up to Planner level
    static void normalize(const uint8_t tool_index);
    FORCE_INLINE static void normalize() { normalize(selected_v_tool); }

    FORCE_INLINE static uint8_t get_current_v_tool(void) { return selected_v_tool; }
    FORCE_INLINE static void T(const uint_fast8_t c) { selected_v_tool = c; }
    FORCE_INLINE static void set_M163_collector(const uint8_t c, const float f) { M163_collector[c] = f; }

    // Used when dealing with blocks
    FORCE_INLINE static void populate_block(mixer_color_t b_color[MIXING_STEPPERS]) {
      if (gradient.enabled) {
        MIXING_STEPPER_LOOP(i) b_color[i] = i < 2 ? gradient.color[i] : 0;
        return;
      }
      uint_fast8_t j = get_current_v_tool();
      COPY_ARRAY(b_color, color[selected_v_tool]);
    }

    FORCE_INLINE static void stepper_setup(mixer_color_t b_color[MIXING_STEPPERS]) { MIXING_STEPPER_LOOP(i) s_color[i] = b_color[i]; }

    static inline void update_mix_from_gradient() {
      float ctot = 0;
      MIXING_GRADIENT_LOOP(i) ctot += gradient.color[i];
      clear_mix();
      mix[0] = uint8_t(CEIL(100.0f * gradient.color[0] / ctot));
      mix[1] = 100 - mix[0];
      #if ENABLED(MIXING_DEBUG)
        SERIAL_MV("Gradient [ ", int(gradient.color[0]));
        SERIAL_MV(", ", int(gradient.color[1]));
        SERIAL_MV("] to Mix [ ", mix[0]);
        SERIAL_MV(", ", mix[1]);
        SERIAL_EM("]");
      #endif
    }

    static inline void update_mix_from_vtool(const uint8_t j=selected_v_tool) {
      float ctot = 0;
      MIXING_GRADIENT_LOOP(i) ctot += color[j][i];
      clear_mix();
      mix[0] = uint8_t(CEIL(100.0f * color[j][0] / ctot));
      mix[1] = 100 - mix[0];
      #if ENABLED(MIXING_DEBUG)
        SERIAL_MV("Mixer::update_mix_from_vtool ... VTool ", int(j));
        SERIAL_MV(" [ ", int(color[j][0]));
        SERIAL_MV(", ", int(color[j][1]));
        SERIAL_MV("] to Mix [ ", mix[0]);
        SERIAL_MV(", ", mix[1]);
        SERIAL_EM("]");
      #endif
    }

    static inline void update_vtool_from_mix() {
      // Scale each component to the largest one in terms of COLOR_A_MASK
      // So the largest component will be COLOR_A_MASK and the other will be in proportion to it
      const float inverse_max = RECIPROCAL(float(MAX(mix[0], mix[1])));

      // Scale all values so their maximum is COLOR_A_MASK
      MIXING_STEPPER_LOOP(i)
        color[selected_v_tool][i] = mix[i] * COLOR_A_MASK * inverse_max;

      #if ENABLED(MIXING_DEBUG)
        SERIAL_MV("Mix [ ", mix[0]);
        SERIAL_MV(", ", mix[1]);
        SERIAL_MV("] to Color [ ", int(color[selected_v_tool][0]));
        SERIAL_MV(", ", int(color[selected_v_tool][1]));
        SERIAL_EM("]");
      #endif
    }

    static void update_gradient_for_z(const float z);
    static void update_gradient_for_planner_z();
    static void gradient_control(const float z);

    static inline void update_gradient_from_mix() {
      // Scale each component to the largest one in terms of COLOR_A_MASK
      // So the largest component will be COLOR_A_MASK and the other will be in proportion to it
      const float inverse_max = RECIPROCAL(float(MAX(mix[0], mix[1])));

      // Scale all values so their maximum is COLOR_A_MASK
      MIXING_GRADIENT_LOOP(i)
        gradient.color[i] = mix[i] * COLOR_A_MASK * inverse_max;

      #if ENABLED(MIXING_DEBUG)
        SERIAL_MV("Mix [ ", mix[0]);
        SERIAL_MV(", ", mix[1]);
        SERIAL_MV("] to Color [ ", int(gradient.color[0]));
        SERIAL_MV(", ", int(gradient.color[1]));
        SERIAL_EM("]");
      #endif
    }

    static void refresh_gradient() {
      const bool ena = (gradient.start_vtool != gradient.end_vtool && gradient.start_z < gradient.end_z);
      if ((gradient.enabled = ena)) {
        update_mix_from_vtool(gradient.start_vtool);
        MIXING_GRADIENT_LOOP(i) gradient.start_mix[i] = mix[i];
        update_mix_from_vtool(gradient.end_vtool);
        MIXING_GRADIENT_LOOP(i) gradient.end_mix[i] = mix[i];
        update_gradient_for_planner_z();
      }
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

  private: /** Private Function */

    FORCE_INLINE static void clear_mix() { MIXING_STEPPER_LOOP(i) mix[i] = 0; }

};

extern Mixer mixer;

#endif // ENABLED(COLOR_MIXING_EXTRUDER)

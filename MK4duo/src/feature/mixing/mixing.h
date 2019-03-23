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
#pragma once

/**
 * mixing.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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

#define MIXING_STEPPER_LOOP(VAR) \
  for (uint8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++)

#if HAS_GRADIENT_MIX

  typedef struct {
    bool          enabled;                    // This gradient is enabled
    mixer_color_t color[MIXING_STEPPERS];     // The current gradient color
    float         start_z, end_z;             // Region for gradient
    int8_t        start_vtool, end_vtool;     // Start and end virtual tools
    mixer_perc_t  start_mix[MIXING_STEPPERS], // Start and end mixes from those tools
                  end_mix[MIXING_STEPPERS];
  } gradient_t;

#endif

class Mixer {

  public: /** Constructor */

    Mixer() {}

  public: /** Public Parameters */

    static float collector[MIXING_STEPPERS];

    #if HAS_GRADIENT_MIX
      static mixer_perc_t mix[MIXING_STEPPERS]; // Scratch array for the Mix in proportion to 100
      static gradient_t gradient;
      static float prev_z;
    #endif

  private: /** Private Parameters */

    // Used up to Planner level
    static uint_fast8_t   selected_vtool;
    static mixer_color_t  color[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];

    // Used in Stepper
    static int_fast8_t    runner;
    static mixer_color_t  s_color[MIXING_STEPPERS];
    static mixer_accu_t   accu[MIXING_STEPPERS];

  public: /** Public Function */

    static void init(); // Populate colors at boot time

    static void reset_vtools();
    static void refresh_collector(const float proportion=1.0, const uint8_t t=selected_vtool);

    // Used up to Planner level
    FORCE_INLINE static void set_collector(const uint8_t c, const float f) { collector[c] = MAX(f, 0.0f); }

    static void normalize(const uint8_t tool_index);
    FORCE_INLINE static void normalize() { normalize(selected_vtool); }

    FORCE_INLINE static uint8_t get_current_vtool(void) { return selected_vtool; }

    FORCE_INLINE static void T(const uint_fast8_t c) {
      selected_vtool = c;
      #if HAS_GRADIENT_MIX
        update_mix_from_vtool();
      #endif
    }

    // Used when dealing with blocks
    FORCE_INLINE static void populate_block(mixer_color_t b_color[MIXING_STEPPERS]) {
      #if HAS_GRADIENT_MIX
        if (gradient.enabled) {
          MIXING_STEPPER_LOOP(i) b_color[i] = gradient.color[i];
          return;
        }
      #endif
      MIXING_STEPPER_LOOP(i) b_color[i] = color[selected_vtool][i];
    }

    FORCE_INLINE static void stepper_setup(mixer_color_t b_color[MIXING_STEPPERS]) {
      MIXING_STEPPER_LOOP(i) s_color[i] = b_color[i];
    }

    #if HAS_GRADIENT_MIX

      static inline void copy_mix_to_color(mixer_color_t (&tcolor)[MIXING_STEPPERS]) {
        // Scale each component to the largest one in terms of COLOR_A_MASK
        // So the largest component will be COLOR_A_MASK and the other will be in proportion to it
        const float scale = (COLOR_A_MASK) * RECIPROCAL(float(MAX(mix[0], mix[1])));
        // Scale all values so their maximum is COLOR_A_MASK
        MIXING_STEPPER_LOOP(i) tcolor[i] = mix[i] * scale;
      }

      static inline void update_mix_from_vtool(const uint8_t j=selected_vtool) {
        float ctot = 0;
        MIXING_STEPPER_LOOP(i) ctot += color[j][i];
        mix[0] = mixer_perc_t(100.0f * color[j][0] / ctot);
        mix[1] = 100 - mix[0];
      }

      // Update the virtual tool from an edited mix
      static inline void update_vtool_from_mix() {
        copy_mix_to_color(color[selected_vtool]);
        refresh_gradient();
      }

      static void update_gradient_for_z(const float z);
      static void update_gradient_for_planner_z();
      static inline void gradient_control(const float z) {
        if (gradient.enabled) {
          if (z >= gradient.end_z)
            T(gradient.end_vtool);
          else
            update_gradient_for_z(z);
        }
      }

      static inline void update_mix_from_gradient() {
        float ctot = 0;
        MIXING_STEPPER_LOOP(i) ctot += gradient.color[i];
        mix[0] = (mixer_perc_t)CEIL(100.0f * gradient.color[0] / ctot);
        mix[1] = 100 - mix[0];
      }

      static void refresh_gradient() {
        gradient.enabled = gradient.start_vtool != gradient.end_vtool && gradient.start_z < gradient.end_z;
        if (gradient.enabled) {
          mixer_perc_t mix_bak[MIXING_STEPPERS];
          COPY_ARRAY(mix_bak, mix);
          update_mix_from_vtool(gradient.start_vtool);
          COPY_ARRAY(gradient.start_mix, mix);
          update_mix_from_vtool(gradient.end_vtool);
          COPY_ARRAY(gradient.end_mix, mix);
          update_gradient_for_planner_z();
          COPY_ARRAY(mix, mix_bak);
          prev_z = -1;
        }
      }

    #endif // HAS_GRADIENT_MIX

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

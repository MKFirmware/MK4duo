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
 *  endstops.h - manages endstops
 */

#ifndef _ENDSTOPS_H_
#define _ENDSTOPS_H_

#if ENABLED(Z_TWO_ENDSTOPS) || ENABLED(Z_THREE_ENDSTOPS) || ENABLED(Z_FOUR_ENDSTOPS) || ENABLED(NPR2)
  typedef uint16_t esbits_t;
#else
  typedef byte esbits_t;
#endif

enum EndstopEnum {
  X_MIN,
  Y_MIN,
  Z_MIN,
  Z_PROBE,
  X_MAX,
  Y_MAX,
  Z_MAX,
  Z2_MIN,
  Z2_MAX,
  Z3_MIN,
  Z3_MAX,
  Z4_MIN,
  Z4_MAX,
  E_MIN
};

class Endstops {

  public: /** Constructor */

    Endstops() {
      printer.setEndstopGlobally(
        #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
          false
        #else
          true
        #endif
      );
    };

  public: /** Public Parameters */

    #if IS_KINEMATIC
      static float  soft_endstop_radius_2;
    #else
      static float  soft_endstop_min[XYZ],
                    soft_endstop_max[XYZ];
    #endif

    #if ENABLED(Z_FOUR_ENDSTOPS)
      static float  z2_endstop_adj,
                    z3_endstop_adj,
                    z4_endstop_adj;
    #elif ENABLED(Z_THREE_ENDSTOPS)
      static float  z2_endstop_adj,
                    z3_endstop_adj;
    #elif ENABLED(Z_TWO_ENDSTOPS)
      static float  z2_endstop_adj;
    #endif

    static volatile char endstop_hit_bits; // use X_MIN, Y_MIN, Z_MIN and Z_PROBE as BIT value

    static volatile uint8_t e_hit;  // Different from 0 when the endstops shall be tested in detail.
                                    // Must be reset to 0 by the test function when the tests are finished.

    static esbits_t current_endstop_bits, old_endstop_bits;

  public: /** Public Function */

    /**
     * Initialize the endstop pins
     */
    void init();

    /**
     * Update the endstops bits from the pins
     */
    static void update();

    /**
     * Print an error message reporting the position when the endstops were last hit.
     */
    static void report_state(); // call from somewhere to create an serial error message with the locations the endstops where hit, in case they were triggered

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    static void hit_on_purpose() { endstop_hit_bits = 0; }

    // Constrain the given coordinates to the software endstops.
    void clamp_to_software_endstops(float target[XYZ]);

    #if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
      void update_software_endstops(const AxisEnum axis);
    #endif

    #if ENABLED(PINS_DEBUGGING)
      static void endstop_monitor();
    #endif

    #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
      static void setup_endstop_interrupts(void);
    #endif

  private: /** Private Function */

    #if ENABLED(Z_FOUR_ENDSTOPS)
      static void test_four_z_endstops(const EndstopEnum es1, const EndstopEnum es2, const EndstopEnum es3, const EndstopEnum es4);
    #elif ENABLED(Z_THREE_ENDSTOPS)
      static void test_three_z_endstops(const EndstopEnum es1, const EndstopEnum es2, const EndstopEnum es3);
    #elif ENABLED(Z_TWO_ENDSTOPS)
      static void test_two_z_endstops(const EndstopEnum es1, const EndstopEnum es2);
    #endif

};

extern Endstops endstops;

#if HAS_BED_PROBE
  #define ENDSTOPS_ENABLED  (printer.IsEndstopEnabled() || printer.IsProbeEndstop())
#else
  #define ENDSTOPS_ENABLED  printer.IsEndstopEnabled()
#endif

#endif /* _ENDSTOPS_H_ */

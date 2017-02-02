/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#ifndef ENDSTOPS_H
#define ENDSTOPS_H

class Endstops {

  public:

    static bool enabled, enabled_globally;
    static volatile char endstop_hit_bits; // use X_MIN, Y_MIN, Z_MIN and Z_PROBE as BIT value

    #if ENABLED(Z_TWO_ENDSTOPS) || ENABLED(Z_THREE_ENDSTOPS) || ENABLED(Z_FOUR_ENDSTOPS)
      static uint16_t
    #else
      static byte
    #endif
        current_endstop_bits, old_endstop_bits;

    Endstops() {};

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

    /**
     * Report endstop positions in response to M119
     */
    static void M119();

    // Enable / disable endstop checking globally
    static void enable_globally(bool onoff = true) { enabled_globally = enabled = onoff; }

    // Enable / disable endstop checking
    static void enable(bool onoff = true) { enabled = onoff; }

    // Disable / Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    static void not_homing() { enabled = enabled_globally; }

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    static void hit_on_purpose() { endstop_hit_bits = 0; }

    // Enable / disable endstop z-probe checking
    #if HAS(BED_PROBE)
      static volatile bool z_probe_enabled;
      static void enable_z_probe(bool onoff = true) { z_probe_enabled = onoff; }
    #endif

  private:

    #if ENABLED(Z_FOUR_ENDSTOPS)
      static void test_four_z_endstops(EndstopEnum es1, EndstopEnum es2, EndstopEnum es3, EndstopEnum es4);
    #elif ENABLED(Z_THREE_ENDSTOPS)
      static void test_three_z_endstops(EndstopEnum es1, EndstopEnum es2, EndstopEnum es3);
    #elif ENABLED(Z_TWO_ENDSTOPS)
      static void test_two_z_endstops(EndstopEnum es1, EndstopEnum es2);
    #endif
};

extern Endstops endstops;

#if HAS(BED_PROBE)
  #define ENDSTOPS_ENABLED  (endstops.enabled || endstops.z_probe_enabled)
#else
  #define ENDSTOPS_ENABLED  endstops.enabled
#endif

#endif // ENDSTOPS_H

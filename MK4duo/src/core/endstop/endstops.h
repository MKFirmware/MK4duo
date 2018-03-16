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

enum Flag1Enum {
  flag1_endstop_enabled,
  flag1_endstop_globally,
  flag1_soft_endstop,
  flag1_probe_endstop,
  flag1_g38_endstop_hit
};
  
enum EndstopEnum {
  X_MIN,
  Y_MIN,
  Z_MIN,
  Z_PROBE,
  X_MAX,
  Y_MAX,
  Z_MAX,
  X2_MIN,
  X2_MAX,
  Y2_MIN,
  Y2_MAX,
  Z2_MIN,
  Z2_MAX,
  FIL_RUNOUT,
  DOOR_OPEN_SENSOR,
  POWER_CHECK_SENSOR
};

class Endstops {

  public: /** Constructor */

    Endstops() {
      setGlobally(
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

    #if ENABLED(X_TWO_ENDSTOPS)
      static float x_endstop_adj;
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      static float y_endstop_adj;
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      static float z_endstop_adj;
    #endif

    static volatile char hit_bits; // use X_MIN, Y_MIN, Z_MIN and Z_PROBE as BIT value

    static volatile uint8_t e_hit;  // Different from 0 when the endstops shall be tested in detail.
                                    // Must be reset to 0 by the test function when the tests are finished.

    static uint16_t logic_bits,
                    pullup_bits,
                    current_bits,
                    old_bits;

  private: /** Private Parameters */

    static uint8_t  flag1_bits;

  public: /** Public Function */

    /**
     * Initialize the endstop pins
     */
    void init();

    /**
     * Setup Pullup
     */
    void setup_pullup();

    /**
     * Update the endstops bits from the pins
     */
    static void update();

    /**
     * Print logicl and pullup
     */
    static void report();

    /**
     * Print an error message reporting the position when the endstops were last hit.
     */
    static void report_state(); // call from somewhere to create an serial error message with the locations the endstops where hit, in case they were triggered

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    static void hit_on_purpose() { hit_bits = 0; }

    // Constrain the given coordinates to the software endstops.
    void clamp_to_software_endstops(float target[XYZ]);

    #if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
      void update_software_endstops(const AxisEnum axis);
    #endif

    #if ENABLED(PINS_DEBUGGING)
      static void endstop_monitor();
    #endif

    FORCE_INLINE static void setLogic(const EndstopEnum endstop, const bool logic) {
      SET_BIT(logic_bits, endstop, logic);
    }
    FORCE_INLINE static bool isLogic(const EndstopEnum endstop) { return TEST(logic_bits, endstop); }

    FORCE_INLINE static void setPullup(const EndstopEnum endstop, const bool pullup) {
      SET_BIT(pullup_bits, endstop, pullup);
    }
    FORCE_INLINE static bool isPullup(const EndstopEnum endstop) { return TEST(pullup_bits, endstop); }

    FORCE_INLINE static void setEnabled(const bool onoff) {
      SET_BIT(flag1_bits, flag1_endstop_enabled, onoff);
    }
    FORCE_INLINE static bool isEnabled() { return TEST(flag1_bits, flag1_endstop_enabled); }

    FORCE_INLINE static void setGlobally(const bool onoff) {
      SET_BIT(flag1_bits, flag1_endstop_globally, onoff);
      setEnabled(onoff);
    }
    FORCE_INLINE static bool isGlobally() { return TEST(flag1_bits, flag1_endstop_globally); }

    FORCE_INLINE static void setSoftEndstop(const bool onoff) {
      SET_BIT(flag1_bits, flag1_soft_endstop, onoff);
    }
    FORCE_INLINE static bool isSoftEndstop() { return TEST(flag1_bits, flag1_soft_endstop); }

    FORCE_INLINE static void setProbeEndstop(const bool onoff) {
      SET_BIT(flag1_bits, flag1_probe_endstop, onoff);
    }
    FORCE_INLINE static bool isProbeEndstop() { return TEST(flag1_bits, flag1_probe_endstop); }

    FORCE_INLINE static void setG38EndstopHit(const bool onoff) {
      SET_BIT(flag1_bits, flag1_g38_endstop_hit, onoff);
    }
    FORCE_INLINE static bool isG38EndstopHit() { return TEST(flag1_bits, flag1_g38_endstop_hit); }

    // Disable-Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    FORCE_INLINE static void setNotHoming() { setEnabled(isGlobally()); }

  private: /** Private Function */

    #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
      static void setup_endstop_interrupts(void);
    #endif

    #if ENABLED(X_TWO_ENDSTOPS)
      static void test_two_x_endstops(const EndstopEnum es1, const EndstopEnum es2);
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      static void test_two_y_endstops(const EndstopEnum es1, const EndstopEnum es2);
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      static void test_two_z_endstops(const EndstopEnum es1, const EndstopEnum es2);
    #endif

};

extern Endstops endstops;

#if HAS_BED_PROBE
  #define ENDSTOPS_ENABLED  (endstops.isEnabled() || endstops.isProbeEndstop())
#else
  #define ENDSTOPS_ENABLED  endstops.isEnabled()
#endif

#endif /* _ENDSTOPS_H_ */

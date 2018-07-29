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

enum Flag1Enum : char {
  bit_endstop_enabled,
  bit_endstop_globally,
  bit_soft_endstop,
  bit_probe_endstop,
  bit_g38_endstop_hit,
  bit_monitor_flag
};
  
enum EndstopEnum : char {
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

    Endstops() {}

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

    static uint16_t logic_bits,
                    pullup_bits,
                    live_state;

  private: /** Private Parameters */

    static uint8_t  flag_bits;

    static volatile uint8_t hit_state; // use X_MIN, Y_MIN, Z_MIN and Z_PROBE as BIT value

  public: /** Public Function */

    /**
     * Initialize the endstop pins
     */
    static void init();

    /**
     * Initialize Factory parameters
     */
    static void factory_parameters();

    /**
     * Setup Pullup
     */
    static void setup_pullup();

    /**
     * Periodic call to Tick endstops if required.
     */
    static void Tick();

    /**
     * Update endstops bits from the pins. Apply filtering to get a verified state.
     * If should_check() and moving towards a triggered switch, abort the current move.
     * Called from ISR contexts.
     */
    static void update();

    /**
     * Get Endstop hit state.
     */
    FORCE_INLINE static uint8_t trigger_state() { return hit_state; }

    /**
     * Print logical and pullup
     */
    static void report();

    /**
     * Report endstop hits to serial. Called from loop().
     */
    static void report_state();

    // If the last move failed to trigger an endstop, call kill
    static void validate_homing_move();

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    FORCE_INLINE static void hit_on_purpose() { hit_state = 0; }

    // Constrain the given coordinates to the software endstops.
    static void clamp_to_software(float target[XYZ]);

    #if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
      static void update_software_endstops(const AxisEnum axis);
    #endif

    #if ENABLED(PINS_DEBUGGING)
      static void run_monitor();
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
      SET_BIT(flag_bits, bit_endstop_enabled, onoff);
      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        update();
      #endif
    }
    FORCE_INLINE static bool isEnabled() { return TEST(flag_bits, bit_endstop_enabled); }

    FORCE_INLINE static void setGlobally(const bool onoff) {
      SET_BIT(flag_bits, bit_endstop_globally, onoff);
      setEnabled(onoff);
    }
    FORCE_INLINE static bool isGlobally() { return TEST(flag_bits, bit_endstop_globally); }

    FORCE_INLINE static void setSoftEndstop(const bool onoff) {
      SET_BIT(flag_bits, bit_soft_endstop, onoff);
    }
    FORCE_INLINE static bool isSoftEndstop() { return TEST(flag_bits, bit_soft_endstop); }

    FORCE_INLINE static void setProbeEnabled(const bool onoff) {
      SET_BIT(flag_bits, bit_probe_endstop, onoff);
      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        update();
      #endif
    }
    FORCE_INLINE static bool isProbeEnabled() { return TEST(flag_bits, bit_probe_endstop); }

    FORCE_INLINE static void setG38EndstopHit(const bool onoff) {
      SET_BIT(flag_bits, bit_g38_endstop_hit, onoff);
    }
    FORCE_INLINE static bool isG38EndstopHit() { return TEST(flag_bits, bit_g38_endstop_hit); }

    FORCE_INLINE static void setMonitorEnabled(const bool onoff) {
      SET_BIT(flag_bits, bit_monitor_flag, onoff);
    }
    FORCE_INLINE static bool isMonitorEnabled() { return TEST(flag_bits, bit_monitor_flag); }

    // Disable-Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    FORCE_INLINE static void setNotHoming() { setEnabled(isGlobally()); }

  private: /** Private Function */

    #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
      static void setup_interrupts(void);
    #endif

    #if ENABLED(PINS_DEBUGGING)
      static void monitor();
    #endif

};

extern Endstops endstops;

#endif /* _ENDSTOPS_H_ */

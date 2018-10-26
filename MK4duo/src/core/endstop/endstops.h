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

class Endstops {

  public: /** Constructor */

    Endstops() {}

  public: /** Public Parameters */

    #if IS_DELTA
      static float  soft_endstop_radius_2;
    #else
      static float  soft_endstop_min[XYZ],
                    soft_endstop_max[XYZ];
    #endif

    #if ENABLED(X_TWO_ENDSTOPS)
      static float  x2_endstop_adj;
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      static float  y2_endstop_adj;
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      static float  z2_endstop_adj;
    #endif
    #if ENABLED(Z_THREE_ENDSTOPS)
      static float  z2_endstop_adj,
                    z3_endstop_adj;
    #endif

    static flagword_t logic_flag,
                      pullup_flag;

    static uint16_t live_state;

  private: /** Private Parameters */

    static flagbyte_t  flag;

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
     * Print endstops parameters in memory
     */
    static void print_parameters();

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
      SET_BIT(logic_flag._word, endstop, logic);
    }
    FORCE_INLINE static bool isLogic(const EndstopEnum endstop) { return TEST(logic_flag._word, endstop); }

    FORCE_INLINE static void setPullup(const EndstopEnum endstop, const bool pullup) {
      SET_BIT(pullup_flag._word, endstop, pullup);
    }
    FORCE_INLINE static bool isPullup(const EndstopEnum endstop) { return TEST(pullup_flag._word, endstop); }

    // Flag bit 0 Endstop enabled
    FORCE_INLINE static void setEnabled(const bool onoff) {
      flag.bit0 = onoff;
      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        update();
      #endif
    }
    FORCE_INLINE static bool isEnabled() { return flag.bit0; }

    // Flag bit 1 setGlobally
    FORCE_INLINE static void setGlobally(const bool onoff) {
      flag.bit1 = onoff;
      setEnabled(onoff);
    }
    FORCE_INLINE static bool isGlobally() { return flag.bit1; }

    // Flag bit 2 set Software Endstop
    FORCE_INLINE static void setSoftEndstop(const bool onoff) {
      flag.bit2 = onoff;
    }
    FORCE_INLINE static bool isSoftEndstop() { return flag.bit2; }

    // Flag bit 3 set Probe Enabled
    FORCE_INLINE static void setProbeEnabled(const bool onoff) {
      flag.bit3 = onoff;
      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        update();
      #endif
    }
    FORCE_INLINE static bool isProbeEnabled() { return flag.bit3; }

    // Flag bit 4 set G38 Endstop Hit
    FORCE_INLINE static void setG38EndstopHit(const bool onoff) {
      flag.bit4 = onoff;
    }
    FORCE_INLINE static bool isG38EndstopHit() { return flag.bit4; }

    // Flag bit 5 set Monitor Enabled
    FORCE_INLINE static void setMonitorEnabled(const bool onoff) {
      flag.bit5 = onoff;
    }
    FORCE_INLINE static bool isMonitorEnabled() { return flag.bit5; }

    // Disable-Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    FORCE_INLINE static void setNotHoming() {
      setEnabled(isGlobally());
      if (!isEnabled()) live_state = 0;
    }

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

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
 * probe.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _PROBE_H_
#define _PROBE_H_

// TRIGGERED_WHEN_STOWED_TEST can easily be extended to servo probes, ... if needed.
#if ENABLED(PROBE_IS_TRIGGERED_WHEN_STOWED_TEST)
  #if HAS_Z_PROBE_PIN
    #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_PROBE_PIN) != Z_PROBE_ENDSTOP_INVERTING)
  #else
    #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING)
  #endif
#endif

#if HAS_Z_SERVO_PROBE
  #define DEPLOY_Z_SERVO() MOVE_SERVO(Z_ENDSTOP_SERVO_NR, probe.z_servo_angle[0])
  #define STOW_Z_SERVO()   MOVE_SERVO(Z_ENDSTOP_SERVO_NR, probe.z_servo_angle[1])
#endif

#if HAS_BED_PROBE
  #define DEPLOY_PROBE()  probe.set_deployed(true)
  #define STOW_PROBE()    probe.set_deployed(false)
#else
  #define DEPLOY_PROBE()
  #define STOW_PROBE()
#endif

class Probe {

  public: /** Constructor */

    Probe() {};

  public: /** Public Parameters */

    static float  offset[XYZ];
    static bool   enabled;

    #if HAS_Z_SERVO_PROBE
      static const int z_servo_angle[2];
    #endif

  public: /** Public Function */

    /**
     * Enable / disable endstop z-probe checking
     */
    static void set_enable(bool onoff = true) { enabled = onoff; }

    static bool set_deployed(const bool deploy);

    /**
     * Check Pt (ex probe_pt)
     * - Move to the given XY
     * - Deploy the probe, if not already deployed
     * - Probe the bed, get the Z position
     * - Depending on the 'stow' flag
     *   - Stow the probe, or
     *   - Raise to the BETWEEN height
     * - Return the probed Z position
     */
    static float check_pt(const float &lx, const float &ly, const bool stow=true, const int verbose_level=1, const bool printable=true);

    #if QUIET_PROBING
      static void probing_pause(const bool p);
    #endif

    #if ENABLED(BLTOUCH)
      static void bltouch_command(int angle);
      static bool set_bltouch_deployed(const bool deploy);
    #endif

    static void refresh_offset(const bool no_babystep=false);

  private: /** Private Parameters */

  private: /** Private Function */

    /**
     * @brief Used by run_z_probe to do a single Z probe move.
     *
     * @param  z        Z destination
     * @param  fr_mm_s  Feedrate in mm/s
     * @return true to indicate an error
     */
    static bool move_to_z(const float z, const float fr_mm_m);

    /**
     * @details Used by check_pt to do a single Z probe.
     *          Leaves current_position[Z_AXIS] at the height where the probe triggered.
     *
     * @param  short_move Flag for a shorter probe move towards the bed
     * @return The raw Z position where the probe was triggered
     */
    static float run_z_probe(const bool short_move=true);

    #if ENABLED(Z_PROBE_ALLEN_KEY)
      static void run_deploy_moves_script();
    #endif

    #if HAS_Z_PROBE_SLED
      static void dock_sled(bool stow);
    #endif

};

extern Probe probe;

#if IS_DELTA
  // Check for this in the code instead
  #define MIN_PROBE_X -(mechanics.delta_print_radius)
  #define MAX_PROBE_X  (mechanics.delta_print_radius)
  #define MIN_PROBE_Y -(mechanics.delta_print_radius)
  #define MAX_PROBE_Y  (mechanics.delta_print_radius)
#elif IS_SCARA
    #define SCARA_PRINTABLE_RADIUS (SCARA_LINKAGE_1 + SCARA_LINKAGE_2)
    #define MIN_PROBE_X (X_CENTER - SCARA_PRINTABLE_RADIUS)
    #define MAX_PROBE_X (X_CENTER + SCARA_PRINTABLE_RADIUS)
    #define MIN_PROBE_Y (Y_CENTER - SCARA_PRINTABLE_RADIUS)
    #define MAX_PROBE_Y (Y_CENTER + SCARA_PRINTABLE_RADIUS)
#else
  // Boundaries for probing based on set limits
  #define MIN_PROBE_X (max(X_MIN_POS, X_MIN_POS + probe.offset[X_AXIS]))
  #define MAX_PROBE_X (min(X_MAX_POS, X_MAX_POS + probe.offset[X_AXIS]))
  #define MIN_PROBE_Y (max(Y_MIN_POS, Y_MIN_POS + probe.offset[Y_AXIS]))
  #define MAX_PROBE_Y (min(Y_MAX_POS, Y_MAX_POS + probe.offset[Y_AXIS]))
#endif

#endif /* _PROBE_H_ */

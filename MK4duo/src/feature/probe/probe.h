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
    #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_PROBE_PIN) != endstops.isLogic(Z_PROBE))
  #else
    #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_MIN_PIN) != endstops.isLogic(Z_MIN))
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

    static float offset[XYZ];

    #if HAS_Z_SERVO_PROBE
      static const int z_servo_angle[2];
    #endif

  public: /** Public Function */

    static bool set_deployed(const bool deploy);

    #if Z_PROBE_AFTER_PROBING > 0
      static void move_z_after_probing();
    #endif

    #if HAS_BED_PROBE || ENABLED(PROBE_MANUALLY)

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
      static float check_pt(const float &rx, const float &ry, const bool stow, const int verbose_level, const bool probe_relative=true);

    #endif

    #if QUIET_PROBING
      static void probing_pause(const bool onoff);
    #endif

    #if ENABLED(BLTOUCH)
      static void bltouch_command(int angle);
      static bool set_bltouch_deployed(const bool deploy);
      FORCE_INLINE void bltouch_init() {
        // Make sure any BLTouch error condition is cleared
        bltouch_command(BLTOUCH_RESET);
        set_bltouch_deployed(true);
        set_bltouch_deployed(false);
      }
    #endif

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
     * @return The raw Z position where the probe was triggered
     */
    static float run_z_probe();

    #if ENABLED(Z_PROBE_ALLEN_KEY)
      static void run_deploy_moves_script();
    #endif

    #if HAS_Z_PROBE_SLED
      static void dock_sled(bool stow);
    #endif

};

extern Probe probe;

#endif /* _PROBE_H_ */

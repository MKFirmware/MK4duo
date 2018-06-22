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

#if HAS_Z_SERVO_PROBE
  #define DEPLOY_Z_SERVO() MOVE_SERVO(Z_PROBE_SERVO_NR, probe.z_servo_angle[0])
  #define STOW_Z_SERVO()   MOVE_SERVO(Z_PROBE_SERVO_NR, probe.z_servo_angle[1])
#endif

#if HAS_BED_PROBE
  #define DEPLOY_PROBE()  probe.set_deployed(true)
  #define STOW_PROBE()    probe.set_deployed(false)
#else
  #define DEPLOY_PROBE()
  #define STOW_PROBE()
#endif

enum ProbePtRaise : unsigned char {
  PROBE_PT_NONE,  // No raise or stow after run_probing
  PROBE_PT_STOW,  // Do a complete stow after run_probing
  PROBE_PT_RAISE  // Raise to "between" clearance after run_probing
};

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
      static float check_pt(const float &rx, const float &ry, const ProbePtRaise raise_after=PROBE_PT_NONE, const uint8_t verbose_level=0, const bool probe_relative=true);

    #endif

    #if QUIET_PROBING
      static void probing_pause(const bool onoff);
    #endif

    #if ENABLED(BLTOUCH)
      static void bltouch_command(const int angle);
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

    static bool move_to_z(const float z, const float fr_mm_s);

    static void do_raise(const float z_raise);

    static float run_probing();

    #if ENABLED(Z_PROBE_ALLEN_KEY)
      static void run_deploy_moves_script();
    #endif

    #if HAS_Z_PROBE_SLED
      static void dock_sled(bool stow);
    #endif

};

extern Probe probe;

#endif /* _PROBE_H_ */

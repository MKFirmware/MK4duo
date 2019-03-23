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
 * probe.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if HAS_Z_SERVO_PROBE
  #define DEPLOY_Z_SERVO() MOVE_SERVO(Z_PROBE_SERVO_NR, servo[Z_PROBE_SERVO_NR].angle[0])
  #define STOW_Z_SERVO()   MOVE_SERVO(Z_PROBE_SERVO_NR, servo[Z_PROBE_SERVO_NR].angle[1])
#endif

#if HAS_BED_PROBE
  #define DEPLOY_PROBE()  probe.set_deployed(true)
  #define STOW_PROBE()    probe.set_deployed(false)
#else
  #define DEPLOY_PROBE()
  #define STOW_PROBE()
#endif

// Struct Probe data
typedef struct {

  float     offset[XYZ];

  uint16_t  speed_fast,
            speed_slow;

  uint8_t   repetitions;

} probe_data_t;

class Probe {

  public: /** Constructor */

    Probe() {};

  public: /** Public Parameters */

    static probe_data_t data;

  public: /** Public Function */

    /**
     * Initialize Factory parameters
     */
    static void factory_parameters();

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
      static float check_pt(const float &rx, const float &ry, const ProbePtRaiseEnum raise_after=PROBE_PT_NONE, const uint8_t verbose_level=0, const bool probe_relative=true);

    #endif

    #if QUIET_PROBING
      static void probing_pause(const bool onoff);
    #endif

    static void servo_test();

  private: /** Private Function */

    static bool specific_action(const bool deploy);

    static bool move_to_z(const float z, const float fr_mm_s);

    static void do_raise(const float z_raise);

    static float run_probing();

    #if ENABLED(Z_PROBE_ALLEN_KEY)
      static void run_deploy_moves_script();
      static void run_stow_moves_script();
    #endif

    #if HAS_Z_PROBE_SLED
      static void dock_sled(bool stow);
    #endif

};

extern Probe probe;

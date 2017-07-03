/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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

#if HAS_BED_PROBE

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

  class Probe {

    public: /** Constructor */

      Probe() {};

    public: /** Public Parameters */

      static float  z_offset;
      static bool   enabled;

      #if HAS_Z_SERVO_PROBE
        static const int z_servo_angle[2];
      #endif

    public: /** Public Function */

      /**
       * Enable / disable endstop z-probe checking
       */
      static void set_enable(bool onoff = true) { enabled = onoff; }

      static bool set_deployed(bool deploy);

      /**
       * Raise Z to a minimum height to make room for a servo to move
       */
      static void raise(const float z_raise);

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

      /**
       * Do a single Z probe
       * Usage:
       *    G30 <X#> <Y#> <S#> <Z#> <P#>
       *      X = Probe X position (default=current probe position)
       *      Y = Probe Y position (default=current probe position)
       *      S = <bool> Stows the probe if 1 (default=1)
       *      Z = <bool> with a non-zero value will apply the result to current delta_height (ONLY DELTA)
       *      P = <bool> with a non-zero value will apply the result to current probe.z_offset (ONLY DELTA)
       */
      static void single_probe();

      #if ENABLED(BLTOUCH)
        static void bltouch_command(int angle);
        static void set_bltouch_deployed(const bool deploy);
      #endif

      static void refresh_zprobe_zoffset();

    private: /** Private Parameters */

    private: /** Private Function */

      static void move_to_z(float z, float fr_mm_m);

  };

  extern Probe probe;

#endif // HAS_BED_PROBE

#endif /* _PROBE_H_ */

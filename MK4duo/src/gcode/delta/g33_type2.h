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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(DELTA_AUTO_CALIBRATION_2)

  #define CODE_G33

  constexpr uint8_t _7P_STEP = 1,              // 7-point step - to change number of calibration points
                    _4P_STEP = _7P_STEP * 2,   // 4-point step
                    NPP      = _7P_STEP * 6;   // number of calibration points on the radius
  enum CalEnum {                               // the 7 main calibration points - add definitions if needed
    CEN      = 0,
    __A      = 1,
    _AB      = __A + _7P_STEP,
    __B      = _AB + _7P_STEP,
    _BC      = __B + _7P_STEP,
    __C      = _BC + _7P_STEP,
    _CA      = __C + _7P_STEP,
  };

  #define LOOP_CAL_PT(VAR, S, N) for (uint8_t VAR=S; VAR<=NPP; VAR+=N)
  #define F_LOOP_CAL_PT(VAR, S, N) for (float VAR=S; VAR<NPP+0.9999; VAR+=N)
  #define I_LOOP_CAL_PT(VAR, S, N) for (float VAR=S; VAR>CEN+0.9999; VAR-=N)
  #define LOOP_CAL_ALL(VAR) LOOP_CAL_PT(VAR, CEN, 1)
  #define LOOP_CAL_RAD(VAR) LOOP_CAL_PT(VAR, __A, _7P_STEP)
  #define LOOP_CAL_ACT(VAR, _4P, _OP) LOOP_CAL_PT(VAR, _OP ? _AB : __A, _4P ? _4P_STEP : _7P_STEP)

  void Calibration_cleanup(
    #if HOTENDS > 1
      const uint8_t old_tool_index
    #endif
  ) {
    #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
      mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
    #endif
    STOW_PROBE();
    printer.clean_up_after_endstop_or_probe_move();
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif
  }

  void Report_signed_float(const char * const prefix, const float &f) {
    SERIAL_MSG("  ");
    SERIAL_PS(prefix);
    SERIAL_CHR(':');
    if (f >= 0) SERIAL_CHR('+');
    SERIAL_VAL(f, 2);
  }

  void Report_settings(const bool end_stops, const bool tower_angles) {
    SERIAL_MV(".Height:", mechanics.delta_height, 2);
    if (end_stops) {
      Report_signed_float(PSTR("  Ex"), mechanics.delta_endstop_adj[A_AXIS]);
      Report_signed_float(PSTR("Ey"), mechanics.delta_endstop_adj[B_AXIS]);
      Report_signed_float(PSTR("Ez"), mechanics.delta_endstop_adj[C_AXIS]);
      SERIAL_MV("    Radius:", mechanics.delta_radius, 2);
    }
    SERIAL_EOL();
    if (tower_angles) {
      SERIAL_MSG(".Tower angle :  ");
      Report_signed_float(PSTR("Tx"), mechanics.delta_tower_angle_adj[A_AXIS]);
      Report_signed_float(PSTR("Ty"), mechanics.delta_tower_angle_adj[B_AXIS]);
      Report_signed_float(PSTR("Tz"), mechanics.delta_tower_angle_adj[C_AXIS]);
      SERIAL_EMV("       Rod:", mechanics.delta_diagonal_rod, 2);
    }
  }

  static void Report_results(const float z_at_pt[NPP + 1], const bool tower_points, const bool opposite_points) {
    SERIAL_MSG(".    ");
    Report_signed_float(PSTR("c"), z_at_pt[CEN]);
    if (tower_points) {
      Report_signed_float(PSTR(" x"), z_at_pt[__A]);
      Report_signed_float(PSTR(" y"), z_at_pt[__B]);
      Report_signed_float(PSTR(" z"), z_at_pt[__C]);
    }
    if (tower_points && opposite_points) {
      SERIAL_EOL();
      SERIAL_CHR('.');
      SERIAL_SP(13);
    }
    if (opposite_points) {
      Report_signed_float(PSTR("yz"), z_at_pt[_BC]);
      Report_signed_float(PSTR("zx"), z_at_pt[_CA]);
      Report_signed_float(PSTR("xy"), z_at_pt[_AB]);
    }
    SERIAL_EOL();
  }

  static float probe_points(float z_at_pt[NPP + 1], const int8_t num_points, const bool towers_set, const bool stow_after_each) {

    const bool  _0p_calibration      = num_points == 0,
                _1p_calibration      = num_points == 1,
                _4p_calibration      = num_points == 2,
                _4p_opposite_points  = _4p_calibration && !towers_set,
                _7p_calibration      = num_points >= 3 || num_points == 0,
                _7p_no_intermediates = num_points == 3,
                _7p_1_intermediates  = num_points == 4,
                _7p_2_intermediates  = num_points == 5,
                _7p_4_intermediates  = num_points == 6,
                _7p_6_intermediates  = num_points == 7,
                _7p_8_intermediates  = num_points == 8,
                _7p_11_intermediates = num_points == 9,
                _7p_14_intermediates = num_points == 10,
                _7p_intermed_points  = num_points >= 4,
                _7p_6_centre         = num_points >= 5 && num_points <= 7,
                _7p_9_centre         = num_points >= 8;

    LOOP_CAL_ALL(axis) z_at_pt[axis] = 0.0;

    if (!_0p_calibration) {

      if (!_7p_no_intermediates && !_7p_4_intermediates && !_7p_11_intermediates) // probe the center
        z_at_pt[CEN] += probe.check_pt(probe.offset[X_AXIS], probe.offset[Y_AXIS], stow_after_each, 1, false);

      if (_7p_calibration) {  // probe extra center points
        const float start  = _7p_9_centre ? _CA + _7P_STEP / 3.0 : _7p_6_centre ? _CA : __C,
                    steps  = _7p_9_centre ? _4P_STEP / 3.0 : _7p_6_centre ? _7P_STEP : _4P_STEP;
        I_LOOP_CAL_PT(axis, start, steps) {
          const float a = RADIANS(210 + (360 / NPP) *  (axis - 1)),
                      r = mechanics.delta_probe_radius * 0.1;
          z_at_pt[CEN] += probe.check_pt(COS(a) * r + probe.offset[X_AXIS], SIN(a) * r + probe.offset[Y_AXIS], stow_after_each, 1);
        }
        z_at_pt[CEN] /= float(_7p_2_intermediates ? 7 : num_points);
      }

      if (!_1p_calibration) {   // probe the radius
        const CalEnum start  =  _4p_opposite_points ? _AB : __A;
        const float   steps  =  _7p_14_intermediates ? _7P_STEP / 15.0 : // 15r * 6 + 10c = 100
                                _7p_11_intermediates ? _7P_STEP / 12.0 : // 12r * 6 +  9c = 81
                                _7p_8_intermediates  ? _7P_STEP /  9.0 : //  9r * 6 + 10c = 64
                                _7p_6_intermediates  ? _7P_STEP /  7.0 : //  7r * 6 +  7c = 49
                                _7p_4_intermediates  ? _7P_STEP /  5.0 : //  5r * 6 +  6c = 36
                                _7p_2_intermediates  ? _7P_STEP /  3.0 : //  3r * 6 +  7c = 25
                                _7p_1_intermediates  ? _7P_STEP /  2.0 : //  2r * 6 +  4c = 16
                                _7p_no_intermediates ? _7P_STEP :        //  1r * 6 +  3c = 9
                                _4P_STEP;                                // .5r * 6 +  1c = 4

        bool zig_zag = true;
        F_LOOP_CAL_PT(axis, start, _7p_9_centre ? steps * 3 : steps) {
          const int8_t offset = _7p_9_centre ? 1 : 0;
          for (int8_t circle = -offset; circle <= offset; circle++) {
            const float a = RADIANS(210 + (360 / NPP) *  (axis - 1)),
                        r = mechanics.delta_probe_radius * (1 + 0.1 * (zig_zag ? circle : - circle)),
                        interpol = FMOD(axis, 1);
            float z_temp = probe.check_pt(COS(a) * r + probe.offset[X_AXIS], SIN(a) * r + probe.offset[Y_AXIS], stow_after_each, 1);
            // split probe point to neighbouring calibration points
            z_at_pt[uint8_t(round(axis - interpol + NPP - 1)) % NPP + 1] += z_temp * sq(COS(RADIANS(interpol * 90)));
            z_at_pt[uint8_t(round(axis - interpol))           % NPP + 1] += z_temp * sq(SIN(RADIANS(interpol * 90)));
          }
          zig_zag = !zig_zag;
        }

        if (_7p_intermed_points) {
          LOOP_CAL_RAD(axis)
            z_at_pt[axis] /= _7P_STEP / steps;
        }
      }

      float S1  = z_at_pt[CEN],
            S2  = sq(z_at_pt[CEN]);
      int16_t N = 1;
      if (!_1p_calibration) { // std dev from zero plane
        LOOP_CAL_ACT(axis, _4p_calibration, _4p_opposite_points) {
          S1 += z_at_pt[axis];
          S2 += sq(z_at_pt[axis]);
          N++;
        }
        return round(SQRT(S2 / N) * 1000.0) / 1000.0 + 0.00001;
      }
    }

    return 0.00001;
  }

  #if DISABLED(PROBE_MANUALLY)

    static void tune() {

      float z_at_pt[NPP + 1]      = { 0.0 },
            z_at_pt_base[NPP + 1] = { 0.0 },
            h_fac                 = 0.0,
            r_fac                 = 0.0,
            a_fac                 = 0.0,
            norm                  = 0.8;

      #define ZP(N,I) ((N) * z_at_pt[I])
      #define Z06(I)  ZP(6, I)
      #define Z03(I)  ZP(3, I)
      #define Z02(I)  ZP(2, I)
      #define Z01(I)  ZP(1, I)
      #define Z32(I)  ZP(3/2, I)

      SERIAL_EM("AUTO TUNE baseline");
      probe_points(z_at_pt_base, 3, true, false);
      Report_results(z_at_pt_base, true, true);

      // Tuning Endstop
      LOOP_XYZ(axis) {

        mechanics.delta_endstop_adj[axis] -= 1.0;

        endstops.enable(true);
        if (!mechanics.Home()) return;
        endstops.not_homing();

        SERIAL_MSG("Tuning E");
        SERIAL_TXT((char)tolower(axis_codes[axis]));
        SERIAL_EOL();

        probe_points(z_at_pt, 3, true, false);

        LOOP_CAL_ALL(axis) z_at_pt[axis] -= z_at_pt_base[axis];

        Report_results(z_at_pt, true, true);

        mechanics.delta_endstop_adj[axis] += 1.0;

        switch (axis) {
          case A_AXIS :
            h_fac += 4.0 / (Z03(CEN) +Z01(__A)                           +Z32(_CA) +Z32(_AB)); // Offset by X-tower end-stop
            break;
          case B_AXIS :
            h_fac += 4.0 / (Z03(CEN)         +Z01(__B)         +Z32(_BC)           +Z32(_AB)); // Offset by Y-tower end-stop
            break;
          case C_AXIS :
            h_fac += 4.0 / (Z03(CEN)                 +Z01(__C) +Z32(_BC) +Z32(_CA)          ); // Offset by Z-tower end-stop
            break;
        }
      }

      h_fac /= 3.0;
      h_fac *= norm; // Normalize to 1.02 for Kossel mini

      // Tuning Radius
      for (int8_t zig_zag = -1; zig_zag < 2; zig_zag += 2) {
        mechanics.delta_radius += 1.0 * zig_zag;
        mechanics.recalc_delta_settings();

        endstops.enable(true);
        if (!mechanics.Home()) return;
        endstops.not_homing();

        SERIAL_MSG("Tuning R");
        SERIAL_TXT(zig_zag == -1 ? "-" : "+");
        SERIAL_EOL();

        probe_points(z_at_pt, 3, true, false);

        LOOP_CAL_ALL(axis) z_at_pt[axis] -= z_at_pt_base[axis];

        Report_results(z_at_pt, true, true);
        mechanics.delta_radius -= 1.0 * zig_zag;
        mechanics.recalc_delta_settings();
        r_fac -= zig_zag * 6.0 / (Z03(__A) +Z03(__B) +Z03(__C) +Z03(_BC) +Z03(_CA) +Z03(_AB)); // Offset by delta radius
      }

      r_fac /= 2.0;
      r_fac *= 3 * norm; // Normalize to 2.25 for Kossel mini

      // Tuning Tower
      LOOP_XYZ(axis) {

        mechanics.delta_tower_angle_adj[axis] += 1.0;
        mechanics.delta_endstop_adj[(axis + 1) % 3] -= 1.0 / 4.5;
        mechanics.delta_endstop_adj[(axis + 2) % 3] += 1.0 / 4.5;

        float z_temp = MAX3(mechanics.delta_endstop_adj[A_AXIS], mechanics.delta_endstop_adj[B_AXIS], mechanics.delta_endstop_adj[C_AXIS]);
        mechanics.delta_height -= z_temp;
        LOOP_XYZ(i) mechanics.delta_endstop_adj[i] -= z_temp;

        mechanics.recalc_delta_settings();

        endstops.enable(true);
        if (!mechanics.Home()) return;
        endstops.not_homing();

        SERIAL_MSG("Tuning T");
        SERIAL_TXT((char)tolower(axis_codes[axis]));
        SERIAL_EOL();

        probe_points(z_at_pt, 3, true, false);

        LOOP_CAL_ALL(axis) z_at_pt[axis] -= z_at_pt_base[axis];

        Report_results(z_at_pt, true, true);

        mechanics.delta_tower_angle_adj[axis] -= 1.0;
        mechanics.delta_endstop_adj[(axis + 1) % 3] += 1.0 / 4.5;
        mechanics.delta_endstop_adj[(axis + 2) % 3] -= 1.0 / 4.5;

        z_temp = MAX3(mechanics.delta_endstop_adj[A_AXIS], mechanics.delta_endstop_adj[B_AXIS], mechanics.delta_endstop_adj[C_AXIS]);
        mechanics.delta_height -= z_temp;
        LOOP_XYZ(i) mechanics.delta_endstop_adj[i] -= z_temp;

        mechanics.recalc_delta_settings();

        switch (axis) {
          case A_AXIS :
            a_fac += 4.0 / (          Z06(__B) -Z06(__C)           +Z06(_CA) -Z06(_AB)); // Offset by alpha tower angle
            break;
          case B_AXIS :
            a_fac += 4.0 / (-Z06(__A)          +Z06(__C) -Z06(_BC)           +Z06(_AB)); // Offset by beta tower angle
            break;
          case C_AXIS :
            a_fac += 4.0 / (Z06(__A) -Z06(__B)           +Z06(_BC) -Z06(_CA)          ); // Offset by gamma tower angle
            break;
        }
      }

      a_fac /= 3.0;
      a_fac *= norm; // Normalize to 0.83 for Kossel mini

      endstops.enable(true);
      if (!mechanics.Home()) return;
      endstops.not_homing();

      Report_signed_float(PSTR( "H_FACTOR"), h_fac);
      Report_signed_float(PSTR(" R_FACTOR"), r_fac);
      Report_signed_float(PSTR(" A_FACTOR"), a_fac);
      SERIAL_EOL();
      SERIAL_MSG("Copy these values to Configuration_delta.h");
      SERIAL_EOL();
    }

  #endif // DISABLED(PROBE_MANUALLY)

  /**
   * Delta AutoCalibration Algorithm based on Thinkyhead Marlin
   *       Calibrate height, endstops, delta radius, and tower angles.
   *
   * Parameters:
   *
   *   Pn Number of probe points:
   *
   *      P0     No probe. Normalize only.
   *      P1     Probe center and set height only.
   *      P2     Probe center and towers. Set height, endstops, and delta radius.
   *      P3     Probe all positions: center, towers and opposite towers. Set all.
   *      P4-P7  Probe all positions at different locations and average them.
   *
   *   T0  Don't calibrate tower angle corrections
   *
   *   Cn.nn Calibration precision; when omitted calibrates to maximum precision
   *
   *   Fn  Force to run at least n iterations and takes the best result
   *
   *   A   Auto tune calibartion factors (set in Configuration_delta.h)
   *
   *   Vn Verbose level:
   *
   *      V0  Dry-run mode. Report settings and probe results. No calibration.
   *      V1  Report settings
   *      V2  Report settings and probe results
   *
   *   E   Engage the probe for each point
   */
  inline void gcode_G33(void) {

    const int8_t num_points = parser.intval('P', DELTA_AUTO_CALIBRATION_2_DEFAULT_POINTS);
    if (!WITHIN(num_points, 0, 7)) {
      SERIAL_EM("?(P)oints is implausible (0-7).");
      return;
    }

    const int8_t verbose_level = parser.byteval('V', 1);
    if (!WITHIN(verbose_level, 0, 2)) {
      SERIAL_EM("?(V)erbose Level is implausible (0-2).");
      return;
    }

    const float calibration_precision = parser.floatval('C');
    if (calibration_precision < 0) {
      SERIAL_EM("?(C)alibration precision is implausible (>0).");
      return;
    }

    const int8_t force_iterations = parser.intval('F', 0);
    if (!WITHIN(force_iterations, 0, 30)) {
      SERIAL_EM("?(F)orce iteration is implausible (0-30).");
      return;
    }

    const bool  towers_set            = !parser.boolval('T'),
                auto_tune             = parser.boolval('A'),
                stow_after_each       = parser.boolval('E'),
                _0p_calibration       = num_points == 0,
                _1p_calibration       = num_points == 1,
                _4p_calibration       = num_points == 2,
                _7p_9_centre          = num_points >= 8,
                _tower_results        = (_4p_calibration && towers_set)
                                        || num_points >= 3 || num_points == 0,
                _opposite_results     = (_4p_calibration && !towers_set)
                                        || num_points >= 3 || num_points == 0,
                _endstop_results      = num_points != 1,
                _angle_results        = (num_points >= 3 || num_points == 0) && towers_set;

    const static char save_message[] PROGMEM = "Save with M500 and/or copy to configuration_delta.h";

    int8_t iterations = 0;
    float test_precision,
          zero_std_dev = (verbose_level ? 999.0 : 0.0), // 0.0 in dry-run mode : forced end
          zero_std_dev_min = zero_std_dev,
          e_old[ABC] = {
            mechanics.delta_endstop_adj[A_AXIS],
            mechanics.delta_endstop_adj[B_AXIS],
            mechanics.delta_endstop_adj[C_AXIS]
          },
          dr_old = mechanics.delta_radius,
          dh_old = mechanics.delta_height,
          ta_old[ABC] = {
            mechanics.delta_tower_angle_adj[A_AXIS],
            mechanics.delta_tower_angle_adj[B_AXIS],
            mechanics.delta_tower_angle_adj[C_AXIS]
          };

    if (!_1p_calibration && !_0p_calibration) {  // test if the outer radius is reachable
      LOOP_CAL_RAD(axis) {
        const float a = RADIANS(210 + (360 / NPP) * (axis - 1)),
                    r = mechanics.delta_probe_radius * (1 + (_7p_9_centre ? 0.1 : 0.0));
        if (!mechanics.position_is_reachable_by_probe(COS(a) * r + probe.offset[X_AXIS], SIN(a) * r + probe.offset[Y_AXIS])) {
          SERIAL_EM("?(M666 P)robe radius is implausible.");
          return;
        }
      }
    }

    SERIAL_EM("G33 Auto Calibrate");

    stepper.synchronize();

    #if HAS_LEVELING
      bedlevel.reset(); // After calibration bed-level data is no longer valid
    #endif

    #if HOTENDS > 1
      const uint8_t old_tool_index = tools.active_extruder;
      tools.change(0, 0, true);
      #define CALIBRATION_CLEANUP() Calibration_cleanup(old_tool_index)
    #else
      #define CALIBRATION_CLEANUP() Calibration_cleanup()
    #endif

    printer.setup_for_endstop_or_probe_move();
    endstops.enable(true);
    if (!_0p_calibration) {
      if (!mechanics.Home()) return;
      endstops.not_homing();
      DEPLOY_PROBE();
    }

    if (auto_tune) {
      #if ENABLED(PROBE_MANUALLY)
        SERIAL_EM("A probe is needed for auto-tune");
      #else
        tune();
      #endif
      CALIBRATION_CLEANUP();
      return;
    }

    // Report settings

    SERIAL_MSG(MSG_DELTA_CHECKING);
    if (verbose_level == 0) SERIAL_MSG(" (DRY-RUN)");
    SERIAL_EOL();
    LCD_MESSAGEPGM(MSG_DELTA_CHECKING);

    Report_settings(_endstop_results, _angle_results);

    do {

      float z_at_pt[NPP + 1] = { 0.0 };

      test_precision = zero_std_dev;

      iterations++;

      // Probe the points

      zero_std_dev = probe_points(z_at_pt, num_points, towers_set, stow_after_each);

      // Solve matrices

      if ((zero_std_dev < test_precision || iterations <= force_iterations) && zero_std_dev > calibration_precision) {
        if (zero_std_dev < zero_std_dev_min) {
          COPY_ARRAY(e_old, mechanics.delta_endstop_adj);
          dr_old = mechanics.delta_radius;
          dh_old = mechanics.delta_height;
          COPY_ARRAY(ta_old, mechanics.delta_tower_angle_adj);
        }

        float e_delta[ABC] = { 0.0 }, r_delta = 0.0, t_delta[ABC] = { 0.0 };
        const float r_diff = mechanics.delta_radius - mechanics.delta_probe_radius,
                    h_factor = 1 / 6.0 *
                    #if ENABLED(H_FACTOR)
                      (H_FACTOR),
                    #else
                      (1.00 + r_diff * 0.001),                        // 1.02 for r_diff = 20mm
                    #endif
                    r_factor = 1 / 6.0 *
                    #if ENABLED(R_FACTOR)
                      -(R_FACTOR),
                    #else
                      -(1.75 + 0.005 * r_diff + 0.001 * sq(r_diff)),  // 2.25 for r_diff = 20mm
                    #endif
                    a_factor = 1 / 6.0 *
                    #if ENABLED(A_FACTOR)
                      (A_FACTOR);
                    #else
                      (66.66 / mechanics.delta_probe_radius);         // 0.83 for cal_rd = 80mm
                    #endif

        #define ZP(N,I) ((N) * z_at_pt[I])
        #define Z6(I)   ZP(6, I)
        #define Z4(I)   ZP(4, I)
        #define Z2(I)   ZP(2, I)
        #define Z1(I)   ZP(1, I)

        #if ENABLED(PROBE_MANUALLY)
          test_precision = 0.00; // forced end
        #endif

        switch (num_points) {
          case 0:
            test_precision = 0.00; // forced end
            break;

          case 1:
            test_precision = 0.00; // forced end
            LOOP_XYZ(i) e_delta[i] = Z1(0);
            break;

          case 2:
            if (towers_set) {
              e_delta[A_AXIS] = (Z6(CEN) +Z4(__A) -Z2(__B) -Z2(__C)) * h_factor;
              e_delta[B_AXIS] = (Z6(CEN) -Z2(__A) +Z4(__B) -Z2(__C)) * h_factor;
              e_delta[C_AXIS] = (Z6(CEN) -Z2(__A) -Z2(__B) +Z4(__C)) * h_factor;
              r_delta         = (Z6(CEN) -Z2(__A) -Z2(__B) -Z2(__C)) * r_factor;
            }
            else {
              e_delta[A_AXIS] = (Z6(CEN) -Z4(_BC) +Z2(_CA) +Z2(_AB)) * h_factor;
              e_delta[B_AXIS] = (Z6(CEN) +Z2(_BC) -Z4(_CA) +Z2(_AB)) * h_factor;
              e_delta[C_AXIS] = (Z6(CEN) +Z2(_BC) +Z2(_CA) -Z4(_AB)) * h_factor;
              r_delta         = (Z6(CEN) -Z2(_BC) -Z2(_CA) -Z2(_AB)) * r_factor;
            }
            break;

          default:
            e_delta[A_AXIS] = (Z6(CEN) +Z2(__A) -Z1(__B) -Z1(__C) -Z2(_BC) +Z1(_CA) +Z1(_AB)) * h_factor;
            e_delta[B_AXIS] = (Z6(CEN) -Z1(__A) +Z2(__B) -Z1(__C) +Z1(_BC) -Z2(_CA) +Z1(_AB)) * h_factor;
            e_delta[C_AXIS] = (Z6(CEN) -Z1(__A) -Z1(__B) +Z2(__C) +Z1(_BC) +Z1(_CA) -Z2(_AB)) * h_factor;
            r_delta         = (Z6(CEN) -Z1(__A) -Z1(__B) -Z1(__C) -Z1(_BC) -Z1(_CA) -Z1(_AB)) * r_factor;

            if (towers_set) {
              t_delta[A_AXIS] = (         -Z4(__B) +Z4(__C)          -Z4(_CA) +Z4(_AB)) * a_factor;
              t_delta[B_AXIS] = ( Z4(__A)          -Z4(__C) +Z4(_BC)          -Z4(_AB)) * a_factor;
              t_delta[C_AXIS] = (-Z4(__A) +Z4(__B)          -Z4(_BC) +Z4(_CA)         ) * a_factor;
              e_delta[A_AXIS] += (t_delta[B_AXIS] - t_delta[C_AXIS]) / 4.5;
              e_delta[B_AXIS] += (t_delta[C_AXIS] - t_delta[A_AXIS]) / 4.5;
              e_delta[C_AXIS] += (t_delta[A_AXIS] - t_delta[B_AXIS]) / 4.5;
            }
            break;
        }

        LOOP_XYZ(axis) mechanics.delta_endstop_adj[axis] += e_delta[axis];
        mechanics.delta_radius += r_delta;
        LOOP_XYZ(axis) mechanics.delta_tower_angle_adj[axis] += t_delta[axis];
      }
      else if (zero_std_dev >= test_precision) {   // step one back
        COPY_ARRAY(mechanics.delta_endstop_adj, e_old);
        mechanics.delta_radius = dr_old;
        mechanics.delta_height = dh_old;
        COPY_ARRAY(mechanics.delta_tower_angle_adj, ta_old);
      }
      if (verbose_level != 0) {                                    // !dry run
        // Normalise angles to least squares
        if (_angle_results) {
          float a_sum = 0.0;
          LOOP_XYZ(axis) a_sum += mechanics.delta_tower_angle_adj[axis];
          LOOP_XYZ(axis) mechanics.delta_tower_angle_adj[axis] -= a_sum / 3.0;
        }

        // Adjust delta_height and endstops by the max amount
        const float z_temp = MAX3(mechanics.delta_endstop_adj[A_AXIS], mechanics.delta_endstop_adj[B_AXIS], mechanics.delta_endstop_adj[C_AXIS]);
        mechanics.delta_height -= z_temp;
        LOOP_XYZ(i) mechanics.delta_endstop_adj[i] -= z_temp;
      }
      mechanics.recalc_delta_settings();
      NOMORE(zero_std_dev_min, zero_std_dev);

      // Report results
      if (verbose_level != 1)
        Report_results(z_at_pt, _tower_results, _opposite_results);

      if (verbose_level != 0) {                                    // !dry run
        if ((zero_std_dev >= test_precision && iterations > force_iterations) || zero_std_dev <= calibration_precision) {  // end iterations
          SERIAL_MSG("Calibration OK");
          SERIAL_SP(36);
          #if DISABLED(PROBE_MANUALLY)
            if (zero_std_dev >= test_precision && !_1p_calibration)
              SERIAL_MSG("rolling back.");
            else
          #endif
          {
            SERIAL_MV("std dev:", zero_std_dev_min, 3);
          }

          SERIAL_EOL();
          char mess[21];
          strcpy_P(mess, PSTR("Calibration sd:"));
          if (zero_std_dev_min < 1)
            sprintf_P(&mess[15], PSTR("0.%03i"), (int)round(zero_std_dev_min * 1000.0));
          else
            sprintf_P(&mess[15], PSTR("%03i.x"), (int)round(zero_std_dev_min));
          lcd_setstatus(mess);
          Report_settings(_endstop_results, _angle_results);
          SERIAL_PS(save_message);
          SERIAL_EOL();
        }
        else {                                                     // !end iterations
          char mess[15];
          if (iterations < 31)
            sprintf_P(mess, PSTR(".Iteration: %02i"), (int)iterations);
          else
            strcpy_P(mess, PSTR("No convergence"));
          SERIAL_TXT(mess);
          SERIAL_SP(36);
          SERIAL_EMV("std dev:", zero_std_dev, 3);
          lcd_setstatus(mess);
          Report_settings(_endstop_results, _angle_results);
        }
      }
      else {
        const char *enddryrun = PSTR("End DRY-RUN");
        SERIAL_PS(enddryrun);
        SERIAL_SP(35);
        SERIAL_EMV("std dev:", zero_std_dev, 3);

        char mess[21];
        strcpy_P(mess, enddryrun);
        strcpy_P(&mess[11], PSTR(" sd:"));
        if (zero_std_dev < 1)
          sprintf_P(&mess[15], PSTR("0.%03i"), (int)round(zero_std_dev * 1000.0));
        else
          sprintf_P(&mess[15], PSTR("%03i.x"), (int)round(zero_std_dev));
        lcd_setstatus(mess);
      }

      endstops.enable(true);
      if (!mechanics.Home()) return;
      endstops.not_homing();

    } while (((zero_std_dev < test_precision && iterations < 31) || iterations <= force_iterations) && zero_std_dev > calibration_precision);

    CALIBRATION_CLEANUP();
  }

#endif // ENABLED(DELTA_AUTO_CALIBRATION_2)

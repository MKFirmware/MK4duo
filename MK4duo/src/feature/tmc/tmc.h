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
 * tmc_util.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _TMC_H_
#define _TMC_H_

#if HAS_TRINAMIC

  #if ENABLED(HAVE_TMC2130)
    #define TMC TMC2130Stepper
  #elif ENABLED(HAVE_TMC2208)
    #define TMC TMC2208Stepper
  #endif

  static bool report_tmc_status = false;
  const char extended_axis_codes[12][3] = { "X", "X2", "Y", "Y2", "Z", "Z2", "E0", "E1", "E2", "E3", "E4", "E5" };
  enum TMC_AxisEnum {
    TMC_X, TMC_X2, TMC_Y, TMC_Y2, TMC_Z, TMC_Z2,
    TMC_E0, TMC_E1, TMC_E2, TMC_E3, TMC_E4, TMC_E5
  };

  void tmc_get_current(TMC &st, const char name[]);
  void tmc_set_current(TMC &st, const char name[], const int mA);
  void tmc_report_otpw(TMC &st, const char name[]);
  void tmc_clear_otpw(TMC &st, const char name[]);
  void tmc_get_pwmthrs(TMC &st, const char name[], const uint16_t spmm);
  void tmc_set_pwmthrs(TMC &st, const char name[], const int32_t thrs, const uint32_t spmm);
  void tmc_get_sgt(TMC &st, const char name[]);
  void tmc_set_sgt(TMC &st, const char name[], const int8_t sgt_val);

  void monitor_tmc_driver();

  /**
   * TMC2130 specific sensorless homing using stallGuard2.
   * stallGuard2 only works when in spreadCycle mode.
   * spreadCycle and stealthChop are mutually exclusive.
   *
   * Defined here because of limitations with templates and headers.
   */
  #if ENABLED(SENSORLESS_HOMING)
    template<typename TMC>
    void tmc_sensorless_homing(TMC &st, bool enable=true) {
      #if ENABLED(STEALTHCHOP)
        if (enable) {
          st.coolstep_min_speed(1024UL * 1024UL - 1UL);
          st.stealthChop(0);
        }
        else {
          st.coolstep_min_speed(0);
          st.stealthChop(1);
        }
      #endif

      st.diag1_stall(enable ? 1 : 0);
    }
  #endif

#endif // HAS_TRINAMIC

#endif /* _TMC_H_ */

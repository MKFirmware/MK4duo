/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
 * pid.h - pid object
 */

#if ENABLED(PID_ADD_EXTRUSION_RATE)
  static long last_e_position   = 0,
              lpq[LPQ_MAX_LEN]  = { 0 };
  static int  lpq_ptr           = 0;
#endif

struct pid_data_t {

  public: /** Public Parameters */

    float           Kp, Ki, Kd, Kc;
    uint8_t         Max;
    limit_uchar_t   drive;

  private: /** Private Parameters */

    float iState_sum  = 0.0,
          pid_output  = 0.0,
          last_temp   = 0.0;

    short_timer_t next_sample_ms;

  public: /** Public Function */

    void init() { next_sample_ms.start(); }

    void reset() { iState_sum = pid_output = 0.0; }

    float compute(const float target_temp, const float current_temp
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        , const uint8_t tid, const int16_t lpq_len=0
      #endif
    ) {

      if (next_sample_ms.expired(1000UL)) {

        const float pid_error = target_temp - current_temp,
                    dInput    = current_temp - last_temp,
                    dTerm     = Kd * dInput;

        // Compute PID output
        iState_sum += Ki * pid_error;
        iState_sum -= Kp * dInput;
        LIMIT(iState_sum, drive.min, drive.max);
        pid_output = iState_sum - dTerm;

        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          if (tid == toolManager.active_hotend()) {
            const long e_position = stepper.position(E_AXIS);
            if (e_position > last_e_position) {
              lpq[lpq_ptr] = e_position - last_e_position;
              last_e_position = e_position;
            }
            else {
              lpq[lpq_ptr] = 0;
            }
            if (++lpq_ptr >= lpq_len) lpq_ptr = 0;
            pid_output += (lpq[lpq_ptr] * extruders[toolManager.extruder.active]->steps_to_mm) * Kc;
          }
        #endif // PID_ADD_EXTRUSION_RATE

        LIMIT(pid_output, 0, Max);

        last_temp = current_temp;

      }

      return pid_output;
    }

};

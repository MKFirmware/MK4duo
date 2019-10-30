/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
    limit_uchar_t drive;

  private: /** Private Parameters */

    float temp_istate         = 0.0,
          last_temperature    = 0.0,
          temperature_1s      = 0.0;

    limit_float_t temp_istate_limit;

  public: /** Public Function */

    uint8_t spin(const int16_t target_temp, const float current_temp
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        , const uint8_t tid
      #endif
    ) {

      static short_timer_t cycle_1s_timer(true);
      float pid_output = 0.0;

      const float pid_error = float(target_temp) - current_temp;

      if (pid_error > PID_FUNCTIONAL_RANGE) {
        pid_output = Max;
        temp_istate = temp_istate_limit.min;
      }
      else if (pid_error < -(PID_FUNCTIONAL_RANGE) || target_temp <= 20)
        pid_output = 0;
      else {
        pid_output = Kp * pid_error;
        temp_istate = constrain(temp_istate + pid_error, temp_istate_limit.min, temp_istate_limit.max);
        pid_output += Ki * temp_istate * 0.1;
        float dgain = Kd * (last_temperature - temperature_1s);
        pid_output += dgain;

        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          if (tid == tools.active_hotend()) {
            const long e_position = stepper.position(E_AXIS);
            if (e_position > last_e_position) {
              lpq[lpq_ptr] = e_position - last_e_position;
              last_e_position = e_position;
            }
            else {
              lpq[lpq_ptr] = 0;
            }
            if (++lpq_ptr >= tools.data.lpq_len) lpq_ptr = 0;
            pid_output += (lpq[lpq_ptr] * extruders[tools.extruder.active]->steps_to_mm) * Kc;
          }
        #endif // PID_ADD_EXTRUSION_RATE

        LIMIT(pid_output, 0, Max);

      }

      if (cycle_1s_timer.expired(1000U)) {
        last_temperature = temperature_1s;
        temperature_1s = current_temp;
      }

      return pid_output;
    }

    void update() {
      if (Ki != 0) {
        temp_istate_limit.min = (float)drive.min * 10.0f / Ki;
        temp_istate_limit.max = (float)drive.max * 10.0f / Ki;
      }
    }

};

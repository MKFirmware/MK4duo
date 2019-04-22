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
 * pid.h - pid object
 */

#if ENABLED(PID_ADD_EXTRUSION_RATE)
  static long last_e_position   = 0,
              lpq[LPQ_MAX_LEN]  = { 0 };
  static int  lpq_ptr           = 0;
#endif

typedef struct {

  public: /** Public Parameters */

    float   Kp, Ki, Kd, Kc;
    uint8_t DriveMin,
            DriveMax,
            Max;

  private: /** Private Parameters */

    float tempIState          = 0.0,
          tempIStateLimitMin  = 0.0,
          tempIStateLimitMax  = 0.0,
          last_temperature    = 0.0,
          temperature_1s      = 0.0;

  public: /** Public Function */

    uint8_t spin(const int16_t target_temp, const float current_temp
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        , const uint8_t tid
      #endif
    ) {

      static millis_s cycle_1s_ms = 0;
      float pid_output = 0.0;

      const float pid_error = target_temp - current_temp;

      if (pid_error > PID_FUNCTIONAL_RANGE) {
        pid_output = Max;
        tempIState = tempIStateLimitMin;
      }
      else if (pid_error < -(PID_FUNCTIONAL_RANGE) || target_temp == 0)
        pid_output = 0;
      else {
        pid_output = Kp * pid_error;
        tempIState = constrain(tempIState + pid_error, tempIStateLimitMin, tempIStateLimitMax);
        pid_output += Ki * tempIState * 0.1;
        float dgain = Kd * (last_temperature - temperature_1s);
        pid_output += dgain;

        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          if (tid == ACTIVE_HOTEND) {
            const long e_position = stepper.position(E_AXIS);
            if (e_position > last_e_position) {
              lpq[lpq_ptr] = e_position - last_e_position;
              last_e_position = e_position;
            }
            else {
              lpq[lpq_ptr] = 0;
            }
            if (++lpq_ptr >= tools.lpq_len) lpq_ptr = 0;
            pid_output += (lpq[lpq_ptr] * mechanics.steps_to_mm[E_AXIS + tools.active_extruder]) * Kc;
          }
        #endif // PID_ADD_EXTRUSION_RATE

        if (pid_output > Max) {
          if (pid_error > 0) tempIState -= pid_error;
          pid_output = Max;
        }
        else if (pid_output < 0) {
          if (pid_error < 0) tempIState -= pid_error;
          pid_output = 0;
        }
      }

      if (expired(&cycle_1s_ms, 1000U)) {
        last_temperature = temperature_1s;
        temperature_1s = current_temp;
      }

      return pid_output;
    }

    void update() {
      if (Ki != 0) {
        tempIStateLimitMin = (float)DriveMin * 10.0f / Ki;
        tempIStateLimitMax = (float)DriveMax * 10.0f / Ki;
      }
    }

} pid_data_t;

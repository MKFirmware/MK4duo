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
 * pid.h - pid object
 */

#pragma once

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

    uint8_t get_output(const int16_t target_temp, const float current_temp, const millis_t tnow
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        , const uint8_t tid
      #endif
    ) {

      static millis_t cycle_1s = 0;
      uint8_t output = 0;

      const float error = target_temp - current_temp;

      if (error > PID_FUNCTIONAL_RANGE) {
        output = Max;
        tempIState = tempIStateLimitMin;
      }
      else if (error < -(PID_FUNCTIONAL_RANGE) || target_temp == 0)
        output = 0;
      else {
        float pidTerm = Kp * error;
        tempIState = constrain(tempIState + error, tempIStateLimitMin, tempIStateLimitMax);
        pidTerm += Ki * tempIState * 0.1;
        float dgain = Kd * (last_temperature - temperature_1s);
        pidTerm += dgain;

        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          if (tid == ACTIVE_HOTEND) {
            long e_position = stepper.position(E_AXIS);
            if (e_position > last_e_position) {
              lpq[lpq_ptr] = e_position - last_e_position;
              last_e_position = e_position;
            }
            else {
              lpq[lpq_ptr] = 0;
            }
            if (++lpq_ptr >= tools.lpq_len) lpq_ptr = 0;
            pidTerm += (lpq[lpq_ptr] * mechanics.steps_to_mm[E_AXIS + tools.active_extruder]) * Kc;
          }
        #endif // PID_ADD_EXTRUSION_RATE

        output = constrain((int)pidTerm, 0, Max);
      }

      if (ELAPSED(tnow, cycle_1s)) {
        cycle_1s = tnow + 1000UL;
        last_temperature = temperature_1s;
        temperature_1s = current_temp;
      }

      return output;
    }

    void update() {
      if (Ki != 0) {
        tempIStateLimitMin = (float)DriveMin * 10.0f / Ki;
        tempIStateLimitMax = (float)DriveMax * 10.0f / Ki;
      }
    }

} pid_data_t;

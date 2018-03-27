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

#include "../../../MK4duo.h"

#if HAS_POWER_SWITCH || HAS_POWER_CONSUMPTION_SENSOR

  Power powerManager;

  #if HAS_POWER_SWITCH
    millis_t Power::lastPowerOn = 0;
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    int16_t   Power::current_raw_powconsumption = 0;    // Holds measured power consumption
    float     Power::consumption_meas           = 0.0;
    uint32_t  Power::consumption_hour           = 0,
              Power::startpower                 = 0;
  #endif

  #if HAS_POWER_SWITCH

    void Power::spin() {
      static millis_t nextPowerCheck = 0;
      millis_t ms = millis();

      if (ELAPSED(ms, nextPowerCheck)) {
        nextPowerCheck = ms + 2500UL;
        if (is_power_needed()) {
          if (!lastPowerOn) power_on();
        }
        #if (POWER_TIMEOUT > 0)
          else if (ELAPSED(ms, lastPowerOn + (POWER_TIMEOUT) * 1000UL)) {
            if (lastPowerOn) power_off();
          }
        #endif
      }
    }

    void Power::power_on() {
      lastPowerOn = millis();
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
      #if HAS_TRINAMIC
        HAL::delayMilliseconds(100); // Wait for power to settle
        restore_stepper_drivers();
      #endif
      HAL::delayMilliseconds((DELAY_AFTER_POWER_ON) * 1000UL);
    }

    void Power::power_off() {
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      lastPowerOn = 0;
    }

    bool Power::is_power_needed() {

      #if HEATER_COUNT > 0
        LOOP_HEATER() if (heaters[h].target_temperature > 0) return true;
      #endif

      #if FAN_COUNT > 0
        LOOP_FAN() if (fans[f].Speed > 0) return true;
      #endif

      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON
          || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
          #if E_STEPPERS > 1
            || E1_ENABLE_READ == E_ENABLE_ON
            #if HAS_X2_ENABLE
              || X2_ENABLE_READ == X_ENABLE_ON
            #endif
            #if E_STEPPERS > 2
              || E2_ENABLE_READ == E_ENABLE_ON
              #if E_STEPPERS > 3
                || E3_ENABLE_READ == E_ENABLE_ON
                #if E_STEPPERS > 4
                  || E4_ENABLE_READ == E_ENABLE_ON
                  #if E_STEPPERS > 5
                    || E5_ENABLE_READ == E_ENABLE_ON
                  #endif
                #endif
              #endif
            #endif
          #endif
      ) return true;

      return false;
    }

  #endif // HAS_POWER_SWITCH

  #if HAS_POWER_CONSUMPTION_SENSOR

    // Convert raw Power Consumption to watt
    float Power::raw_analog2voltage() {
      return ((HAL_VOLTAGE_PIN) * current_raw_powconsumption) / (AD_RANGE);
    }

    float Power::analog2voltage() {
      float power_zero_raw = (POWER_ZERO * AD_RANGE) / (HAL_VOLTAGE_PIN);
      float rel_raw_power = (current_raw_powconsumption < power_zero_raw) ? (2 * power_zero_raw - current_raw_powconsumption) : current_raw_powconsumption;
      return ((HAL_VOLTAGE_PIN) * rel_raw_power) / (AD_RANGE - POWER_ZERO);
    }

    float Power::analog2current() {
      float temp = analog2voltage() / POWER_SENSITIVITY;
      temp = (((100 - POWER_ERROR) / 100) * temp) - (POWER_OFFSET);
      return temp > 0 ? temp : 0;
    }

    float Power::analog2power() {
      return (analog2current() * POWER_VOLTAGE * 100) / (POWER_EFFICIENCY);
    }

    float Power::analog2error(float current) {
      float temp1 = (analog2voltage() / POWER_SENSITIVITY - POWER_OFFSET) * (POWER_VOLTAGE);
      if (temp1 <= 0) return 0.0;
      float temp2 = current * (POWER_VOLTAGE);
      if (temp2 <= 0) return 0.0;
      return ((temp2 / temp1) - 1) * 100;
    }

    float Power::analog2efficiency(float watt) {
      return (analog2current() * (POWER_VOLTAGE) * 100) / watt;
    }

  #endif // HAS_POWER_CONSUMPTION_SENSOR

#endif // HAS_POWER_SWITCH || HAS_POWER_CONSUMPTION_SENSOR

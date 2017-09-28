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

#include "../../../base.h"

#if HAS_POWER_SWITCH || HAS_POWER_CONSUMPTION_SENSOR

  Power powerManager;

  bool Power::powersupply_on = 
    #if ENABLED(PS_DEFAULT_OFF)
      false
    #else
      true
    #endif
  ;

  #if HAS_POWER_CONSUMPTION_SENSOR
    int16_t       Power::current_raw_powconsumption = 0;  // Holds measured power consumption
    float         Power::power_consumption_meas     = 0.0;
    unsigned long Power::power_consumption_hour     = 0,
                  Power::startpower                 = 0;
  #endif

  void Power::check() {
    static millis_t nextPowerCheck = 0;
    millis_t ms = millis();

    if (ELAPSED(ms, nextPowerCheck)) {
      nextPowerCheck = ms + 2000UL;
      if (is_power_needed()) power_on();
    }
  }

  void Power::power_on() {
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    HAL::delayMilliseconds((DELAY_AFTER_POWER_ON) * 1000L);
    powersupply_on = true;
  }

  void Power::power_off() {
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    powersupply_on = false;
  }

  #if HAS_POWER_CONSUMPTION_SENSOR

    // Convert raw Power Consumption to watt
    float Power::raw_analog2voltage() {
      return ((HAL_VOLTAGE_PIN) * current_raw_powconsumption) / (1023.0);
    }

    float Power::analog2voltage() {
      float power_zero_raw = (POWER_ZERO * 1023.0) / HAL_VOLTAGE_PIN;
      float rel_raw_power = (current_raw_powconsumption < power_zero_raw) ? (2 * power_zero_raw - current_raw_powconsumption) : (current_raw_powconsumption);
      return (((HAL_VOLTAGE_PIN) * rel_raw_power) / (1023.0) - POWER_ZERO;
    }

    float Power::analog2current() {
      float temp = analog2voltage() / POWER_SENSITIVITY;
      temp = (((100 - POWER_ERROR) / 100) * temp) - POWER_OFFSET;
      return temp > 0 ? temp : 0;
    }

    float Power::analog2power() {
      return (analog2current() * POWER_VOLTAGE * 100) /  POWER_EFFICIENCY;
    }

    float Power::analog2error(float current) {
      float temp1 = (analog2voltage() / POWER_SENSITIVITY - POWER_OFFSET) * POWER_VOLTAGE;
      if (temp1 <= 0) return 0.0;
      float temp2 = (current) * POWER_VOLTAGE;
      if (temp2 <= 0) return 0.0;
      return ((temp2/temp1) - 1) * 100;
    }

    float Power::analog2efficiency(float watt) {
      return (analog2current() * POWER_VOLTAGE * 100) / watt;
    }

  #endif

  bool Power::is_power_needed() {

    #if HEATER_COUNT > 0
      LOOP_HEATER() if (heaters[h].target_temperature > 0) return true;
    #endif

    #if FAN_COUNT > 0
      LOOP_FAN() if (fans[f].Speed > 0) return true;
    #endif

    return false;
  }

#endif // POWER_SUPPLY > 0

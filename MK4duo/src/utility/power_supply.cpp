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

#include "../../base.h"

#if HAS_POWER_SWITCH

  bool Power::powersupply_on = 
    #if ENABLED(PS_DEFAULT_OFF)
      false
    #else
      true
    #endif
  ;

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

  bool Power::is_power_needed() {

    #if FAN_COUNT > 0
      for (uint8_t i = 0; i < FAN_COUNT; i++)
        if (fanSpeeds[i] > 0) return true;
    #endif

    #if HAS_AUTO_FAN
      LOOP_HOTEND() if (autoFanSpeeds[h] > 0) return true;
    #endif

    LOOP_HOTEND() if (thermalManager.target_temperature[h] > 0) return true;

    if (thermalManager.target_temperature_bed > 0) return true;

    #if HAS_TEMP_CHAMBER
      if (thermalManager.target_temperature_chamber > 0) return true;
    #endif

    return false;
  }

#endif // POWER_SUPPLY > 0

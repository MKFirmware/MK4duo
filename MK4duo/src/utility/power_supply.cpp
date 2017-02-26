/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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
#include "power_supply.h"

#if HAS(POWER_SWITCH)

  bool Power::powersupply = 
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
    powersupply = true;
  }

  void Power::power_off() {
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    powersupply = false;
  }

  bool Power::is_power_needed() {

    #if FAN_COUNT > 0
      for (uint8_t i = 0; i < FAN_COUNT; i++)
        if (fanSpeeds[i] > 0) return true;
    #endif

    #if HAS(CONTROLLERFAN)
      if (controllerFanSpeed > 0) return true;
    #endif

    #if HAS(AUTO_FAN)
      HOTEND_LOOP() if (autoFanSpeeds[h] > 0) return true;
    #endif

    if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || thermalManager.soft_pwm_bed > 0
      || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
      #if EXTRUDERS > 1
        || E1_ENABLE_READ == E_ENABLE_ON
        #if HAS(X2_ENABLE)
          || X2_ENABLE_READ == X_ENABLE_ON
        #endif
        #if EXTRUDERS > 2
          || E2_ENABLE_READ == E_ENABLE_ON
          #if EXTRUDERS > 3
            || E3_ENABLE_READ == E_ENABLE_ON
            #if EXTRUDERS > 4
              || E4_ENABLE_READ == E_ENABLE_ON
              #if EXTRUDERS > 5
                || E5_ENABLE_READ == E_ENABLE_ON
              #endif
            #endif
          #endif
        #endif
      #endif
    ) return true;

    HOTEND_LOOP() if (thermalManager.target_temperature[h] > 0) return true;

    if (thermalManager.target_temperature_bed > 0) return true;

    #if HAS(TEMP_CHAMBER)
      if (thermalManager.target_temperature_chamber > 0) return true;
    #endif

    return false;
  }

#endif // POWER_SUPPLY > 0
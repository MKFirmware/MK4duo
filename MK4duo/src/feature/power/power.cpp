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

#include "../../../MK4duo.h"
#include "sanitycheck.h"

#if HAS_POWER_SWITCH || HAS_POWER_CONSUMPTION_SENSOR || HAS_POWER_CHECK

Power powerManager;

/** Public Parameters */
#if HAS_POWER_CHECK
  power_data_t Power::data;
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  int16_t   Power::current_raw_powconsumption = 0;    // Holds measured power consumption
  float     Power::consumption_meas           = 0.0;
  uint32_t  Power::startpower                 = 0;
#endif

/** Private Parameters */
#if HAS_POWER_SWITCH
  bool        Power::powersupply_on   = false;
  #if (POWER_TIMEOUT > 0)
    millis_l  Power::last_Power_On_ms = 0;
  #endif
#endif

/** Public Function */
#if HAS_POWER_SWITCH || HAS_POWER_CHECK

  void Power::init() {
    #if HAS_POWER_CHECK
      SET_INPUT(POWER_CHECK_PIN);
    #endif
    #if HAS_POWER_SWITCH
      #if PS_DEFAULT_OFF
        powersupply_on = true; power_off();
      #else
        powersupply_on = false; power_on();
      #endif
    #endif
  }

  #if HAS_POWER_CHECK

    void Power::factory_parameters() {
      setLogic(POWER_CHECK_LOGIC);
      setPullup(PULLUP_POWER_CHECK);
    }

    void Power::setup_pullup() {
      HAL::setInputPullup(POWER_CHECK_PIN, isPullup());
    }

    void Power::report() {
      SERIAL_LOGIC("POWER CHECK Logic", isLogic());
      SERIAL_LOGIC(" Pullup", isPullup());
    }

    void Power::outage() {
      if (IS_SD_PRINTING() && READ(POWER_CHECK_PIN) != isLogic())
        card.setAbortSDprinting(true);
    }

  #endif 

  #if HAS_POWER_SWITCH

    void Power::spin() {
      if (is_power_needed()) power_on();
      #if (POWER_TIMEOUT > 0)
        else if (expired(&last_Power_On_ms, millis_l(POWER_TIMEOUT * 1000UL)))
          power_off();
      #endif
    }

    void Power::power_on() {
      #if (POWER_TIMEOUT > 0)
        last_Power_On_ms = millis();
      #endif
      if (!powersupply_on) {
        OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
        powersupply_on = true;
        #if HAS_TRINAMIC
          HAL::delayMilliseconds(100); // Wait for power to settle
          tmc.restore();
        #endif
        HAL::delayMilliseconds((DELAY_AFTER_POWER_ON) * 1000UL);
      }
    }

    void Power::power_off() {
      if (powersupply_on) {
        OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
        powersupply_on = false;
        #if (POWER_TIMEOUT > 0)
          last_Power_On_ms = 0;
        #endif
      }
    }

  #endif // HAS_POWER_SWITCH

#endif // HAS_POWER_SWITCH || HAS_POWER_CHECK

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

/** Private Function */
#if HAS_POWER_SWITCH

  bool Power::is_power_needed() {

    if (thermalManager.heaters_isActive()) return true;

    if (X_ENABLE_READ() == driver[X_DRV]->isEnable() || Y_ENABLE_READ() == driver[Y_DRV]->isEnable() || Z_ENABLE_READ() == driver[Z_DRV]->isEnable()
        || E0_ENABLE_READ() == driver.e[E0_DRV]->isEnable() // If any of the drivers are enabled...
        #if MAX_DRIVER_E > 1
          || E1_ENABLE_READ() == driver.e[E1_DRV]->isEnable()
          #if MAX_DRIVER_E > 2
            || E2_ENABLE_READ() == driver.e[E2_DRV]->isEnable()
            #if MAX_DRIVER_E > 3
              || E3_ENABLE_READ() == driver.e[E3_DRV]->isEnable()
              #if MAX_DRIVER_E > 4
                || E4_ENABLE_READ() == driver.e[E4_DRV]->isEnable()
                #if MAX_DRIVER_E > 5
                  || E5_ENABLE_READ() == driver.e[E5_DRV]->isEnable()
                #endif
              #endif
            #endif
          #endif
        #endif
    ) return true;

    return false;
  }

#endif // HAS_POWER_SWITCH

#endif // HAS_POWER_SWITCH || HAS_POWER_CONSUMPTION_SENSOR || HAS_POWER_CHECK

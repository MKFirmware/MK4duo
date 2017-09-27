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
 * heater.cpp - heater object
 */

#include "../../base.h"

#if HEATER_COUNT > 0

  Heater heaters[HEATER_COUNT] = {

    #if HAS_HEATER_0
      // Hotend 0
      { IS_HOTEND, HEATER_0_PIN, TEMP_0_PIN, TEMP_SENSOR_0, 0, 0, PID_MIN, PID_MAX, 0, 0,
        HEATER_0_MINTEMP, HEATER_0_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMP, PWM_HARDWARE, INVERTED_HEATER_PINS
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
      },
    #endif

    #if HAS_HEATER_1
      // Hotend 1
      { IS_HOTEND, HEATER_1_PIN, TEMP_1_PIN, TEMP_SENSOR_1, 0, 0, PID_MIN, PID_MAX, 0, 0,
        HEATER_1_MINTEMP, HEATER_1_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMP, PWM_HARDWARE, INVERTED_HEATER_PINS
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
      },
    #endif

    #if HAS_HEATER_2
      // Hotend 2
      { IS_HOTEND, HEATER_2_PIN, TEMP_2_PIN, TEMP_SENSOR_2, 0, 0, PID_MIN, PID_MAX, 0, 0,
        HEATER_2_MINTEMP, HEATER_2_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMP, PWM_HARDWARE, INVERTED_HEATER_PINS
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
      },
    #endif

    #if HAS_HEATER_3
      // Hotend 3
      { IS_HOTEND, HEATER_3_PIN, TEMP_3_PIN, TEMP_SENSOR_3, 0, 0, PID_MIN, PID_MAX, 0, 0,
        HEATER_3_MINTEMP, HEATER_3_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMP, PWM_HARDWARE, INVERTED_HEATER_PINS
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
      },
    #endif

    #if HAS_HEATER_BED
      // BED
      { IS_BED, HEATER_BED_PIN, TEMP_BED_PIN,
        #if HEATER_USES_MAX
          -1,
        #endif
        TEMP_SENSOR_BED, 0, 0, MIN_BED_POWER, MAX_BED_POWER, 0, 0,
        BED_MINTEMP, BED_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMPBED, PWM_HARDWARE, INVERTED_BED_PIN
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
      },
    #endif

    #if HAS_HEATER_CHAMBER
      // CHAMBER
      { IS_CHAMBER, HEATER_CHAMBER_PIN, TEMP_CHAMBER_PIN,
        #if HEATER_USES_MAX
          -1,
        #endif
        TEMP_SENSOR_CHAMBER, 0, 0, MIN_CHAMBER_POWER, MAX_CHAMBER_POWER, 0, 0,
        CHAMBER_MINTEMP, CHAMBER_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMPCHAMBER, PWM_HARDWARE, INVERTED_CHAMBER_PIN
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
      },
    #endif

    #if HAS_HEATER_COOLER
      // COOLER
      { IS_COOLER, HEATER_COOLER_PIN, TEMP_COOLER_PIN,
        #if HEATER_USES_MAX
          -1,
        #endif
        TEMP_SENSOR_COOLER, 0, 0, MIN_COOLER_POWER, MAX_COOLER_POWER, 0, 0,
        COOLER_MINTEMP, COOLER_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMPCOOLER, PWM_HARDWARE, INVERTED_COOLER_PIN
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
      }
    #endif

  };

  /**
   * Initialize Heater
   */
  void Heater::init() {
    if (this->output_pin > -1)
      HAL::pinMode(this->output_pin, OUTPUT);

    #if ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855)
      if (this->sensor_type == -2 || this->sensor_type == -1) {
        OUT_WRITE(SCK_PIN, LOW);
        OUT_WRITE(MOSI_PIN, HIGH);
        SET_INPUT_PULLUP(MISO_PIN);
        OUT_WRITE(SS_PIN, HIGH);

        HAL::pinMode(this->output_pin, OUTPUT);
        HAL::digitalWrite(this->output_pin, HIGH);
      }
    #endif
  }

  void Heater::setTarget(int16_t celsius) {

    NOMORE(celsius, this->maxtemp);
    this->target_temperature = celsius;

    #if WATCH_THE_HEATER
      thermalManager.start_watching(this);
    #endif
  }

  #if PWM_HARDWARE

    void Heater::SetHardwarePwm() {
      uint8_t pwm_val = 0;

      if (this->pwm_hardware) {

        if (this->hardwareInverted)
          pwm_val = 255 - this->soft_pwm;
        else
          pwm_val = this->soft_pwm;

        this->pwm_hardware = HAL::analogWrite(this->output_pin, pwm_val, HEATER_PWM_FREQ);
      }
    }

  #endif

#endif // HEATER_COUNT > 0

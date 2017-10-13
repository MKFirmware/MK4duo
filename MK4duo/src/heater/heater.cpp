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
#include "sensor/thermistor.h"

#if HEATER_COUNT > 0

  Heater heaters[HEATER_COUNT] = {

    #if HAS_HEATER_0
      // Hotend 0
      { IS_HOTEND, HEATER_0_PIN, 0, 0, PID_MIN, PID_MAX, 0,
        HEATER_0_MINTEMP, HEATER_0_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMP, PWM_HARDWARE, INVERTED_HEATER_PINS
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
        , { TEMP_0_PIN, TEMP_SENSOR_0, 2048, 0, 0, HOT0_R25, HOT0_BETA, THERMISTOR_SERIES_RS, 0.0, 0.0, 0.0
          #if HEATER_USES_AD595
            , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
          #endif
        }
      },
    #endif

    #if HAS_HEATER_1
      // Hotend 1
      { IS_HOTEND, HEATER_1_PIN, 0, 0, PID_MIN, PID_MAX, 0,
        HEATER_1_MINTEMP, HEATER_1_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMP, PWM_HARDWARE, INVERTED_HEATER_PINS
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
        , { TEMP_1_PIN, TEMP_SENSOR_1, 2048, 0, 0, HOT1_R25, HOT1_BETA, THERMISTOR_SERIES_RS, 0.0, 0.0, 0.0
          #if HEATER_USES_AD595
            , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
          #endif
        }
      },
    #endif

    #if HAS_HEATER_2
      // Hotend 2
      { IS_HOTEND, HEATER_2_PIN, 0, 0, PID_MIN, PID_MAX, 0,
        HEATER_2_MINTEMP, HEATER_2_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMP, PWM_HARDWARE, INVERTED_HEATER_PINS
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
        , { TEMP_2_PIN, TEMP_SENSOR_2, 2048, 0, 0, HOT2_R25, HOT2_BETA, THERMISTOR_SERIES_RS, 0.0, 0.0, 0.0
          #if HEATER_USES_AD595
            , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
          #endif
        }
      },
    #endif

    #if HAS_HEATER_3
      // Hotend 3
      { IS_HOTEND, HEATER_3_PIN, 0, 0, PID_MIN, PID_MAX, 0,
        HEATER_3_MINTEMP, HEATER_3_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMP, PWM_HARDWARE, INVERTED_HEATER_PINS
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
        , { TEMP_3_PIN, TEMP_SENSOR_3, 2048, 0, 0, HOT3_R25, HOT3_BETA, THERMISTOR_SERIES_RS, 0.0, 0.0, 0.0
          #if HEATER_USES_AD595
            , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
          #endif
        }
      },
    #endif

    #if HAS_HEATER_BED
      // BED
      { IS_BED, HEATER_BED_PIN, 0, 0, MIN_BED_POWER, MAX_BED_POWER, 0,
        BED_MINTEMP, BED_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMPBED, PWM_HARDWARE, INVERTED_BED_PIN
        #if HEATER_USES_AD595
          , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
        #endif
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
        , { TEMP_BED_PIN, TEMP_SENSOR_BED, 2048, 0, 0, BED_R25, BED_BETA, THERMISTOR_SERIES_RS, 0.0, 0.0, 0.0
          #if HEATER_USES_AD595
            , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
          #endif
        }
      },
    #endif

    #if HAS_HEATER_CHAMBER
      // CHAMBER
      { IS_CHAMBER, HEATER_CHAMBER_PIN, 0, 0, MIN_CHAMBER_POWER, MAX_CHAMBER_POWER, 0,
        CHAMBER_MINTEMP, CHAMBER_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMPCHAMBER, PWM_HARDWARE, INVERTED_CHAMBER_PIN
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
        , { TEMP_CHAMBER_PIN, TEMP_SENSOR_CHAMBER, 2048, 0, 0, BED_R25, BED_BETA, THERMISTOR_SERIES_RS, 0.0, 0.0, 0.0
          #if HEATER_USES_AD595
            , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
          #endif
        }
      },
    #endif

    #if HAS_HEATER_COOLER
      // COOLER
      { IS_COOLER, HEATER_COOLER_PIN, 0, 0, MIN_COOLER_POWER, MAX_COOLER_POWER, 0,
        COOLER_MINTEMP, COOLER_MAXTEMP, 25.0, 0.0, 0.0, 0.0, 0.0, PIDTEMPCOOLER, PWM_HARDWARE, INVERTED_COOLER_PIN
        #if WATCH_THE_HEATER
          , 0, 0
        #endif
        , { TEMP_COOLER_PIN, TEMP_SENSOR_COOLER, 2048, 0, 0, BED_R25, BED_BETA, THERMISTOR_SERIES_RS, 0.0, 0.0, 0.0
          #if HEATER_USES_AD595
            , TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN
          #endif
        }
      }
    #endif

  };

  /**
   * Initialize Heater
   */
  void Heater::init() {

    // Reset valor
    this->target_temperature    = 0;
    this->current_temperature   = 25.0;
    this->soft_pwm              = 0;
    this->pwm_pos               = 0;

    #if WATCH_THE_HEATER
      this->watch_target_temp   = 0;
      this->watch_next_ms       = 0;
    #endif

    if (this->output_pin > -1)
      HAL::pinMode(this->output_pin, OUTPUT);

    #if ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855)
      if (this->sensor.type == -2 || this->sensor.type == -1) {
        OUT_WRITE(SCK_PIN, LOW);
        OUT_WRITE(MOSI_PIN, HIGH);
        SET_INPUT_PULLUP(MISO_PIN);
        OUT_WRITE(SS_PIN, HIGH);

        HAL::pinMode(this->output_pin, OUTPUT);
        HAL::digitalWrite(this->output_pin, HIGH);
      }
    #endif

    this->sensor.CalcDerivedParameters();
  }

  void Heater::setTarget(int16_t celsius) {

    NOMORE(celsius, this->maxtemp);
    this->target_temperature = celsius;

    #if WATCH_THE_HEATER
      thermalManager.start_watching(this);
    #endif
  }

  void Heater::print_PID(const uint8_t h/*=0*/) {

    if (this->type == IS_HOTEND)
      SERIAL_SMV(CFG, "  M301 H", (int)h);
    #if (PIDTEMPBED)
      else if (this->type == IS_BED) SERIAL_SM(CFG, "  M301 H-1");
    #endif
    #if (PIDTEMPCHAMBER)
      else if (this->type == IS_CHAMBER) SERIAL_SM(CFG, "  M301 H-2");
    #endif
    #if (PIDTEMPCOOLER)
      else if (this->type == IS_COOLER) SERIAL_SM(CFG, "  M301 H-3");
    #endif
    else return;

    SERIAL_MV(" P", this->Kp);
    SERIAL_MV(" I", this->Ki);
    SERIAL_MV(" D", this->Kd);
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      SERIAL_MV(" C", this->Kc);
    #endif
    SERIAL_EOL();
  }

  void Heater::sensor_print_parameters(const uint8_t h/*=0*/) {

    if (this->type == IS_HOTEND)
      SERIAL_SMV(CFG, "  M305 H", (int)h);
    #if HAS_HEATER_BED
      else if (this->type == IS_BED) SERIAL_SM(CFG, "  M305 H-1");
    #endif
    #if HAS_HEATER_CHAMBER
      else if (this->type == IS_CHAMBER) SERIAL_SM(CFG, "  M305 H-2");
    #endif
    #if HAS_HEATER_COOLER
      else if (this->type == IS_COOLER) SERIAL_SM(CFG, "  M305 H-3");
    #endif
    else return;

    SERIAL_EM(" Sensor");
    SERIAL_LMV(CFG, " Pin: ", this->sensor.pin);
    SERIAL_LMV(CFG, " Thermistor resistance at 25 C:", this->sensor.r25, 1);
    SERIAL_LMV(CFG, " BetaK value: ", this->sensor.beta, 1);
    SERIAL_LMV(CFG, " Steinhart-Hart C coefficien: ", this->sensor.shC, 1);
    SERIAL_LMV(CFG, " Pullup resistor value: ", this->sensor.pullupR, 1);
    SERIAL_LMV(CFG, " ADC low offset correction: ", this->sensor.adcLowOffset);
    SERIAL_LMV(CFG, " ADC high offset correction: ", this->sensor.adcHighOffset);

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

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

#include "../../MK4duo.h"
#include "sensor/thermistor.h"

#if HEATER_COUNT > 0

  Heater heaters[HEATER_COUNT];

  /**
   * Initialize Heater
   */
  void Heater::init() {

    // Reset valor
    soft_pwm              = 0;
    pwm_pos               = 0;
    target_temperature    = 0;
    current_temperature   = 25.0;
    sensor.raw            = 0;
    sensor.adcLowOffset   = 0;
    sensor.adcHighOffset  = 0;
    sensor.shC            = 0.0;

    #if WATCH_THE_HEATER
      watch_target_temp   = 0;
      watch_next_ms       = 0;
    #endif

    #if ENABLED(__AVR__)
      if (pin > NoPin) HAL::pinMode(pin, OUTPUT);
    #endif

    #if ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855)
      if (sensor.type == -2 || sensor.type == -1) {
        OUT_WRITE(SCK_PIN, LOW);
        OUT_WRITE(MOSI_PIN, HIGH);
        SET_INPUT_PULLUP(MISO_PIN);
        OUT_WRITE(SS_PIN, HIGH);
      }
    #endif

    sensor.CalcDerivedParameters();
  }

  void Heater::setTarget(int16_t celsius) {

    NOMORE(celsius, maxtemp);
    target_temperature = celsius;

    #if WATCH_THE_HEATER
      thermalManager.start_watching(this);
    #endif
  }

  void Heater::print_PID(const uint8_t h/*=0*/) {

    if (type == IS_HOTEND)
      SERIAL_SMV(CFG, "  M301 H", (int)h);
    #if (PIDTEMPBED)
      else if (type == IS_BED) SERIAL_SM(CFG, "  M301 H-1");
    #endif
    #if (PIDTEMPCHAMBER)
      else if (type == IS_CHAMBER) SERIAL_SM(CFG, "  M301 H-2");
    #endif
    #if (PIDTEMPCOOLER)
      else if (type == IS_COOLER) SERIAL_SM(CFG, "  M301 H-3");
    #endif
    else return;

    SERIAL_MV(" P", Kp);
    SERIAL_MV(" I", Ki);
    SERIAL_MV(" D", Kd);
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      SERIAL_MV(" C", Kc);
    #endif
    SERIAL_EOL();
  }

  void Heater::sensor_print_parameters(const uint8_t h/*=0*/) {

    if (type == IS_HOTEND)
      SERIAL_SMV(CFG, "  M305 H", (int)h);
    #if HAS_HEATER_BED
      else if (type == IS_BED) SERIAL_SM(CFG, "  M305 H-1");
    #endif
    #if HAS_HEATER_CHAMBER
      else if (type == IS_CHAMBER) SERIAL_SM(CFG, "  M305 H-2");
    #endif
    #if HAS_HEATER_COOLER
      else if (type == IS_COOLER) SERIAL_SM(CFG, "  M305 H-3");
    #endif
    else return;

    SERIAL_EM(" Sensor");
    SERIAL_LMV(CFG, " Pin: ", sensor.pin);
    SERIAL_LMV(CFG, " Thermistor resistance at 25 C:", sensor.r25, 1);
    SERIAL_LMV(CFG, " BetaK value: ", sensor.beta, 1);
    SERIAL_LMV(CFG, " Steinhart-Hart C coefficien: ", sensor.shC, 1);
    SERIAL_LMV(CFG, " Pullup resistor value: ", sensor.pullupR, 1);
    SERIAL_LMV(CFG, " ADC low offset correction: ", sensor.adcLowOffset);
    SERIAL_LMV(CFG, " ADC high offset correction: ", sensor.adcHighOffset);

  }

  #if HARDWARE_PWM
    void Heater::SetHardwarePwm() {
      uint8_t pwm_val = 0;

      if (hardwareInverted)
        pwm_val = 255 - soft_pwm;
      else
        pwm_val = soft_pwm;

      HAL::analogWrite(pin, pwm_val, (type == IS_HOTEND) ? 250 : 10);
    }
  #endif

#endif // HEATER_COUNT > 0

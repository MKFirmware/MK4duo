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
    this->soft_pwm              = 0;
    this->pwm_pos               = 0;
    this->target_temperature    = 0;
    this->current_temperature   = 25.0;
    this->sensor.raw            = 0;
    this->sensor.adcLowOffset   = 0;
    this->sensor.adcHighOffset  = 0;
    this->sensor.shC            = 0.0;

    #if WATCH_THE_HEATER
      this->watch_target_temp   = 0;
      this->watch_next_ms       = 0;
    #endif

    if (this->output_pin > NoPin)
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

  #if HARDWARE_PWM
    void Heater::SetHardwarePwm() {
      uint8_t pwm_val = 0;

      if (this->hardwareInverted)
        pwm_val = 255 - this->soft_pwm;
      else
        pwm_val = this->soft_pwm;

      HAL::analogWrite(this->output_pin, pwm_val, (this->type == IS_HOTEND) ? 250 : 10);
    }
  #endif

#endif // HEATER_COUNT > 0

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

#include "../../../MK4duo.h"
#include "sensor/thermistor.h"

#if HEATER_COUNT > 0

  Heater heaters[HEATER_COUNT];

  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    static long last_e_position   = 0,
                lpq[LPQ_MAX_LEN]  = { 0 };
    static int  lpq_ptr           = 0;
  #endif

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
    last_temperature      = 0.0;
    temperature_1s        = 0.0;

    #if WATCH_THE_HEATER
      watch_target_temp   = 0;
      watch_next_ms       = 0;
    #endif

    setIdle(false);

    sensor.CalcDerivedParameters();

    if (printer.isRunning()) return; // All running not reinitialize

    if (pin > 0) HAL::pinMode(pin, (isHWInverted()) ? OUTPUT_HIGH : OUTPUT_LOW);

    #if ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855)
      if (sensor.type == -2 || sensor.type == -1) {
        HAL::pinMode(sensor.pin, OUTPUT_HIGH);
      }
    #endif

  }

  void Heater::setTarget(int16_t celsius) {

    NOMORE(celsius, maxtemp);
    target_temperature = celsius;

    #if WATCH_THE_HEATER
      start_watching();
    #endif
  }

  void Heater::updatePID() {
    if (isUsePid() && Ki != 0) {
      tempIStateLimitMin = (float)pidDriveMin * 10.0f / Ki;
      tempIStateLimitMax = (float)pidDriveMax * 10.0f / Ki;
    }
  }

  void Heater::get_pid_output(const bool cycle_1s) {

    const float difference = (float)target_temperature - current_temperature;

    millis_t ms = millis();

    #if HEATER_IDLE_HANDLER
      if (!isIdle() && idle_timeout_ms && ELAPSED(ms, idle_timeout_ms))
        setIdle(true);
    #endif

    if (isOFF() || isIdle())
      soft_pwm = 0;
    else if (isUsePid()) {
      if (!isTuning()) {
        SERIAL_LM(ER, " Need Tuning PID");
        LCD_ALERTMESSAGEPGM(MSG_NEED_TUNE_PID);
        setTarget(0);
      }
      else if (difference > PID_FUNCTIONAL_RANGE) {
        soft_pwm = pidMax;
        tempIState = tempIStateLimitMin;
      }
      else if (difference < -(PID_FUNCTIONAL_RANGE) || target_temperature == 0)
        soft_pwm = 0;
      else {
        float pidTerm = Kp * difference;
        tempIState = constrain(tempIState + difference, tempIStateLimitMin, tempIStateLimitMax);
        pidTerm += Ki * tempIState * 0.1; // 0.1 = 10Hz
        float dgain = Kd * (last_temperature - temperature_1s);
        pidTerm += dgain;

        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          if (ID == EXTRUDER_IDX) {
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

        soft_pwm = constrain((int)pidTerm, 0, PID_MAX);
      }

      if (cycle_1s) {
        last_temperature = temperature_1s;
        temperature_1s = current_temperature;
      }
    }
    else if (ELAPSED(ms, next_check_ms)) {
      next_check_ms = ms + temp_check_interval[type];
      if (current_temperature <= target_temperature - temp_hysteresis[type])
        soft_pwm = pidMax;
      else if (current_temperature >= target_temperature + temp_hysteresis[type])
        soft_pwm = 0;
    }

    #if ENABLED(PID_DEBUG)
      SERIAL_SMV(ECHO, MSG_PID_DEBUG, HOTEND_INDEX);
      SERIAL_MV(MSG_PID_DEBUG_INPUT, current_temperature);
      SERIAL_EMV(MSG_PID_DEBUG_OUTPUT, soft_pwm);
    #endif // PID_DEBUG

  }

  void Heater::print_PID() {

    if (isUsePid()) {
      if (type == IS_HOTEND) SERIAL_SMV(CFG, "  M301 H", (int)ID);
      else if (type == IS_BED) SERIAL_SM(CFG, "  M301 H-1");
      else if (type == IS_CHAMBER) SERIAL_SM(CFG, "  M301 H-2");
      else if (type == IS_COOLER) SERIAL_SM(CFG, "  M301 H-3");
      else return;

      SERIAL_MV(" P", Kp);
      SERIAL_MV(" I", Ki);
      SERIAL_MV(" D", Kd);
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        SERIAL_MV(" C", Kc);
      #endif
      SERIAL_EOL();
    }
  }

  void Heater::print_parameters() {

    if (type == IS_HOTEND)
      SERIAL_SMV(CFG, "  M306 H", (int)ID);
    #if HAS_HEATER_BED
      else if (type == IS_BED) SERIAL_SM(CFG, "  M306 H-1");
    #endif
    #if HAS_HEATER_CHAMBER
      else if (type == IS_CHAMBER) SERIAL_SM(CFG, "  M306 H-2");
    #endif
    #if HAS_HEATER_COOLER
      else if (type == IS_COOLER) SERIAL_SM(CFG, "  M306 H-3");
    #endif
    else return;

    SERIAL_EM(" Heater");
    SERIAL_LMV(CFG, " Pin:", pin);
    SERIAL_LMV(CFG, " Min temp:", (int)mintemp);
    SERIAL_LMV(CFG, " Max temp:", (int)maxtemp);
    SERIAL_LMT(CFG, " Use PID:", isUsePid() ? "On" : "Off");
    if (isUsePid()) {
      SERIAL_LMV(CFG, " PID drive min:", (int)pidDriveMin);
      SERIAL_LMV(CFG, " PID drive max:", (int)pidDriveMax);
      SERIAL_LMV(CFG, " PID max:", (int)pidMax);
      if (!isTuning())
        SERIAL_LM(CFG, " NOT TUNING PID");
    }
    SERIAL_LMT(CFG, " Hardware inverted:", isHWInverted() ? "On" : "Off");

  }

  void Heater::sensor_print_parameters() {

    if (type == IS_HOTEND)
      SERIAL_SMV(CFG, "  M305 H", (int)ID);
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
    SERIAL_LMV(CFG, " Pin:", sensor.pin);
    SERIAL_LMV(CFG, " Thermistor resistance at 25 C:", sensor.r25, 1);
    SERIAL_LMV(CFG, " BetaK value:", sensor.beta, 1);
    SERIAL_LMV(CFG, " Steinhart-Hart A coefficien:", sensor.shA, 10);
    SERIAL_LMV(CFG, " Steinhart-Hart B coefficien:", sensor.shB, 10);
    SERIAL_LMV(CFG, " Steinhart-Hart C coefficien:", sensor.shC, 10);
    SERIAL_LMV(CFG, " Pullup resistor value:", sensor.pullupR, 1);
    SERIAL_LMV(CFG, " ADC low offset correction:", sensor.adcLowOffset);
    SERIAL_LMV(CFG, " ADC high offset correction:", sensor.adcHighOffset);

  }

  #if HARDWARE_PWM
    void Heater::SetHardwarePwm() {
      uint8_t pwm_val = 0;

      if (isHWInverted())
        pwm_val = 255 - soft_pwm;
      else
        pwm_val = soft_pwm;

      HAL::analogWrite(pin, pwm_val, (type == IS_HOTEND) ? 250 : 10);
    }
  #endif

  #if WATCH_THE_HEATER
    /**
     * Start Heating Sanity Check for heaters that are below
     * their target temperature by a configurable margin.
     * This is called when the temperature is set.
     */
    void Heater::start_watching() {
      if (isON() && current_temperature < target_temperature - (WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
        watch_target_temp = current_temperature + WATCH_TEMP_INCREASE;
        watch_next_ms = millis() + (WATCH_TEMP_PERIOD) * 1000UL;
      }
      else
        watch_next_ms = 0;
    }
  #endif

  #if HEATER_IDLE_HANDLER
    void Heater::start_idle_timer(const millis_t timeout_ms) {
      idle_timeout_ms = millis() + timeout_ms;
      setIdle(false);
    }

    void Heater::reset_idle_timer() {
      idle_timeout_ms = 0;
      setIdle(false);
      #if WATCH_THE_HOTEND
        if (type == IS_HOTEND) start_watching();
      #endif
      #if WATCH_THE_BED
        if (type == IS_BED) start_watching();
      #endif
    }
  #endif

#endif // HEATER_COUNT > 0

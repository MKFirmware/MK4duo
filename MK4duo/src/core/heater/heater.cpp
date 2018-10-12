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
#endif

/**
 * Initialize Heater
 */
void Heater::init() {

  // Reset valor
  soft_pwm              = 0;
  pwm_pos               = 0;
  target_temperature    = 0;
  idle_temperature      = 0;
  current_temperature   = 25.0;
  sensor.raw            = 0;

  setActive(false);
  setIdle(false);

  #if WATCH_THE_HEATER
    watch_target_temp   = 0;
    watch_next_ms       = 0;
  #endif

  sensor.CalcDerivedParameters();

  if (printer.isRunning()) return; // All running not reinitialize

  if (data.pin > 0) HAL::pinMode(data.pin, (isHWInverted()) ? OUTPUT_HIGH : OUTPUT_LOW);

  #if ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855)
    if (sensor.type == -2 || sensor.type == -1) {
      HAL::pinMode(sensor.pin, OUTPUT_HIGH);
    }
  #endif

}

void Heater::setTarget(int16_t celsius) {

  NOMORE(celsius, data.maxtemp);

  if (!isTuning() && isUsePid()) {
    SERIAL_LM(ER, " Need Tuning PID");
    LCD_ALERTMESSAGEPGM(MSG_NEED_TUNE_PID);
  }
  else if (isFault())
    SERIAL_LM(ER, " Heater not switched on to temperature fault.");
  else if (celsius == 0)
    SwitchOff();
  else {
    setActive(true);
    if (isActive()) {
      target_temperature = celsius;
      #if WATCH_THE_HEATER
        start_watching();
      #endif
    }
  }
}

void Heater::get_output() {

  millis_t now = millis();

  if (!isIdle() && idle_timeout_ms && ELAPSED(now, idle_timeout_ms)) setIdle(true);

  if (isActive()) {

    // Get the target temperature and the error
    const float targetTemperature = isIdle() ? idle_temperature : target_temperature;

    if (isUsePid()) {
      soft_pwm = pid.get_output(targetTemperature, current_temperature, now
        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          , data.ID
        #endif
      );
    }
    else if (ELAPSED(now, next_check_ms)) {
      next_check_ms = now + temp_check_interval[data.type];
      if (current_temperature <= targetTemperature - temp_hysteresis[data.type])
        soft_pwm = pid.Max;
      else if (current_temperature >= targetTemperature + temp_hysteresis[data.type])
        soft_pwm = 0;
    }

    #if ENABLED(PID_DEBUG)
      SERIAL_SMV(ECHO, MSG_PID_DEBUG, ACTIVE_HOTEND);
      SERIAL_MV(MSG_PID_DEBUG_INPUT, current_temperature);
      SERIAL_EMV(MSG_PID_DEBUG_OUTPUT, soft_pwm);
    #endif // PID_DEBUG
  }
}

void Heater::print_sensor_parameters() {
  const int8_t heater_id = data.type == 0 ? data.ID : -data.type;
  SERIAL_LM(CFG, "Heater Sensor parameters: H<Heater> P<Pin> T<Type> A<R25> B<BetaK> C<Steinhart-Hart C> R<Pullup> L<ADC low offset> O<ADC high offset>:");
  SERIAL_SMV(CFG, "  M305 H", (int)heater_id);
  SERIAL_MV(" P", sensor.pin);
  SERIAL_MV(" T", sensor.type);
  if (WITHIN(sensor.type, 1, 9)) {
    SERIAL_MV(" A", sensor.r25, 1);
    SERIAL_MV(" B", sensor.beta, 1);
    SERIAL_MV(" C", sensor.shC, 10);
    SERIAL_MV(" R", sensor.pullupR, 1);
    SERIAL_MV(" L", sensor.adcLowOffset);
    SERIAL_MV(" O", sensor.adcHighOffset);
  }
  SERIAL_EOL();
}

void Heater::print_heater_parameters() {
  const int8_t heater_id = data.type == IS_HOTEND ? data.ID : -data.type;
  SERIAL_LM(CFG, "Heater parameters: H<Heater> P<Pin> A<Pid Drive Min> B<Pid Drive Max> C<Pid Max> L<Min Temp> O<Max Temp> U<Use Pid 0-1> I<Hardware Inverted 0-1>:");
  SERIAL_SMV(CFG, "  M306 H", (int)heater_id);
  SERIAL_MV(" P", data.pin);
  SERIAL_MV(" A", pid.DriveMin);
  SERIAL_MV(" B", pid.DriveMax);
  SERIAL_MV(" C", pid.Max);
  SERIAL_MV(" L", data.mintemp);
  SERIAL_MV(" O", data.maxtemp);
  SERIAL_MV(" U", isUsePid());
  SERIAL_MV(" I", isHWInverted());
  SERIAL_EOL();
}

void Heater::print_PID_parameters() {
  const uint8_t heater_id = data.type == IS_HOTEND ? data.ID : -1;
  if (isUsePid()) {
    SERIAL_SM(CFG, "Heater PID parameters: H<Heater> P<Proportional> I<Integral> D<Derivative>");
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      if (data.type == IS_HOTEND) SERIAL_MSG(" C<Kc term> L<LPQ length>");
    #endif
    SERIAL_CHR(':');
    SERIAL_EOL();
    SERIAL_SMV(CFG, "  M301 H", heater_id);
    SERIAL_MV(" P", pid.Kp);
    SERIAL_MV(" I", pid.Ki);
    SERIAL_MV(" D", pid.Kd);
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      if (data.type == IS_HOTEND) {
        SERIAL_MV(" C", pid.Kc);
        SERIAL_MV(" L", (int)tools.lpq_len);
      }
    #endif
    SERIAL_EOL();
  }
}

#if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
  void Heater::print_AD595_parameters() {
    SERIAL_LM(CFG, "AD595 or AD8495 parameters: H<Hotend> O<Offset> S<Gain>:");
    SERIAL_SMV(CFG, "  M595 H", (int)data.ID);
    SERIAL_MV(" O", sensor.ad595_offset);
    SERIAL_MV(" S", sensor.ad595_gain);
    SERIAL_EOL();
  }
#endif

void Heater::start_idle_timer(const millis_t timeout_ms) {
  idle_timeout_ms = millis() + timeout_ms;
  setIdle(false);
}

void Heater::reset_idle_timer() {
  idle_timeout_ms = 0;
  setIdle(false);
  #if WATCH_THE_HOTEND
    if (data.type == IS_HOTEND) start_watching();
  #endif
  #if WATCH_THE_BED
    if (data.type == IS_BED) start_watching();
  #endif
}

#if HARDWARE_PWM
  void Heater::SetHardwarePwm() {
    uint8_t pwm_val = 0;

    if (isHWInverted())
      pwm_val = 255 - soft_pwm;
    else
      pwm_val = soft_pwm;

    HAL::analogWrite(data.pin, pwm_val, (data.type == IS_HOTEND) ? 250 : 10);
  }
#endif

#if WATCH_THE_HEATER
  /**
   * Start Heating Sanity Check for heaters that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set.
   */
  void Heater::start_watching() {
    const float targetTemperature = isIdle() ? idle_temperature : target_temperature;
    if (isActive() && current_temperature < targetTemperature - (WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
      watch_target_temp = current_temperature + WATCH_TEMP_INCREASE;
      watch_next_ms = millis() + (WATCH_TEMP_PERIOD) * 1000UL;
    }
    else
      watch_next_ms = 0;
  }
#endif

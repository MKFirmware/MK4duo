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

void Heater::init() {

  // Reset valor
  pwm_value             = 0;
  target_temperature    = 0;
  idle_temperature      = 0;
  current_temperature   = 25.0;
  sensor.raw            = 0;

  setActive(false);
  setIdle(false);

  watch_target_temp     = 0;
  watch_next_ms         = 0;
  thermal_runaway_timer = 0;

  thermal_runaway_state = TRInactive;

  sensor.CalcDerivedParameters();

  if (printer.isRunning()) return; // All running not reinitialize

  if (data.pin > 0) HAL::pinMode(data.pin, (isHWInverted()) ? OUTPUT_HIGH : OUTPUT_LOW);

  #if ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855)
    if (sensor.type == -2 || sensor.type == -1) {
      HAL::pinMode(sensor.pin, OUTPUT_HIGH);
    }
  #endif

}

void Heater::setTarget(const int16_t celsius) {

  if (celsius == 0)
    SwitchOff();
  else if (!isTuning() && isUsePid()) {
    SERIAL_LM(ER, " Need Tuning PID");
    LCD_MESSAGEPGM(MSG_NEED_TUNE_PID);
  }
  else if (isFault())
    SERIAL_LM(ER, " Heater not switched on to temperature fault.");
  else {
    setActive(true);
    if (isActive()) {
      target_temperature = MIN(celsius, data.maxtemp - 15);
      thermal_runaway_state = target_temperature > 0 ? TRFirstHeating : TRInactive;
      start_watching();
    }
  }
}

void Heater::waitForTarget(bool no_wait_for_cooling/*=true*/) {

  #if TEMP_RESIDENCY_TIME > 0
    millis_t residency_start_ms = 0;
    // Loop until the temperature has stabilized
    #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
  #else
    #define TEMP_CONDITIONS (wants_to_cool ? isCooling() : isHeating())
  #endif

  float     old_temp            = 9999.0;
  bool      wants_to_cool       = false;
  millis_t  now,
            next_cool_check_ms  = 0;

  const bool oldReport = printer.isAutoreportTemp();
  
  printer.setWaitForHeatUp(true);
  printer.setAutoreportTemp(true);

  #if ENABLED(PRINTER_EVENT_LEDS)
    const float start_temp = current_temperature;
    const bool isHotend = data.type == IS_HOTEND;
    ledevents.onHeatingStart(isHotend);
  #endif

  do {

    wants_to_cool = isCooling();
    // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
    if (no_wait_for_cooling && wants_to_cool) break;

    now = millis();
    printer.idle();
    printer.keepalive(WaitHeater);
    printer.move_watch.start(); // Keep steppers powered

    const float temp = current_temperature;

    #if ENABLED(PRINTER_EVENT_LEDS)
      if (!wants_to_cool) ledevents.onHeating(isHotend, start_temp, temp, target_temperature);
    #endif

    #if TEMP_RESIDENCY_TIME > 0

      const float temp_diff = ABS(target_temperature - temp);

      if (!residency_start_ms) {
        // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
        if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
      }
      else if (temp_diff > TEMP_HYSTERESIS) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = now;
      }

    #endif

    // Prevent a wait-forever situation if R is misused i.e. M190 R0
    if (wants_to_cool) {
      // Break after 60 seconds
      // if the temperature did not drop at least 1.5
      if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
        if (old_temp - temp < 1.5) break;
        next_cool_check_ms = now + 60000UL;
        old_temp = temp;
      }
    }

  } while (printer.isWaitForHeatUp() && TEMP_CONDITIONS);

  if (printer.isWaitForHeatUp()) {
    lcdui.set_status_P(no_wait_for_cooling ? PSTR(MSG_HEATING_COMPLETE) : PSTR(MSG_COOLING_COMPLETE));
    #if ENABLED(PRINTER_EVENT_LEDS)
      ledevents.onHeatingDone();
    #endif
  }

  printer.setAutoreportTemp(oldReport);
}

void Heater::getOutput() {

  millis_t now = millis();

  if (!isIdle() && idle_timeout_ms && ELAPSED(now, idle_timeout_ms)) setIdle(true);

  if (isActive()) {

    // Get the target temperature and the error
    const float targetTemperature = isIdle() ? idle_temperature : target_temperature;

    if (isUsePid()) {
      pwm_value = pid.spin(targetTemperature, current_temperature, now
        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          , data.ID
        #endif
      );
    }
    else if (ELAPSED(now, next_check_ms)) {
      next_check_ms = now + temp_check_interval[data.type];
      if (current_temperature <= targetTemperature - temp_hysteresis[data.type])
        pwm_value = pid.Max;
      else if (current_temperature >= targetTemperature + temp_hysteresis[data.type])
        pwm_value = 0;
    }

    #if ENABLED(PID_DEBUG)
      SERIAL_SMV(ECHO, MSG_PID_DEBUG, ACTIVE_HOTEND);
      SERIAL_MV(MSG_PID_DEBUG_INPUT, current_temperature);
      SERIAL_EMV(MSG_PID_DEBUG_OUTPUT, pwm_value);
    #endif // PID_DEBUG

  }

}

void Heater::setOutputPwm() {
  const uint8_t new_pwm_value = isHWInverted() ? 255 - pwm_value : pwm_value;
  HAL::analogWrite(data.pin, new_pwm_value, (data.type == IS_HOTEND) ? 250 : 10);
}

void Heater::print_M305() {
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

void Heater::print_M306() {
  const int8_t heater_id = data.type == IS_HOTEND ? data.ID : -data.type;
  SERIAL_LM(CFG, "Heater parameters: H<Heater> P<Pin> A<Pid Drive Min> B<Pid Drive Max> C<Pid Max> L<Min Temp> O<Max Temp> U<Use Pid 0-1> I<Hardware Inverted 0-1> T<Thermal Protection 0-1>:");
  SERIAL_SMV(CFG, "  M306 H", (int)heater_id);
  SERIAL_MV(" P", data.pin);
  SERIAL_MV(" A", pid.DriveMin);
  SERIAL_MV(" B", pid.DriveMax);
  SERIAL_MV(" C", pid.Max);
  SERIAL_MV(" L", data.mintemp);
  SERIAL_MV(" O", data.maxtemp);
  SERIAL_MV(" U", isUsePid());
  SERIAL_MV(" I", isHWInverted());
  SERIAL_MV(" T", isThermalProtection());
  SERIAL_EOL();
}

void Heater::print_M301() {
  if (isUsePid()) {
    SERIAL_SM(CFG, "Heater PID parameters: H<Heater> P<Proportional> I<Integral> D<Derivative>");
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      if (data.type == IS_HOTEND) SERIAL_MSG(" C<Kc term> L<LPQ length>");
    #endif
    SERIAL_CHR(':');
    SERIAL_EOL();
    SERIAL_SM(CFG, "  M301 H");
    #if HOTENDS > 0
      if (data.type == IS_HOTEND) SERIAL_VAL(data.ID);
    #endif
    #if HAS_TEMP_BED
      if (data.type == IS_BED) SERIAL_VAL(-1);
    #endif
    #if HAS_TEMP_CHAMBER
      if (data.type == IS_CHAMBER) SERIAL_VAL(-2);
    #endif
    #if HAS_TEMP_COOLER
      if (data.type == IS_COOLER) SERIAL_VAL(-3);
    #endif
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
  void Heater::print_M595() {
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
  start_watching();
}

void Heater::thermal_runaway_protection() {

  switch (thermal_runaway_state) {

    // Inactive state waits for a target temperature to be set
    case TRInactive: break;

    // When first heating, wait for the temperature to be reached then go to Stable state
    case TRFirstHeating:
      if (current_temperature < target_temperature) break;
      thermal_runaway_state = TRStable;

    // While the temperature is stable watch for a bad temperature
    case TRStable:

      #if ENABLED(ADAPTIVE_FAN_SPEED) && FAN_COUNT > 0
        if (data.type == IS_HOTEND) {
          if (fans[0].Speed == 0)
            fans[0].scaled_Speed = 128;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.25f))
            fans[0].scaled_Speed = 128;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.40f))
            fans[0].scaled_Speed = 96;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.60f))
            fans[0].scaled_Speed = 64;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.90f))
            fans[0].scaled_Speed = 32;
          else
            fans[0].scaled_Speed = 0;
        }
      #endif

      if (current_temperature >= target_temperature - THERMAL_PROTECTION_HYSTERESIS) {
        thermal_runaway_timer = millis() + THERMAL_PROTECTION_PERIOD * 1000UL;
        break;
      }
      else if (PENDING(millis(), thermal_runaway_timer)) break;
      thermal_runaway_state = TRRunaway;

    default: break;
  }
}

/**
 * Start Heating Sanity Check for heaters that are below
 * their target temperature by a configurable margin.
 * This is called when the temperature is set.
 */
void Heater::start_watching() {

  if (!isThermalProtection()) return;

  const float targetTemperature = isIdle() ? idle_temperature : target_temperature;
  if (isActive() && current_temperature < targetTemperature - (watch_temp_increase[data.type] + TEMP_HYSTERESIS + 1)) {
    watch_target_temp = current_temperature + watch_temp_increase[data.type];
    watch_next_ms = millis() + (watch_temp_period[data.type]) * 1000UL;
  }
  else
    watch_next_ms = 0;

}

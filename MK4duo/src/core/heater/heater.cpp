/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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

Heater hotends[HOTENDS]   = Heater(IS_HOTEND, HOTEND_CHECK_INTERVAL, HOTEND_HYSTERESIS, WATCH_HOTEND_PERIOD, WATCH_HOTEND_INCREASE);
Heater beds[BEDS]         = Heater(IS_BED, BED_CHECK_INTERVAL, BED_HYSTERESIS, WATCH_BED_PERIOD, WATCH_BED_INCREASE);
Heater chambers[CHAMBERS] = Heater(IS_CHAMBER, CHAMBER_CHECK_INTERVAL, CHAMBER_HYSTERESIS, WATCH_CHAMBER_PERIOD, WATCH_CHAMBER_INCREASE);
Heater coolers[COOLERS]   = Heater(IS_COOLER, COOLER_CHECK_INTERVAL, COOLER_HYSTERESIS, WATCH_COOLER_PERIOD, WATCH_COOLER_INCREASE);

/** Public Function */
void Heater::init() {

  // Reset valor
  pwm_value             = 0;
  consecutive_low_temp  = 0;
  target_temperature    = 0;
  idle_temperature      = 0;
  current_temperature   = 25.0;
  sensor.raw            = 0;

  setActive(false);
  setIdle(false);
  ResetFault();

  watch_target_temp     = 0;
  watch_next_ms         = 0;
  idle_timeout_time     = 0;
  Pidtuning             = false;

  thermal_runaway_state = TRInactive;

  sensor.CalcDerivedParameters();

  if (printer.isRunning()) return; // All running not reinitialize

  if (data.pin > 0) HAL::pinMode(data.pin, (isHWinvert()) ? OUTPUT_HIGH : OUTPUT_LOW);

  #if HAS_MAX6675 || HAS_MAX31855
    if (sensor.type == -2 || sensor.type == -1) {
      HAL::pinMode(sensor.pin, OUTPUT_HIGH);
    }
  #endif

}

void Heater::setTarget(const int16_t celsius) {

  if (celsius == 0)
    SwitchOff();
  else if (!isPidTuned() && isUsePid()) {
    SERIAL_LM(ER, " Need Tuning PID");
    LCD_MESSAGEPGM(MSG_NEED_TUNE_PID);
  }
  else if (isFault())
    SERIAL_LM(ER, " Heater not switched on to temperature fault.");
  else {
    setActive(true);
    if (isActive()) {
      target_temperature = MIN(celsius, data.maxtemp - 10);
      thermal_runaway_state = target_temperature > 0 ? TRFirstHeating : TRInactive;
      start_watching();
    }
  }
}

void Heater::wait_for_target(bool no_wait_for_cooling/*=true*/) {

  #if TEMP_RESIDENCY_TIME > 0
    millis_l residency_start_ms = 0;
    // Loop until the temperature has stabilized
    #define TEMP_CONDITIONS (!residency_start_ms || (int32_t(now - residency_start_ms) <= (TEMP_RESIDENCY_TIME) * 1000UL))
  #else
    #define TEMP_CONDITIONS (wants_to_cool ? isCooling() : isHeating())
  #endif

  float     old_temp            = 9999.0;
  bool      wants_to_cool       = false,
            first_loop          = true;
  millis_l  now                 = 0,
            next_cool_check_ms  = 0;

  const bool oldReport = printer.isAutoreportTemp();
  
  printer.setWaitForHeatUp(true);
  printer.setAutoreportTemp(true);

  #if ENABLED(PRINTER_EVENT_LEDS)
    const float start_temp = current_temperature;
    const bool isHotend = type == IS_HOTEND;
    ledevents.onHeatingStart(isHotend);
  #endif

  do {

    wants_to_cool = isCooling();
    // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
    if (no_wait_for_cooling && wants_to_cool) break;

    now = millis();
    printer.idle();
    printer.keepalive(WaitHeater);
    printer.reset_move_ms();  // Keep steppers powered

    const float temp = current_temperature;

    #if ENABLED(PRINTER_EVENT_LEDS)
      if (!wants_to_cool) ledevents.onHeating(isHotend, start_temp, temp, target_temperature);
    #endif

    #if TEMP_RESIDENCY_TIME > 0

      const float temp_diff = ABS(target_temperature - temp);

      if (!residency_start_ms) {
        // Start the residency_start_ms timer when we reach target temp for the first time.
        if (temp_diff < TEMP_WINDOW) {
          residency_start_ms = now;
          if (first_loop) residency_start_ms += (TEMP_RESIDENCY_TIME) * 1000UL;
        }
      }
      else if (temp_diff > temp_hysteresis) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = now;
      }

    #endif

    // Prevent a wait-forever situation if R is misused i.e. M190 R0
    if (wants_to_cool) {
      // Break after 60 seconds
      // if the temperature did not drop at least 1.5
      if (!next_cool_check_ms || expired(&next_cool_check_ms, 60000UL)) {
        if (old_temp - temp < 1.5) break;
        next_cool_check_ms = now;
        old_temp = temp;
      }
    }

    first_loop = false;

  } while (printer.isWaitForHeatUp() && TEMP_CONDITIONS);

  if (printer.isWaitForHeatUp()) {
    lcdui.reset_status();
    #if ENABLED(PRINTER_EVENT_LEDS)
      ledevents.onHeatingDone();
    #endif
  }

  printer.setAutoreportTemp(oldReport);
}

void Heater::get_output() {

  static millis_l idle_timeout_ms = millis();
  static millis_s next_check_ms   = millis();

  if (!isIdle() && idle_timeout_time && expired(&idle_timeout_ms, idle_timeout_time)) setIdle(true);

  if (isActive()) {

    // Get the target temperature and the error
    const float targetTemperature = isIdle() ? idle_temperature : target_temperature;

    #if COOLERS > 0
      if (type == IS_COOLER) {
        if (isUsePid()) {
          pwm_value = pid.spin(current_temperature, targetTemperature
            #if ENABLED(PID_ADD_EXTRUSION_RATE)
              , 0xFF
            #endif
          );
        }
        else if (expired(&next_check_ms, temp_check_interval)) {
          if (current_temperature <= targetTemperature - temp_hysteresis)
            pwm_value = 0;
          else if (current_temperature >= targetTemperature + temp_hysteresis)
            pwm_value = pid.Max;
        }
      }
      else
    #endif
      {
        if (isUsePid()) {
          #if ENABLED(PID_ADD_EXTRUSION_RATE)
            const uint8_t id = (type == IS_HOTEND) ? data.ID : 0xFF;
          #endif
          pwm_value = pid.spin(targetTemperature, current_temperature
            #if ENABLED(PID_ADD_EXTRUSION_RATE)
              , id
            #endif
          );
        }
        else if (expired(&next_check_ms, temp_check_interval)) {
          if (current_temperature >= targetTemperature + temp_hysteresis)
            pwm_value = 0;
          else if (current_temperature <= targetTemperature - temp_hysteresis)
            pwm_value = pid.Max;
        }
      }

    /**
    if (printer.debugFeature() && type == IS_HOTEND) {
      DEBUG_SMV(DEB, MSG_PID_DEBUG, ACTIVE_HOTEND);
      DEBUG_MV(MSG_PID_DEBUG_INPUT, current_temperature);
      DEBUG_MV(MSG_PID_DEBUG_OUTPUT, pwm_value);
      DEBUG_MV(MSG_PID_DEBUG_PTERM, pid.Kp);
      DEBUG_MV(MSG_PID_DEBUG_ITERM, pid.Ki);
      DEBUG_MV(MSG_PID_DEBUG_DTERM, pid.Kd);
      DEBUG_EOL();
    }
    */

  }

}

void Heater::set_output_pwm() {
  HAL::analogWrite(data.pin, isHWinvert() ? (255 - pwm_value) : pwm_value, data.freq, data.flag.HWpwm);
}

void Heater::check_and_power() {

  if (isActive() && current_temperature > data.maxtemp) max_temp_error();
  if (isActive() && current_temperature < data.mintemp) {
    if (++consecutive_low_temp >= MAX_CONSECUTIVE_LOW_TEMP)
      min_temp_error();
  }
  else
    consecutive_low_temp = 0;

  // Check for thermal runaway
  if (isThermalProtection()) {
    thermal_runaway_protection();
    if (thermal_runaway_state == TRRunaway)
      _temp_error(PSTR(MSG_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
  }

  // Ignore heater we are currently testing
  if (Pidtuning) return;

  get_output();

  // Make sure temperature is increasing
  if (isThermalProtection() && watch_next_ms && expired(&watch_next_ms, millis_s(watch_period * 1000U))) {
    if (current_temperature < watch_target_temp)
      _temp_error(PSTR(MSG_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
    else
      start_watching(); // Start again if the target is still far off
  }
}

/**
 * PID Autotuning (M303)
 *
 * Alternately heat and cool the nozzle, observing its behavior to
 * determine the best PID values to achieve a stable temperature.
 */
void Heater::PID_autotune(const float target_temp, const uint8_t ncycles, const uint8_t method, const bool storeValues/*=false*/) {

  float       current_temp  = 0.0;
  int         cycles        = 0;
  bool        heating       = true;
  const bool  isHotend      = type == IS_HOTEND,
              oldReport     = printer.isAutoreportTemp();

  thermalManager.disable_all_heaters(); // switch off all heaters.

  millis_l  t1      = millis(),
            t2      = t1;

  int32_t   t_high  = 0.0,
            t_low   = 0.0;

  float     maxTemp = 0.0,
            minTemp = 1000.0;

  pid_data_t tune_pid;

  int32_t bias  = pid.Max >> 1,
          d     = pid.Max >> 1;

  printer.setWaitForHeatUp(true);
  printer.setAutoreportTemp(true);

  Pidtuning = true;
  ResetFault();

  // Turn ON this heater to max power.
  pwm_value = pid.Max;

  #if ENABLED(PRINTER_EVENT_LEDS)
    const float start_temp = current_temperature;
    LEDColor color = ledevents.onHeatingStart(isHotend);
  #endif

  // PID Tuning loop
  while (printer.isWaitForHeatUp()) {

    watchdog.reset(); // Reset the watchdog
    printer.idle();
    printer.keepalive(WaitHeater);

    update_current_temperature();

    const millis_l now = millis();

    current_temp = current_temperature;
    NOLESS(maxTemp, current_temp);
    NOMORE(minTemp, current_temp);

    #if ENABLED(PRINTER_EVENT_LEDS)
      ledevents.onHeating(isHotend, start_temp, current_temp, target_temp);
    #endif

    if (heating && current_temp > target_temp) {
      if (int32_t(now - t2) >= 5000UL) {
        heating = false;
        pwm_value = (bias - d);
        t1 = now;
        t_high = t1 - t2;

        #if COOLERS > 0
          if (type == IS_COOLER)
            minTemp = target_temp;
          else
            maxTemp = target_temp;
        #else
          maxTemp = target_temp;
        #endif
      }
    }

    if (!heating && current_temp < target_temp) {
      if (int32_t(now - t1) >= 5000UL) {
        heating = true;
        t2 = now;
        t_low = t2 - t1;
        if (cycles > 0) {

          bias += (d * (t_high - t_low)) / (t_low + t_high);
          bias = constrain(bias, 20, pid.Max - 20);
          d = (bias > pid.Max >> 1) ? pid.Max - 1 - bias : bias;

          SERIAL_MV(MSG_BIAS, bias);
          SERIAL_MV(MSG_D, d);
          SERIAL_MV(MSG_T_MIN, minTemp);
          SERIAL_MV(MSG_T_MAX, maxTemp);

          if (cycles > 2) {
            float Ku = (4.0f * d) / (float(M_PI) * (maxTemp - minTemp) * 0.5f),
                  Tu = ((float)(t_low + t_high) * 0.001f);
            SERIAL_MV(MSG_KU, Ku);
            SERIAL_MV(MSG_TU, Tu);
            SERIAL_EOL();

            if (method == 0) {
              tune_pid.Kp = 0.6f * Ku;
              tune_pid.Ki = 1.2f * Ku / Tu;
              tune_pid.Kd = 0.075f * Ku * Tu;
              SERIAL_MSG(MSG_CLASSIC_PID);
            }
            else if (method == 1) {
              tune_pid.Kp = 0.33f * Ku;
              tune_pid.Ki = 0.66f * Ku / Tu;
              tune_pid.Kd = 0.11f * Ku * Tu;
              SERIAL_MSG(MSG_SOME_OVERSHOOT_PID);
            }
            else if (method == 2) {
              tune_pid.Kp = 0.2f * Ku;
              tune_pid.Ki = 0.4f * Ku / Tu;
              tune_pid.Kd = 0.2f * Ku * Tu / 3.0f;
              SERIAL_MSG(MSG_NO_OVERSHOOT_PID);
            }
            else if (method == 3) {
              tune_pid.Kp = 0.7f * Ku;
              tune_pid.Ki = 1.75f * Ku / Tu;
              tune_pid.Kd = 0.105f * Ku * Tu;
              SERIAL_MSG(MSG_PESSEN_PID);
            }
            else if (method == 4) {
              tune_pid.Kp = 0.4545f * Ku;
              tune_pid.Ki = 0.4545f * Ku / Tu / 2.2f;
              tune_pid.Kd = 0.4545f * Ku * Tu / 6.3f;
              SERIAL_MSG(MSG_TYREUS_LYBEN_PID);
            }
            SERIAL_MV(MSG_KP, tune_pid.Kp);
            SERIAL_MV(MSG_KI, tune_pid.Ki);
            SERIAL_MV(MSG_KD, tune_pid.Kd);
          }
        }

        SERIAL_EOL();

        pwm_value = (bias + d);
        cycles++;

        #if COOLERS > 0
          if (type == IS_COOLER)
            maxTemp = target_temp;
          else
            minTemp = target_temp;
        #else
          minTemp = target_temp;
        #endif
      }
    }

    #if DISABLED(MAX_OVERSHOOT_PID_AUTOTUNE)
      #define MAX_OVERSHOOT_PID_AUTOTUNE 20
    #endif
    if (current_temp > target_temp + MAX_OVERSHOOT_PID_AUTOTUNE
      #if COOLERS > 0
        && type != IS_COOLER
      #endif
    ) {
      SERIAL_LM(ER, MSG_PID_TEMP_TOO_HIGH);
      LCD_ALERTMESSAGEPGM(MSG_PID_TEMP_TOO_HIGH);
      Pidtuning = false;
      break;
    }
    #if COOLERS > 0
      else if (current_temp < target_temp + MAX_OVERSHOOT_PID_AUTOTUNE && type == IS_COOLER) {
        SERIAL_LM(ER, MSG_PID_TEMP_TOO_LOW);
        LCD_ALERTMESSAGEPGM(MSG_PID_TEMP_TOO_LOW);
        Pidtuning = false;
        break;
      }
    #endif

    // Timeout after MAX_CYCLE_TIME_PID_AUTOTUNE minutes since the last undershoot/overshoot cycle
    #if DISABLED(MAX_CYCLE_TIME_PID_AUTOTUNE)
      #define MAX_CYCLE_TIME_PID_AUTOTUNE 20L
    #endif
    if (((now - t1) + (now - t2)) > (MAX_CYCLE_TIME_PID_AUTOTUNE * 60L * 1000L)) {
      SERIAL_LM(ER, MSG_PID_TIMEOUT);
      LCD_ALERTMESSAGEPGM(MSG_PID_TIMEOUT);
      Pidtuning = false;
      break;
    }

    if (cycles > ncycles) {

      SERIAL_EM(MSG_PID_AUTOTUNE_FINISHED);
      Pidtuning = false;

      if (isHotend) {
        SERIAL_MV(MSG_KP, tune_pid.Kp);
        SERIAL_MV(MSG_KI, tune_pid.Ki);
        SERIAL_EMV(MSG_KD, tune_pid.Kd);
      }

      #if BEDS > 0
        if (type == IS_BED) {
          SERIAL_EMV("#define BED_Kp ", tune_pid.Kp);
          SERIAL_EMV("#define BED_Ki ", tune_pid.Ki);
          SERIAL_EMV("#define BED_Kd ", tune_pid.Kd);
        }
      #endif

      #if CHAMBERS > 0
        if (type == IS_CHAMBER) {
          SERIAL_EMV("#define CHAMBER_Kp ", tune_pid.Kp);
          SERIAL_EMV("#define CHAMBER_Ki ", tune_pid.Ki);
          SERIAL_EMV("#define CHAMBER_Kd ", tune_pid.Kd);
        }
      #endif

      #if COOLERS > 0
        if (type == IS_COOLER) {
          SERIAL_EMV("#define COOLER_Kp ", tune_pid.Kp);
          SERIAL_EMV("#define COOLER_Ki ", tune_pid.Ki);
          SERIAL_EMV("#define COOLER_Kd ", tune_pid.Kd);
        }
      #endif

      pid.Kp = tune_pid.Kp;
      pid.Ki = tune_pid.Ki;
      pid.Kd = tune_pid.Kd;
      pid.update();

      setPidTuned(true);
      Pidtuning = false;
      ResetFault();

      if (storeValues) eeprom.store();

      #if ENABLED(PRINTER_EVENT_LEDS)
        ledevents.onPidTuningDone(color);
      #endif

      break;
    }

    lcdui.update();

  }

  thermalManager.disable_all_heaters();

  printer.setAutoreportTemp(oldReport);

  LCD_MESSAGEPGM(WELCOME_MSG);

}

void Heater::print_M301() {
  if (isUsePid()) {
    const int8_t heater_id = type == IS_HOTEND ? data.ID : -type;
    SERIAL_SM(CFG, "Heater PID parameters: H<Heater>");
    if (heater_id < 0) SERIAL_MSG(" T<tools>");
    SERIAL_MSG(" P<Proportional> I<Integral> D<Derivative>");
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      if (type == IS_HOTEND) SERIAL_MSG(" C<Kc term> L<LPQ length>");
    #endif
    SERIAL_CHR(':');
    SERIAL_EOL();
    SERIAL_SMV(CFG, "  M301 H", int(heater_id));
    if (heater_id < 0) SERIAL_MV(" T", int(data.ID));
    SERIAL_MV(" P", pid.Kp);
    SERIAL_MV(" I", pid.Ki);
    SERIAL_MV(" D", pid.Kd);
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      if (type == IS_HOTEND) {
        SERIAL_MV(" C", pid.Kc);
        SERIAL_MV(" L", (int)tools.lpq_len);
      }
    #endif
    SERIAL_EOL();
  }
}

void Heater::print_M305() {
  const int8_t heater_id = type == IS_HOTEND ? data.ID : -type;
  SERIAL_SM(CFG, "Heater Sensor parameters: H<Heater>");
  if (heater_id < 0) SERIAL_MSG(" T<tools>");
  SERIAL_EM(" P<Pin> S<Type> A<R25> B<BetaK> C<Steinhart-Hart C> R<Pullup> L<ADC low offset> O<ADC high offset>:");
  SERIAL_SMV(CFG, "  M305 H", (int)heater_id);
  if (heater_id < 0) SERIAL_MV(" T", int(data.ID));
  SERIAL_MV(" P", sensor.pin);
  SERIAL_MV(" S", sensor.type);
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
  const int8_t heater_id = type == IS_HOTEND ? data.ID : -type;
  SERIAL_SM(CFG, "Heater parameters: H<Heater>");
  if (heater_id < 0) SERIAL_MSG(" T<tools>");
  SERIAL_EM(" P<Pin> A<Pid Drive Min> B<Pid Drive Max> C<Pid Max> F<Freq> L<Min Temp> O<Max Temp> U<Use Pid 0-1> I<Hardware Inverted 0-1> R<Thermal Protection 0-1> Q<Pwm Hardware 0-1>:");
  SERIAL_SMV(CFG, "  M306 H", (int)heater_id);
  if (heater_id < 0) SERIAL_MV(" T", int(data.ID));
  SERIAL_MV(" P", data.pin);
  SERIAL_MV(" A", pid.DriveMin);
  SERIAL_MV(" B", pid.DriveMax);
  SERIAL_MV(" C", pid.Max);
  SERIAL_MV(" F", data.freq);
  SERIAL_MV(" L", data.mintemp);
  SERIAL_MV(" O", data.maxtemp);
  SERIAL_MV(" U", isUsePid());
  SERIAL_MV(" I", isHWinvert());
  SERIAL_MV(" Q", isHWpwm());
  SERIAL_MV(" R", isThermalProtection());
  SERIAL_EOL();

  if (printer.debugFeature()) {
    DEBUG_SMV(DEB, " Type:", type);
    DEBUG_MV(" temp_check_interval:", temp_check_interval);
    DEBUG_MV(" temp_hysteresis:", temp_hysteresis);
    DEBUG_MV(" watch_period:", watch_period);
    DEBUG_MV(" watch_increase:", watch_increase);
    DEBUG_EOL();
  }

}

#if HAS_AD8495 || HAS_AD595
  void Heater::print_M595() {
    const int8_t heater_id = type == IS_HOTEND ? data.ID : -type;
    SERIAL_LM(CFG, "AD595 or AD8495 parameters: H<Hotend>");
    if (heater_id < 0) SERIAL_MSG(" T<tools>");
    SERIAL_MSG(" O<Offset> S<Gain>:");
    SERIAL_SMV(CFG, "  M595 H", (int)heater_id);
    if (heater_id < 0) SERIAL_MV(" T", int(data.ID));
    SERIAL_MV(" O", sensor.ad595_offset);
    SERIAL_MV(" S", sensor.ad595_gain);
    SERIAL_EOL();
  }
#endif

void Heater::start_idle_timer(const millis_l timeout_time) {
  idle_timeout_time = timeout_time;
  setIdle(false);
}

void Heater::reset_idle_timer() {
  idle_timeout_time = 0;
  setIdle(false);
  start_watching();
}

void Heater::thermal_runaway_protection() {

  static millis_s thermal_runaway_ms = millis();

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
        if (type == IS_HOTEND) {
          if (fans[0].speed == 0)
            fans[0].scaled_speed = 128;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.25f))
            fans[0].scaled_speed = 128;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.40f))
            fans[0].scaled_speed = 96;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.60f))
            fans[0].scaled_speed = 64;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.90f))
            fans[0].scaled_speed = 32;
          else
            fans[0].scaled_speed = 0;
        }
      #endif

      if (current_temperature >= target_temperature - THERMAL_PROTECTION_HYSTERESIS) {
        thermal_runaway_ms = millis();
        break;
      }
      else if (pending(&thermal_runaway_ms, millis_s(THERMAL_PROTECTION_PERIOD * 1000U))) break;
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
  if (isActive() && current_temperature < targetTemperature - (watch_increase + temp_hysteresis + 1)) {
    watch_target_temp = current_temperature + watch_increase;
    watch_next_ms = millis();
  }
  else
    watch_next_ms = 0;

}

/** Private Function */
// Temperature Error Handlers
void Heater::_temp_error(PGM_P const serial_msg, PGM_P const lcd_msg) {
  if (isActive()) {
    SERIAL_STR(ER);
    SERIAL_STR(serial_msg);
    SERIAL_MSG(MSG_HEATER_STOPPED);
    switch (type) {
      case IS_HOTEND:
        SERIAL_EMV(MSG_HEATER_HOTEND " ", int(data.ID));
        break;
      #if BEDS > 0
        case IS_BED:
          SERIAL_EMV(MSG_HEATER_BED " ", int(data.ID));
          break;
      #endif
      #if CHAMBERS > 0
        case IS_CHAMBER:
          SERIAL_EMV(MSG_HEATER_CHAMBER " ", int(data.ID));
          break;
      #endif
      #if HAS_TEMP_COOLER
        case IS_COOLER:
          SERIAL_EM(MSG_HEATER_COOLER);
          break;
      #endif
      default: break;
    }
  }

  lcdui.set_status_P(lcd_msg);
  setFault();

}

void Heater::min_temp_error() {
  switch (type) {
    case IS_HOTEND:
      _temp_error(PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP));
      break;
    #if BEDS > 0
      case IS_BED:
        _temp_error(PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP_BED));
        break;
    #endif
    #if CHAMBERS > 0
      case IS_CHAMBER:
        _temp_error(PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP_CHAMBER));
        break;
    #endif
    default: break;
  }
}

void Heater::max_temp_error() {
  switch (type) {
    case IS_HOTEND:
      _temp_error(PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP));
      break;
    #if BEDS > 0
      case IS_BED:
        _temp_error(PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP_BED));
        break;
    #endif
    #if CHAMBERS > 0
      case IS_CHAMBER:
        _temp_error(PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP_CHAMBER));
        break;
    #endif
    default: break;
  }
}

#endif // HEATER_COUNT > 0

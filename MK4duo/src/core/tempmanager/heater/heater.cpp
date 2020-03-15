/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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

#include "../../../../MK4duo.h"
#include "sanitycheck.h"

#if HAS_HOTENDS
  Heater* hotends[MAX_HOTEND]   = { nullptr };
#endif
#if HAS_BEDS
  Heater* beds[MAX_BED]         = { nullptr };
#endif
#if HAS_CHAMBERS
  Heater* chambers[MAX_CHAMBER] = { nullptr };
#endif
#if HAS_COOLERS
  Heater* coolers[MAX_COOLER]   = { nullptr };
#endif

/** Public Function */
void Heater::init() {

  // Reset valor
  pwm_value             = 0;
  pwm_soft_pos          = 0;
  consecutive_low_temp  = 0;
  target_temperature    = 0;
  idle_temperature      = 0;
  data.sensor.adc_raw   = 0;

  current_temperature   = 25.0;

  setActive(false);
  setIdle(false);
  ResetFault();
  next_check_timer.start();
  data.pid.init();

  watch_target_temp     = 0;
  idle_timeout_ms       = 0;
  Pidtuning             = false;

  thermal_runaway_state = TRInactive;

  data.sensor.CalcDerivedParameters();

  if (printer.isRunning()) return; // All running not reinitialize

  if (data.pin > NoPin) HAL::pinMode(data.pin, (isHWinvert()) ? OUTPUT_HIGH : OUTPUT_LOW);

  #if HAS_MAX6675 || HAS_MAX31855
    if (data.sensor.type == -2 || data.sensor.type == -1) {
      HAL::pinMode(data.sensor.pin, OUTPUT_HIGH);
    }
  #endif

}

void Heater::set_target_temp(const int16_t celsius) {

  if (celsius == 0)
    SwitchOff();
  else if (!isPidTuned() && isUsePid()) {
    SwitchOff();
    SERIAL_LM(ER, " Need Tuning PID");
    LCD_MESSAGEPGM(MSG_NEED_TUNE_PID);
  }
  else if (isFault()) {
    SwitchOff();
    SERIAL_LM(ER, " Heater not switched on to temperature fault.");
  }
  else if (celsius < data.temp.min) {
    SwitchOff();
    print_low_high_temp(celsius, true);
  }
  else if (celsius > data.temp.max - 10) {
    SwitchOff();
    print_low_high_temp(celsius, false);
  }
  else {
    setActive(true);
    target_temperature = celsius;
  }

  if (isActive()) {
    thermal_runaway_state = target_temperature > 0 ? TRFirstHeating : TRInactive;
    start_watching();
  }

}

void Heater::set_idle_temp(const int16_t celsius) {
  if (celsius < data.temp.min)
    print_low_high_temp(celsius, true);
  else if (celsius > data.temp.max - 10)
    print_low_high_temp(celsius, false);
  else
    idle_temperature = celsius;
}

void Heater::wait_for_target(bool no_wait_for_cooling/*=true*/) {

  #if TEMP_RESIDENCY_TIME > 0
    long_timer_t residency_start_timer;
    // Loop until the temperature has stabilized
    #define TEMP_CONDITIONS (!residency_start_timer.isRunning() || residency_start_timer.pending((TEMP_RESIDENCY_TIME) * 1000))
  #else
    #define TEMP_CONDITIONS (wants_to_cool ? isCooling() : isHeating())
  #endif

  float old_temp      = 9999.0;
  bool  wants_to_cool = false,
        first_loop    = true;

  short_timer_t next_cool_check_timer;

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

    printer.idle();
    printer.reset_move_timer();  // Keep steppers powered

    const float temp = current_temperature;

    #if ENABLED(PRINTER_EVENT_LEDS)
      if (!wants_to_cool) ledevents.onHeating(isHotend, start_temp, temp, target_temperature);
    #endif

    #if TEMP_RESIDENCY_TIME > 0

      const float temp_diff = ABS(target_temperature - temp);

      if (!residency_start_timer.isRunning()) {
        // Start the residency_start_timer timer when we reach target temp for the first time.
        if (temp_diff < TEMP_WINDOW) {
          first_loop ? residency_start_timer.start((TEMP_RESIDENCY_TIME) * 1000UL) : residency_start_timer.start();
        }
      }
      else if (temp_diff > temp_hysteresis) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_timer.start();
      }

    #endif

    // Prevent a wait-forever situation if R is misused i.e. M190 R0
    if (wants_to_cool) {
      // Break after 60 seconds
      // if the temperature did not drop at least 1.5
      if (!next_cool_check_timer.isRunning() || next_cool_check_timer.expired(60000)) {
        if (old_temp - temp < 1.5) break;
        next_cool_check_timer.start();
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

  printer.setWaitForHeatUp(false);
  printer.setAutoreportTemp(oldReport);
}

void Heater::get_output() {

  update_idle_timer();

  if (isActive()) {

    // Get the target temperature and the error
    const float targetTemperature = isIdle() ? idle_temperature : target_temperature;

    #if HAS_COOLERS
      if (type == IS_COOLER) {
        if (isUsePid()) {
          pwm_value = data.pid.compute(current_temperature, targetTemperature
            #if ENABLED(PID_ADD_EXTRUSION_RATE)
              , 0xFF
            #endif
          );
        }
        else if (next_check_timer.expired(temp_check_interval))
          pwm_value = current_temperature >= targetTemperature ? data.pid.drive.max : 0;
      }
      else
    #endif
      {
        if (current_temperature >= targetTemperature + temp_hysteresis)
          pwm_value = 0;
        else if (current_temperature <= targetTemperature - temp_hysteresis)
          pwm_value = data.pid.Max;
        else if (isUsePid()) {
          #if ENABLED(PID_ADD_EXTRUSION_RATE)
            const uint8_t id = (type == IS_HOTEND) ? data.ID : 0xFF;
          #endif
          pwm_value = data.pid.compute(targetTemperature, current_temperature
            #if ENABLED(PID_ADD_EXTRUSION_RATE)
              , id, tempManager.heater.lpq_len
            #endif
          );
        }
        else if (next_check_timer.expired(temp_check_interval)) {
          pwm_value = current_temperature < targetTemperature ? data.pid.drive.max : data.pid.drive.min;
        }
      }

    /**
    if (printer.debugFeature() && type == IS_HOTEND) {
      DEBUG_SMV(DEB, MSG_HOST_PID_DEBUG, toolManager.active_hotend());
      DEBUG_MV(MSG_HOST_PID_DEBUG_INPUT, current_temperature);
      DEBUG_MV(MSG_HOST_PID_DEBUG_OUTPUT, pwm_value);
      DEBUG_MV(MSG_HOST_PID_DEBUG_PTERM, data.pid.Kp);
      DEBUG_MV(MSG_HOST_PID_DEBUG_ITERM, data.pid.Ki);
      DEBUG_MV(MSG_HOST_PID_DEBUG_DTERM, data.pid.Kd);
      DEBUG_EOL();
    }
    */

  }

}

void Heater::set_output_pwm() {

  const uint8_t new_pwm = isHWinvert() ? 255 - pwm_value : pwm_value;

  if (data.pin > NoPin) {
    if (isHWpwm())
      HAL::analogWrite(data.pin, new_pwm, data.freq);
    else {
      #if ENABLED(SOFTWARE_PDM)
        const uint8_t carry = pwm_soft_pos + new_pwm;
        HAL::digitalWrite(data.pin, (carry < pwm_soft_pos));
        pwm_soft_pos = carry;
      #else // SOFTWARE PWM
        // Turn HIGH Software PWM
        if (tempManager.pwm_soft_count == 0 && ((pwm_soft_pos = (new_pwm & SOFT_PWM_MASK)) > 0))
          HAL::digitalWrite(data.pin, HIGH);
        // Turn LOW Software PWM
        if (pwm_soft_pos == tempManager.pwm_soft_count && pwm_soft_pos != SOFT_PWM_MASK)
          HAL::digitalWrite(data.pin, LOW);
      #endif
    }
  }

}

void Heater::check_and_power() {

  if (isActive() && current_temperature > data.temp.max) max_temp_error();
  if (isActive() && current_temperature < data.temp.min) {
    if (++consecutive_low_temp >= MAX_CONSECUTIVE_LOW_TEMP)
      min_temp_error();
  }
  else
    consecutive_low_temp = 0;

  // Check for thermal runaway
  if (isThermalProtection()) {
    thermal_runaway_protection();
    if (thermal_runaway_state == TRRunaway)
      temp_error(PSTR(MSG_HOST_T_THERMAL_RUNAWAY), GET_TEXT(MSG_THERMAL_RUNAWAY));
  }

  // Ignore heater we are currently testing
  if (Pidtuning) return;

  get_output();

  // Make sure temperature is increasing
  if (isThermalProtection() && next_watch_timer.isRunning() && next_watch_timer.expired(watch_period * 1000, false)) {
    if (current_temperature < watch_target_temp)
      temp_error(PSTR(MSG_HOST_HEATING_FAILED), GET_TEXT(MSG_HEATING_FAILED));
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

  float       current_temp  = 0.0f;
  int         cycles        = 0;
  bool        heating       = true;
  const bool  isHotend      = type == IS_HOTEND,
              oldReport     = printer.isAutoreportTemp();

  tempManager.disable_all_heaters(); // switch off all heaters.

  millis_l  t1      = millis(),
            t2      = t1;

  long      t_high  = 0,
            t_low   = 0;

  float     maxTemp = 0.0f,
            minTemp = 1000.0f;

  pid_data_t tune_pid;

  tune_pid.Kp = 0.0;
  tune_pid.Ki = 0.0;
  tune_pid.Kd = 0.0;

  long    bias  = data.pid.Max >> 1,
          d     = data.pid.Max >> 1;

  printer.setWaitForHeatUp(true);
  printer.setAutoreportTemp(true);

  Pidtuning = true;
  ResetFault();

  // Turn ON this heater to max power.
  pwm_value = data.pid.Max;

  #if ENABLED(PRINTER_EVENT_LEDS)
    const float start_temp = current_temperature;
    LEDColor color = ledevents.onHeatingStart(isHotend);
  #endif

  // PID Tuning loop
  while (printer.isWaitForHeatUp()) {

    printer.idle();

    update_current_temperature();

    const millis_l now = millis();

    current_temp = current_temperature;
    NOLESS(maxTemp, current_temp);
    NOMORE(minTemp, current_temp);

    #if ENABLED(PRINTER_EVENT_LEDS)
      ledevents.onHeating(isHotend, start_temp, current_temp, target_temp);
    #endif

    if (heating && current_temp > target_temp) {
      if (ELAPSED(now, t2 + 5000UL)) {
        heating = false;
        pwm_value = (bias - d);
        t1 = now;
        t_high = t1 - t2;

        #if HAS_COOLERS
          type == IS_COOLER ? minTemp = target_temp : maxTemp = target_temp;
        #else
          maxTemp = target_temp;
        #endif
      }
    }

    if (!heating && current_temp < target_temp) {
      if (ELAPSED(now, t1 + 5000UL)) {
        heating = true;
        t2 = now;
        t_low = t2 - t1;
        if (cycles > 0) {

          bias += (d * (t_high - t_low)) / (t_low + t_high);
          LIMIT(bias, 20, data.pid.Max - 20);
          d = (bias > data.pid.Max >> 1) ? data.pid.Max - 1 - bias : bias;

          SERIAL_MV(MSG_HOST_BIAS, bias);
          SERIAL_MV(MSG_HOST_D, d);
          SERIAL_MV(MSG_HOST_T_MIN, minTemp);
          SERIAL_MV(MSG_HOST_T_MAX, maxTemp);

          if (cycles > 2) {
            const float Ku = (4.0f * d) / (float(M_PI) * (maxTemp - minTemp) * 0.5f),
                        Tu = float(t_low + t_high) * 0.001f,
                        pf = isHotend ? 0.6f : 0.2f,
                        df = isHotend ? 1.0f / 8.0f : 1.0f / 3.0f;
            SERIAL_MV(MSG_HOST_KU, Ku);
            SERIAL_MV(MSG_HOST_TU, Tu);
            SERIAL_EOL();

            if (method == 0) {
              tune_pid.Kp = Ku * pf;
              tune_pid.Ki = Ku * pf * 2 / Tu;
              tune_pid.Kd = Ku * pf * df * Tu;
              SERIAL_MSG(MSG_HOST_CLASSIC_PID);
            }
            else if (method == 1) {
              tune_pid.Kp = 0.33f * Ku;
              tune_pid.Ki = 0.66f * Ku / Tu;
              tune_pid.Kd = 0.11f * Ku * Tu;
              SERIAL_MSG(MSG_HOST_SOME_OVERSHOOT_PID);
            }
            else if (method == 2) {
              tune_pid.Kp = 0.2f * Ku;
              tune_pid.Ki = 0.4f * Ku / Tu;
              tune_pid.Kd = 0.2f * Ku * Tu / 3.0f;
              SERIAL_MSG(MSG_HOST_NO_OVERSHOOT_PID);
            }
            else if (method == 3) {
              tune_pid.Kp = 0.7f * Ku;
              tune_pid.Ki = 1.75f * Ku / Tu;
              tune_pid.Kd = 0.105f * Ku * Tu;
              SERIAL_MSG(MSG_HOST_PESSEN_PID);
            }
            else if (method == 4) {
              tune_pid.Kp = 0.4545f * Ku;
              tune_pid.Ki = 0.4545f * Ku / Tu / 2.2f;
              tune_pid.Kd = 0.4545f * Ku * Tu / 6.3f;
              SERIAL_MSG(MSG_HOST_TYREUS_LYBEN_PID);
            }
            SERIAL_MV(MSG_HOST_KP, tune_pid.Kp);
            SERIAL_MV(MSG_HOST_KI, tune_pid.Ki);
            SERIAL_MV(MSG_HOST_KD, tune_pid.Kd);
          }
        }

        SERIAL_EOL();

        pwm_value = (bias + d);
        cycles++;

        #if HAS_COOLERS
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
      #if HAS_COOLERS
        && type != IS_COOLER
      #endif
    ) {
      SERIAL_LM(ER, MSG_HOST_PID_TEMP_TOO_HIGH);
      LCD_ALERTMESSAGEPGM_P(PSTR(MSG_HOST_PID_TEMP_TOO_HIGH));
      Pidtuning = false;
      break;
    }
    #if HAS_COOLERS
      else if (current_temp < target_temp + MAX_OVERSHOOT_PID_AUTOTUNE && type == IS_COOLER) {
        SERIAL_LM(ER, MSG_HOST_PID_TEMP_TOO_LOW);
        LCD_ALERTMESSAGEPGM_P(PSTR(MSG_HOST_PID_TEMP_TOO_LOW));
        Pidtuning = false;
        break;
      }
    #endif

    // Timeout after MAX_CYCLE_TIME_PID_AUTOTUNE minutes since the last undershoot/overshoot cycle
    #if DISABLED(MAX_CYCLE_TIME_PID_AUTOTUNE)
      #define MAX_CYCLE_TIME_PID_AUTOTUNE 20L
    #endif
    if (((now - t1) + (now - t2)) > (MAX_CYCLE_TIME_PID_AUTOTUNE * 60L * 1000L)) {
      SERIAL_LM(ER, MSG_HOST_PID_TIMEOUT);
      LCD_ALERTMESSAGEPGM_P(PSTR(MSG_HOST_PID_TIMEOUT));
      Pidtuning = false;
      break;
    }

    if (cycles > ncycles) {

      SERIAL_EM(MSG_HOST_PID_AUTOTUNE_FINISHED);
      Pidtuning = false;

      if (isHotend) {
        SERIAL_MV(MSG_HOST_KP, tune_pid.Kp);
        SERIAL_MV(MSG_HOST_KI, tune_pid.Ki);
        SERIAL_EMV(MSG_HOST_KD, tune_pid.Kd);
      }

      #if HAS_BEDS
        if (type == IS_BED) {
          SERIAL_EMV("#define BED_Kp ", tune_pid.Kp);
          SERIAL_EMV("#define BED_Ki ", tune_pid.Ki);
          SERIAL_EMV("#define BED_Kd ", tune_pid.Kd);
        }
      #endif

      #if HAS_CHAMBERS
        if (type == IS_CHAMBER) {
          SERIAL_EMV("#define CHAMBER_Kp ", tune_pid.Kp);
          SERIAL_EMV("#define CHAMBER_Ki ", tune_pid.Ki);
          SERIAL_EMV("#define CHAMBER_Kd ", tune_pid.Kd);
        }
      #endif

      #if HAS_COOLERS
        if (type == IS_COOLER) {
          SERIAL_EMV("#define COOLER_Kp ", tune_pid.Kp);
          SERIAL_EMV("#define COOLER_Ki ", tune_pid.Ki);
          SERIAL_EMV("#define COOLER_Kd ", tune_pid.Kd);
        }
      #endif

      data.pid.Kp = tune_pid.Kp;
      data.pid.Ki = tune_pid.Ki;
      data.pid.Kd = tune_pid.Kd;

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

  tempManager.disable_all_heaters();

  printer.setWaitForHeatUp(false);
  printer.setAutoreportTemp(oldReport);

  LCD_MESSAGEPGM(MSG_WELCOME);

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
    SERIAL_MV(" P", data.pid.Kp);
    SERIAL_MV(" I", data.pid.Ki);
    SERIAL_MV(" D", data.pid.Kd);
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      if (type == IS_HOTEND) {
        SERIAL_MV(" C", data.pid.Kc);
        SERIAL_MV(" L", (int)tempManager.heater.lpq_len);
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
  SERIAL_MV(" P", data.sensor.pin);
  SERIAL_MV(" S", data.sensor.type);
  if (WITHIN(data.sensor.type, 1, 9)) {
    SERIAL_MV(" A", data.sensor.res_25, 1);
    SERIAL_MV(" B", data.sensor.beta, 1);
    SERIAL_MV(" C", data.sensor.shC, 10);
    SERIAL_MV(" R", data.sensor.pullup_res, 1);
    SERIAL_MV(" L", data.sensor.adc_low_offset);
    SERIAL_MV(" O", data.sensor.adc_high_offset);
  }
  SERIAL_EOL();
}

void Heater::print_M306() {
  const int8_t heater_id = type == IS_HOTEND ? data.ID : -type;
  SERIAL_SM(CFG, "Heater parameters: H<Heater>");
  if (heater_id < 0) SERIAL_MSG(" T<tools>");
  SERIAL_EM(" P<Pin> A<Power Drive Min> B<Power Drive Max> C<Power Max> F<Freq> L<Min Temp> O<Max Temp> U<Use Pid 0-1> I<Hardware Inverted 0-1> R<Thermal Protection 0-1> Q<Pwm Hardware 0-1>:");
  SERIAL_SMV(CFG, "  M306 H", (int)heater_id);
  if (heater_id < 0) SERIAL_MV(" T", int(data.ID));
  SERIAL_MV(" P", data.pin);
  SERIAL_MV(" A", data.pid.drive.min);
  SERIAL_MV(" B", data.pid.drive.max);
  SERIAL_MV(" C", data.pid.Max);
  SERIAL_MV(" F", data.freq);
  SERIAL_MV(" L", data.temp.min);
  SERIAL_MV(" O", data.temp.max);
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
    SERIAL_MV(" O", data.sensor.ad595_offset);
    SERIAL_MV(" S", data.sensor.ad595_gain);
    SERIAL_EOL();
  }
#endif

void Heater::start_idle_timer(const millis_l &ms) {
  idle_timeout_ms = millis() + ms;
  setIdle(false);
}

void Heater::reset_idle_timer() {
  idle_timeout_ms = 0;
  setIdle(false);
  start_watching();
}

void Heater::thermal_runaway_protection() {

  static long_timer_t thermal_runaway_timer(millis());

  switch (thermal_runaway_state) {

    // Inactive state waits for a target temperature to be set
    case TRInactive: break;

    // When first heating, wait for the temperature to be reached then go to Stable state
    case TRFirstHeating:
      if (current_temperature < target_temperature) break;
      thermal_runaway_state = TRStable;

    // While the temperature is stable watch for a bad temperature
    case TRStable:

      #if ENABLED(ADAPTIVE_FAN_SPEED) && HAS_FAN
        if (type == IS_HOTEND) {
          if (fans[0]->speed == 0)
            fans[0]->scaled_speed = 128;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.25f))
            fans[0]->scaled_speed = 128;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.40f))
            fans[0]->scaled_speed = 96;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.60f))
            fans[0]->scaled_speed = 64;
          else if (current_temperature >= target_temperature - (THERMAL_PROTECTION_HYSTERESIS * 0.90f))
            fans[0]->scaled_speed = 32;
          else
            fans[0]->scaled_speed = 0;
        }
      #endif

      if (current_temperature >= target_temperature - THERMAL_PROTECTION_HYSTERESIS) {
        thermal_runaway_timer.start();
        break;
      }
      else if (thermal_runaway_timer.pending((THERMAL_PROTECTION_PERIOD) * 1000)) break;
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
    next_watch_timer.start();
  }
  else
    next_watch_timer.stop();

}

/** Private Function */
// Temperature Error Handlers
void Heater::temp_error(PGM_P const serial_msg, PGM_P const lcd_msg) {
  if (isActive()) {
    SERIAL_STR(ER);
    SERIAL_STR(serial_msg);
    SERIAL_MSG(MSG_HOST_HEATER_STOPPED);
    switch (type) {
      #if HAS_HOTENDS
        case IS_HOTEND:
          SERIAL_EMV(MSG_HOST_HEATER_HOTEND " ", int(data.ID));
          break;
      #endif
      #if HAS_BEDS
        case IS_BED:
          SERIAL_EMV(MSG_HOST_HEATER_BED " ", int(data.ID));
          break;
      #endif
      #if HAS_CHAMBERS
        case IS_CHAMBER:
          SERIAL_EMV(MSG_HOST_HEATER_CHAMBER " ", int(data.ID));
          break;
      #endif
      #if HAS_TEMP_COOLER
        case IS_COOLER:
          SERIAL_EM(MSG_HOST_HEATER_COOLER);
          break;
      #endif
      default: break;
    }
  }

  sound.playtone(500, NOTE_G4);
  sound.playtone(500, NOTE_E5);
  sound.playtone(500, NOTE_G4);
  sound.playtone(500, NOTE_E5);

  lcdui.set_status_P(lcd_msg);
  setFault();

}

void Heater::min_temp_error() {
  switch (type) {
    #if HAS_HOTENDS
      case IS_HOTEND:
        temp_error(PSTR(MSG_HOST_T_MINTEMP), GET_TEXT(MSG_ERR_MINTEMP));
        break;
    #endif
    #if HAS_BEDS
      case IS_BED:
        temp_error(PSTR(MSG_HOST_T_MINTEMP), GET_TEXT(MSG_ERR_MINTEMP_BED));
        break;
    #endif
    #if HAS_CHAMBERS
      case IS_CHAMBER:
        temp_error(PSTR(MSG_HOST_T_MINTEMP), GET_TEXT(MSG_ERR_MINTEMP_CHAMBER));
        break;
    #endif
    default: break;
  }
}

void Heater::max_temp_error() {
  switch (type) {
    #if HAS_HOTENDS
      case IS_HOTEND:
        temp_error(PSTR(MSG_HOST_T_MAXTEMP), GET_TEXT(MSG_ERR_MAXTEMP));
        break;
    #endif
    #if HAS_BEDS
      case IS_BED:
        temp_error(PSTR(MSG_HOST_T_MAXTEMP), GET_TEXT(MSG_ERR_MAXTEMP_BED));
        break;
    #endif
    #if HAS_CHAMBERS
      case IS_CHAMBER:
        temp_error(PSTR(MSG_HOST_T_MAXTEMP), GET_TEXT(MSG_ERR_MAXTEMP_CHAMBER));
        break;
    #endif
    default: break;
  }
}

void Heater::print_low_high_temp(const int16_t celsius, const bool min_temp) {
  SERIAL_SMV(ER, " Temperature ", celsius);
  SERIAL_MSG(" too ");
  min_temp ? SERIAL_MSG("low") : SERIAL_MSG("high");
  SERIAL_MSG(" for heater");
  SERIAL_EOL();
}

void Heater::update_idle_timer() {
  if (!isIdle() && idle_timeout_ms && (ELAPSED(millis(), idle_timeout_ms)))
    setIdle(true);
}

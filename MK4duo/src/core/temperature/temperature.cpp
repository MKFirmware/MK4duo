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
 * temperature.cpp - temperature control
 */

#include "../../../MK4duo.h"

Temperature thermalManager;

/** Public Parameters */
#if HAS_MCU_TEMPERATURE
  float   Temperature::mcu_current_temperature  = 0.0,
          Temperature::mcu_highest_temperature  = 0.0,
          Temperature::mcu_lowest_temperature   = 4096.0,
          Temperature::mcu_alarm_temperature    = 80.0;
  int16_t Temperature::mcu_current_temperature_raw;
#endif

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  float Temperature::redundant_temperature = 0.0;
#endif

#if HAS_TEMP_HOTEND && ENABLED(PREVENT_COLD_EXTRUSION)
  int16_t Temperature::extrude_min_temp   = EXTRUDE_MINTEMP;
#endif

/** Private Parameters */
uint8_t Temperature::pid_pointer = 255;

#if ENABLED(FILAMENT_SENSOR)
  int8_t    Temperature::meas_shift_index;          // Index of a delayed sample in buffer
  uint16_t  Temperature::current_raw_filwidth = 0;  // Measured filament diameter - one extruder only
#endif

#if ENABLED(PROBING_HEATERS_OFF)
  bool Temperature::paused;
#endif

/** Public Function */
void Temperature::init() {

  HAL::analogStart();

  // Wait for temperature measurement to settle
  HAL::delayMilliseconds(250);

  #if ENABLED(PROBING_HEATERS_OFF)
    paused = false;
  #endif

  // Reset Fault for all Heaters
  #if ANALOG_INPUTS > 0
    LOOP_HEATER() heaters[h].ResetFault();
  #endif
}

void Temperature::set_current_temp_raw() {

  #if ANALOG_INPUTS > 0
    LOOP_HEATER() {
      if (WITHIN(heaters[h].sensor.pin, 0, 15))
        heaters[h].sensor.raw = HAL::AnalogInputValues[heaters[h].sensor.pin];
    }
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    powerManager.current_raw_powconsumption = HAL::AnalogInputValues[POWER_CONSUMPTION_PIN];
  #endif

  #if HAS_FILAMENT_SENSOR
    current_raw_filwidth = HAL::AnalogInputValues[FILWIDTH_PIN];
  #endif

}

/**
 * Spin Manage heating activities for heaters, bed, chamber and cooler
 *  - Is called every 100ms.
 *  - Acquire updated temperature readings
 *  - Also resets the watchdog timer
 *  - Invoke thermal runaway protection
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */
void Temperature::spin() {

  millis_t now = millis();

  #if ENABLED(EMERGENCY_PARSER)
    if (emergency_parser.killed_by_M112) printer.kill();
  #endif

  LOOP_HEATER() {

    Heater *act = &heaters[h];

    // Update Current Temperature
    act->updateCurrentTemperature();

    if (act->isActive() && act->current_temperature > act->data.maxtemp) max_temp_error(act->data.ID);
    if (act->isActive() && act->current_temperature < act->data.mintemp) min_temp_error(act->data.ID);

    // Check for thermal runaway
    if (act->isThermalProtection()) {
      act->thermal_runaway_protection();
      if (act->thermal_runaway_state == TRRunaway)
        _temp_error(h, PSTR(MSG_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
    }

    // Ignore heater we are currently testing
    if (pid_pointer == act->data.ID) continue;

    act->getOutput();

    // Make sure temperature is increasing
    if (act->isThermalProtection() && act->watch_next_ms && ELAPSED(now, act->watch_next_ms)) {
      if (act->current_temperature < act->watch_target_temp)
        _temp_error(h, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
      else
        act->start_watching(); // Start again if the target is still far off
    }

  } // LOOP_HEATER

  #if HAS_MCU_TEMPERATURE
    mcu_current_temperature = analog2tempMCU(mcu_current_temperature_raw);
    NOLESS(mcu_highest_temperature, mcu_current_temperature);
    NOMORE(mcu_lowest_temperature, mcu_current_temperature);
  #endif

  // Control the extruder rate based on the width sensor
  #if ENABLED(FILAMENT_SENSOR)

    filament_width_meas = analog2widthFil();

    if (filament_sensor) {
      meas_shift_index = filwidth_delay_index[0] - meas_delay_cm;
      if (meas_shift_index < 0) meas_shift_index += MAX_MEASUREMENT_DELAY + 1;  //loop around buffer if needed
      meas_shift_index = constrain(meas_shift_index, 0, MAX_MEASUREMENT_DELAY);

      // Convert the ratio value given by the filament width sensor
      // into a volumetric multiplier. Conversion differs when using
      // linear extrusion vs volumetric extrusion.
      const float nom_meas_ratio = 1.0 + 0.01f * measurement_delay[meas_shift_index],
                  ratio_2 = sq(nom_meas_ratio);

      tools.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM] = printer.isVolumetric()
        ? ratio_2 / CIRCLE_AREA(filament_width_nominal * 0.5f)  // Volumetric uses a true volumetric multiplier
        : ratio_2;                                              // Linear squares the ratio, which scales the volume

      tools.refresh_e_factor(FILAMENT_SENSOR_EXTRUDER_NUM);
    }

  #endif // FILAMENT_SENSOR
  
  #if HAS_POWER_CONSUMPTION_SENSOR

    static millis_t last_update = millis();
    millis_t temp_last_update = millis();
    millis_t from_last_update = temp_last_update - last_update;
    static float watt_overflow = 0.0;
    powerManager.consumption_meas = powerManager.analog2power();
    /*SERIAL_MV("raw:", powerManager.raw_analog2voltage(), 5);
    SERIAL_MV(" - V:", powerManager.analog2voltage(), 5);
    SERIAL_MV(" - I:", powerManager.analog2current(), 5);
    SERIAL_EMV(" - P:", powerManager.analog2power(), 5);*/
    watt_overflow += (powerManager.consumption_meas * from_last_update) / 3600000.0;
    if (watt_overflow >= 1.0) {
      print_job_counter.incConsumptionHour();
      watt_overflow--;
    }
    last_update = temp_last_update;

  #endif

  // Reset the watchdog after we know we have a temperature measurement.
  watchdog.reset();

}

/**
 * PID Autotuning (M303)
 *
 * Alternately heat and cool the nozzle, observing its behavior to
 * determine the best PID values to achieve a stable temperature.
 */
void Temperature::PID_autotune(Heater *act, const float target_temp, const uint8_t ncycles, const uint8_t method, const bool storeValues/*=false*/) {

  float       current_temp  = 0.0;
  int         cycles        = 0;
  bool        heating       = true;
  const bool  isHotend      = act->data.type == IS_HOTEND,
              oldReport     = printer.isAutoreportTemp();

  disable_all_heaters(); // switch off all heaters.

  millis_t  t1      = millis(),
            t2      = t1;
  int32_t   t_high  = 0.0,
            t_low   = 0.0;

  float     maxTemp = 0.0,
            minTemp = 1000.0;

  pid_data_t tune_pid;

  act->pwm_value = act->pid.Max;

  int32_t bias  = act->pid.Max >> 1,
          d     = act->pid.Max >> 1;

  printer.setWaitForHeatUp(true);
  printer.setAutoreportTemp(true);

  pid_pointer = act->data.ID;

  #if ENABLED(PRINTER_EVENT_LEDS)
    const float start_temp = act->current_temperature;
    LEDColor color = ledevents.onHeatingStart(isHotend);
  #endif

  // PID Tuning loop
  while (printer.isWaitForHeatUp()) {

    watchdog.reset(); // Reset the watchdog
    printer.idle();
    printer.keepalive(WaitHeater);

    act->updateCurrentTemperature();

    const millis_t time = millis();
    current_temp = act->current_temperature;
    NOLESS(maxTemp, current_temp);
    NOMORE(minTemp, current_temp);

    #if ENABLED(PRINTER_EVENT_LEDS)
      ledevents.onHeating(isHotend, start_temp, current_temp, target_temp);
    #endif

    if (heating && current_temp > target_temp) {
      if (ELAPSED(time, t2 + 5000UL)) {
        heating = false;

        act->pwm_value = (bias - d);

        t1 = time;
        t_high = t1 - t2;

        #if HAS_TEMP_COOLER
          if (act->data.type == IS_COOLER)
            minTemp = target_temp;
          else
        #endif
          maxTemp = target_temp;
      }
    }

    if (!heating && current_temp < target_temp) {
      if (ELAPSED(time, t1 + 5000UL)) {
        heating = true;
        t2 = time;
        t_low = t2 - t1;
        if (cycles > 0) {

          bias += (d * (t_high - t_low)) / (t_low + t_high);
          bias = constrain(bias, 20, act->pid.Max - 20);
          d = (bias > act->pid.Max >> 1) ? act->pid.Max - 1 - bias : bias;

          SERIAL_MV(MSG_BIAS, bias);
          SERIAL_MV(MSG_D, d);
          SERIAL_MV(MSG_T_MIN, minTemp);
          SERIAL_MV(MSG_T_MAX, maxTemp);

          if (cycles > 2) {
            float Ku = (4.0f * d) / (float(M_PI) * (maxTemp - minTemp) * 0.5f),
                  Tu = ((float)(t_low + t_high) * 0.001f);
            SERIAL_MV(MSG_KU, Ku);
            SERIAL_MV(MSG_TU, Tu);

            if (method == 0) {
              tune_pid.Kp = 0.6f * Ku;
              tune_pid.Ki = 1.2f * Ku / Tu;
              tune_pid.Kd = 0.075f * Ku * Tu;
              SERIAL_EM("\n" MSG_CLASSIC_PID);
            }
            else if (method == 1) {
              tune_pid.Kp = 0.33f * Ku;
              tune_pid.Ki = 0.66f * Ku / Tu;
              tune_pid.Kd = 0.11f * Ku * Tu;
              SERIAL_EM("\n" MSG_SOME_OVERSHOOT_PID);
            }
            else if (method == 2) {
              tune_pid.Kp = 0.2f * Ku;
              tune_pid.Ki = 0.4f * Ku / Tu;
              tune_pid.Kd = 0.2f * Ku * Tu / 3.0f;
              SERIAL_EM("\n" MSG_NO_OVERSHOOT_PID);
            }
            else if (method == 3) {
              tune_pid.Kp = 0.7f * Ku;
              tune_pid.Ki = 1.75f * Ku / Tu;
              tune_pid.Kd = 0.105f * Ku * Tu;
              SERIAL_EM("\n" MSG_PESSEN_PID);
            }
            else if (method == 4) {
              tune_pid.Kp = 0.4545f * Ku;
              tune_pid.Ki = 0.4545f * Ku / Tu / 2.2f;
              tune_pid.Kd = 0.4545f * Ku * Tu / 6.3f;
              SERIAL_EM("\n" MSG_TYREUS_LYBEN_PID);
            }
            SERIAL_MV(MSG_KP, tune_pid.Kp);
            SERIAL_MV(MSG_KI, tune_pid.Ki);
            SERIAL_MV(MSG_KD, tune_pid.Kd);
          }
        }

        SERIAL_EOL();

        act->pwm_value = (bias + d);

        cycles++;

        #if HAS_TEMP_COOLER
          if (act->data.type == IS_COOLER)
            maxTemp = target_temp;
          else
        #endif
          minTemp = target_temp;
      }
    }

    #if DISABLED(MAX_OVERSHOOT_PID_AUTOTUNE)
      #define MAX_OVERSHOOT_PID_AUTOTUNE 20
    #endif
    if (current_temp > target_temp + MAX_OVERSHOOT_PID_AUTOTUNE
      #if HAS_TEMP_COOLER
        && act->data.type != IS_COOLER
      #endif
    ) {
      SERIAL_LM(ER, MSG_PID_TEMP_TOO_HIGH);
      LCD_ALERTMESSAGEPGM(MSG_PID_TEMP_TOO_HIGH);
      pid_pointer = 255;
      break;
    }
    #if HAS_TEMP_COOLER
      else if (current_temp < target_temp + MAX_OVERSHOOT_PID_AUTOTUNE && act->data.type == IS_COOLER) {
        SERIAL_LM(ER, MSG_PID_TEMP_TOO_LOW);
        LCD_ALERTMESSAGEPGM(MSG_PID_TEMP_TOO_LOW);
        pid_pointer = 255;
        break;
      }
    #endif

    // Timeout after MAX_CYCLE_TIME_PID_AUTOTUNE minutes since the last undershoot/overshoot cycle
    #if DISABLED(MAX_CYCLE_TIME_PID_AUTOTUNE)
      #define MAX_CYCLE_TIME_PID_AUTOTUNE 20L
    #endif
    if (((time - t1) + (time - t2)) > (MAX_CYCLE_TIME_PID_AUTOTUNE * 60L * 1000L)) {
      SERIAL_LM(ER, MSG_PID_TIMEOUT);
      LCD_ALERTMESSAGEPGM(MSG_PID_TIMEOUT);
      pid_pointer = 255;
      break;
    }

    if (cycles > ncycles) {

      SERIAL_EM(MSG_PID_AUTOTUNE_FINISHED);
      pid_pointer = 255;

      if (isHotend) {
        SERIAL_MV(MSG_KP, tune_pid.Kp);
        SERIAL_MV(MSG_KI, tune_pid.Ki);
        SERIAL_EMV(MSG_KD, tune_pid.Kd);
      }

      #if HAS_TEMP_BED
        if (act->data.type == IS_BED) {
          SERIAL_EMV("#define DEFAULT_bedKp ", tune_pid.Kp);
          SERIAL_EMV("#define DEFAULT_bedKi ", tune_pid.Ki);
          SERIAL_EMV("#define DEFAULT_bedKd ", tune_pid.Kd);
        }
      #endif

      #if HAS_TEMP_CHAMBER
        if (act->data.type == IS_CHAMBER) {
          SERIAL_EMV("#define DEFAULT_chamberKp ", tune_pid.Kp);
          SERIAL_EMV("#define DEFAULT_chamberKi ", tune_pid.Ki);
          SERIAL_EMV("#define DEFAULT_chamberKd ", tune_pid.Kd);
        }
      #endif

      #if HAS_TEMP_COOLER
        if (act->data.type == IS_COOLER) {
          SERIAL_EMV("#define DEFAULT_coolerKp ", tune_pid.Kp);
          SERIAL_EMV("#define DEFAULT_coolerKi ", tune_pid.Ki);
          SERIAL_EMV("#define DEFAULT_coolerKd ", tune_pid.Kd);
        }
      #endif

      act->pid.Kp = tune_pid.Kp;
      act->pid.Ki = tune_pid.Ki;
      act->pid.Kd = tune_pid.Kd;
      act->pid.update();

      act->setTuning(true);
      act->ResetFault();

      if (storeValues) eeprom.store();

      #if ENABLED(PRINTER_EVENT_LEDS)
        ledevents.onPidTuningDone(color);
      #endif

      break;
    }

    lcdui.update();

  }

  disable_all_heaters();

  printer.setAutoreportTemp(oldReport);

  LCD_MESSAGEPGM(WELCOME_MSG);

}

/**
 * Switch off all heaters, set all target temperatures to 0
 */
void Temperature::disable_all_heaters() {

  #if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)
    planner.autotemp_enabled = false;
  #endif

  #if HEATER_COUNT > 0
    LOOP_HEATER() {
      heaters[h].setTarget(0);
      heaters[h].start_watching();
    }
  #endif

  #if ENABLED(LASER)
    // No laser firing with no coolers running! (paranoia)
    laser.extinguish();
  #endif

  // Unpause and reset everything
  #if HAS_BED_PROBE && ENABLED(PROBING_HEATERS_OFF)
    probe.probing_pause(false);
  #endif

  // If all heaters go down then for sure our print job has stopped
  print_job_counter.stop();

  pid_pointer = 255;

}

/**
 * Check if there are heaters Active
 */
bool Temperature::heaters_isActive() {
  #if HEATER_COUNT > 0
    LOOP_HEATER() if (heaters[h].isActive()) return true;
  #endif
  return false;
}

#if ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855)

  void Temperature::getTemperature_SPI() {
    LOOP_HEATER() {
      Heater *act = &heaters[h];
      #if ENABLED(SUPPORT_MAX31855)
        if (act->sensor.type == -4)
          act->sensor.raw = act->sensor.read_max31855();
      #endif
      #if ENABLED(SUPPORT_MAX6675)
        if (act->sensor.type == -3)
          act->sensor.raw = act->sensor.read_max6675();
      #endif
    }
  }

#endif

#if ENABLED(FILAMENT_SENSOR)

  // Convert raw Filament Width to millimeters
  float Temperature::analog2widthFil() {
    return current_raw_filwidth * (HAL_VOLTAGE_PIN) * (1.0 / 16383.0);
  }

  /**
   * Convert Filament Width (mm) to a simple ratio
   * and reduce to an 8 bit value.
   *
   * A nominal width of 1.75 and measured width of 1.73
   * gives (100 * 1.75 / 1.73) for a ratio of 101 and
   * a return value of 1.
   */
  int8_t Temperature::widthFil_to_size_ratio() {
    if (ABS(filament_width_nominal - filament_width_meas) <= FILWIDTH_ERROR_MARGIN)
      return int(100.0 * filament_width_nominal / filament_width_meas) - 100;
    return 0;
  }

#endif

#if ENABLED(PROBING_HEATERS_OFF)
  void Temperature::pause(const bool p) {
    if (p != paused) {
      paused = p;
      if (p)
        LOOP_HEATER() heaters[h].start_idle_timer(0); // timeout immediately
      else
        LOOP_HEATER() heaters[h].reset_idle_timer();
    }
  }
#endif // PROBING_HEATERS_OFF

void Temperature::report_temperatures(const bool showRaw/*=false*/) {

  #if HAS_TEMP_HOTEND
    print_heater_state(&heaters[ACTIVE_HOTEND], false, showRaw);
  #endif

  #if HAS_TEMP_BED
    print_heater_state(&heaters[BED_INDEX], false, showRaw);
    SERIAL_MV(MSG_BAT, (int)heaters[BED_INDEX].pwm_value);
  #endif

  #if HAS_TEMP_CHAMBER
    print_heater_state(&heaters[CHAMBER_INDEX], false, showRaw);
    SERIAL_MV(MSG_CAT, (int)heaters[CHAMBER_INDEX].pwm_value);
  #endif

  #if HAS_TEMP_COOLER
    print_heater_state(&heaters[COOLER_INDEX], false, showRaw);
  #endif

  #if HAS_TEMP_HOTEND
    SERIAL_MV(" @:", (int)heaters[ACTIVE_HOTEND].pwm_value);
  #endif

  #if HOTENDS > 1
    LOOP_HOTEND() {
      print_heater_state(&heaters[h], true, showRaw);
      SERIAL_MV(MSG_AT, h);
      SERIAL_CHR(':');
      SERIAL_VAL((int)heaters[h].pwm_value);
    }
  #endif

  #if HAS_MCU_TEMPERATURE
    SERIAL_MV(" MCU min:", mcu_lowest_temperature, 2);
    SERIAL_MV(", current:", mcu_current_temperature, 2);
    SERIAL_MV(", max:", mcu_highest_temperature, 2);
    if (showRaw)
      SERIAL_MV(" C->", mcu_current_temperature_raw);
  #endif

  #if ENABLED(DHT_SENSOR)
    SERIAL_MV(" DHT Temp:", dhtsensor.Temperature, 0);
    SERIAL_MV(", Humidity:", dhtsensor.Humidity, 0);
  #endif

}

// Private function

#if HAS_MCU_TEMPERATURE
  float Temperature::analog2tempMCU(const int raw) {
    const float voltage = (float)raw * ((HAL_VOLTAGE_PIN) / (float)16384);
    return (voltage - 0.8) * (1000.0 / 2.65) + 27.0; // + mcuTemperatureAdjust;			// accuracy at 27C is +/-45C
  }
#endif

// Temperature Error Handlers
void Temperature::_temp_error(const uint8_t h, PGM_P const serial_msg, PGM_P const lcd_msg) {
  if (heaters[h].isActive()) {
    SERIAL_STR(ER);
    SERIAL_PGM(serial_msg);
    SERIAL_MSG(MSG_STOPPED_HEATER);
    switch (heaters[h].data.type) {
      case IS_HOTEND:
        SERIAL_EV((int)h);
        break;
      #if HAS_TEMP_BED
        case IS_BED:
          SERIAL_EM(MSG_HEATER_BED);
          break;
      #endif
      #if HAS_TEMP_CHAMBER
        case IS_CHAMBER:
          SERIAL_EM(MSG_HEATER_CHAMBER);
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
  heaters[h].setFault();

}
void Temperature::min_temp_error(const uint8_t h) {
  switch (heaters[h].data.type) {
    case IS_HOTEND:
      _temp_error(h, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP));
      break;
    #if HAS_TEMP_BED
      case IS_BED:
        _temp_error(h, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP_BED));
        break;
    #endif
    #if HAS_TEMP_CHAMBER
      case IS_CHAMBER:
        _temp_error(h, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP_CHAMBER));
        break;
    #endif
    default: break;
  }
}
void Temperature::max_temp_error(const uint8_t h) {
  switch (heaters[h].data.type) {
    case IS_HOTEND:
      _temp_error(h, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP));
      break;
    #if HAS_TEMP_BED
      case IS_BED:
        _temp_error(h, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP_BED));
        break;
    #endif
    #if HAS_TEMP_CHAMBER
      case IS_CHAMBER:
        _temp_error(h, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP_CHAMBER));
        break;
    #endif
    default: break;
  }
}

void Temperature::print_heater_state(Heater *act, const bool print_ID, const bool showRaw) {

  SERIAL_CHR(' ');

  #if HAS_TEMP_HOTEND
    if (act->data.type == IS_HOTEND) {
      SERIAL_CHR('T');
      #if HOTENDS > 1
        if (print_ID) SERIAL_VAL(act->data.ID);
      #else
        UNUSED(print_ID);
      #endif
    }
  #endif

  #if HAS_TEMP_BED
    if (act->data.type == IS_BED) SERIAL_CHR('B');
  #endif

  #if HAS_TEMP_CHAMBER
    if (act->data.type == IS_CHAMBER) SERIAL_CHR('C');
  #endif

  #if HAS_TEMP_COOLER
    if (act->data.type == IS_COOLER) SERIAL_CHR('C');
  #endif

  const int16_t targetTemperature = act->isIdle() ? act->idle_temperature : act->target_temperature;
  SERIAL_CHR(':');
  SERIAL_VAL(act->current_temperature, 2);
  SERIAL_MV(" /" , targetTemperature);
  if (showRaw) {
    SERIAL_MV(" (", act->sensor.raw);
    SERIAL_CHR(')');
  }

}

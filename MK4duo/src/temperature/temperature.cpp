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

#include "../../base.h"

Temperature thermalManager;

constexpr uint8_t   temp_residency_time[HEATER_TYPE]  = { TEMP_RESIDENCY_TIME, TEMP_BED_RESIDENCY_TIME, TEMP_CHAMBER_RESIDENCY_TIME, TEMP_COOLER_RESIDENCY_TIME },
                    temp_hysteresis[HEATER_TYPE]      = { TEMP_HYSTERESIS, TEMP_BED_HYSTERESIS, TEMP_CHAMBER_HYSTERESIS, TEMP_COOLER_HYSTERESIS },
                    temp_window[HEATER_TYPE]          = { TEMP_WINDOW, TEMP_BED_WINDOW, TEMP_CHAMBER_WINDOW, TEMP_COOLER_WINDOW };
constexpr uint16_t  temp_check_interval[HEATER_TYPE]  = { 0, BED_CHECK_INTERVAL, CHAMBER_CHECK_INTERVAL, COOLER_CHECK_INTERVAL };
constexpr bool      thermal_protection[HEATER_TYPE]   = { THERMAL_PROTECTION_HOTENDS, THERMAL_PROTECTION_BED, THERMAL_PROTECTION_CHAMBER, THERMAL_PROTECTION_COOLER };

#if HOTENDS > 0
  static void*    heater_ttbl_map[HOTENDS]    = ARRAY_BY_HOTENDS_N((void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE, (void*)HEATER_2_TEMPTABLE, (void*)HEATER_3_TEMPTABLE);
  static uint8_t  heater_ttbllen_map[HOTENDS] = ARRAY_BY_HOTENDS_N(HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN, HEATER_3_TEMPTABLE_LEN);
#endif

// public:
volatile bool Temperature::wait_for_heatup = true;

#if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
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
  bool    Temperature::allow_cold_extrude = false;
  int16_t Temperature::extrude_min_temp   = EXTRUDE_MINTEMP;
#endif

#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
  uint8_t   Temperature::auto_report_temp_interval  = 0;
  millis_t  Temperature::next_temp_report_ms        = 0;
#endif

// private:

float Temperature::temp_iState[HEATER_COUNT]     = { 0.0 },
      Temperature::temp_dState[HEATER_COUNT][4]  = { 0.0 },
      Temperature::temp_iState_min[HEATER_COUNT] = { 0.0 },
      Temperature::temp_iState_max[HEATER_COUNT] = { 0.0 },
      Temperature::pid_error[HEATER_COUNT];

#if ENABLED(PID_ADD_EXTRUSION_RATE)
  float Temperature::cTerm[HOTENDS];
  long  Temperature::last_e_position,
        Temperature::lpq[LPQ_MAX_LEN];
  int   Temperature::lpq_ptr = 0,
        Temperature::lpq_len = 20;
#endif

uint8_t Temperature::pid_pointer[HEATER_COUNT] = { 0 };

millis_t Temperature::next_check_ms[HEATER_COUNT];

#if ENABLED(FILAMENT_SENSOR)
  int8_t    Temperature::meas_shift_index;          // Index of a delayed sample in buffer
  uint16_t  Temperature::current_raw_filwidth = 0;  // Measured filament diameter - one extruder only
#endif

#if HAS_AUTO_FAN
  millis_t Temperature::next_auto_fan_check_ms = 0;
#endif

#if ENABLED(PROBING_HEATERS_OFF)
  bool Temperature::paused;
#endif

#if HEATER_IDLE_HANDLER
  millis_t Temperature::heater_idle_timeout_ms[HEATER_COUNT] = { 0 };
  bool Temperature::heater_idle_timeout_exceeded[HEATER_COUNT] = { false };
#endif

// Public Function

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_temp_controller()
 */
void Temperature::init() {

  #if MB(RUMBA) && ((TEMP_SENSOR_0==-1)||(TEMP_SENSOR_1==-1)||(TEMP_SENSOR_2==-1)||(TEMP_SENSOR_BED==-1)||(TEMP_SENSOR_CHAMBER==-1)||(TEMP_SENSOR_COOLER==-1))
    // disable RUMBA JTAG in case the thermocouple extension is plugged on top of JTAG connector
    MCUCR = _BV(JTD);
    MCUCR = _BV(JTD);
  #endif

  #if (PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
    last_e_position = 0;
  #endif

  HAL::analogStart();

  // Use timer for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  HAL_TEMP_TIMER_START();
  ENABLE_TEMP_INTERRUPT();

  // Wait for temperature measurement to settle
  HAL::delayMilliseconds(250);

  #if ENABLED(PROBING_HEATERS_OFF)
    paused = false;
  #endif
}

void Temperature::wait_heater(const uint8_t h, bool no_wait_for_cooling/*=true*/) {

  millis_t residency_start_ms = 0;

  float target_temp = -1.0, old_temp = 9999.0;
  bool  wants_to_cool   = false,
        temp_conditions = false;
  millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

  wait_for_heatup = true;

  #if DISABLED(BUSY_WHILE_HEATING)
    KEEPALIVE_STATE(NOT_BUSY);
  #endif

  #if ENABLED(PRINTER_EVENT_LEDS)
    const float start_temp = heaters[h].current_temperature;
    uint8_t old_blue = 0;
    uint8_t old_red  = 255;
  #endif

  do {
    // Target temperature might be changed during the loop
    if (target_temp != heaters[h].target_temperature)
      target_temp = heaters[h].target_temperature;

    wants_to_cool = heaters[h].isCooling();

    // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
    if (no_wait_for_cooling && wants_to_cool) break;

    now = millis();
    if (ELAPSED(now, next_temp_ms)) { // Print temp & remaining time every 1s while waiting
      next_temp_ms = now + 1000UL;
      print_heaterstates();
      if (temp_residency_time[heaters[h].type] > 0) {
        SERIAL_MSG(MSG_W);
        if (residency_start_ms)
          SERIAL_VAL(long(((temp_residency_time[heaters[h].type] * 1000UL) - (now - residency_start_ms)) / 1000UL));
        else
          SERIAL_CHR("?");
      }
      SERIAL_EOL();
    }

    printer.idle();
    commands.refresh_cmd_timeout(); // to prevent stepper.stepper_inactive_time from running out

    const float temp = heaters[h].current_temperature;

    #if ENABLED(PRINTER_EVENT_LEDS)
      if (!wants_to_cool) {
        if (h < HOTENDS) {
          // Gradually change LED strip from violet to red as nozzle heats up
          const uint8_t blue = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 255, 0);
          if (blue != old_blue) {
            old_blue = blue;
            printer.set_led_color(255, 0, blue
              #if ENABLED(NEOPIXEL_RGBW_LED)
                , 0
              #endif
              #if HAS_NEOPIXEL
                , true
              #endif
            );
          }
        }
        else if (h == BED_INDEX) {
          // Gradually change LED strip from blue to violet as bed heats up
          const uint8_t red = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 0, 255);
          if (red != old_red) {
            old_red = red;
            printer.set_led_color(red, 0, 255
              #if ENABLED(NEOPIXEL_RGBW_LED)
                , 0
              #endif
              #if HAS_NEOPIXEL
                , true
              #endif
            );
          }
        }
      }
    #endif

    if (temp_residency_time[heaters[h].type] > 0) {

      float temp_diff = FABS(target_temp - temp);

      if (!residency_start_ms) {
        // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
        if (temp_diff < temp_window[heaters[h].type]) residency_start_ms = now;
      }
      else if (temp_diff > temp_hysteresis[heaters[h].type]) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = now;
      }
    }

    // Prevent a wait-forever situation if R is misused i.e. M109 R0
    if (wants_to_cool) {
      // Break after 60 seconds
      // if the temperature did not drop at least 1.5
      if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
        if (old_temp - temp < 1.5) break;
        next_cool_check_ms = now + 60000UL;
        old_temp = temp;
      }
    }

    if (temp_residency_time[heaters[h].type] > 0)
      temp_conditions = (!residency_start_ms || PENDING(now, residency_start_ms + temp_residency_time[heaters[h].type] * 1000UL));
    else
      temp_conditions = (wants_to_cool ? heaters[h].isCooling() : heaters[h].isHeating());

  } while (wait_for_heatup && temp_conditions);

  if (wait_for_heatup) {
    LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
    #if ENABLED(PRINTER_EVENT_LEDS)
      #if ENABLED(RGBW_LED) || ENABLED(NEOPIXEL_RGBW_LED)
        printer.set_led_color(0, 0, 0, 255);  // Turn on the WHITE LED
      #else
        printer.set_led_color(255, 255, 255); // Set LEDs All On
      #endif
    #endif
  }

  #if DISABLED(BUSY_WHILE_HEATING)
    KEEPALIVE_STATE(IN_HANDLER);
  #endif
}

void Temperature::set_current_temp_raw() {

  LOOP_HEATER() heaters[h].current_temperature_raw = HAL::AnalogInputValues[heaters[h].sensor_pin];

  #if HAS_POWER_CONSUMPTION_SENSOR
    powerManager.current_raw_powconsumption = HAL::AnalogInputValues[POWER_CONSUMPTION_PIN];
  #endif

  #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
    mcu_current_temperature_raw = HAL::AnalogInputValues[ADC_TEMPERATURE_SENSOR];
  #endif

  #if HAS_FILAMENT_SENSOR
    current_raw_filwidth = HAL::AnalogInputValues[FILWIDTH_PIN];
  #endif

}

/**
 * Manage heating activities for heaters, bed, chamber and cooler
 *  - Is called every 100ms.
 *  - Acquire updated temperature readings
 *  - Also resets the watchdog timer
 *  - Invoke thermal runaway protection
 *  - Manage extruder auto-fan
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */
void Temperature::manage_temp_controller() {

  updateTemperaturesFromRawValues(); // also resets the watchdog

  millis_t ms = millis();

  LOOP_HEATER() {

    Heater *act = &heaters[h];

    if (act->isON() && act->current_temperature > act->maxtemp) max_temp_error(h);
    if (act->isON() && act->current_temperature < act->mintemp) min_temp_error(h);

    #if HEATER_IDLE_HANDLER
      if (!heater_idle_timeout_exceeded[h] && heater_idle_timeout_ms[h] && ELAPSED(ms, heater_idle_timeout_ms[h]))
        heater_idle_timeout_exceeded[h] = true;
    #endif

    // Check for thermal runaway
    #if HAS_THERMALLY_PROTECTED_HEATER
      if (thermal_protection[act->type])
        thermal_runaway_protection(&thermal_runaway_state_machine[h], &thermal_runaway_timer[h], act->current_temperature, act->target_temperature, h, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
    #endif

    if (act->use_pid)
      act->soft_pwm = act->tempisrange() ? (int)get_pid_output(h) : 0;
    else if (ELAPSED(ms, next_check_ms[h])) {
      next_check_ms[h] = ms + temp_check_interval[act->type];
      if (act->tempisrange())
        act->soft_pwm = act->isHeating() ? act->pid_max : 0;
      else {
        act->soft_pwm = 0;
        HAL::digitalWrite(act->output_pin, act->hardwareInverted ? HIGH : LOW);
      }
    }

    #if WATCH_THE_HEATER
      // Make sure temperature is increasing
      if (act->watch_next_ms && ELAPSED(ms, act->watch_next_ms)) {
        if (act->current_temperature < act->watch_target_temp)
          _temp_error(h, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
        else
          start_watching(act); // Start again if the target is still far off
      }
    #endif

  } // LOOP_HEATER

  #if HAS_AUTO_FAN
    if (ELAPSED(ms, next_auto_fan_check_ms)) { // only need to check fan state very infrequently
      checkExtruderAutoFans();
      next_auto_fan_check_ms = ms + 2500UL;
    }
  #endif

  // Control the extruder rate based on the width sensor
  #if ENABLED(FILAMENT_SENSOR)
    if (filament_sensor) {
      meas_shift_index = filwidth_delay_index[0] - meas_delay_cm;
      if (meas_shift_index < 0) meas_shift_index += MAX_MEASUREMENT_DELAY + 1;  //loop around buffer if needed
      meas_shift_index = constrain(meas_shift_index, 0, MAX_MEASUREMENT_DELAY);

      // Get the delayed info and add 100 to reconstitute to a percent of
      // the nominal filament diameter then square it to get an area
      const float vmroot = measurement_delay[meas_shift_index] * 0.01 + 1.0;
      tools.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM] = vmroot <= 0.1 ? 0.01 : sq(vmroot);
    }
  #endif // FILAMENT_SENSOR
}

void Temperature::PID_autotune(int8_t temp_controller, const float temp, int ncycles, bool storeValues/*=false*/) {

    if (!WITHIN(temp_controller, 0, HOTENDS)
      #if HAS_TEMP_BED
        && temp_controller != -1
      #elif HAS_TEMP_CHAMBER
        && temp_controller != -2
      #elif HAS_TEMP_COOLER
        && temp_controller != -3 
      #endif
    ) {
      SERIAL_LM(ER, MSG_PID_BAD_TEMP_CONTROLLER_NUM);
      return;
    }

    float currentTemp = 0.0;
    int cycles = 0;
    bool heating = true;

    millis_t temp_ms = millis(), t1 = temp_ms, t2 = temp_ms;
    int32_t t_high = 0, t_low = 0;

    int32_t bias, d;
    float Ku, Tu;
    float workKp = 0, workKi = 0, workKd = 0;
    float maxTemp = 20, minTemp = 20;

    NOLESS(ncycles, 5);
    NOMORE(ncycles, 20);

    #if HAS_AUTO_FAN
      next_auto_fan_check_ms = temp_ms + 2500UL;
    #endif

    SERIAL_EM(MSG_PID_AUTOTUNE_START);

    if (temp_controller >= 0) {
      SERIAL_MV("Hotend: ", temp_controller);
    }
    #if HAS_TEMP_BED
      else if (temp_controller == -1) {
        temp_controller = BED_INDEX;
        SERIAL_MSG("BED");
      }
    #endif
    #if HAS_TEMP_CHAMBER
      else if(temp_controller == -2) {
        temp_controller = CHAMBER_INDEX;
        SERIAL_MSG("CHAMBER");
      }
    #endif
    #if HAS_TEMP_COOLER
      else if(temp_controller == -3) {
        temp_controller = COOLER_INDEX;
        SERIAL_MSG("COOLER");
      }
    #endif
    SERIAL_MV(" Temp: ", temp);
    SERIAL_MV(" Cycles: ", ncycles);
    if (storeValues)
      SERIAL_EM(" Apply result");
    else
      SERIAL_EOL();

    disable_all_heaters(); // switch off all heaters.

    const uint8_t pidMax = heaters[temp_controller].pid_max;
    heaters[temp_controller].soft_pwm = pidMax;

    bias = pidMax >> 1;
    d = pidMax >> 1;

    wait_for_heatup = true;

    // PID Tuning loop
    while (wait_for_heatup) {

      updateTemperaturesFromRawValues();
      millis_t ms = millis();

      currentTemp = heaters[temp_controller].current_temperature;

      NOLESS(maxTemp, currentTemp);
      NOMORE(minTemp, currentTemp);

      #if HAS_AUTO_FAN
        if (ELAPSED(ms, next_auto_fan_check_ms)) {
          checkExtruderAutoFans();
          next_auto_fan_check_ms = ms + 2500UL;
        }
      #endif

      if (heating && currentTemp > temp) {
        if (ELAPSED(ms, t2 + 2500UL)) {
          heating = false;

          heaters[temp_controller].soft_pwm = (bias - d);

          t1 = ms;
          t_high = t1 - t2;

          #if HAS_TEMP_COOLER
            if (temp_controller == COOLER_INDEX)
              minTemp = temp;
            else
          #endif
            maxTemp = temp;
        }
      }

      if (!heating && currentTemp < temp) {
        if (ELAPSED(ms, t1 + 5000UL)) {
          heating = true;
          t2 = ms;
          t_low = t2 - t1;
          if (cycles > 0) {

            bias += (d * (t_high - t_low)) / (t_low + t_high);
            bias = constrain(bias, 20, pidMax - 20);
            d = (bias > pidMax / 2) ? pidMax - 1 - bias : bias;

            SERIAL_MV(MSG_BIAS, bias);
            SERIAL_MV(MSG_D, d);
            SERIAL_MV(MSG_T_MIN, minTemp);
            SERIAL_MV(MSG_T_MAX, maxTemp);
            if (cycles > 2) {
              Ku = (4.0 * d) / (M_PI * (maxTemp - minTemp));
              Tu = ((float)(t_low + t_high) * 0.001);
              SERIAL_MV(MSG_KU, Ku);
              SERIAL_EMV(MSG_TU, Tu);
              workKp = 0.6 * Ku;
              workKi = 2 * workKp / Tu;
              workKd = workKp * Tu * 0.125;
              
              SERIAL_EM(MSG_CLASSIC_PID);
              SERIAL_MV(MSG_KP, workKp);
              SERIAL_MV(MSG_KI, workKi);
              SERIAL_EMV(MSG_KD, workKd);
            }
          }

          heaters[temp_controller].soft_pwm = (bias + d);

          cycles++;

          #if HAS_TEMP_COOLER
            if (temp_controller == COOLER_INDEX)
              maxTemp = temp;
            else
          #endif
            minTemp = temp;
        }
      }

      #define MAX_OVERSHOOT_PID_AUTOTUNE 40
      if (currentTemp > temp + MAX_OVERSHOOT_PID_AUTOTUNE
        #if HAS_TEMP_COOLER
          && temp_controller != COOLER_INDEX
        #endif
      ) {
        SERIAL_LM(ER, MSG_PID_TEMP_TOO_HIGH);
        return;
      }
      #if HAS_TEMP_COOLER
        else if (currentTemp < temp + MAX_OVERSHOOT_PID_AUTOTUNE && temp_controller == COOLER_INDEX) {
          SERIAL_LM(ER, MSG_PID_TEMP_TOO_LOW);
          return;
        }
      #endif

      // Every 1 seconds...
      if (ELAPSED(ms, temp_ms + 1000UL)) {
        print_heaterstates();
        SERIAL_EOL();
        temp_ms = ms;
      }

      // Over 2 minutes?
      if (((ms - t1) + (ms - t2)) > (10L * 60L * 1000L * 2L)) {
        SERIAL_EM(MSG_PID_TIMEOUT);
        return;
      }

      if (cycles > ncycles) {

        SERIAL_EM(MSG_PID_AUTOTUNE_FINISHED);

        #if (PIDTEMP)
          if (temp_controller >= 0) {
            SERIAL_MV(MSG_KP, workKp);
            SERIAL_MV(MSG_KI, workKi);
            SERIAL_EMV(MSG_KD, workKd);
          }
        #endif

        #if (PIDTEMPBED)
          if (temp_controller == BED_INDEX) {
            SERIAL_EMV("#define DEFAULT_bedKp ", workKp);
            SERIAL_EMV("#define DEFAULT_bedKi ", workKi);
            SERIAL_EMV("#define DEFAULT_bedKd ", workKd);
          }
        #endif

        #if (PIDTEMPCHAMBER)
          if (temp_controller == -2) {
            SERIAL_EMV("#define DEFAULT_chamberKp ", workKp);
            SERIAL_EMV("#define DEFAULT_chamberKi ", workKi);
            SERIAL_EMV("#define DEFAULT_chamberKd ", workKd);
          }
        #endif

        #if (PIDTEMPCOOLER)
          if (temp_controller == -3) {
            SERIAL_EMV("#define DEFAULT_coolerKp ", workKp);
            SERIAL_EMV("#define DEFAULT_coolerKi ", workKi);
            SERIAL_EMV("#define DEFAULT_coolerKd ", workKd);
          }
        #endif

        if (storeValues) {
          heaters[temp_controller].Kp = workKp;
          heaters[temp_controller].Ki = workKi;
          heaters[temp_controller].Kd = workKd;
          updatePID();
        }

        return;
      }
    }

    disable_all_heaters();
  }

void Temperature::updatePID() {

  LOOP_HEATER() {
    if (heaters[h].use_pid) {
      temp_iState_min[h] = (float)heaters[h].pid_min * 10.0f / heaters[h].Ki;
      temp_iState_max[h] = (float)heaters[h].pid_max * 10.0f / heaters[h].Ki;
    }
  }
  
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    last_e_position = 0;
  #endif
}

void Temperature::disable_all_heaters() {

  #if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)
    planner.autotemp_enabled = false;
  #endif

  #if HEATER_COUNT > 0
    LOOP_HEATER() {
      heaters[h].target_temperature = 0;
      heaters[h].soft_pwm = 0;
    }
  #endif

  #if ENABLED(LASER)
    // No laser firing with no coolers running! (paranoia)
    laser.extinguish();
  #endif

  // Unpause and reset everything
  #if ENABLED(PROBING_HEATERS_OFF)
    probe.probing_pause(false);
  #endif

  // If all heaters go down then for sure our print job has stopped
  print_job_counter.stop();
}

#if WATCH_THE_HEATER
  /**
   * Start Heating Sanity Check for heaters that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set.
   */
  void Temperature::start_watching(Heater *act) {
    if (act->isON() && act->current_temperature < act->target_temperature - (WATCH_TEMP_INCREASE + temp_hysteresis[act->type] + 1)) {
      act->watch_target_temp = act->current_temperature + WATCH_TEMP_INCREASE;
      act->watch_next_ms = millis() + (WATCH_TEMP_PERIOD) * 1000UL;
    }
    else
      act->watch_next_ms = 0;
  }
#endif

#if ENABLED(FILAMENT_SENSOR)
  // Convert raw Filament Width to a ratio
  int Temperature::widthFil_to_size_ratio() {
    float temp = filament_width_meas;
    if (temp < MEASURED_LOWER_LIMIT) temp = filament_width_nominal;  // assume sensor cut out
    else NOMORE(temp, MEASURED_UPPER_LIMIT);
    return filament_width_nominal / temp * 100;
  }
#endif

#if ENABLED(PROBING_HEATERS_OFF)
  void Temperature::pause(const bool p) {
    if (p != paused) {
      paused = p;
      if (p)
        LOOP_HEATER() start_heater_idle_timer(h, 0); // timeout immediately
      else
        LOOP_HEATER() reset_heater_idle_timer(h);
    }
  }
#endif // PROBING_HEATERS_OFF

#if ENABLED(AUTO_REPORT_TEMPERATURES)
  void Temperature::auto_report_temperatures() {
    if (auto_report_temp_interval && ELAPSED(millis(), next_temp_report_ms)) {
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
      print_heaterstates();
      SERIAL_EOL();
    }
  }
#endif // AUTO_REPORT_TEMPERATURES

void Temperature::print_heaterstates() {

  #if HAS_TEMP_HOTEND
    print_heater_state(heaters[TRG_EXTRUDER_IDX].current_temperature, heaters[TRG_EXTRUDER_IDX].target_temperature,
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        heaters[TRG_EXTRUDER_IDX].current_temperature_raw,
      #endif
      -1
    );
  #endif

  #if HAS_TEMP_BED
    print_heater_state(heaters[BED_INDEX].current_temperature, heaters[BED_INDEX].target_temperature,
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        heaters[BED_INDEX].current_temperature_raw,
      #endif
      BED_INDEX
    );
  #endif

  #if HAS_TEMP_CHAMBER
    print_heater_state(heaters[CHAMBER_INDEX].current_temperature, heaters[CHAMBER_INDEX].target_temperature,
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        heaters[CHAMBER_INDEX].current_temperature_raw,
      #endif
      CHAMBER_INDEX
    );
  #endif

  #if HAS_TEMP_COOLER
    print_heater_state(heaters[COOLER_INDEX].current_temperature, heaters[COOLER_INDEX].target_temperature,
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        heaters[COOLER_INDEX].current_temperature_raw,
      #endif
      COOLER_INDEX
    );
  #endif

  #if HOTENDS > 1
    LOOP_HOTEND() print_heater_state(heaters[h].current_temperature, heaters[h].target_temperature,
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        heaters[h].current_temperature_raw,
      #endif
      h
    );
  #endif

  SERIAL_MV(MSG_AT ":", (int)heaters[TRG_EXTRUDER_IDX].soft_pwm);

  #if HAS_TEMP_BED
    SERIAL_MV(MSG_BAT, (int)heaters[BED_INDEX].soft_pwm);
  #endif

  #if HAS_TEMP_CHAMBER
    SERIAL_MV(MSG_CAT, (int)heaters[CHAMBER_INDEX].soft_pwm);
  #endif
  
  #if HOTENDS > 1
    LOOP_HOTEND() {
      SERIAL_MV(MSG_AT, h);
      SERIAL_CHR(':');
      SERIAL_VAL((int)heaters[h].soft_pwm);
    }
  #endif

  #if ENABLED(ARDUINO_ARCH_SAM)&& !MB(RADDS)
    SERIAL_MV(" MCU min:", mcu_lowest_temperature, 1);
    SERIAL_MV(", current:", mcu_current_temperature, 1);
    SERIAL_MV(", max:", mcu_highest_temperature, 1);
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV(" C->", mcu_current_temperature_raw);
    #endif
  #endif
}

// Private function
/**
 * Get the raw values into the actual temperatures.
 * The raw values are created in interrupt context,
 * and this function is called from normal context
 * as it would block the stepper routine.
 */
void Temperature::updateTemperaturesFromRawValues() {

  LOOP_HEATER() heaters[h].current_temperature = analog2temp(h);

  #if ENABLED(FILAMENT_SENSOR)
    filament_width_meas = analog2widthFil();
  #endif

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
      powerManager.consumption_hour++;
      watt_overflow--;
    }
    last_update = temp_last_update;
  #endif

  #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
    mcu_current_temperature = analog2tempMCU(mcu_current_temperature_raw);
    NOLESS(mcu_highest_temperature, mcu_current_temperature);
    NOMORE(mcu_lowest_temperature, mcu_current_temperature);
  #endif

  #if ENABLED(USE_WATCHDOG)
    // Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();
  #endif

}

float Temperature::analog2temp(const uint8_t h) {

  #define PGM_RD_W(x)   (short)pgm_read_word(&x)

  int16_t type  = heaters[h].sensor_type;
  int16_t raw   = heaters[h].current_temperature_raw;

  #if ENABLED(SUPPORT_MAX31855)
    if (type == -3)
      return read_max31855(heaters[h].sensor_pin);
  #endif
  #if ENABLED(SUPPORT_MAX6675)
    if (type == -2)
      return read_max6675(heaters[h].sensor_pin, h);
  #endif
  #if HEATER_USES_AD595
    if (type == -1)
      return ((raw * (((HAL_VOLTAGE_PIN) * 100.0) / 1024.0)) * heaters[h].ad595_gain) + heaters[h].ad595_offset;
  #endif
  if (type > 0) {

    float celsius = 0;
    uint8_t i;

    #if HOTENDS > 0 
      if (h < HOTENDS && heater_ttbl_map[h] != NULL) {
        short(*tt)[][2] = (short(*)[][2])(heater_ttbl_map[h]);

        for (i = 1; i < heater_ttbllen_map[h]; i++) {
          if (PGM_RD_W((*tt)[i][0]) > raw) {
            celsius = PGM_RD_W((*tt)[i - 1][1]) +
                      (raw - PGM_RD_W((*tt)[i - 1][0])) *
                      (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i - 1][1])) /
                      (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i - 1][0]));
            break;
          }
        }

        // Overflow: Set to last value in the table
        if (i == heater_ttbllen_map[h]) celsius = PGM_RD_W((*tt)[i - 1][1]);

        return celsius;
      }
    #endif

    #if HAS_TEMP_BED
      if (h == BED_INDEX) {
        for (i = 1; i < BEDTEMPTABLE_LEN; i++) {
          if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw) {
            celsius = PGM_RD_W(BEDTEMPTABLE[i - 1][1]) +
                      (raw - PGM_RD_W(BEDTEMPTABLE[i - 1][0])) *
                      (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i - 1][1])) /
                      (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i - 1][0]));
            break;
          }
        }

        // Overflow: Set to last value in the table
        if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i - 1][1]);

        return celsius;
      }
    #endif

    #if HAS_TEMP_CHAMBER
      if (h == CHAMBER_INDEX) {
        for (i = 1; i < CHAMBERTEMPTABLE_LEN; i++) {
          if (PGM_RD_W(CHAMBERTEMPTABLE[i][0]) > raw) {
            celsius = PGM_RD_W(CHAMBERTEMPTABLE[i - 1][1]) +
                      (raw - PGM_RD_W(CHAMBERTEMPTABLE[i - 1][0])) *
                      (float)(PGM_RD_W(CHAMBERTEMPTABLE[i][1]) - PGM_RD_W(CHAMBERTEMPTABLE[i - 1][1])) /
                      (float)(PGM_RD_W(CHAMBERTEMPTABLE[i][0]) - PGM_RD_W(CHAMBERTEMPTABLE[i - 1][0]));
            break;
          }
        }

        // Overflow: Set to last value in the table
        if (i == CHAMBERTEMPTABLE_LEN) celsius = PGM_RD_W(CHAMBERTEMPTABLE[i - 1][1]);

        return celsius;
      }
    #endif

    #if HAS_TEMP_COOLER
      if (h == COOLER_INDEX) {
        for (i = 1; i < COOLERTEMPTABLE_LEN; i++) {
          if (PGM_RD_W(COOLERTEMPTABLE[i][0]) > raw) {
            celsius = PGM_RD_W(COOLERTEMPTABLE[i - 1][1]) +
                      (raw - PGM_RD_W(COOLERTEMPTABLE[i - 1][0])) *
                      (float)(PGM_RD_W(COOLERTEMPTABLE[i][1]) - PGM_RD_W(COOLERTEMPTABLE[i - 1][1])) /
                      (float)(PGM_RD_W(COOLERTEMPTABLE[i][0]) - PGM_RD_W(COOLERTEMPTABLE[i - 1][0]));
            break;
          }
        }

        // Overflow: Set to last value in the table
        if (i == COOLERTEMPTABLE_LEN) celsius = PGM_RD_W(COOLERTEMPTABLE[i - 1][1]);

        return celsius;
      }
    #endif
  }

  return 25;
}

#if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
  float Temperature::analog2tempMCU(const int raw) {
    float mcutemp = (float)raw * (3.3 / 4096.0);
    return (mcutemp - 0.8) * (1000.0 / 2.65) + 27.0; // + mcuTemperatureAdjust;			// accuracy at 27C is +/-45C
  }
#endif

#if ENABLED(FILAMENT_SENSOR)
  // Convert raw Filament Width to millimeters
  float Temperature::analog2widthFil() {
    return current_raw_filwidth * 5.0 * (1.0 / 16383.0);
    //return current_raw_filwidth;
  }
#endif

uint8_t Temperature::get_pid_output(const int8_t h) {

  uint8_t pid_output = 0;

  temp_dState[h][pid_pointer[h]++] = heaters[h].current_temperature;
  pid_pointer[h] &= 3;
  float error = heaters[h].target_temperature - heaters[h].current_temperature;
  #if HEATER_IDLE_HANDLER
    if (heater_idle_timeout_exceeded[h]) {
      pid_output = 0;
      temp_iState[h] = 0;
    }
    else
  #endif
  if (error > PID_FUNCTIONAL_RANGE) {
    pid_output = heaters[h].pid_max;
  }
  else if (error < -(PID_FUNCTIONAL_RANGE) || heaters[h].target_temperature == 0 
    #if HEATER_IDLE_HANDLER
      || heater_idle_timeout_exceeded[h]
    #endif
  ) {
    pid_output = 0;
  }
  else {
    float pidTerm = heaters[h].Kp * error;
    temp_iState[h] = constrain(temp_iState[h] + error, temp_iState_min[h], temp_iState_max[h]);
    pidTerm += heaters[h].Ki * temp_iState[h] * 0.1; // 0.1 = 10Hz
    float dgain = heaters[h].Kd * (temp_dState[h][pid_pointer[h]] - heaters[h].current_temperature) * 3.333f;
    pidTerm += dgain;

    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      cTerm[h] = 0;
      if (h == tools.active_extruder) {
        long e_position = stepper.position(E_AXIS);
        if (e_position > last_e_position) {
          lpq[lpq_ptr] = e_position - last_e_position;
          last_e_position = e_position;
        }
        else {
          lpq[lpq_ptr] = 0;
        }
        if (++lpq_ptr >= lpq_len) lpq_ptr = 0;
        cTerm[h] = (lpq[lpq_ptr] * mechanics.steps_to_mm[E_AXIS]) * heaters[h].Kc;
        pidTerm += cTerm[h];
      }
    #endif // PID_ADD_EXTRUSION_RATE

    pid_output = constrain((int)pidTerm, 0, PID_MAX);

  }

  #if ENABLED(PID_DEBUG)
    SERIAL_SMV(ECHO, MSG_PID_DEBUG, HOTEND_INDEX);
    SERIAL_MV(MSG_PID_DEBUG_INPUT, heaters[h].current_temperature);
    SERIAL_EMV(MSG_PID_DEBUG_OUTPUT, pid_output);
  #endif // PID_DEBUG

  return pid_output;
}

#if ENABLED(SUPPORT_MAX6675)

  #define MAX6675_ERROR_MASK 4
  #define MAX6675_DISCARD_BITS 3

  int16_t Temperature::read_max6675(const Pin cs_pin, const int8_t h) {

    static millis_t last_max6675_read[HOTENDS]  = ARRAY_BY_HOTENDS(0);
    static int16_t  max6675_temp[HOTENDS]       = ARRAY_BY_HOTENDS(2000);

    if (HAL::timeInMilliseconds() - last_max6675_read[h] > 230) {

      HAL::spiBegin();
      HAL::spiInit(2);

      HAL::digitalWrite(cs_pin, LOW); // enable TT_MAX6675

      // ensure 100ns delay - a bit extra is fine
      #if ENABLED(ARDUINO_ARCH_SAM)
        HAL::delayMicroseconds(1);
      #else
        asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
        asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
      #endif

      max6675_temp[h] = HAL::spiReceive(0);
      max6675_temp[h] <<= 8;
      max6675_temp[h] |= HAL::spiReceive(0);

      HAL::digitalWrite(cs_pin, HIGH); // disable TT_MAX6675
      last_max6675_read[h] = millis();
    }

    if (max6675_temp[h] & MAX6675_ERROR_MASK) {
      SERIAL_LM(ER, "MAX6675 Temp measurement error!");
      max6675_temp[h] = 2000; // thermocouple open
    }
    else {
      max6675_temp[h] >> MAX6675_DISCARD_BITS;
    }

    return max6675_temp[h];
  }

#endif //HEATER_0_USES_MAX6675

#if ENABLED(SUPPORT_MAX31855)

  #define MAX31855_DISCARD_BITS 18

  int16_t Temperature::read_max31855(const Pin cs_pin) {

    uint32_t data = 0;
    int16_t temperature;

    HAL::spiBegin();
    HAL::spiInit(2);

    HAL::digitalWrite(cs_pin, LOW); // enable TT_MAX31855

    // ensure 100ns delay - a bit extra is fine
    #if ENABLED(ARDUINO_ARCH_SAM)
      HAL::delayMicroseconds(1);
    #else
      asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
      asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
    #endif

    for (uint16_t byte = 0; byte < 4; byte++) {
      data <<= 8;
      data |= HAL::spiReceive();
    }

    HAL::digitalWrite(cs_pin, 1); // disable TT_MAX31855

    // Process temp
    if (data & 0x00010000)
      return 20000; // Some form of error.
    else {
      data = data >> MAX31855_DISCARD_BITS;
      temperature = data & 0x00001FFF;

      if (data & 0x00002000) {
        data = ~data;
        temperature = -1 * ((data & 0x00001FFF) + 1);
      }
    }

    return temperature;
  }

#endif

#if HAS_AUTO_FAN

  void Temperature::checkExtruderAutoFans() {
    const int fanBit[] = {
                    0,
      AUTO_1_IS_0 ? 0 :               1,
      AUTO_2_IS_0 ? 0 : AUTO_2_IS_1 ? 1 :               2,
      AUTO_3_IS_0 ? 0 : AUTO_3_IS_1 ? 1 : AUTO_3_IS_2 ? 2 : 3
    };
    uint8_t fanState = 0;
 
    LOOP_HOTEND() {
      if (heaters[h].current_temperature > HOTEND_AUTO_FAN_TEMPERATURE)
        SBI(fanState, fanBit[h]);
    }
 
    uint8_t fanDone = 0;
    uint8_t f = 0;
    for (uint8_t fan = AUTO_FAN0_INDEX; fan < (AUTO_FAN0_INDEX + AUTO_FAN_COUNT); fan++) {
      if (!TEST(fanDone, fanBit[f])) {
        fans[fan].Speed = TEST(fanState, fanBit[f]) ? HOTEND_AUTO_FAN_SPEED : HOTEND_AUTO_FAN_MIN_SPEED;
        SBI(fanDone, fanBit[f]);
        f++;
      }
    }
  }

#endif // HAS_AUTO_FAN

// Temperature Error Handlers
void Temperature::_temp_error(const int8_t tc, const char * const serial_msg, const char * const lcd_msg) {
  static bool killed = false;
  if (printer.IsRunning()) {
    SERIAL_ST(ER, serial_msg);
    SERIAL_MSG(MSG_STOPPED_HEATER);
    if (tc >= 0)
      SERIAL_EV((int)tc);
    #if HAS_TEMP_BED
      else if (tc == -1)
        SERIAL_EM(MSG_HEATER_BED);
    #endif
    #if HAS_TEMP_CHAMBER
      else if (tc == -2)
        SERIAL_EM(MSG_HEATER_CHAMBER);
    #endif
    #if HAS_TEMP_COOLER
      else if (tc == -3)
        SERIAL_EM(MSG_HEATER_COOLER);
    #endif
  }

  #if DISABLED(BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE)
    if (!killed) {
      printer.setRunning(false);
      killed = true;
      printer.kill(lcd_msg);
    }
    else
      disable_all_heaters();
  #endif
}
void Temperature::min_temp_error(const int8_t h) {
  #if HAS_TEMP_BED
    _temp_error(h, PSTR(MSG_T_MINTEMP), h >= 0 ? PSTR(MSG_ERR_MINTEMP) : PSTR(MSG_ERR_MINTEMP_BED));
  #else
    _temp_error(HOTEND_INDEX, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP));
    #if HOTENDS <= 1
      UNUSED(h);
    #endif
  #endif
}
void Temperature::max_temp_error(const int8_t h) {
  #if HAS_TEMP_BED
    _temp_error(h, PSTR(MSG_T_MAXTEMP), h >= 0 ? PSTR(MSG_ERR_MAXTEMP) : PSTR(MSG_ERR_MAXTEMP_BED));
  #else
    _temp_error(HOTEND_INDEX, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP));
    #if HOTENDS <= 1
      UNUSED(h);
    #endif
  #endif
}

#if HAS_THERMALLY_PROTECTED_HEATER

  Temperature::TRState Temperature::thermal_runaway_state_machine[HEATER_COUNT] = { TRInactive };
  millis_t Temperature::thermal_runaway_timer[HEATER_COUNT] = { 0 };

  void Temperature::thermal_runaway_protection(Temperature::TRState* state, millis_t* timer, float temperature, float target_temperature, uint8_t temp_controller_id, int period_seconds, int hysteresis_degc) {

    static float tr_target_temperature[HEATER_COUNT] = { 0.0 };

    /*
        SERIAL_MSG("Thermal Thermal Runaway Running. Heater ID: ");
        if (temp_controller_id < 0) SERIAL_MSG("bed"); else SERIAL_VAL(temp_controller_id);
        SERIAL_MV(" ;  State:", *state);
        SERIAL_MV(" ;  Timer:", *timer);
        SERIAL_MV(" ;  Temperature:", temperature);
        SERIAL_EMV(" ;  Target Temp:", target_temperature);
    */
   
    #if HEATER_IDLE_HANDLER
      // If the heater idle timeout expires, restart
      if (heater_idle_timeout_exceeded[temp_controller_id]) {
        *state = TRInactive;
        tr_target_temperature[temp_controller_id] = 0;
      }
      else
    #endif
    // If the target temperature changes, restart
    if (tr_target_temperature[temp_controller_id] != target_temperature) {
      tr_target_temperature[temp_controller_id] = target_temperature;
      *state = target_temperature > 0 ? TRFirstHeating : TRInactive;
    }

    switch (*state) {
      // Inactive state waits for a target temperature to be set
      case TRInactive: break;
      // When first heating, wait for the temperature to be reached then go to Stable state
      case TRFirstHeating:
        if (temperature < tr_target_temperature[temp_controller_id]) break;
        *state = TRStable;
      // While the temperature is stable watch for a bad temperature
      case TRStable:
        if (temperature >= tr_target_temperature[temp_controller_id] - hysteresis_degc) {
          *timer = millis() + period_seconds * 1000UL;
          break;
        }
        else if (PENDING(millis(), *timer)) break;
        *state = TRRunaway;
      case TRRunaway:
        _temp_error(temp_controller_id, PSTR(MSG_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
    }
  }

#endif // THERMAL_PROTECTION_HOTENDS || THERMAL_PROTECTION_BED

void Temperature::print_heater_state(const float &c, const int16_t &t,
  #if ENABLED(SHOW_TEMP_ADC_VALUES)
    const int16_t r,
  #endif
  const int8_t h
) {
  SERIAL_CHR(' ');

  #if HAS_TEMP_HOTEND
    if (h < HOTENDS) {
      SERIAL_CHR('T');
      #if HOTENDS > 1
        if (h >= 0) SERIAL_VAL((int)h);
      #endif
    }
  #endif

  #if HAS_TEMP_BED
    if (h == BED_INDEX) SERIAL_CHR('B');
  #endif

  #if HAS_TEMP_CHAMBER
    if (h == CHAMBER_INDEX) SERIAL_CHR('C');
  #endif

  #if HAS_TEMP_COOLER
    if (h == COOLER_INDEX) SERIAL_CHR('C');
  #endif

  SERIAL_CHR(':');
  SERIAL_VAL(c, 1);
  SERIAL_MV(" /" , t);
  #if ENABLED(SHOW_TEMP_ADC_VALUES)
    SERIAL_MV(" (", r);
    SERIAL_CHR(')');
  #endif
}

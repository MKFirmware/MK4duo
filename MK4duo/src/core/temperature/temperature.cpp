/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
#include "sanitycheck.h"

Temperature thermalManager;

/** Public Parameters */
heater_max_t Temperature::data;

#if HAS_MCU_TEMPERATURE
  float   Temperature::mcu_current_temperature  = 0.0f,
          Temperature::mcu_highest_temperature  = 0.0f,
          Temperature::mcu_alarm_temperature    = 80.0f;
  int16_t Temperature::mcu_current_temperature_raw;
#endif

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  float Temperature::redundant_temperature = 0.0;
#endif

#if HAS_TEMP_HOTEND && ENABLED(PREVENT_COLD_EXTRUSION)
  int16_t Temperature::extrude_min_temp   = EXTRUDE_MINTEMP;
#endif

/** Private Parameters */

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  int8_t    Temperature::meas_shift_index;          // Index of a delayed sample in buffer
  uint16_t  Temperature::current_raw_filwidth = 0;  // Measured filament diameter - one extruder only
#endif

#if ENABLED(PROBING_HEATERS_OFF)
  bool Temperature::paused;
#endif

/** Public Function */
void Temperature::create_object() {

  #if MAX_HOTEND > 0
    LOOP_HOTEND() {
      if (!hotends[h]) {
        hotends[h] = new Heater(IS_HOTEND, HOTEND_CHECK_INTERVAL, HOTEND_HYSTERESIS, WATCH_HOTEND_PERIOD, WATCH_HOTEND_INCREASE);
        hotends_factory_parameters(h);
        SERIAL_LMV(ECHO, "Create H", int(h));
        hotends[h]->init();
      }
    }
  #endif

  #if MAX_BED > 0
    LOOP_BED() {
      if (!beds[h]) {
        beds[h] = new Heater(IS_BED, BED_CHECK_INTERVAL, BED_HYSTERESIS, WATCH_BED_PERIOD, WATCH_BED_INCREASE);
        beds_factory_parameters(h);
        SERIAL_LMV(ECHO, "Create Bed", int(h));
        beds[h]->init();
      }
    }
  #endif

  #if MAX_CHAMBER > 0
    LOOP_CHAMBER() {
      if (!chambers[h]) {
        chambers[h] = new Heater(IS_CHAMBER, CHAMBER_CHECK_INTERVAL, CHAMBER_HYSTERESIS, WATCH_CHAMBER_PERIOD, WATCH_CHAMBER_INCREASE);
        chambers_factory_parameters(h);
        SERIAL_LMV(ECHO, "Create Chamber", int(h));
        chambers[h]->init();
      }
    }
  #endif

  #if MAX_COOLER > 0
    LOOP_COOLER() {
      if (!coolers[h]) {
        coolers[h] = new Heater(IS_COOLER, COOLER_CHECK_INTERVAL, COOLER_HYSTERESIS, WATCH_COOLER_PERIOD, WATCH_COOLER_INCREASE);
        coolers_factory_parameters(h);
        SERIAL_LMV(ECHO, "Create Cooler", int(h));
        coolers[h]->init();
      }
    }
  #endif

  #if MAX_FAN > 0
    LOOP_FAN() {
      if (!fans[f]) {
        fans[f] = new Fan();
        fans_factory_parameters(f);
        fans[f]->init();
      }
    }
  #endif

}

void Temperature::init() {

  HAL::analogStart();

  // Wait for temperature measurement to settle
  HAL::delayMilliseconds(250);

  #if ENABLED(PROBING_HEATERS_OFF)
    paused = false;
  #endif

  // Reset Fault for all Heaters
  #if MAX_HOTEND > 0
    LOOP_HOTEND() hotends[h]->ResetFault();
  #endif
  #if MAX_BED > 0
    LOOP_BED() beds[h]->ResetFault();
  #endif
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER() chambers[h]->ResetFault();
  #endif
  #if MAX_COOLER > 0
    LOOP_COOLER() coolers[h]->ResetFault();
  #endif
}

void Temperature::factory_parameters() {

  data.hotends  = HOTENDS;
  data.beds     = BEDS;
  data.chambers = CHAMBERS;
  data.coolers  = COOLERS;
  data.fans     = FAN_COUNT;

  create_object();

  #if MAX_HOTEND > 0
    LOOP_HOTEND()   if (hotends[h])   hotends_factory_parameters(h);
  #endif
  #if MAX_BED > 0
    LOOP_BED()      if (beds[h])      beds_factory_parameters(h);
  #endif
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER()  if (chambers[h])  chambers_factory_parameters(h);
  #endif
  #if MAX_COOLER > 0
    LOOP_COOLER()   if (coolers[h])   coolers_factory_parameters(h);
  #endif
  #if MAX_FAN > 0
    LOOP_FAN()      if (fans[f])      fans_factory_parameters(f);
  #endif

}

void Temperature::change_number_heater(const HeatertypeEnum type, const uint8_t h) {

  if (type == IS_HOTEND) {
    if (data.hotends < h) {
      data.hotends = h;
      create_object();
    }
    else if (data.hotends > h) {
      for (uint8_t hh = h; hh < MAX_HOTEND; hh++) {
        Heater * tmpdriver = nullptr;
        swap(tmpdriver, hotends[hh]);
        delete(tmpdriver);
      }
      data.hotends = h;
    }
  }
  else if (type == IS_BED) {
    if (data.beds < h) {
      data.beds = h;
      create_object();
    }
    else if (data.beds > h) {
      for (uint8_t hh = h; hh < MAX_BED; hh++) {
        Heater * tmpdriver = nullptr;
        swap(tmpdriver, beds[hh]);
        delete(tmpdriver);
      }
      data.beds = h;
    }
  }
  else if (type == IS_CHAMBER) {
    if (data.chambers < h) {
      data.chambers = h;
      create_object();
    }
    else if (data.chambers > h) {
      for (uint8_t hh = h; hh < MAX_CHAMBER; hh++) {
        Heater * tmpdriver = nullptr;
        swap(tmpdriver, chambers[hh]);
        delete(tmpdriver);
      }
      data.chambers = h;
    }
  }

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

  #if ENABLED(EMERGENCY_PARSER)
    if (emergency_parser.killed_by_M112) printer.kill(PSTR("M112"));
  #endif

  #if MAX_HOTEND > 0
    LOOP_HOTEND() {
      // Update Current Temperature
      hotends[h]->update_current_temperature();
      hotends[h]->check_and_power();
    }
  #endif

  #if MAX_BED > 0
    LOOP_BED() {
      // Update Current Temperature
      beds[h]->update_current_temperature();
      beds[h]->check_and_power();
    } // LOOP_BED
  #endif

  #if MAX_CHAMBER > 0
    LOOP_CHAMBER() {
      // Update Current Temperature
      chambers[h]->update_current_temperature();
      chambers[h]->check_and_power();
    } // LOOP_CHAMBER
  #endif

  #if MAX_COOLER > 0
    LOOP_COOLER() {
      // Update Current Temperature
      coolers[h]->update_current_temperature();
      coolers[h]->check_and_power();
    } // LOOP_COOLER
  #endif

  #if HAS_MCU_TEMPERATURE
    mcu_current_temperature = analog2tempMCU(mcu_current_temperature_raw);
    NOLESS(mcu_highest_temperature, mcu_current_temperature);
  #endif

  // Control the extruder rate based on the width sensor
  #if ENABLED(FILAMENT_WIDTH_SENSOR)

    filament_width_meas = analog2widthFil();

    if (filament_sensor) {
      meas_shift_index = filwidth_delay_index[0] - meas_delay_cm;
      if (meas_shift_index < 0) meas_shift_index += MAX_MEASUREMENT_DELAY + 1;  //loop around buffer if needed
      LIMIT(meas_shift_index, 0, MAX_MEASUREMENT_DELAY);

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

  #endif // FILAMENT_WIDTH_SENSOR
  
  #if HAS_POWER_CONSUMPTION_SENSOR

    static millis_l last_update = millis();
    millis_l temp_last_update = millis();
    millis_l from_last_update = temp_last_update - last_update;
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
 * Switch off all heaters, set all target temperatures to 0
 */
void Temperature::disable_all_heaters() {

  #if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)
    planner.autotemp_enabled = false;
  #endif

  #if MAX_HOTEND > 0
    LOOP_HOTEND() {
      hotends[h]->set_target_temp(0);
      hotends[h]->start_watching();
    }
  #endif
  #if MAX_BED > 0
    LOOP_BED() {
      beds[h]->set_target_temp(0);
      beds[h]->start_watching();
    }
  #endif
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER() {
      chambers[h]->set_target_temp(0);
      chambers[h]->start_watching();
    }
  #endif
  #if MAX_COOLER > 0
    LOOP_COOLER() {
      coolers[h]->set_target_temp(0);
      coolers[h]->start_watching();
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

}

/**
 * Check if there are heaters Active
 */
bool Temperature::heaters_isActive() {
  #if MAX_HOTEND > 0
    LOOP_HOTEND()   if (hotends[h]->isActive())   return true;
  #endif
  #if MAX_BED > 0
    LOOP_BED()      if (beds[h]->isActive())      return true;
  #endif
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER()  if (chambers[h]->isActive())  return true;
  #endif
  #if MAX_COOLER > 0
    LOOP_COOLER()   if (coolers[h]->isActive())   return true;
  #endif
  #if MAX_FAN > 0
    LOOP_FAN()      if (fans[f]->speed > 0)       return true;
  #endif
  return false;
}


/**
 * Calc min & max temp of all heaters
 */
#define MAX_TEMP_RANGE 10

#if MAX_HOTEND > 0

  /**
   * Calc min & max temp of all hotends
   */
  int16_t Temperature::hotend_mintemp_all() {
    int16_t mintemp = 9999;
    LOOP_HOTEND() mintemp = MIN(mintemp, hotends[h]->data.temp.min);
    return mintemp;
  }

  int16_t Temperature::hotend_maxtemp_all() {
    int16_t maxtemp = 0;
    LOOP_HOTEND() maxtemp = MAX(maxtemp, hotends[h]->data.temp.max);
    return maxtemp - MAX_TEMP_RANGE;
  }

#endif

#if MAX_BED > 0

  int16_t Temperature::bed_mintemp_all() {
    int16_t mintemp = 9999;
    LOOP_BED() mintemp = MIN(mintemp, beds[h]->data.temp.min);
    return mintemp;
  }

  int16_t Temperature::bed_maxtemp_all() {
    int16_t maxtemp = 0;
    LOOP_HOTEND() maxtemp = MAX(maxtemp, beds[h]->data.temp.max);
    return maxtemp - MAX_TEMP_RANGE;
  }

#endif

#if MAX_CHAMBER > 0

  int16_t Temperature::chamber_mintemp_all() {
    int16_t mintemp = 9999;
    LOOP_CHAMBER() mintemp = MIN(mintemp, chambers[h]->data.temp.min);
    return mintemp;
  }

  int16_t Temperature::chamber_maxtemp_all() {
    int16_t maxtemp = 0;
    LOOP_CHAMBER() maxtemp = MAX(maxtemp, chambers[h]->data.temp.max);
    return maxtemp - MAX_TEMP_RANGE;
  }

#endif

#if MAX_COOLER > 0

  int16_t Temperature::cooler_mintemp_all() {
    int16_t mintemp = 9999;
    LOOP_COOLER() mintemp = MIN(mintemp, coolers[h]->data.temp.min);
    return mintemp;
  }

  int16_t Temperature::cooler_maxtemp_all() {
    int16_t maxtemp = 0;
    LOOP_COOLER() maxtemp = MAX(maxtemp, coolers[h]->data.temp.max);
    return maxtemp - MAX_TEMP_RANGE;
  }

#endif

#if HAS_MAX31855 || HAS_MAX6675

  void Temperature::getTemperature_SPI() {

    #if MAX_HOTEND > 0
      LOOP_HOTEND() {
        if (false) {}
        #if HAS_MAX31855
          else if (hotends[h]->data.sensor.type == -4)
            hotends[h]->data.sensor.raw = hotends[h]->data.sensor.read_max31855();
        #endif
        #if HAS_MAX6675
          else if (hotends[h]->data.sensor.type == -3)
            hotends[h]->data.sensor.raw = hotends[h]->data.sensor.read_max6675();
        #endif
      }
    #endif
    #if MAX_BED > 0
      LOOP_BED() {
        if (false) {}
        #if HAS_MAX31855
          else if (beds[h]->data.sensor.type == -4)
            beds[h]->data.sensor.raw = beds[h]->data.sensor.read_max31855();
        #endif
        #if HAS_MAX6675
          else if (beds[h]->data.sensor.type == -3)
            beds[h]->data.sensor.raw = beds[h]->data.sensor.read_max6675();
        #endif
      }
    #endif
    #if MAX_CHAMBER > 0
      LOOP_CHAMBER() {
        if (false) {}
        #if HAS_MAX31855
          else if (chambers[h]->data.sensor.type == -4)
            chambers[h]->data.sensor.raw = chambers[h]->data.sensor.read_max31855();
        #endif
        #if HAS_MAX6675
          else if (chambers[h]->data.sensor.type == -3)
            chambers[h]->data.sensor.raw = chambers[h]->data.sensor.read_max6675();
        #endif
      }
    #endif

  }

#endif // HAS_MAX31855 || HAS_MAX6675

#if ENABLED(FILAMENT_WIDTH_SENSOR)

  // Convert raw Filament Width to millimeters
  float Temperature::analog2widthFil() {
    return current_raw_filwidth * (HAL_VOLTAGE_PIN) * (1.0f / float(AD_RANGE));
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
        LOOP_HOTEND() hotends[h]->start_idle_timer(0); // timeout immediately
      else
        LOOP_HOTEND() hotends[h]->reset_idle_timer();
    }
  }
#endif // PROBING_HEATERS_OFF

void Temperature::report_temperatures(const bool showRaw/*=false*/) {

  #if MAX_HOTEND > 0
    if (thermalManager.data.hotends > 0) {
      print_heater_state(hotends[tools.active_hotend()], false, showRaw);
      SERIAL_MV(MSG_AT ":", hotends[tools.active_hotend()]->pwm_value);
    }
  #endif

  #if MAX_BED > 0
    if (thermalManager.data.beds > 0) {
      print_heater_state(beds[0], false, showRaw);
      SERIAL_MV(MSG_BAT ":", beds[0]->pwm_value);
    }
  #endif

  #if MAX_CHAMBER > 0
    if (thermalManager.data.chambers > 0) {
      print_heater_state(chambers[0], false, showRaw);
      SERIAL_MV(MSG_CAT ":", chambers[0]->pwm_value);
    }
  #endif

  #if MAX_COOLER > 0
    if (thermalManager.data.coolers > 0) {
      print_heater_state(coolers[0], false, showRaw);
      SERIAL_MV(MSG_CAT ":", coolers[0]->pwm_value);
    }
  #endif

  #if MAX_HOTEND > 1
    if (thermalManager.data.hotends > 1) {
      LOOP_HOTEND() {
        print_heater_state(hotends[h], true, showRaw);
        SERIAL_MV(MSG_AT, int(h));
        SERIAL_CHR(':');
        SERIAL_VAL(hotends[h]->pwm_value);
      }
    }
  #endif

  #if MAX_BED > 1
    if (thermalManager.data.beds > 1) {
      LOOP_BED() {
        print_heater_state(beds[h], true, showRaw);
        SERIAL_MV(MSG_BAT, int(h));
        SERIAL_CHR(':');
        SERIAL_VAL(beds[h]->pwm_value);
      }
    }
  #endif

  #if MAX_CHAMBER > 1
    if (thermalManager.data.chambers > 1) {
      LOOP_CHAMBER() {
        print_heater_state(chambers[h], true, showRaw);
        SERIAL_MV(MSG_CAT, int(h));
        SERIAL_CHR(':');
        SERIAL_VAL(chambers[h]->pwm_value);
      }
    }
  #endif

  #if HAS_MCU_TEMPERATURE
    SERIAL_MV(" MCU:", mcu_current_temperature, 2);
    SERIAL_MV(" max:", mcu_highest_temperature, 2);
    if (showRaw) {
      SERIAL_MV(" (", mcu_current_temperature_raw);
      SERIAL_CHR(')');
    }
  #endif

  #if HAS_DHT
    SERIAL_MV(" DHT Temp:", dhtsensor.Temperature, 1);
    SERIAL_MV(" Humidity:", dhtsensor.Humidity, 1);
  #endif

}

/** Private Function */
#if MAX_HOTEND > 0
  void Temperature::hotends_factory_parameters(const uint8_t h) {

    Heater        *heat;
    pid_data_t    *pid;
    sensor_data_t *sens;

    constexpr float   HEKp[]    = HOTEND_Kp,
                      HEKi[]    = HOTEND_Ki,
                      HEKd[]    = HOTEND_Kd,
                      HEKc[]    = HOTEND_Kc;
    constexpr pin_t   HE_pin[]  = { HEATER_HE0_PIN, HEATER_HE1_PIN, HEATER_HE2_PIN, HEATER_HE3_PIN, HEATER_HE4_PIN, HEATER_HE5_PIN },
                      SE_pin[]  = { TEMP_HE0_PIN, TEMP_HE1_PIN, TEMP_HE2_PIN, TEMP_HE3_PIN, TEMP_HE4_PIN, TEMP_HE5_PIN };
    constexpr int16_t HE_min[]  = { HOTEND_0_MINTEMP, HOTEND_1_MINTEMP, HOTEND_2_MINTEMP, HOTEND_3_MINTEMP, HOTEND_4_MINTEMP, HOTEND_5_MINTEMP },
                      HE_max[]  = { HOTEND_0_MAXTEMP, HOTEND_1_MAXTEMP, HOTEND_2_MAXTEMP, HOTEND_3_MAXTEMP, HOTEND_4_MAXTEMP, HOTEND_5_MAXTEMP },
                      SE_type[] = { TEMP_SENSOR_HE0, TEMP_SENSOR_HE1, TEMP_SENSOR_HE2, TEMP_SENSOR_HE3, TEMP_SENSOR_HE4, TEMP_SENSOR_HE5 };

    heat                  = hotends[h];
    sens                  = &heat->data.sensor;
    pid                   = &heat->data.pid;
    heat->data.pin        = HE_pin[h];
    heat->data.ID         = h;
    heat->data.temp.min   = HE_min[h];
    heat->data.temp.max   = HE_max[h];
    heat->data.freq       = HOTEND_PWM_FREQUENCY;
    // Pid
    pid->Kp               = HEKp[ALIM(h, HEKp)];
    pid->Ki               = HEKi[ALIM(h, HEKi)];
    pid->Kd               = HEKd[ALIM(h, HEKd)];
    pid->Kc               = HEKc[ALIM(h, HEKc)];
    pid->drive.min        = POWER_DRIVE_MIN;
    pid->drive.max        = POWER_DRIVE_MAX;
    pid->Max              = POWER_MAX;
    // Sensor
    sens->pin             = SE_pin[h];
    sens->type            = SE_type[h];
    sens->res_25          = HOT0_R25;
    sens->beta            = HOT0_BETA;
    sens->pullup_res      = THERMISTOR_SERIES_RS;
    sens->shC             = 0;
    sens->adcLowOffset    = 0;
    sens->adcHighOffset   = 0;
    #if HAS_AD8495 || HAS_AD595
      sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
      sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
    #endif
    heat->resetFlag();
    heat->setUsePid(PIDTEMP);
    heat->setHWinvert(INVERTED_HEATER_PINS);
    heat->setHWpwm(USEABLE_HARDWARE_PWM(heat->data.pin));
    heat->setThermalProtection(THERMAL_PROTECTION_HOTENDS);
    #if HAS_EEPROM
      heat->setPidTuned(false);
    #else
      heat->setPidTuned(true);
    #endif

  }
#endif

#if MAX_BED > 0
  void Temperature::beds_factory_parameters(const uint8_t h) {

    Heater        *heat;
    pid_data_t    *pid;
    sensor_data_t *sens;

    constexpr float   BEDKp[]   = BED_Kp,
                      BEDKi[]   = BED_Ki,
                      BEDKd[]   = BED_Kd;
    constexpr pin_t   BE_pin[]  = { HEATER_BED0_PIN, HEATER_BED1_PIN, HEATER_BED2_PIN, HEATER_BED3_PIN },
                      SB_pin[]  = { TEMP_BED0_PIN, TEMP_BED1_PIN, TEMP_BED2_PIN, TEMP_BED3_PIN };
    constexpr int16_t BE_type[] = { TEMP_SENSOR_BED0, TEMP_SENSOR_BED1, TEMP_SENSOR_BED2, TEMP_SENSOR_BED3 };

    heat                  = beds[h];
    sens                  = &heat->data.sensor;
    pid                   = &heat->data.pid;
    heat->data.pin        = BE_pin[h];
    heat->data.ID         = h;
    heat->data.temp.min   = BED_MINTEMP;
    heat->data.temp.max   = BED_MAXTEMP;
    heat->data.freq       = BED_PWM_FREQUENCY;
    // Pid
    pid->Kp               = BEDKp[ALIM(h, BEDKp)];
    pid->Ki               = BEDKi[ALIM(h, BEDKi)];
    pid->Kd               = BEDKd[ALIM(h, BEDKd)];
    pid->drive.min        = BED_POWER_DRIVE_MIN;
    pid->drive.max        = BED_POWER_DRIVE_MAX;
    pid->Max              = BED_POWER_MAX;
    // Sensor
    sens->pin             = SB_pin[h];
    sens->type            = BE_type[h];
    sens->res_25          = BED0_R25;
    sens->beta            = BED0_BETA;
    sens->pullup_res      = THERMISTOR_SERIES_RS;
    sens->shC             = 0;
    sens->adcLowOffset    = 0;
    sens->adcHighOffset   = 0;
    #if HAS_AD8495 || HAS_AD595
      sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
      sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
    #endif
    heat->resetFlag();
    heat->setUsePid(PIDTEMPBED);
    heat->setHWinvert(INVERTED_BED_PIN);
    heat->setHWpwm(USEABLE_HARDWARE_PWM(heat->data.pin));
    heat->setThermalProtection(THERMAL_PROTECTION_BED);
    #if HAS_EEPROM
      heat->setPidTuned(false);
    #else
      heat->setPidTuned(true);
    #endif

  }
#endif

#if MAX_CHAMBER > 0
  void Temperature::chambers_factory_parameters(const uint8_t h) {

    Heater        *heat;
    pid_data_t    *pid;
    sensor_data_t *sens;

    constexpr float   CHAMBERKp[] = CHAMBER_Kp,
                      CHAMBERKi[] = CHAMBER_Ki,
                      CHAMBERKd[] = CHAMBER_Kd;
    constexpr pin_t   CH_pin[]    = { HEATER_CHAMBER0_PIN, HEATER_CHAMBER1_PIN, HEATER_CHAMBER2_PIN, HEATER_CHAMBER3_PIN },
                      SCH_pin[]   = { TEMP_CHAMBER0_PIN, TEMP_CHAMBER1_PIN, TEMP_CHAMBER2_PIN, TEMP_CHAMBER3_PIN };
    constexpr int16_t CH_type[]   = { TEMP_SENSOR_CHAMBER0, TEMP_SENSOR_CHAMBER1, TEMP_SENSOR_CHAMBER2, TEMP_SENSOR_CHAMBER3 };

    heat                  = chambers[h];
    sens                  = &heat->data.sensor;
    pid                   = &heat->data.pid;
    heat->data.pin        = CH_pin[h];
    heat->data.ID         = h;
    heat->data.temp.min   = CHAMBER_MINTEMP;
    heat->data.temp.max   = CHAMBER_MAXTEMP;
    heat->data.freq       = CHAMBER_PWM_FREQUENCY;
    // Pid
    pid->Kp               = CHAMBERKp[ALIM(h, CHAMBERKp)];
    pid->Ki               = CHAMBERKi[ALIM(h, CHAMBERKi)];
    pid->Kd               = CHAMBERKd[ALIM(h, CHAMBERKd)];
    pid->drive.min        = CHAMBER_POWER_DRIVE_MIN;
    pid->drive.max        = CHAMBER_POWER_DRIVE_MAX;
    pid->Max              = CHAMBER_POWER_MAX;
    // Sensor
    sens->pin             = SCH_pin[h];
    sens->type            = CH_type[h];
    sens->res_25          = CHAMBER0_R25;
    sens->beta            = CHAMBER0_BETA;
    sens->pullup_res      = THERMISTOR_SERIES_RS;
    sens->shC             = 0;
    sens->adcLowOffset    = 0;
    sens->adcHighOffset   = 0;
    #if HAS_AD8495 || HAS_AD595
      sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
      sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
    #endif
    heat->resetFlag();
    heat->setUsePid(PIDTEMPCHAMBER);
    heat->setHWinvert(INVERTED_CHAMBER_PIN);
    heat->setHWpwm(USEABLE_HARDWARE_PWM(heat->data.pin));
    heat->setThermalProtection(THERMAL_PROTECTION_CHAMBER);
    #if HAS_EEPROM
      heat->setPidTuned(false);
    #else
      heat->setPidTuned(true);
    #endif

  }
#endif

#if MAX_COOLER > 0
  void Temperature::coolers_factory_parameters(const uint8_t h) {

    Heater        *heat;
    pid_data_t    *pid;
    sensor_data_t *sens;

    heat                  = coolers[h];
    sens                  = &heat->data.sensor;
    pid                   = &heat->data.pid;
    heat->data.pin        = HEATER_COOLER_PIN;
    heat->data.ID         = 0;
    heat->data.temp.min   = COOLER_MINTEMP;
    heat->data.temp.max   = COOLER_MAXTEMP;
    heat->data.freq       = COOLER_PWM_FREQUENCY;
    // Pid
    pid->Kp               = COOLER_Kp;
    pid->Ki               = COOLER_Ki;
    pid->Kd               = COOLER_Kd;
    pid->drive.min        = COOLER_POWER_DRIVE_MIN;
    pid->drive.max        = COOLER_POWER_DRIVE_MAX;
    pid->Max              = COOLER_POWER_MAX;
    // Sensor
    sens->pin             = TEMP_COOLER_PIN;
    sens->type            = TEMP_SENSOR_COOLER;
    sens->res_25          = COOLER_R25;
    sens->beta            = COOLER_BETA;
    sens->pullup_res      = THERMISTOR_SERIES_RS;
    sens->shC             = 0;
    sens->adcLowOffset    = 0;
    sens->adcHighOffset   = 0;
    #if HAS_AD8495 || HAS_AD595
      sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
      sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
    #endif
    heat->resetFlag();
    heat->setUsePid(PIDTEMPCOOLER);
    heat->setHWinvert(INVERTED_COOLER_PIN);
    heat->setHWpwm(USEABLE_HARDWARE_PWM(heat->data.pin));
    heat->setThermalProtection(THERMAL_PROTECTION_COOLER);
    #if HAS_EEPROM
      heat->setPidTuned(false);
    #else
      heat->setPidTuned(true);
    #endif

  }
#endif

#if MAX_FAN > 0
  void Temperature::fans_factory_parameters(const uint8_t f) {

    constexpr pin_t   fanCh[]   = FANS_CHANNELS;
    constexpr int8_t  fanAuto[] = AUTO_FAN;

    #if ENABLED(TACHOMETRIC)
      constexpr pin_t tacho_temp_pin[] = { TACHO0_PIN, TACHO1_PIN, TACHO2_PIN, TACHO3_PIN, TACHO4_PIN, TACHO5_PIN };
    #endif

    Fan         *fan;
    fan_data_t  *fdata;

    fan                         = fans[f];
    fdata                       = &fan->data;
    fdata->ID                   = f;
    fdata->pin                  = fanCh[f];
    fdata->speed_limit.min      = FAN_MIN_PWM;
    fdata->speed_limit.max      = FAN_MAX_PWM;
    fdata->freq                 = FAN_PWM_FREQUENCY;
    fdata->trigger_temperature  = HOTEND_AUTO_FAN_TEMPERATURE;
    fdata->auto_monitor         = 0;
    fdata->flag.all             = false;
    fan->set_auto_monitor(fanAuto[f]);
    fan->setHWinvert(FAN_INVERTED);
    #if ENABLED(TACHOMETRIC)
      fdata->tacho.pin          = tacho_temp_pin[f];
    #endif

    LOOP_HOTEND() {
      if (TEST(fdata->auto_monitor, h)) {
        fdata->speed_limit.min  = HOTEND_AUTO_FAN_MIN_SPEED;
        fdata->speed_limit.max  = HOTEND_AUTO_FAN_SPEED;
      }
    }

    if (TEST(fdata->auto_monitor, 7)) {
      fdata->speed_limit.min    = CONTROLLERFAN_MIN_SPEED;
      fdata->speed_limit.max    = CONTROLLERFAN_SPEED;
    }

  }
#endif

#if HAS_MCU_TEMPERATURE
  float Temperature::analog2tempMCU(const int raw) {
    const float voltage = (float)raw * ((HAL_VOLTAGE_PIN) / (float)AD_RANGE);
    return (voltage - 0.8f) * (1000.0f / 2.65f) + 27.0f; // + mcuTemperatureAdjust;
  }
#endif

void Temperature::print_heater_state(Heater* act, const bool print_ID, const bool showRaw) {

  SERIAL_CHR(' ');

  #if MAX_HOTEND > 0
    if (act->type == IS_HOTEND) {
      SERIAL_CHR('T');
      if (print_ID) SERIAL_VAL(act->data.ID);
    }
  #endif

  #if MAX_BED > 0
    if (act->type == IS_BED) {
      SERIAL_CHR('B');
      if (print_ID) SERIAL_VAL(act->data.ID);
    }
  #endif

  #if MAX_CHAMBER > 0
    if (act->type == IS_CHAMBER) {
      SERIAL_CHR('C');
      if (print_ID) SERIAL_VAL(act->data.ID);
    }
  #endif

  #if MAX_COOLER > 0
    if (act->type == IS_COOLER) SERIAL_CHR('W');
  #endif

  const int16_t targetTemperature = act->isIdle() ? act->deg_idle() : act->deg_target();
  SERIAL_CHR(':');
  SERIAL_VAL(act->deg_current());
  SERIAL_MV(" /" , targetTemperature);
  if (showRaw) {
    SERIAL_MV(" (", act->data.sensor.raw);
    SERIAL_CHR(')');
  }

}

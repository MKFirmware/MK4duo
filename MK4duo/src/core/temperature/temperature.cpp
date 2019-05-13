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

#if ENABLED(FILAMENT_WIDTH_SENSOR)
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
  #if HOTENDS > 0
    LOOP_HOTEND() hotends[h].ResetFault();
  #endif
  #if BEDS > 0
    LOOP_BED() beds[h].ResetFault();
  #endif
  #if CHAMBERS > 0
    LOOP_CHAMBER() chambers[h].ResetFault();
  #endif
  #if COOLERS > 0
    LOOP_COOLER() coolers[h].ResetFault();
  #endif
}

void Temperature::set_current_temp_raw() {

  #if HOTENDS > 0
    LOOP_HOTEND() {
      if (WITHIN(hotends[h].sensor.pin, 0, 15))
        hotends[h].sensor.raw = HAL::AnalogInputValues[hotends[h].sensor.pin];
    }
  #endif
  #if BEDS > 0
    LOOP_BED() {
      if (WITHIN(beds[h].sensor.pin, 0, 15))
        beds[h].sensor.raw = HAL::AnalogInputValues[beds[h].sensor.pin];
    }
  #endif
  #if CHAMBERS > 0
    LOOP_CHAMBER() {
      if (WITHIN(chambers[h].sensor.pin, 0, 15))
        chambers[h].sensor.raw = HAL::AnalogInputValues[chambers[h].sensor.pin];
    }
  #endif
  #if COOLERS > 0
    LOOP_COOLER() {
      if (WITHIN(coolers[h].sensor.pin, 0, 15))
        coolers[h].sensor.raw = HAL::AnalogInputValues[coolers[h].sensor.pin];
    }
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    powerManager.current_raw_powconsumption = HAL::AnalogInputValues[POWER_CONSUMPTION_PIN];
  #endif

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
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

  #if ENABLED(EMERGENCY_PARSER)
    if (emergency_parser.killed_by_M112) printer.kill(PSTR("M112"));
  #endif

  LOOP_HOTEND() {
    Heater *act = &hotends[h];
    // Update Current Temperature
    act->update_current_temperature();
    act->check_and_power();
  }

  #if BEDS > 0
    LOOP_BED() {
      Heater *act = &beds[h];
      // Update Current Temperature
      act->update_current_temperature();
      act->check_and_power();
    } // LOOP_BED
  #endif

  #if CHAMBERS > 0
    LOOP_CHAMBER() {
      Heater *act = &chambers[h];
      // Update Current Temperature
      act->update_current_temperature();
      act->check_and_power();
    } // LOOP_CHAMBER
  #endif

  #if COOLERS > 0
    LOOP_COOLER() {
      Heater *act = &coolers[h];
      // Update Current Temperature
      act->update_current_temperature();
      act->check_and_power();
    } // LOOP_COOLER
  #endif

  #if HAS_MCU_TEMPERATURE
    mcu_current_temperature = analog2tempMCU(mcu_current_temperature_raw);
    NOLESS(mcu_highest_temperature, mcu_current_temperature);
    NOMORE(mcu_lowest_temperature, mcu_current_temperature);
  #endif

  // Control the extruder rate based on the width sensor
  #if ENABLED(FILAMENT_WIDTH_SENSOR)

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

  #if HOTENDS > 0
    LOOP_HOTEND() {
      hotends[h].setTarget(0);
      hotends[h].start_watching();
    }
  #endif
  #if BEDS > 0
    LOOP_BED() {
      beds[h].setTarget(0);
      beds[h].start_watching();
    }
  #endif
  #if CHAMBERS > 0
    LOOP_CHAMBER() {
      chambers[h].setTarget(0);
      chambers[h].start_watching();
    }
  #endif
  #if COOLERS > 0
    LOOP_COOLER() {
      coolers[h].setTarget(0);
      coolers[h].start_watching();
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
  #if HOTENDS > 0
    LOOP_HOTEND() if (hotends[h].isActive()) return true;
  #endif
  #if BEDS > 0
    LOOP_BED() if (beds[h].isActive()) return true;
  #endif
  #if CHAMBERS > 0
    LOOP_CHAMBER() if (chambers[h].isActive()) return true;
  #endif
  #if COOLERS > 0
    LOOP_COOLER() if (coolers[h].isActive()) return true;
  #endif
  return false;
}


/**
 * Calc min & max temp of all heaters
 */
#define MAX_TEMP_RANGE 10

#if HOTENDS > 0

  /**
   * Calc min & max temp of all hotends
   */
  int16_t Temperature::hotend_mintemp_all() {
    int16_t mintemp = 9999;
    LOOP_HOTEND() mintemp = MIN(mintemp, hotends[h].data.mintemp);
    return mintemp;
  }

  int16_t Temperature::hotend_maxtemp_all() {
    int16_t maxtemp = 0;
    LOOP_HOTEND() maxtemp = MAX(maxtemp, hotends[h].data.maxtemp);
    return maxtemp - MAX_TEMP_RANGE;
  }

#endif

#if BEDS > 0

  int16_t Temperature::bed_mintemp_all() {
    int16_t mintemp = 9999;
    LOOP_BED() mintemp = MIN(mintemp, beds[h].data.mintemp);
    return mintemp;
  }

  int16_t Temperature::bed_maxtemp_all() {
    int16_t maxtemp = 0;
    LOOP_HOTEND() maxtemp = MAX(maxtemp, beds[h].data.maxtemp);
    return maxtemp - MAX_TEMP_RANGE;
  }

#endif

#if CHAMBERS > 0

  int16_t Temperature::chamber_mintemp_all() {
    int16_t mintemp = 9999;
    LOOP_CHAMBER() mintemp = MIN(mintemp, chambers[h].data.mintemp);
    return mintemp;
  }

  int16_t Temperature::chamber_maxtemp_all() {
    int16_t maxtemp = 0;
    LOOP_CHAMBER() maxtemp = MAX(maxtemp, chambers[h].data.maxtemp);
    return maxtemp - MAX_TEMP_RANGE;
  }

#endif

#if COOLERS > 0

  int16_t Temperature::cooler_mintemp_all() {
    int16_t mintemp = 9999;
    LOOP_COOLER() mintemp = MIN(mintemp, coolers[h].data.mintemp);
    return mintemp;
  }

  int16_t Temperature::cooler_maxtemp_all() {
    int16_t maxtemp = 0;
    LOOP_COOLER() maxtemp = MAX(maxtemp, coolers[h].data.maxtemp);
    return maxtemp - MAX_TEMP_RANGE;
  }

#endif

#if HAS_MAX6675 || HAS_MAX31855

  void Temperature::getTemperature_SPI() {
    LOOP_HOTEND() {
      Heater *act = &hotends[h];
      #if HAS_MAX31855
        if (act->sensor.type == -4)
          act->sensor.raw = act->sensor.read_max31855();
      #endif
      #if HAS_MAX6675
        if (act->sensor.type == -3)
          act->sensor.raw = act->sensor.read_max6675();
      #endif
    }
  }

#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)

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
        LOOP_HOTEND() hotends[h].start_idle_timer(0); // timeout immediately
      else
        LOOP_HOTEND() hotends[h].reset_idle_timer();
    }
  }
#endif // PROBING_HEATERS_OFF

void Temperature::report_temperatures(const bool showRaw/*=false*/) {

  #if HOTENDS > 0
    print_heater_state(&hotends[ACTIVE_HOTEND], false, showRaw);
    SERIAL_MV(MSG_AT ":", int(hotends[ACTIVE_HOTEND].pwm_value));
  #endif

  #if BEDS > 0
    print_heater_state(&beds[0], false, showRaw);
    SERIAL_MV(MSG_BAT ":", int(beds[0].pwm_value));
  #endif

  #if CHAMBERS > 0
    print_heater_state(&chambers[0], false, showRaw);
    SERIAL_MV(MSG_CAT ":", int(chambers[0].pwm_value));
  #endif

  #if COOLERS > 0
    print_heater_state(&coolers[0], false, showRaw);
    SERIAL_MV(MSG_CAT ":", int(coolers[0].pwm_value));
  #endif

  #if HOTENDS > 1
    LOOP_HOTEND() {
      print_heater_state(&hotends[h], true, showRaw);
      SERIAL_MV(MSG_AT, int(h));
      SERIAL_CHR(':');
      SERIAL_VAL(int(hotends[h].pwm_value));
    }
  #endif

  #if BEDS > 1
    LOOP_BED() {
      print_heater_state(&beds[h], true, showRaw);
      SERIAL_MV(MSG_BAT, int(h));
      SERIAL_CHR(':');
      SERIAL_VAL(int(beds[h].pwm_value));
    }
  #endif

  #if CHAMBERS > 1
    LOOP_CHAMBER() {
      print_heater_state(&chambers[h], true, showRaw);
      SERIAL_MV(MSG_CAT, int(h));
      SERIAL_CHR(':');
      SERIAL_VAL(int(chambers[h].pwm_value));
    }
  #endif

  #if COOLERS > 1
    LOOP_COOLER() {
      print_heater_state(&coolers[h], true, showRaw);
      SERIAL_MV(MSG_CAT, int(h));
      SERIAL_CHR(':');
      SERIAL_VAL(int(coolers[h].pwm_value));
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
    SERIAL_MV(" DHT Temp:", dhtsensor.Temperature, 1);
    SERIAL_MV(", Humidity:", dhtsensor.Humidity, 1);
  #endif

}

// Private function

#if HAS_MCU_TEMPERATURE
  float Temperature::analog2tempMCU(const int raw) {
    const float voltage = (float)raw * ((HAL_VOLTAGE_PIN) / (float)16384);
    return (voltage - 0.8) * (1000.0 / 2.65) + 27.0; // + mcuTemperatureAdjust;			// accuracy at 27C is +/-45C
  }
#endif

void Temperature::print_heater_state(Heater *act, const bool print_ID, const bool showRaw) {

  SERIAL_CHR(' ');

  #if HAS_TEMP_HOTEND
    if (act->type == IS_HOTEND) {
      SERIAL_CHR('T');
      #if HOTENDS > 1
        if (print_ID) SERIAL_VAL(act->data.ID);
      #else
        UNUSED(print_ID);
      #endif
    }
  #endif

  #if BEDS > 0
    if (act->type == IS_BED) SERIAL_CHR('B');
  #endif

  #if CHAMBERS > 0
    if (act->type == IS_CHAMBER) SERIAL_CHR('C');
  #endif

  #if COOLERS > 0
    if (act->type == IS_COOLER) SERIAL_CHR('W');
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

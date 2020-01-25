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
#pragma once

#include "dhtsensor/dhtsensor.h"
#include "sensor/sensor.h"
#include "pid/pid.h"
#include "heater/heater.h"

struct temp_data_t {
  uint8_t hotends   : 4;
  uint8_t beds      : 4;
  uint8_t chambers  : 4;
  uint8_t coolers   : 4;
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    int16_t lpq_len : 16;
  #endif
};

/**
 * tempmanager.h - temperature manager
 */
class TempManager {

  public: /** Constructor */

    TempManager() {};

  public: /** Public Parameters */

    static temp_data_t heater;

    #if DISABLED(SOFTWARE_PDM)
      static uint8_t pwm_soft_count;
    #endif

    #if HAS_MCU_TEMPERATURE
      static int16_t  mcu_current_temperature,
                      mcu_highest_temperature,
                      mcu_alarm_temperature,
                      mcu_current_temperature_raw;
    #endif

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static float    redundant_temperature;
    #endif

    #if ENABLED(PREVENT_COLD_EXTRUSION)
      static int16_t extrude_min_temp;
    #endif

  private: /** Private Parameters */

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      static int8_t   meas_shift_index;     // Index of a delayed sample in buffer
      static uint16_t current_raw_filwidth; // Measured filament diameter - one extruder only
    #endif

    #if ENABLED(PROBING_HEATERS_OFF)
      static bool paused;
    #endif

  public: /** Public Function */

    /**
     * Initialize the temperature manager
     */
    static void init();

    /**
     * Create Object Heater and fan
     */
    static void create_object();

    /**
     * Initialize to the factory parameters
     */
    static void factory_parameters();

    /**
     * Change number heater
     */
    static void change_number_heater(const HeatertypeEnum type, const uint8_t h);

    /**
     * Set Heaters pwm output
     */
    static void set_output_pwm();

    /**
     * Call periodically to HAL isr
     */
    static void spin();

    /**
     * Switch off all heaters, set all target temperatures to 0
     */
    static void disable_all_heaters();

    /**
     * Check if there are heaters Active
     */
    static bool heaters_isActive();

    /**
     * Calc min & max temp of all hotends
     */
    #if HAS_HOTENDS
      static int16_t hotend_mintemp_all();
      static int16_t hotend_maxtemp_all();
    #endif

    /**
     * Calc min & max temp of all beds
     */
    #if HAS_BEDS
      static int16_t bed_mintemp_all();
      static int16_t bed_maxtemp_all();
    #endif

    /**
     * Calc min & max temp of all chambers
     */
    #if HAS_CHAMBERS
      static int16_t chamber_mintemp_all();
      static int16_t chamber_maxtemp_all();
    #endif

    /**
     * Calc min & max temp of all coolers
     */
    #if HAS_COOLERS
      static int16_t cooler_mintemp_all();
      static int16_t cooler_maxtemp_all();
    #endif

    #if HAS_MAX31855 || HAS_MAX6675
      static void getTemperature_SPI();
    #endif

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      static int8_t widthFil_to_size_ratio(); // Convert Filament Width (mm) to an extrusion ratio
    #endif    

    #if ENABLED(PROBING_HEATERS_OFF)
      static void pause(const bool p);
      static bool is_paused() { return paused; }
    #endif

    static void report_temperatures(const bool showRaw=false);

    #if ENABLED(PREVENT_COLD_EXTRUSION)
      FORCE_INLINE static bool tooCold(const int16_t temp) {
        return printer.isAllowColdExtrude() ? false : temp < extrude_min_temp;
      }
      FORCE_INLINE static bool tooColdToExtrude(const uint8_t e) {
        return tooCold(hotends[extruders[e]->get_hotend()]->deg_current());
      }
      FORCE_INLINE static bool targetTooColdToExtrude(const uint8_t e) {
        return tooCold(hotends[extruders[e]->get_hotend()]->deg_target());
      }
    #else
      FORCE_INLINE static bool tooColdToExtrude(const uint8_t) { return false; }
      FORCE_INLINE static bool targetTooColdToExtrude(const uint8_t) { return false; }
    #endif

    FORCE_INLINE static bool hotEnoughToExtrude(const uint8_t h) { return !tooColdToExtrude(h); }
    FORCE_INLINE static bool targetHotEnoughToExtrude(const uint8_t h) { return !targetTooColdToExtrude(h); }

  private: /** Private Function */

    /**
     * Hotends Factory parameters
     */
    #if HAS_HOTENDS
      static void hotends_factory_parameters(const uint8_t h);
    #endif

    /**
     * Beds Factory parameters
     */
    #if HAS_BEDS
      static void beds_factory_parameters(const uint8_t h);
    #endif

    /**
     * Chambers Factory parameters
     */
    #if HAS_CHAMBERS
      static void chambers_factory_parameters(const uint8_t h);
    #endif

    /**
     * Coolers Factory parameters
     */
    #if HAS_COOLERS
      static void coolers_factory_parameters(const uint8_t h);
    #endif

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      static float analog2widthFil(); // Convert adc_raw Filament Width to millimeters
    #endif

    static void print_heater_state(Heater* act, const bool print_ID, const bool showRaw);

};

extern TempManager tempManager;

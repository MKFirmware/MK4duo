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
#pragma once

#include "duration_t.h"

//#define DEBUG_PRINTCOUNTER

struct printStatistics {
  uint16_t  totalPrints,      // Number of prints
            finishedPrints;   // Number of complete prints
  uint32_t  timePrint,        // Accumulated printing time
            longestPrint,     // Longest successful print job
            timePowerOn;      // Accumulated printer in power on
  float     filamentUsed;     // Accumulated filament consumed in mm

  #if HAS_POWER_CONSUMPTION_SENSOR
    uint32_t consumptionHour; // Holds the power consumption per hour as accurately measured
  #endif

};

class PrintCounter: public Stopwatch {

  private: /** Private Parameters */

    typedef Stopwatch super;

    static printStatistics data;

    #if ENABLED(EEPROM_I2C) || ENABLED(EEPROM_SPI) || ENABLED(CPU_32_BIT)
      static const uint32_t address;
    #else
      static const uint16_t address;
    #endif

    /**
     * @brief Timestamp of the last call to deltaDuration()
     * @details Stores the timestamp of the last deltaDuration(), this is
     * required due to the updateInterval cycle.
     */
    static millis_t lastDuration;

  public: /** Public Function */

    /**
     * @brief Initialize the print counter
     */
    static inline void init() {
      super::init();
      loadStats();
    }

    /**
     * @brief Resets the Print Statistics
     * @details Resets the statistics to zero
     * also the magic header.
     */
    static void initStats();

    /**
     * @brief Serial output the Print Statistics
     * @details This function may change in the future, for now it directly
     * prints the statistical data to serial.
     */
    static void showStats();

    /**
     * @brief Loads the Print Statistics
     * @details Loads the statistics from SDCARD
     */
    static void loadStats();

    /**
     * @brief Saves the Print Statistics
     * @details Saves the statistics to EEPROM
     */
    static void saveStats();

    /**
     * @brief Return the currently loaded statistics
     * @details Return the raw data, in the same structure used internally
     */
    static printStatistics getStats() { return data; }

    /**
     * @brief Loop function
     * @details This function should be called at loop, it will take care of
     * periodically save the statistical data to SDCARD and do time keeping.
     */
    static void tick();

    /**
     * @brief Increment the total filament used
     * @details The total filament used counter will be incremented by "amount".
     *
     * @param amount The amount of filament used in mm
     */
    static void incFilamentUsed(float const &amount);

    /**
     * @brief Increment the total Consumtion Hour
     */
    #if HAS_POWER_CONSUMPTION_SENSOR
      static void incConsumptionHour() { data.consumptionHour++; }
      static uint32_t getConsumptionHour() { return data.consumptionHour; }
    #endif

    /**
     * The following functions are being overridden
     */
    static bool start();
    static bool stop();
    static void reset();

    #if ENABLED(DEBUG_PRINTCOUNTER)

      /**
       * @brief Prints a debug message
       * @details Prints a simple debug message "PrintCounter::function"
       */
      static void debug(const char func[]);

    #endif

  protected: /** Protected Function */

    /**
     * @brief dT since the last call
     * @details Returns the elapsed time in seconds since the last call, this is
     * used internally for print statistics accounting is not intended to be a
     * user callable function.
     */
    static millis_t deltaDuration();

};

extern PrintCounter print_job_counter;

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

#ifndef _PRINTCOUNTER_H_
#define _PRINTCOUNTER_H_

#include "stopwatch.h"
#include "duration_t.h"

//#define DEBUG_PRINTCOUNTER

struct printStatistics {
  uint16_t  totalPrints;    // Number of prints
  uint16_t  finishedPrints; // Number of complete prints
  uint32_t  printTime;      // Accumulated printing time
  uint32_t  printer_usage;  // Printer usage ON
  double    filamentUsed;   // Accumulated filament consumed in mm
};

class PrintCounter: public Stopwatch {

  public: /** Public Parameters */

    static printStatistics data;

    /**
     * @brief Stats were loaded from SDCARD
     * @details If set to true it indicates if the statistical data was already
     * loaded from the SDCARD.
     */
    static bool loaded;

  private: /** Private Parameters */

    typedef Stopwatch super;

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
     * @brief Loads the Print Statistics
     * @details Loads the statistics from SDCARD
     */
    static void loadStats();

    /**
     * @brief Saves the Print Statistics
     * @details Saves the statistics to SDCARD
     */
    static void saveStats();

    /**
     * @brief Serial output the Print Statistics
     * @details This function may change in the future, for now it directly
     * prints the statistical data to serial.
     */
    static void showStats();

    /**
     * @brief Loop function
     * @details This function should be called at loop, it will take care of
     * periodically save the statistical data to SDCARD and do time keeping.
     */
    static void tick();

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

  private: /** Private Function */

    /**
     * @brief Interval in seconds between counter updates
     * @details This const value defines what will be the time between each
     * accumulator update.
     */
    static const uint16_t updateInterval;

    /**
     * @brief Interval in seconds between SDCARD saves
     * @details This const value defines what will be the time between each
     */
    static const uint16_t saveInterval;

    /**
     * @brief Timestamp of the last call to deltaDuration()
     * @details Stores the timestamp of the last deltaDuration(), this is
     * required due to the updateInterval cycle.
     */
    static millis_t lastDuration;

  protected: /** Protected Parameters */

    /**
     * @brief dT since the last call
     * @details Returns the elapsed time in seconds since the last call, this is
     * used internally for print statistics accounting is not intended to be a
     * user callable function.
     */
    static millis_t deltaDuration();

};

extern PrintCounter print_job_counter;

#endif /* _PRINTCOUNTER_H_ */

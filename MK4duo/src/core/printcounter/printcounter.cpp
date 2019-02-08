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

#include "../../../MK4duo.h"

#define STATS_EEPROM_ADDRESS  0x32
#define STATS_UPDATE_INTERVAL   10
#define STATS_SAVE_INTERVAL   3600

PrintCounter print_job_counter;

/** Private Parameters */
const char statistics_version[3] = "MK";

printStatistics PrintCounter::data;

#if ENABLED(EEPROM_I2C) || ENABLED(EEPROM_SPI) || ENABLED(CPU_32_BIT)
  const uint32_t PrintCounter::address = STATS_EEPROM_ADDRESS;
#else
  const uint16_t PrintCounter::address = STATS_EEPROM_ADDRESS;
#endif

millis_t PrintCounter::lastDuration;

/** Public Function */
void PrintCounter::initStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    debug(PSTR("initStats"));
  #endif

  data.totalPrints    = 0;
  data.finishedPrints = 0;
  data.timePrint      = 0;
  data.longestPrint   = 0;
  data.timePowerOn    = 0;
  data.filamentUsed   = 0.0;

  #if HAS_POWER_CONSUMPTION_SENSOR
    data.consumptionHour = 0;
  #endif

  #if HAS_EEPROM
    memorystore.write_data(address, (uint8_t*)&statistics_version, sizeof(statistics_version));
    memorystore.access_write();
  #endif
}

void PrintCounter::showStats() {
  char buffer[21];
  duration_t elapsed;

  SERIAL_MSG(MSG_STATS);

  SERIAL_MV("Total:", data.totalPrints);
  SERIAL_MV(", Finished:", data.finishedPrints);
  SERIAL_MSG(", Failed:");
  SERIAL_EV (data.totalPrints - data.finishedPrints -
            ((isRunning() || isPaused()) ? 1 : 0));

  SERIAL_MSG(MSG_STATS);

  elapsed = data.timePrint;
  elapsed.toString(buffer);
  SERIAL_MT("Total print time:", buffer);

  elapsed = data.longestPrint;
  elapsed.toString(buffer);
  SERIAL_EMT(", Longest job:", buffer);

  SERIAL_MSG(MSG_STATS);

  elapsed = data.timePowerOn;
  elapsed.toString(buffer);
  SERIAL_EMT("Power on time:", buffer);

  SERIAL_MSG(MSG_STATS);

  lengthtoString(buffer, data.filamentUsed);
  SERIAL_EMT("Filament used:", buffer);

  #if HAS_POWER_CONSUMPTION_SENSOR
    SERIAL_MSG(MSG_STATS);
    SERIAL_MV(CFG, "Watt/h consumed:", data.consumptionHour);
    SERIAL_EM(" Wh");
  #endif

}

void PrintCounter::loadStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    debug(PSTR("loadStats"));
  #endif

  #if HAS_EEPROM

    // Check if the EEPROM block is initialized
    char value[3];
    memorystore.access_read();

    memorystore.read_data(address, (uint8_t*)&value, sizeof(value));

    if (strncmp(statistics_version, value, 2) != 0)
      initStats();
    else
      memorystore.read_data(address + sizeof(statistics_version), (uint8_t*)&data, sizeof(printStatistics));

  #endif

  printer.setStatisticsLoaded(true);

}

void PrintCounter::saveStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    debug(PSTR("saveStats"));
  #endif

  // Refuses to save data is object is not loaded
  if (!printer.IsStatisticsLoaded()) return;

  #if HAS_EEPROM
    // Saves the struct to EEPROM
    memorystore.write_data(address + sizeof(statistics_version), (uint8_t*)&data, sizeof(data));
    memorystore.access_write();
  #endif

}

void PrintCounter::tick() {

  static millis_t update_next = 0,
                  eeprom_next = 0;

  millis_t now = millis();

  if (ELAPSED(now, update_next)) {
    #if ENABLED(DEBUG_PRINTCOUNTER)
      debug(PSTR("tick"));
    #endif
    data.timePrint += deltaDuration();
    data.timePowerOn += STATS_UPDATE_INTERVAL;
    update_next = now + (STATS_UPDATE_INTERVAL * 1000);
  }

  if (ELAPSED(now, eeprom_next)) {
    eeprom_next = now + (STATS_SAVE_INTERVAL * 1000);
    saveStats();
  }

}

void PrintCounter::incFilamentUsed(float const &amount) {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    debug(PSTR("incFilamentUsed"));
  #endif

  // Refuses to update data if object is not loaded
  if (!printer.IsStatisticsLoaded()) return;

  data.filamentUsed += amount; // mm
}

bool PrintCounter::start() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    debug(PSTR("start"));
  #endif

  bool paused = isPaused();

  #if HAS_POWER_CONSUMPTION_SENSOR
    powerManager.startpower = data.consumptionHour;
  #endif

  if (super::start()) {
    if (!paused) {
      data.totalPrints++;
      lastDuration = 0;
    }
    return true;
  }
  else return false;
}

bool PrintCounter::stop() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    debug(PSTR("stop"));
  #endif

  if (super::stop()) {
    data.finishedPrints++;
    data.timePrint += deltaDuration();

    if (duration() > data.longestPrint)
      data.longestPrint = duration();

    saveStats();
    return true;
  }
  else return false;
}

void PrintCounter::reset() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    debug(PSTR("stop"));
  #endif

  super::reset();
  lastDuration = 0;
}

#if ENABLED(DEBUG_PRINTCOUNTER)

  void PrintCounter::debug(const char func[]) {
    SERIAL_SM(DEB, "PrintCounter::");
    SERIAL_MSG(func);
    SERIAL_EM("()");
  }

#endif

/** Protected Function */
millis_t PrintCounter::deltaDuration() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    debug(PSTR("deltaDuration"));
  #endif

  millis_t tmp = lastDuration;
  lastDuration = duration();
  return lastDuration - tmp;
}

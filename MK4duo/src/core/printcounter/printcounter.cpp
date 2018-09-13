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

PrintCounter print_job_counter;

printStatistics PrintCounter::data;

millis_t PrintCounter::lastDuration;
bool PrintCounter::loaded = false;

millis_t PrintCounter::deltaDuration() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("deltaDuration"));
  #endif

  millis_t tmp = lastDuration;
  lastDuration = duration();
  return lastDuration - tmp;
}

void PrintCounter::initStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("initStats"));
  #endif

  data = { 0, 0, 0, 0, 0.0 };
}

void PrintCounter::loadStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("loadStats"));
  #endif

  #if HAS_SD_SUPPORT && ENABLED(SD_SETTINGS)
    // Checks if the SDCARD is inserted
    if (IS_SD_INSERTED && !IS_SD_PRINTING)
      card.RetrieveSettings(true);
  #endif
}

void PrintCounter::saveStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("saveStats"));
  #endif

  // Refuses to save data is object is not loaded
  if (!loaded) return;

  #if HAS_SD_SUPPORT && ENABLED(SD_SETTINGS)
    card.StoreSettings();
  #endif
}

void PrintCounter::showStats() {
  char buffer[21];
  duration_t elapsed;

  SERIAL_MSG(MSG_STATS);
  SERIAL_MV("Total: ", data.totalPrints);
  SERIAL_MV(", Finished: ", data.finishedPrints);
  SERIAL_MSG(", Failed: "); // Note: Removes 1 from failures with an active counter
  SERIAL_EV (data.totalPrints - data.finishedPrints -
            ((isRunning() || isPaused()) ? 1 : 0));

  SERIAL_MSG(MSG_STATS);
  elapsed = data.printTime;
  elapsed.toString(buffer);
  SERIAL_MT("Total print time: ", buffer);
  elapsed = data.printer_usage;
  elapsed.toString(buffer);
  SERIAL_EMT(", Power on time: ", buffer);

  SERIAL_MSG(MSG_STATS);

  uint16_t  kmeter = (long)data.filamentUsed / 1000 / 1000,
            meter = ((long)data.filamentUsed / 1000) % 1000,
            centimeter = ((long)data.filamentUsed / 10) % 100,
            millimeter = ((long)data.filamentUsed) % 10;
  sprintf_P(buffer, PSTR("%uKm %um %ucm %umm"), kmeter, meter, centimeter, millimeter);

  SERIAL_EMT("Filament used: ", buffer);
}

void PrintCounter::tick() {

  static millis_t update_last = millis(),
                  config_last = millis();

  millis_t now = millis();

  if (now - update_last >= 10000UL) {
    #if ENABLED(DEBUG_PRINTCOUNTER)
      debug(PSTR("tick"));
    #endif
    data.printer_usage += 10;
    data.printTime += deltaDuration();
    update_last = now;
  }

  #if HAS_SD_SUPPORT && ENABLED(SD_SETTINGS)
    if (!loaded) {
      loadStats();
      saveStats();
    }
    else if (now - config_last >= ((SD_CFG_SECONDS) * 1000UL)) {
      config_last = now;
      saveStats();
    }
  #endif
}

bool PrintCounter::start() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("start"));
  #endif

  bool paused = isPaused();

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
    PrintCounter::debug(PSTR("stop"));
  #endif

  if (super::stop()) {
    data.finishedPrints++;
    data.printTime += deltaDuration();
    saveStats();
    return true;
  }
  else return false;
}

void PrintCounter::reset() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("stop"));
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

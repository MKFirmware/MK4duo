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

#include "../../../base.h"

PrintCounter print_job_counter = PrintCounter();

PrintCounter::PrintCounter(): super() {
  this->initStats();
}

millis_t PrintCounter::deltaDuration() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("deltaDuration"));
  #endif

  millis_t tmp = this->lastDuration;
  this->lastDuration = this->duration();
  return this->lastDuration - tmp;
}

void PrintCounter::initStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("initStats"));
  #endif

  this->data = { 0, 0, 0, 0, 0.0 };
}

void PrintCounter::loadStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("loadStats"));
  #endif

  #if HAS_SDSUPPORT && ENABLED(SD_SETTINGS)
    // Checks if the SDCARD is inserted
    if(IS_SD_INSERTED && !IS_SD_PRINTING) {
      card.RetrieveSettings(true);
    }
  #endif
}

void PrintCounter::saveStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("saveStats"));
  #endif

  // Refuses to save data is object is not loaded
  if (!this->loaded) return;

  #if HAS_SDSUPPORT && ENABLED(SD_SETTINGS)
    card.StoreSettings();
  #endif
}

void PrintCounter::showStats() {
  char buffer[21];
  duration_t elapsed;

  SERIAL_MSG(MSG_STATS);
  SERIAL_MV("Total: ", this->data.totalPrints);
  SERIAL_MV(", Finished: ", this->data.finishedPrints);
  SERIAL_MSG(", Failed: "); // Note: Removes 1 from failures with an active counter
  SERIAL_EV (this->data.totalPrints - this->data.finishedPrints -
            ((this->isRunning() || this->isPaused()) ? 1 : 0));

  SERIAL_MSG(MSG_STATS);
  elapsed = this->data.printTime;
  elapsed.toString(buffer);
  SERIAL_MT("Total print time: ", buffer);
  elapsed = this->data.printer_usage;
  elapsed.toString(buffer);
  SERIAL_EMT(", Power on time: ", buffer);

  SERIAL_MSG(MSG_STATS);

  uint16_t  kmeter = (long)this->data.filamentUsed / 1000 / 1000,
            meter = ((long)this->data.filamentUsed / 1000) % 1000,
            centimeter = ((long)this->data.filamentUsed / 10) % 100,
            millimeter = ((long)this->data.filamentUsed) % 10;
  sprintf_P(buffer, PSTR("%uKm %um %ucm %umm"), kmeter, meter, centimeter, millimeter);

  SERIAL_EMT("Filament used: ", buffer);
}

void PrintCounter::tick() {

  static millis_t update_last = millis(),
                  config_last = millis();

  millis_t now = millis();

  // Trying to get the amount of calculations down to the bare min
  const static uint16_t i = this->updateInterval * 1000UL;

  if (now - update_last >= i) {
    this->data.printer_usage += this->updateInterval;

    if (this->isRunning())
      this->data.printTime += this->deltaDuration();

    update_last = now;

    #if ENABLED(DEBUG_PRINTCOUNTER)
      PrintCounter::debug(PSTR("tick"));
    #endif
  }

  #if HAS_SDSUPPORT && ENABLED(SD_SETTINGS)
    const static millis_t j = this->saveInterval * 1000UL;
    if (!this->loaded) {
      this->loadStats();
      this->saveStats();
    }
    else if (now - config_last >= j) {
      config_last = now;
      this->saveStats();
    }
  #endif
}

bool PrintCounter::start() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("start"));
  #endif

  bool paused = this->isPaused();

  if (super::start()) {
    if (!paused) {
      this->data.totalPrints++;
      this->lastDuration = 0;
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
    this->data.finishedPrints++;
    this->data.printTime += this->deltaDuration();
    this->saveStats();
    return true;
  }
  else return false;
}

void PrintCounter::reset() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("stop"));
  #endif

  super::reset();
  this->lastDuration = 0;
}

#if ENABLED(DEBUG_PRINTCOUNTER)

  void PrintCounter::debug(const char func[]) {
    SERIAL_SM(DEB, "PrintCounter::");
    SERIAL_MSG(func);
    SERIAL_EM("()");
  }

#endif

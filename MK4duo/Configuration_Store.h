/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#ifndef CONFIGURATION_STORE_H
  #define CONFIGURATION_STORE_H

  void Config_ResetDefault();
  void ConfigSD_ResetDefault();

  #if DISABLED(DISABLE_M503)
    void Config_PrintSettings(bool forReplay = false);
    void ConfigSD_PrintSettings();
  #else
    FORCE_INLINE void Config_PrintSettings(bool forReplay = false) {}
    FORCE_INLINE void ConfigSD_PrintSettings() {}
  #endif

  #if ENABLED(EEPROM_SETTINGS)
    void Config_StoreSettings();
    void Config_RetrieveSettings();
  #else
    FORCE_INLINE void Config_StoreSettings() {}
    FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); Config_PrintSettings(); }
  #endif

  #if ENABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
    #define CFG_SD_MAX_KEY_LEN    3+1         // icrease this if you add key name longer than the actual value.
    #define CFG_SD_MAX_VALUE_LEN  10+1        // this should be enought for int, long and float if you need to retrive strings increase this carefully
    //(11 = strlen("4294967295")+1) (4294967295 = (2^32)-1) (32 = the num of bits of the bigger basic data scructor used)
    //If yuou need to save string increase this to strlen("YOUR LONGER STRING")+1
    void ConfigSD_StoreSettings();
    void ConfigSD_RetrieveSettings(bool addValue = false);
    int ConfigSD_KeyIndex(char *key);
  #else
    FORCE_INLINE void ConfigSD_RetrieveSettings() {  ConfigSD_ResetDefault(); }
  #endif

#endif //CONFIGURATION_STORE_H

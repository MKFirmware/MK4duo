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

#ifndef CARDREADER_H
#define CARDREADER_H

#if HAS_SD_SUPPORT

  #if ENABLED(SD_SETTINGS)
    /**
     * SD Settings
     */
    enum cfgSD_ENUM {   // This need to be in the same order as cfgSD_KEY
      SD_CFG_CPR,
      SD_CFG_FIL,
      SD_CFG_NPR,
    #if HAS_POWER_CONSUMPTION_SENSOR
      SD_CFG_PWR,
    #endif
      SD_CFG_TME,
      SD_CFG_TPR,
      SD_CFG_END // Leave this always as the last
    };
  #endif

  enum LsAction : char { LS_Count, LS_GetFilename };

  enum FlagCardReader : char {
    flag_SD_OK,
    flag_SD_saving,
    flag_SD_printing,
    flag_SD_filenameIsDir
  };

  #include "SDFat.h"

  class CardReader {

    public: /** Constructor */

      CardReader() {};

    public: /** Public Parameters */

      static SdFat      fat;
      static SdFile     gcode_file;
      static SdBaseFile root,
                        workDir,
                        workDirParents[SD_MAX_FOLDER_DEPTH];

      static int autostart_index;

      static uint32_t fileSize,
                      sdpos;

      static float  objectHeight,
                    firstlayerHeight,
                    layerHeight,
                    filamentNeeded;

      static char fileName[LONG_FILENAME_LENGTH],
                  tempLongFilename[LONG_FILENAME_LENGTH + 1],
                  generatedBy[GENBY_SIZE];

    private: /** Private Parameters */

      static uint8_t card_flag;

      static uint16_t nrFile_index;

      #if HAS_SD_RESTART
        static SdFile restart_file;
      #endif

      #if HAS_EEPROM_SD
        static SdFile eeprom_file;
      #endif

      #if ENABLED(SD_SETTINGS)
        static SdFile settings_file;
      #endif

      static uint16_t workDirDepth,
                      nrFiles;              // counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
      static LsAction lsAction;             // stored for recursion.

      // Sort files and folders alphabetically.
      #if ENABLED(SDCARD_SORT_ALPHA)
        static uint16_t sort_count;         // Count of sorted items in the current directory
        #if ENABLED(SDSORT_GCODE)
          static bool sort_alpha;           // Flag to enable / disable the feature
          static int sort_folders;          // Flag to enable / disable folder sorting
          //static bool sort_reverse;       // Flag to enable / disable reverse sorting
        #endif

        // By default the sort index is static
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          static uint8_t *sort_order;
        #else
          static uint8_t sort_order[SDSORT_LIMIT];
        #endif

        #if ENABLED(SDSORT_USES_RAM) && ENABLED(SDSORT_CACHE_NAMES) && DISABLED(SDSORT_DYNAMIC_RAM)
          #define SORTED_LONGNAME_MAXLEN ((SDSORT_CACHE_VFATS) * (FILENAME_LENGTH) + 1)
        #else
          #define SORTED_LONGNAME_MAXLEN LONG_FILENAME_LENGTH
        #endif

        // Cache filenames to speed up SD menus.
        #if ENABLED(SDSORT_USES_RAM)

          // If using dynamic ram for names, allocate on the heap.
          #if ENABLED(SDSORT_CACHE_NAMES)
            #if ENABLED(SDSORT_DYNAMIC_RAM)
              static char **sortshort, **sortnames;
            #else
              static char sortnames[SDSORT_LIMIT][SORTED_LONGNAME_MAXLEN];
            #endif
          #elif DISABLED(SDSORT_USES_STACK)
            static char sortnames[SDSORT_LIMIT][SORTED_LONGNAME_MAXLEN];
          #endif

          // Folder sorting uses an isDir array when caching items.
          #if HAS_FOLDER_SORTING
            #if ENABLED(SDSORT_DYNAMIC_RAM)
              static uint8_t *isDir;
            #elif ENABLED(SDSORT_CACHE_NAMES) || DISABLED(SDSORT_USES_STACK)
              static uint8_t isDir[(SDSORT_LIMIT + 7)>>3];
            #endif
          #endif

        #endif // SDSORT_USES_RAM

      #endif // SDCARD_SORT_ALPHA

    public: /** Public Function */

      static void mount();
      static void unmount();
      static void ls();
      static void getfilename(uint16_t nr, const char* const match=NULL);
      static void getAbsFilename(char* name);
      static void startFileprint();
      static void openAndPrintFile(const char* name);
      static void stopSDPrint();
      static void write_command(char* buf);
      static void printStatus();
      static void startWrite(char* filename, const bool silent=false);
      static void deleteFile(char* filename);
      static void finishWrite();
      static void makeDirectory(char* filename);
      static void closeFile();
      static void printingHasFinished();
      static void chdir(const char* relpath);
      static void reset_default();
      static void print_settings();
      static void beginautostart();
      static void checkautostart();
      static void setroot();
      static void setlast();

      static void printEscapeChars(const char* s);

      static bool selectFile(const char* filename);

      static int8_t updir();
      static uint16_t getnrfilenames();
      static uint16_t get_num_Files();

      #if HAS_SD_RESTART
        static void open_restart_file(const bool read);
        static void close_restart_file();
        static void delete_restart_file();
        static bool exist_restart_file();
        static int16_t save_restart_data();
        static int16_t read_restart_data();
      #endif

      #if HAS_EEPROM_SD
        static void open_eeprom_sd(const bool read);
        static void close_eeprom_sd();
        FORCE_INLINE static int16_t write_eeprom_data(void* buf, uint16_t nbyte)  { return eeprom_file.isOpen() ? eeprom_file.write(buf, nbyte) : -1; }
        FORCE_INLINE static int16_t read_eeprom_data(void* buf, uint16_t nbyte)   { return eeprom_file.isOpen() ? eeprom_file.read(buf, nbyte) : -1; }
      #endif

      #if ENABLED(SD_SETTINGS)
        #define CFG_SD_MAX_KEY_LEN    3+1         // increase this if you add key name longer than the actual value.
        #define CFG_SD_MAX_VALUE_LEN  10+1        // this should be enough for int, long and float: if you need to retrieve strings increase this carefully
        //(11 = strlen("4294967295")+1) (4294967295 = (2^32)-1) (32 = the num of bits of the bigger basic data structure used)
        //If you need to save string increase this to strlen("YOUR LONGER STRING")+1
        static void StoreSettings();
        static void RetrieveSettings(bool addValue = false);
        static void parseKeyLine(char* key, char* value, int &len_k, int &len_v);
        static void unparseKeyLine(const char* key, char* value);
        static int  KeyIndex(char* key);
      #else
        static inline void RetrieveSettings() { reset_default(); }
      #endif

      #if ENABLED(SDCARD_SORT_ALPHA)
        static void presort();
        static void getfilename_sorted(const uint16_t nr);
        #if ENABLED(SDSORT_GCODE)
          FORCE_INLINE static void setSortOn(const bool b) { sort_alpha = b; presort(); }
          FORCE_INLINE static void setSortFolders(const int i) { sort_folders = i; presort(); }
          //FORCE_INLINE void setSortReverse(const bool b) { sort_reverse = b; }
        #endif
      #endif

      // Flag function
      FORCE_INLINE static void setOK(const bool onoff) {
        SET_BIT(card_flag, flag_SD_OK, onoff);
      }
      FORCE_INLINE static bool isOK() { return TEST(card_flag, flag_SD_OK); }

      FORCE_INLINE static void setSaving(const bool onoff) {
        SET_BIT(card_flag, flag_SD_saving, onoff);
      }
      FORCE_INLINE static bool isSaving() { return TEST(card_flag, flag_SD_saving); }

      FORCE_INLINE static void setSDprinting(const bool onoff) {
        SET_BIT(card_flag, flag_SD_printing, onoff);
      }
      FORCE_INLINE static bool isSDprinting() { return TEST(card_flag, flag_SD_printing); }

      FORCE_INLINE static void setFilenameIsDir(const bool onoff) {
        SET_BIT(card_flag, flag_SD_filenameIsDir, onoff);
      }
      FORCE_INLINE static bool isFilenameIsDir() { return TEST(card_flag, flag_SD_filenameIsDir); }

      FORCE_INLINE static void pauseSDPrint() { setSDprinting(false); }
      FORCE_INLINE static void setIndex(uint32_t newpos) { sdpos = newpos; gcode_file.seekSet(sdpos); }
      FORCE_INLINE static uint32_t getIndex() { return sdpos; }
      FORCE_INLINE static bool isFileOpen() { return gcode_file.isOpen(); }
      FORCE_INLINE static bool eof() { return sdpos >= fileSize; }
      FORCE_INLINE static int16_t get() { sdpos = gcode_file.curPosition(); return (int16_t)gcode_file.read(); }
      FORCE_INLINE static uint8_t percentDone() { return (isFileOpen() && fileSize) ? sdpos / ((fileSize + 99) / 100) : 0; }
      FORCE_INLINE static char* getWorkDirName() { workDir.getFilename(fileName); return fileName; }

    private: /** Private Function */

      static void lsDive(SdBaseFile parent, const char* const match = NULL);
      static void parsejson(SdBaseFile &parser_file);
      static bool findGeneratedBy(char* buf, char* genBy);
      static bool findFirstLayerHeight(char* buf, float &firstlayerHeight);
      static bool findLayerHeight(char* buf, float &layerHeight);
      static bool findFilamentNeed(char* buf, float &filament);
      static bool findTotalHeight(char* buf, float &objectHeight);

      #if ENABLED(SDCARD_SORT_ALPHA)
        static void flush_presort();
      #endif

  };

  extern CardReader card;

  #define IS_SD_PRINTING (card.isSDprinting())
  #define IS_SD_FILE_OPEN (card.isFileOpen())

  #if PIN_EXISTS(SD_DETECT)
    #if ENABLED(SD_DETECT_INVERTED)
      #define IS_SD_INSERTED (READ(SD_DETECT_PIN) != 0)
    #else
      #define IS_SD_INSERTED (READ(SD_DETECT_PIN) == 0)
    #endif
  #else
    //No card detect line? Assume the card is inserted.
    #define IS_SD_INSERTED true
  #endif

#else

  #define IS_SD_PRINTING (false)
  #define IS_SD_FILE_OPEN (false)

#endif //SDSUPPORT

#endif //__CARDREADER_H

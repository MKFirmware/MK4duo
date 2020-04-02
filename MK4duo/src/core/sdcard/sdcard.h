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

#if HAS_SD_SUPPORT

#include <SdFat.h>

#if SD_FAT_VERSION < 10101
  #error "Update SDFAT library to 1.1.1 or newer."
#endif

union flagcard_t {
  uint8_t all;
  struct {
    bool  Mounted         : 1;
    bool  Saving          : 1;
    bool  Printing        : 1;
    bool  PrintComplete   : 1;
    bool  Autoreport      : 1;
    bool  Abortprinting   : 1;
    bool  FilenameIsDir   : 1;
    bool  WorkdirIsRoot   : 1;
  };
  flagcard_t() { all = 0x00; }
};

class SDCard {

  public: /** Constructor */

    SDCard() {};

  public: /** Public Parameters */

    static flagcard_t flag;

    static SdFat      fat;
    static SdFile     gcode_file,
                      root,
                      workDir,
                      workDirParents[SD_MAX_FOLDER_DEPTH];

    static int autostart_index;

    static uint32_t fileSize,
                    sdpos;

    static float  objectHeight,
                  firstlayerHeight,
                  layerHeight,
                  filamentNeeded;

    static char fileName[LONG_FILENAME_LENGTH*SD_MAX_FOLDER_DEPTH+SD_MAX_FOLDER_DEPTH+1],
                tempLongFilename[LONG_FILENAME_LENGTH+1],
                generatedBy[GENBY_SIZE];

  private: /** Private Parameters */

    static uint16_t nrFile_index;

    #if HAS_EEPROM_SD
      #define EEPROM_FILE_NAME "eeprom.bin"
      static SdFile eeprom_file;
    #endif

    static uint16_t     workDirDepth,
                        nrFiles;          // counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
    static LsActionEnum lsAction;         // stored for recursion.

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

    #if ENABLED(ADVANCED_SD_COMMAND)

      static Sd2Card  sd;

      static uint32_t cardSizeBlocks,
                      cardCapacityMB;

      // Cache for SD block
      static cache_t  cache;

      // MBR information
      static uint8_t  partType;
      static uint32_t relSector,
                      partSize;

      // Fake disk geometry
      static uint8_t  numberOfHeads,
                      sectorsPerTrack;

      // FAT parameters
      static uint16_t reservedSectors;
      static uint8_t  sectorsPerCluster;
      static uint32_t fatStart,
                      fatSize,
                      dataStart;

      // constants for file system structure
      static uint16_t const BU16 = 128,
                            BU32 = 8192;

      static const uint32_t ERASE_SIZE = 262144L;

    #endif // ADVANCED_SD_COMMAND

  public: /** Public Function */

    static void mount();
    static void unmount();
    static void ls();
    static void getfilename(uint16_t nr, PGM_P const match=nullptr);
    static void getAbsFilename(char * name);
    static void openAndPrintFile(const char * const path);
    static void startFilePrint();
    static void endFilePrint();
    static void write_command(char * buf);
    static void print_status();
    static void startWrite(const char * const path, const bool silent=false);
    static void deleteFile(const char * const path);
    static void finishWrite();
    static void makeDirectory(const char * const path);
    static void closeFile();
    static void fileHasFinished();
    static void chdir(const char * const relpath);
    static void reset_default();
    static void beginautostart();
    static void checkautostart();
    static void setroot();
    static void setlast();

    static void printEscapeChars(PGM_P s);

    static bool selectFile(const char * const path, const bool silent=false);

    static int8_t updir();
    static uint16_t getnrfilenames();
    static uint16_t get_num_Files();

    #if HAS_SD_RESTART
      static void open_restart_file(const bool read);
      static void delete_restart_file();
      static bool exist_restart_file();
    #endif

    #if HAS_EEPROM_SD
      static void import_eeprom();
      static void write_eeprom();
    #endif

    #if ENABLED(SDCARD_SORT_ALPHA)
      static void presort();
      static void getfilename_sorted(const uint16_t nr);
      #if ENABLED(SDSORT_GCODE)
        static inline void setSortOn(const bool b) { sort_alpha = b; presort(); }
        static inline void setSortFolders(const int i) { sort_folders = i; presort(); }
        //FORCE_INLINE void setSortReverse(const bool b) { sort_reverse = b; }
      #endif
    #else
      static inline void getfilename_sorted(const uint16_t nr) { getfilename(nr); }
    #endif

    // Card flag bit 0 SD Mounted
    FORCE_INLINE static void setMounted(const bool onoff) { flag.Mounted = onoff; }
    FORCE_INLINE static bool isMounted() { return flag.Mounted; }

    // Card flag bit 1 saving
    FORCE_INLINE static void setSaving(const bool onoff) { flag.Saving = onoff; }
    FORCE_INLINE static bool isSaving() { return flag.Saving; }

    // Card flag bit 2 printing
    FORCE_INLINE static void setPrinting(const bool onoff) { flag.Printing = onoff; }
    FORCE_INLINE static bool isPrinting() { return flag.Printing; }

    // Card flag bit 3 print complete
    FORCE_INLINE static void setComplete(const bool onoff) { flag.PrintComplete = onoff; }
    FORCE_INLINE static bool isComplete() { return flag.PrintComplete; }

    // Card flag bit 4 Autoreport
    FORCE_INLINE static void setAutoreport(const bool onoff) { flag.Autoreport = onoff; }
    FORCE_INLINE static bool isAutoreport() { return flag.Autoreport; }

    // Card flag bit 5 Abortprinting
    FORCE_INLINE static void setAbortSDprinting(const bool onoff) { flag.Abortprinting = onoff; }
    FORCE_INLINE static bool isAbortSDprinting() { return flag.Abortprinting; }

    // Card flag bit 6 Filename is dir
    FORCE_INLINE static void setFilenameIsDir(const bool onoff) { flag.FilenameIsDir = onoff; }
    FORCE_INLINE static bool isFilenameIsDir() { return flag.FilenameIsDir; }

    static inline void pauseSDPrint() { setPrinting(false); }
    static inline bool isFileOpen()   { return isMounted() && gcode_file.isOpen(); }
    static inline bool isPaused()     { return isFileOpen() && !isPrinting(); }
    static inline void setIndex(uint32_t newpos) { sdpos = newpos; gcode_file.seekSet(sdpos); }
    static inline uint32_t getIndex() { return sdpos; }
    static inline bool eof() { return sdpos >= fileSize; }
    static inline int16_t get() { sdpos = gcode_file.curPosition(); return (int16_t)gcode_file.read(); }
    static inline uint8_t percentDone() { return (isFileOpen() && fileSize) ? sdpos / ((fileSize + 99) / 100) : 0; }
    static inline void getWorkDirName() { workDir.getName(fileName, LONG_FILENAME_LENGTH); }
    static inline size_t read(void* buf, uint16_t nbyte) { return gcode_file.isOpen() ? gcode_file.read(buf, nbyte) : -1; }
    static inline size_t write(void* buf, uint16_t nbyte) { return gcode_file.isOpen() ? gcode_file.write(buf, nbyte) : -1; }

    #if ENABLED(ADVANCED_SD_COMMAND)
      // Format SD Card
      static void formatSD();
      // Info SD Card
      static void infoSD();
    #endif

  private: /** Private Function */

    static void openFailed(const char * const path);
    static void lsRecursive(FatFile *dir, uint8_t level=0);
    static void lsDive(SdFile parent, PGM_P const match = NULL);
    static void parsejson(SdFile &parser_file);
    static bool findGeneratedBy(char* buf, char* genBy);
    static bool findFirstLayerHeight(char* buf, float &firstlayerHeight);
    static bool findLayerHeight(char* buf, float &layerHeight);
    static bool findFilamentNeed(char* buf, float &filament);
    static bool findTotalHeight(char* buf, float &objectHeight);

    #if ENABLED(SDCARD_SORT_ALPHA)
      static void flush_presort();
    #endif

    #if ENABLED(ADVANCED_SD_COMMAND)

      // write cached block to the card
      static uint8_t writeCache(uint32_t lbn) {
        return sd.writeBlock(lbn, cache.data);
      }
      // return cylinder number for a logical block number
      static uint16_t lbnToCylinder(uint32_t lbn) {
        return lbn / (numberOfHeads * sectorsPerTrack);
      }
      // return head number for a logical block number
      static uint8_t lbnToHead(uint32_t lbn) {
        return (lbn % (numberOfHeads * sectorsPerTrack)) / sectorsPerTrack;
      }
      // return sector number for a logical block number
      static uint8_t lbnToSector(uint32_t lbn) {
        return (lbn % sectorsPerTrack) + 1;
      }
      // generate serial number from card size and micros since boot
      static uint32_t volSerialNumber() {
        return (cardSizeBlocks << 8) + micros();
      }

      // Info SD Card
      static uint8_t cidDmp();
      static uint8_t csdDmp();
      static void volDmp();

      // initialize appropriate sizes for SD capacity
      static void initSizes();

      // zero cache and optionally set the sector signature
      static void clearCache(uint8_t addSig);

      // zero FAT and root dir area on SD
      static void clearFatDir(uint32_t bgn, uint32_t count);

      // format and write the Master Boot Record
      static void writeMbr();

      // format the SD as FAT16
      static void makeFat16();
      // format the SD as FAT32
      static void makeFat32();

    #endif

};

extern SDCard card;

#define IS_SD_PRINTING()  card.isPrinting()
#define IS_SD_PAUSED()    card.isPaused()
#define IS_SD_FILE_OPEN() card.isFileOpen()
#define IS_SD_MOUNTED()   card.isMounted()

#if PIN_EXISTS(SD_DETECT)
  #if ENABLED(SD_DETECT_INVERTED)
    #define IS_SD_INSERTED()  READ(SD_DETECT_PIN)
  #else
    #define IS_SD_INSERTED()  !READ(SD_DETECT_PIN)
  #endif
#else
  //No card detect line? Assume the card is inserted.
  #define IS_SD_INSERTED() true
#endif

#else

#define IS_SD_PRINTING()  false
#define IS_SD_PAUSED()    false
#define IS_SD_FILE_OPEN() false
#define IS_SD_MOUNTED()   false

#endif // HAS_SD_SUPPORT

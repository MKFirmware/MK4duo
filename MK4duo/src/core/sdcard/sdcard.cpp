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

#include "../../../MK4duo.h"
#include "sanitycheck.h"

#if HAS_SD_SUPPORT

SDCard card;

/** Public Parameters */
flagcard_t  SDCard::flag;

SdFat       SDCard::fat;
SdFile      SDCard::gcode_file,
            SDCard::root,
            SDCard::workDir,
            SDCard::workDirParents[SD_MAX_FOLDER_DEPTH];

int SDCard::autostart_index = -1;

uint32_t  SDCard::fileSize  = 0,
          SDCard::sdpos     = 0;

float SDCard::objectHeight      = 0.0,
      SDCard::firstlayerHeight  = 0.0,
      SDCard::layerHeight       = 0.0,
      SDCard::filamentNeeded    = 0.0;

char  SDCard::fileName[LONG_FILENAME_LENGTH*SD_MAX_FOLDER_DEPTH+SD_MAX_FOLDER_DEPTH+1],
      SDCard::tempLongFilename[LONG_FILENAME_LENGTH+1],
      SDCard::generatedBy[GENBY_SIZE];

/** Private Parameters */
uint16_t SDCard::nrFile_index = 0;

#if HAS_EEPROM_SD
  SdFile SDCard::eeprom_file;
#endif

uint16_t  SDCard::workDirDepth  = 0,
          SDCard::nrFiles       = 0;

LsActionEnum SDCard::lsAction   = LS_Count;

// Sort files and folders alphabetically.
#if ENABLED(SDCARD_SORT_ALPHA)
  uint16_t SDCard::sort_count = 0;
  #if ENABLED(SDSORT_GCODE)
    bool  SDCard::sort_alpha    = true;
    int   SDCard::sort_folders  = FOLDER_SORTING;
    //static bool sort_reverse;      // Flag to enable / disable reverse sorting
  #endif

  // By default the sort index is static
  #if ENABLED(SDSORT_DYNAMIC_RAM)
    uint8_t *SDCard::sort_order;
  #else
    uint8_t SDCard::sort_order[SDSORT_LIMIT];
  #endif

  // Cache filenames to speed up SD menus.
  #if ENABLED(SDSORT_USES_RAM)

    // If using dynamic ram for names, allocate on the heap.
    #if ENABLED(SDSORT_CACHE_NAMES)
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        char **SDCard::sortshort, **SDCard::sortnames;
      #else
        char SDCard::sortnames[SDSORT_LIMIT][SORTED_LONGNAME_MAXLEN];
      #endif
    #elif DISABLED(SDSORT_USES_STACK)
      char SDCard::sortnames[SDSORT_LIMIT][SORTED_LONGNAME_MAXLEN];
    #endif

    // Folder sorting uses an isDir array when caching items.
    #if HAS_FOLDER_SORTING
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        uint8_t *SDCard::isDir;
      #elif ENABLED(SDSORT_CACHE_NAMES) || DISABLED(SDSORT_USES_STACK)
        uint8_t SDCard::isDir[(SDSORT_LIMIT + 7) >> 3];
      #endif
    #endif

  #endif // SDSORT_USES_RAM

#endif // SDCARD_SORT_ALPHA

#if ENABLED(ADVANCED_SD_COMMAND)

  Sd2Card   SDCard::sd;

  uint32_t  SDCard::cardSizeBlocks,
            SDCard::cardCapacityMB;

  // Cache for SD block
  cache_t   SDCard::cache;

  // MBR information
  uint8_t   SDCard::partType;
  uint32_t  SDCard::relSector,
            SDCard::partSize;

  // Fake disk geometry
  uint8_t   SDCard::numberOfHeads,
            SDCard::sectorsPerTrack;

  // FAT parameters
  uint16_t  SDCard::reservedSectors;
  uint8_t   SDCard::sectorsPerCluster;
  uint32_t  SDCard::fatStart,
            SDCard::fatSize,
            SDCard::dataStart;

#endif

/** Public Function */

void SDCard::mount() {

  if (isMounted()) return;

  if (root.isOpen()) root.close();

  if (!fat.begin(SS_PIN, SPI_SPEED)
    #if ENABLED(LCD_SDSS) && (LCD_SDSS != SS_PIN)
      && !fat.begin(LCD_SDSS, SPI_SPEED)
    #endif
  ) {
    SERIAL_LM(ER, STR_SD_INIT_FAIL);
    if (fat.card()->errorCode()) {
      SERIAL_MV("SD initialization failed.\n"
                "Do not reformat the card!\n"
                "Is the card correctly inserted?\n"
                "Is chipSelect set to the correct value?\n"
                "Does another SPI device need to be disabled?\n"
                "Is there a wiring/soldering problem?\n"
                "errorCode: ", int(fat.card()->errorCode())
                );
      SERIAL_EOL();
      return;
    }
    if (fat.vol()->fatType() == 0) {
      SERIAL_MSG("Can't find a valid FAT16/FAT32 partition.\n");
      return;
    }
    if (!fat.vwd()->isOpen()) {
      SERIAL_MSG("Can't open root directory.\n");
      return;
    }
    return;
  }
  else {
    setMounted(true);
    SERIAL_LM(ECHO, STR_SD_CARD_OK);
  }

  fat.chdir();
  root.openRoot(fat.vol());
  setroot();
  flag.WorkdirIsRoot = true;

  #if HAS_EEPROM_SD
    import_eeprom();
  #endif

  if (selectFile("init.g", true)) startFilePrint();

  lcdui.refresh();
}

void SDCard::unmount() {
  setMounted(false);
  endFilePrint();
}

void SDCard::ls() {
  setroot();
  lsRecursive(&root);
}

void SDCard::getfilename(uint16_t nr, PGM_P const match/*=nullptr*/) {
  #if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_CACHE_NAMES)
    if (match != nullptr) {
      while (nr < sort_count) {
        if (strcasecmp(match, sortshort[nr]) == 0) break;
        nr++;
      }
    }
    if (nr < sort_count) {
      strcpy(fileName, sortnames[nr]);
      setFilenameIsDir(TEST(isDir[nr>>3], nr & 0x07));
      return;
    }
  #endif // SDSORT_CACHE_NAMES
  lsAction = LS_GetFilename;
  nrFile_index = nr;
  lsDive(workDir, match);
}

void SDCard::getAbsFilename(char * name) {

  *name++ = '/';
  uint8_t cnt = 1;
  uint8_t i = 0;

  for (i = 0; i < workDirDepth; i++) {
    workDirParents[i].getName(name, LONG_FILENAME_LENGTH);
    while (*name && cnt < MAX_PATH_NAME_LENGHT) { name++; cnt++; }
    if (cnt < MAX_PATH_NAME_LENGHT) { *name = '/'; name++; cnt++; }
  }

  i = 0;
  while (fileName[i]) { *name++ = fileName[i]; i++; }
  *name = '\0';

}

void SDCard::openAndPrintFile(const char * const path) {
  char cmd[4 + strlen(path) + 1]; // Room for "M23 ", fileName, and null
  sprintf_P(cmd, M23_CMD, path);
  for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
  commands.enqueue_one_now(cmd);
  commands.enqueue_now_P(M24_CMD);
}

void SDCard::startFilePrint() {
  if (isMounted()) {
    setPrinting(true);
    #if ENABLED(SDCARD_SORT_ALPHA)
      flush_presort();
    #endif
  }
}

void SDCard::endFilePrint() {
  setPrinting(false);
  if (isFileOpen()) gcode_file.close();
}

void SDCard::write_command(char* buf) {
  char* begin = buf;
  char* npos = 0;
  char* end = buf + strlen(buf) - 1;

  gcode_file.clearWriteError();
  if ((npos = strchr(buf, 'N')) != NULL) {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  end[3] = '\0';
  gcode_file.write(begin);
  if (gcode_file.getWriteError()) {
    SERIAL_LM(ER, STR_SD_ERR_WRITE_TO_FILE);
  }
}

void SDCard::print_status() {
  if (isPrinting()) {
    SERIAL_MV(STR_SD_PRINTING_BYTE, sdpos);
    SERIAL_EMV(STR_SD_SLASH, fileSize);
  }
  else
    SERIAL_EM(STR_SD_NOT_PRINTING);
}

void SDCard::startWrite(const char * const path, const bool silent/*=false*/) {
  if (!isMounted()) return;

  fat.chdir();
  if (gcode_file.open(path, FILE_WRITE)) {
    setSaving(true);
    #if ENABLED(EMERGENCY_PARSER)
      emergency_parser.disable();
    #endif
    if (!silent) {
      SERIAL_EMT(STR_SD_WRITE_TO_FILE, path);
      lcdui.set_status(path);
    }
  }
  else openFailed(path);
}

void SDCard::deleteFile(const char * const path) {
  if (!isMounted()) return;
  endFilePrint();
  gcode_file.close();
  if (fat.remove(path)) {
    SERIAL_EMT(STR_SD_FILE_DELETED, path);
  }
  else {
    if (fat.rmdir(path)) {
      SERIAL_EMT(STR_SD_FILE_DELETED, path);
      sdpos = 0;
      #if ENABLED(SDCARD_SORT_ALPHA)
        presort();
      #endif
    }
    else
      SERIAL_EM(STR_SD_FILE_DELETION_ERR);
  }
}

void SDCard::finishWrite() {
  gcode_file.sync();
  gcode_file.close();
  setSaving(false);
  SERIAL_EM(STR_SD_FILE_SAVED);
}

void SDCard::makeDirectory(const char * const path) {
  if (!isMounted()) return;
  endFilePrint();
  gcode_file.close();
  if (fat.mkdir(path)) {
    SERIAL_EM(STR_SD_DIRECTORY_CREATED);
  }
  else {
    SERIAL_EM(STR_SD_CREATION_FAILED);
  }
}

void SDCard::closeFile() {
  gcode_file.sync();
  gcode_file.close();
  setSaving(false);
  #if ENABLED(EMERGENCY_PARSER)
    emergency_parser.enable();
  #endif
}

void SDCard::fileHasFinished() {
  planner.synchronize();
  endFilePrint();
  #if ENABLED(SDCARD_SORT_ALPHA)
    presort();
  #endif
  flag.PrintComplete = true;
}

void SDCard::chdir(const char * const relpath) {
  SdFile newDir;
  SdFile *parent = workDir.isOpen() ? &workDir : &root;

  if (!newDir.open(parent, relpath, O_READ)) {
    SERIAL_LMT(ECHO, STR_SD_CANT_ENTER_SUBDIR, relpath);
  }
  else {
    workDir = newDir;
    flag.WorkdirIsRoot = false;
    if (workDirDepth < SD_MAX_FOLDER_DEPTH)
      workDirParents[workDirDepth++] = workDir;
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
}

void SDCard::beginautostart() {
  autostart_index = 0;
  setroot();
}

void SDCard::checkautostart() {
  /*
  if (autostart_index < 0 || isPrinting()) return;

  if (!isMounted()) mount();
  
  if (isMounted()
    #if HAS_SD_RESTART
      && !restart.valid() // Don't run auto#.g when a restart file exists
    #endif
  ) {
    char autoname[10];
    sprintf_P(autoname, PSTR("auto%i.g"), autostart_index);
    dir_t p;
    root.rewind();
    while (root.readDir(&p, NULL) > 0) {
      for (int8_t i = (int8_t)strlen((char*)p.name); i--;) p.name[i] = tolower(p.name[i]);
      if (p.name[9] != '~' && strncmp((char*)p.name, autoname, 5) == 0) {
        openAndPrintFile(autoname);
        autostart_index++;
        return;
      }
    }
  }
  autostart_index = -1;
  */
}

void SDCard::setroot() {
  workDir = root;
  flag.WorkdirIsRoot = true;
  #if ENABLED(SDCARD_SORT_ALPHA)
    presort();
  #endif
}

void SDCard::printEscapeChars(PGM_P s) {
  for (unsigned int i = 0; i < strlen(s); ++i) {
    switch (s[i]) {
      case '"':
      case '/':
      case '\b':
      case '\f':
      case '\n':
      case '\r':
      case '\t':
      case '\\':
      SERIAL_CHR('\\');
      break;
    }
    SERIAL_CHR(s[i]);
  }
}

bool SDCard::selectFile(const char * const path, const bool silent/*=false*/) {
  const char * fname = path;

  if (!isMounted()) return false;

  gcode_file.close();
  if (gcode_file.open(&workDir, path, O_READ)) {
    if ((fname = strrchr(path, '/')) != NULL)
      fname++;
    else
      fname = path;

    fileSize = gcode_file.fileSize();
    sdpos = 0;

    if (!silent) {
      SERIAL_MT(STR_SD_FILE_OPENED, fname);
      SERIAL_EMV(STR_SD_SIZE, fileSize);
    }

    for (uint16_t c = 0; c < sizeof(fileName); c++)
      const_cast<char&>(fileName[c]) = '\0';
    strncpy(fileName, path, strlen(path));

    #if ENABLED(JSON_OUTPUT)
      parsejson(gcode_file);
    #endif

    return true;
  }
  else {
    if (!silent) openFailed(fname);
    return false;
  }
}

int8_t SDCard::updir() {
  if (workDirDepth > 0) {                                               // At least 1 dir has been saved
    workDir = --workDirDepth ? workDirParents[workDirDepth - 1] : root; // Use parent, or root if none
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
  if (!workDirDepth) flag.WorkdirIsRoot = true;
  return workDirDepth;
}

uint16_t SDCard::getnrfilenames() {
  lsAction = LS_Count;
  nrFiles = 0;
  lsDive(workDir);
  return nrFiles;
}

uint16_t SDCard::get_num_Files() {
  return
    #if ENABLED(SDCARD_SORT_ALPHA) && SDSORT_USES_RAM && SDSORT_CACHE_NAMES
      nrFiles // no need to access the SD card for filenames
    #else
      getnrfilenames()
    #endif
  ;
}

#if HAS_SD_RESTART

  constexpr char restart_file_name[8] = "restart";

  void SDCard::open_restart_file(const bool read) {

    if (!isMounted() || restart.job_file.isOpen()) return;

    if (!restart.job_file.open(fat.vwd(), restart_file_name, read ? O_READ : (O_RDWR | O_CREAT | O_SYNC)))
      openFailed(restart_file_name);
    else if (!read) {
      if (printer.debugFeature()) DEBUG_EMT(STR_SD_WRITE_TO_FILE, restart_file_name);
    }
  }

  void SDCard::delete_restart_file() {
    if (exist_restart_file()) {
      restart.job_file.remove(fat.vwd(), restart_file_name);
      if (printer.debugFeature()) {
        DEBUG_SM(DEB, " File restart delete");
        DEBUG_STR(exist_restart_file() ? PSTR(" failed.\n") : PSTR("d.\n"));
      }
    }
  }

  bool SDCard::exist_restart_file() {
    const bool exist = restart.job_file.open(fat.vwd(), restart_file_name, O_READ);
    if (exist) restart.job_file.close();
    return exist;
  }

#endif

#if HAS_EEPROM_SD

  void SDCard::import_eeprom() {
    if (!eeprom_file.open(EEPROM_FILE_NAME, O_READ) ||
        !eeprom_file.read(memorystore.eeprom_data, EEPROM_SIZE) == EEPROM_SIZE
    ) openFailed(EEPROM_FILE_NAME);
    else SERIAL_LM(ECHO, "EEPROM read from sd card.");
    eeprom_file.close();
  }

  void SDCard::write_eeprom() {
    if (!isMounted()) {
      SERIAL_LM(ER, "No SD Card");
      return;
    }

    if (!eeprom_file.open(EEPROM_FILE_NAME, O_RDWR | O_CREAT | O_SYNC) ||
        !eeprom_file.seekSet(0) ||
        !eeprom_file.write(memorystore.eeprom_data, EEPROM_SIZE) == EEPROM_SIZE
    ) SERIAL_LM(ER, "Could not write eeprom to sd card");

    eeprom_file.close();
  }

#endif

#if ENABLED(SDCARD_SORT_ALPHA)

  /**
   * Get the name of a file in the current directory by sort-index
   */
  void SDCard::getfilename_sorted(const uint16_t nr) {
    getfilename(
      #if ENABLED(SDSORT_GCODE)
        sort_alpha &&
      #endif
      (nr < sort_count) ? sort_order[nr] : nr
    );
  }

  /**
   * Read all the files and produce a sort key
   *
   * We can do this in 3 ways...
   *  - Minimal RAM: Read two filenames at a time sorting along...
   *  - Some RAM: Buffer the directory just for this sort
   *  - Most RAM: Buffer the directory and return filenames from RAM
   */
  void SDCard::presort() {

    // Sorting may be turned off
    #if ENABLED(SDSORT_GCODE)
      if (!sort_alpha) return;
    #endif

    // Throw away old sort index
    flush_presort();

    // If there are files, sort up to the limit
    uint16_t fileCnt = getnrfilenames();
    if (fileCnt > 0) {

      // Never sort more than the max allowed
      // If you use folders to organize, 20 may be enough
      NOMORE(fileCnt, uint16_t(SDSORT_LIMIT));

      // Sort order is always needed. May be static or dynamic.
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        sort_order = new uint8_t[fileCnt];
      #endif

      // Use RAM to store the entire directory during pre-sort.
      // SDSORT_LIMIT should be set to prevent over-allocation.
      #if ENABLED(SDSORT_USES_RAM)

        // If using dynamic ram for names, allocate on the heap.
        #if ENABLED(SDSORT_CACHE_NAMES)
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            sortnames = new char*[fileCnt];
          #endif
        #elif ENABLED(SDSORT_USES_STACK)
          char sortnames[fileCnt][SORTED_LONGNAME_MAXLEN];
        #endif

        // Folder sorting needs 1 bit per entry for flags.
        #if HAS_FOLDER_SORTING
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            isDir = new uint8_t[(fileCnt + 7) >> 3];
          #elif ENABLED(SDSORT_USES_STACK)
            uint8_t isDir[(fileCnt + 7) >> 3];
          #endif
        #endif

      #else // !SDSORT_USES_RAM

        // By default re-read the names from SD for every compare
        // retaining only two filenames at a time. This is very
        // slow but is safest and uses minimal RAM.
        char name1[LONG_FILENAME_LENGTH + 1];

      #endif

      if (fileCnt > 1) {

        // Init sort order.
        for (uint16_t i = 0; i < fileCnt; i++) {
          sort_order[i] = i;
          // If using RAM then read all filenames now.
          #if ENABLED(SDSORT_USES_RAM)
            getfilename(i);
            #if ENABLED(SDSORT_DYNAMIC_RAM)
              // Use dynamic method to copy long fileName
              sortnames[i] = strdup(fileName);
            #else
              // Copy filenames into the static array
              #if SORTED_LONGNAME_MAXLEN != LONG_FILENAME_LENGTH
                strncpy(sortnames[i], fileName, SORTED_LONGNAME_MAXLEN);
                sortnames[i][SORTED_LONGNAME_MAXLEN - 1] = '\0';
              #else
                strcpy(sortnames[i], SORTED_LONGNAME_MAXLEN);
              #endif
            #endif
            // char out[30];
            // sprintf_P(out, PSTR("---- %i %s %s"), i, isFilenameIsDir( ? "D" : " ", sortnames[i]);
            // SERIAL_ECHOLN(out);
            #if HAS_FOLDER_SORTING
              const uint16_t bit = i & 0x07, ind = i >> 3;
              if (bit == 0) isDir[ind] = 0x00;
              if (isFilenameIsDir()) isDir[ind] |= _BV(bit);
            #endif
          #endif
        }

        // Bubble Sort
        for (uint16_t i = fileCnt; --i;) {
          bool didSwap = false;
          for (uint16_t j = 0; j < i; ++j) {
            const uint16_t o1 = sort_order[j], o2 = sort_order[j + 1];

            // Compare names from the array or just the two buffered names
            #if ENABLED(SDSORT_USES_RAM)
              #define _SORT_CMP_NODIR() (strcasecmp(sortnames[o1], sortnames[o2]) > 0)
            #else
              #define _SORT_CMP_NODIR() (strcasecmp(name1, name2) > 0)
            #endif

            #if HAS_FOLDER_SORTING
              #if ENABLED(SDSORT_USES_RAM)
                // Folder sorting needs an index and bit to test for folder-ness.
                const uint8_t ind1 = o1 >> 3, bit1 = o1 & 0x07,
                              ind2 = o2 >> 3, bit2 = o2 & 0x07;
                #define _SORT_CMP_DIR(fs) \
                  (((isDir[ind1] & _BV(bit1)) != 0) == ((isDir[ind2] & _BV(bit2)) != 0) \
                    ? _SORT_CMP_NODIR() \
                    : (isDir[fs > 0 ? ind1 : ind2] & (fs > 0 ? _BV(bit1) : _BV(bit2))) != 0)
              #else
                #define _SORT_CMP_DIR(fs) ((dir1 == isFilenameIsDir()) ? _SORT_CMP_NODIR() : (fs > 0 ? dir1 : !dir1))
              #endif
            #endif

            // The most economical method reads names as-needed
            // throughout the loop. Slow if there are many.
            #if DISABLED(SDSORT_USES_RAM)
              getfilename(o1);
              strcpy(name1, fileName); // save (or getfilename below will trounce it)
              #if HAS_FOLDER_SORTING
                bool dir1 = isFilenameIsDir();
              #endif
              getfilename(o2);
              char *name2 = fileName; // use the string in-place
            #endif // !SDSORT_USES_RAM

            // Sort the current pair according to settings.
            if (
              #if HAS_FOLDER_SORTING
                #if ENABLED(SDSORT_GCODE)
                  sort_folders ? _SORT_CMP_DIR(sort_folders) : _SORT_CMP_NODIR()
                #else
                  _SORT_CMP_DIR(FOLDER_SORTING)
                #endif
              #else
                _SORT_CMP_NODIR()
              #endif
            ) {
              sort_order[j] = o2;
              sort_order[j + 1] = o1;
              didSwap = true;
            }
          }
          if (!didSwap) break;
        }
        // Using RAM but not keeping names around
        #if ENABLED(SDSORT_USES_RAM) && DISABLED(SDSORT_CACHE_NAMES)
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            for (uint16_t i = 0; i < fileCnt; ++i) free(sortnames[i]);
            #if HAS_FOLDER_SORTING
              free(isDir);
            #endif
          #endif
        #endif
      }
      else {
        sort_order[0] = 0;
        #if ENABLED(SDSORT_USES_RAM) && ENABLED(SDSORT_CACHE_NAMES)
          getfilename(0);
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            sortnames = new char*[1];
            sortnames[0] = strdup(fileName); // malloc
            isDir = new uint8_t[1];
          #else
            #if SORTED_LONGNAME_MAXLEN != LONG_FILENAME_LENGTH
              strncpy(sortnames[0], fileName, SORTED_LONGNAME_MAXLEN);
              sortnames[0][SORTED_LONGNAME_MAXLEN - 1] = '\0';
            #else
              strcpy(sortnames[0], SORTED_LONGNAME_MAXLEN);
            #endif
          #endif
          isDir[0] = isFilenameIsDir() ? 0x01 : 0x00;
        #endif
      }

      sort_count = fileCnt;
    }
  }

  void SDCard::flush_presort() {
    if (sort_count > 0) {
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        delete sort_order;
      #endif
      sort_count = 0;
    }
  }

#endif // SDCARD_SORT_ALPHA

#if ENABLED(ADVANCED_SD_COMMAND)

  void SDCard::formatSD() {

    card.unmount();

    if (!sd.begin(SS_PIN, SPI_SPEED)) {
      SERIAL_LM(ER, "SD initialization failure!");
      return;
    }

    cardSizeBlocks = sd.cardSize();
    if (cardSizeBlocks == 0) {
      SERIAL_LM(ER, "cardSize");
      return;
    }

    cardCapacityMB = (cardSizeBlocks + 2047) / 2048;

    SERIAL_SMV(ECHO, " Card Size:", (int)(1.048576 * cardCapacityMB));
    SERIAL_EM(" MB, (MB = 1,000,000 bytes)");
    SERIAL_LM(ECHO, "Formatting... ");

    initSizes();

    if (sd.type() != SD_CARD_TYPE_SDHC) {
      SERIAL_LM(ECHO, "FAT16");
      makeFat16();
    }
    else {
      SERIAL_LM(ECHO, "FAT32");
      makeFat32();
    }

    SERIAL_LM(ECHO, "Format done.");
  }

  void SDCard::infoSD() {

    cardSizeBlocks = fat.card()->cardSize();
    if (cardSizeBlocks == 0) {
      SERIAL_LM(ER, "cardSize failed");
      return;
    }

    SERIAL_SM(ECHO, "SD Card type:");
    switch (fat.card()->type()) {
      case SD_CARD_TYPE_SD1:
        SERIAL_EM("SD1");
        break;
      case SD_CARD_TYPE_SD2:
        SERIAL_EM("SD2");
        break;
      case SD_CARD_TYPE_SDHC:
        if (cardSizeBlocks < 70000000)
          SERIAL_EM("SDHC");
        else
          SERIAL_EM("SDXC");
        break;
      default:
        SERIAL_EM("Unknown");
    }

    if (!cidDmp()) return;
    if (!csdDmp()) return;

    volDmp();
  }

#endif

/** Private Function */
void SDCard::openFailed(const char * const path) {
  SERIAL_LMT(ER, STR_SD_OPEN_FILE_FAIL, path);
}

void SDCard::lsRecursive(FatFile *dir, uint8_t level/*=0*/) {

  FatFile file;

  if (!dir->isDir() || dir->getError()) return;

  dir->rewind();

  while (file.openNext(dir, O_READ)) {
    file.getName(tempLongFilename, LONG_FILENAME_LENGTH);
    if (file.isHidden()) {
      file.close();
      continue;
    }
    // if (! (file.isFile() || file.isDir())) continue;
    if (strcmp(tempLongFilename, "..") == 0) {
      file.close();
      continue;
    }
    if (tempLongFilename[0] == '.') {
      file.close();
      continue; // MAC CRAP
    }
    if (file.isDir()) {
      if (level >= SD_MAX_FOLDER_DEPTH) {
        file.close();
        continue; // can't go deeper
      }
      if (level) {
        SERIAL_TXT(fileName);
        SERIAL_CHR('/');
      }
      SERIAL_TXT(card.tempLongFilename);
      SERIAL_CHR('/');
      SERIAL_EOL();

      char* tmp;
      // Add directory name
      if (level) strcat(fileName, "/");
      strcat(fileName, tempLongFilename);
      lsRecursive(&file, level + 1);

      // remove added directory name
      if ((tmp = strrchr(fileName, '/')) != NULL)
        *tmp = 0;
      else
        *fileName = 0;
    }
    else { // is filename
      if (level) {
        SERIAL_TXT(fileName);
        SERIAL_CHR('/');
      }
      SERIAL_TXT(tempLongFilename);
      #ifdef SD_EXTENDED_DIR
        SERIAL_MV(" ", (long)file.fileSize());
      #endif
      SERIAL_EOL();
    }
    file.close();
  }

}

/**
 * Dive into a folder and recurse depth-first to perform a pre-set operation lsAction:
 *   LS_Count       - Add +1 to nrFiles for every file within the parent
 *   LS_GetFilename - Get the fileName of the file indexed by nrFile_index
 */
void SDCard::lsDive(SdFile parent, PGM_P const match/*=NULL*/) {
  //dir_t* p = NULL;
  SdFile file;
  parent.rewind();
  uint8_t cnt = 0;

  // Read the next entry from a directory
  while (file.openNext(&parent, O_READ)) {
    file.getName(tempLongFilename, LONG_FILENAME_LENGTH);

    if (workDirDepth >= SD_MAX_FOLDER_DEPTH && strcmp(tempLongFilename, "..") == 0) {
      file.close();
      continue;
    }
    if (tempLongFilename[0] == '.' && tempLongFilename[1] != '.') {
      file.close();
      continue; // MAC CRAP
    }

    if (!(file.isFile() || file.isSubDir()) || file.isHidden()) {
      file.close();
      continue;
    }

    setFilenameIsDir(file.isSubDir());

    switch (lsAction) {
      case LS_Count:
        nrFiles++;
        file.close();
        break;
      case LS_GetFilename:
        if (match != NULL && strcasecmp(match, tempLongFilename) == 0) {
          strcpy(fileName, tempLongFilename);
          file.close();
          return;
        }
        else if (cnt == nrFile_index) {
          strcpy(fileName, tempLongFilename);
          file.close();
          return;
        }
        cnt++;
        file.close();
        break;
    }

  } // while readDir
}

// --------------------------------------------------------------- //
// Code that gets gcode information is adapted from RepRapFirmware //
// Originally licenced under GPL                                   //
// Authors: reprappro, dc42, dcnewman, others                      //
// Source: https://github.com/dcnewman/RepRapFirmware              //
// Copy date: 27 FEB 2016                                          //
// --------------------------------------------------------------- //
void SDCard::parsejson(SdFile &parser_file) {
  fileSize = parser_file.fileSize();
  filamentNeeded    = 0.0;
  objectHeight      = 0.0;
  firstlayerHeight  = 0.0;
  layerHeight       = 0.0;

  if (!parser_file.isOpen()) return;

  bool genByFound = false, firstlayerHeightFound = false, layerHeightFound = false, filamentNeedFound = false;

  #if ENABLED(__AVR__)
    #define GCI_BUF_SIZE 120
  #else
    #define GCI_BUF_SIZE 1024
  #endif

  // READ 4KB FROM THE BEGINNING
  char buf[GCI_BUF_SIZE];
  for (int i = 0; i < 4096; i += GCI_BUF_SIZE - 50) {
    if(!parser_file.seekSet(i)) break;
    parser_file.read(buf, GCI_BUF_SIZE);
    if (!genByFound && findGeneratedBy(buf, generatedBy)) genByFound = true;
    if (!firstlayerHeightFound && findFirstLayerHeight(buf, firstlayerHeight)) firstlayerHeightFound = true;
    if (!layerHeightFound && findLayerHeight(buf, layerHeight)) layerHeightFound = true;
    if (!filamentNeedFound && findFilamentNeed(buf, filamentNeeded)) filamentNeedFound = true;
    if(genByFound && layerHeightFound && filamentNeedFound) goto get_objectHeight;
  }

  // READ 4KB FROM END
  for (int i = 0; i < 4096; i += GCI_BUF_SIZE - 50) {
    if(!parser_file.seekEnd(-4096 + i)) break;
    parser_file.read(buf, GCI_BUF_SIZE);
    if (!genByFound && findGeneratedBy(buf, generatedBy)) genByFound = true;
    if (!firstlayerHeightFound && findFirstLayerHeight(buf, firstlayerHeight)) firstlayerHeightFound = true;
    if (!layerHeightFound && findLayerHeight(buf, layerHeight)) layerHeightFound = true;
    if (!filamentNeedFound && findFilamentNeed(buf, filamentNeeded)) filamentNeedFound = true;
    if(genByFound && layerHeightFound && filamentNeedFound) goto get_objectHeight;
  }

  get_objectHeight:
  // MOVE FROM END UP IN 1KB BLOCKS UP TO 30KB
  for (int i = GCI_BUF_SIZE; i < 30000; i += GCI_BUF_SIZE - 50) {
    if(!parser_file.seekEnd(-i)) break;
    parser_file.read(buf, GCI_BUF_SIZE);
    if (findTotalHeight(buf, objectHeight)) break;
  }
  parser_file.rewind();
}

bool SDCard::findGeneratedBy(char* buf, char* genBy) {
  // Slic3r & S3D
  PGM_P generatedByString = PSTR("generated by ");
  char* pos = strstr_P(buf, generatedByString);
  if (pos) {
    pos += strlen_P(generatedByString);
    size_t i = 0;
    while (i < GENBY_SIZE - 1 && *pos >= ' ') {
      char c = *pos++;
      if (c == '"' || c == '\\') {
        // Need to escape the quote-mark for JSON
        if (i > GENBY_SIZE - 3) break;
        genBy[i++] = '\\';
      }
      genBy[i++] = c;
    }
    genBy[i] = 0;
    return true;
  }

  // CURA
  PGM_P slicedAtString = PSTR(";Sliced at: ");
  pos = strstr_P(buf, slicedAtString);
  if (pos) {
    strcpy_P(genBy, PSTR("Cura"));
    return true;
  }

  // UNKNOWN
  strcpy_P(genBy, PSTR("Unknown"));
  return false;
}

bool SDCard::findFirstLayerHeight(char* buf, float &firstlayerHeight) {
  // SLIC3R
  firstlayerHeight = 0;
  PGM_P layerHeightSlic3r = PSTR("; first_layer_height ");
  char *pos = strstr_P(buf, layerHeightSlic3r);
  if (pos) {
    pos += strlen_P(layerHeightSlic3r);
    while (*pos == ' ' || *pos == 't' || *pos == '=' || *pos == ':') {
      ++pos;
    }
    firstlayerHeight = strtod(pos, NULL);
    return true;
  }

  // CURA
  PGM_P layerHeightCura = PSTR("First layer height: ");
  pos = strstr_P(buf, layerHeightCura);
  if (pos) {
    pos += strlen_P(layerHeightCura);
    while (*pos == ' ' || *pos == 't' || *pos == '=' || *pos == ':') {
      ++pos;
    }
    firstlayerHeight = strtod(pos, NULL);
    return true;
  }

  return false;
}

bool SDCard::findLayerHeight(char* buf, float &layerHeight) {
  // SLIC3R
  layerHeight = 0;
  PGM_P layerHeightSlic3r = PSTR("; layer_height ");
  char *pos = strstr_P(buf, layerHeightSlic3r);
  if (pos) {
    pos += strlen_P(layerHeightSlic3r);
    while (*pos == ' ' || *pos == 't' || *pos == '=' || *pos == ':') {
      ++pos;
    }
    layerHeight = strtod(pos, NULL);
    return true;
  }

  // CURA
  PGM_P layerHeightCura = PSTR("Layer height: ");
  pos = strstr_P(buf, layerHeightCura);
  if (pos) {
    pos += strlen_P(layerHeightCura);
    while (*pos == ' ' || *pos == 't' || *pos == '=' || *pos == ':') {
      ++pos;
    }
    layerHeight = strtod(pos, NULL);
    return true;
  }

  return false;
}

bool SDCard::findFilamentNeed(char* buf, float &filament) {
  PGM_P filamentUsedStr = PSTR("filament used");
  PGM_P pos = strstr_P(buf, filamentUsedStr);
  filament = 0;
  if (pos != NULL) {
    pos += strlen_P(filamentUsedStr);
    while (*pos == ' ' || *pos == 't' || *pos == '=' || *pos == ':') {
      ++pos;    // this allows for " = " from default slic3r comment and ": " from default Cura comment
    }
    if (isDigit(*pos)) {
      char *q;
      filament += strtod(pos, &q);
      if (*q == 'm' && *(q + 1) != 'm') {
        filament *= 1000.0;        // Cura outputs filament used in metres not mm
      }
    }
    return true;
  }
  return false;
}

bool SDCard::findTotalHeight(char* buf, float &height) {
  int len = 1024;
  bool inComment, inRelativeMode = false;
  unsigned int zPos;
  for (int i = len - 5; i > 0; i--) {
    if (inRelativeMode) {
      inRelativeMode = !(buf[i] == 'G' && buf[i + 1] == '9' && buf[i + 2] == '1' && buf[i + 3] <= ' ');
    }
    else if (buf[i] == 'G') {
      // Ignore G0/G1 codes if absolute mode was switched back using G90 (typical for Cura files)
      if (buf[i + 1] == '9' && buf[i + 2] == '0' && buf[i + 3] <= ' ') {
        inRelativeMode = true;
      }
      else if ((buf[i + 1] == '0' || buf[i + 1] == '1') && buf[i + 2] == ' ') {
        // Look for last "G0/G1 ... Z#HEIGHT#" command as generated by common slicers
        // Looks like we found a controlled move, however it could be in a comment, especially when using slic3r 1.1.1
        inComment = false;
        size_t j = i;
        while (j != 0) {
          --j;
          char c = buf[j];
          if (c == '\n' || c == '\r') break;
          if (c == ';') {
            // It is in a comment, so give up on this one
            inComment = true;
            break;
          }
        }
        if (inComment) continue;

        // Find 'Z' position and grab that value
        zPos = 0;
        for (int j = i + 3; j < len - 2; j++) {
          char c = buf[j];
          if (c < ' ') {
            // Skip all whitespaces...
            while (j < len - 2 && c <= ' ') {
              c = buf[++j];
            }
            // ...to make sure ";End" doesn't follow G0 .. Z#HEIGHT#
            if (zPos != 0) {
              //debugPrintf("Found at offset %u text: %.100s\n", zPos, &buf[zPos + 1]);
              height = strtod(&buf[zPos + 1], NULL);
              return true;
            }
            break;
          }
          else if (c == ';') break;
          else if (c == 'Z') zPos = j;
        }
      }
    }
  }
  return false;
}

#if ENABLED(ADVANCED_SD_COMMAND)

  uint8_t SDCard::cidDmp() {
    cid_t cid;
    if (!fat.card()->readCID(&cid)) {
      SERIAL_LM(ER, "readCID failed");
      return false;
    }

    SERIAL_LMV(ECHO, "Manufacturer ID:", int(cid.mid));
    SERIAL_SM(ECHO, "OEM ID:");
    SERIAL_VAL(cid.oid[0]);
    SERIAL_VAL(cid.oid[1]);
    SERIAL_EOL();

    SERIAL_SM(ECHO, "Product:");
    for (uint8_t i = 0; i < 5; i++) SERIAL_VAL(cid.pnm[i]);
    SERIAL_EOL();

    SERIAL_SM(ECHO, "Version:");
    SERIAL_VAL(int(cid.prv_n));
    SERIAL_CHR('.');
    SERIAL_VAL(int(cid.prv_m));
    SERIAL_EOL();

    SERIAL_SM(ECHO, "Serial number:");
    SERIAL_VAL(cid.psn);
    SERIAL_EOL();

    SERIAL_SM(ECHO, "Manufacturing date:");
    SERIAL_VAL(int(cid.mdt_month));
    SERIAL_CHR('/');
    SERIAL_VAL(2000 + cid.mdt_year_low + 10 * cid.mdt_year_high);
    SERIAL_EOL();

    return true;
  }

  uint8_t SDCard::csdDmp() {
    csd_t csd;
    uint32_t eraseSize;
    if (!fat.card()->readCSD(&csd)) {
      SERIAL_LM(ER, "readCSD failed");
      return false;
    }

    if (csd.v1.csd_ver == 0)
      eraseSize = (csd.v1.sector_size_high << 1) | csd.v1.sector_size_low;
    else if (csd.v2.csd_ver == 1)
      eraseSize = (csd.v2.sector_size_high << 1) | csd.v2.sector_size_low;
    else {
      SERIAL_LM(ER, "csd version error\n");
      return false;
    }

    eraseSize++;
    SERIAL_SMV(ECHO, "CardSize:", int(0.000512 * cardSizeBlocks));
    SERIAL_EM(" MB (MB = 1,000,000 bytes)");

    SERIAL_SM(ECHO, "flashEraseSize:");
    SERIAL_VAL(int(eraseSize));
    SERIAL_MSG(" blocks");
    SERIAL_EOL();

    return true;
  }

  void SDCard::volDmp() {
    SERIAL_LMV(ECHO, "Volume is FAT", int(fat.vol()->fatType()));
    SERIAL_LMV(ECHO, "blocksPerCluster:", int(fat.vol()->blocksPerCluster()));
    SERIAL_LMV(ECHO, "clusterCount:", fat.vol()->clusterCount());
    SERIAL_SM(ECHO, "freeClusters:");
    uint32_t volFree = fat.vol()->freeClusterCount();
    SERIAL_VAL(volFree);
    SERIAL_EOL();
    uint32_t fs = 0.000512 * volFree * fat.vol()->blocksPerCluster();
    SERIAL_SMV(ECHO, "freeSpace:", fs);
    SERIAL_MSG(" MB (MB = 1,000,000 bytes)");
    SERIAL_EOL();
  }

  void SDCard::initSizes() {
    if (cardCapacityMB <= 6) {
      SERIAL_LM(ER, "Card is too small.");
    } else if (cardCapacityMB <= 16) {
      sectorsPerCluster = 2;
    } else if (cardCapacityMB <= 32) {
      sectorsPerCluster = 4;
    } else if (cardCapacityMB <= 64) {
      sectorsPerCluster = 8;
    } else if (cardCapacityMB <= 128) {
      sectorsPerCluster = 16;
    } else if (cardCapacityMB <= 1024) {
      sectorsPerCluster = 32;
    } else if (cardCapacityMB <= 32768) {
      sectorsPerCluster = 64;
    } else {
      // SDXC cards
      sectorsPerCluster = 128;
    }

    SERIAL_LMV(ECHO, "Blocks/Cluster:", int(sectorsPerCluster));

    // set fake disk geometry
    sectorsPerTrack = cardCapacityMB <= 256 ? 32 : 63;

    if (cardCapacityMB <= 16) {
      numberOfHeads = 2;
    } else if (cardCapacityMB <= 32) {
      numberOfHeads = 4;
    } else if (cardCapacityMB <= 128) {
      numberOfHeads = 8;
    } else if (cardCapacityMB <= 504) {
      numberOfHeads = 16;
    } else if (cardCapacityMB <= 1008) {
      numberOfHeads = 32;
    } else if (cardCapacityMB <= 2016) {
      numberOfHeads = 64;
    } else if (cardCapacityMB <= 4032) {
      numberOfHeads = 128;
    } else {
      numberOfHeads = 255;
    }
  }

  void SDCard::clearCache(uint8_t addSig) {
    memset(&cache, 0, sizeof(cache));
    if (addSig) {
      cache.mbr.mbrSig0 = BOOTSIG0;
      cache.mbr.mbrSig1 = BOOTSIG1;
    }
  }

  void SDCard::clearFatDir(uint32_t bgn, uint32_t count) {
    clearCache(false);
    if (!sd.writeStart(bgn, count)) {
      SERIAL_LM(ER, "Clear FAT/DIR writeStart failed");
    }
    for (uint32_t i = 0; i < count; i++) {
      if (!sd.writeData(cache.data)) {
        SERIAL_LM(ER, "Clear FAT/DIR writeData failed");
      }
    }
    if (!sd.writeStop()) {
      SERIAL_LM(ER, "Clear FAT/DIR writeStop failed");
    }
  }

  void SDCard::writeMbr() {
    clearCache(true);
    part_t* p = cache.mbr.part;
    p->boot = 0;
    uint16_t c = lbnToCylinder(relSector);
    if (c > 1023) {
      SERIAL_EM("MBR CHS");
    }
    p->beginCylinderHigh = c >> 8;
    p->beginCylinderLow = c & 0XFF;
    p->beginHead = lbnToHead(relSector);
    p->beginSector = lbnToSector(relSector);
    p->type = partType;
    uint32_t endLbn = relSector + partSize - 1;
    c = lbnToCylinder(endLbn);
    if (c <= 1023) {
      p->endCylinderHigh = c >> 8;
      p->endCylinderLow = c & 0XFF;
      p->endHead = lbnToHead(endLbn);
      p->endSector = lbnToSector(endLbn);
    } else {
      // Too big flag, c = 1023, h = 254, s = 63
      p->endCylinderHigh = 3;
      p->endCylinderLow = 255;
      p->endHead = 254;
      p->endSector = 63;
    }
    p->firstSector = relSector;
    p->totalSectors = partSize;
    if (!writeCache(0)) {
      SERIAL_EM("write MBR");
    }
  }

  void SDCard::makeFat16() {

    char sdName[]   = "PRINTER3D";
    char fat16str[] = "FAT16   ";
    uint32_t nc;

    for (dataStart = 2 * BU16;; dataStart += BU16) {
      nc = (cardSizeBlocks - dataStart) / sectorsPerCluster;
      fatSize = (nc + 2 + 255)/256;
      uint32_t r = BU16 + 1 + 2 * fatSize + 32;
      if (dataStart < r) {
        continue;
      }
      relSector = dataStart - r + BU16;
      break;
    }
    // check valid cluster count for FAT16 volume
    if (nc < 4085 || nc >= 65525) {
      SERIAL_EM("Bad cluster count");
    }
    reservedSectors = 1;
    fatStart = relSector + reservedSectors;
    partSize = nc * sectorsPerCluster + 2 * fatSize + reservedSectors + 32;
    if (partSize < 32680) {
      partType = 0X01;
    } else if (partSize < 65536) {
      partType = 0X04;
    } else {
      partType = 0X06;
    }
    // write MBR
    writeMbr();
    clearCache(true);
    fat_boot_t* pb = &cache.fbs;
    pb->jump[0] = 0XEB;
    pb->jump[1] = 0X00;
    pb->jump[2] = 0X90;
    for (uint8_t i = 0; i < sizeof(pb->oemId); i++) {
      pb->oemId[i] = ' ';
    }
    pb->bytesPerSector = 512;
    pb->sectorsPerCluster = sectorsPerCluster;
    pb->reservedSectorCount = reservedSectors;
    pb->fatCount = 2;
    pb->rootDirEntryCount = 512;
    pb->mediaType = 0XF8;
    pb->sectorsPerFat16 = fatSize;
    pb->sectorsPerTrack = sectorsPerTrack;
    pb->headCount = numberOfHeads;
    pb->hidddenSectors = relSector;
    pb->totalSectors32 = partSize;
    pb->driveNumber = 0X80;
    pb->bootSignature = EXTENDED_BOOT_SIG;
    pb->volumeSerialNumber = volSerialNumber();
    memcpy(pb->volumeLabel, sdName, sizeof(pb->volumeLabel));
    memcpy(pb->fileSystemType, fat16str, sizeof(pb->fileSystemType));
    // write partition boot sector
    if (!writeCache(relSector)) {
      SERIAL_EM("FAT16 write PBS failed");
    }
    // clear FAT and root directory
    clearFatDir(fatStart, dataStart - fatStart);
    clearCache(false);
    cache.fat16[0] = 0XFFF8;
    cache.fat16[1] = 0XFFFF;
    // write first block of FAT and backup for reserved clusters
    if (!writeCache(fatStart)
        || !writeCache(fatStart + fatSize)) {
      SERIAL_EM("FAT16 reserve failed");
    }
  }

  // format the SD as FAT32
  void SDCard::makeFat32() {

    char sdName[]   = "PRINTER3D";
    char fat32str[] = "FAT32   ";
    uint32_t nc;

    relSector = BU32;
    for (dataStart = 2 * BU32;; dataStart += BU32) {
      nc = (cardSizeBlocks - dataStart)/sectorsPerCluster;
      fatSize = (nc + 2 + 127)/128;
      uint32_t r = relSector + 9 + 2 * fatSize;
      if (dataStart >= r) {
        break;
      }
    }
    // error if too few clusters in FAT32 volume
    if (nc < 65525) {
      SERIAL_EM("Bad cluster count");
    }
    reservedSectors = dataStart - relSector - 2 * fatSize;
    fatStart = relSector + reservedSectors;
    partSize = nc * sectorsPerCluster + dataStart - relSector;
    // type depends on address of end sector
    // max CHS has lbn = 16450560 = 1024*255*63
    if ((relSector + partSize) <= 16450560) {
      // FAT32
      partType = 0X0B;
    } else {
      // FAT32 with INT 13
      partType = 0X0C;
    }
    writeMbr();
    clearCache(true);

    fat32_boot_t* pb = &cache.fbs32;
    pb->jump[0] = 0XEB;
    pb->jump[1] = 0X00;
    pb->jump[2] = 0X90;
    for (uint8_t i = 0; i < sizeof(pb->oemId); i++) {
      pb->oemId[i] = ' ';
    }
    pb->bytesPerSector = 512;
    pb->sectorsPerCluster = sectorsPerCluster;
    pb->reservedSectorCount = reservedSectors;
    pb->fatCount = 2;
    pb->mediaType = 0XF8;
    pb->sectorsPerTrack = sectorsPerTrack;
    pb->headCount = numberOfHeads;
    pb->hidddenSectors = relSector;
    pb->totalSectors32 = partSize;
    pb->sectorsPerFat32 = fatSize;
    pb->fat32RootCluster = 2;
    pb->fat32FSInfo = 1;
    pb->fat32BackBootBlock = 6;
    pb->driveNumber = 0X80;
    pb->bootSignature = EXTENDED_BOOT_SIG;
    pb->volumeSerialNumber = volSerialNumber();
    memcpy(pb->volumeLabel, sdName, sizeof(pb->volumeLabel));
    memcpy(pb->fileSystemType, fat32str, sizeof(pb->fileSystemType));
    // write partition boot sector and backup
    if (!writeCache(relSector)
        || !writeCache(relSector + 6)) {
      SERIAL_EM("FAT32 write PBS failed");
    }
    clearCache(true);
    // write extra boot area and backup
    if (!writeCache(relSector + 2)
        || !writeCache(relSector + 8)) {
      SERIAL_EM("FAT32 PBS ext failed");
    }
    fat32_fsinfo_t* pf = &cache.fsinfo;
    pf->leadSignature = FSINFO_LEAD_SIG;
    pf->structSignature = FSINFO_STRUCT_SIG;
    pf->freeCount = 0XFFFFFFFF;
    pf->nextFree = 0XFFFFFFFF;
    // write FSINFO sector and backup
    if (!writeCache(relSector + 1)
        || !writeCache(relSector + 7)) {
      SERIAL_EM("FAT32 FSINFO failed");
    }
    clearFatDir(fatStart, 2 * fatSize + sectorsPerCluster);
    clearCache(false);
    cache.fat32[0] = 0x0FFFFFF8;
    cache.fat32[1] = 0x0FFFFFFF;
    cache.fat32[2] = 0x0FFFFFFF;
    // write first block of FAT and backup for reserved clusters
    if (!writeCache(fatStart)
        || !writeCache(fatStart + fatSize)) {
      SERIAL_EM("FAT32 reserve failed");
    }
  }

#endif // ADVANCED_SD_COMMAND

#endif //SDSUPPORT

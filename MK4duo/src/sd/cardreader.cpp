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

#include "../../MK4duo.h"

#if HAS_SDSUPPORT

  CardReader card;

  #if ENABLED(ARDUINO_ARCH_SAM)
    #include <avr/dtostrf.h>
  #endif

  CardReader::CardReader() {
    #if ENABLED(SDCARD_SORT_ALPHA)
      sort_count = 0;
      #if ENABLED(SDSORT_GCODE)
        sort_alpha = true;
        sort_folders = FOLDER_SORTING;
        //sort_reverse = false;
      #endif
    #endif
    sdprinting = cardOK = saving = false;
    fileSize = 0;
    sdpos = 0;
    workDirDepth = 0;
    ZERO(workDirParents);

    autostart_stilltocheck = true; // the SD start is delayed, because otherwise the serial cannot answer fast enough to make contact with the host software.

    // power to SD reader
    #if PIN_EXISTS(SDPOWER)
      OUT_WRITE(SDPOWER_PIN, HIGH);
    #endif // SDPOWER_PIN

    next_autostart_ms = millis() + BOOTSCREEN_TIMEOUT;
  }

  /**
   * Dive into a folder and recurse depth-first to perform a pre-set operation lsAction:
   *   LS_Count       - Add +1 to nrFiles for every file within the parent
   *   LS_GetFilename - Get the filename of the file indexed by nrFile_index
   */

  uint16_t nrFile_index;

  void CardReader::lsDive(SdBaseFile parent, const char* const match/*=NULL*/) {
    dir_t* p    = NULL;
    uint8_t cnt = 0;

    // Read the next entry from a directory
    while ((p = parent.getLongFilename(p, fileName)) != NULL) {
      uint8_t pn0 = p->name[0];
      if (pn0 == DIR_NAME_FREE) break;

      // ignore hidden or deleted files:
      if (pn0 == DIR_NAME_DELETED || pn0 == '.') continue;
      if (fileName[0] == '.') continue;
      if (!DIR_IS_FILE_OR_SUBDIR(p) || (p->attributes & DIR_ATT_HIDDEN)) continue;

      filenameIsDir = DIR_IS_SUBDIR(p);

      if (!filenameIsDir && (p->name[8] != 'G' || p->name[9] == '~')) continue;
      switch (lsAction) {
        case LS_Count:
          nrFiles++;
          break;
        case LS_GetFilename:
          if (match != NULL) {
            if (strcasecmp(match, fileName) == 0) return;
          }
          else if (cnt == nrFile_index) return;
          cnt++;
          break;
      }

    } // while readDir
  }

  void CardReader::ls()  {
    root.openRoot(fat.vol());
    root.ls();
    workDir = root;
    curDir = &workDir;
  }

  void CardReader::mount() {
    cardOK = false;
    if (root.isOpen()) root.close();

    #if ENABLED(SDEXTRASLOW)
      #define SPI_SPEED SPI_QUARTER_SPEED
    #elif ENABLED(SDSLOW)
      #define SPI_SPEED SPI_HALF_SPEED
    #else
      #define SPI_SPEED SPI_FULL_SPEED
    #endif

    if(!fat.begin(SDSS, SPI_SPEED)
      #if ENABLED(LCD_SDSS) && (LCD_SDSS != SDSS)
        && !fat.begin(LCD_SDSS, SPI_SPEED)
      #endif
    ) {
      SERIAL_LM(ER, MSG_SD_INIT_FAIL);
    }
    else {
      cardOK = true;
      SERIAL_EM(MSG_SD_CARD_OK);
    }
    fat.chdir(true);
    root = *fat.vwd();
    setroot();
  }

  void CardReader::unmount() {
    cardOK = false;
    sdprinting = false;
  }

  void CardReader::startFileprint() {
    if (cardOK) {
      sdprinting = true;
      #if ENABLED(SDCARD_SORT_ALPHA)
        flush_presort();
      #endif
    }
  }

  void CardReader::openAndPrintFile(const char *name) {
    char cmd[4 + strlen(name) + 1]; // Room for "M23 ", filename, and null
    sprintf_P(cmd, PSTR("M23 %s"), name);
    for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
    commands.enqueue_and_echo_now(cmd);
    commands.enqueue_and_echo_P(PSTR("M24"));
  }

  void CardReader::stopSDPrint() {
    if (isFileOpen()) {

      if (sdprinting) {
        sdprinting = false;
        closeFile(true);
      }
      else
        closeFile(false);

      lcd_setstatus(MSG_PRINT_ABORTED, true);
    }
  }

  void CardReader::write_command(char* buf) {
    char* begin = buf;
    char* npos = 0;
    char* end = buf + strlen(buf) - 1;

    gcode_file.writeError = false;
    if ((npos = strchr(buf, 'N')) != NULL) {
      begin = strchr(npos, ' ') + 1;
      end = strchr(npos, '*') - 1;
    }
    end[1] = '\r';
    end[2] = '\n';
    end[3] = '\0';
    gcode_file.write(begin);
    if (gcode_file.writeError) {
      SERIAL_LM(ER, MSG_SD_ERR_WRITE_TO_FILE);
    }
  }

  #if HAS_EEPROM_SD

    bool CardReader::write_data(SdFile *currentfile, const uint8_t value) {
      currentfile->writeError = false;
      currentfile->write(value);
      if (currentfile->writeError) {
        SERIAL_LM(ER, MSG_SD_ERR_WRITE_TO_FILE);
        return false;
      }
      return true;
    }

    uint8_t CardReader::read_data(SdFile *currentfile) { return (char)currentfile->read(); }

  #endif

  bool CardReader::selectFile(const char* filename) {
    const char *oldP = filename;

    if (!cardOK) return false;

    curDir = &workDir; // Relative paths start in current directory

    if (gcode_file.open(curDir, filename, O_READ)) {
      if ((oldP = strrchr(filename, '/')) != NULL)
        oldP++;
      else
        oldP = filename;

      fileSize = gcode_file.fileSize();
      sdpos = 0;

      SERIAL_MT(MSG_SD_FILE_OPENED, oldP);
      SERIAL_EMV(MSG_SD_SIZE, fileSize);
      SERIAL_EM(MSG_SD_FILE_SELECTED);

      for (uint16_t c = 0; c < sizeof(fileName); c++)
        const_cast<char&>(fileName[c]) = '\0';
      strncpy(fileName, filename, strlen(filename));

      #if ENABLED(JSON_OUTPUT)
        parsejson(gcode_file);
      #endif

      return true;
    }
    else {
      SERIAL_EMT(MSG_SD_OPEN_FILE_FAIL, oldP);
      return false;
    }
  }

  void CardReader::printStatus() {
    if (isFileOpen() && sdprinting) {
      SERIAL_MV(MSG_SD_PRINTING_BYTE, sdpos);
      SERIAL_EMV(MSG_SD_SLASH, fileSize);
    }
    else
      SERIAL_EM(MSG_SD_NOT_PRINTING);
  }

  void CardReader::startWrite(char *filename, const bool silent/*=false*/) {
    if (!cardOK) return;

    if (!gcode_file.open(curDir, filename, O_CREAT | O_APPEND | O_WRITE | O_TRUNC)) {
      SERIAL_LMT(ER, MSG_SD_OPEN_FILE_FAIL, filename);
    }
    else {
      saving = true;
      if (!silent) {
        SERIAL_EMT(MSG_SD_WRITE_TO_FILE, filename);
        lcd_setstatus(filename);
      }
    }
  }

  void CardReader::deleteFile(char *filename) {
    if (!cardOK) return;
    sdprinting = false;
    gcode_file.close();
    if (fat.remove(filename)) {
      SERIAL_EMT(MSG_SD_FILE_DELETED, filename);
    }
    else {
      if (fat.rmdir(filename)) {
        SERIAL_EMT(MSG_SD_FILE_DELETED, filename);
        sdpos = 0;
        #if ENABLED(SDCARD_SORT_ALPHA)
          presort();
        #endif
      }
      else
        SERIAL_EM(MSG_SD_FILE_DELETION_ERR);
    }
  }

  void CardReader::finishWrite() {
    gcode_file.sync();
    gcode_file.close();
    saving = false;
    SERIAL_EM(MSG_SD_FILE_SAVED);
  }

  void CardReader::makeDirectory(char *filename) {
    if (!cardOK) return;
    sdprinting = false;
    gcode_file.close();
    if (fat.mkdir(filename)) {
      SERIAL_EM(MSG_SD_DIRECTORY_CREATED);
    }
    else {
      SERIAL_EM(MSG_SD_CREATION_FAILED);
    }
  }

  /**
   * Get the name of a file in the current directory by index
   */
  void CardReader::getfilename(uint16_t nr, const char* const match/*=NULL*/) {
    #if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_CACHE_NAMES)
      if (match != NULL) {
        while (nr < sort_count) {
          if (strcasecmp(match, sortshort[nr]) == 0) break;
          nr++;
        }
      }
      if (nr < sort_count) {
        strcpy(fileName, sortnames[nr]);
        filenameIsDir = TEST(isDir[nr>>3], nr & 0x07);
        return;
      }
    #endif // SDSORT_CACHE_NAMES
    curDir = &workDir;
    lsAction = LS_GetFilename;
    nrFile_index = nr;
    curDir->rewind();
    lsDive(*curDir, match);
  }

  uint16_t CardReader::getnrfilenames() {
    curDir = &workDir;
    lsAction = LS_Count;
    nrFiles = 0;
    curDir->rewind();
    lsDive(*curDir);
    return nrFiles;
  }

  void CardReader::chdir(const char* relpath) {
    SdBaseFile newfile;
    SdBaseFile* parent = &root;

    if (workDir.isOpen()) parent = &workDir;

    if (!newfile.open(parent, relpath, O_READ)) {
      SERIAL_EMT(MSG_SD_CANT_ENTER_SUBDIR, relpath);
    }
    else {
      if (workDirDepth < SD_MAX_FOLDER_DEPTH) {
        ++workDirDepth;
        for (int d = workDirDepth; d--;) workDirParents[d + 1] = workDirParents[d];
        workDirParents[0] = *parent;
      }
      workDir = newfile;
      #if ENABLED(SDCARD_SORT_ALPHA)
        presort();
      #endif
    }
  }

  void CardReader::closeFile(const bool store_position/*=false*/) {

    if (!store_position) {
      gcode_file.sync();
      gcode_file.close();
      saving = false;
      return;
    }

    SERIAL_EM("Save restart.gcode");

    SdFile restart_file;

    char  bufferFilerestart[100],
          buffer_G1[50],
          buffer_G92_Z[50],
          buffer_G92_E[50],
          buffer_SDpos[11];

    uint32_t saved_sdpos = 0;

    float saved_pos[XYZE] = { 0.0, 0.0, 0.0, 0.0 };

    uint8_t saved_active_extruder = 0;

    const char* restart_name_File = "restart.gcode";

    int16_t old_temp[HEATER_COUNT];
    LOOP_HEATER() {
      old_temp[h] = heaters[h].target_temperature;
      heaters[h].target_temperature = 0;
      heaters[h].soft_pwm = 0;
    }

    #if FAN_COUNT > 0
      uint16_t old_fan[FAN_COUNT];
      LOOP_FAN() {
        old_fan[f] = fans[f].Speed;
        fans[f].Speed = 0;
        fans[f].pwm_pos = 0;
      }
    #endif

    CRITICAL_SECTION_START

      // Saved position of SD file
      saved_sdpos  = sdpos + 1;
      saved_sdpos -= planner.command_in_planner_len();
      snprintf(buffer_SDpos, sizeof buffer_SDpos, "%lu", saved_sdpos);

      // Clear all movement in planned and abort printing
      stepper.kill_current_block();
      planner.abort();
      COPY_ARRAY(saved_pos, mechanics.current_position);
      saved_active_extruder = tools.active_extruder;

      commands.clear_queue(); // Empty command queue
      sdprinting = false;

    CRITICAL_SECTION_END

    mechanics.destination[Z_AXIS] = saved_pos[Z_AXIS] + 5;
    mechanics.destination[E_AXIS] = saved_pos[E_AXIS] - 1;
    mechanics.prepare_move_to_destination();
    stepper.finish_and_disable();

    #if 0
      SERIAL_EMV("SDPOS:", sdpos + 1);
      SERIAL_EMV("PLANNER LENGHT:", planner.command_in_planner_len());
      SERIAL_EMV("PLANNER BLOCKS:", planner.number_of_blocks());
      SERIAL_EMV("SAVED_SDPOS:", saved_sdpos);
      SERIAL_EMT("FILENAME:", fileName);
    #endif

    strcpy(bufferFilerestart, "M32 S");
    strcat(bufferFilerestart, buffer_SDpos);
    strcat(bufferFilerestart, " ");
    strcat(bufferFilerestart, fileName);

    strcpy(buffer_G1, "G1 X");
    dtostrf(saved_pos[X_AXIS], 1, 3, &buffer_G1[strlen(buffer_G1)]);
    strcat(buffer_G1, " Y");
    dtostrf(saved_pos[Y_AXIS], 1, 3, &buffer_G1[strlen(buffer_G1)]);
    strcat(buffer_G1, " Z");
    dtostrf(saved_pos[Z_AXIS], 1, 3, &buffer_G1[strlen(buffer_G1)]);
    strcat(buffer_G1, " F3600\n");

    #if MECH(DELTA)
      strcpy(buffer_G92_Z, "; Nothing for delta\n\n");
    #else
      strcpy(buffer_G92_Z, "G92 Z");
      dtostrf(saved_pos[Z_AXIS] + 5 + MIN_Z_HEIGHT_FOR_HOMING, 1, 3, &buffer_G92_Z[strlen(buffer_G92_Z)]);
      strcat(buffer_G92_Z, "\n\n");
    #endif

    strcpy(buffer_G92_E, "G92 E");
    dtostrf(saved_pos[E_AXIS], 1, 3, &buffer_G92_E[strlen(buffer_G92_E)]);
    strcat(buffer_G92_E, "\n");

    gcode_file.sync();
    gcode_file.close();
    saving = false;

    if (!restart_file.exists(restart_name_File)) {
      restart_file.createContiguous(&workDir, restart_name_File, 1);
      restart_file.close();
    }

    restart_file.open(&workDir, restart_name_File, O_WRITE);
    restart_file.truncate(0);

    #if MECH(DELTA)
      restart_file.write("G28\n");
    #else
      restart_file.write("G28 X Y\n");
    #endif

    #if HAS_LEVELING
      if (bedlevel.leveling_active) restart_file.write("M420 S1\n");
    #endif

    restart_file.write(buffer_G92_Z);

    #if HAS_TEMP_BED
      if (old_temp[BED_INDEX] > 0) {
        char Bedtemp[15];
        sprintf(Bedtemp, "M190 S%i\n", (int)old_temp[BED_INDEX]);
        restart_file.write(Bedtemp);
      }
    #endif

    char CurrHotend[10];
    sprintf(CurrHotend, "T%i\n", saved_active_extruder);
    restart_file.write(CurrHotend);

    for (uint8_t h = 0; h < HOTENDS; h++) {
      if (old_temp[h] > 0) {
        char Hotendtemp[15];
        sprintf(Hotendtemp, "M109 T%i S%i\n", (int)h, (int)old_temp[h]);
        restart_file.write(Hotendtemp);
      }
    }

    restart_file.write("G92 E0\nG1 E10 F300\nG92 E0\n");

    restart_file.write(buffer_G1);

    #if FAN_COUNT > 0
      LOOP_FAN() {
        if (old_fan[f] > 0) {
          char fanSp[20];
          sprintf(fanSp, "M106 S%i P%i\n", (int)old_fan[f], (int)f);
          restart_file.write(fanSp);
        }
      }
    #endif

    restart_file.write(buffer_G92_E);
    restart_file.write("\n");
    restart_file.write(bufferFilerestart);
    restart_file.write("\n");

    restart_file.sync();
    restart_file.close();
    saving = false;
    sdprinting = false;

  }

  void CardReader::checkautostart(bool force) {
    if (!force && (!autostart_stilltocheck || next_autostart_ms >= millis()))
      return;

    autostart_stilltocheck = false;

    if (!cardOK) {
      mount();
      if (!cardOK) return; // fail
    }

    fat.chdir(true);
    if (selectFile("init.g")) startFileprint();
  }

  void CardReader::printingHasFinished() {
    stepper.synchronize();
    gcode_file.close();
    sdprinting = false;

    #if SD_FINISHED_STEPPERRELEASE && ENABLED(SD_FINISHED_RELEASECOMMAND)
      stepper.cleaning_buffer_counter = 1; // The command will fire from the Stepper ISR
    #endif

    print_job_counter.stop();

    if (print_job_counter.duration() > 60)
      commands.enqueue_and_echo_P(PSTR("M31"));

    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }

  void CardReader::setroot() {
    lastDir = workDir;
    workDir = root;
    curDir = &workDir;
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }

  void CardReader::setlast() {
    workDir = lastDir;
    curDir = &workDir;
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }

  #if ENABLED(SDCARD_SORT_ALPHA)

    /**
     * Get the name of a file in the current directory by sort-index
     */
    void CardReader::getfilename_sorted(const uint16_t nr) {
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
    void CardReader::presort() {

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
        if (fileCnt > SDSORT_LIMIT) fileCnt = SDSORT_LIMIT;

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
                // Use dynamic method to copy long filename
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
              // sprintf_P(out, PSTR("---- %i %s %s"), i, filenameIsDir ? "D" : " ", sortnames[i]);
              // SERIAL_ECHOLN(out);
              #if HAS_FOLDER_SORTING
                const uint16_t bit = i & 0x07, ind = i >> 3;
                if (bit == 0) isDir[ind] = 0x00;
                if (filenameIsDir) isDir[ind] |= _BV(bit);
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
                  #define _SORT_CMP_DIR(fs) ((dir1 == filenameIsDir) ? _SORT_CMP_NODIR() : (fs > 0 ? dir1 : !dir1))
                #endif
              #endif

              // The most economical method reads names as-needed
              // throughout the loop. Slow if there are many.
              #if DISABLED(SDSORT_USES_RAM)
                getfilename(o1);
                strcpy(name1, fileName); // save (or getfilename below will trounce it)
                #if HAS_FOLDER_SORTING
                  bool dir1 = filenameIsDir;
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
            isDir[0] = filenameIsDir ? 0x01 : 0x00;
          #endif
        }

        sort_count = fileCnt;
      }
    }

    void CardReader::flush_presort() {
      if (sort_count > 0) {
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          delete sort_order;
        #endif
        sort_count = 0;
      }
    }

  #endif // SDCARD_SORT_ALPHA

  int8_t CardReader::updir() {
    if (workDirDepth > 0) {                                               // At least 1 dir has been saved
      workDir = --workDirDepth ? workDirParents[workDirDepth - 1] : root; // Use parent, or root if none
      #if ENABLED(SDCARD_SORT_ALPHA)
        presort();
      #endif
    }
    return workDirDepth;
  }

  uint16_t CardReader::get_num_Files() {
    return
      #if ENABLED(SDCARD_SORT_ALPHA) && SDSORT_USES_RAM && SDSORT_CACHE_NAMES
        nrFiles // no need to access the SD card for filenames
      #else
        getnrfilenames()
      #endif
    ;
  }

  // --------------------------------------------------------------- //
  // Code that gets gcode information is adapted from RepRapFirmware //
  // Originally licenced under GPL                                   //
  // Authors: reprappro, dc42, dcnewman, others                      //
  // Source: https://github.com/dcnewman/RepRapFirmware              //
  // Copy date: 27 FEB 2016                                          //
  // --------------------------------------------------------------- //

  void CardReader::parsejson(SdBaseFile &parser_file) {
    fileSize = parser_file.fileSize();
    filamentNeeded    = 0.0;
    objectHeight      = 0.0;
    firstlayerHeight  = 0.0;
    layerHeight       = 0.0;

    if (!parser_file.isOpen()) return;

    bool genByFound = false, firstlayerHeightFound = false, layerHeightFound = false, filamentNeedFound = false;

    #if CPU_ARCH==ARCH_AVR
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
    parser_file.seekSet(0);
  }

  void CardReader::printEscapeChars(const char* s) {
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

  bool CardReader::findGeneratedBy(char* buf, char* genBy) {
    // Slic3r & S3D
    const char* generatedByString = PSTR("generated by ");
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
    const char* slicedAtString = PSTR(";Sliced at: ");
    pos = strstr_P(buf, slicedAtString);
    if (pos) {
      strcpy_P(genBy, PSTR("Cura"));
      return true;
    }

    // UNKNOWN
    strcpy_P(genBy, PSTR("Unknown"));
    return false;
  }

  bool CardReader::findFirstLayerHeight(char* buf, float &firstlayerHeight) {
    // SLIC3R
    firstlayerHeight = 0;
    const char* layerHeightSlic3r = PSTR("; first_layer_height ");
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
    const char* layerHeightCura = PSTR("First layer height: ");
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

  bool CardReader::findLayerHeight(char* buf, float &layerHeight) {
    // SLIC3R
    layerHeight = 0;
    const char* layerHeightSlic3r = PSTR("; layer_height ");
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
    const char* layerHeightCura = PSTR("Layer height: ");
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

  bool CardReader::findFilamentNeed(char* buf, float &filament) {
    const char* filamentUsedStr = PSTR("filament used");
    const char* pos = strstr_P(buf, filamentUsedStr);
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

  bool CardReader::findTotalHeight(char* buf, float &height) {
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

  void CardReader::PrintSettings() {
    // Always have this function, even with SD_SETTINGS disabled, the current values will be shown

    #if HAS_POWER_CONSUMPTION_SENSOR
      SERIAL_LM(CFG, "Watt/h consumed:");
      SERIAL_SV(CFG, powerManager.consumption_hour);
      SERIAL_EM(" Wh");
    #endif

    print_job_counter.showStats();
  }

  void CardReader::ResetDefault() {
    #if HAS_POWER_CONSUMPTION_SENSOR
      powerManager.consumption_hour = 0;
    #endif
    print_job_counter.initStats();
    SERIAL_LM(OK, "Hardcoded SD Default Settings Loaded");
  }

  #if ENABLED(SD_SETTINGS)

    /**
     * File parser for KEY->VALUE format from files
     *
     * Author: Simone Primarosa
     *
     */
    void CardReader::parseKeyLine(char* key, char* value, int &len_k, int &len_v) {
      if (!cardOK || !settings_file.isOpen()) {
        key[0] = value[0] = '\0';
        len_k = len_v = 0;
        return;
      }

      int ln_buf = 0;
      char ln_char;
      bool ln_space = false, ln_ignore = false, key_found = false;

      while (!(settings_file.curPosition() >= settings_file.fileSize())) {  // READ KEY
        ln_char = (char)settings_file.read();
        if (ln_char == '\n') {
          ln_buf = 0;
          ln_ignore = false;  // We've reached a new line try to find a key again
          continue;
        }
        if (ln_ignore) continue;
        if (ln_char == ' ') {
          ln_space = true;
          continue;
        }
        if (ln_char == '=') {
          key[ln_buf] = '\0';
          len_k = ln_buf;
          key_found = true;
          break; //key finded and buffered
        }
        if (ln_char == ';' || (ln_buf+1 >= len_k) || (ln_space && ln_buf > 0)) { //comments on key is not allowd. Also key len can't be longer than len_k or contain spaces. Stop buffering and try the next line
          ln_ignore = true;
          continue;
        }
        ln_space = false;
        key[ln_buf] = ln_char;
        ln_buf++;
      }
      if (!key_found) { // definitly there isn't no more key that can be readed in the file
        key[0] = value[0] = '\0';
        len_k = len_v = 0;
        return;
      }
      ln_buf = 0;
      ln_ignore = false;
      while (!(settings_file.curPosition() >= settings_file.fileSize())) {   // READ VALUE
        ln_char = (char)settings_file.read();
        if (ln_char == '\n') {
          value[ln_buf] = '\0';
          len_v = ln_buf;
          break;  // new line reached, we can stop
        }
        if (ln_ignore || (ln_char == ' ' && ln_buf == 0)) continue;  // ignore also initial spaces of the value
        if (ln_char == ';' || ln_buf+1 >= len_v) {  // comments reached or value len longer than len_v. Stop buffering and go to the next line.
          ln_ignore = true;
          continue;
        }
        value[ln_buf] = ln_char;
        ln_buf++;
      }
    }

    void CardReader::unparseKeyLine(const char* key, char* value) {
      if (!cardOK || !settings_file.isOpen()) return;
      settings_file.writeError = false;
      settings_file.write(key);
      if (settings_file.writeError) {
        SERIAL_LM(ER, MSG_SD_ERR_WRITE_TO_FILE);
        return;
      }

      settings_file.writeError = false;
      settings_file.write("=");
      if (settings_file.writeError) {
        SERIAL_LM(ER, MSG_SD_ERR_WRITE_TO_FILE);
        return;
      }

      settings_file.writeError = false;
      settings_file.write(value);
      if (settings_file.writeError) {
        SERIAL_LM(ER, MSG_SD_ERR_WRITE_TO_FILE);
        return;
      }

      settings_file.writeError = false;
      settings_file.write("\n");
      if (settings_file.writeError) {
        SERIAL_LM(ER, MSG_SD_ERR_WRITE_TO_FILE);
        return;
      }
    }

    static const char *cfgSD_KEY[] = { // Keep this in lexicographical order for better search performance(O(Nlog2(N)) insted of O(N*N)) (if you don't keep this sorted, the algorithm for find the key index won't work, keep attention.)
      "CPR",  // Number of complete prints
      "FIL",  // Filament Usage
      "NPR",  // Number of prints
    #if HAS_POWER_CONSUMPTION_SENSOR
      "PWR",  // Power Consumption
    #endif
      "TME",  // Longest print job
      "TPR"   // Total printing time
    };

    void CardReader::StoreSettings() {
      if (!IS_SD_INSERTED || sdprinting || print_job_counter.isRunning()) return;

      setroot();

      if (settings_file.open(curDir, "INFO.cfg", O_CREAT | O_APPEND | O_WRITE | O_TRUNC)) {
        char buff[CFG_SD_MAX_VALUE_LEN];
        ltoa(print_job_counter.data.finishedPrints, buff, 10);
        unparseKeyLine(cfgSD_KEY[SD_CFG_CPR], buff);
        ltoa(print_job_counter.data.filamentUsed, buff, 10);
        unparseKeyLine(cfgSD_KEY[SD_CFG_FIL], buff);
        ltoa(print_job_counter.data.totalPrints, buff, 10);
        unparseKeyLine(cfgSD_KEY[SD_CFG_NPR], buff);
        #if HAS_POWER_CONSUMPTION_SENSOR
          ltoa(powerManager.consumption_hour, buff, 10);
          unparseKeyLine(cfgSD_KEY[SD_CFG_PWR], buff);
        #endif
        ltoa(print_job_counter.data.printer_usage, buff, 10);
        unparseKeyLine(cfgSD_KEY[SD_CFG_TME], buff);
        ltoa(print_job_counter.data.printTime, buff, 10);
        unparseKeyLine(cfgSD_KEY[SD_CFG_TPR], buff);

        settings_file.sync();
        settings_file.close();
        SERIAL_LM(ECHO, " Statistics stored");
      }

      setlast();
    }

    void CardReader::RetrieveSettings(bool addValue) {
      if (!IS_SD_INSERTED || sdprinting || !cardOK) return;

      char key[CFG_SD_MAX_KEY_LEN], value[CFG_SD_MAX_VALUE_LEN];
      int k_idx;
      int k_len, v_len;

      setroot();

      if (settings_file.open(curDir, "INFO.cfg", O_READ)) {

        while (true) {
          k_len = CFG_SD_MAX_KEY_LEN;
          v_len = CFG_SD_MAX_VALUE_LEN;
          parseKeyLine(key, value, k_len, v_len);

          if (k_len == 0 || v_len == 0) break; // no valid key or value founded

          k_idx = KeyIndex(key);
          if (k_idx == -1) continue; // unknow key ignore it

          switch (k_idx) {
            case SD_CFG_CPR: {
              if (addValue) print_job_counter.data.finishedPrints += (unsigned long)atol(value);
              else print_job_counter.data.finishedPrints = (unsigned long)atol(value);
            }
            break;
            case SD_CFG_FIL: {
              if (addValue) print_job_counter.data.filamentUsed += (unsigned long)atol(value);
              else print_job_counter.data.filamentUsed = (unsigned long)atol(value);
            }
            break;
            case SD_CFG_NPR: {
              if (addValue) print_job_counter.data.totalPrints += (unsigned long)atol(value);
              else print_job_counter.data.totalPrints = (unsigned long)atol(value);
            }
            break;
          #if HAS_POWER_CONSUMPTION_SENSOR
            case SD_CFG_PWR: {
              if (addValue) powerManager.consumption_hour += (unsigned long)atol(value);
              else powerManager.consumption_hour = (unsigned long)atol(value);
            }
            break;
          #endif
            case SD_CFG_TME: {
              if (addValue) print_job_counter.data.printer_usage += (unsigned long)atol(value);
              else print_job_counter.data.printer_usage = (unsigned long)atol(value);
            }
            break;
            case SD_CFG_TPR: {
              if (addValue) print_job_counter.data.printTime += (unsigned long)atol(value);
              else print_job_counter.data.printTime = (unsigned long)atol(value);
            }
            break;
          }
        }
        settings_file.sync();
        settings_file.close();
      }

      print_job_counter.loaded = true;
      SERIAL_LM(ECHO, " Statistics retrived");

      setlast();
    }

    int CardReader::KeyIndex(char *key) {  // At the moment a binary search algorithm is used for simplicity, if it will be necessary (Eg. tons of key), an hash search algorithm will be implemented.
      int begin = 0, end = SD_CFG_END - 1, middle, cond;

      while (begin <= end) {
        middle = (begin + end) / 2;
        cond = strcmp(cfgSD_KEY[middle], key);
        if (!cond) return middle;
        else if (cond < 0) begin = middle + 1;
        else end = middle - 1;
      }

      return -1;
    }

  #endif

#endif //SDSUPPORT

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

/**
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if HAS_SDSUPPORT

  #define CODE_M20
  #define CODE_M21
  #define CODE_M22
  #define CODE_M23
  #define CODE_M24
  #define CODE_M25
  #define CODE_M26
  #define CODE_M27
  #define CODE_M28
  #define CODE_M29
  #define CODE_M30
  #define CODE_M32
  #define CODE_M33

  /**
   * M20: List SD card to serial output
   */
  inline void gcode_M20(void) {
    SERIAL_EM(MSG_BEGIN_FILE_LIST);
    card.ls();
    SERIAL_EM(MSG_END_FILE_LIST);
  }

  /**
   * M21: Init SD Card
   */
  inline void gcode_M21(void) {
    card.mount();
  }

  /**
   * M22: Release SD Card
   */
  inline void gcode_M22(void) {
    card.unmount();
  }

  /**
   * M23: Select a file
   */
  inline void gcode_M23(void) {
    // Simplify3D includes the size, so zero out all spaces (#7227)
    // Questa funzione blocca il nome al primo spazio quindi file con spazio nei nomi non funziona da rivedere
    //for (char *fn = parser.string_arg; *fn; ++fn) if (*fn == ' ') *fn = '\0';
    card.selectFile(parser.string_arg);
    lcd_setstatus(card.fileName);
  }

  /**
   * M24: Start or Resume SD Print
   */
  inline void gcode_M24(void) {
    #if HAS_SD_RESTART
      card.delete_restart_file();
    #endif

    #if ENABLED(PARK_HEAD_ON_PAUSE)
      resume_print();
    #endif

    card.startFileprint();
    print_job_counter.start();
    #if HAS_POWER_CONSUMPTION_SENSOR
      powerManager.startpower = powerManager.consumption_hour;
    #endif
  }

  /**
   * M25: Pause SD Print
   */
  void gcode_M25(void) {
    card.pauseSDPrint();
    print_job_counter.pause();
    SERIAL_STR(PAUSE);
    SERIAL_EOL();

    #if ENABLED(PARK_HEAD_ON_PAUSE)
      commands.enqueue_and_echo_P(PSTR("M125")); // Must be enqueued with pauseSDPrint set to be last in the buffer
    #endif
  }

  /**
   * M26: Set SD Card file index
   */
  inline void gcode_M26(void) {
    if (card.isOK() && parser.seen('S'))
      card.setIndex(parser.value_long());
  }

  /**
   * M27: Get SD Card status or set the SD status auto-report interval.
   */
  inline void gcode_M27(void) {
    bool to_enable = false;
    if (parser.seenval('S')) {
      to_enable = parser.value_bool();
      printer.setAutoreportSD(to_enable);
    }
    else
      card.printStatus();
  }

  /**
   * M28: Start SD Write
   */
  inline void gcode_M28(void) { card.startWrite(parser.string_arg, false); }

  /**
   * M29: Stop SD Write
   * Processed in write to file routine above
   */
  inline void gcode_M29(void) { card.setSaving(false); }

  /**
   * M30 <filename>: Delete SD Card file
   */
  inline void gcode_M30(void) {
    if (card.isOK()) {
      card.closeFile();
      card.deleteFile(parser.string_arg);
    }
  }

  /**
   * M32: Select file and start SD print
   */
  inline void gcode_M32(void) {
    if (IS_SD_PRINTING) planner.synchronize();

    if (card.isOK()) {
      card.closeFile();

      char* namestartpos = parser.string_arg ; // default name position

      SERIAL_MV("Open file: ", namestartpos);
      SERIAL_EM(" and start print.");
      card.selectFile(namestartpos);
      if (parser.seenval('S')) card.setIndex(parser.value_long());

      mechanics.feedrate_mm_s       = 20.0; // 20 units/sec
      mechanics.feedrate_percentage = 100;  // 100% mechanics.feedrate_mm_s
      card.startFileprint();
      print_job_counter.start();
      #if HAS_POWER_CONSUMPTION_SENSOR
        powerManager.startpower = powerManager.consumption_hour;
      #endif
    }
  }

  /**
   * M33: Stop printing, close file and save restart.gcode
   */
  inline void gcode_M33(void) {
    if (card.isOK() && IS_SD_PRINTING)
      printer.setAbortSDprinting(true);
  }

  #if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)

    #define CODE_M34

    /**
     * M34: Set SD Card Sorting Options
     */
    inline void gcode_M34(void) {
      if (parser.seen('S')) card.setSortOn(parser.value_bool());
      if (parser.seenval('F')) {
        const int v = parser.value_long();
        card.setSortFolders(v < 0 ? -1 : v > 0 ? 1 : 0);
      }
      //if (parser.seen('R')) card.setSortReverse(parser.value_bool());
    }

  #endif // SDCARD_SORT_ALPHA && SDSORT_GCODE

#endif // HAS_SDSUPPORT

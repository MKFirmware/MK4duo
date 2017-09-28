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
  #define CODE_M34

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
    for (char *fn = parser.string_arg; *fn; ++fn) if (*fn == ' ') *fn = '\0';
    card.selectFile(parser.string_arg);
  }

  /**
   * M24: Start or Resume SD Print
   */
  inline void gcode_M24(void) {
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
    SERIAL_LM(REQUEST_PAUSE, "SD pause");

    #if ENABLED(PARK_HEAD_ON_PAUSE)
      commands.enqueue_and_echo_commands_P(PSTR("M125")); // Must be enqueued with pauseSDPrint set to be last in the buffer
    #endif
  }

  /**
   * M26: Set SD Card file index
   */
  inline void gcode_M26(void) {
    if (card.cardOK && parser.seen('S'))
      card.setIndex(parser.value_long());
  }

  /**
   * M27: Get SD Card status
   */
  inline void gcode_M27(void) { card.printStatus(); }

  /**
   * M28: Start SD Write
   */
  inline void gcode_M28(void) { card.startWrite(parser.string_arg, false); }

  /**
   * M29: Stop SD Write
   * Processed in write to file routine above
   */
  inline void gcode_M29(void) { card.saving = false; }

  /**
   * M30 <filename>: Delete SD Card file
   */
  inline void gcode_M30(void) {
    if (card.cardOK) {
      card.closeFile();
      card.deleteFile(parser.string_arg);
    }
  }

  /**
   * M32: Make Directory
   */
  inline void gcode_M32(void) {
    if (card.cardOK) {
      card.makeDirectory(parser.string_arg);
      card.mount();
    }
  }

  /**
   * M33: Close File and store location in restart.gcode
   */
  inline void gcode_M33(void) {
    printer.stopSDPrint(true);
  }

  /**
   * M34: Select file and start SD print
   */
  inline void gcode_M34(void) {
    if (card.sdprinting)
      stepper.synchronize();

    if (card.cardOK) {
      char* namestartpos = (strchr(parser.string_arg, '@'));
      if (namestartpos == NULL) {
        namestartpos = parser.string_arg ; // default name position
      }
      else
        namestartpos++; // to skip the '@'

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

#endif // HAS_SDSUPPORT

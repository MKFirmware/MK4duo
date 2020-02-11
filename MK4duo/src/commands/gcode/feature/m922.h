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

/**
 * mcode
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if HAS_TRINAMIC

#define CODE_M922

/**
 * M922: Debug TMC drivers
 */
inline void gcode_M922() {

  bool print_axis[XYZE] = { false, false, false, false },
       print_all = true;

  LOOP_XYZE(axis) if (parser.seen(axis_codes[axis])) { print_axis[axis] = true; print_all = false; }

  if (print_all) LOOP_XYZE(axis) print_axis[axis] = true;

  #if ENABLED(TMC_DEBUG)
    #if ENABLED(MONITOR_DRIVER_STATUS)
      const bool sflag = parser.seen('S'), s0 = sflag && !parser.value_bool();
      if (sflag) tmcManager.set_report_interval(s0 ? 0 : MONITOR_DRIVER_STATUS_INTERVAL_MS);
      if (!s0 && parser.seenval('P')) tmcManager.set_report_interval(MIN(parser.value_ushort(), MONITOR_DRIVER_STATUS_INTERVAL_MS));
    #endif

    if (parser.seen('V'))
      tmcManager.get_registers(print_axis[X_AXIS], print_axis[Y_AXIS], print_axis[Z_AXIS], print_axis[E_AXIS]);
    else
      tmcManager.report_all(print_axis[X_AXIS], print_axis[Y_AXIS], print_axis[Z_AXIS], print_axis[E_AXIS]);
  #endif

  tmcManager.test_connection(print_axis[X_AXIS], print_axis[Y_AXIS], print_axis[Z_AXIS], print_axis[E_AXIS]);

}

#elif HAS_L64XX

inline void L64XX_say_status(Driver* drv) {

  if (l64xxManager.flag.spi_abort) return;

  const L64XX_Manager::L64XX_shadow_t &sh = l64xxManager.shadow;

  l64xxManager.get_stepper_status(drv);
  l64xxManager.say_axis(drv);

  #if ENABLED(L6470_CHITCHAT)
    char temp_buf[20];
    sprintf_P(temp_buf, PSTR("   status: %4x   "), sh.STATUS_AXIS_RAW);
    SERIAL_TXT(temp_buf);
    //print_bin(sh.STATUS_AXIS_RAW);
    switch (sh.STATUS_AXIS_LAYOUT) {
      case L6470_STATUS_LAYOUT: SERIAL_STR(PSTR("   L6470")); break;
      case L6474_STATUS_LAYOUT: SERIAL_STR(PSTR("   L6474")); break;
      case L6480_STATUS_LAYOUT: SERIAL_STR(PSTR("   L6480")); break;
    }
  #endif

  SERIAL_ONOFF("\n...OUTPUT", sh.STATUS_AXIS & STATUS_HIZ);
  SERIAL_YESNO("   BUSY", (sh.STATUS_AXIS & STATUS_BUSY) == 0);
  SERIAL_MSG("   DIR: ");
  SERIAL_STR((((sh.STATUS_AXIS & STATUS_DIR) >> 4) ^ drv->isDir()) ? PSTR("FORWARD") : PSTR("REVERSE"));
  if (sh.STATUS_AXIS_LAYOUT == L6480_STATUS_LAYOUT) {
    SERIAL_MSG("   Last Command: ");
    if (sh.STATUS_AXIS & sh.STATUS_AXIS_WRONG_CMD) SERIAL_MSG("VALID");
    else                                           SERIAL_MSG("ERROR");
    SERIAL_MSG("\n...THERMAL: ");
    switch ((sh.STATUS_AXIS & (sh.STATUS_AXIS_TH_SD | sh.STATUS_AXIS_TH_WRN)) >> 11) {
      case 0: SERIAL_MSG("DEVICE SHUTDOWN"); break;
      case 1: SERIAL_MSG("BRIDGE SHUTDOWN"); break;
      case 2: SERIAL_MSG("WARNING        "); break;
      case 3: SERIAL_MSG("OK             "); break;
    }
  }
  else {
    SERIAL_MSG("   Last Command: ");
    if (!(sh.STATUS_AXIS & sh.STATUS_AXIS_WRONG_CMD)) SERIAL_MSG("IN");
    SERIAL_MSG("VALID    ");
    SERIAL_STR(sh.STATUS_AXIS & sh.STATUS_AXIS_NOTPERF_CMD ?  PSTR("COMPLETED    ") : PSTR("Not PERFORMED"));
    SERIAL_MSG("\n...THERMAL: ");
    SERIAL_MSG(!(sh.STATUS_AXIS & sh.STATUS_AXIS_TH_SD) ? "SHUTDOWN       " : !(sh.STATUS_AXIS & sh.STATUS_AXIS_TH_WRN) ? "WARNING        " : "OK             ");
  }

  SERIAL_YESNO("   OVERCURRENT", (sh.STATUS_AXIS & sh.STATUS_AXIS_OCD) == 0);

  if (sh.STATUS_AXIS_LAYOUT != L6474_STATUS_LAYOUT) {
    SERIAL_YESNO("   STALL", (sh.STATUS_AXIS & sh.STATUS_AXIS_STEP_LOSS_A) == 0 || (sh.STATUS_AXIS & sh.STATUS_AXIS_STEP_LOSS_B) == 0);
    SERIAL_YESNO("   STEP-CLOCK MODE", (sh.STATUS_AXIS & sh.STATUS_AXIS_SCK_MOD) != 0);
  }
  else {
    SERIAL_YESNO("   STALL: NA "
                 "   STEP-CLOCK MODE: NA"
                 "   UNDER VOLTAGE LOCKOUT", (sh.STATUS_AXIS & sh.STATUS_AXIS_UVLO) == 0);
  }

  SERIAL_EOL();

}

/**
 * M922: Debug L64XX drivers
 */
inline void gcode_M922() {

  l64xxManager.flag.monitor_paused = true;
  l64xxManager.flag.spi_active = true;

  LOOP_DRV_XYZ() {
    if (driver[d] && driver[d]->l64) L64XX_say_status(driver[d]);
  }
  LOOP_DRV_EXT() {
    if (driver.e[d] && driver.e[d]->l64) L64XX_say_status(driver.e[d]);
  }

  l64xxManager.flag.spi_active = false;
  l64xxManager.flag.spi_abort = false;
  l64xxManager.flag.monitor_paused = false;

}

#endif

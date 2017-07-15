/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(DELTA_AUTO_CALIBRATION_1) || ENABLED(DELTA_AUTO_CALIBRATION_2)

  #define CODE_G33

  inline void gcode_G33(void) { mechanics.auto_calibration(); }

#elif ENABLED(DELTA_AUTO_CALIBRATION_3)

  #define CODE_G33

  /**
   * G33: Delta AutoCalibration Algorithm based on Rich Cattell Marlin
   * Usage:
   *    G33 <An> <En> <Rn> <In> <Dn> <Tn>
   *      A = Autocalibration +- precision
   *      E = Adjust Endstop +- precision
   *      R = Adjust Endstop & Delta Radius +- precision
   *      I = Adjust Tower
   *      D = Adjust Diagonal Rod
   *      T = Adjust Tower Radius
   */
  inline void gcode_G33(void) {

    // Homing
    mechanics.Home(true);

    mechanics.do_blocking_move_to_z(_Z_PROBE_DEPLOY_HEIGHT, mechanics.homing_feedrate_mm_s[Z_AXIS]);

    stepper.synchronize();  // wait until the machine is idle

    SERIAL_EM("Starting Auto Calibration...");
    LCD_MESSAGEPGM("Auto Calibration...");

    ac_prec = ((parser.seen('A') || parser.seen('E') || parser.seen('R')) && parser.has_value()) ? constrain(parser.value_float(), 0.01, 1) : AUTOCALIBRATION_PRECISION;

    SERIAL_MV("Calibration precision: +/-", ac_prec, 2);
    SERIAL_EM(" mm");

    // Probe all points
    bed_probe_all();

    // Show calibration report      
    calibration_report();

    if (parser.seen('E')) {
      SERIAL_EM("Calibration Endstop.");
      int iteration = 0;
      do {
        iteration ++;
        SERIAL_EMV("Iteration: ", iteration);

        SERIAL_EM("Checking/Adjusting Endstop offsets");
        adj_endstops();

        bed_probe_all();
        calibration_report();
      } while (FABS(bed_level_x) > ac_prec
            or FABS(bed_level_y) > ac_prec
            or FABS(bed_level_z) > ac_prec);

      SERIAL_EM("Endstop adjustment complete");
    }

    if (parser.seen('R')) {
      SERIAL_EM("Calibration Endstop & Delta Radius.");
      int iteration = 0;
      do {
        iteration ++;
        SERIAL_EMV("Iteration: ", iteration);

        SERIAL_EM("Checking/Adjusting Endstop offsets");
        adj_endstops();

        bed_probe_all();
        calibration_report();

        SERIAL_EM("Checking delta radius");
        adj_deltaradius();

      } while (FABS(bed_level_c) > ac_prec
            or FABS(bed_level_x) > ac_prec
            or FABS(bed_level_y) > ac_prec
            or FABS(bed_level_z) > ac_prec);

      SERIAL_EM("Endstop & Delta Radius adjustment complete");
    }

    if (parser.seen('I')) {
      SERIAL_EMV("Adjusting Tower Delta for tower", parser.value_byte());
      adj_tower_delta(parser.value_byte());
      SERIAL_EM("Tower Delta adjustment complete");
    }

    if (parser.seen('D')) {
      SERIAL_EM("Adjusting Diagonal Rod Length");
      adj_diagrod_length();
      SERIAL_EM("Diagonal Rod Length adjustment complete");
    }

    if (parser.seen('T')) {
      SERIAL_EMV("Adjusting Tower Radius for tower", parser.value_byte());
      adj_tower_radius(parser.value_byte());
      SERIAL_EM("Tower Radius adjustment complete");
    }

    if (parser.seen('A')) {
      SERIAL_EM("Calibration All.");
      int iteration = 0;
      bool dr_adjusted;

      do {
        do {
          iteration ++;
          SERIAL_EMV("Iteration: ", iteration);

          SERIAL_EM("Checking/Adjusting endstop offsets");
          adj_endstops();

          bed_probe_all();
          calibration_report();

          if (FABS(bed_level_c) > ac_prec) {
            SERIAL_EM("Checking delta radius");
            dr_adjusted = adj_deltaradius();
          }
          else
            dr_adjusted = false;

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_EMV("bed_level_c = ", bed_level_c, 4);
              SERIAL_EMV("bed_level_x = ", bed_level_x, 4);
              SERIAL_EMV("bed_level_y = ", bed_level_y, 4);
              SERIAL_EMV("bed_level_z = ", bed_level_z, 4);
            }
          #endif

        } while (FABS(bed_level_c) > ac_prec
              or FABS(bed_level_x) > ac_prec
              or FABS(bed_level_y) > ac_prec
              or FABS(bed_level_z) > ac_prec
              or dr_adjusted);

        if  (FABS(bed_level_ox) > ac_prec
          or FABS(bed_level_oy) > ac_prec
          or FABS(bed_level_oz) > ac_prec) {
          SERIAL_EM("Checking for tower geometry errors..");
          if (fix_tower_errors() != 0 ) {
            // Tower positions have been changed .. home to endstops
            SERIAL_EM("Tower Positions changed .. Homing");
            mechanics.Home(true);
            probe.raise(_Z_PROBE_DEPLOY_HEIGHT);
          }
          else {
            SERIAL_EM("Checking Diagonal Rod Length");
            if (adj_diagrod_length() != 0) { 
              // If diagonal rod length has been changed .. home to endstops
              SERIAL_EM("Diagonal Rod Length changed .. Homing");
              mechanics.Home(true);
              probe.raise(_Z_PROBE_DEPLOY_HEIGHT);
            }
          }
          bed_probe_all();
          calibration_report();
        }

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_EMV("bed_level_c = ", bed_level_c, 4);
            SERIAL_EMV("bed_level_x = ", bed_level_x, 4);
            SERIAL_EMV("bed_level_y = ", bed_level_y, 4);
            SERIAL_EMV("bed_level_z = ", bed_level_z, 4);
            SERIAL_EMV("bed_level_ox = ", bed_level_ox, 4);
            SERIAL_EMV("bed_level_oy = ", bed_level_oy, 4);
            SERIAL_EMV("bed_level_oz = ", bed_level_oz, 4);
          }
        #endif
      } while(FABS(bed_level_c) > ac_prec
           or FABS(bed_level_x) > ac_prec
           or FABS(bed_level_y) > ac_prec
           or FABS(bed_level_z) > ac_prec
           or FABS(bed_level_ox) > ac_prec
           or FABS(bed_level_oy) > ac_prec
           or FABS(bed_level_oz) > ac_prec);

      SERIAL_EM("Autocalibration Complete");
    }

    probe.set_deployed(false);

    // reset LCD alert message
    lcd_reset_alert_level();

    clean_up_after_endstop_or_probe_move();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G33");
    #endif

    mechanics.report_current_position();
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif // DELTA_AUTO_CALIBRATION_1, DELTA_AUTO_CALIBRATION_2 or DELTA_AUTO_CALIBRATION_3

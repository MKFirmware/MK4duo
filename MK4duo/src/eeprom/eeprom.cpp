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
 * eeprom.cpp
 *
 * Configuration and EEPROM storage
 *
 * IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
 * in the functions below, also increment the version number. This makes sure that
 * the default values are used whenever there is a change to the data, to prevent
 * wrong data being written to the variables.
 *
 * ALSO: Variables in the Store and Retrieve sections must be in the same order.
 *       If a feature is disabled, some data must still be written that, when read,
 *       either sets a Sane Default, or results in No Change to the existing value.
 *
 */

#include "../../base.h"

#define EEPROM_VERSION "MKV37"

/**
 * MKV437 EEPROM Layout:
 *
 *  Version (char x6)
 *  EEPROM Checksum (uint16_t)
 *
 *  M92   XYZ E0 ...      mechanics.axis_steps_per_mm X,Y,Z,E0 ... (float x9)
 *  M203  XYZ E0 ...      mechanics.max_feedrate_mm_s X,Y,Z,E0 ... (float x9)
 *  M201  XYZ E0 ...      mechanics.max_acceleration_mm_per_s2 X,Y,Z,E0 ... (uint32_t x9)
 *  M204  P               mechanics.acceleration                (float)
 *  M204  R   E0 ...      mechanics.retract_acceleration        (float x6)
 *  M204  T               mechanics.travel_acceleration         (float)
 *  M205  S               mechanics.min_feedrate_mm_s           (float)
 *  M205  T               mechanics.min_travel_feedrate_mm_s    (float)
 *  M205  B               mechanics.min_segment_time            (ulong)
 *  M205  X               mechanics.max_jerk[X_AXIS]            (float)
 *  M205  Y               mechanics.max_jerk[Y_AXIS]            (float)
 *  M205  Z               mechanics.max_jerk[Z_AXIS]            (float)
 *  M205  E   E0 ...      mechanics.max_jerk[E_AXIS * EXTRDURES](float x6)
 *  M206  XYZ             mechanics.home_offset                 (float x3)
 *  M218  T   XY          tools.hotend_offset                   (float x6)
 *
 * Global Leveling:
 *                        z_fade_height                         (float)
 *
 * MESH_BED_LEVELING:
 *  M420  S               from mbl.status                       (bool)
 *                        mbl.zprobe_zoffset                    (float)
 *                        GRID_MAX_POINTS_X                     (uint8 as set in firmware)
 *                        GRID_MAX_POINTS_Y                     (uint8 as set in firmware)
 *  G29   S3  XYZ         z_values[][]                          (float x9, by default, up to float x 81) +288
 *
 * ABL_PLANAR:
 *                        bedlevel.matrix                       (matrix_3x3 = float x9)
 *
 * AUTO_BED_LEVELING_BILINEAR:
 *                        GRID_MAX_POINTS_X                     (uint8_t)
 *                        GRID_MAX_POINTS_Y                     (uint8_t)
 *                        bedlevel.bilinear_grid_spacing        (int x2)   from G29: (B-F)/X, (R-L)/Y
 *  G29   L F             bedlevel.bilinear_start               (int x2)
 *                        bedlevel.z_values[][]                 (float x9, up to float x256)
 *
 * AUTO_BED_LEVELING_UBL:
 *  G29 A                 ubl.state.active                      (bool)
 *  G29 Z                 ubl.state.z_offset                    (float)
 *  G29 S                 ubl.state.storage_slot                (int8_t)
 *
 * HAS_BED_PROBE:
 *  M851  XYZ             probe.offset                          (float x3)
 *
 * DELTA:
 *  M666  XYZ             mechanics.delta_endstop_adj           (float x3)
 *  M666  R               mechanics.delta_radius                (float)
 *  M666  D               mechanics.delta_diagonal_rod          (float)
 *  M666  S               mechanics.delta_segments_per_second   (float)
 *  M666  H               mechanics.delta_height                (float)
 *  M666  ABC             mechanics.delta_diagonal_rod_adj      (float x3)
 *  M666  IJK             mechanics.delta_tower_angle_adj       (float x3)
 *  M666  UVW             mechanics.delta_tower_radius_adj      (float x3)
 *  M666  O               mechanics.delta_print_radius          (float)
 *  M666  P               mechanics.delta_probe_radius          (float)
 *
 * ULTIPANEL:
 *  M145  S0  H           lcd_preheat_hotend_temp               (int x3)
 *  M145  S0  B           lcd_preheat_bed_temp                  (int x3)
 *  M145  S0  F           lcd_preheat_fan_speed                 (int x3)
 *
 *
 * HEATERS AD595:
 *  M595  H   OS          Heaters AD595 Offset & Gain
 *
 * PIDTEMP:
 *  M301  E0  PIDC        Kp[0], Ki[0], Kd[0], Kc[0]            (float x4)
 *  M301  E1  PIDC        Kp[1], Ki[1], Kd[1], Kc[1]            (float x4)
 *  M301  E2  PIDC        Kp[2], Ki[2], Kd[2], Kc[2]            (float x4)
 *  M301  E3  PIDC        Kp[3], Ki[3], Kd[3], Kc[3]            (float x4)
 *  M301  L               thermalManager.lpq_len
 *
 * PIDTEMPBED:
 *  M304      PID         Kp, Ki, Kd                            (float x3)
 * PIDTEMPCHAMBER
 *  M305      PID         Kp, Ki, Kd                            (float x3)
 * PIDTEMPCOOLER
 *  M306      PID         Kp, Ki, Kd                            (float x3)
 *
 * DOGLCD:
 *  M250  C               lcd_contrast                          (uint16_t)
 *
 * FWRETRACT:
 *  M209  S               fwretract.autoretract_enabled                 (bool)
 *  M207  S               fwretract.retract_length                      (float)
 *  M207  F               fwretract.retract_feedrate_mm_s               (float)
 *  M207  Z               fwretract.retract_zlift                       (float)
 *  M208  S               fwretract.retract_recover_length              (float)
 *  M208  F               fwretract.retract_recover_feedrate_mm_s       (float)
 *  M207  W               fwretract.swap_retract_length                 (float)
 *  M208  W               fwretract.swap_retract_recover_length         (float)
 *  M208  R               fwretract.swap_retract_recover_feedrate_mm_s  (float)
 *
 * Volumetric Extrusion:
 *  M200  D               tools.volumetric_enabled              (bool)
 *  M200  T D             tools.filament_size                   (float x6)
 *
 *  M???  S               printer.IDLE_OOZING_enabled
 *
 * ALLIGATOR:
 *  M906  XYZ T0-4 E      Motor current                         (float x7)
 *
 * HAVE_TMC2130:
 *  M906  X               stepperX current                      (uint16_t)
 *  M906  Y               stepperY current                      (uint16_t)
 *  M906  Z               stepperZ current                      (uint16_t)
 *  M906  X2              stepperX2 current                     (uint16_t)
 *  M906  Y2              stepperY2 current                     (uint16_t)
 *  M906  Z2              stepperZ2 current                     (uint16_t)
 *  M906  E0              stepperE0 current                     (uint16_t)
 *  M906  E1              stepperE1 current                     (uint16_t)
 *  M906  E2              stepperE2 current                     (uint16_t)
 *  M906  E3              stepperE3 current                     (uint16_t)
 *  M906  E4              stepperE4 current                     (uint16_t)
 *  M906  E5              stepperE5 current                     (uint16_t)
 *
 * LIN_ADVANCE:
 *  M900  K               planner.extruder_advance_k            (float)
 *  M900  WHD             planner.advance_ed_ratio              (float)
 *
 */

EEPROM eeprom;

#if HAS_EEPROM_SD
  SdFile eeprom_file;
#endif

/**
 * Post-process after Retrieve or Reset
 */
void EEPROM::Postprocess() {
  // steps per s2 needs to be updated to agree with units per s2
  mechanics.reset_acceleration_rates();

  // Make sure delta kinematics are updated before refreshing the
  // planner position so the stepper counts will be set correctly.
  #if MECH(DELTA)
    mechanics.recalc_delta_settings();
  #endif

  // Refresh steps_to_mm with the reciprocal of axis_steps_per_mm
  // and init stepper.count[], planner.position[] with current_position
  mechanics.refresh_positioning();

  #if HAS_PID
    thermalManager.updatePID();
  #endif

  printer.calculate_volumetric_multipliers();

  #if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
    // Software endstops depend on home_offset
    LOOP_XYZ(i) endstops.update_software_endstops((AxisEnum)i);
  #endif

  #if HAS_LEVELING && ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    bedlevel.set_z_fade_height(bedlevel.z_fade_height);
  #endif

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    bedlevel.refresh_bed_level();
  #endif

  #if ENABLED(HYSTERESIS)
    mechanics.calc_hysteresis_steps();
  #endif
}

#if HAS_EEPROM

  #define EEPROM_START()    int eeprom_index = EEPROM_OFFSET
  #define EEPROM_SKIP(VAR)  eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR) write_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ(VAR)  read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)

  const char version[6] = EEPROM_VERSION;

  bool EEPROM::eeprom_error = false;

  #if ENABLED(AUTO_BED_LEVELING_UBL)
    int EEPROM::meshes_begin = 0;
  #endif

  void EEPROM::crc16(uint16_t *crc, const void * const data, uint16_t cnt) {
    uint8_t *ptr = (uint8_t *)data;
    while (cnt--) {
      *crc = (uint16_t)(*crc ^ (uint16_t)(((uint16_t)*ptr++) << 8));
      for (uint8_t x = 0; x < 8; x++)
        *crc = (uint16_t)((*crc & 0x8000) ? ((uint16_t)(*crc << 1) ^ 0x1021) : (*crc << 1));
    }
  }

  void EEPROM::write_data(int &pos, const uint8_t *value, uint16_t size, uint16_t *crc) {
    if (eeprom_error) return;

    while(size--) {

      #if HAS_EEPROM_SD

        uint8_t v = *value;
        if (!card.write_data(&eeprom_file, v)) {
          SERIAL_LM(ECHO, MSG_ERR_EEPROM_WRITE);
          eeprom_error = true;
          return;
        }

      #else

        uint8_t * const p = (uint8_t * const)pos;
        uint8_t v = *value;
        // EEPROM has only ~100,000 write cycles,
        // so only write bytes that have changed!
        if (v != eeprom_read_byte(p)) {
          eeprom_write_byte(p, v);
          if (eeprom_read_byte(p) != v) {
            SERIAL_LM(ECHO, MSG_ERR_EEPROM_WRITE);
            eeprom_error = true;
            return;
          }
        }
      #endif

      crc16(crc, &v, 1);
      pos++;
      value++;
    };
  }

  void EEPROM::read_data(int &pos, uint8_t *value, uint16_t size, uint16_t *crc) {
    if (eeprom_error) return;

    do {
      #if HAS_EEPROM_SD
        uint8_t c = card.read_data(&eeprom_file);
      #else
        uint8_t c = eeprom_read_byte((unsigned char*)pos);
      #endif
      *value = c;
      crc16(crc, &c, 1);
      pos++;
      value++;
    } while (--size);
  }

  /**
   * M500 - Store Configuration
   */
  bool EEPROM::Store_Settings() {
    char ver[6] = "00000";

    uint16_t working_crc = 0;

    EEPROM_START();

    eeprom_error = false;

    #if HAS_EEPROM_SD
      // EEPROM on SDCARD
      if (!IS_SD_INSERTED) {
        SERIAL_LM(ER, MSG_NO_CARD);
        return false;
      }
      else if (card.sdprinting || !card.cardOK)
        return false;
      else {
        card.setroot();
        eeprom_file.open(card.curDir, "EEPROM.bin", O_CREAT | O_APPEND | O_WRITE | O_TRUNC);
        eeprom_file.truncate(0);
        EEPROM_WRITE(version);
      }
    #else
      // EEPROM on SPI or IC2
      EEPROM_WRITE(ver);        // invalidate data first
      EEPROM_SKIP(working_crc); // Skip the checksum slot
    #endif

    working_crc = 0; // clear before first "real data"

    EEPROM_WRITE(mechanics.axis_steps_per_mm);
    EEPROM_WRITE(mechanics.max_feedrate_mm_s);
    EEPROM_WRITE(mechanics.max_acceleration_mm_per_s2);
    EEPROM_WRITE(mechanics.acceleration);
    EEPROM_WRITE(mechanics.retract_acceleration);
    EEPROM_WRITE(mechanics.travel_acceleration);
    EEPROM_WRITE(mechanics.min_feedrate_mm_s);
    EEPROM_WRITE(mechanics.min_travel_feedrate_mm_s);
    EEPROM_WRITE(mechanics.min_segment_time);
    EEPROM_WRITE(mechanics.max_jerk);
    #if ENABLED(WORKSPACE_OFFSETS)
      EEPROM_WRITE(mechanics.home_offset);
    #endif
    EEPROM_WRITE(tools.hotend_offset);

    //
    // General Leveling
    //
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      EEPROM_WRITE(bedlevel.z_fade_height);
    #endif

    //
    // Mesh Bed Leveling
    //
    #if ENABLED(MESH_BED_LEVELING)
      static_assert(
        sizeof(mbl.z_values) == GRID_MAX_POINTS * sizeof(mbl.z_values[0][0]),
        "MBL Z array is the wrong size."
      );
      const bool leveling_is_on = TEST(mbl.status, MBL_STATUS_HAS_MESH_BIT);
      const uint8_t mesh_num_x = GRID_MAX_POINTS_X, mesh_num_y = GRID_MAX_POINTS_Y;
      EEPROM_WRITE(leveling_is_on);
      EEPROM_WRITE(mbl.zprobe_zoffset);
      EEPROM_WRITE(mesh_num_x);
      EEPROM_WRITE(mesh_num_y);
      EEPROM_WRITE(mbl.z_values);
    #endif // MESH_BED_LEVELING

    //
    // Planar Bed Leveling matrix
    //
    #if ABL_PLANAR
      EEPROM_WRITE(bedlevel.matrix);
    #endif

    //
    // Bilinear Auto Bed Leveling
    //
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      static_assert(
        sizeof(bedlevel.z_values) == GRID_MAX_POINTS * sizeof(bedlevel.z_values[0][0]),
        "Bilinear Z array is the wrong size."
      );
      const uint8_t grid_max_x = GRID_MAX_POINTS_X, grid_max_y = GRID_MAX_POINTS_Y;
      EEPROM_WRITE(grid_max_x);             // 1 byte
      EEPROM_WRITE(grid_max_y);             // 1 byte
      EEPROM_WRITE(bedlevel.bilinear_grid_spacing);  // 2 ints
      EEPROM_WRITE(bedlevel.bilinear_start);         // 2 ints
      EEPROM_WRITE(bedlevel.z_values);               // 9-256 floats
    #endif // AUTO_BED_LEVELING_BILINEAR

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      EEPROM_WRITE(ubl.state.active);
      EEPROM_WRITE(ubl.state.z_offset);
      EEPROM_WRITE(ubl.state.storage_slot);
    #endif

    #if HAS_BED_PROBE
      EEPROM_WRITE(probe.offset);
    #endif

    #if MECH(DELTA)
      EEPROM_WRITE(mechanics.delta_endstop_adj);
      EEPROM_WRITE(mechanics.delta_radius);
      EEPROM_WRITE(mechanics.delta_diagonal_rod);
      EEPROM_WRITE(mechanics.delta_segments_per_second);
      EEPROM_WRITE(mechanics.delta_height);
      EEPROM_WRITE(mechanics.delta_tower_angle_adj);
      EEPROM_WRITE(mechanics.delta_tower_radius_adj);
      EEPROM_WRITE(mechanics.delta_diagonal_rod_adj);
      EEPROM_WRITE(mechanics.delta_print_radius);
      EEPROM_WRITE(mechanics.delta_probe_radius);
    #endif

    #if ENABLED(Z_FOUR_ENDSTOPS)
      EEPROM_WRITE(endstops.z2_endstop_adj);
      EEPROM_WRITE(endstops.z3_endstop_adj);
      EEPROM_WRITE(endstops.z4_endstop_adj);
    #elif ENABLED(Z_THREE_ENDSTOPS)
      EEPROM_WRITE(endstops.z3_endstop_adj);
      EEPROM_WRITE(endstops.z4_endstop_adj);
    #elif ENABLED(Z_TWO_ENDSTOPS)
      EEPROM_WRITE(endstops.z2_endstop_adj);
    #endif

    #if DISABLED(ULTIPANEL)
      const int lcd_preheat_hotend_temp[3] = { PREHEAT_1_TEMP_HOTEND, PREHEAT_2_TEMP_HOTEND, PREHEAT_3_TEMP_HOTEND },
                lcd_preheat_bed_temp[3] = { PREHEAT_1_TEMP_BED, PREHEAT_2_TEMP_BED, PREHEAT_3_TEMP_BED },
                lcd_preheat_fan_speed[3] = { PREHEAT_1_FAN_SPEED, PREHEAT_2_FAN_SPEED, PREHEAT_3_FAN_SPEED };
    #endif

    EEPROM_WRITE(lcd_preheat_hotend_temp);
    EEPROM_WRITE(lcd_preheat_bed_temp);
    EEPROM_WRITE(lcd_preheat_fan_speed);

    LOOP_HEATER() {

      #if HEATER_USES_AD595
        EEPROM_WRITE(heaters[h].ad595_offset);
        EEPROM_WRITE(heaters[h].ad595_gain);
      #endif

      EEPROM_WRITE(heaters[h].Kp);
      EEPROM_WRITE(heaters[h].Ki);
      EEPROM_WRITE(heaters[h].Kd);
      EEPROM_WRITE(heaters[h].Kc);
    }
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      EEPROM_WRITE(thermalManager.lpq_len);
    #endif

    #if !HAS_LCD_CONTRAST
      const uint16_t lcd_contrast = 32;
    #endif
    EEPROM_WRITE(lcd_contrast);

    #if ENABLED(FWRETRACT)
      EEPROM_WRITE(fwretract.autoretract_enabled);
      EEPROM_WRITE(fwretract.retract_length);
      EEPROM_WRITE(fwretract.retract_feedrate_mm_s);
      EEPROM_WRITE(fwretract.retract_zlift);
      EEPROM_WRITE(fwretract.retract_recover_length);
      EEPROM_WRITE(fwretract.retract_recover_feedrate_mm_s);
      EEPROM_WRITE(fwretract.swap_retract_length);
      EEPROM_WRITE(fwretract.swap_retract_recover_length);
      EEPROM_WRITE(fwretract.swap_retract_recover_feedrate_mm_s);
    #endif // FWRETRACT

    EEPROM_WRITE(tools.volumetric_enabled);

    // Save filament sizes
    for (uint8_t e = 0; e < EXTRUDERS; e++)
      EEPROM_WRITE(tools.filament_size[e]);

    #if ENABLED(IDLE_OOZING_PREVENT)
      EEPROM_WRITE(printer.IDLE_OOZING_enabled);
    #endif

    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      EEPROM_WRITE(printer.motor_current);
    #endif

    // Save TCM2130 Configuration, and placeholder values
    #if ENABLED(HAVE_TMC2130)
      uint16_t val;
      #if ENABLED(X_IS_TMC2130)
        val = stepperX.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(Y_IS_TMC2130)
        val = stepperY.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(Z_IS_TMC2130)
        val = stepperZ.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(X2_IS_TMC2130)
        val = stepperX2.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(Y2_IS_TMC2130)
        val = stepperY2.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(Z2_IS_TMC2130)
        val = stepperZ2.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E0_IS_TMC2130)
        val = stepperE0.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E1_IS_TMC2130)
        val = stepperE1.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E2_IS_TMC2130)
        val = stepperE2.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E3_IS_TMC2130)
        val = stepperE3.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E4_IS_TMC2130)
        val = stepperE4.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E5_IS_TMC2130)
        val = stepperE5.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
    #endif

    //
    // Linear Advance
    //
    #if ENABLED(LIN_ADVANCE)
      EEPROM_WRITE(planner.extruder_advance_k);
      EEPROM_WRITE(planner.advance_ed_ratio);
    #endif

    if (!eeprom_error) {
      const int eeprom_size = eeprom_index;

      const uint16_t final_crc = working_crc;

      // Write the EEPROM header
      eeprom_index = EEPROM_OFFSET;
      EEPROM_WRITE(version);
      EEPROM_WRITE(final_crc);

      // Report storage size
      SERIAL_SMV(ECHO, "Settings Stored (", eeprom_size - (EEPROM_OFFSET));
      SERIAL_MV(" bytes; crc ", final_crc);
      SERIAL_EM(")");
    }

    #if HAS_EEPROM_SD
      eeprom_file.sync();
      eeprom_file.close();
      card.setlast();
    #endif

    return !eeprom_error;
  }

  /**
   * M501 - Load Configuration
   */
  bool EEPROM::Load_Settings() {
    uint16_t working_crc = 0;

    EEPROM_START();

    char stored_ver[6];
    uint16_t stored_crc;

    eeprom_error = false;

    #if HAS_EEPROM_SD
      // EEPROM on SDCARD
      if (!IS_SD_INSERTED) {
        SERIAL_LM(ER, MSG_NO_CARD);
        return false;
      }
      else if (card.sdprinting || !card.cardOK)
        return false;
      else {
        card.setroot();
        eeprom_file.open(card.curDir, "EEPROM.bin", O_READ);
        EEPROM_READ(stored_ver);
      }
    #else
      EEPROM_READ(stored_ver);
      EEPROM_READ(stored_crc);
    #endif

    if (strncmp(version, stored_ver, 5) != 0) {
      if (stored_ver[0] != 'M') {
        stored_ver[0] = '?';
        stored_ver[1] = '?';
        stored_ver[2] = '\0';
      }
      SERIAL_SM(ECHO, "EEPROM version mismatch ");
      SERIAL_MT("(EEPROM=", stored_ver);
      SERIAL_EM(" MK4duo=" EEPROM_VERSION ")");
      Factory_Settings();
    }
    else {
      float dummy = 0;

      working_crc = 0; // clear before reading first "real data"

      // version number match
      EEPROM_READ(mechanics.axis_steps_per_mm);
      EEPROM_READ(mechanics.max_feedrate_mm_s);
      EEPROM_READ(mechanics.max_acceleration_mm_per_s2);
      EEPROM_READ(mechanics.acceleration);
      EEPROM_READ(mechanics.retract_acceleration);
      EEPROM_READ(mechanics.travel_acceleration);
      EEPROM_READ(mechanics.min_feedrate_mm_s);
      EEPROM_READ(mechanics.min_travel_feedrate_mm_s);
      EEPROM_READ(mechanics.min_segment_time);
      EEPROM_READ(mechanics.max_jerk);
      #if ENABLED(WORKSPACE_OFFSETS)
        EEPROM_READ(mechanics.home_offset);
      #endif
      EEPROM_READ(tools.hotend_offset);

      //
      // General Leveling
      //
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        EEPROM_READ(bedlevel.z_fade_height);
      #endif

      //
      // Mesh (Manual) Bed Leveling
      //
      #if ENABLED(MESH_BED_LEVELING)
        bool leveling_is_on;
        uint8_t mesh_num_x = 0, mesh_num_y = 0;
        EEPROM_READ(leveling_is_on);
        EEPROM_READ(dummy);
        EEPROM_READ(mesh_num_x);
        EEPROM_READ(mesh_num_y);
        mbl.status = leveling_is_on ? _BV(MBL_STATUS_HAS_MESH_BIT) : 0;
        mbl.zprobe_zoffset = dummy;
        if (mesh_num_x == GRID_MAX_POINTS_X && mesh_num_y == GRID_MAX_POINTS_Y) {
          // EEPROM data fits the current mesh
          EEPROM_READ(mbl.z_values);
        }
        else {
          // EEPROM data is stale
          mbl.reset();
          for (uint8_t q = 0; q < mesh_num_x * mesh_num_y; q++) EEPROM_READ(dummy);
        }
      #endif // MESH_BED_LEVELING

      //
      // Planar Bed Leveling matrix
      //
      #if ABL_PLANAR
        EEPROM_READ(bedlevel.matrix);
      #endif

      //
      // Bilinear Auto Bed Leveling
      //
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        uint8_t grid_max_x, grid_max_y;
        EEPROM_READ(grid_max_x);              // 1 byte
        EEPROM_READ(grid_max_y);              // 1 byte
        if (grid_max_x == GRID_MAX_POINTS_X && grid_max_y == GRID_MAX_POINTS_Y) {
          bedlevel.set_bed_leveling_enabled(false);
          EEPROM_READ(bedlevel.bilinear_grid_spacing); // 2 ints
          EEPROM_READ(bedlevel.bilinear_start);        // 2 ints
          EEPROM_READ(bedlevel.z_values);              // 9 to 256 floats
        }
        else { // EEPROM data is stale
          // Skip past disabled (or stale) Bilinear Grid data
          int bgs[2], bs[2];
          EEPROM_READ(bgs);
          EEPROM_READ(bs);
          for (uint16_t q = grid_max_x * grid_max_y; q--;) EEPROM_READ(dummy);
        }
      #endif // AUTO_BED_LEVELING_BILINEAR

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        EEPROM_READ(ubl.state.active);
        EEPROM_READ(ubl.state.z_offset);
        EEPROM_READ(ubl.state.storage_slot);
      #endif

      #if HAS_BED_PROBE
        EEPROM_READ(probe.offset);
      #endif

      #if MECH(DELTA)
        EEPROM_READ(mechanics.delta_endstop_adj);
        EEPROM_READ(mechanics.delta_radius);
        EEPROM_READ(mechanics.delta_diagonal_rod);
        EEPROM_READ(mechanics.delta_segments_per_second);
        EEPROM_READ(mechanics.delta_height);
        EEPROM_READ(mechanics.delta_tower_angle_adj);
        EEPROM_READ(mechanics.delta_tower_radius_adj);
        EEPROM_READ(mechanics.delta_diagonal_rod_adj);
        EEPROM_READ(mechanics.delta_print_radius);
        EEPROM_READ(mechanics.delta_probe_radius);
      #endif

      #if ENABLED(Z_FOUR_ENDSTOPS)
        EEPROM_READ(endstops.z2_endstop_adj);
        EEPROM_READ(endstops.z3_endstop_adj);
        EEPROM_READ(endstops.z4_endstop_adj);
      #elif ENABLED(Z_THREE_ENDSTOPS)
        EEPROM_READ(endstops.z3_endstop_adj);
        EEPROM_READ(endstops.z4_endstop_adj);
      #elif ENABLED(Z_TWO_ENDSTOPS)
        EEPROM_READ(endstops.z2_endstop_adj);
      #endif

      #if DISABLED(ULTIPANEL)
        int lcd_preheat_hotend_temp[3], lcd_preheat_bed_temp[3], lcd_preheat_fan_speed[3];
      #endif

      EEPROM_READ(lcd_preheat_hotend_temp);
      EEPROM_READ(lcd_preheat_bed_temp);
      EEPROM_READ(lcd_preheat_fan_speed);

      LOOP_HEATER() {

        #if HEATER_USES_AD595
          EEPROM_READ(heaters[h].ad595_offset);
          EEPROM_READ(heaters[h].ad595_gain);
          if (heaters[h].ad595_gain == 0) heaters[h].ad595_gain = TEMP_SENSOR_AD595_GAIN;
        #endif

        EEPROM_READ(heaters[h].Kp);
        EEPROM_READ(heaters[h].Ki);
        EEPROM_READ(heaters[h].Kd);
        EEPROM_READ(heaters[h].Kc);
      }
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        EEPROM_READ(thermalManager.lpq_len);
      #endif

      #if !HAS_LCD_CONTRAST
        uint16_t lcd_contrast;
      #endif
      EEPROM_READ(lcd_contrast);

      #if ENABLED(FWRETRACT)
        EEPROM_READ(fwretract.autoretract_enabled);
        EEPROM_READ(fwretract.retract_length);
        EEPROM_READ(fwretract.retract_feedrate_mm_s);
        EEPROM_READ(fwretract.retract_zlift);
        EEPROM_READ(fwretract.retract_recover_length);
        EEPROM_READ(fwretract.retract_recover_feedrate_mm_s);
        EEPROM_READ(fwretract.swap_retract_length);
        EEPROM_READ(fwretract.swap_retract_recover_length);
        EEPROM_READ(fwretract.swap_retract_recover_feedrate_mm_s);
      #endif // FWRETRACT

      EEPROM_READ(tools.volumetric_enabled);

      for (int8_t e = 0; e < EXTRUDERS; e++)
        EEPROM_READ(tools.filament_size[e]);

      #if ENABLED(IDLE_OOZING_PREVENT)
        EEPROM_READ(printer.IDLE_OOZING_enabled);
      #endif

      #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
        EEPROM_READ(printer.motor_current);
      #endif

      #if ENABLED(HAVE_TMC2130)
        uint16_t val;
        EEPROM_READ(val);
        #if ENABLED(X_IS_TMC2130)
          stepperX.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(Y_IS_TMC2130)
          stepperY.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(Z_IS_TMC2130)
          stepperZ.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(X2_IS_TMC2130)
          stepperX2.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(Y2_IS_TMC2130)
          stepperY2.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(Z2_IS_TMC2130)
          stepperZ2.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E0_IS_TMC2130)
          stepperE0.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E1_IS_TMC2130)
          stepperE1.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E2_IS_TMC2130)
          stepperE2.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E3_IS_TMC2130)
          stepperE3.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E4_IS_TMC2130)
          stepperE4.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E5_IS_TMC2130)
          stepperE5.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
      #endif

      //
      // Linear Advance
      //
      #if ENABLED(LIN_ADVANCE)
        EEPROM_READ(planner.extruder_advance_k);
        EEPROM_READ(planner.advance_ed_ratio);
      #endif

      #if HAS_EEPROM_SD

        eeprom_file.sync();
        eeprom_file.close();
        card.setlast();

        if (eeprom_error)
          Factory_Settings();
        else {
          Postprocess();
          SERIAL_VAL(version);
          SERIAL_MV(" stored settings retrieved (", eeprom_index - (EEPROM_OFFSET));
          SERIAL_EM(" bytes)");
        }

      #else

        if (working_crc == stored_crc) {
          Postprocess();
          SERIAL_VAL(version);
          SERIAL_MV(" stored settings retrieved (", eeprom_index - (EEPROM_OFFSET));
          SERIAL_MV(" bytes; crc ", working_crc);
          SERIAL_EM(")");
        }
        else {
          SERIAL_SMV(ER, "EEPROM CRC mismatch - (stored) ", stored_crc);
          SERIAL_MV(" != ", working_crc);
          SERIAL_EM(" (calculated)!");
          Factory_Settings();
        }

      #endif

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        meshes_begin = (eeprom_index + 32) & 0xFFF8;  // Pad the end of configuration data so it
                                                      // can float up or down a little bit without
                                                      // disrupting the mesh data
        ubl.report_state();

        if (!ubl.sanity_check()) {
          SERIAL_EOL();
          #if ENABLED(EEPROM_CHITCHAT)
            ubl.echo_name();
            SERIAL_EM(" initialized.");
          #endif
        }
        else {
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_MSG("?Can't enable ");
            ubl.echo_name();
            SERIAL_EM(".");
          #endif
          ubl.reset();
        }

        if (ubl.state.storage_slot >= 0) {
          load_mesh(ubl.state.storage_slot);
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_MV("Mesh ", ubl.state.storage_slot);
            SERIAL_EM(" loaded from storage.");
          #endif
        }
        else {
          ubl.reset();
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_EM("UBL System reset()");
          #endif
        }
      #endif
    }

    #if ENABLED(EEPROM_CHITCHAT)
      Print_Settings();
    #endif

    return !eeprom_error;
  }

  #if ENABLED(AUTO_BED_LEVELING_UBL)

    #if ENABLED(EEPROM_CHITCHAT)
      void ubl_invalid_slot(const int s) {
        SERIAL_EM("?Invalid slot.");
        SERIAL_VAL(s);
        SERIAL_EM(" mesh slots available.");
      }
    #endif

    int EEPROM::calc_num_meshes() {
      if (meshes_begin <= 0) return 0;
      return (meshes_end - meshes_begin) / sizeof(ubl.z_values);
    }

    void EEPROM::store_mesh(int8_t slot) {

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        const int a = calc_num_meshes();
        if (!WITHIN(slot, 0, a - 1)) {
          #if ENABLED(EEPROM_CHITCHAT)
            ubl_invalid_slot(a);
            SERIAL_MV("E2END=", E2END);
            SERIAL_MV(" meshes_end=", meshes_end);
            SERIAL_EMV(" slot=", slot);
          #endif
          return;
        }

        uint16_t crc = 0;
        int pos = meshes_end - (slot + 1) * sizeof(ubl.z_values);

        write_data(pos, (uint8_t *)&ubl.z_values, sizeof(ubl.z_values), &crc);

        // Write crc to MAT along with other data, or just tack on to the beginning or end

        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_EMV("Mesh saved in slot ", slot);
        #endif

      #else

        // Other mesh types

      #endif
    }

    void EEPROM::load_mesh(int8_t slot, void *into /* = 0 */) {

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        const int16_t a = calc_num_meshes();

        if (!WITHIN(slot, 0, a - 1)) {
          #if ENABLED(EEPROM_CHITCHAT)
            ubl_invalid_slot(a);
          #endif
          return;
        }

        uint16_t crc = 0;
        int pos = meshes_end - (slot + 1) * sizeof(ubl.z_values);
        uint8_t * const dest = into ? (uint8_t*)into : (uint8_t*)&ubl.z_values;
        read_data(pos, dest, sizeof(ubl.z_values), &crc);

        // Compare crc with crc from MAT, or read from end

        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_EMV("Mesh loaded from slot ", slot);
        #endif

      #else

        // Other mesh types

      #endif
    }

    //void MarlinSettings::delete_mesh() { return; }
    //void MarlinSettings::defrag_meshes() { return; }

  #endif // AUTO_BED_LEVELING_UBL

#else // !EEPROM_SETTINGS

  bool EEPROM::Store_Settings() { SERIAL_LM(ER, "EEPROM disabled"); return false; }

#endif // EEPROM_SETTINGS

/**
 * M502 - Reset Configuration
 */
void EEPROM::Factory_Settings() {
  const float     tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT,
                  tmp2[] = DEFAULT_MAX_FEEDRATE;
  const uint32_t  tmp3[] = DEFAULT_MAX_ACCELERATION,
                  tmp4[] = DEFAULT_RETRACT_ACCELERATION;
  const float     tmp5[] = DEFAULT_EJERK,
                  tmp6[] = DEFAULT_Kp,
                  tmp7[] = DEFAULT_Ki,
                  tmp8[] = DEFAULT_Kd,
                  tmp9[] = DEFAULT_Kc;

  #if ENABLED(HOTEND_OFFSET_X) && ENABLED(HOTEND_OFFSET_Y) && ENABLED(HOTEND_OFFSET_Z)
    constexpr float tmp10[XYZ][4] = {
      HOTEND_OFFSET_X,
      HOTEND_OFFSET_Y,
      HOTEND_OFFSET_Z
    };
  #else
    constexpr float tmp10[XYZ][HOTENDS] = { 0.0 };
  #endif

  #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
    const float tmp11[] = MOTOR_CURRENT;
    for (uint8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++)
      printer.motor_current[i] = tmp11[i < COUNT(tmp11) ? i : COUNT(tmp11) - 1];
  #endif

  LOOP_XYZE_N(i) {
    mechanics.axis_steps_per_mm[i]          = tmp1[i < COUNT(tmp1) ? i : COUNT(tmp1) - 1];
    mechanics.max_feedrate_mm_s[i]          = tmp2[i < COUNT(tmp2) ? i : COUNT(tmp2) - 1];
    mechanics.max_acceleration_mm_per_s2[i] = tmp3[i < COUNT(tmp3) ? i : COUNT(tmp3) - 1];
  }

  for (uint8_t i = 0; i < EXTRUDERS; i++) {
    mechanics.retract_acceleration[i]       = tmp4[i < COUNT(tmp4) ? i : COUNT(tmp4) - 1];
    mechanics.max_jerk[E_AXIS + i]          = tmp5[i < COUNT(tmp5) ? i : COUNT(tmp5) - 1];
  }

  static_assert(
    tmp10[X_AXIS][0] == 0 && tmp10[Y_AXIS][0] == 0 && tmp10[Z_AXIS][0] == 0,
    "Offsets for the first hotend must be 0.0."
  );
  LOOP_XYZ(i) {
    LOOP_HOTEND() tools.hotend_offset[i][h] = tmp10[i][h];
  }

  mechanics.acceleration = DEFAULT_ACCELERATION;
  mechanics.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  mechanics.min_feedrate_mm_s = DEFAULT_MINIMUMFEEDRATE;
  mechanics.min_segment_time = DEFAULT_MINSEGMENTTIME;
  mechanics.min_travel_feedrate_mm_s = DEFAULT_MINTRAVELFEEDRATE;
  mechanics.max_jerk[X_AXIS] = DEFAULT_XJERK;
  mechanics.max_jerk[Y_AXIS] = DEFAULT_YJERK;
  mechanics.max_jerk[Z_AXIS] = DEFAULT_ZJERK;

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    bedlevel.z_fade_height = 0.0;
  #endif

  #if ENABLED(WORKSPACE_OFFSETS)
    ZERO(mechanics.home_offset);
  #endif

  #if HAS_LEVELING
    bedlevel.reset();
  #endif

  #if HAS_BED_PROBE
    probe.offset[0] = X_PROBE_OFFSET_FROM_NOZZLE;
    probe.offset[1] = Y_PROBE_OFFSET_FROM_NOZZLE;
    probe.offset[2] = Z_PROBE_OFFSET_FROM_NOZZLE;
  #endif

  mechanics.Init();

  #if ENABLED(ULTIPANEL)
    lcd_preheat_hotend_temp[0] = PREHEAT_1_TEMP_HOTEND;
    lcd_preheat_hotend_temp[1] = PREHEAT_2_TEMP_HOTEND;
    lcd_preheat_hotend_temp[2] = PREHEAT_3_TEMP_HOTEND;
    lcd_preheat_bed_temp[0] = PREHEAT_1_TEMP_BED;
    lcd_preheat_bed_temp[1] = PREHEAT_2_TEMP_BED;
    lcd_preheat_bed_temp[2] = PREHEAT_3_TEMP_BED;
    lcd_preheat_fan_speed[0] = PREHEAT_1_FAN_SPEED;
    lcd_preheat_fan_speed[1] = PREHEAT_2_FAN_SPEED;
    lcd_preheat_fan_speed[2] = PREHEAT_3_FAN_SPEED;
  #endif

  #if HAS_LCD_CONTRAST
    lcd_contrast = DEFAULT_LCD_CONTRAST;
  #endif

  #if (PIDTEMP)
    LOOP_HOTEND() {
      heaters[h].Kp = tmp6[h];
      heaters[h].Ki = tmp7[h];
      heaters[h].Kd = tmp8[h];
      heaters[h].Kc = tmp9[h];
    }
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      thermalManager.lpq_len = 20; // default last-position-queue size
    #endif
  #endif // PIDTEMP

  #if (PIDTEMPBED)
    heaters[BED_INDEX].Kp = DEFAULT_bedKp;
    heaters[BED_INDEX].Ki = DEFAULT_bedKi;
    heaters[BED_INDEX].Kd = DEFAULT_bedKd;
  #endif

  #if (PIDTEMPCHAMBER)
    heaters[CHAMBER_INDEX].Kp = DEFAULT_chamberKp;
    heaters[CHAMBER_INDEX].Ki = DEFAULT_chamberKi;
    heaters[CHAMBER_INDEX].Kd = DEFAULT_chamberKd;
  #endif

  #if (PIDTEMPCOOLER)
    heaters[COOLER_INDEX].Kp = DEFAULT_coolerKp;
    heaters[COOLER_INDEX].Ki = DEFAULT_coolerKi;
    heaters[COOLER_INDEX].Kd = DEFAULT_coolerKd;
  #endif

  #if ENABLED(FWRETRACT)
    fwretract.reset();
  #endif

  #if ENABLED(VOLUMETRIC_DEFAULT_ON)
    tools.volumetric_enabled = true;
  #else
    tools.volumetric_enabled = false;
  #endif

  for (uint8_t q = 0; q < COUNT(tools.filament_size); q++)
    tools.filament_size[q] = DEFAULT_NOMINAL_FILAMENT_DIA;

  endstops.enable_globally(
    #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
      (false)
    #else
      (true)
    #endif
  );

  #if ENABLED(IDLE_OOZING_PREVENT)
    printer.IDLE_OOZING_enabled = true;
  #endif

  #if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
      stepperX.setCurrent(X_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      stepperY.setCurrent(Y_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(Z_IS_TMC2130)
      stepperZ.setCurrent(Z_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(X2_IS_TMC2130)
      stepperX2.setCurrent(X2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
      stepperY2.setCurrent(Y2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
      stepperZ2.setCurrent(Z2_MAX_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E0_IS_TMC2130)
      stepperE0.setCurrent(E0_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E1_IS_TMC2130)
      stepperE1.setCurrent(E1_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E2_IS_TMC2130)
      stepperE2.setCurrent(E2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E3_IS_TMC2130)
      stepperE3.setCurrent(E3_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E4_IS_TMC2130)
      stepperE4.setCurrent(E4_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E5_IS_TMC2130)
      stepperE5.setCurrent(E5_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
  #endif

  #if ENABLED(LIN_ADVANCE)
    planner.extruder_advance_k = LIN_ADVANCE_K;
    planner.advance_ed_ratio = LIN_ADVANCE_E_D_RATIO;
  #endif

  Postprocess();

  SERIAL_LM(ECHO, "Hardcoded Default Settings Loaded");
}

#if DISABLED(DISABLE_M503)

  #define CONFIG_MSG_START(str) do{ if (!forReplay) SERIAL_STR(CFG); SERIAL_EM(str); }while(0)

  /**
   * M503 - Print Configuration
   */
  void EEPROM::Print_Settings(bool forReplay) {
    // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown

    /**
     * Announce current units, in case inches are being displayed
     */
    #if ENABLED(INCH_MODE_SUPPORT)
      #define LINEAR_UNIT(N) ((N) / parser.linear_unit_factor)
      #define VOLUMETRIC_UNIT(N) ((N) / (tools.volumetric_enabled ? parser.volumetric_unit_factor : parser.linear_unit_factor))
      SERIAL_SM(CFG, "  G2");
      SERIAL_CHR(parser.linear_unit_factor == 1.0 ? '1' : '0');
      SERIAL_MSG(" ; Units in ");
      SERIAL_PS(parser.linear_unit_factor == 1.0 ? PSTR("mm\n") : PSTR("inches\n"));
    #else
      #define LINEAR_UNIT(N) N
      #define VOLUMETRIC_UNIT(N) N
      SERIAL_LM(CFG, "  G21 ; Units in mm");
    #endif

    #if ENABLED(ULTIPANEL)

      // Temperature units - for Ultipanel temperature options

      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        #define TEMP_UNIT(N) parser.to_temp_units(N)
        SERIAL_SM(CFG, "  M149 ");
        SERIAL_CHR(parser.temp_units_code);
        SERIAL_MSG(" ; Units in ");
        SERIAL_PS(parser.temp_units_name());
      #else
        #define TEMP_UNIT(N) N
        SERIAL_LM(CFG, "  M149 C ; Units in Celsius");
      #endif

    #endif

    CONFIG_MSG_START("Steps per unit:");
    SERIAL_SMV(CFG, "  M92 X", LINEAR_UNIT(mechanics.axis_steps_per_mm[X_AXIS]), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(mechanics.axis_steps_per_mm[Y_AXIS]), 3);
    SERIAL_MV(" Z", LINEAR_UNIT(mechanics.axis_steps_per_mm[Z_AXIS]), 3);
    #if EXTRUDERS == 1
      SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(mechanics.axis_steps_per_mm[E_AXIS]), 3);
    #endif
    SERIAL_EOL();
    #if EXTRUDERS > 1
      for (int8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M92 T", i);
        SERIAL_EMV(" E", VOLUMETRIC_UNIT(mechanics.axis_steps_per_mm[E_AXIS + i]), 3);
      }
    #endif // EXTRUDERS > 1

    CONFIG_MSG_START("Maximum feedrates (units/s):");
    SERIAL_SMV(CFG, "  M203 X", LINEAR_UNIT(mechanics.max_feedrate_mm_s[X_AXIS]), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(mechanics.max_feedrate_mm_s[Y_AXIS]), 3);
    SERIAL_MV(" Z", LINEAR_UNIT(mechanics.max_feedrate_mm_s[Z_AXIS]), 3);
    #if EXTRUDERS == 1
      SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(mechanics.max_feedrate_mm_s[E_AXIS]), 3);
    #endif
    SERIAL_EOL();
    #if EXTRUDERS > 1
      for (int8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M203 T", i);
        SERIAL_EMV(" E", VOLUMETRIC_UNIT(mechanics.max_feedrate_mm_s[E_AXIS + i]), 3);
      }
    #endif // EXTRUDERS > 1

    CONFIG_MSG_START("Maximum Acceleration (units/s2):");
    SERIAL_SMV(CFG, "  M201 X", LINEAR_UNIT(mechanics.max_acceleration_mm_per_s2[X_AXIS]));
    SERIAL_MV(" Y", LINEAR_UNIT(mechanics.max_acceleration_mm_per_s2[Y_AXIS]));
    SERIAL_MV(" Z", LINEAR_UNIT(mechanics.max_acceleration_mm_per_s2[Z_AXIS]));
    #if EXTRUDERS == 1
      SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(mechanics.max_acceleration_mm_per_s2[E_AXIS]));
    #endif
    SERIAL_EOL();
    #if EXTRUDERS > 1
      for (int8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M201 T", i);
        SERIAL_EMV(" E", VOLUMETRIC_UNIT(mechanics.max_acceleration_mm_per_s2[E_AXIS + i]));
      }
    #endif // EXTRUDERS > 1

    CONFIG_MSG_START("Acceleration (units/s2): P<print_accel> V<travel_accel> T* R<retract_accel>");
    SERIAL_SMV(CFG,"  M204 P", LINEAR_UNIT(mechanics.acceleration), 3);
    SERIAL_MV(" V", LINEAR_UNIT(mechanics.travel_acceleration), 3);
    #if EXTRUDERS == 1
      SERIAL_MV(" T0 R", LINEAR_UNIT(mechanics.retract_acceleration[0]), 3);
    #endif
    SERIAL_EOL();
    #if EXTRUDERS > 1
      for (int8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M204 T", i);
        SERIAL_EMV(" R", LINEAR_UNIT(mechanics.retract_acceleration[i]), 3);
      }
    #endif

    CONFIG_MSG_START("Advanced variables: S<min_feedrate> V<min_travel_feedrate> B<min_segment_time_ms> X<max_xy_jerk> Z<max_z_jerk> T* E<max_e_jerk>");
    SERIAL_SMV(CFG, "  M205 S", LINEAR_UNIT(mechanics.min_feedrate_mm_s), 3);
    SERIAL_MV(" V", LINEAR_UNIT(mechanics.min_travel_feedrate_mm_s), 3);
    SERIAL_MV(" B", mechanics.min_segment_time);
    SERIAL_MV(" X", LINEAR_UNIT(mechanics.max_jerk[X_AXIS]), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(mechanics.max_jerk[Y_AXIS]), 3);
    SERIAL_MV(" Z", LINEAR_UNIT(mechanics.max_jerk[Z_AXIS]), 3);
    #if EXTRUDERS == 1
      SERIAL_MV(" T0 E", LINEAR_UNIT(mechanics.max_jerk[E_AXIS]), 3);
    #endif
    SERIAL_EOL();
    #if (EXTRUDERS > 1)
      for(int8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M205 T", i);
        SERIAL_EMV(" E" , LINEAR_UNIT(mechanics.max_jerk[E_AXIS + i]), 3);
      }
    #endif

    #if ENABLED(WORKSPACE_OFFSETS)
      CONFIG_MSG_START("Home offset:");
      SERIAL_SMV(CFG, "  M206 X", LINEAR_UNIT(mechanics.home_offset[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(mechanics.home_offset[Y_AXIS]), 3);
      SERIAL_EMV(" Z", LINEAR_UNIT(mechanics.home_offset[Z_AXIS]), 3);
    #endif

    #if HOTENDS > 1
      CONFIG_MSG_START("Hotend offset (mm):");
      for (int8_t h = 1; h < HOTENDS; h++) {
        SERIAL_SMV(CFG, "  M218 H", h);
        SERIAL_MV(" X", LINEAR_UNIT(tools.hotend_offset[X_AXIS][h]), 3);
        SERIAL_MV(" Y", LINEAR_UNIT(tools.hotend_offset[Y_AXIS][h]), 3);
        SERIAL_EMV(" Z", LINEAR_UNIT(tools.hotend_offset[Z_AXIS][h]), 3);
      }
    #endif

    #if HAS_LCD_CONTRAST
      CONFIG_MSG_START("LCD Contrast:");
      SERIAL_LMV(CFG, "  M250 C", lcd_contrast);
    #endif

    #if ENABLED(MESH_BED_LEVELING)

      CONFIG_MSG_START("Mesh Bed Leveling:");
      SERIAL_SMV(CFG, "  M420 S", bedlevel.leveling_is_valid() ? 1 : 0);
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        SERIAL_MV(" Z", LINEAR_UNIT(bedlevel.z_fade_height));
      #endif
      SERIAL_EOL();

      for (uint8_t py = 0; py < GRID_MAX_POINTS_Y; py++) {
        for (uint8_t px = 0; px < GRID_MAX_POINTS_X; px++) {
          SERIAL_SMV(CFG, "  G29 S3 X", (int)px + 1);
          SERIAL_MV(" Y", (int)py + 1);
          SERIAL_EMV(" Z", LINEAR_UNIT(mbl.z_values[px][py]), 5);
        }
      }

    #elif HAS_ABL

      CONFIG_MSG_START("Auto Bed Leveling:");
      SERIAL_SMV(CFG, "  M320 S", bedlevel.leveling_is_active() ? 1 : 0);
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        SERIAL_MV(" Z", LINEAR_UNIT(bedlevel.z_fade_height));
      #endif
      SERIAL_EOL();

    #endif

    #if IS_DELTA

      CONFIG_MSG_START("Endstop adjustment:");
      SERIAL_SM(CFG, "  M666");
      SERIAL_MV(" X", LINEAR_UNIT(mechanics.delta_endstop_adj[A_AXIS]));
      SERIAL_MV(" Y", LINEAR_UNIT(mechanics.delta_endstop_adj[B_AXIS]));
      SERIAL_MV(" Z", LINEAR_UNIT(mechanics.delta_endstop_adj[C_AXIS]));
      SERIAL_EOL();

      CONFIG_MSG_START("Geometry adjustment: ABC=TOWER_DIAGROD_ADJ, IJK=TOWER_ANGLE_ADJ, UVW=TOWER_RADIUS_ADJ");
      CONFIG_MSG_START("                     R=DELTA_RADIUS, D=DELTA_DIAGONAL_ROD, S=DELTA_SEGMENTS_PER_SECOND");
      CONFIG_MSG_START("                     O=DELTA_PRINTABLE_RADIUS, P=DELTA_PROBEABLE_RADIUS, H=DELTA_HEIGHT");
      SERIAL_SM(CFG, "  M666");
      SERIAL_MV(" A", LINEAR_UNIT(mechanics.delta_diagonal_rod_adj[0]), 3);
      SERIAL_MV(" B", LINEAR_UNIT(mechanics.delta_diagonal_rod_adj[1]), 3);
      SERIAL_MV(" C", LINEAR_UNIT(mechanics.delta_diagonal_rod_adj[2]), 3);
      SERIAL_MV(" I", mechanics.delta_tower_angle_adj[0], 3);
      SERIAL_MV(" J", mechanics.delta_tower_angle_adj[1], 3);
      SERIAL_MV(" K", mechanics.delta_tower_angle_adj[2], 3);
      SERIAL_MV(" U", LINEAR_UNIT(mechanics.delta_tower_radius_adj[0]), 3);
      SERIAL_MV(" V", LINEAR_UNIT(mechanics.delta_tower_radius_adj[1]), 3);
      SERIAL_MV(" W", LINEAR_UNIT(mechanics.delta_tower_radius_adj[2]), 3);
      SERIAL_MV(" R", LINEAR_UNIT(mechanics.delta_radius));
      SERIAL_MV(" D", LINEAR_UNIT(mechanics.delta_diagonal_rod));
      SERIAL_MV(" S", mechanics.delta_segments_per_second);
      SERIAL_MV(" O", LINEAR_UNIT(mechanics.delta_print_radius));
      SERIAL_MV(" P", LINEAR_UNIT(mechanics.delta_probe_radius));
      SERIAL_MV(" H", LINEAR_UNIT(mechanics.delta_height), 3);
      SERIAL_EOL();

    #endif

    /**
     * Auto Bed Leveling
     */
    #if HAS_BED_PROBE
      CONFIG_MSG_START(MSG_PROBE_OFFSET ":");
      SERIAL_SMV(CFG, "  M851 X", LINEAR_UNIT(probe.offset[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(probe.offset[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(probe.offset[Z_AXIS]), 3);
      SERIAL_EOL();
    #endif

    #if ENABLED(ULTIPANEL)
      CONFIG_MSG_START("Material heatup parameters:");
      for (int8_t i = 0; i < COUNT(lcd_preheat_hotend_temp); i++) {
        SERIAL_SMV(CFG, "  M145 S", i);
        SERIAL_MV(" H", TEMP_UNIT(lcd_preheat_hotend_temp[i]));
        SERIAL_MV(" B", TEMP_UNIT(lcd_preheat_bed_temp[i]));
        SERIAL_MV(" F", lcd_preheat_fan_speed[i]);
        SERIAL_EOL();
      }
    #endif // ULTIPANEL
    
    #if HEATER_USES_AD595
      CONFIG_MSG_START("AD595 Offset and Gain:");
      LOOP_HOTEND() {
        SERIAL_SMV(CFG, "  M595 H", h);
        SERIAL_MV(" O", heaters[h].ad595_offset);
        SERIAL_EMV(", S", heaters[h].ad595_gain);
      }
    #endif // HEATER_USES_AD595

    #if HAS_PID
      CONFIG_MSG_START("PID settings:");
      #if (PIDTEMP)
        #if HOTENDS == 1
          SERIAL_SM(CFG, "  M301 H0");
          SERIAL_MV(" P", heaters[0].Kp);
          SERIAL_MV(" I", heaters[0].Ki);
          SERIAL_MV(" D", heaters[0].Kd);
          #if ENABLED(PID_ADD_EXTRUSION_RATE)
            SERIAL_MV(" C", heaters[0].Kc);
          #endif
          SERIAL_EOL();
          #if ENABLED(PID_ADD_EXTRUSION_RATE)
            SERIAL_LMV(CFG, "  M301 L", thermalManager.lpq_len);
          #endif
        #elif HOTENDS > 1
          for (int8_t h = 0; h < HOTENDS; h++) {
            SERIAL_SMV(CFG, "  M301 H", h);
            SERIAL_MV(" P", heaters[h].Kp);
            SERIAL_MV(" I", heaters[h].Ki);
            SERIAL_MV(" D", heaters[h].Kd);
            #if ENABLED(PID_ADD_EXTRUSION_RATE)
              SERIAL_MV(" C", heaters[h].Kc);
            #endif
            SERIAL_EOL();
          }
          #if ENABLED(PID_ADD_EXTRUSION_RATE)
            SERIAL_LMV(CFG, "  M301 L", thermalManager.lpq_len);
          #endif
        #endif
      #endif
      #if (PIDTEMPBED)
        SERIAL_SMV(CFG, "  M304 P", heaters[BED_INDEX].Kp);
        SERIAL_MV(" I", heaters[BED_INDEX].Ki);
        SERIAL_EMV(" D", heaters[BED_INDEX].Kd);
      #endif
      #if (PIDTEMPCHAMBER)
        SERIAL_SMV(CFG, "  M305 P", heaters[CHAMBER_INDEX].Kp);
        SERIAL_MV(" I", heaters[CHAMBER_INDEX].Ki);
        SERIAL_EMV(" D", heaters[CHAMBER_INDEX].Kd);
      #endif
      #if (PIDTEMPCOOLER)
        SERIAL_SMV(CFG, "  M306 P", heaters[COOLER_INDEX].Kp);
        SERIAL_MV(" I", heaters[COOLER_INDEX].Ki);
        SERIAL_EMV(" D", heaters[COOLER_INDEX].Kd);
      #endif
    #endif

    #if ENABLED(FWRETRACT)
      CONFIG_MSG_START("Retract: S<length> F<units/m> Z<lift>");
      SERIAL_SMV(CFG, "  M207 S", LINEAR_UNIT(fwretract.retract_length));
      SERIAL_MV(" W", LINEAR_UNIT(fwretract.swap_retract_length));
      SERIAL_MV(" F", MMS_TO_MMM(LINEAR_UNIT(fwretract.retract_feedrate_mm_s)));
      SERIAL_EMV(" Z", LINEAR_UNIT(fwretract.retract_zlift));

      CONFIG_MSG_START("Recover: S<length> F<units/m>");
      SERIAL_SMV(CFG, "  M208 S", LINEAR_UNIT(fwretract.retract_recover_length));
      SERIAL_MV(" W", LINEAR_UNIT(fwretract.swap_retract_recover_length));
      SERIAL_MV(" F", MMS_TO_MMM(LINEAR_UNIT(fwretract.retract_recover_feedrate_mm_s)));

      CONFIG_MSG_START("Auto-Retract: S=0 to disable, 1 to interpret E-only moves as retract/recover");
      SERIAL_LMV(CFG, "  M209 S", fwretract.autoretract_enabled ? 1 : 0);
    #endif // FWRETRACT

    /**
     * Volumetric extrusion M200
     */
    if (!forReplay) {
      SERIAL_SM(CFG, "Filament settings:");
      if (tools.volumetric_enabled)
        SERIAL_EOL();
      else
        SERIAL_EM(" Disabled");
    }
    #if EXTRUDERS == 1
      SERIAL_LMV(CFG, "  M200 T0 D", tools.filament_size[0], 3);
    #endif
    #if EXTRUDERS > 1
      for (uint8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M200 T", (int)i);
        SERIAL_EMV(" D", tools.filament_size[i], 3);
      }
    #endif

    /**
     * Alligator current drivers M906
     */
    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      CONFIG_MSG_START("Motor current:");
      SERIAL_SMV(CFG, "  M906 X", printer.motor_current[X_AXIS], 2);
      SERIAL_MV(" Y", printer.motor_current[Y_AXIS], 2);
      SERIAL_MV(" Z", printer.motor_current[Z_AXIS], 2);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", printer.motor_current[E_AXIS], 2);
      #endif
      SERIAL_EOL();
      #if DRIVER_EXTRUDERS > 1
        for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
          SERIAL_SMV(CFG, "  M906 T", i);
          SERIAL_EMV(" E", printer.motor_current[E_AXIS + i], 2);
        }
      #endif // DRIVER_EXTRUDERS > 1
    #endif // ALLIGATOR

    /**
     * TMC2130 stepper driver current
     */
    #if ENABLED(HAVE_TMC2130)
      CONFIG_MSG_START("Stepper driver current:");
      SERIAL_SM(CFG, "  M906");
      #if ENABLED(X_IS_TMC2130)
        SERIAL_MV(" X", stepperX.getCurrent());
      #endif
      #if ENABLED(Y_IS_TMC2130)
        SERIAL_MV(" Y", stepperY.getCurrent());
      #endif
      #if ENABLED(Z_IS_TMC2130)
        SERIAL_MV(" Z", stepperZ.getCurrent());
      #endif
      #if ENABLED(X2_IS_TMC2130)
        SERIAL_MV(" X2", stepperX2.getCurrent());
      #endif
      #if ENABLED(Y2_IS_TMC2130)
        SERIAL_MV(" Y2", stepperY2.getCurrent());
      #endif
      #if ENABLED(Z2_IS_TMC2130)
        SERIAL_MV(" Z2", stepperZ2.getCurrent());
      #endif
      #if ENABLED(E0_IS_TMC2130)
        SERIAL_MV(" E0", stepperE0.getCurrent());
      #endif
      #if ENABLED(E1_IS_TMC2130)
        SERIAL_MV(" E1", stepperE1.getCurrent());
      #endif
      #if ENABLED(E2_IS_TMC2130)
        SERIAL_MV(" E2", stepperE2.getCurrent());
      #endif
      #if ENABLED(E3_IS_TMC2130)
        SERIAL_MV(" E3", stepperE3.getCurrent());
      #endif
      SERIAL_EOL();
      #if ENABLED(E4_IS_TMC2130)
        SERIAL_MV(" E4", stepperE4.getCurrent());
      #endif
      SERIAL_EOL();
      #if ENABLED(E5_IS_TMC2130)
        SERIAL_MV(" E5", stepperE5.getCurrent());
      #endif
      SERIAL_EOL();
    #endif

    /**
     * Linear Advance
     */
    #if ENABLED(LIN_ADVANCE)
      CONFIG_MSG_START("Linear Advance:");
      SERIAL_SMV(CFG, "  M900 K", planner.extruder_advance_k);
      SERIAL_EMV(" R", planner.advance_ed_ratio);
    #endif

    #if HAS_SDSUPPORT
      card.PrintSettings();
    #endif

  }

#endif // !DISABLE_M503

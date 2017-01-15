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

#define EEPROM_VERSION "MKV30"
#define EEPROM_OFFSET 10

/**
 * MKV430 EEPROM Layout:
 *
 *  Version (char x6)
 *  EEPROM Checksum (uint16_t)
 *
 *  M92   XYZ E0 ...      planner.axis_steps_per_mm X,Y,Z,E0 ... (float x9)
 *  M203  XYZ E0 ...      planner.max_feedrate_mm_s X,Y,Z,E0 ... (float x9)
 *  M201  XYZ E0 ...      planner.max_acceleration_mm_per_s2 X,Y,Z,E0 ... (uint32_t x9)
 *  M204  P               planner.acceleration (float)
 *  M204  R   E0 ...      planner.retract_acceleration (float x6)
 *  M204  T               planner.travel_acceleration (float)
 *  M205  S               planner.min_feedrate_mm_s (float)
 *  M205  T               planner.min_travel_feedrate_mm_s (float)
 *  M205  B               planner.min_segment_time (ulong)
 *  M205  X               planner.max_jerk[X_AXIS] (float)
 *  M205  Y               planner.max_jerk[Y_AXIS] (float)
 *  M205  Z               planner.max_jerk[Z_AXIS] (float)
 *  M205  E   E0 ...      planner.max_jerk[E_AXIS * EXTRDURES] (float x6)
 *  M206  XYZ             home_offset (float x3)
 *  M218  T   XY          hotend_offset (float x6)
 *
 * Mesh bed leveling:
 *  M420  S               from mbl.status (bool)
 *                        mbl.z_offset (float)
 *                        MESH_NUM_X_POINTS (uint8 as set in firmware)
 *                        MESH_NUM_Y_POINTS (uint8 as set in firmware)
 *  G29   S3  XYZ         z_values[][] (float x9, by default, up to float x 81) +288
 *
 * ABL_PLANAR:
 *                        planner.bed_level_matrix        (matrix_3x3 = float x9)
 *
 * AUTO_BED_LEVELING_BILINEAR:
 *                        ABL_GRID_MAX_POINTS_X           (uint8_t)
 *                        ABL_GRID_MAX_POINTS_Y           (uint8_t)
 *                        bilinear_grid_spacing           (int x2)   from G29: (B-F)/X, (R-L)/Y
 *                        bilinear_start                  (int x2)
 *                        bilinear_level_grid[][]         (float x9, up to float x256)
 *
 * Z PROBE:
 *  M666  P               zprobe_zoffset (float)
 *
 * HOTENDS AD595:
 *  M595  H OS            Hotend AD595 Offset & Gain
 *
 * DELTA:
 *  M666  XYZ             deltaParams.endstop_adj (float x3)
 *  M666  R               deltaParams.radius (float)
 *  M666  D               deltaParams.diagonal_rod (float)
 *  M666  S               deltaParams.segments_per_second (float)
 *  M666  H               deltaParams.base_max_pos (float)
 *  M666  ABCIJK          deltaParams.tower_adj (float x6)
 *  M666  UVW             deltaParams.diagonal_rod_adj (float x3)
 *
 * Z_TWO_ENDSTOPS:
 *  M666  Z               z2_endstop_adj (float)
 *
 * ULTIPANEL:
 *  M145  S0  H           lcd_preheat_hotend_temp (int x3)
 *  M145  S0  B           lcd_preheat_bed_temp (int x3)
 *  M145  S0  F           lcd_preheat_fan_speed (int x3)
 *
 * PIDTEMP:
 *  M301  E0  PIDC        Kp[0], Ki[0], Kd[0], Kc[0] (float x4)
 *  M301  E1  PIDC        Kp[1], Ki[1], Kd[1], Kc[1] (float x4)
 *  M301  E2  PIDC        Kp[2], Ki[2], Kd[2], Kc[2] (float x4)
 *  M301  E3  PIDC        Kp[3], Ki[3], Kd[3], Kc[3] (float x4)
 *  M301  L               lpq_len
 *
 * PIDTEMPBED:
 *  M304      PID         bedKp, bedKi, bedKd (float x3)
 * PIDTEMPCHAMBER
 *  M305      PID         chamberKp, chamberKi, chamberKd (float x3)
 * PIDTEMPCOOLER
 *  M306      PID         coolerKp, coolerKi, coolerKd (float x3)
 *
 * DOGLCD:
 *  M250  C               lcd_contrast (int)
 *
 * FWRETRACT:
 *  M209  S               autoretract_enabled (bool)
 *  M207  S               retract_length (float)
 *  M207  W               retract_length_swap (float)
 *  M207  F               retract_feedrate (float)
 *  M207  Z               retract_zlift (float)
 *  M208  S               retract_recover_length (float)
 *  M208  W               retract_recover_length_swap (float)
 *  M208  F               retract_recover_feedrate (float)
 *
 * Volumetric Extrusion:
 *  M200  D               volumetric_enabled (bool)
 *  M200  T D             filament_size (float x6)
 *
 *  M???  S               IDLE_OOZING_enabled
 *
 * ALLIGATOR:
 *  M906  XYZ T0-4 E      Motor current (float x7)
 *
 */

EEPROM eeprom;

/**
 * Post-process after Retrieve or Reset
 */
void EEPROM::Postprocess() {
  // steps per s2 needs to be updated to agree with units per s2
  planner.reset_acceleration_rates();

  // Make sure delta kinematics are updated before refreshing the
  // planner position so the stepper counts will be set correctly.
  #if MECH(DELTA)
    deltaParams.Recalc_delta_constants();
  #endif

  // Refresh steps_to_mm with the reciprocal of axis_steps_per_mm
  // and init stepper.count[], planner.position[] with current_position
  planner.refresh_positioning();

  #if ENABLED(PIDTEMP)
    thermalManager.updatePID();
  #endif

  calculate_volumetric_multipliers();

  // Software endstops depend on home_offset
  LOOP_XYZ(i) update_software_endstops((AxisEnum)i);
}

#if ENABLED(EEPROM_SETTINGS)

  uint16_t EEPROM::eeprom_checksum;
  const char EEPROM::version[6] = EEPROM_VERSION;

  bool  eeprom_write_error,
        eeprom_read_error;

  void EEPROM::writeData(int &pos, const uint8_t* value, uint16_t size) {
    if (eeprom_write_error) return;

    while(size--) {
      uint8_t * const p = (uint8_t * const)pos;
      const uint8_t v = *value;
      // EEPROM has only ~100,000 write cycles,
      // so only write bytes that have changed!
      if (v != eeprom_read_byte(p)) {
        eeprom_write_byte(p, v);
        if (eeprom_read_byte(p) != v) {
          SERIAL_LM(ECHO, MSG_ERR_EEPROM_WRITE);
          eeprom_write_error = true;
          return;
        }
      }
      eeprom_checksum += v;
      pos++;
      value++;
    };
  }

  void EEPROM::readData(int &pos, uint8_t* value, uint16_t size) {
    do {
      uint8_t c = eeprom_read_byte((unsigned char*)pos);
      if (!eeprom_read_error) *value = c;
      eeprom_checksum += c;
      pos++;
      value++;
    } while (--size);
  }

  #define EEPROM_START() int eeprom_index = EEPROM_OFFSET
  #define EEPROM_SKIP(VAR) eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR) writeData(eeprom_index, (uint8_t*)&VAR, sizeof(VAR))
  #define EEPROM_READ(VAR)  readData(eeprom_index, (uint8_t*)&VAR, sizeof(VAR))
  #define EEPROM_ASSERT(TST,ERR) if () do{ SERIAL_LT(ER, ERR); eeprom_read_error |= true; }while(0)

  /**
   * M500 - Store Configuration
   */
  void EEPROM::StoreSettings() {
    float dummy = 0.0f;
    char ver[6] = "00000";

    EEPROM_START();

    eeprom_write_error = false;

    EEPROM_WRITE(ver);     // invalidate data first
    EEPROM_SKIP(eeprom_checksum); // Skip the checksum slot

    eeprom_checksum = 0; // clear before first "real data"

    EEPROM_WRITE(planner.axis_steps_per_mm);
    EEPROM_WRITE(planner.max_feedrate_mm_s);
    EEPROM_WRITE(planner.max_acceleration_mm_per_s2);
    EEPROM_WRITE(planner.acceleration);
    EEPROM_WRITE(planner.retract_acceleration);
    EEPROM_WRITE(planner.travel_acceleration);
    EEPROM_WRITE(planner.min_feedrate_mm_s);
    EEPROM_WRITE(planner.min_travel_feedrate_mm_s);
    EEPROM_WRITE(planner.min_segment_time);
    EEPROM_WRITE(planner.max_jerk);
    EEPROM_WRITE(home_offset);
    EEPROM_WRITE(hotend_offset);

    //
    // Mesh Bed Leveling
    //
    #if ENABLED(MESH_BED_LEVELING)
      // Compile time test that sizeof(mbl.z_values) is as expected
      typedef char c_assert[(sizeof(mbl.z_values) == (MESH_NUM_X_POINTS) * (MESH_NUM_Y_POINTS) * sizeof(dummy)) ? 1 : -1];
      const bool leveling_is_on = TEST(mbl.status, MBL_STATUS_HAS_MESH_BIT);
      const uint8_t mesh_num_x = MESH_NUM_X_POINTS, mesh_num_y = MESH_NUM_Y_POINTS;
      EEPROM_WRITE(leveling_is_on);
      EEPROM_WRITE(mbl.z_offset);
      EEPROM_WRITE(mesh_num_x);
      EEPROM_WRITE(mesh_num_y);
      EEPROM_WRITE(mbl.z_values);
    #endif // MESH_BED_LEVELING

    //
    // Planar Bed Leveling matrix
    //
    #if ABL_PLANAR
      EEPROM_WRITE(planner.bed_level_matrix);
    #endif

    //
    // Bilinear Auto Bed Leveling
    //
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      // Compile time test that sizeof(mbl.z_values) is as expected
      typedef char c_assert[(sizeof(bilinear_level_grid) == (ABL_GRID_POINTS_X) * (ABL_GRID_POINTS_Y) * sizeof(dummy)) ? 1 : -1];
      uint8_t grid_max_x = ABL_GRID_POINTS_X, grid_max_y = ABL_GRID_POINTS_Y;
      EEPROM_WRITE(grid_max_x);             // 1 byte
      EEPROM_WRITE(grid_max_y);             // 1 byte
      EEPROM_WRITE(bilinear_grid_spacing);  // 2 ints
      EEPROM_WRITE(bilinear_start);         // 2 ints
      EEPROM_WRITE(bilinear_level_grid);    // 9-256 floats
    #endif // AUTO_BED_LEVELING_BILINEAR

    #if HASNT(BED_PROBE)
      float zprobe_zoffset = 0;
    #endif
    EEPROM_WRITE(zprobe_zoffset);

    #if HEATER_USES_AD595
      EEPROM_WRITE(ad595_offset);
      EEPROM_WRITE(ad595_gain);
    #endif

    #if MECH(DELTA)
      EEPROM_WRITE(deltaParams.endstop_adj);
      EEPROM_WRITE(deltaParams.radius);
      EEPROM_WRITE(deltaParams.diagonal_rod);
      EEPROM_WRITE(deltaParams.segments_per_second);
      EEPROM_WRITE(deltaParams.base_max_pos);
      EEPROM_WRITE(deltaParams.tower_adj);
      EEPROM_WRITE(deltaParams.diagonal_rod_adj);
    #elif ENABLED(Z_TWO_ENDSTOPS)
      EEPROM_WRITE(z2_endstop_adj);
    #endif

    #if DISABLED(ULTIPANEL)
      const int lcd_preheat_hotend_temp[3] = { PREHEAT_1_TEMP_HOTEND, PREHEAT_2_TEMP_HOTEND, PREHEAT_3_TEMP_HOTEND },
                lcd_preheat_bed_temp[3] = { PREHEAT_1_TEMP_BED, PREHEAT_2_TEMP_BED, PREHEAT_3_TEMP_BED },
                lcd_preheat_fan_speed[3] = { PREHEAT_1_FAN_SPEED, PREHEAT_2_FAN_SPEED, PREHEAT_3_FAN_SPEED };
    #endif

    EEPROM_WRITE(lcd_preheat_hotend_temp);
    EEPROM_WRITE(lcd_preheat_bed_temp);
    EEPROM_WRITE(lcd_preheat_fan_speed);

    #if ENABLED(PIDTEMP)
      for (uint8_t h = 0; h < HOTENDS; h++) {
        EEPROM_WRITE(PID_PARAM(Kp, h));
        EEPROM_WRITE(PID_PARAM(Ki, h));
        EEPROM_WRITE(PID_PARAM(Kd, h));
        EEPROM_WRITE(PID_PARAM(Kc, h));
      }
    #endif

    #if DISABLED(PID_ADD_EXTRUSION_RATE)
      int lpq_len = 20;
    #endif
    EEPROM_WRITE(lpq_len);
    
    #if ENABLED(PIDTEMPBED)
      EEPROM_WRITE(thermalManager.bedKp);
      EEPROM_WRITE(thermalManager.bedKi);
      EEPROM_WRITE(thermalManager.bedKd);
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      EEPROM_WRITE(thermalManager.chamberKp);
      EEPROM_WRITE(thermalManager.chamberKi);
      EEPROM_WRITE(thermalManager.chamberKd);
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      EEPROM_WRITE(thermalManager.coolerKp);
      EEPROM_WRITE(thermalManager.coolerKi);
      EEPROM_WRITE(thermalManager.coolerKd);
    #endif

    #if HASNT(LCD_CONTRAST)
      const int lcd_contrast = 32;
    #endif
    EEPROM_WRITE(lcd_contrast);

    #if ENABLED(FWRETRACT)
      EEPROM_WRITE(autoretract_enabled);
      EEPROM_WRITE(retract_length);
      #if EXTRUDERS > 1
        EEPROM_WRITE(retract_length_swap);
      #else
        EEPROM_WRITE(dummy);
      #endif
      EEPROM_WRITE(retract_feedrate);
      EEPROM_WRITE(retract_zlift);
      EEPROM_WRITE(retract_recover_length);
      #if EXTRUDERS > 1
        EEPROM_WRITE(retract_recover_length_swap);
      #else
        EEPROM_WRITE(dummy);
      #endif
      EEPROM_WRITE(retract_recover_feedrate);
    #endif // FWRETRACT

    EEPROM_WRITE(volumetric_enabled);

    // Save filament sizes
    for (uint8_t e = 0; e < EXTRUDERS; e++)
      EEPROM_WRITE(filament_size[e]);

    #if ENABLED(IDLE_OOZING_PREVENT)
      EEPROM_WRITE(IDLE_OOZING_enabled);
    #endif

    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      EEPROM_WRITE(motor_current);
    #endif

    uint16_t  final_checksum = eeprom_checksum,
              eeprom_size = eeprom_index;

    eeprom_index = EEPROM_OFFSET;
    EEPROM_WRITE(version);
    EEPROM_WRITE(final_checksum);

    // Report storage size
    SERIAL_SMV(ECHO, "Settings Stored (", eeprom_size - (EEPROM_OFFSET));
    SERIAL_EM(" bytes)");
  }

  /**
   * M501 - Retrieve Configuration
   */
  void EEPROM::RetrieveSettings() {

    EEPROM_START();
    eeprom_read_error = false; // If set EEPROM_READ won't write into RAM

    char stored_ver[6];
    EEPROM_READ(stored_ver);

    uint16_t stored_checksum;
    EEPROM_READ(stored_checksum);

    if (DEBUGGING(INFO)) {
      SERIAL_SMV(INFO, "Version: [", version);
      SERIAL_MV("] Stored version: [", stored_ver);
      SERIAL_EM("]");
    }

    if (strncmp(version, stored_ver, 5) != 0) {
      ResetDefault();
    }
    else {
      float dummy = 0;

      eeprom_checksum = 0; // clear before reading first "real data"

      // version number match
      EEPROM_READ(planner.axis_steps_per_mm);
      EEPROM_READ(planner.max_feedrate_mm_s);
      EEPROM_READ(planner.max_acceleration_mm_per_s2);

      EEPROM_READ(planner.acceleration);
      EEPROM_READ(planner.retract_acceleration);
      EEPROM_READ(planner.travel_acceleration);
      EEPROM_READ(planner.min_feedrate_mm_s);
      EEPROM_READ(planner.min_travel_feedrate_mm_s);
      EEPROM_READ(planner.min_segment_time);
      EEPROM_READ(planner.max_jerk);
      EEPROM_READ(home_offset);
      EEPROM_READ(hotend_offset);

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
        mbl.z_offset = dummy;
        if (mesh_num_x == MESH_NUM_X_POINTS && mesh_num_y == MESH_NUM_Y_POINTS) {
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
        EEPROM_READ(planner.bed_level_matrix);
      #endif

      //
      // Bilinear Auto Bed Leveling
      //
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        uint8_t grid_max_x, grid_max_y;
        EEPROM_READ(grid_max_x);              // 1 byte
        EEPROM_READ(grid_max_y);              // 1 byte
        if (grid_max_x == ABL_GRID_POINTS_X && grid_max_y == ABL_GRID_POINTS_Y) {
          set_bed_leveling_enabled(false);
          EEPROM_READ(bilinear_grid_spacing); // 2 ints
          EEPROM_READ(bilinear_start);        // 2 ints
          EEPROM_READ(bilinear_level_grid);   // 9 to 256 floats
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            bilinear_grid_spacing_virt[X_AXIS] = bilinear_grid_spacing[X_AXIS] / (BILINEAR_SUBDIVISIONS);
            bilinear_grid_spacing_virt[Y_AXIS] = bilinear_grid_spacing[Y_AXIS] / (BILINEAR_SUBDIVISIONS);
            bed_level_virt_interpolate();
          #endif
        }
        else { // EEPROM data is stale
          // Skip past disabled (or stale) Bilinear Grid data
          int bgs[2], bs[2];
          EEPROM_READ(bgs);
          EEPROM_READ(bs);
          for (uint16_t q = grid_max_x * grid_max_y; q--;) EEPROM_READ(dummy);
        }
      #endif // AUTO_BED_LEVELING_BILINEAR

      #if HASNT(BED_PROBE)
        float zprobe_zoffset = 0;
      #endif
      EEPROM_READ(zprobe_zoffset);

      #if HEATER_USES_AD595
        EEPROM_READ(ad595_offset);
        EEPROM_READ(ad595_gain);
        for (int8_t h = 0; h < HOTENDS; h++)
          if (ad595_gain[h] == 0) ad595_gain[h] == TEMP_SENSOR_AD595_GAIN;
      #endif

      #if MECH(DELTA)
        EEPROM_READ(deltaParams.endstop_adj);
        EEPROM_READ(deltaParams.radius);
        EEPROM_READ(deltaParams.diagonal_rod);
        EEPROM_READ(deltaParams.segments_per_second);
        EEPROM_READ(deltaParams.base_max_pos);
        EEPROM_READ(deltaParams.tower_adj);
        EEPROM_READ(deltaParams.diagonal_rod_adj);
      #elif ENABLED(Z_TWO_ENDSTOPS)
        EEPROM_READ(z2_endstop_adj);
      #endif

      #if DISABLED(ULTIPANEL)
        int lcd_preheat_hotend_temp[3], lcd_preheat_bed_temp[3], lcd_preheat_fan_speed[3];
      #endif

      EEPROM_READ(lcd_preheat_hotend_temp);
      EEPROM_READ(lcd_preheat_bed_temp);
      EEPROM_READ(lcd_preheat_fan_speed);

      #if ENABLED(PIDTEMP)
        for (int8_t h = 0; h < HOTENDS; h++) {
          EEPROM_READ(PID_PARAM(Kp, h));
          EEPROM_READ(PID_PARAM(Ki, h));
          EEPROM_READ(PID_PARAM(Kd, h));
          EEPROM_READ(PID_PARAM(Kc, h));
        }
      #endif // PIDTEMP

      #if DISABLED(PID_ADD_EXTRUSION_RATE)
        int lpq_len;
      #endif
      EEPROM_READ(lpq_len);

      #if ENABLED(PIDTEMPBED)
        EEPROM_READ(thermalManager.bedKp);
        EEPROM_READ(thermalManager.bedKi);
        EEPROM_READ(thermalManager.bedKd);
      #endif

      #if ENABLED(PIDTEMPCHAMBER)
        EEPROM_READ(thermalManager.chamberKp);
        EEPROM_READ(thermalManager.chamberKi);
        EEPROM_READ(thermalManager.chamberKd);
      #endif

      #if ENABLED(PIDTEMPCOOLER)
        EEPROM_READ(thermalManager.coolerKp);
        EEPROM_READ(thermalManager.coolerKi);
        EEPROM_READ(thermalManager.coolerKd);
      #endif

      #if HASNT(LCD_CONTRAST)
        int lcd_contrast;
      #endif
      EEPROM_READ(lcd_contrast);

      #if ENABLED(FWRETRACT)
        EEPROM_READ(autoretract_enabled);
        EEPROM_READ(retract_length);
        #if EXTRUDERS > 1
          EEPROM_READ(retract_length_swap);
        #else
          EEPROM_READ(dummy);
        #endif
        EEPROM_READ(retract_feedrate);
        EEPROM_READ(retract_zlift);
        EEPROM_READ(retract_recover_length);
        #if EXTRUDERS > 1
          EEPROM_READ(retract_recover_length_swap);
        #else
          EEPROM_READ(dummy);
        #endif
        EEPROM_READ(retract_recover_feedrate);
      #endif // FWRETRACT

      EEPROM_READ(volumetric_enabled);

      for (int8_t e = 0; e < EXTRUDERS; e++)
        EEPROM_READ(filament_size[e]);

      #if ENABLED(IDLE_OOZING_PREVENT)
        EEPROM_READ(IDLE_OOZING_enabled);
      #endif

      #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
        EEPROM_READ(motor_current);
      #endif

      if (eeprom_checksum == stored_checksum) {
        if (eeprom_read_error)
          ResetDefault();
        else {
          Postprocess();
          SERIAL_V(version);
          SERIAL_MV(" stored settings retrieved (", eeprom_index - (EEPROM_OFFSET));
          SERIAL_EM(" bytes)");
        }
      }
      else {
        SERIAL_LM(ER, "EEPROM checksum mismatch");
        ResetDefault();
      }
    }

    #if ENABLED(EEPROM_CHITCHAT)
      PrintSettings();
    #endif
  }

#else // !EEPROM_SETTINGS

  void EEPROM::StoreSettings() { SERIAL_LM(ER, "EEPROM disabled"); }

#endif // EEPROM_SETTINGS

/**
 * M502 - Reset Configuration
 */
void EEPROM::ResetDefault() {
  const float     tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
  const float     tmp2[] = DEFAULT_MAX_FEEDRATE;
  const uint32_t  tmp3[] = DEFAULT_MAX_ACCELERATION;
  const uint32_t  tmp4[] = DEFAULT_RETRACT_ACCELERATION;
  const float     tmp5[] = DEFAULT_EJERK;
  const float     tmp6[] = DEFAULT_Kp;
  const float     tmp7[] = DEFAULT_Ki;
  const float     tmp8[] = DEFAULT_Kd;
  const float     tmp9[] = DEFAULT_Kc;

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
    float tmp11[] = MOTOR_CURRENT;
    for (int8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++)
      motor_current[i] = tmp11[i];
  #endif

  LOOP_XYZE_N(i) {
    planner.axis_steps_per_mm[i]          = tmp1[i < COUNT(tmp1) ? i : COUNT(tmp1) - 1];
    planner.max_feedrate_mm_s[i]          = tmp2[i < COUNT(tmp2) ? i : COUNT(tmp2) - 1];
    planner.max_acceleration_mm_per_s2[i] = tmp3[i < COUNT(tmp3) ? i : COUNT(tmp3) - 1];
  }

  for (int8_t i = 0; i < EXTRUDERS; i++) {
    planner.retract_acceleration[i]       = tmp4[i < COUNT(tmp4) ? i : COUNT(tmp4) - 1];
    planner.max_jerk[E_AXIS + i]          = tmp5[i < COUNT(tmp5) ? i : COUNT(tmp5) - 1];
  }

  static_assert(
    tmp10[X_AXIS][0] == 0 && tmp10[Y_AXIS][0] == 0 && tmp10[Z_AXIS][0] == 0,
    "Offsets for the first hotend must be 0.0."
  );
  LOOP_XYZ(i) {
    HOTEND_LOOP() {
      hotend_offset[i][h] = tmp10[i][h];
    }
  }

  planner.acceleration = DEFAULT_ACCELERATION;
  planner.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  planner.min_feedrate_mm_s = DEFAULT_MINIMUMFEEDRATE;
  planner.min_segment_time = DEFAULT_MINSEGMENTTIME;
  planner.min_travel_feedrate_mm_s = DEFAULT_MINTRAVELFEEDRATE;
  planner.max_jerk[X_AXIS] = DEFAULT_XJERK;
  planner.max_jerk[Y_AXIS] = DEFAULT_YJERK;
  planner.max_jerk[Z_AXIS] = DEFAULT_ZJERK;
  home_offset[X_AXIS] = home_offset[Y_AXIS] = home_offset[Z_AXIS] = 0;

  #if PLANNER_LEVELING
    reset_bed_level();
  #endif

  #if HAS(BED_PROBE)
    zprobe_zoffset = Z_PROBE_OFFSET_FROM_NOZZLE;
  #endif

  #if MECH(DELTA)
    deltaParams.radius = DEFAULT_DELTA_RADIUS;
    deltaParams.diagonal_rod = DELTA_DIAGONAL_ROD;
    deltaParams.segments_per_second =  DELTA_SEGMENTS_PER_SECOND;
    deltaParams.base_max_pos[A_AXIS] = X_MAX_POS;
    deltaParams.base_max_pos[B_AXIS] = Y_MAX_POS;
    deltaParams.base_max_pos[C_AXIS] = Z_MAX_POS;
    deltaParams.endstop_adj[A_AXIS] = TOWER_A_ENDSTOP_ADJ;
    deltaParams.endstop_adj[B_AXIS] = TOWER_B_ENDSTOP_ADJ;
    deltaParams.endstop_adj[C_AXIS] = TOWER_C_ENDSTOP_ADJ;
    deltaParams.tower_adj[0] = TOWER_A_RADIUS_ADJ;
    deltaParams.tower_adj[1] = TOWER_B_RADIUS_ADJ;
    deltaParams.tower_adj[2] = TOWER_C_RADIUS_ADJ;
    deltaParams.tower_adj[3] = TOWER_A_POSITION_ADJ;
    deltaParams.tower_adj[4] = TOWER_B_POSITION_ADJ;
    deltaParams.tower_adj[5] = TOWER_C_POSITION_ADJ;
    deltaParams.diagonal_rod_adj[A_AXIS] = TOWER_A_DIAGROD_ADJ;
    deltaParams.diagonal_rod_adj[B_AXIS] = TOWER_B_DIAGROD_ADJ;
    deltaParams.diagonal_rod_adj[C_AXIS] = TOWER_C_DIAGROD_ADJ;
  #endif

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

  #if HAS(LCD_CONTRAST)
    lcd_contrast = DEFAULT_LCD_CONTRAST;
  #endif

  #if ENABLED(PIDTEMP)
    #if HOTENDS > 1
      HOTEND_LOOP()
    #else
      int h = 0; UNUSED(h); // only need to write once
    #endif
    {
      PID_PARAM(Kp, h) = tmp6[h];
      PID_PARAM(Ki, h) = scalePID_i(tmp7[h]);
      PID_PARAM(Kd, h) = scalePID_d(tmp8[h]);
      PID_PARAM(Kc, h) = tmp9[h];
    }
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      lpq_len = 20; // default last-position-queue size
    #endif
  #endif // PIDTEMP

  #if ENABLED(PIDTEMPBED)
    thermalManager.bedKp = DEFAULT_bedKp;
    thermalManager.bedKi = scalePID_i(DEFAULT_bedKi);
    thermalManager.bedKd = scalePID_d(DEFAULT_bedKd);
  #endif

  #if ENABLED(PIDTEMPCHAMBER)
    thermalManager.chamberKp = DEFAULT_chamberKp;
    thermalManager.chamberKi = scalePID_i(DEFAULT_chamberKi);
    thermalManager.chamberKd = scalePID_d(DEFAULT_chamberKd);
  #endif

  #if ENABLED(PIDTEMPCOOLER)
    thermalManager.coolerKp = DEFAULT_coolerKp;
    thermalManager.coolerKi = scalePID_i(DEFAULT_coolerKi);
    thermalManager.coolerKd = scalePID_d(DEFAULT_coolerKd);
  #endif

  #if ENABLED(FWRETRACT)
    autoretract_enabled = false;
    retract_length = RETRACT_LENGTH;
    #if EXTRUDERS > 1
      retract_length_swap = RETRACT_LENGTH_SWAP;
    #endif
    retract_feedrate = RETRACT_FEEDRATE;
    retract_zlift = RETRACT_ZLIFT;
    retract_recover_length = RETRACT_RECOVER_LENGTH;
    #if EXTRUDERS > 1
      retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
    #endif
    retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;
  #endif

  #if ENABLED(VOLUMETRIC_DEFAULT_ON)
    volumetric_enabled = true;
  #else
    volumetric_enabled = false;
  #endif

  for (uint8_t q = 0; q < COUNT(filament_size); q++)
    filament_size[q] = DEFAULT_NOMINAL_FILAMENT_DIA;

  endstops.enable_globally(
    #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
      (false)
    #else
      (true)
    #endif
  );

  #if ENABLED(IDLE_OOZING_PREVENT)
    IDLE_OOZING_enabled = true;
  #endif

  Postprocess();

  SERIAL_EM("Hardcoded Default Settings Loaded");
}

#if DISABLED(DISABLE_M503)

  #define CONFIG_MSG_START(str) do{ if (!forReplay) SERIAL_S(CFG); SERIAL_EM(str); }while(0)

  /**
   * M503 - Print Configuration
   */
  void EEPROM::PrintSettings(bool forReplay) {
    // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown

    CONFIG_MSG_START("Steps per unit:");
    SERIAL_SMV(CFG, "  M92 X", planner.axis_steps_per_mm[X_AXIS], 3);
    SERIAL_MV(" Y", planner.axis_steps_per_mm[Y_AXIS], 3);
    SERIAL_MV(" Z", planner.axis_steps_per_mm[Z_AXIS], 3);
    #if EXTRUDERS == 1
      SERIAL_MV(" E", planner.axis_steps_per_mm[E_AXIS], 3);
    #endif
    SERIAL_E;
    #if EXTRUDERS > 1
      for (uint8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M92 T", i);
        SERIAL_EMV(" E", planner.axis_steps_per_mm[E_AXIS + i], 3);
      }
    #endif // EXTRUDERS > 1

    CONFIG_MSG_START("Maximum feedrates (mm/s):");
    SERIAL_SMV(CFG, "  M203 X", planner.max_feedrate_mm_s[X_AXIS], 3);
    SERIAL_MV(" Y", planner.max_feedrate_mm_s[Y_AXIS], 3);
    SERIAL_MV(" Z", planner.max_feedrate_mm_s[Z_AXIS], 3);
    #if EXTRUDERS == 1
      SERIAL_MV(" E", planner.max_feedrate_mm_s[E_AXIS], 3);
    #endif
    SERIAL_E;
    #if EXTRUDERS > 1
      for (uint8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M203 T", i);
        SERIAL_EMV(" E", planner.max_feedrate_mm_s[E_AXIS + i], 3);
      }
    #endif // EXTRUDERS > 1

    CONFIG_MSG_START("Maximum Acceleration (mm/s2):");
    SERIAL_SMV(CFG, "  M201 X", planner.max_acceleration_mm_per_s2[X_AXIS]);
    SERIAL_MV(" Y", planner.max_acceleration_mm_per_s2[Y_AXIS]);
    SERIAL_MV(" Z", planner.max_acceleration_mm_per_s2[Z_AXIS]);
    #if EXTRUDERS == 1
      SERIAL_MV(" E", planner.max_acceleration_mm_per_s2[E_AXIS]);
    #endif
    SERIAL_E;
    #if EXTRUDERS > 1
      for (uint8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M201 T", i);
        SERIAL_EMV(" E", planner.max_acceleration_mm_per_s2[E_AXIS + i]);
      }
    #endif // EXTRUDERS > 1

    CONFIG_MSG_START("Accelerations: P=printing, V=travel and T* R=retract");
    SERIAL_SMV(CFG,"  M204 P", planner.acceleration, 3);
    SERIAL_MV(" V", planner.travel_acceleration, 3);
    #if EXTRUDERS == 1
      SERIAL_MV(" R", planner.retract_acceleration[0], 3);
    #endif
    SERIAL_E;
    #if EXTRUDERS > 1
      for (uint8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M204 T", i);
        SERIAL_EMV(" R", planner.retract_acceleration[i], 3);
      }
    #endif

    CONFIG_MSG_START("Advanced variables: S=Min feedrate (mm/s), V=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum X jerk (mm/s), Y=maximum Y jerk (mm/s), Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    SERIAL_SMV(CFG, "  M205 S", planner.min_feedrate_mm_s, 3);
    SERIAL_MV(" V", planner.min_travel_feedrate_mm_s, 3);
    SERIAL_MV(" B", planner.min_segment_time);
    SERIAL_MV(" X", planner.max_jerk[X_AXIS], 3);
    SERIAL_MV(" Y", planner.max_jerk[Y_AXIS], 3);
    SERIAL_MV(" Z", planner.max_jerk[Z_AXIS], 3);
    #if EXTRUDERS == 1
      SERIAL_MV(" E", planner.max_jerk[E_AXIS], 3);
    #endif
    SERIAL_E;
    #if (EXTRUDERS > 1)
      for(uint8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M205 T", i);
        SERIAL_EMV(" E" , planner.max_jerk[E_AXIS + i], 3);
      }
    #endif

    CONFIG_MSG_START("Home offset (mm):");
    SERIAL_SMV(CFG, "  M206 X", home_offset[X_AXIS], 3);
    SERIAL_MV(" Y", home_offset[Y_AXIS], 3);
    SERIAL_EMV(" Z", home_offset[Z_AXIS], 3);

    #if HOTENDS > 1
      CONFIG_MSG_START("Hotend offset (mm):");
      for (uint8_t h = 1; h < HOTENDS; h++) {
        SERIAL_SMV(CFG, "  M218 T", h);
        SERIAL_MV(" X", hotend_offset[X_AXIS][h], 3);
        SERIAL_MV(" Y", hotend_offset[Y_AXIS][h], 3);
        SERIAL_EMV(" Z", hotend_offset[Z_AXIS][h], 3);
      }
    #endif

    #if HAS(LCD_CONTRAST)
      CONFIG_MSG_START("LCD Contrast:");
      SERIAL_LMV(CFG, "  M250 C", lcd_contrast);
    #endif

    #if ENABLED(MESH_BED_LEVELING)
      CONFIG_MSG_START("Mesh Bed Leveling:");
      SERIAL_SMV(CFG, "  M420 S", mbl.has_mesh() ? 1 : 0);
      SERIAL_MV(" X", MESH_NUM_X_POINTS);
      SERIAL_MV(" Y", MESH_NUM_Y_POINTS);
      SERIAL_E;

      for (uint8_t py = 1; py <= MESH_NUM_Y_POINTS; py++) {
        for (uint8_t px = 1; px <= MESH_NUM_X_POINTS; px++) {
          SERIAL_SMV(CFG, "  G29 S3 X", px);
          SERIAL_MV(" Y", py);
          SERIAL_EMV(" Z", mbl.z_values[py-1][px-1], 5);
        }
      }
    #endif

    #if HEATER_USES_AD595
      CONFIG_MSG_START("AD595 Offset and Gain:");
      for (uint8_t h = 0; h < HOTENDS; h++) {
        SERIAL_SMV(CFG, "  M595 T", h);
        SERIAL_MV(" O", ad595_offset[h]);
        SERIAL_EMV(", S", ad595_gain[h]);
      }
    #endif // HEATER_USES_AD595

    #if MECH(DELTA)

      CONFIG_MSG_START("Endstop adjustment (mm):");
      SERIAL_SMV(CFG, "  M666 X", deltaParams.endstop_adj[A_AXIS]);
      SERIAL_MV(" Y", deltaParams.endstop_adj[B_AXIS]);
      SERIAL_EMV(" Z", deltaParams.endstop_adj[C_AXIS]);

      CONFIG_MSG_START("Geometry adjustment: ABC=TOWER_RADIUS_ADJ, IJK=TOWER_POSITION_ADJ, UVW=TOWER_DIAGROD_ADJ, R=Delta Radius, D=Diagonal Rod, S=Segments per second, H=Z Height");
      SERIAL_SMV(CFG, "  M666 A", deltaParams.tower_adj[0], 3);
      SERIAL_MV(" B", deltaParams.tower_adj[1], 3);
      SERIAL_MV(" C", deltaParams.tower_adj[2], 3);
      SERIAL_MV(" I", deltaParams.tower_adj[3], 3);
      SERIAL_MV(" J", deltaParams.tower_adj[4], 3);
      SERIAL_MV(" K", deltaParams.tower_adj[5], 3);
      SERIAL_MV(" U", deltaParams.diagonal_rod_adj[0], 3);
      SERIAL_MV(" V", deltaParams.diagonal_rod_adj[1], 3);
      SERIAL_MV(" W", deltaParams.diagonal_rod_adj[2], 3);
      SERIAL_MV(" R", deltaParams.radius);
      SERIAL_MV(" D", deltaParams.diagonal_rod);
      SERIAL_MV(" S", deltaParams.segments_per_second, 3);
      SERIAL_EMV(" H", deltaParams.base_max_pos[C_AXIS], 3);

    #elif ENABLED(Z_TWO_ENDSTOPS)

      CONFIG_MSG_START("Z2 Endstop adjustement (mm):");
      SERIAL_LMV(CFG, "  M666 Z", z2_endstop_adj );

    #endif // DELTA

    /**
     * Auto Bed Leveling
     */
    #if HAS(BED_PROBE)
      CONFIG_MSG_START("Z Probe offset (mm):");
      SERIAL_LMV(CFG, "  M666 P", zprobe_zoffset);
    #endif

    #if ENABLED(ULTIPANEL)
      CONFIG_MSG_START("Material heatup parameters:");
      for (uint8_t i = 0; i < COUNT(lcd_preheat_hotend_temp); i++) {
        SERIAL_SMV(CFG, "  M145 S", (int)i);
        SERIAL_MV(" H", lcd_preheat_hotend_temp[i]);
        SERIAL_MV(" B", lcd_preheat_bed_temp[i]);
        SERIAL_MV(" F", lcd_preheat_fan_speed[i]);
        SERIAL_E;
      }
    #endif // ULTIPANEL

    #if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED) || ENABLED(PIDTEMPCHAMBER) || ENABLED(PIDTEMPCOOLER)
      CONFIG_MSG_START("PID settings:");
      #if ENABLED(PIDTEMP)
        for (uint8_t h = 0; h < HOTENDS; h++) {
          SERIAL_SMV(CFG, "  M301 H", h);
          SERIAL_MV(" P", PID_PARAM(Kp, h));
          SERIAL_MV(" I", unscalePID_i(PID_PARAM(Ki, h)));
          SERIAL_MV(" D", unscalePID_d(PID_PARAM(Kd, h)));
          #if ENABLED(PID_ADD_EXTRUSION_RATE)
            SERIAL_MV(" C", PID_PARAM(Kc, h));
          #endif
          SERIAL_E;
        }
        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          SERIAL_SMV(CFG, "  M301 L", lpq_len);
        #endif
      #endif
      #if ENABLED(PIDTEMPBED)
        SERIAL_SMV(CFG, "  M304 P", thermalManager.bedKp);
        SERIAL_MV(" I", unscalePID_i(thermalManager.bedKi));
        SERIAL_EMV(" D", unscalePID_d(thermalManager.bedKd));
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        SERIAL_SMV(CFG, "  M305 P", thermalManager.chamberKp);
        SERIAL_MV(" I", unscalePID_i(thermalManager.chamberKi));
        SERIAL_EMV(" D", unscalePID_d(thermalManager.chamberKd));
      #endif
      #if ENABLED(PIDTEMPCOOLER)
        SERIAL_SMV(CFG, "  M306 P", thermalManager.coolerKp);
        SERIAL_MV(" I", unscalePID_i(thermalManager.coolerKi));
        SERIAL_EMV(" D", unscalePID_d(thermalManager.coolerKd));
      #endif
    #endif

    #if ENABLED(FWRETRACT)
      CONFIG_MSG_START("Retract: S=Length (mm) F:Speed (mm/m) Z: ZLift (mm)");
      SERIAL_SMV(CFG, "  M207 S", retract_length);
      #if EXTRUDERS > 1
        SERIAL_MV(" W", retract_length_swap);
      #endif
      SERIAL_MV(" F", retract_feedrate * 60);
      SERIAL_EMV(" Z", retract_zlift);

      CONFIG_MSG_START("Recover: S=Extra length (mm) F:Speed (mm/m)");
      SERIAL_SMV(CFG, "  M208 S", retract_recover_length);
      #if EXTRUDERS > 1
        SERIAL_MV(" W", retract_recover_length_swap);
      #endif
      SERIAL_MV(" F", retract_recover_feedrate * 60);

      CONFIG_MSG_START("Auto-Retract: S=0 to disable, 1 to interpret extrude-only moves as retracts or recoveries");
      SERIAL_LMV(CFG, "  M209 S", autoretract_enabled ? 1 : 0);
    #endif // FWRETRACT

    /**
     * Volumetric extrusion M200
     */
    if (!forReplay) {
      SERIAL_SM(CFG, "Filament settings:");
      if (volumetric_enabled)
        SERIAL_E;
      else
        SERIAL_EM(" Disabled");
    }
    #if EXTRUDERS > 0
      SERIAL_LMV(CFG, "  M200 D", filament_size[0], 3);
    #endif
    #if EXTRUDERS > 1
      for(uint8_t i = 0; i < EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M200 T", (int)i);
        SERIAL_EMV(" D", filament_size[i], 3);
      }
    #endif

    /**
     * Alligator current drivers M906
     */
    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      CONFIG_MSG_START("Motor current:");
      SERIAL_SMV(CFG, "  M906 X", motor_current[X_AXIS], 2);
      SERIAL_MV(" Y", motor_current[Y_AXIS], 2);
      SERIAL_MV(" Z", motor_current[Z_AXIS], 2);
      #if EXTRUDERS == 1
        SERIAL_MV(" E", motor_current[E_AXIS], 2);
      #endif
      SERIAL_E;
      #if DRIVER_EXTRUDERS > 1
        for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
          SERIAL_SMV(CFG, "  M906 T", i);
          SERIAL_EMV(" E", motor_current[E_AXIS + i], 2);
        }
      #endif // DRIVER_EXTRUDERS > 1
    #endif // ALLIGATOR

    #if ENABLED(SDSUPPORT)
      card.PrintSettings();
    #endif

  }

#endif // !DISABLE_M503

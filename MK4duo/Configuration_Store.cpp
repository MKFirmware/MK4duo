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
 * configuration_store.cpp
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

#include "base.h"

#define EEPROM_VERSION "MKV29"
#define EEPROM_OFFSET 100

/**
 * MKV429 EEPROM Layout:
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
 *  M205  X               planner.max_x_jerk (float)
 *  M205  Y               planner.max_y_jerk (float)
 *  M205  Z               planner.max_z_jerk (float)
 *  M205  E   E0 ...      planner.max_e_jerk (float x6)
 *  M206  XYZ             home_offset (float x3)
 *  M218  T   XY          hotend_offset (float x6)
 *
 * Mesh bed leveling:
 *  M420  S               status (uint8)
 *                        z_offset (float)
 *                        mesh_num_x (uint8 as set in firmware)
 *                        mesh_num_y (uint8 as set in firmware)
 *  G29   S3  XYZ         z_values[][] (float x9, by default)
 *
 * HOTENDS AD595:
 *  M595  H OS            Hotend AD595 Offset & Gain
 *
 * DELTA:
 *  M666  XYZ             endstop_adj (float x3)
 *  M666  R               delta_radius (float)
 *  M666  D               delta_diagonal_rod (float)
 *  M666  S               delta_segments_per_second (float)
 *  M666  H               Z soft_endstop_max (float)
 *  M666  ABCIJK          tower_adj (float x6)
 *  M666  UVW             diagrod_adj (float x3)
 *
 * Z_DUAL_ENDSTOPS:
 *  M666  Z               z_endstop_adj (float)
 *
 * Z PROBE:
 *  M666  P               zprobe_zoffset (float)
 *
 * ULTIPANEL:
 *  M145  S0  H           preheatHotendTemp1 (int)
 *  M145  S0  B           preheatBedTemp1 (int)
 *  M145  S0  F           preheatFanSpeed1 (int)
 *  M145  S1  H           preheatHotendTemp2 (int)
 *  M145  S1  B           preheatBedTemp2 (int)
 *  M145  S1  F           preheatFanSpeed2 (int)
 *  M145  S2  H           preheatHotendTemp3 (int)
 *  M145  S2  B           preheatBedTemp3 (int)
 *  M145  S2  F           preheatFanSpeed3 (int)
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
 *  M200  D               volumetric_enabled (bool)
 *
 *  M200  T D             filament_size (float x6)
 *
 *  M???  S               IDLE_OOZING_enabled
 *
 * ALLIGATOR:
 *  M906  XYZ T0-4 E      Motor current (float x7)
 *
 */

uint16_t eeprom_checksum;
const char version[6] = EEPROM_VERSION;

void _EEPROM_writeData(int& pos, uint8_t* value, uint8_t size) {
  uint8_t c;
  while(size--) {
    eeprom_write_byte((unsigned char*)pos, *value);
    c = eeprom_read_byte((unsigned char*)pos);
    if (c != *value) {
      SERIAL_LM(ER, "Error writing to EEPROM!");
    }
    eeprom_checksum += c;
    pos++;
    value++;
  };
}

void _EEPROM_readData(int& pos, uint8_t* value, uint8_t size) {
  do {
    byte c = eeprom_read_byte((unsigned char*)pos);
    *value = c;
    eeprom_checksum += c;
    pos++;
    value++;
  } while (--size);
}

/**
 * Post-process after Retrieve or Reset
 */
void Config_Postprocess() {
  // steps per s2 needs to be updated to agree with units per s2
  planner.reset_acceleration_rates();

  // Make sure delta kinematics are updated before refreshing the
  // planner position so the stepper counts will be set correctly.
  #if MECH(DELTA)
    set_delta_constants();
  #endif

  // Refresh steps_to_mm with the reciprocal of axis_steps_per_mm
  // and init stepper.count[], planner.position[] with current_position
  planner.refresh_positioning();

  #if ENABLED(PIDTEMP)
    updatePID();
  #endif

  calculate_volumetric_multipliers();

  // Software endstops depend on home_offset
  LOOP_XYZ(i) update_software_endstops((AxisEnum)i);
}

#if ENABLED(EEPROM_SETTINGS)

#define EEPROM_START() int eeprom_index = EEPROM_OFFSET
#define EEPROM_SKIP(VAR) eeprom_index += sizeof(VAR)
#define EEPROM_WRITE(VAR) _EEPROM_writeData(eeprom_index, (uint8_t*)&VAR, sizeof(VAR))
#define EEPROM_READ(VAR) _EEPROM_readData(eeprom_index, (uint8_t*)&VAR, sizeof(VAR))

/**
 * M500 - Store Configuration
 */
void Config_StoreSettings() {
  float dummy = 0.0f;
  char ver[6] = "00000";

  EEPROM_START();

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

  #if ENABLED(MESH_BED_LEVELING)
    // Compile time test that sizeof(mbl.z_values) is as expected
    typedef char c_assert[(sizeof(mbl.z_values) == (MESH_NUM_X_POINTS) * (MESH_NUM_Y_POINTS) * sizeof(dummy)) ? 1 : -1];
    uint8_t mesh_num_x  = MESH_NUM_X_POINTS,
            mesh_num_y  = MESH_NUM_Y_POINTS,
            dummy_uint8 = mbl.status & _BV(MBL_STATUS_HAS_MESH_BIT);
    EEPROM_WRITE(dummy_uint8);
    EEPROM_WRITE(mbl.z_offset);
    EEPROM_WRITE(mesh_num_x);
    EEPROM_WRITE(mesh_num_y);
    EEPROM_WRITE(mbl.z_values);
  #endif

  #if HEATER_USES_AD595
    EEPROM_WRITE(ad595_offset);
    EEPROM_WRITE(ad595_gain);
  #endif

  #if MECH(DELTA)
    EEPROM_WRITE(endstop_adj);
    EEPROM_WRITE(delta_radius);
    EEPROM_WRITE(delta_diagonal_rod);
    EEPROM_WRITE(delta_segments_per_second);
    EEPROM_WRITE(soft_endstop_max);
    EEPROM_WRITE(tower_adj);
    EEPROM_WRITE(diagrod_adj);
  #elif ENABLED(Z_DUAL_ENDSTOPS)
    EEPROM_WRITE(z_endstop_adj);
  #endif

  #if HASNT(BED_PROBE)
    float zprobe_zoffset = 0;
  #endif
  EEPROM_WRITE(zprobe_zoffset);

  #if DISABLED(ULTIPANEL)
    int preheatHotendTemp1 = PREHEAT_1_TEMP_HOTEND, preheatBedTemp1 = PREHEAT_1_TEMP_BED, preheatFanSpeed1 = PREHEAT_1_FAN_SPEED,
        preheatHotendTemp2 = PREHEAT_2_TEMP_HOTEND, preheatBedTemp2 = PREHEAT_2_TEMP_BED, preheatFanSpeed2 = PREHEAT_2_FAN_SPEED,
        preheatHotendTemp3 = PREHEAT_3_TEMP_HOTEND, preheatBedTemp3 = PREHEAT_3_TEMP_BED, preheatFanSpeed3 = PREHEAT_3_FAN_SPEED;
  #endif

  EEPROM_WRITE(preheatHotendTemp1);
  EEPROM_WRITE(preheatBedTemp1);
  EEPROM_WRITE(preheatFanSpeed1);
  EEPROM_WRITE(preheatHotendTemp2);
  EEPROM_WRITE(preheatBedTemp2);
  EEPROM_WRITE(preheatFanSpeed2);
  EEPROM_WRITE(preheatHotendTemp3);
  EEPROM_WRITE(preheatBedTemp3);
  EEPROM_WRITE(preheatFanSpeed3);

  #if ENABLED(PIDTEMP)
    for (int h = 0; h < HOTENDS; h++) {
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
    EEPROM_WRITE(bedKp);
    EEPROM_WRITE(bedKi);
    EEPROM_WRITE(bedKd);
  #endif

  #if ENABLED(PIDTEMPCHAMBER)
    EEPROM_WRITE(chamberKp);
    EEPROM_WRITE(chamberKi);
    EEPROM_WRITE(chamberKd);
  #endif

  #if ENABLED(PIDTEMPCOOLER)
    EEPROM_WRITE(coolerKp);
    EEPROM_WRITE(coolerKi);
    EEPROM_WRITE(coolerKd);
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
      dummy = 0.0f;
      EEPROM_WRITE(dummy);
    #endif
    EEPROM_WRITE(retract_feedrate);
    EEPROM_WRITE(retract_zlift);
    EEPROM_WRITE(retract_recover_length);
    #if EXTRUDERS > 1
      EEPROM_WRITE(retract_recover_length_swap);
    #else
      dummy = 0.0f;
      EEPROM_WRITE(dummy);
    #endif
    EEPROM_WRITE(retract_recover_feedrate);
  #endif // FWRETRACT

  EEPROM_WRITE(volumetric_enabled);

  // Save filament sizes
  for (int e = 0; e < EXTRUDERS; e++)
    EEPROM_WRITE(filament_size[e]);

  #if ENABLED(IDLE_OOZING_PREVENT)
    EEPROM_WRITE(IDLE_OOZING_enabled);
  #endif

  #if MB(ALLIGATOR)
    EEPROM_WRITE(motor_current);
  #endif

  uint16_t  final_checksum = eeprom_checksum,
            eeprom_size = eeprom_index;

  eeprom_index = EEPROM_OFFSET;
  EEPROM_WRITE(version);
  EEPROM_WRITE(final_checksum);

  // Report storage size
  SERIAL_MV("Settings Stored (", eeprom_size);
  SERIAL_EM(" bytes)");
}

/**
 * M501 - Retrieve Configuration
 */
void Config_RetrieveSettings() {
  char stored_ver[6];
  uint16_t stored_checksum;

  EEPROM_START();
  EEPROM_READ(stored_ver);
  EEPROM_READ(stored_checksum);

  if (DEBUGGING(INFO)) {
    SERIAL_SMV(INFO, "Version: [", version);
    SERIAL_MV("] Stored version: [", stored_ver);
    SERIAL_EM("]");
  }

  if (strncmp(version, stored_ver, 5) != 0) {
    Config_ResetDefault();
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

    #if ENABLED(MESH_BED_LEVELING)
      uint8_t mesh_num_x = 0, mesh_num_y = 0;
      EEPROM_READ(mbl.status);
      EEPROM_READ(mbl.z_offset);
      EEPROM_READ(mesh_num_x);
      EEPROM_READ(mesh_num_y);
      EEPROM_READ(mbl.z_values);
    #endif

    #if HEATER_USES_AD595
      EEPROM_READ(ad595_offset);
      EEPROM_READ(ad595_gain);
      for (int8_t h = 0; h < HOTENDS; h++)
        if (ad595_gain[h] == 0) ad595_gain[h] == TEMP_SENSOR_AD595_GAIN;
    #endif

    #if MECH(DELTA)
      EEPROM_READ(endstop_adj);
      EEPROM_READ(delta_radius);
      EEPROM_READ(delta_diagonal_rod);
      EEPROM_READ(delta_segments_per_second);
      EEPROM_READ(soft_endstop_max);
      EEPROM_READ(tower_adj);
      EEPROM_READ(diagrod_adj);
    #endif //DELTA

    #if HASNT(BED_PROBE)
      float zprobe_zoffset = 0;
    #endif
    EEPROM_READ(zprobe_zoffset);

    #if DISABLED(ULTIPANEL)
      int preheatHotendTemp1, preheatBedTemp1, preheatFanSpeed1,
          preheatHotendTemp2, preheatBedTemp2, preheatFanSpeed2,
          preheatHotendTemp3, preheatBedTemp3, preheatFanSpeed3;
    #endif

    EEPROM_READ(preheatHotendTemp1);
    EEPROM_READ(preheatBedTemp1);
    EEPROM_READ(preheatFanSpeed1);
    EEPROM_READ(preheatHotendTemp2);
    EEPROM_READ(preheatBedTemp2);
    EEPROM_READ(preheatFanSpeed2);
    EEPROM_READ(preheatHotendTemp3);
    EEPROM_READ(preheatBedTemp3);
    EEPROM_READ(preheatFanSpeed3);

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
      EEPROM_READ(bedKp);
      EEPROM_READ(bedKi);
      EEPROM_READ(bedKd);
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      EEPROM_READ(chamberKp);
      EEPROM_READ(chamberKi);
      EEPROM_READ(chamberKd);
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      EEPROM_READ(coolerKp);
      EEPROM_READ(coolerKi);
      EEPROM_READ(coolerKd);
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

    #if MB(ALLIGATOR)
      EEPROM_READ(motor_current);
    #endif

    if (eeprom_checksum == stored_checksum) {
      Config_Postprocess();
      SERIAL_V(version);
      SERIAL_MV(" stored settings retrieved (", eeprom_index);
      SERIAL_EM(" bytes)");
    }
    else {
      SERIAL_LM(ER, "EEPROM checksum mismatch");
      Config_ResetDefault();
    }
  }

  #if ENABLED(EEPROM_CHITCHAT)
    Config_PrintSettings();
  #endif
}

#endif // EEPROM_SETTINGS

/**
 * M502 - Reset Configuration
 */
void Config_ResetDefault() {
  float tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
  float tmp2[] = DEFAULT_MAX_FEEDRATE;
  float tmp3[] = DEFAULT_MAX_ACCELERATION;
  float tmp4[] = DEFAULT_RETRACT_ACCELERATION;
  float tmp5[] = DEFAULT_EJERK;
  float tmp6[] = DEFAULT_Kp;
  float tmp7[] = DEFAULT_Ki;
  float tmp8[] = DEFAULT_Kd;
  float tmp9[] = DEFAULT_Kc;

  #if ENABLED(HOTEND_OFFSET_X) && ENABLED(HOTEND_OFFSET_Y) && ENABLED(HOTEND_OFFSET_Z)
    float tmp10[] = HOTEND_OFFSET_X;
    float tmp11[] = HOTEND_OFFSET_Y;
    float tmp12[] = HOTEND_OFFSET_Z;
  #endif

  #if MB(ALLIGATOR)
    float tmp13[] = MOTOR_CURRENT;
    for (int8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++)
      motor_current[i] = tmp13[i];
  #endif

  for (int8_t i = 0; i < XYZEn; i++) {
    planner.axis_steps_per_mm[i] = tmp1[i];
    planner.max_feedrate_mm_s[i] = tmp2[i];
    planner.max_acceleration_mm_per_s2[i] = tmp3[i];
  }

  for (int8_t i = 0; i < EXTRUDERS; i++) {
    planner.retract_acceleration[i] = tmp4[i];
    planner.max_jerk[E_AXIS + i] = tmp5[i];
  }

  for (int8_t i = 0; i < HOTENDS; i++) {
    #if ENABLED(HOTEND_OFFSET_X) && ENABLED(HOTEND_OFFSET_Y) && ENABLED(HOTEND_OFFSET_Z)
      hotend_offset[X_AXIS][i] = tmp10[i];
      hotend_offset[Y_AXIS][i] = tmp11[i];
      hotend_offset[Z_AXIS][i] = tmp12[i];
    #else
      hotend_offset[X_AXIS][i] = 0;
      hotend_offset[Y_AXIS][i] = 0;
      hotend_offset[Z_AXIS][i] = 0;
    #endif
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

  #if ENABLED(MESH_BED_LEVELING)
    mbl.reset();
  #endif

  #if HAS(BED_PROBE)
    zprobe_zoffset = Z_PROBE_OFFSET_FROM_NOZZLE;
  #endif

  #if MECH(DELTA)
    delta_radius = DEFAULT_DELTA_RADIUS;
    delta_diagonal_rod = DELTA_DIAGONAL_ROD;
    delta_segments_per_second =  DELTA_SEGMENTS_PER_SECOND;
    soft_endstop_max[0] = X_MAX_POS;
    soft_endstop_max[1] = Y_MAX_POS;
    soft_endstop_max[2] = Z_MAX_POS;
    endstop_adj[0] = TOWER_A_ENDSTOP_ADJ;
    endstop_adj[1] = TOWER_B_ENDSTOP_ADJ;
    endstop_adj[2] = TOWER_C_ENDSTOP_ADJ;
    tower_adj[0] = TOWER_A_POSITION_ADJ;
    tower_adj[1] = TOWER_B_POSITION_ADJ;
    tower_adj[2] = TOWER_C_POSITION_ADJ;
    tower_adj[3] = TOWER_A_RADIUS_ADJ;
    tower_adj[4] = TOWER_B_RADIUS_ADJ;
    tower_adj[5] = TOWER_C_RADIUS_ADJ;
    diagrod_adj[0] = TOWER_A_DIAGROD_ADJ;
    diagrod_adj[1] = TOWER_B_DIAGROD_ADJ;
    diagrod_adj[2] = TOWER_C_DIAGROD_ADJ;
  #endif

  #if ENABLED(ULTIPANEL)
    preheatHotendTemp1 = PREHEAT_1_TEMP_HOTEND;
    preheatBedTemp1 = PREHEAT_1_TEMP_BED;
    preheatFanSpeed1 = PREHEAT_1_FAN_SPEED;
    preheatHotendTemp2 = PREHEAT_2_TEMP_HOTEND;
    preheatBedTemp2 = PREHEAT_2_TEMP_BED;
    preheatFanSpeed2 = PREHEAT_2_FAN_SPEED;
    preheatHotendTemp3 = PREHEAT_3_TEMP_HOTEND;
    preheatBedTemp3 = PREHEAT_3_TEMP_BED;
    preheatFanSpeed3 = PREHEAT_3_FAN_SPEED;
  #endif

  #if HAS(LCD_CONTRAST)
    lcd_contrast = DEFAULT_LCD_CONTRAST;
  #endif

  #if ENABLED(PIDTEMP)
    for (int8_t h = 0; h < HOTENDS; h++) {
      Kp[h] = tmp6[h];
      Ki[h] = scalePID_i(tmp7[h]);
      Kd[h] = scalePID_d(tmp8[h]);
      Kc[h] = tmp9[h];
    }
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      lpq_len = 20; // default last-position-queue size
    #endif
  #endif // PIDTEMP

  #if ENABLED(PIDTEMPBED)
    bedKp = DEFAULT_bedKp;
    bedKi = scalePID_i(DEFAULT_bedKi);
    bedKd = scalePID_d(DEFAULT_bedKd);
  #endif

  #if ENABLED(PIDTEMPCHAMBER)
    chamberKp = DEFAULT_chamberKp;
    chamberKi = scalePID_i(DEFAULT_chamberKi);
    chamberKd = scalePID_d(DEFAULT_chamberKd);
  #endif

  #if ENABLED(PIDTEMPCOOLER)
    coolerKp = DEFAULT_coolerKp;
    coolerKi = scalePID_i(DEFAULT_coolerKi);
    coolerKd = scalePID_d(DEFAULT_coolerKd);
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

  volumetric_enabled = false;

  #if ENABLED(IDLE_OOZING_PREVENT)
    IDLE_OOZING_enabled = true;
  #endif

  Config_Postprocess();

  SERIAL_EM("Hardcoded Default Settings Loaded");
}

#if DISABLED(DISABLE_M503)

#define CONFIG_MSG_START(str) do{ if (!forReplay) SERIAL_S(CFG); SERIAL_EM(str); }while(0)

/**
 * M503 - Print Configuration
 */
void Config_PrintSettings(bool forReplay) {
  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown

  CONFIG_MSG_START("Steps per unit:");
  SERIAL_SMV(CFG, "  M92 X", planner.axis_steps_per_mm[X_AXIS]);
  SERIAL_MV(" Y", planner.axis_steps_per_mm[Y_AXIS]);
  SERIAL_MV(" Z", planner.axis_steps_per_mm[Z_AXIS]);
  SERIAL_EMV(" E", planner.axis_steps_per_mm[E_AXIS]);
  #if EXTRUDERS > 1
    for (int8_t i = 1; i < EXTRUDERS; i++) {
      SERIAL_SMV(CFG, "  M92 T", i);
      SERIAL_EMV(" E", planner.axis_steps_per_mm[E_AXIS + i]);
    }
  #endif //EXTRUDERS > 1

  CONFIG_MSG_START("Maximum feedrates (mm/s):");
  SERIAL_SMV(CFG, "  M203 X", planner.max_feedrate_mm_s[X_AXIS]);
  SERIAL_MV(" Y", planner.max_feedrate_mm_s[Y_AXIS] );
  SERIAL_MV(" Z", planner.max_feedrate_mm_s[Z_AXIS] );
  SERIAL_EMV(" E", planner.max_feedrate_mm_s[E_AXIS]);
  #if EXTRUDERS > 1
    for (int8_t i = 1; i < EXTRUDERS; i++) {
      SERIAL_SMV(CFG, "  M203 T", i);
      SERIAL_EMV(" E", planner.max_acceleration_mm_per_s2[E_AXIS + i]);
    }
  #endif //EXTRUDERS > 1

  CONFIG_MSG_START("Maximum Acceleration (mm/s2):");
  SERIAL_SMV(CFG, "  M201 X", planner.max_acceleration_mm_per_s2[X_AXIS] );
  SERIAL_MV(" Y", planner.max_acceleration_mm_per_s2[Y_AXIS] );
  SERIAL_MV(" Z", planner.max_acceleration_mm_per_s2[Z_AXIS] );
  SERIAL_EMV(" E", planner.max_acceleration_mm_per_s2[E_AXIS]);
  #if EXTRUDERS > 1
    for (int8_t i = 1; i < EXTRUDERS; i++) {
      SERIAL_SMV(CFG, "  M201 T", i);
      SERIAL_EMV(" E", planner.max_acceleration_mm_per_s2[E_AXIS + i]);
    }
  #endif //EXTRUDERS > 1
  
  CONFIG_MSG_START("Accelerations: P=printing, V=travel and T* R=retract");
  SERIAL_SMV(CFG,"  M204 P", planner.acceleration);
  SERIAL_EMV(" V", planner.travel_acceleration);
  #if EXTRUDERS > 0
    for (int8_t i = 0; i < EXTRUDERS; i++) {
      SERIAL_SMV(CFG, "  M204 T", i);
      SERIAL_EMV(" R", planner.retract_acceleration[i]);
    }
  #endif

  CONFIG_MSG_START("Advanced variables: S=Min feedrate (mm/s), V=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum X jerk (mm/s), Y=maximum Y jerk (mm/s), Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
  SERIAL_SMV(CFG, "  M205 S", planner.min_feedrate_mm_s );
  SERIAL_MV(" V", planner.min_travel_feedrate_mm_s );
  SERIAL_MV(" B", planner.min_segment_time );
  SERIAL_MV(" X", planner.max_jerk[X_AXIS]);
  SERIAL_MV(" Y", planner.max_jerk[Y_AXIS]);
  SERIAL_MV(" Z", planner.max_jerk[Z_AXIS]);
  SERIAL_EMV(" E", planner.max_jerk[E_AXIS]);
  #if (EXTRUDERS > 1)
    for(int8_t i = 1; i < EXTRUDERS; i++) {
      SERIAL_SMV(CFG, "  M205 T", i);
      SERIAL_EMV(" E" , planner.max_jerk[E_AXIS + i]);
    }
  #endif

  CONFIG_MSG_START("Home offset (mm):");
  SERIAL_SMV(CFG, "  M206 X", home_offset[X_AXIS] );
  SERIAL_MV(" Y", home_offset[Y_AXIS] );
  SERIAL_EMV(" Z", home_offset[Z_AXIS] );

  CONFIG_MSG_START("Hotend offset (mm):");
  for (int8_t h = 0; h < HOTENDS; h++) {
    SERIAL_SMV(CFG, "  M218 T", h);
    SERIAL_MV(" X", hotend_offset[X_AXIS][h]);
    SERIAL_MV(" Y", hotend_offset[Y_AXIS][h]);
    SERIAL_EMV(" Z", hotend_offset[Z_AXIS][h]);
  }

  #if HAS(LCD_CONTRAST)
    CONFIG_MSG_START("LCD Contrast:");
    SERIAL_LMV(CFG, "  M250 C", lcd_contrast);
  #endif

  #if ENABLED(MESH_BED_LEVELING)
    CONFIG_MSG_START("Mesh bed leveling:");
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
    for (int8_t h = 0; h < HOTENDS; h++) {
      SERIAL_SMV(CFG, "  M595 T", h);
      SERIAL_MV(" O", ad595_offset[h]);
      SERIAL_EMV(", S", ad595_gain[h]);
    }
  #endif // HEATER_USES_AD595

  #if MECH(DELTA)
    CONFIG_MSG_START("Delta Geometry adjustment:");
    SERIAL_SMV(CFG, "  M666 A", tower_adj[0], 3);
    SERIAL_MV(" B", tower_adj[1], 3);
    SERIAL_MV(" C", tower_adj[2], 3);
    SERIAL_MV(" I", tower_adj[3], 3);
    SERIAL_MV(" J", tower_adj[4], 3);
    SERIAL_MV(" K", tower_adj[5], 3);
    SERIAL_MV(" U", diagrod_adj[0], 3);
    SERIAL_MV(" V", diagrod_adj[1], 3);
    SERIAL_MV(" W", diagrod_adj[2], 3);
    SERIAL_MV(" R", delta_radius);
    SERIAL_MV(" D", delta_diagonal_rod);
    SERIAL_EMV(" H", soft_endstop_max[2]);

    CONFIG_MSG_START("Endstop Offsets:");
    SERIAL_SMV(CFG, "  M666 X", endstop_adj[X_AXIS]);
    SERIAL_MV(" Y", endstop_adj[Y_AXIS]);
    SERIAL_EMV(" Z", endstop_adj[Z_AXIS]);

  #elif ENABLED(Z_DUAL_ENDSTOPS)
    CONFIG_MSG_START("Z2 Endstop adjustement (mm):");
    SERIAL_LMV(CFG, "  M666 Z", z_endstop_adj );
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
    SERIAL_SMV(CFG, "  M145 S0 H", preheatHotendTemp1);
    SERIAL_MV(" B", preheatBedTemp1);
    SERIAL_MV(" F", preheatFanSpeed1);
    SERIAL_EM(" (Material PLA)");
    SERIAL_SMV(CFG, "  M145 S1 H", preheatHotendTemp2);
    SERIAL_MV(" B", preheatBedTemp2);
    SERIAL_MV(" F", preheatFanSpeed2);
    SERIAL_EM(" (Material ABS)");
    SERIAL_SMV(CFG, "  M145 S2 H", preheatHotendTemp3);
    SERIAL_MV(" B", preheatBedTemp3);
    SERIAL_MV(" F", preheatFanSpeed3);
    SERIAL_EM(" (Material GUM)");
  #endif // ULTIPANEL

  #if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED) || ENABLED(PIDTEMPCHAMBER) || ENABLED(PIDTEMPCOOLER)
    CONFIG_MSG_START("PID settings:");
    #if ENABLED(PIDTEMP)
      for (int8_t h = 0; h < HOTENDS; h++) {
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
      SERIAL_SMV(CFG, "  M304 P", bedKp);
      SERIAL_MV(" I", unscalePID_i(bedKi));
      SERIAL_EMV(" D", unscalePID_d(bedKd));
    #endif
    #if ENABLED(PIDTEMPCHAMBER)
      SERIAL_SMV(CFG, "  M305 P", chamberKp);
      SERIAL_MV(" I", unscalePID_i(chamberKi));
      SERIAL_EMV(" D", unscalePID_d(chamberKd));
    #endif
    #if ENABLED(PIDTEMPCOOLER)
      SERIAL_SMV(CFG, "  M306 P", coolerKp);
      SERIAL_MV(" I", unscalePID_i(coolerKi));
      SERIAL_EMV(" D", unscalePID_d(coolerKd));
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

  if (volumetric_enabled) {
    CONFIG_MSG_START("Filament settings:");
    SERIAL_LMV(CFG, "  M200 D", filament_size[0]);

    #if EXTRUDERS > 1
      SERIAL_LMV(CFG, "  M200 T1 D", filament_size[1]);
      #if EXTRUDERS > 2
        SERIAL_LMV(CFG, "  M200 T2 D", filament_size[2]);
        #if EXTRUDERS > 3
          SERIAL_LMV(CFG, "  M200 T3 D", filament_size[3]);
        #endif
      #endif
    #endif

  }
  else
    CONFIG_MSG_START("  M200 D0");

  #if MB(ALLIGATOR)
    CONFIG_MSG_START("Motor current:");
    SERIAL_SMV(CFG, "  M906 X", motor_current[X_AXIS]);
    SERIAL_MV(" Y", motor_current[Y_AXIS]);
    SERIAL_MV(" Z", motor_current[Z_AXIS]);
    SERIAL_EMV(" E", motor_current[E_AXIS]);
    #if DRIVER_EXTRUDERS > 1
      for (uint8_t i = 1; i < DRIVER_EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M906 T", i);
        SERIAL_EMV(" E", motor_current[E_AXIS + i]);
      }
    #endif // DRIVER_EXTRUDERS > 1
  #endif // ALLIGATOR

  ConfigSD_PrintSettings(forReplay);

}

void ConfigSD_PrintSettings(bool forReplay) {
  // Always have this function, even with SD_SETTINGS disabled, the current values will be shown

  #if HAS(POWER_CONSUMPTION_SENSOR)
    CONFIG_MSG_START("Watt/h consumed:");
    SERIAL_LMV(INFO, power_consumption_hour," Wh");
  #endif

  print_job_counter.showStats();
}

#endif // !DISABLE_M503

/**
* Configuration on SD card
*
* Author: Simone Primarosa
*
*/
void ConfigSD_ResetDefault() {
  #if HAS(POWER_CONSUMPTION_SENSOR)
    power_consumption_hour = 0;
  #endif
  print_job_counter.initStats();
  SERIAL_LM(OK, "Hardcoded SD Default Settings Loaded");
}

#if ENABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)

static const char *cfgSD_KEY[] = { // Keep this in lexicographical order for better search performance(O(Nlog2(N)) insted of O(N*N)) (if you don't keep this sorted, the algorithm for find the key index won't work, keep attention.)
  "CPR",  // Number of complete prints
  "FIL",  // Filament Usage
  "NPR",  // Number of prints
#if HAS(POWER_CONSUMPTION_SENSOR)
  "PWR",  // Power Consumption
#endif
  "TME",  // Longest print job
  "TPR"   // Total printing time
};

void ConfigSD_StoreSettings() {
  if(!IS_SD_INSERTED || card.isFileOpen() || card.sdprinting) return;

  set_sd_dot();
  card.setroot(true);
  card.startWrite((char *)CFG_SD_FILE, false);
  char buff[CFG_SD_MAX_VALUE_LEN];
  ltoa(print_job_counter.data.finishedPrints, buff, 10);
  card.unparseKeyLine(cfgSD_KEY[SD_CFG_CPR], buff);
  ltoa(print_job_counter.data.filamentUsed, buff, 10);
  card.unparseKeyLine(cfgSD_KEY[SD_CFG_FIL], buff);
  ltoa(print_job_counter.data.totalPrints, buff, 10);
  card.unparseKeyLine(cfgSD_KEY[SD_CFG_NPR], buff);
  #if HAS(POWER_CONSUMPTION_SENSOR)
    ltoa(power_consumption_hour, buff, 10);
    card.unparseKeyLine(cfgSD_KEY[SD_CFG_PWR], buff);
  #endif
  ltoa(print_job_counter.data.printer_usage, buff, 10);
  card.unparseKeyLine(cfgSD_KEY[SD_CFG_TME], buff);
  ltoa(print_job_counter.data.printTime, buff, 10);
  card.unparseKeyLine(cfgSD_KEY[SD_CFG_TPR], buff);

  card.closeFile();
  card.setlast();
  unset_sd_dot();
}

void ConfigSD_RetrieveSettings(bool addValue) {
  if(!IS_SD_INSERTED || card.isFileOpen() || card.sdprinting || !card.cardOK) return;

  set_sd_dot();
  char key[CFG_SD_MAX_KEY_LEN], value[CFG_SD_MAX_VALUE_LEN];
  int k_idx;
  int k_len, v_len;
  card.setroot(true);
  card.selectFile((char *)CFG_SD_FILE);

  while(true) {
    k_len = CFG_SD_MAX_KEY_LEN;
    v_len = CFG_SD_MAX_VALUE_LEN;
    card.parseKeyLine(key, value, k_len, v_len);

    if(k_len == 0 || v_len == 0) break; // no valid key or value founded

    k_idx = ConfigSD_KeyIndex(key);
    if(k_idx == -1) continue;    // unknow key ignore it

    switch(k_idx) {
      case SD_CFG_CPR: {
        if(addValue) print_job_counter.data.finishedPrints += (unsigned long)atol(value);
        else print_job_counter.data.finishedPrints = (unsigned long)atol(value);
      }
      break;
      case SD_CFG_FIL: {
        if(addValue) print_job_counter.data.filamentUsed += (unsigned long)atol(value);
        else print_job_counter.data.filamentUsed = (unsigned long)atol(value);
      }
      break;
      case SD_CFG_NPR: {
        if(addValue) print_job_counter.data.totalPrints += (unsigned long)atol(value);
        else print_job_counter.data.totalPrints = (unsigned long)atol(value);
      }
      break;
    #if HAS(POWER_CONSUMPTION_SENSOR)
      case SD_CFG_PWR: {
        if(addValue) power_consumption_hour += (unsigned long)atol(value);
        else power_consumption_hour = (unsigned long)atol(value);
      }
      break;
    #endif
      case SD_CFG_TME: {
        if(addValue) print_job_counter.data.printer_usage += (unsigned long)atol(value);
        else print_job_counter.data.printer_usage = (unsigned long)atol(value);
      }
      break;
      case SD_CFG_TPR: {
        if(addValue) print_job_counter.data.printTime += (unsigned long)atol(value);
        else print_job_counter.data.printTime = (unsigned long)atol(value);
      }
      break;
    }
  }

  print_job_counter.loaded = true;
  card.closeFile();
  card.setlast();
  unset_sd_dot();
}

int ConfigSD_KeyIndex(char *key) {  // At the moment a binary search algorithm is used for simplicity, if it will be necessary (Eg. tons of key), an hash search algorithm will be implemented.
  int begin = 0, end = SD_CFG_END - 1, middle, cond;

  while(begin <= end) {
    middle = (begin + end) / 2;
    cond = strcmp(cfgSD_KEY[middle], key);
    if(!cond) return middle;
    else if(cond < 0) begin = middle + 1;
    else end = middle - 1;
  }

  return -1;
}

#endif

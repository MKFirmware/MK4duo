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

#include "../../../MK4duo.h"

#define EEPROM_VERSION "MKV45"

/**
 * MKV45 EEPROM Layout:
 *
 *  Version                                                     (char x6)
 *  EEPROM Checksum                                             (uint16_t)
 *
 *  M92   XYZ E0 ...      mechanics.axis_steps_per_mm X,Y,Z,E0 ... (float x9)
 *  M203  XYZ E0 ...      mechanics.max_feedrate_mm_s X,Y,Z,E0 ... (float x9)
 *  M201  XYZ E0 ...      mechanics.max_acceleration_mm_per_s2 X,Y,Z,E0 ... (uint32_t x9)
 *  M204  P               mechanics.acceleration                (float)
 *  M204  R   E0 ...      mechanics.retract_acceleration        (float x6)
 *  M204  T               mechanics.travel_acceleration         (float)
 *  M205  S               mechanics.min_feedrate_mm_s           (float)
 *  M205  T               mechanics.min_travel_feedrate_mm_s    (float)
 *  M205  B               mechanics.min_segment_time_us         (ulong)
 *  M205  X               mechanics.max_jerk[X_AXIS]            (float)
 *  M205  Y               mechanics.max_jerk[Y_AXIS]            (float)
 *  M205  Z               mechanics.max_jerk[Z_AXIS]            (float)
 *  M205  E   E0 ...      mechanics.max_jerk[E_AXIS * EXTRUDERS](float x6)
 *  M206  XYZ             mechanics.home_offset                 (float x3)
 *  M218  T   XY          tools.hotend_offset                   (float x6)
 *
 * ENDSTOPS:
 *                        endstops.logic_bits                   (uint16_t)
 *                        endstops.pullup_bits                  (uint16_t)
 *
 * Global Leveling:
 *                        z_fade_height                         (float)
 *
 * MESH_BED_LEVELING:
 *  M420  S               bedlevel.leveling_active              (bool)
 *                        mbl.z_offset                          (float)
 *                        GRID_MAX_POINTS_X                     (uint8_t)
 *                        GRID_MAX_POINTS_Y                     (uint8_t)
 *  G29   S3  XYZ         z_values[][]                          (float x9, by default, up to float x 81) +288
 *
 * ABL_PLANAR:
 *                        bedlevel.matrix                       (matrix_3x3 = float x9)
 *
 * AUTO_BED_LEVELING_BILINEAR:
 *                        GRID_MAX_POINTS_X                     (uint8_t)
 *                        GRID_MAX_POINTS_Y                     (uint8_t)
 *                        abl.bilinear_grid_spacing             (int x2)   from G29: (B-F)/X, (R-L)/Y
 *  G29   L F             abl.bilinear_start                    (int x2)
 *                        abl.z_values[][]                      (float x9, up to float x256)
 *
 * AUTO_BED_LEVELING_UBL:
 *  G29 A                 bedlevel.leveling_active              (bool)
 *  G29 S                 ubl.storage_slot                      (int8_t)
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
 * HEATER:
 *  M301  H0  PIDC        Kp[0], Ki[0], Kd[0], Kc[0]            (float x4)
 *  M301  H1  PIDC        Kp[1], Ki[1], Kd[1], Kc[1]            (float x4)
 *  M301  H2  PIDC        Kp[2], Ki[2], Kd[2], Kc[2]            (float x4)
 *  M301  H3  PIDC        Kp[3], Ki[3], Kd[3], Kc[3]            (float x4)
 *  M301  L               lpq_len                               (int   x1)
 *  M301  H-1 PID         Kp, Ki, Kd                            (float x3)
 *  M301  H-2 PID         Kp, Ki, Kd                            (float x3)
 *  M301  H-3 PID         Kp, Ki, Kd                            (float x3)
 *
 *  M305  H0              Hotend 0  Sensor parameters
 *  M305  H1              Hotend 1  Sensor parameters
 *  M305  H2              Hotend 2  Sensor parameters
 *  M305  H3              Hotend 3  Sensor parameters
 *  M305  H-1             BED       Sensor parameters
 *  M305  H-2             CHAMBER   Sensor parameters
 *  M306  H-3             COOLER    Sensor parameters
 *
 *  M306  H0              Hotend 0  parameters
 *  M306  H1              Hotend 1  parameters
 *  M306  H2              Hotend 2  parameters
 *  M306  H3              Hotend 3  parameters
 *  M306  H-1             BED       parameters
 *  M306  H-2             CHAMBER   parameters
 *  M306  H-3             COOLER    parameters
 *
 *  M595  H   OS          Heaters AD595 Offset & Gain
 *
 * DHT SENSOR:
 *  M305  D0  SP          DHT Sensor parameters
 *
 * FANS:
 *  M106  P   SFHULI      Fans parameters
 *
 * DOGLCD:
 *  M250  C               lcd_contrast                                  (uint16_t)
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
 * SENSORLESS HOMING:
 *  M914  X               Stepper X and X2 threshold            (int16_t)
 *  M914  Y               Stepper Y and Y2 threshold            (int16_t)
 *  M914  Z               Stepper Z and Z2 threshold            (int16_t)
 *
 * LIN_ADVANCE:
 *  M900  K               planner.extruder_advance_K            (float)
 *
 * ADVANCED_PAUSE_FEATURE:
 *  M603 U                filament_change_unload_length         (float)
 *  M603 L                filament_change_load_length           (float)
 *
 * ========================================================================
 * meshes_begin (between max and min end-point, directly above)
 * -- MESHES --
 * meshes_end
 * -- MAT (Mesh Allocation Table) --                128 bytes (placeholder size)
 * mat_end = E2END (0xFFF)
 *
 */

EEPROM eeprom;

#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
  float new_z_fade_height;
#endif

/**
 * Post-process after Retrieve or Reset
 */
void EEPROM::Postprocess() {

  const float oldpos[] = {
    mechanics.current_position[X_AXIS],
    mechanics.current_position[Y_AXIS],
    mechanics.current_position[Z_AXIS]
  };

  // steps per s2 needs to be updated to agree with units per s2
  mechanics.reset_acceleration_rates();

  // Make sure delta kinematics are updated before refreshing the
  // planner position so the stepper counts will be set correctly.
  #if MECH(DELTA)
    mechanics.recalc_delta_settings();
  #endif

  #if HEATER_COUNT > 0
    LOOP_HEATER() {
      heaters[h].init();
      heaters[h].updatePID();
    }
  #endif

  #if ENABLED(DHT_SENSOR)
    dhtsensor.init();
  #endif

  #if FAN_COUNT > 0
    LOOP_FAN() fans[f].init();
  #endif

  #if ENABLED(VOLUMETRIC_EXTRUSION)
    tools.calculate_volumetric_multipliers();
  #else
    for (uint8_t i = COUNT(tools.e_factor); i--;)
      tools.refresh_e_factor(i);
  #endif

  #if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
    // Software endstops depend on home_offset
    LOOP_XYZ(i) endstops.update_software_endstops((AxisEnum)i);
  #endif

  #if HAS_LEVELING && ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    bedlevel.set_z_fade_height(new_z_fade_height, false);
  #endif

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    abl.refresh_bed_level();
  #endif

  #if ENABLED(FWRETRACT)
    fwretract.refresh_autoretract();
  #endif

  #if ENABLED(HYSTERESIS)
    mechanics.calc_hysteresis_steps();
  #endif

  // Setup Endstops pullup
  endstops.setup_pullup();

  // Refresh steps_to_mm with the reciprocal of axis_steps_per_mm
  // and init stepper.count[], planner.position[] with current_position
  mechanics.refresh_positioning();

  if (memcmp(oldpos, mechanics.current_position, sizeof(oldpos)))
    mechanics.report_current_position();

}

#if HAS_EEPROM

  #define EEPROM_READ_START()   int eeprom_index = EEPROM_OFFSET; eeprom_error = access_start(true)
  #define EEPROM_WRITE_START()  int eeprom_index = EEPROM_OFFSET; eeprom_error = access_start(false)
  #define EEPROM_FINISH()       access_finish()
  #define EEPROM_SKIP(VAR)      eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR)     eeprom_error = write_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ(VAR)      eeprom_error = read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)

  const char version[6] = EEPROM_VERSION;

  bool EEPROM::eeprom_error = false;

  #if ENABLED(AUTO_BED_LEVELING_UBL)
    int16_t EEPROM::meshes_begin = 0;
  #endif

  /**
   * M500 - Store Configuration
   */
  bool EEPROM::Store_Settings() {
    char ver[6] = "00000";

    uint16_t working_crc = 0;

    EEPROM_WRITE_START();

    #if HAS_EEPROM_FLASH
      EEPROM_SKIP(ver);         // Flash doesn't allow rewriting without erase
      EEPROM_SKIP(working_crc); // Skip the checksum slot
    #elif HAS_EEPROM_SD
      EEPROM_WRITE(version);
    #else
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
    EEPROM_WRITE(mechanics.min_segment_time_us);
    EEPROM_WRITE(mechanics.max_jerk);
    #if ENABLED(WORKSPACE_OFFSETS)
      EEPROM_WRITE(mechanics.home_offset);
    #endif
    EEPROM_WRITE(tools.hotend_offset);

    //
    // Endstops bit
    //
    EEPROM_WRITE(endstops.logic_bits);
    EEPROM_WRITE(endstops.pullup_bits);

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
      const uint8_t mesh_num_x = GRID_MAX_POINTS_X, mesh_num_y = GRID_MAX_POINTS_Y;
      EEPROM_WRITE(mbl.z_offset);
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
        sizeof(abl.z_values) == GRID_MAX_POINTS * sizeof(abl.z_values[0][0]),
        "Bilinear Z array is the wrong size."
      );
      const uint8_t grid_max_x = GRID_MAX_POINTS_X, grid_max_y = GRID_MAX_POINTS_Y;
      EEPROM_WRITE(grid_max_x);             // 1 byte
      EEPROM_WRITE(grid_max_y);             // 1 byte
      EEPROM_WRITE(abl.bilinear_grid_spacing);  // 2 ints
      EEPROM_WRITE(abl.bilinear_start);         // 2 ints
      EEPROM_WRITE(abl.z_values);               // 9-256 floats
    #endif // AUTO_BED_LEVELING_BILINEAR

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      EEPROM_WRITE(bedlevel.leveling_active);
      EEPROM_WRITE(ubl.storage_slot);
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

    #if ENABLED(X_TWO_ENDSTOPS)
      EEPROM_WRITE(endstops.x_endstop_adj);
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      EEPROM_WRITE(endstops.y_endstop_adj);
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      EEPROM_WRITE(endstops.z_endstop_adj);
    #endif

    #if ENABLED(ULTIPANEL)
      EEPROM_WRITE(lcd_preheat_hotend_temp);
      EEPROM_WRITE(lcd_preheat_bed_temp);
      EEPROM_WRITE(lcd_preheat_fan_speed);
    #endif

    #if HEATER_COUNT > 0
      LOOP_HEATER() {
        EEPROM_WRITE(heaters[h].type);
        EEPROM_WRITE(heaters[h].pin);
        EEPROM_WRITE(heaters[h].ID);
        EEPROM_WRITE(heaters[h].pidDriveMin);
        EEPROM_WRITE(heaters[h].pidDriveMax);
        EEPROM_WRITE(heaters[h].pidMax);
        EEPROM_WRITE(heaters[h].mintemp);
        EEPROM_WRITE(heaters[h].maxtemp);
        EEPROM_WRITE(heaters[h].Kp);
        EEPROM_WRITE(heaters[h].Ki);
        EEPROM_WRITE(heaters[h].Kd);
        EEPROM_WRITE(heaters[h].Kc);
        EEPROM_WRITE(heaters[h].HeaterFlag);
        EEPROM_WRITE(heaters[h].sensor.pin);
        EEPROM_WRITE(heaters[h].sensor.type);
        EEPROM_WRITE(heaters[h].sensor.adcLowOffset);
        EEPROM_WRITE(heaters[h].sensor.adcHighOffset);
        EEPROM_WRITE(heaters[h].sensor.r25);
        EEPROM_WRITE(heaters[h].sensor.beta);
        EEPROM_WRITE(heaters[h].sensor.pullupR);
        EEPROM_WRITE(heaters[h].sensor.shC);
        #if HEATER_USES_AD595
          EEPROM_WRITE(heaters[h].sensor.ad595_offset);
          EEPROM_WRITE(heaters[h].sensor.ad595_gain);
        #endif
      }
    #endif

    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      EEPROM_WRITE(tools.lpq_len);
    #endif

    #if ENABLED(DHT_SENSOR)
      EEPROM_WRITE(dhtsensor.pin);
      EEPROM_WRITE(dhtsensor.type);
    #endif

    #if FAN_COUNT > 0
      LOOP_FAN() {
        EEPROM_WRITE(fans[f].pin);
        EEPROM_WRITE(fans[f].freq);
        EEPROM_WRITE(fans[f].min_Speed);
        EEPROM_WRITE(fans[f].autoMonitored);
        EEPROM_WRITE(fans[f].FanFlag);
      }
    #endif

    #if HAS_LCD_CONTRAST
      EEPROM_WRITE(lcd_contrast);
    #endif

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

    //
    // Volumetric & Filament Size
    //
    #if ENABLED(VOLUMETRIC_EXTRUSION)

      const bool volumetric_enabled = printer.isVolumetric();
      EEPROM_WRITE(volumetric_enabled);

      // Save filament sizes
      for (uint8_t e = 0; e < EXTRUDERS; e++)
        EEPROM_WRITE(tools.filament_size[e]);

    #endif

    #if ENABLED(IDLE_OOZING_PREVENT)
      EEPROM_WRITE(printer.IDLE_OOZING_enabled);
    #endif

    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      EEPROM_WRITE(externaldac.motor_current);
    #endif

    //
    // Save TMC2130 or TMC2208 Configuration, and placeholder values
    //
    #if HAS_TRINAMIC
      uint16_t currents[12] = {
        #if X_IS_TRINAMIC
          stepperX.getCurrent(),
        #else
          0,
        #endif
        #if Y_IS_TRINAMIC
          stepperY.getCurrent(),
        #else
          0,
        #endif
        #if Z_IS_TRINAMIC
          stepperZ.getCurrent(),
        #else
          0,
        #endif
        #if X2_IS_TRINAMIC
          stepperX2.getCurrent(),
        #else
          0,
        #endif
        #if Y2_IS_TRINAMIC
          stepperY2.getCurrent(),
        #else
          0,
        #endif
        #if Z2_IS_TRINAMIC
          stepperZ2.getCurrent(),
        #else
          0,
        #endif
        #if E0_IS_TRINAMIC
          stepperE0.getCurrent(),
        #else
          0,
        #endif
        #if E1_IS_TRINAMIC
          stepperE1.getCurrent(),
        #else
          0,
        #endif
        #if E2_IS_TRINAMIC
          stepperE2.getCurrent(),
        #else
          0,
        #endif
        #if E3_IS_TRINAMIC
          stepperE3.getCurrent(),
        #else
          0,
        #endif
        #if E4_IS_TRINAMIC
          stepperE4.getCurrent(),
        #else
          0,
        #endif
        #if E5_IS_TRINAMIC
          stepperE5.getCurrent()
        #else
          0
        #endif
      };
      EEPROM_WRITE(currents);
    #endif

    //
    // TMC2130 Sensorless homing threshold
    //
    #if ENABLED(SENSORLESS_HOMING)
      int16_t thrs;
      #if ENABLED(X_IS_TMC2130) && ENABLED(X_HOMING_SENSITIVITY)
        thrs = stepperX.sgt();
      #else
        thrs = 0;
      #endif
      EEPROM_WRITE(thrs);
      #if ENABLED(Y_IS_TMC2130) && ENABLED(Y_HOMING_SENSITIVITY)
        thrs = stepperY.sgt();
      #else
        thrs = 0;
      #endif
      EEPROM_WRITE(thrs);
      #if ENABLED(Z_IS_TMC2130) && ENABLED(Z_HOMING_SENSITIVITY)
        thrs = stepperZ.sgt();
      #else
        thrs = 0;
      #endif
      EEPROM_WRITE(thrs);
    #endif

    //
    // Linear Advance
    //
    #if ENABLED(LIN_ADVANCE)
      EEPROM_WRITE(planner.extruder_advance_K);
    #endif

    //
    // Advanced Pause
    //
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      EEPROM_WRITE(filament_change_unload_length);
      EEPROM_WRITE(filament_change_load_length);
    #endif

    if (!eeprom_error) {
      const int eeprom_size = eeprom_index;

      const uint16_t final_crc = working_crc;

      // Write the EEPROM header
      eeprom_index = EEPROM_OFFSET;

      EEPROM_WRITE(version);
      EEPROM_WRITE(final_crc);

      // Report storage size
      #if ENABLED(EEPROM_CHITCHAT)
        SERIAL_SMV(ECHO, "Settings Stored (", eeprom_size - (EEPROM_OFFSET));
        SERIAL_MV(" bytes; crc ", final_crc);
        SERIAL_EM(")");
      #endif
    }

    #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(UBL_SAVE_ACTIVE_ON_M500)
      if (ubl.storage_slot >= 0)
        store_mesh(ubl.storage_slot);
    #endif

    EEPROM_FINISH();

    return !eeprom_error;
  }

  /**
   * M501 - Load Configuration
   */
  bool EEPROM::Load_Settings() {

    uint16_t  working_crc = 0,
              stored_crc  = 0;

    char stored_ver[6];

    EEPROM_READ_START();

    #if HAS_EEPROM_SD
      EEPROM_READ(stored_ver);
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
      #if ENABLED(EEPROM_CHITCHAT)
        SERIAL_SM(ECHO, "EEPROM version mismatch ");
        SERIAL_MT("(EEPROM=", stored_ver);
        SERIAL_EM(" MK4duo=" EEPROM_VERSION ")");
      #endif
      Factory_Settings();
      eeprom_error = true;
    }
    else {

      float dummy = 0;

      working_crc = 0; // Init to 0. Accumulated by EEPROM_READ

      // version number match
      EEPROM_READ(mechanics.axis_steps_per_mm);
      EEPROM_READ(mechanics.max_feedrate_mm_s);
      EEPROM_READ(mechanics.max_acceleration_mm_per_s2);
      EEPROM_READ(mechanics.acceleration);
      EEPROM_READ(mechanics.retract_acceleration);
      EEPROM_READ(mechanics.travel_acceleration);
      EEPROM_READ(mechanics.min_feedrate_mm_s);
      EEPROM_READ(mechanics.min_travel_feedrate_mm_s);
      EEPROM_READ(mechanics.min_segment_time_us);
      EEPROM_READ(mechanics.max_jerk);
      #if ENABLED(WORKSPACE_OFFSETS)
        EEPROM_READ(mechanics.home_offset);
      #endif
      EEPROM_READ(tools.hotend_offset);

      //
      // Endstops bit
      //
      EEPROM_READ(endstops.logic_bits);
      EEPROM_READ(endstops.pullup_bits);

      //
      // General Leveling
      //
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        EEPROM_READ(new_z_fade_height);
      #endif

      //
      // Mesh (Manual) Bed Leveling
      //
      #if ENABLED(MESH_BED_LEVELING)
        uint8_t mesh_num_x = 0, mesh_num_y = 0;
        EEPROM_READ(mbl.z_offset);
        EEPROM_READ(mesh_num_x);
        EEPROM_READ(mesh_num_y);
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
          EEPROM_READ(abl.bilinear_grid_spacing); // 2 ints
          EEPROM_READ(abl.bilinear_start);        // 2 ints
          EEPROM_READ(abl.z_values);              // 9 to 256 floats
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
        EEPROM_READ(bedlevel.leveling_active);
        EEPROM_READ(ubl.storage_slot);
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

      #if ENABLED(X_TWO_ENDSTOPS)
        EEPROM_READ(endstops.x_endstop_adj);
      #endif
      #if ENABLED(Y_TWO_ENDSTOPS)
        EEPROM_READ(endstops.y_endstop_adj);
      #endif
      #if ENABLED(Z_TWO_ENDSTOPS)
        EEPROM_READ(endstops.z_endstop_adj);
      #endif

      #if ENABLED(ULTIPANEL)
        EEPROM_READ(lcd_preheat_hotend_temp);
        EEPROM_READ(lcd_preheat_bed_temp);
        EEPROM_READ(lcd_preheat_fan_speed);
      #endif

      #if HEATER_COUNT > 0
        LOOP_HEATER() {
          EEPROM_READ(heaters[h].type);
          EEPROM_READ(heaters[h].pin);
          EEPROM_READ(heaters[h].ID);
          EEPROM_READ(heaters[h].pidDriveMin);
          EEPROM_READ(heaters[h].pidDriveMax);
          EEPROM_READ(heaters[h].pidMax);
          EEPROM_READ(heaters[h].mintemp);
          EEPROM_READ(heaters[h].maxtemp);
          EEPROM_READ(heaters[h].Kp);
          EEPROM_READ(heaters[h].Ki);
          EEPROM_READ(heaters[h].Kd);
          EEPROM_READ(heaters[h].Kc);
          EEPROM_READ(heaters[h].HeaterFlag);
          EEPROM_READ(heaters[h].sensor.pin);
          EEPROM_READ(heaters[h].sensor.type);
          EEPROM_READ(heaters[h].sensor.adcLowOffset);
          EEPROM_READ(heaters[h].sensor.adcHighOffset);
          EEPROM_READ(heaters[h].sensor.r25);
          EEPROM_READ(heaters[h].sensor.beta);
          EEPROM_READ(heaters[h].sensor.pullupR);
          EEPROM_READ(heaters[h].sensor.shC);
          #if HEATER_USES_AD595
            EEPROM_READ(heaters[h].sensor.ad595_offset);
            EEPROM_READ(heaters[h].sensor.ad595_gain);
            if (heaters[h].sensor.ad595_gain == 0) heaters[h].sensor.ad595_gain = TEMP_SENSOR_AD595_GAIN;
          #endif
        }
      #endif

      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        EEPROM_READ(tools.lpq_len);
      #endif

      #if ENABLED(DHT_SENSOR)
        EEPROM_READ(dhtsensor.pin);
        EEPROM_READ(dhtsensor.type);
      #endif

      #if FAN_COUNT > 0
        LOOP_FAN() {
          EEPROM_READ(fans[f].pin);
          EEPROM_READ(fans[f].freq);
          EEPROM_READ(fans[f].min_Speed);
          EEPROM_READ(fans[f].autoMonitored);
          EEPROM_READ(fans[f].FanFlag);
        }
      #endif

      #if HAS_LCD_CONTRAST
        EEPROM_READ(lcd_contrast);
      #endif

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

      //
      // Volumetric & Filament Size
      //
      #if ENABLED(VOLUMETRIC_EXTRUSION)

        bool volumetric_enabled;
        EEPROM_READ(volumetric_enabled);
        printer.setVolumetric(volumetric_enabled);

        for (uint8_t e = 0; e < EXTRUDERS; e++)
          EEPROM_READ(tools.filament_size[e]);

      #endif

      #if ENABLED(IDLE_OOZING_PREVENT)
        EEPROM_READ(printer.IDLE_OOZING_enabled);
      #endif

      #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
        EEPROM_READ(externaldac.motor_current);
      #endif

      //
      // TMC2130 or TMC2208 Stepper Current
      //
      #if HAS_TRINAMIC
        #define SET_CURR(N,Q) stepper##Q.setCurrent(val[N] ? val[N] : Q##_CURRENT, R_SENSE, HOLD_MULTIPLIER)
        uint16_t val[12];
        EEPROM_READ(val);
        #if X_IS_TRINAMIC
          SET_CURR(0, X);
        #endif
        #if Y_IS_TRINAMIC
          SET_CURR(1, Y);
        #endif
        #if Z_IS_TRINAMIC
          SET_CURR(2, Z);
        #endif
        #if X2_IS_TRINAMIC
          SET_CURR(3, X2);
        #endif
        #if Y2_IS_TRINAMIC
          SET_CURR(4, Y2);
        #endif
        #if Z2_IS_TRINAMIC
          SET_CURR(5, Z2);
        #endif
        #if E0_IS_TRINAMIC
          SET_CURR(6, E0);
        #endif
        #if E1_IS_TRINAMIC
          SET_CURR(7, E1);
        #endif
        #if E2_IS_TRINAMIC
          SET_CURR(8, E2);
        #endif
        #if E3_IS_TRINAMIC
          SET_CURR(9, E3);
        #endif
        #if E4_IS_TRINAMIC
          SET_CURR(10, E4);
        #endif
        #if E5_IS_TRINAMIC
          SET_CURR(11, E5);
        #endif
      #endif

      /*
       * TMC2130 Sensorless homing threshold.
       * X and X2 use the same value
       * Y and Y2 use the same value
       */
      #if ENABLED(SENSORLESS_HOMING)
        int16_t thrs;
        EEPROM_READ(thrs);
        #if ENABLED(X_HOMING_SENSITIVITY)
          #if ENABLED(X_IS_TMC2130)
            stepperX.sgt(thrs);
          #endif
          #if ENABLED(X2_IS_TMC2130)
            stepperX2.sgt(thrs);
          #endif
        #endif
        EEPROM_READ(thrs);
        #if ENABLED(Y_HOMING_SENSITIVITY)
          #if ENABLED(Y_IS_TMC2130)
            stepperY.sgt(thrs);
          #endif
          #if ENABLED(Y2_IS_TMC2130)
            stepperY2.sgt(thrs);
          #endif
        #endif
        EEPROM_READ(thrs);
        #if ENABLED(Z_HOMING_SENSITIVITY)
          #if ENABLED(Z_IS_TMC2130)
            stepperZ.sgt(thrs);
          #endif
          #if ENABLED(Z2_IS_TMC2130)
            stepperZ2.sgt(thrs);
          #endif
        #endif
      #endif

      //
      // Linear Advance
      //
      #if ENABLED(LIN_ADVANCE)
        EEPROM_READ(planner.extruder_advance_K);
      #endif

      //
      // Advanced Pause
      //
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        EEPROM_READ(filament_change_unload_length);
        EEPROM_READ(filament_change_load_length);
      #endif
    
      #if HAS_EEPROM_SD
        // Read last two field
        uint16_t temp_crc;
        read_data(eeprom_index, (uint8_t*)&stored_ver, sizeof(stored_ver), &temp_crc);
        read_data(eeprom_index, (uint8_t*)&stored_crc, sizeof(stored_crc), &temp_crc);
      #endif

      if (working_crc == stored_crc) {
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_VAL(version);
          SERIAL_MV(" stored settings retrieved (", eeprom_index - (EEPROM_OFFSET));
          SERIAL_MV(" bytes; crc ", working_crc);
          SERIAL_EM(")");
        #endif
        Postprocess();
      }
      else {
        eeprom_error = true;
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_SMV(ER, "EEPROM CRC mismatch - (stored) ", stored_crc);
          SERIAL_MV(" != ", working_crc);
          SERIAL_EM(" (calculated)!");
        #endif
        Factory_Settings();
      }

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        meshes_begin = (eeprom_index + EEPROM_OFFSET + 32) & 0xFFF8;

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

        if (ubl.storage_slot >= 0) {
          load_mesh(ubl.storage_slot);
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_MV("Mesh ", ubl.storage_slot);
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

    EEPROM_FINISH();

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

    uint16_t EEPROM::calc_num_meshes() {
      return (meshes_end - meshes_start_index()) / sizeof(ubl.z_values);
    }

    int EEPROM::mesh_slot_offset(const int8_t slot) {
      return meshes_end - (slot + 1) * sizeof(ubl.z_values);
    }

    void EEPROM::store_mesh(const int8_t slot) {

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        const int16_t a = calc_num_meshes();
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
        int pos = mesh_slot_offset(slot);

        const bool status = write_data(pos, (uint8_t *)&ubl.z_values, sizeof(ubl.z_values), &crc);

        if (status)
          SERIAL_MSG("?Unable to save mesh data.\n");

        #if ENABLED(EEPROM_CHITCHAT)
          else
            SERIAL_EMV("Mesh saved in slot ", slot);
        #endif

      #else

        // Other mesh types

      #endif
    }

    void EEPROM::load_mesh(const int8_t slot, void * const into/*=NULL*/) {

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        const int16_t a = calc_num_meshes();

        if (!WITHIN(slot, 0, a - 1)) {
          #if ENABLED(EEPROM_CHITCHAT)
            ubl_invalid_slot(a);
          #endif
          return;
        }

        uint16_t crc = 0;
        int pos = mesh_slot_offset(slot);
        uint8_t * const dest = into ? (uint8_t*)into : (uint8_t*)&ubl.z_values;

        const bool status = read_data(pos, dest, sizeof(ubl.z_values), &crc);

        if (status)
          SERIAL_MSG("?Unable to load mesh data.\n");

        #if ENABLED(EEPROM_CHITCHAT)
          else
            SERIAL_EMV("Mesh loaded from slot ", slot);
        #endif

      #else

        // Other mesh types

      #endif
    }

  #endif // AUTO_BED_LEVELING_UBL

#else // !HAS_EEPROM

  bool EEPROM::Store_Settings() { SERIAL_LM(ER, "EEPROM disabled"); return false; }

#endif // HAS_EEPROM

/**
 * M502 - Reset Configuration
 */
void EEPROM::Factory_Settings() {

  static const float    tmp1[] PROGMEM  = DEFAULT_AXIS_STEPS_PER_UNIT,
                        tmp2[] PROGMEM  = DEFAULT_MAX_FEEDRATE;
  static const uint32_t tmp3[] PROGMEM  = DEFAULT_MAX_ACCELERATION,
                        tmp4[] PROGMEM  = DEFAULT_RETRACT_ACCELERATION;
  static const float    tmp5[] PROGMEM  = DEFAULT_EJERK,
                        tmp6[] PROGMEM  = DEFAULT_Kp,
                        tmp7[] PROGMEM  = DEFAULT_Ki,
                        tmp8[] PROGMEM  = DEFAULT_Kd,
                        tmp9[] PROGMEM  = DEFAULT_Kc;

  #if FAN_COUNT > 0
    static const pin_t  tmp10[] PROGMEM = FANS_CHANNELS;
    static const int8_t tmp11[] PROGMEM = AUTO_FAN;
  #endif

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    new_z_fade_height = 0.0;
  #endif

  #if ENABLED(HOTEND_OFFSET_X) && ENABLED(HOTEND_OFFSET_Y) && ENABLED(HOTEND_OFFSET_Z)
    constexpr float tmp12[XYZ][4] = {
      HOTEND_OFFSET_X,
      HOTEND_OFFSET_Y,
      HOTEND_OFFSET_Z
    };
  #else
    constexpr float tmp12[XYZ][HOTENDS] = { 0.0 };
  #endif

  #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
    const float tmp13[] = MOTOR_CURRENT;
    for (uint8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++)
      externaldac.motor_current[i] = tmp13[i < COUNT(tmp13) ? i : COUNT(tmp13) - 1];
  #endif

  LOOP_XYZE_N(i) {
    mechanics.axis_steps_per_mm[i]          = pgm_read_float(&tmp1[i < COUNT(tmp1) ? i : COUNT(tmp1) - 1]);
    mechanics.max_feedrate_mm_s[i]          = pgm_read_float(&tmp2[i < COUNT(tmp2) ? i : COUNT(tmp2) - 1]);
    mechanics.max_acceleration_mm_per_s2[i] = pgm_read_dword_near(&tmp3[i < COUNT(tmp3) ? i : COUNT(tmp3) - 1]);
  }

  for (uint8_t i = 0; i < EXTRUDERS; i++) {
    mechanics.retract_acceleration[i]       = pgm_read_dword_near(&tmp4[i < COUNT(tmp4) ? i : COUNT(tmp4) - 1]);
    mechanics.max_jerk[E_AXIS + i]          = pgm_read_float(&tmp5[i < COUNT(tmp5) ? i : COUNT(tmp5) - 1]);
  }

  static_assert(
    tmp12[X_AXIS][0] == 0 && tmp12[Y_AXIS][0] == 0 && tmp12[Z_AXIS][0] == 0,
    "Offsets for the first hotend must be 0.0."
  );
  LOOP_XYZ(i) {
    LOOP_HOTEND() tools.hotend_offset[i][h] = tmp12[i][h];
  }

  mechanics.acceleration              = DEFAULT_ACCELERATION;
  mechanics.travel_acceleration       = DEFAULT_TRAVEL_ACCELERATION;
  mechanics.min_feedrate_mm_s         = DEFAULT_MINIMUMFEEDRATE;
  mechanics.min_segment_time_us       = DEFAULT_MINSEGMENTTIME;
  mechanics.min_travel_feedrate_mm_s  = DEFAULT_MINTRAVELFEEDRATE;
  mechanics.max_jerk[X_AXIS]          = DEFAULT_XJERK;
  mechanics.max_jerk[Y_AXIS]          = DEFAULT_YJERK;
  mechanics.max_jerk[Z_AXIS]          = DEFAULT_ZJERK;

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
    probe.offset[X_AXIS] = X_PROBE_OFFSET_FROM_NOZZLE;
    probe.offset[Y_AXIS] = Y_PROBE_OFFSET_FROM_NOZZLE;
    probe.offset[Z_AXIS] = Z_PROBE_OFFSET_FROM_NOZZLE;
  #endif

  mechanics.init();

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

  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    tools.lpq_len = 20; // default last-position-queue size
  #endif

  // Heaters
  #if HEATER_COUNT > 0

    Heater *heat;
    TemperatureSensor *sens;

    #if HOTENDS > 0
      LOOP_HOTEND() {
        heat = &heaters[h];
        heat->Kp  = pgm_read_float(&tmp6[h < COUNT(tmp6) ? h : COUNT(tmp6) - 1]);
        heat->Ki  = pgm_read_float(&tmp7[h < COUNT(tmp7) ? h : COUNT(tmp7) - 1]);
        heat->Kd  = pgm_read_float(&tmp8[h < COUNT(tmp8) ? h : COUNT(tmp8) - 1]);
        heat->Kc  = pgm_read_float(&tmp9[h < COUNT(tmp9) ? h : COUNT(tmp9) - 1]);
      }
    #endif

    #if HAS_HEATER_0
      // HOTEND 0
      heat = &heaters[0];
      sens = &heaters[0].sensor;
      heat->type              = IS_HOTEND;
      heat->pin               = HEATER_0_PIN;
      heat->ID                = 0;
      heat->pidDriveMin       = PID_DRIVE_MIN;
      heat->pidDriveMax       = PID_DRIVE_MAX;
      heat->pidMax            = PID_MAX;
      heat->mintemp           = HEATER_0_MINTEMP;
      heat->maxtemp           = HEATER_0_MAXTEMP;
      // Sensor
      sens->pin               = TEMP_0_PIN;
      sens->type              = TEMP_SENSOR_0;
      sens->r25               = HOT0_R25;
      sens->beta              = HOT0_BETA;
      sens->pullupR           = THERMISTOR_SERIES_RS;
      sens->shC               = 0.0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if HEATER_USES_AD595
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->setUsePid(PIDTEMP);
      heat->setHWInverted(INVERTED_HEATER_PINS);
      #if HAS_EEPROM
        heat->setTuning(false);
      #else
        heat->setTuning(true);
      #endif
    #endif // HAS_HEATER_0

    #if HAS_HEATER_1
      // HOTEND 1
      heat = &heaters[1];
      sens = &heaters[1].sensor;
      heat->type              = IS_HOTEND;
      heat->pin               = HEATER_1_PIN;
      heat->ID                = 1;
      heat->pidDriveMin       = PID_DRIVE_MIN;
      heat->pidDriveMax       = PID_DRIVE_MAX;
      heat->pidMax            = PID_MAX;
      heat->mintemp           = HEATER_1_MINTEMP;
      heat->maxtemp           = HEATER_1_MAXTEMP;
      // Sensor
      sens->pin               = TEMP_1_PIN;
      sens->type              = TEMP_SENSOR_1;
      sens->r25               = HOT1_R25;
      sens->beta              = HOT1_BETA;
      sens->pullupR           = THERMISTOR_SERIES_RS;
      sens->shC               = 0.0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if HEATER_USES_AD595
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->setUsePid(PIDTEMP);
      heat->setHWInverted(INVERTED_HEATER_PINS);
      #if HAS_EEPROM
        heat->setTuning(false);
      #else
        heat->setTuning(true);
      #endif
    #endif // HAS_HEATER_1

    #if HAS_HEATER_2
      // HOTEND 2
      heat = &heaters[2];
      sens = &heaters[2].sensor;
      heat->type              = IS_HOTEND;
      heat->pin               = HEATER_2_PIN;
      heat->ID                = 2;
      heat->pidDriveMin       = PID_DRIVE_MIN;
      heat->pidDriveMax       = PID_DRIVE_MAX;
      heat->pidMax            = PID_MAX;
      heat->mintemp           = HEATER_2_MINTEMP;
      heat->maxtemp           = HEATER_2_MAXTEMP;
      // Sensor
      sens->pin               = TEMP_2_PIN;
      sens->type              = TEMP_SENSOR_2;
      sens->r25               = HOT2_R25;
      sens->beta              = HOT2_BETA;
      sens->pullupR           = THERMISTOR_SERIES_RS;
      sens->shC               = 0.0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if HEATER_USES_AD595
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->setUsePid(PIDTEMP);
      heat->setHWInverted(INVERTED_HEATER_PINS);
      #if HAS_EEPROM
        heat->setTuning(false);
      #else
        heat->setTuning(true);
      #endif
    #endif // HAS_HEATER_2

    #if HAS_HEATER_3
      // HOTEND 3
      heat = &heaters[3];
      sens = &heaters[3].sensor;
      heat->type              = IS_HOTEND;
      heat->pin               = HEATER_3_PIN;
      heat->ID                = 3;
      heat->pidDriveMin       = PID_DRIVE_MIN;
      heat->pidDriveMax       = PID_DRIVE_MAX;
      heat->pidMax            = PID_MAX;
      heat->mintemp           = HEATER_3_MINTEMP;
      heat->maxtemp           = HEATER_3_MAXTEMP;
      // Sensor
      sens->pin               = TEMP_3_PIN;
      sens->type              = TEMP_SENSOR_3;
      sens->r25               = HOT3_R25;
      sens->beta              = HOT3_BETA;
      sens->pullupR           = THERMISTOR_SERIES_RS;
      sens->shC               = 0.0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if HEATER_USES_AD595
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->setUsePid(PIDTEMP);
      heat->setHWInverted(INVERTED_HEATER_PINS);
      #if HAS_EEPROM
        heat->setTuning(false);
      #else
        heat->setTuning(true);
      #endif
    #endif // HAS_HEATER_3

    #if HAS_HEATER_BED
      // BED
      heat = &heaters[BED_INDEX];
      sens = &heaters[BED_INDEX].sensor;
      heat->type              = IS_BED;
      heat->pin               = HEATER_BED_PIN;
      heat->ID                = BED_INDEX;
      heat->pidDriveMin       = BED_PID_DRIVE_MIN;
      heat->pidDriveMax       = BED_PID_DRIVE_MAX;
      heat->pidMax            = BED_PID_MAX;
      heat->mintemp           = BED_MINTEMP;
      heat->maxtemp           = BED_MAXTEMP;
      heat->Kp                = DEFAULT_bedKp;
      heat->Ki                = DEFAULT_bedKi;
      heat->Kd                = DEFAULT_bedKd;
      // Sensor
      sens->pin               = TEMP_BED_PIN;
      sens->type              = TEMP_SENSOR_BED;
      sens->r25               = BED_R25;
      sens->beta              = BED_BETA;
      sens->pullupR           = THERMISTOR_SERIES_RS;
      sens->shC               = 0.0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if HEATER_USES_AD595
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->setUsePid(PIDTEMPBED);
      heat->setHWInverted(INVERTED_BED_PIN);
      #if HAS_EEPROM
        heat->setTuning(false);
      #else
        heat->setTuning(true);
      #endif
    #endif // HAS_HEATER_BED

    #if HAS_HEATER_CHAMBER
      // CHAMBER
      heat = &heaters[CHAMBER_INDEX];
      sens = &heaters[CHAMBER_INDEX].sensor;
      heat->type              = IS_CHAMBER;
      heat->pin               = HEATER_CHAMBER_PIN;
      heat->ID                = CHAMBER_INDEX;
      heat->pidDriveMin       = CHAMBER_PID_DRIVE_MIN;
      heat->pidDriveMax       = CHAMBER_PID_DRIVE_MAX;
      heat->pidMax            = CHAMBER_PID_MAX;
      heat->mintemp           = CHAMBER_MINTEMP;
      heat->maxtemp           = CHAMBER_MAXTEMP;
      heat->Kp                = DEFAULT_chamberKp;
      heat->Ki                = DEFAULT_chamberKi;
      heat->Kd                = DEFAULT_chamberKd;
      // Sensor
      sens->pin               = TEMP_CHAMBER_PIN;
      sens->type              = TEMP_SENSOR_CHAMBER;
      sens->r25               = CHAMBER_R25;
      sens->beta              = CHAMBER_BETA;
      sens->pullupR           = THERMISTOR_SERIES_RS;
      sens->shC               = 0.0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if HEATER_USES_AD595
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->setUsePid(PIDTEMPCHAMBER);
      heat->setHWInverted(INVERTED_CHAMBER_PIN);
      #if HAS_EEPROM
        heat->setTuning(false);
      #else
        heat->setTuning(true);
      #endif
    #endif // HAS_HEATER_BED

    #if HAS_HEATER_COOLER
      // COOLER
      heat = &heaters[COOLER_INDEX];
      sens = &heaters[COOLER_INDEX].sensor;
      heat->type              = IS_COOLER;
      heat->pin               = HEATER_COOLER_PIN;
      heat->ID                = COOLER_INDEX;
      heat->pidDriveMin       = COOLER_PID_DRIVE_MIN;
      heat->pidDriveMax       = COOLER_PID_DRIVE_MAX;
      heat->pidMax            = COOLER_PID_MAX;
      heat->mintemp           = COOLER_MINTEMP;
      heat->maxtemp           = COOLER_MAXTEMP;
      heat->Kp                = DEFAULT_coolerKp;
      heat->Ki                = DEFAULT_coolerKi;
      heat->Kd                = DEFAULT_coolerKd;
      // Sensor
      sens->pin               = TEMP_COOLER_PIN;
      sens->type              = TEMP_SENSOR_COOLER;
      sens->r25               = COOLER_R25;
      sens->beta              = COOLER_BETA;
      sens->pullupR           = THERMISTOR_SERIES_RS;
      sens->shC               = 0.0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if HEATER_USES_AD595
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->setUsePid(PIDTEMPCOOLER);
      heat->setHWInverted(INVERTED_COOLER_PIN);
      #if HAS_EEPROM
        heat->setTuning(false);
      #else
        heat->setTuning(true);
      #endif
    #endif // HAS_HEATER_BED

  #endif // HEATER_COUNT > 0

  // Fans
  #if FAN_COUNT > 0
    Fan *fan;
    LOOP_FAN() {
      fan = &fans[f];
      fan->pin            = (int8_t)pgm_read_byte(&tmp10[f]);
      fan->freq           = 250;
      fan->min_Speed      = FAN_MIN_PWM;
      fan->autoMonitored  = 0;
      fan->SetAutoMonitored((int8_t)pgm_read_byte(&tmp11[f]));
      fan->setHWInverted(FAN_INVERTED);
    }
  #endif

  #if ENABLED(DHT_SENSOR)
    dhtsensor.pin   = DHT_DATA_PIN;
    dhtsensor.type  = DHT_TYPE;
  #endif

  #if ENABLED(FWRETRACT)
    fwretract.reset();
  #endif

  #if ENABLED(VOLUMETRIC_EXTRUSION)

    #if ENABLED(VOLUMETRIC_DEFAULT_ON)
      printer.setVolumetric(true);
    #else
      printer.setVolumetric(false);
    #endif

    for (uint8_t q = 0; q < COUNT(tools.filament_size); q++)
      tools.filament_size[q] = DEFAULT_NOMINAL_FILAMENT_DIA;

  #endif

  endstops.setGlobally(
    #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
      (false)
    #else
      (true)
    #endif
  );

  #if ENABLED(X_TWO_ENDSTOPS)
    endstops.x_endstop_adj = 0.0;
  #endif
  #if ENABLED(Y_TWO_ENDSTOPS)
    endstops.y_endstop_adj = 0.0;
  #endif
  #if ENABLED(Z_TWO_ENDSTOPS)
    endstops.z_endstop_adj = 0.0;
  #endif

  #if ENABLED(IDLE_OOZING_PREVENT)
    printer.IDLE_OOZING_enabled = true;
  #endif

  #if X_IS_TRINAMIC
    stepperX.setCurrent(X_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if Y_IS_TRINAMIC
    stepperY.setCurrent(Y_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if Z_IS_TRINAMIC
    stepperZ.setCurrent(Z_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if X2_IS_TRINAMIC
    stepperX2.setCurrent(X2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if Y2_IS_TRINAMIC
    stepperY2.setCurrent(Y2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if Z2_IS_TRINAMIC
    stepperZ2.setCurrent(Z2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if E0_IS_TRINAMIC
    stepperE0.setCurrent(E0_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if E1_IS_TRINAMIC
    stepperE1.setCurrent(E1_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if E2_IS_TRINAMIC
    stepperE2.setCurrent(E2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if E3_IS_TRINAMIC
    stepperE3.setCurrent(E3_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if E4_IS_TRINAMIC
    stepperE4.setCurrent(E4_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif
  #if E5_IS_TRINAMIC
    stepperE5.setCurrent(E5_CURRENT, R_SENSE, HOLD_MULTIPLIER);
  #endif

  #if ENABLED(SENSORLESS_HOMING)
    #if ENABLED(X_HOMING_SENSITIVITY)
      #if ENABLED(X_IS_TMC2130)
        stepperX.sgt(X_HOMING_SENSITIVITY);
      #endif
      #if ENABLED(X2_IS_TMC2130)
        stepperX2.sgt(X_HOMING_SENSITIVITY);
      #endif
    #endif
    #if ENABLED(Y_HOMING_SENSITIVITY)
      #if ENABLED(Y_IS_TMC2130)
        stepperY.sgt(Y_HOMING_SENSITIVITY);
      #endif
      #if ENABLED(Y2_IS_TMC2130)
        stepperY2.sgt(Y_HOMING_SENSITIVITY);
      #endif
    #endif
    #if ENABLED(Z_HOMING_SENSITIVITY)
      #if ENABLED(Z_IS_TMC2130)
        stepperZ.sgt(Z_HOMING_SENSITIVITY);
      #endif
      #if ENABLED(Z2_IS_TMC2130)
        stepperZ2.sgt(Z_HOMING_SENSITIVITY);
      #endif
    #endif
  #endif

  #if ENABLED(LIN_ADVANCE)
    planner.extruder_advance_K = LIN_ADVANCE_K;
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    for (uint8_t e = 0; e < DRIVER_EXTRUDERS; e++) {
      filament_change_unload_length[e] = PAUSE_PARK_UNLOAD_LENGTH;
      filament_change_load_length[e] = PAUSE_PARK_LOAD_LENGTH;
    }
  #endif

  #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
    endstops.setLogic(X_MIN, !X_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(Y_MIN, !Y_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(Z_MIN, !Z_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(X_MAX, !X_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Y_MAX, !Y_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Z_MAX, !Z_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(X2_MIN, !X2_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(Y2_MIN, !Y2_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(Z2_MIN, !Z2_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(X2_MAX, !X2_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Y2_MAX, !Y2_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Z2_MAX, !Z2_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Z_PROBE, !Z_PROBE_ENDSTOP_LOGIC);
    endstops.setLogic(FIL_RUNOUT, !FIL_RUNOUT_LOGIC);
    endstops.setLogic(DOOR_OPEN, !DOOR_OPEN_LOGIC);
    endstops.setLogic(POWER_CHECK, !POWER_CHECK_LOGIC);
  #else
    endstops.setLogic(X_MIN, X_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(Y_MIN, Y_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(Z_MIN, Z_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(X_MAX, X_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Y_MAX, Y_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Z_MAX, Z_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(X2_MIN, X2_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(Y2_MIN, Y2_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(Z2_MIN, Z2_MIN_ENDSTOP_LOGIC);
    endstops.setLogic(X2_MAX, X2_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Y2_MAX, Y2_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Z2_MAX, Z2_MAX_ENDSTOP_LOGIC);
    endstops.setLogic(Z_PROBE, Z_PROBE_ENDSTOP_LOGIC);
    endstops.setLogic(FIL_RUNOUT, FIL_RUNOUT_LOGIC);
    endstops.setLogic(DOOR_OPEN, DOOR_OPEN_LOGIC);
    endstops.setLogic(POWER_CHECK, POWER_CHECK_LOGIC);
  #endif

  endstops.setPullup(X_MIN, ENDSTOPPULLUP_XMIN);
  endstops.setPullup(Y_MIN, ENDSTOPPULLUP_YMIN);
  endstops.setPullup(Z_MIN, ENDSTOPPULLUP_ZMIN);
  endstops.setPullup(X_MAX, ENDSTOPPULLUP_XMAX);
  endstops.setPullup(Y_MAX, ENDSTOPPULLUP_YMAX);
  endstops.setPullup(Z_MAX, ENDSTOPPULLUP_ZMAX);
  endstops.setPullup(X2_MIN, ENDSTOPPULLUP_X2MIN);
  endstops.setPullup(Y2_MIN, ENDSTOPPULLUP_Y2MIN);
  endstops.setPullup(Z2_MIN, ENDSTOPPULLUP_Z2MIN);
  endstops.setPullup(X2_MAX, ENDSTOPPULLUP_X2MAX);
  endstops.setPullup(Y2_MAX, ENDSTOPPULLUP_Y2MAX);
  endstops.setPullup(Z2_MAX, ENDSTOPPULLUP_Z2MAX);
  endstops.setPullup(Z_PROBE, ENDSTOPPULLUP_ZPROBE);
  endstops.setPullup(FIL_RUNOUT, PULLUP_FIL_RUNOUT);
  endstops.setPullup(DOOR_OPEN, PULLUP_DOOR_OPEN);
  endstops.setPullup(POWER_CHECK, PULLUP_POWER_CHECK);

  Postprocess();

  SERIAL_LM(ECHO, "Hardcoded Default Settings Loaded");
}

#if DISABLED(DISABLE_M503)

  #define CONFIG_MSG_START(str) do{ if (!forReplay) SERIAL_STR(CFG); SERIAL_EM(str); }while(0)

  /**
   * M503 - Print Configuration
   */
  void EEPROM::Print_Settings(const bool forReplay) {
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

    CONFIG_MSG_START("Advanced variables: S<min_feedrate> V<min_travel_feedrate> B<min_segment_time_us> X<max_xy_jerk> Z<max_z_jerk> T* E<max_e_jerk>");
    SERIAL_SMV(CFG, "  M205 S", LINEAR_UNIT(mechanics.min_feedrate_mm_s), 3);
    SERIAL_MV(" V", LINEAR_UNIT(mechanics.min_travel_feedrate_mm_s), 3);
    SERIAL_MV(" B", mechanics.min_segment_time_us);
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

    #if HOTENDS > 0
      CONFIG_MSG_START("Hotend Sensor parameters: H<Hotend> P<Pin> A<R25> B<BetaK> C<Steinhart-Hart C> R<Pullup> L<ADC low offset> O<ADC high offset>");
      LOOP_HOTEND() {
        SERIAL_SMV(CFG, "  M305 H", h);
        SERIAL_MV(" P", heaters[h].sensor.pin);
        SERIAL_MV(" A", heaters[h].sensor.r25, 1);
        SERIAL_MV(" B", heaters[h].sensor.beta, 1);
        SERIAL_MV(" C", heaters[h].sensor.shC, 10);
        SERIAL_MV(" R", heaters[h].sensor.pullupR, 1);
        SERIAL_MV(" L", heaters[h].sensor.adcLowOffset);
        SERIAL_EMV(" O", heaters[h].sensor.adcHighOffset);
      }

      CONFIG_MSG_START("Hotend Heater parameters: H<Hotend> P<Pin> A<Pid Drive Min> B<Pid Drive Max> C<Pid Max> L<Min Temp> O<Max Temp> U<Use Pid 0-1> I<Hardware Inverted 0-1>");
      LOOP_HOTEND() {
        SERIAL_SMV(CFG, "  M306 H", h);
        SERIAL_MV(" P", heaters[h].pin);
        SERIAL_MV(" A", heaters[h].pidDriveMin);
        SERIAL_MV(" B", heaters[h].pidDriveMax);
        SERIAL_MV(" C", heaters[h].pidMax);
        SERIAL_MV(" L", heaters[h].mintemp);
        SERIAL_MV(" O", heaters[h].maxtemp);
        SERIAL_MV(" U", heaters[h].isUsePid());
        SERIAL_EMV(" I", heaters[h].isHWInverted());
      }
    #endif

    #if HAS_TEMP_BED
      CONFIG_MSG_START("Bed Sensor parameters: P<Pin> A<R25> B<BetaK> C<Steinhart-Hart C> R<Pullup> L<ADC low offset> O<ADC high offset>");
      SERIAL_SM(CFG, "  M305 H-1");
      SERIAL_MV(" P", heaters[BED_INDEX].sensor.pin);
      SERIAL_MV(" A", heaters[BED_INDEX].sensor.r25, 1);
      SERIAL_MV(" B", heaters[BED_INDEX].sensor.beta, 1);
      SERIAL_MV(" C", heaters[BED_INDEX].sensor.shC, 10);
      SERIAL_MV(" R", heaters[BED_INDEX].sensor.pullupR, 1);
      SERIAL_MV(" L", heaters[BED_INDEX].sensor.adcLowOffset);
      SERIAL_EMV(" O", heaters[BED_INDEX].sensor.adcHighOffset);

      CONFIG_MSG_START("Bed Heater parameters: P<Pin> A<Pid Drive Min> B<Pid Drive Max> C<Pid Max> L<Min Temp> O<Max Temp> U<Use Pid 0-1> I<Hardware Inverted 0-1>");
      LOOP_HOTEND() {
        SERIAL_SM(CFG, "  M306 H-1");
        SERIAL_MV(" P", heaters[BED_INDEX].pin);
        SERIAL_MV(" A", heaters[BED_INDEX].pidDriveMin);
        SERIAL_MV(" B", heaters[BED_INDEX].pidDriveMax);
        SERIAL_MV(" C", heaters[BED_INDEX].pidMax);
        SERIAL_MV(" L", heaters[BED_INDEX].mintemp);
        SERIAL_MV(" O", heaters[BED_INDEX].maxtemp);
        SERIAL_MV(" U", heaters[BED_INDEX].isUsePid());
        SERIAL_EMV(" I", heaters[BED_INDEX].isHWInverted());
      }
    #endif

    CONFIG_MSG_START("PID settings:");
    #if HOTENDS == 1
      heaters[0].print_PID();
    #elif HOTENDS > 1
      LOOP_HOTEND() heaters[h].print_PID();
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        SERIAL_LMV(CFG, "  M301 L", tools.lpq_len);
      #endif
    #endif
    #if (HAS_HEATER_BED)
      heaters[BED_INDEX].print_PID();
    #endif
    #if (HAS_HEATER_CHAMBER)
      heaters[CHAMBER_INDEX].print_PID();
    #endif
    #if (HAS_HEATER_COOLER)
      heaters[COOLER_INDEX].print_PID();
    #endif

    #if HEATER_USES_AD595
      CONFIG_MSG_START("AD595 Offset and Gain:");
      LOOP_HOTEND() {
        SERIAL_SMV(CFG, "  M595 H", h);
        SERIAL_MV(" O", heaters[h].sensor.ad595_offset);
        SERIAL_EMV(", S", heaters[h].sensor.ad595_gain);
      }
    #endif // HEATER_USES_AD595

    #if HOTENDS > 1
      CONFIG_MSG_START("Hotend offset (mm):");
      for (int8_t h = 1; h < HOTENDS; h++) {
        SERIAL_SMV(CFG, "  M218 H", h);
        SERIAL_MV(" X", LINEAR_UNIT(tools.hotend_offset[X_AXIS][h]), 3);
        SERIAL_MV(" Y", LINEAR_UNIT(tools.hotend_offset[Y_AXIS][h]), 3);
        SERIAL_EMV(" Z", LINEAR_UNIT(tools.hotend_offset[Z_AXIS][h]), 3);
      }
    #endif

    #if FAN_COUNT > 0
      CONFIG_MSG_START("Fans: P<Fan> U<Pin> L<Min Speed> F<Freq> H<Auto mode> I<Hardware Inverted 0-1>");
      LOOP_FAN() {
        SERIAL_SMV(CFG, "  M106 P", f);
        SERIAL_MV(" U", fans[f].pin);
        SERIAL_MV(" L", fans[f].min_Speed);
        SERIAL_MV(" F", fans[f].freq);
        LOOP_HOTEND() {
          if (TEST(fans[f].autoMonitored, h)) SERIAL_MV(" H", (int)h);
        }
        if (TEST(fans[f].autoMonitored, 7)) SERIAL_MSG(" H7");
        SERIAL_EMV(" I", fans[f].isHWInverted());
      }
    #endif

    #if ENABLED(WORKSPACE_OFFSETS)
      CONFIG_MSG_START("Home offset:");
      SERIAL_SMV(CFG, "  M206 X", LINEAR_UNIT(mechanics.home_offset[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(mechanics.home_offset[Y_AXIS]), 3);
      SERIAL_EMV(" Z", LINEAR_UNIT(mechanics.home_offset[Z_AXIS]), 3);
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

    #if HAS_LCD_CONTRAST
      CONFIG_MSG_START("LCD Contrast:");
      SERIAL_LMV(CFG, "  M250 C", lcd_contrast);
    #endif

    /**
     * Bed Leveling
     */
    #if HAS_LEVELING

      #if ENABLED(MESH_BED_LEVELING)
        CONFIG_MSG_START("Mesh Bed Leveling:");
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        CONFIG_MSG_START("Unified Bed Leveling:");
      #elif HAS_ABL
        CONFIG_MSG_START("Auto Bed Leveling:");
      #endif

      SERIAL_SMV(CFG, "  M420 S", bedlevel.leveling_is_valid() ? 1 : 0);
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        SERIAL_MV(" Z", LINEAR_UNIT(bedlevel.z_fade_height));
      #endif
      SERIAL_EOL();

      #if ENABLED(MESH_BED_LEVELING)

        if (bedlevel.leveling_is_valid()) {
          for (uint8_t py = 0; py < GRID_MAX_POINTS_Y; py++) {
            for (uint8_t px = 0; px < GRID_MAX_POINTS_X; px++) {
              SERIAL_SMV(CFG, "  G29 S3 X", (int)px + 1);
              SERIAL_MV(" Y", (int)py + 1);
              SERIAL_EMV(" Z", LINEAR_UNIT(mbl.z_values[px][py]), 5);
            }
          }
        }

      #elif ENABLED(AUTO_BED_LEVELING_UBL)

        ubl.report_state();
        SERIAL_LMV(CFG, "  Active Mesh Slot: ", ubl.storage_slot);
        SERIAL_SMV(CFG, "  EEPROM can hold ", calc_num_meshes());
        SERIAL_EM(" meshes.");
        ubl.report_current_mesh();

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        if (bedlevel.leveling_is_valid()) {
          for (uint8_t py = 0; py < GRID_MAX_POINTS_Y; py++) {
            for (uint8_t px = 0; px < GRID_MAX_POINTS_X; px++) {
              SERIAL_SMV(CFG, "  G29 W I", (int)px + 1);
              SERIAL_MV(" J", (int)py + 1);
              SERIAL_MV(" Z", LINEAR_UNIT(abl.z_values[px][py]), 5);
              SERIAL_EOL();
            }
          }
        }

      #endif

    #endif // HAS_LEVELING

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

    #endif // IS_DELTA

    #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)

      CONFIG_MSG_START("Endstop adjustment:");
      SERIAL_SM(CFG, "  M666");
      #if ENABLED(X_TWO_ENDSTOPS)
        SERIAL_MV(" X", LINEAR_UNIT(endstops.x_endstop_adj));
      #endif
      #if ENABLED(Y_TWO_ENDSTOPS)
        SERIAL_MV(" Y", LINEAR_UNIT(endstops.y_endstop_adj));
      #endif
      #if ENABLED(Z_TWO_ENDSTOPS)
        SERIAL_MV(" Z", LINEAR_UNIT(endstops.z_endstop_adj));
      #endif
      SERIAL_EOL();

    #endif // [XYZ]_TWO_ENDSTOPS

    /**
     * Auto Bed Leveling
     */
    #if HAS_BED_PROBE
      CONFIG_MSG_START("Probe Offset:");
      SERIAL_SMV(CFG, "  M851 X", LINEAR_UNIT(probe.offset[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(probe.offset[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(probe.offset[Z_AXIS]), 3);
      SERIAL_EOL();
    #endif

    #if ENABLED(ULTIPANEL)
      CONFIG_MSG_START("Material heatup parameters:");
      for (uint8_t i = 0; i < COUNT(lcd_preheat_hotend_temp); i++) {
        SERIAL_SMV(CFG, "  M145 S", i);
        SERIAL_MV(" H", TEMP_UNIT(lcd_preheat_hotend_temp[i]));
        SERIAL_MV(" B", TEMP_UNIT(lcd_preheat_bed_temp[i]));
        SERIAL_MV(" F", lcd_preheat_fan_speed[i]);
        SERIAL_EOL();
      }
    #endif // ULTIPANEL

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

    #if ENABLED(VOLUMETRIC_EXTRUSION)

      /**
       * Volumetric extrusion M200
       */
      if (!forReplay) {
        SERIAL_SM(CFG, "Filament settings:");
        if (printer.isVolumetric())
          SERIAL_EOL();
        else
          SERIAL_EM(" Disabled");
      }
      #if EXTRUDERS == 1
        SERIAL_LMV(CFG, "  M200 T0 D", tools.filament_size[0], 3);
      #elif EXTRUDERS > 1
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_SMV(CFG, "  M200 T", (int)i);
          SERIAL_EMV(" D", tools.filament_size[i], 3);
        }
      #endif

    #endif // ENABLED(VOLUMETRIC_EXTRUSION)

    /**
     * Alligator current drivers M906
     */
    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      CONFIG_MSG_START("Motor current:");
      SERIAL_SMV(CFG, "  M906 X", externaldac.motor_current[X_AXIS], 2);
      SERIAL_MV(" Y", externaldac.motor_current[Y_AXIS], 2);
      SERIAL_MV(" Z", externaldac.motor_current[Z_AXIS], 2);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", externaldac.motor_current[E_AXIS], 2);
      #endif
      SERIAL_EOL();
      #if DRIVER_EXTRUDERS > 1
        for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
          SERIAL_SMV(CFG, "  M906 T", i);
          SERIAL_EMV(" E", externaldac.motor_current[E_AXIS + i], 2);
        }
      #endif // DRIVER_EXTRUDERS > 1
    #endif // ALLIGATOR

    /**
     * TMC2130 or TMC2208 stepper driver current
     */
    #if HAS_TRINAMIC
      CONFIG_MSG_START("Stepper driver current:");
      SERIAL_SM(CFG, "  M906");
      #if ENABLED(X_IS_TMC2130) || ENABLED(X_IS_TMC2208)
        SERIAL_MV(" X", stepperX.getCurrent());
      #endif
      #if ENABLED(Y_IS_TMC2130) || ENABLED(Y_IS_TMC2208)
        SERIAL_MV(" Y", stepperY.getCurrent());
      #endif
      #if ENABLED(Z_IS_TMC2130) || ENABLED(Z_IS_TMC2208)
        SERIAL_MV(" Z", stepperZ.getCurrent());
      #endif
      #if ENABLED(X2_IS_TMC2130) || ENABLED(X2_IS_TMC2208)
        SERIAL_MV(" X2", stepperX2.getCurrent());
      #endif
      #if ENABLED(Y2_IS_TMC2130) || ENABLED(Y2_IS_TMC2208)
        SERIAL_MV(" Y2", stepperY2.getCurrent());
      #endif
      #if ENABLED(Z2_IS_TMC2130) || ENABLED(Z2_IS_TMC2208)
        SERIAL_MV(" Z2", stepperZ2.getCurrent());
      #endif
      #if ENABLED(E0_IS_TMC2130) || ENABLED(E0_IS_TMC2208)
        SERIAL_MV(" E0", stepperE0.getCurrent());
      #endif
      #if ENABLED(E1_IS_TMC2130) || ENABLED(E1_IS_TMC2208)
        SERIAL_MV(" E1", stepperE1.getCurrent());
      #endif
      #if ENABLED(E2_IS_TMC2130) || ENABLED(E2_IS_TMC2208)
        SERIAL_MV(" E2", stepperE2.getCurrent());
      #endif
      #if ENABLED(E3_IS_TMC2130) || ENABLED(E3_IS_TMC2208)
        SERIAL_MV(" E3", stepperE3.getCurrent());
      #endif
      #if ENABLED(E4_IS_TMC2130) || ENABLED(E4_IS_TMC2208)
        SERIAL_MV(" E4", stepperE4.getCurrent());
      #endif
      #if ENABLED(E5_IS_TMC2130) || ENABLED(E5_IS_TMC2208)
        SERIAL_MV(" E5", stepperE5.getCurrent());
      #endif
      SERIAL_EOL();
    #endif

    /**
     * TMC2130 Sensorless homing thresholds
     */
    #if ENABLED(HAVE_TMC2130) && ENABLED(SENSORLESS_HOMING)
      CONFIG_MSG_START("Sensorless homing threshold:");
      SERIAL_SM(CFG, "  M914");
      #if ENABLED(X_HOMING_SENSITIVITY)
        #if ENABLED(X_IS_TMC2130)
          SERIAL_MV(" X", stepperX.sgt());
        #endif
        #if ENABLED(X2_IS_TMC2130)
          SERIAL_MV(" X2 ", stepperX2.sgt());
        #endif
      #endif
      #if ENABLED(Y_HOMING_SENSITIVITY)
        #if ENABLED(Y_IS_TMC2130)
          SERIAL_MV(" Y", stepperY.sgt());
        #endif
        #if ENABLED(X2_IS_TMC2130)
          SERIAL_MV(" Y2 ", stepperY2.sgt());
        #endif
      #endif
      #if ENABLED(Z_HOMING_SENSITIVITY)
        #if ENABLED(Z_IS_TMC2130)
          SERIAL_MV(" Z ", stepperZ.sgt());
        #endif
        #if ENABLED(Z2_IS_TMC2130)
          SERIAL_MV(" Z2 ", stepperZ2.sgt());
        #endif
      #endif
      SERIAL_EOL();
    #endif

    /**
     * Linear Advance
     */
    #if ENABLED(LIN_ADVANCE)
      CONFIG_MSG_START("Linear Advance:");
      SERIAL_LMV(CFG, "  M900 K", planner.extruder_advance_K);
    #endif

    /**
     * Advanced Pause filament load & unload lengths
     */
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      CONFIG_MSG_START("Filament load/unload lengths:");
      #if EXTRUDERS == 1
        SERIAL_SMV(CFG, "  M603 L", LINEAR_UNIT(filament_change_load_length[0]), 2);
        SERIAL_EMV(" U", LINEAR_UNIT(filament_change_unload_length[0]), 2);
      #else // EXTRUDERS != 1
        for (uint8_t e = 0; e < EXTRUDERS; e++) {
          SERIAL_SMV(CFG, "  M603 T", (int)e);
          SERIAL_MV(" L", LINEAR_UNIT(filament_change_load_length[e]), 2);
          SERIAL_EMV(" U", LINEAR_UNIT(filament_change_unload_length[e]), 2);
        }
      #endif // EXTRUDERS != 1
    #endif // ADVANCED_PAUSE_FEATURE

    #if HAS_SDSUPPORT
      card.PrintSettings();
    #endif

  }

#endif // !DISABLE_M503

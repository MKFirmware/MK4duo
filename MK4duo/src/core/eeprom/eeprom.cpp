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

#define EEPROM_VERSION "MKV51"

/**
 * MKV51 EEPROM Layout:
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
 *
 * Junction Deviation:
 *  M205  J               mechanics.junction_deviation_mm       (float)
 * or Jerk:
 *  M205  X               mechanics.max_jerk[X_AXIS]            (float)
 *  M205  Y               mechanics.max_jerk[Y_AXIS]            (float)
 *  M205  Z               mechanics.max_jerk[Z_AXIS]            (float)
 *  M205  E   E0 ...      mechanics.max_jerk[E_AXIS * EXTRUDERS](float x6)
 *
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
 * SERVO ANGLES:
 *  M281  P   LU          servo.angle                                   (int x2)
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
 * Stepper driver control
 *  M569  XYZ T0-5 E      stepper.direction_flag                (uint16_t)
 *  M569  D               stepper.direction_delay               (uint32_t)
 *  M569  P               stepper.minimum_pulse                 (uint8_t)
 *  M569  R               stepper.maximum_rate                  (uint32_t)
 *
 * ALLIGATOR:
 *  M906  XYZ T0-4 E      Motor current                         (uint16_t x7)
 *
 * TRINAMIC:
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
 * HYSTERESIS FEATURE:
 *  M99   XYZ             planner.hysteresis_mm                 (float x3)
 *  M99   F               planner.hysteresis_correction         (float)
 *
 * ADVANCED_PAUSE_FEATURE:
 *  M603  U               filament_change_unload_length         (float)
 *  M603  L               filament_change_load_length           (float)
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

#if HAS_TRINAMIC
  #define TMC_GET_PWMTHRS(P,Q) _tmc_thrs(stepper##Q.microsteps(), stepper##Q.TPWMTHRS(), mechanics.axis_steps_per_mm[P##_AXIS])
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

  // Recalculate pulse cycle
  HAL_calc_pulse_cycle();

  // steps per s2 needs to be updated to agree with units per s2
  planner.reset_acceleration_rates();

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
    LOOP_FAN() {
      fans[f].init();
      #if ENABLED(TACHOMETRIC)
        tachometrics[f].init(f);
      #endif
    }
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

  #if ENABLED(JUNCTION_DEVIATION) && ENABLED(LIN_ADVANCE)
    mechanics.recalculate_max_e_jerk();
  #endif

  // Setup Endstops pullup
  endstops.setup_pullup();

  // Refresh steps_to_mm with the reciprocal of axis_steps_per_mm
  // and init stepper.count[], planner.position[] with current_position
  planner.refresh_positioning();

  if (memcmp(oldpos, mechanics.current_position, sizeof(oldpos)))
    mechanics.report_current_position();

}

#if HAS_EEPROM

  #define EEPROM_READ_START()   int eeprom_index = EEPROM_OFFSET; eeprom_error = memorystore.access_start(true)
  #define EEPROM_WRITE_START()  int eeprom_index = EEPROM_OFFSET; eeprom_error = memorystore.access_start(false)
  #define EEPROM_READ_FINISH()  eeprom_error = memorystore.access_finish(true)
  #define EEPROM_WRITE_FINISH() eeprom_error = memorystore.access_finish(false)
  #define EEPROM_SKIP(VAR)      eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR)     memorystore.write_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ(VAR)      memorystore.read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)

  const char version[6] = EEPROM_VERSION;

  bool EEPROM::eeprom_error = false;

  #if ENABLED(AUTO_BED_LEVELING_UBL)
    uint16_t EEPROM::meshes_begin = 0;
  #endif

  /**
   * M500 - Store Configuration
   */
  bool EEPROM::Store_Settings() {
    float dummy = 0.0f;
    char ver[6] = "ERROR";

    uint16_t working_crc = 0;

    EEPROM_WRITE_START();

    #if HAS_EEPROM_FLASH
      EEPROM_SKIP(ver);       // Flash doesn't allow rewriting without erase
    #else
      EEPROM_WRITE(ver);      // invalidate data first
    #endif
    EEPROM_SKIP(working_crc); // Skip the checksum slot

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

    #if ENABLED(JUNCTION_DEVIATION)
      EEPROM_WRITE(mechanics.junction_deviation_mm);
    #else
      EEPROM_WRITE(mechanics.max_jerk);
    #endif

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
        #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
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
        EEPROM_WRITE(fans[f].ID);
        EEPROM_WRITE(fans[f].freq);
        EEPROM_WRITE(fans[f].min_Speed);
        EEPROM_WRITE(fans[f].autoMonitored);
        EEPROM_WRITE(fans[f].FanFlag);
        #if ENABLED(TACHOMETRIC)
          EEPROM_WRITE(tachometrics[f].pin);
        #endif
      }
    #endif

    #if HAS_LCD_CONTRAST
      EEPROM_WRITE(lcd_contrast);
    #endif

    #if HAS_SERVOS
      LOOP_SERVO() EEPROM_WRITE(servo[s].angle);
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

    EEPROM_WRITE(stepper.direction_flag);
    EEPROM_WRITE(stepper.direction_delay);
    EEPROM_WRITE(stepper.minimum_pulse);
    EEPROM_WRITE(stepper.maximum_rate);

    #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
      EEPROM_WRITE(externaldac.motor_current);
    #endif

    //
    // Save TMC2130 or TMC2208 Configuration, and placeholder values
    //
    #if HAS_TRINAMIC

      uint16_t tmc_stepper_current[TMC_AXES] = {
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
      EEPROM_WRITE(tmc_stepper_current);

      //
      // Save TMC2130 or TMC2208 Hybrid Threshold, and placeholder values
      //
      uint32_t tmc_hybrid_threshold[TMC_AXES] = {
        #if ENABLED(HYBRID_THRESHOLD)
          #if X_IS_TRINAMIC
            TMC_GET_PWMTHRS(X, X),
          #else
            X_HYBRID_THRESHOLD,
          #endif
          #if Y_IS_TRINAMIC
            TMC_GET_PWMTHRS(Y, Y),
          #else
            Y_HYBRID_THRESHOLD,
          #endif
          #if Z_IS_TRINAMIC
            TMC_GET_PWMTHRS(Z, Z),
          #else
            Z_HYBRID_THRESHOLD,
          #endif
          #if X2_IS_TRINAMIC
            TMC_GET_PWMTHRS(X, X2),
          #else
            X2_HYBRID_THRESHOLD,
          #endif
          #if Y2_IS_TRINAMIC
            TMC_GET_PWMTHRS(Y, Y2),
          #else
            Y2_HYBRID_THRESHOLD,
          #endif
          #if Z2_IS_TRINAMIC
            TMC_GET_PWMTHRS(Z, Z2),
          #else
            Z2_HYBRID_THRESHOLD,
          #endif
          #if E0_IS_TRINAMIC
            TMC_GET_PWMTHRS(E, E0),
          #else
            E0_HYBRID_THRESHOLD,
          #endif
          #if E1_IS_TRINAMIC
            TMC_GET_PWMTHRS(E, E1),
          #else
            E1_HYBRID_THRESHOLD,
          #endif
          #if E2_IS_TRINAMIC
            TMC_GET_PWMTHRS(E, E2),
          #else
            E2_HYBRID_THRESHOLD,
          #endif
          #if E3_IS_TRINAMIC
            TMC_GET_PWMTHRS(E, E3),
          #else
            E3_HYBRID_THRESHOLD,
          #endif
          #if E4_IS_TRINAMIC
            TMC_GET_PWMTHRS(E, E4),
          #else
            E4_HYBRID_THRESHOLD,
          #endif
          #if E5_IS_TRINAMIC
            TMC_GET_PWMTHRS(E, E5)
          #else
            E5_HYBRID_THRESHOLD
          #endif
        #else // !HYBRID_THRESHOLD
          100, 100, 3,            // X, Y, Z
          100, 100, 3,            // X2, Y2, Z2
          30, 30, 30, 30, 30, 30  // E0, E1, E2, E3, E4, E5
        #endif // |HYBRID_THRESHOLD
      };
      EEPROM_WRITE(tmc_hybrid_threshold);

      //
      // TMC2130 Sensorless homing threshold
      //
      int16_t tmc_sgt[XYZ] = {
        #if ENABLED(SENSORLESS_HOMING)
          #if X_HAS_DRV(TMC2130) && ENABLED(X_HOMING_SENSITIVITY)
            stepperX.sgt(),
          #else
            0,
          #endif
          #if Y_HAS_DRV(TMC2130) && ENABLED(Y_HOMING_SENSITIVITY)
            stepperY.sgt(),
          #else
            0,
          #endif
          #if Z_HAS_DRV(TMC2130) && ENABLED(Z_HOMING_SENSITIVITY)
            stepperZ.sgt()
          #else
            0
          #endif
        #else
          0
        #endif
      };
      EEPROM_WRITE(tmc_sgt);

    #endif // HAS_TRINAMIC

    //
    // Linear Advance
    //
    #if ENABLED(LIN_ADVANCE)
      EEPROM_WRITE(planner.extruder_advance_K);
    #endif

    //
    // Hysteresis Feature
    //
    #if ENABLED(HYSTERESIS_FEATURE)
      EEPROM_WRITE(planner.hysteresis_mm);
      EEPROM_WRITE(planner.hysteresis_correction);
    #endif

    //
    // Advanced Pause
    //
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      EEPROM_WRITE(filament_change_unload_length);
      EEPROM_WRITE(filament_change_load_length);
    #endif

    //
    // Validate CRC and Data Size
    //
    if (!eeprom_error) {
      const uint16_t  eeprom_size = eeprom_index - (EEPROM_OFFSET),
                      final_crc = working_crc;

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

      //
      // UBL Mesh
      //
      #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(UBL_SAVE_ACTIVE_ON_M500)
        if (ubl.storage_slot >= 0)
          store_mesh(ubl.storage_slot);
      #endif

      EEPROM_WRITE_FINISH();

    }

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

    EEPROM_READ(stored_ver);
    EEPROM_READ(stored_crc);

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

      #if ENABLED(JUNCTION_DEVIATION)
        EEPROM_READ(mechanics.junction_deviation_mm);
      #else
        EEPROM_READ(mechanics.max_jerk);
      #endif

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
          #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
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
          EEPROM_READ(fans[f].ID);
          EEPROM_READ(fans[f].freq);
          EEPROM_READ(fans[f].min_Speed);
          EEPROM_READ(fans[f].autoMonitored);
          EEPROM_READ(fans[f].FanFlag);
          #if ENABLED(TACHOMETRIC)
            EEPROM_READ(tachometrics[f].pin);
          #endif
        }
      #endif

      #if HAS_LCD_CONTRAST
        EEPROM_READ(lcd_contrast);
      #endif

      #if HAS_SERVOS
        LOOP_SERVO() EEPROM_READ(servo[s].angle);
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

      EEPROM_READ(stepper.direction_flag);
      EEPROM_READ(stepper.direction_delay);
      EEPROM_READ(stepper.minimum_pulse);
      EEPROM_READ(stepper.maximum_rate);

      #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
        EEPROM_READ(externaldac.motor_current);
      #endif

      //
      // TMC2130 or TMC2208 Stepper Current
      //
      #if HAS_TRINAMIC

        #define SET_CURR(Q) stepper##Q.setCurrent(currents[TMC_##Q] ? currents[TMC_##Q] : Q##_CURRENT, R_SENSE, HOLD_MULTIPLIER)
        uint16_t currents[TMC_AXES];
        EEPROM_READ(currents);
        #if X_IS_TRINAMIC
          SET_CURR(X);
        #endif
        #if Y_IS_TRINAMIC
          SET_CURR(Y);
        #endif
        #if Z_IS_TRINAMIC
          SET_CURR(Z);
        #endif
        #if X2_IS_TRINAMIC
          SET_CURR(X2);
        #endif
        #if Y2_IS_TRINAMIC
          SET_CURR(Y2);
        #endif
        #if Z2_IS_TRINAMIC
          SET_CURR(Z2);
        #endif
        #if E0_IS_TRINAMIC
          SET_CURR(E0);
        #endif
        #if E1_IS_TRINAMIC
          SET_CURR(E1);
        #endif
        #if E2_IS_TRINAMIC
          SET_CURR(E2);
        #endif
        #if E3_IS_TRINAMIC
          SET_CURR(E3);
        #endif
        #if E4_IS_TRINAMIC
          SET_CURR(E4);
        #endif
        #if E5_IS_TRINAMIC
          SET_CURR(E5);
        #endif

        #define TMC_SET_PWMTHRS(P,Q) tmc_set_pwmthrs(stepper##Q, tmc_hybrid_threshold[TMC_##Q], mechanics.axis_steps_per_mm[P##_AXIS])
        uint32_t tmc_hybrid_threshold[TMC_AXES];
        EEPROM_READ(tmc_hybrid_threshold);
        #if ENABLED(HYBRID_THRESHOLD)
          #if X_IS_TRINAMIC
            TMC_SET_PWMTHRS(X, X);
          #endif
          #if Y_IS_TRINAMIC
            TMC_SET_PWMTHRS(Y, Y);
          #endif
          #if Z_IS_TRINAMIC
            TMC_SET_PWMTHRS(Z, Z);
          #endif
          #if X2_IS_TRINAMIC
            TMC_SET_PWMTHRS(X, X2);
          #endif
          #if Y2_IS_TRINAMIC
            TMC_SET_PWMTHRS(Y, Y2);
          #endif
          #if Z2_IS_TRINAMIC
            TMC_SET_PWMTHRS(Z, Z2);
          #endif
          #if E0_IS_TRINAMIC
            TMC_SET_PWMTHRS(E, E0);
          #endif
          #if E1_IS_TRINAMIC
            TMC_SET_PWMTHRS(E, E1);
          #endif
          #if E2_IS_TRINAMIC
            TMC_SET_PWMTHRS(E, E2);
          #endif
          #if E3_IS_TRINAMIC
            TMC_SET_PWMTHRS(E, E3);
          #endif
          #if E4_IS_TRINAMIC
            TMC_SET_PWMTHRS(E, E4);
          #endif
          #if E4_IS_TRINAMIC
            TMC_SET_PWMTHRS(E, E5);
          #endif
        #endif

        /*
         * TMC2130 Sensorless homing threshold.
         * X and X2 use the same value
         * Y and Y2 use the same value
         * Z and Z2 use the same value
         */
        int16_t tmc_sgt[XYZ];
        EEPROM_READ(tmc_sgt);
        #if ENABLED(SENSORLESS_HOMING)
          #if ENABLED(X_HOMING_SENSITIVITY)
            #if X_HAS_DRV(TMC2130) || ENABLED(IS_TRAMS)
              stepperX.sgt(tmc_sgt[0]);
            #endif
            #if X2_HAS_DRV(TMC2130)
              stepperX2.sgt(tmc_sgt[0]);
            #endif
          #endif
          #if ENABLED(Y_HOMING_SENSITIVITY)
            #if Y_HAS_DRV(TMC2130) || ENABLED(IS_TRAMS)
              stepperY.sgt(tmc_sgt[1]);
            #endif
            #if Y2_HAS_DRV(TMC2130)
              stepperY2.sgt(tmc_sgt[1]);
            #endif
          #endif
          #if ENABLED(Z_HOMING_SENSITIVITY)
            #if Z_HAS_DRV(TMC2130) || ENABLED(IS_TRAMS)
              stepperZ.sgt(tmc_sgt[2]);
            #endif
            #if Z2_HAS_DRV(TMC2130)
              stepperZ2.sgt(tmc_sgt[2]);
            #endif
          #endif
        #endif
        
      #endif // HAS_TRINAMIC

      //
      // Linear Advance
      //
      #if ENABLED(LIN_ADVANCE)
        EEPROM_READ(planner.extruder_advance_K);
      #endif

      //
      // Hysteresis Feature
      //
      #if ENABLED(HYSTERESIS_FEATURE)
        EEPROM_READ(planner.hysteresis_mm);
        EEPROM_READ(planner.hysteresis_correction);
      #endif

      //
      // Advanced Pause
      //
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        EEPROM_READ(filament_change_unload_length);
        EEPROM_READ(filament_change_load_length);
      #endif

      if (working_crc == stored_crc) {
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_ST(ECHO, version);
          SERIAL_MV(" Stored settings retrieved (", eeprom_index - (EEPROM_OFFSET));
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
          eeprom_error = true;
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

      EEPROM_READ_FINISH();
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

    const uint16_t EEPROM::meshes_end = memorystore.capacity() - 129;

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
            SERIAL_MV("E2END=", (int)(memorystore.capacity() - 1));
            SERIAL_MV(" meshes_end=", (int)meshes_end);
            SERIAL_EMV(" slot=", slot);
          #endif
          return;
        }

        uint16_t crc = 0;
        int pos = mesh_slot_offset(slot);

        const bool status = memorystore.write_data(pos, (uint8_t *)&ubl.z_values, sizeof(ubl.z_values), &crc);

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

        int pos = mesh_slot_offset(slot);
        uint16_t crc = 0;
        uint8_t * const dest = into ? (uint8_t*)into : (uint8_t*)&ubl.z_values;

        memorystore.access_start(true);
        const bool status = memorystore.read_data(pos, dest, sizeof(ubl.z_values), &crc);
        memorystore.access_finish(true);

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

  static const float    tmp1[] PROGMEM  = DEFAULT_Kp,
                        tmp2[] PROGMEM  = DEFAULT_Ki,
                        tmp3[] PROGMEM  = DEFAULT_Kd,
                        tmp4[] PROGMEM  = DEFAULT_Kc;

  #if FAN_COUNT > 0
    static const pin_t  tmp5[] PROGMEM = FANS_CHANNELS;
    static const int8_t tmp6[] PROGMEM = AUTO_FAN;
  #endif

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    new_z_fade_height = 0.0f;
  #endif

  #if ENABLED(HOTEND_OFFSET_X) && ENABLED(HOTEND_OFFSET_Y) && ENABLED(HOTEND_OFFSET_Z)
    constexpr float tmp7[XYZ][4] = {
      HOTEND_OFFSET_X,
      HOTEND_OFFSET_Y,
      HOTEND_OFFSET_Z
    };
  #else
    constexpr float tmp7[XYZ][HOTENDS] = { 0.0f };
  #endif

  static_assert(
    tmp7[X_AXIS][0] == 0 && tmp7[Y_AXIS][0] == 0 && tmp7[Z_AXIS][0] == 0,
    "Offsets for the first hotend must be 0.0."
  );
  LOOP_XYZ(i) {
    LOOP_HOTEND() tools.hotend_offset[i][h] = tmp7[i][h];
  }

  #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
    constexpr uint16_t tmp8[] = { X_CURRENT, Y_CURRENT, Z_CURRENT, E0_CURRENT, E1_CURRENT, E2_CURRENT, E3_CURRENT };
    for (uint8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++)
      externaldac.motor_current[i] = tmp8[i < COUNT(tmp8) ? i : COUNT(tmp8) - 1];
  #endif

  // Call Mechanic Factory parameters
  mechanics.factory_parameters();

  // Call Stepper Factory parameters
  stepper.factory_parameters();

  // Call Endstop Factory parameters
  endstops.factory_parameters();

  // Reset Printer Flag
  printer.reset_flag();

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    bedlevel.z_fade_height = 0.0f;
  #endif

  #if HAS_LEVELING
    bedlevel.reset();
  #endif

  #if HAS_BED_PROBE
    probe.offset[X_AXIS] = X_PROBE_OFFSET_FROM_NOZZLE;
    probe.offset[Y_AXIS] = Y_PROBE_OFFSET_FROM_NOZZLE;
    probe.offset[Z_AXIS] = Z_PROBE_OFFSET_FROM_NOZZLE;
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

  #if HAS_LCD_CONTRAST
    lcd_contrast = DEFAULT_LCD_CONTRAST;
  #endif

  #if HAS_SERVOS

    #if HAS_DONDOLO
      constexpr int16_t angles[] = { DONDOLO_SERVOPOS_E0, DONDOLO_SERVOPOS_E1 };
      servo[DONDOLO_SERVO_INDEX].angle[0] = angles[0];
      servo[DONDOLO_SERVO_INDEX].angle[0] = angles[1];
    #endif

    #if HAS_Z_SERVO_PROBE
      constexpr uint8_t z_probe_angles[2] = Z_SERVO_ANGLES;
      servo[Z_PROBE_SERVO_NR].angle[0] = z_probe_angles[0];
      servo[Z_PROBE_SERVO_NR].angle[1] = z_probe_angles[1];
    #endif

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
        heat->Kp  = pgm_read_float(&tmp1[h < COUNT(tmp1) ? h : COUNT(tmp1) - 1]);
        heat->Ki  = pgm_read_float(&tmp2[h < COUNT(tmp2) ? h : COUNT(tmp2) - 1]);
        heat->Kd  = pgm_read_float(&tmp3[h < COUNT(tmp3) ? h : COUNT(tmp3) - 1]);
        heat->Kc  = pgm_read_float(&tmp4[h < COUNT(tmp4) ? h : COUNT(tmp4) - 1]);
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
      sens->shC               = 0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->resetFlag();
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
      sens->shC               = 0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->resetFlag();
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
      sens->shC               = 0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->resetFlag();
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
      sens->shC               = 0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->resetFlag();
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
      sens->shC               = 0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->resetFlag();
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
      sens->shC               = 0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->resetFlag();
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
      sens->shC               = 0;
      sens->adcLowOffset      = 0;
      sens->adcHighOffset     = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset    = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain      = TEMP_SENSOR_AD595_GAIN;
      #endif
      heat->resetFlag();
      heat->setUsePid(PIDTEMPCOOLER);
      heat->setHWInverted(INVERTED_COOLER_PIN);
      #if HAS_EEPROM
        heat->setTuning(false);
      #else
        heat->setTuning(true);
      #endif
    #endif // HAS_HEATER_BED

  #endif // HEATER_COUNT > 0

  // Fans && Tachometric
  #if FAN_COUNT > 0

    #if ENABLED(TACHOMETRIC)
      constexpr pin_t tacho_temp_pin[] = { TACHO0_PIN, TACHO1_PIN, TACHO2_PIN, TACHO3_PIN, TACHO4_PIN, TACHO5_PIN };
    #endif

    Fan *fan;
    LOOP_FAN() {
      fan = &fans[f];
      fan->ID             = f;
      fan->pin            = (int8_t)pgm_read_byte(&tmp5[f]);
      fan->freq           = 250;
      fan->min_Speed      = FAN_MIN_PWM;
      fan->autoMonitored  = 0;
      fan->FanFlag        = 0;
      fan->setAutoMonitored((int8_t)pgm_read_byte(&tmp6[f]));
      fan->setHWInverted(FAN_INVERTED);
      #if ENABLED(TACHOMETRIC)
        tachometrics[f].pin = tacho_temp_pin[f];
      #endif
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

  #if ENABLED(IDLE_OOZING_PREVENT)
    printer.IDLE_OOZING_enabled = true;
  #endif

  reset_stepper_drivers();

  #if ENABLED(LIN_ADVANCE)
    planner.extruder_advance_K = LIN_ADVANCE_K;
  #endif

  #if ENABLED(HYSTERESIS_FEATURE)
    static const float tmp9[] PROGMEM = HYSTERESIS_AXIS_MM;
    LOOP_XYZ(i) planner.hysteresis_mm[i] = pgm_read_float(&tmp9[i]);
    planner.hysteresis_correction  = HYSTERESIS_CORRECTION;
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    for (uint8_t e = 0; e < DRIVER_EXTRUDERS; e++) {
      filament_change_unload_length[e] = PAUSE_PARK_UNLOAD_LENGTH;
      filament_change_load_length[e] = PAUSE_PARK_FAST_LOAD_LENGTH;
    }
  #endif

  watchdog.reset();

  Postprocess();

  SERIAL_LM(ECHO, "Factory Settings Loaded");
}

#if DISABLED(DISABLE_M503)

  inline void print_units(const bool colon) {
    SERIAL_PS(
      #if ENABLED(INCH_MODE_SUPPORT)
        parser.linear_unit_factor != 1 ? PSTR(" (in)") :
      #endif
      PSTR(" (mm)")
    );
    if (colon) SERIAL_EM(":");
  }
  #define SERIAL_UNITS(COLON) print_units(COLON)

  /**
   * M503 - Print Configuration
   */
  void EEPROM::Print_Settings() {

    /**
     * Announce current units, in case inches are being displayed
     */
    SERIAL_STR(CFG);
    #if ENABLED(INCH_MODE_SUPPORT)
      SERIAL_MSG("  G2");
      SERIAL_CHR(parser.linear_unit_factor == 1 ? '1' : '0');
      SERIAL_MSG(" ;");
      SERIAL_UNITS(false);
    #else
      SERIAL_MSG("  G21    ; Units in");
      SERIAL_UNITS(false);
    #endif
    SERIAL_EOL();

    /**
     * Print mechanics parameters
     */
    mechanics.print_parameters();

    /**
     * Print heaters parameters
     */
    #if HEATER_COUNT > 0
      LOOP_HEATER() {
        heaters[h].print_sensor_parameters();
        heaters[h].print_heater_parameters();
        heaters[h].print_PID_parameters();
      }
    #endif

    /**
     * Print dht parameters
     */
    #if ENABLED(DHT_SENSOR)
      dhtsensor.print_parameters();
    #endif

    /**
     * Print AD595 parameters
     */
    #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
      LOOP_HOTEND() heaters[h].print_AD595_parameters();
    #endif

    /**
     * Print Hotends offsets parameters
     */
    #if HOTENDS > 1
      LOOP_HOTEND() tools.print_parameters(h);
    #endif

    /**
     * Print Fans parameters
     */
    #if FAN_COUNT > 0
      LOOP_FAN() fans[f].print_parameters();
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
      SERIAL_LM(CFG, "LCD Contrast:");
      SERIAL_LMV(CFG, "  M250 C", lcd_contrast);
    #endif

    #if HAS_SERVOS
      LOOP_SERVO() servo[s].print_parameters();
    #endif

    /**
     * Bed Leveling
     */
    #if HAS_LEVELING

      #if ENABLED(MESH_BED_LEVELING)
        SERIAL_LM(CFG, "Mesh Bed Leveling:");
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        SERIAL_LM(CFG, "Unified Bed Leveling:");
      #elif HAS_ABL
        SERIAL_LM(CFG, "Auto Bed Leveling:");
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
        //ubl.report_current_mesh();

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        if (bedlevel.leveling_is_valid()) {
          for (uint8_t py = 0; py < GRID_MAX_POINTS_Y; py++) {
            for (uint8_t px = 0; px < GRID_MAX_POINTS_X; px++) {
              SERIAL_SMV(CFG, "  G29 W I", (int)px);
              SERIAL_MV(" J", (int)py);
              SERIAL_MV(" Z", LINEAR_UNIT(abl.z_values[px][py]), 5);
              SERIAL_EOL();
            }
          }
        }

      #endif

    #endif // HAS_LEVELING

    #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)

      SERIAL_LM(CFG, "Endstop adjustment:");
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
      SERIAL_SM(CFG, "Probe Offset");
      SERIAL_UNITS(true);
      SERIAL_SMV(CFG, "  M851 X", LINEAR_UNIT(probe.offset[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(probe.offset[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(probe.offset[Z_AXIS]), 3);
      SERIAL_EOL();
    #endif

    #if ENABLED(ULTIPANEL)
      SERIAL_LM(CFG, "Material heatup parameters:");
      for (uint8_t i = 0; i < COUNT(lcd_preheat_hotend_temp); i++) {
        SERIAL_SMV(CFG, "  M145 S", i);
        SERIAL_MV(" H", TEMP_UNIT(lcd_preheat_hotend_temp[i]));
        SERIAL_MV(" B", TEMP_UNIT(lcd_preheat_bed_temp[i]));
        SERIAL_MV(" F", lcd_preheat_fan_speed[i]);
        SERIAL_EOL();
      }
    #endif // ULTIPANEL

    #if ENABLED(FWRETRACT)
      SERIAL_LM(CFG, "Retract: S<length> F<units/m> Z<lift>:");
      SERIAL_SMV(CFG, "  M207 S", LINEAR_UNIT(fwretract.retract_length));
      SERIAL_MV(" W", LINEAR_UNIT(fwretract.swap_retract_length));
      SERIAL_MV(" F", MMS_TO_MMM(LINEAR_UNIT(fwretract.retract_feedrate_mm_s)));
      SERIAL_EMV(" Z", LINEAR_UNIT(fwretract.retract_zlift));

      SERIAL_LM(CFG, "Recover: S<length> F<units/m>:");
      SERIAL_SMV(CFG, "  M208 S", LINEAR_UNIT(fwretract.retract_recover_length));
      SERIAL_MV(" W", LINEAR_UNIT(fwretract.swap_retract_recover_length));
      SERIAL_MV(" F", MMS_TO_MMM(LINEAR_UNIT(fwretract.retract_recover_feedrate_mm_s)));
      SERIAL_MV(" R", MMS_TO_MMM(LINEAR_UNIT(fwretract.swap_retract_recover_feedrate_mm_s)));

      SERIAL_LM(CFG, "Auto-Retract: S=0 to disable, 1 to interpret E-only moves as retract/recover:");
      SERIAL_LMV(CFG, "  M209 S", fwretract.autoretract_enabled ? 1 : 0);
    #endif // FWRETRACT

    #if ENABLED(VOLUMETRIC_EXTRUSION)

      /**
       * Volumetric extrusion M200
       */
      SERIAL_SM(CFG, "Filament settings:");
      if (printer.isVolumetric())
        SERIAL_EOL();
      else
        SERIAL_EM(" Disabled");

      #if EXTRUDERS == 1
        SERIAL_LMV(CFG, "  M200 T0 D", tools.filament_size[0], 3);
      #elif EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M200 T", (int)e);
          SERIAL_EMV(" D", tools.filament_size[e], 3);
        }
      #endif

    #endif // ENABLED(VOLUMETRIC_EXTRUSION)

    /**
     * Stepper driver control
     */
    SERIAL_LM(CFG, "Stepper Direction:");
    SERIAL_SMV(CFG, "  M569 X", (int)stepper.isStepDir(X_AXIS));
    SERIAL_MV(" Y", (int)stepper.isStepDir(Y_AXIS));
    SERIAL_MV(" Z", (int)stepper.isStepDir(Z_AXIS));
    #if DRIVER_EXTRUDERS == 1
      SERIAL_MV(" T0 E", (int)stepper.isStepDir(E_AXIS));
    #endif
    SERIAL_EOL();
    #if DRIVER_EXTRUDERS > 1
      for (int8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
        SERIAL_SMV(CFG, "  M569 T", i);
        SERIAL_EMV(" E" , (int)stepper.isStepDir((AxisEnum)(E_AXIS + i)));
      }
    #endif
    SERIAL_LM(CFG, "Stepper driver control:");
    SERIAL_SMV(CFG, "  M569 D", stepper.direction_delay);
    SERIAL_MV(" P", stepper.minimum_pulse);
    SERIAL_MV(" R", stepper.maximum_rate);
    SERIAL_EOL();

    /**
     * Alligator current drivers M906
     */
    #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)

      SERIAL_LM(CFG, "Motor current (mA):");
      SERIAL_SMV(CFG, "  M906 X", externaldac.motor_current[X_AXIS]);
      SERIAL_MV(" Y", externaldac.motor_current[Y_AXIS]);
      SERIAL_MV(" Z", externaldac.motor_current[Z_AXIS]);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", externaldac.motor_current[E_AXIS]);
      #endif
      SERIAL_EOL();
      #if DRIVER_EXTRUDERS > 1
        for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
          SERIAL_SMV(CFG, "  M906 T", i);
          SERIAL_EMV(" E", externaldac.motor_current[E_AXIS + i]);
        }
      #endif

    #endif // ALLIGATOR_R2 || ALLIGATOR_R3

    #if HAS_TRINAMIC

      /**
       * TMC2130 or TMC2208 stepper driver current
       */
      SERIAL_LM(CFG, "Stepper driver current (mA):");
      SERIAL_SM(CFG, "  M906");
      #if X_IS_TRINAMIC
        SERIAL_MV(" X", stepperX.getCurrent());
      #endif
      #if X2_IS_TRINAMIC
        SERIAL_MV(" I1 X", stepperX2.getCurrent());
      #endif
      #if Y_IS_TRINAMIC
        SERIAL_MV(" Y", stepperY.getCurrent());
      #endif
      #if Y2_IS_TRINAMIC
        SERIAL_MV(" I1 Y", stepperY2.getCurrent());
      #endif
      #if Z_IS_TRINAMIC
        SERIAL_MV(" Z", stepperZ.getCurrent());
      #endif
      #if Z2_IS_TRINAMIC
        SERIAL_MV(" I1 Z", stepperZ2.getCurrent());
      #endif
      #if E0_IS_TRINAMIC
        SERIAL_MV(" T0 E", stepperE0.getCurrent());
      #endif
      #if E1_IS_TRINAMIC
        SERIAL_MV(" T1 E", stepperE1.getCurrent());
      #endif
      #if E2_IS_TRINAMIC
        SERIAL_MV(" T2 E", stepperE2.getCurrent());
      #endif
      #if E3_IS_TRINAMIC
        SERIAL_MV(" T3 E", stepperE3.getCurrent());
      #endif
      #if E4_IS_TRINAMIC
        SERIAL_MV(" T4 E", stepperE4.getCurrent());
      #endif
      #if E5_IS_TRINAMIC
        SERIAL_MV(" T5 E", stepperE5.getCurrent());
      #endif
      SERIAL_EOL();

      /**
       * TMC2130 or TMC2208 Hybrid Threshold
       */
      #if ENABLED(HYBRID_THRESHOLD)
        SERIAL_LM(CFG, "Hybrid Threshold:");
        SERIAL_SM(CFG, "  M913");
        #if X_IS_TRINAMIC
          SERIAL_MV(" X", TMC_GET_PWMTHRS(X, X));
        #endif
        #if X2_IS_TRINAMIC
          SERIAL_MV(" I1 X", TMC_GET_PWMTHRS(X, X2));
        #endif
        #if Y_IS_TRINAMIC
          SERIAL_MV(" Y", TMC_GET_PWMTHRS(Y, Y));
        #endif
        #if Y2_IS_TRINAMIC
          SERIAL_MV(" I1 Y", TMC_GET_PWMTHRS(Y, Y2));
        #endif
        #if Z_IS_TRINAMIC
          SERIAL_MV(" Z", TMC_GET_PWMTHRS(Z, Z));
        #endif
        #if Z2_IS_TRINAMIC
          SERIAL_MV(" I1 Z", TMC_GET_PWMTHRS(Z, Z2));
        #endif
        #if E0_IS_TRINAMIC
          SERIAL_MV(" T0 E", TMC_GET_PWMTHRS(E, E0));
        #endif
        #if E1_IS_TRINAMIC
          SERIAL_MV(" T1 E", TMC_GET_PWMTHRS(E, E1));
        #endif
        #if E2_IS_TRINAMIC
          SERIAL_MV(" T2 E", TMC_GET_PWMTHRS(E, E2));
        #endif
        #if E3_IS_TRINAMIC
          SERIAL_MV(" T3 E", TMC_GET_PWMTHRS(E, E3));
        #endif
        #if E4_IS_TRINAMIC
          SERIAL_MV(" T4 E", TMC_GET_PWMTHRS(E, E4));
        #endif
        #if E5_IS_TRINAMIC
          SERIAL_MV(" T5 E", TMC_GET_PWMTHRS(E, E5));
        #endif
        SERIAL_EOL();
      #endif // HYBRID_THRESHOLD

      /**
       * TMC2130 Sensorless homing thresholds
       */
      #if ENABLED(SENSORLESS_HOMING)
        SERIAL_LM(CFG, "Sensorless homing threshold:");
        SERIAL_SM(CFG, "  M914");
        #if ENABLED(X_HOMING_SENSITIVITY)
          #if X_HAS_DRV(TMC2130) || ENABLED(IS_TRAMS)
            SERIAL_MV(" X", stepperX.sgt());
          #endif
          #if X2_HAS_DRV(TMC2130)
            SERIAL_MV(" I1 X", stepperX2.sgt());
          #endif
        #endif
        #if ENABLED(Y_HOMING_SENSITIVITY)
          #if Y_HAS_DRV(TMC2130) || ENABLED(IS_TRAMS)
            SERIAL_MV(" Y", stepperY.sgt());
          #endif
          #if X2_HAS_DRV(TMC2130)
            SERIAL_MV(" I1 Y", stepperY2.sgt());
          #endif
        #endif
        #if ENABLED(Z_HOMING_SENSITIVITY)
          #if Z_HAS_DRV(TMC2130) || ENABLED(IS_TRAMS)
            SERIAL_MV(" Z", stepperZ.sgt());
          #endif
          #if Z2_HAS_DRV(TMC2130)
            SERIAL_MV(" I1 Z", stepperZ2.sgt());
          #endif
        #endif
        SERIAL_EOL();
      #endif

    #endif // HAS_TRINAMIC

    /**
     * Linear Advance
     */
    #if ENABLED(LIN_ADVANCE)
      SERIAL_LM(CFG, "Linear Advance:");
      SERIAL_LMV(CFG, "  M900 K", planner.extruder_advance_K);
    #endif

    /**
     * Hysteresis Feature
     */
    #if ENABLED(HYSTERESIS_FEATURE)
      SERIAL_LM(CFG, "Hysteresis Correction:");
      SERIAL_SMV(CFG, "  M99 X", planner.hysteresis_mm[X_AXIS]);
      SERIAL_MV(" Y", planner.hysteresis_mm[Y_AXIS]);
      SERIAL_MV(" Z", planner.hysteresis_mm[Z_AXIS]);
      SERIAL_MV(" F", planner.hysteresis_correction);
      SERIAL_EOL();
    #endif

    /**
     * Advanced Pause filament load & unload lengths
     */
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      SERIAL_LM(CFG, "Filament load/unload lengths:");
      #if EXTRUDERS == 1
        SERIAL_SMV(CFG, "  M603 L", LINEAR_UNIT(filament_change_load_length[0]), 2);
        SERIAL_EMV(" U", LINEAR_UNIT(filament_change_unload_length[0]), 2);
      #else // EXTRUDERS != 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M603 T", (int)e);
          SERIAL_MV(" L", LINEAR_UNIT(filament_change_load_length[e]), 2);
          SERIAL_EMV(" U", LINEAR_UNIT(filament_change_unload_length[e]), 2);
        }
      #endif // EXTRUDERS != 1
    #endif // ADVANCED_PAUSE_FEATURE

    #if HAS_SD_SUPPORT
      card.print_settings();
    #endif

  }

#endif // !DISABLE_M503

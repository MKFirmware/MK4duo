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

#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
  float new_z_fade_height;
#endif

#if HAS_TRINAMIC
  #define TMC_GET_PWMTHRS(P,ST) tmc.thrs(stepper##ST->microsteps(), stepper##ST->TPWMTHRS(), mechanics.data.axis_steps_per_mm[P##_AXIS])
#endif

#pragma pack(push, 1)

typedef struct { uint32_t X, Y, Z, X2, Y2, Z2, Z3, E0, E1, E2, E3, E4, E5; } tmc_hybrid_threshold_t;
typedef struct {  int16_t X, Y, Z;                                         } tmc_sgt_t;

/**
 * Current EEPROM Layout && Version
 *
 * Keep this data structure up to date so
 * EEPROM size is known at compile time!
 */
#define EEPROM_VERSION "MKV55"
typedef struct EepromDataStruct {

  char      version[6];                                 // MKVnn\0
  uint16_t  crc;                                        // Data Checksum

  //
  // Mechanics data
  //
  mechanics_data_t  mechanics_data;

  //
  // Hotend offset
  //
  float             hotend_offset[XYZ][HOTENDS];

  //
  // Endstop
  //
  flagword_t        endstop_logic_flag,
                    endstop_pullup_flag;

  #if ENABLED(X_TWO_ENDSTOPS)
    float           x2_endstop_adj;
  #endif
  #if ENABLED(Y_TWO_ENDSTOPS)
    float           y2_endstop_adj;
  #endif
  #if ENABLED(Z_TWO_ENDSTOPS)
    float           z2_endstop_adj;
  #endif
  #if ENABLED(Z_THREE_ENDSTOPS)
    float           z3_endstop_adj;
  #endif

  //
  // Filament Runout
  //
  #if HAS_FIL_RUNOUT_0
    flagbyte_t      filrunout_logic_flag,
                    filrunout_pullup_flag;
  #endif

  //
  // Power Check
  //
  #if HAS_POWER_CHECK
    flagbyte_t      power_flag;
  #endif

  //
  // Z fade height
  //
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    float           z_fade_height;
  #endif

  //
  // MESH_BED_LEVELING
  //
  #if ENABLED(MESH_BED_LEVELING)
    float           mbl_z_offset;
    uint8_t         mesh_num_x,
                    mesh_num_y;
    float           mbl_z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
  #endif

  //
  // Planar Bed Leveling matrix
  //
  #if ABL_PLANAR
    matrix_3x3      bed_level_matrix;
  #endif

  //
  // Bilinear Auto Bed Leveling
  //
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    uint8_t         grid_max_x,
                    grid_max_y;
    int             bilinear_grid_spacing[2],
                    bilinear_start[2];
    float           z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
  #endif

  //
  // Universal Bed Leveling
  //
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    bool            ubl_leveling_active;
    int8_t          ubl_storage_slot;
  #endif

  //
  // Probe offset
  //
  #if HAS_BED_PROBE
    probe_data_t    probe_data;
  #endif

  //
  // Ultipanel
  //
  #if HAS_LCD_MENU
    int16_t         lcdui_preheat_hotend_temp[3],
                    lcdui_preheat_bed_temp[3],
                    lcdui_preheat_fan_speed[3];
  #endif

  //
  // Heaters
  //
  #if HEATER_COUNT > 0
    heater_data_t   heater_data[HEATER_COUNT];
    pid_data_t      pid_data[HEATER_COUNT];
    sensor_data_t   sensor_data[HEATER_COUNT];
  #endif

  //
  // PID add extrusion rate
  //
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    int16_t         lpq_len;
  #endif

  //
  // DHT sensor
  //
  #if ENABLED(DHT_SENSOR)
    pin_t             dht_pin;
    uint8_t           dht_type;
  #endif

  //
  // Fans
  //
  #if FAN_COUNT > 0
    fan_data_t        fans_data[FAN_COUNT];
    #if ENABLED(TACHOMETRIC)
      tacho_data_t    tacho_data[FAN_COUNT];
    #endif
  #endif

  //
  // LCD contrast
  //
  #if HAS_LCD_CONTRAST
    uint8_t           lcdui_contrast;
  #endif

  //
  // SD Restart
  //
  #if HAS_SD_RESTART
    bool              restart_enabled;
  #endif

  //
  // Servo angles
  //
  #if HAS_SERVOS
    int               servo_angles[NUM_SERVOS][2];
  #endif

  //
  // FWRETRACT
  //
  #if ENABLED(FWRETRACT)
    fwretract_data_t  fwretract_data;
    bool              autoretract_enabled;
  #endif

  //
  // Volumetric & Filament Size
  //
  #if ENABLED(VOLUMETRIC_EXTRUSION)
    bool              volumetric_enabled;
    float             filament_size[EXTRUDERS];
  #endif

  //
  // IDLE oozing
  //
  #if ENABLED(IDLE_OOZING_PREVENT)
    bool              IDLE_OOZING_enabled;
  #endif

  //
  // Stepper
  //
  uint16_t            stepper_direction_flag;
  uint32_t            stepper_direction_delay,
                      stepper_maximum_rate;
  uint8_t             stepper_minimum_pulse;

  //
  // Sound
  //
  SoundModeEnum       sound_mode;

  //
  // External DAC
  //
  #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
    uint16_t          motor_current[3 + DRIVER_EXTRUDERS];
  #endif

  //
  // Linear Advance
  //
  #if ENABLED(LIN_ADVANCE)
    float             planner_extruder_advance_K;
  #endif

  //
  // Hysteresis Feature
  //
  #if ENABLED(HYSTERESIS_FEATURE)
    float             planner_hysteresis_mm,
                      planner_hysteresis_correction;
  #endif

  //
  // Filament Change
  //
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    advanced_pause_data_t advanced_pause_data[EXTRUDERS];
  #endif

  //
  // Trinamic
  //
  #if HAS_TRINAMIC
    uint16_t                tmc_stepper_current[TMC_AXIS],
                            tmc_stepper_microstep[TMC_AXIS];
    tmc_hybrid_threshold_t  tmc_hybrid_threshold;
    tmc_sgt_t               tmc_sgt;
  #endif

} eepromData;

EEPROM eeprom;

uint16_t EEPROM::datasize() { return sizeof(eepromData); }

/**
 * Post-process after Retrieve or Reset
 */
void EEPROM::post_process() {

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
      heaters[h].pid.update();
    }
  #endif

  #if ENABLED(DHT_SENSOR)
    dhtsensor.init();
  #endif

  #if FAN_COUNT > 0
    LOOP_FAN() {
      fans[f].init();
      #if ENABLED(TACHOMETRIC)
        fans[f].tacho.init(f);
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

  // Setup FilRunout pullup
  #if HAS_FIL_RUNOUT_0
    filamentrunout.setup_pullup();
  #endif

  // Setup power check pullup
  #if HAS_POWER_CHECK
    powerManager.setup_pullup();
  #endif

  // Refresh steps_to_mm with the reciprocal of axis_steps_per_mm
  // and init stepper.count[], planner.position[] with current_position
  planner.refresh_positioning();

  if (memcmp(oldpos, mechanics.current_position, sizeof(oldpos)))
    mechanics.report_current_position();

}

#if HAS_EEPROM

  #define EEPROM_READ_START()     int eeprom_index = EEPROM_OFFSET; eeprom_error = memorystore.access_start(true)
  #define EEPROM_WRITE_START()    int eeprom_index = EEPROM_OFFSET; eeprom_error = memorystore.access_start(false)
  #define EEPROM_READ_FINISH()    memorystore.access_finish(true)
  #define EEPROM_WRITE_FINISH()   memorystore.access_finish(false)
  #define EEPROM_SKIP(VAR)        eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR)       memorystore.write_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ_ALWAYS(VAR) memorystore.read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ(VAR)        memorystore.read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc, !validating)

  #define EEPROM_ASSERT(TST,ERR) do{ if (!(TST)) { SERIAL_LM(ER, ERR); eeprom_error = true; } }while(0)
  #define _FIELD_TEST(FIELD) \
    EEPROM_ASSERT( \
      eeprom_error || eeprom_index == offsetof(eepromData, FIELD) + EEPROM_OFFSET, \
      "Field " STRINGIFY(FIELD) " mismatch." \
    )

  const char version[6] = EEPROM_VERSION;

  bool  EEPROM::eeprom_error  = false,
        EEPROM::validating    = false;

  #if ENABLED(AUTO_BED_LEVELING_UBL)
    uint16_t EEPROM::meshes_begin = 0;
  #endif

  bool EEPROM::size_error(const uint16_t size) {
    if (size != datasize()) {
      #if ENABLED(EEPROM_CHITCHAT)
        SERIAL_LM(ER, "EEPROM datasize error.");
      #endif
      return true;
    }
    return false;
  }

  /**
   * M500 - Store Configuration
   */
  bool EEPROM::store() {

    float dummy = 0;
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

    //
    // Mechanics data
    //
    EEPROM_WRITE(mechanics.data);

    //
    // Hotend offset
    //
    EEPROM_WRITE(tools.hotend_offset);

    //
    // Endstops bit
    //
    EEPROM_WRITE(endstops.logic_flag);
    EEPROM_WRITE(endstops.pullup_flag);

    //
    // TWO or THREE Endstops adj
    //
    #if ENABLED(X_TWO_ENDSTOPS)
      EEPROM_WRITE(endstops.x2_endstop_adj);
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      EEPROM_WRITE(endstops.y2_endstop_adj);
    #endif
    #if ENABLED(Z_THREE_ENDSTOPS)
      EEPROM_WRITE(endstops.z2_endstop_adj);
      EEPROM_WRITE(endstops.z3_endstop_adj);
    #elif ENABLED(Z_TWO_ENDSTOPS)
      EEPROM_WRITE(endstops.z2_endstop_adj);
    #endif

    //
    // Filament Runout
    //
    #if HAS_FIL_RUNOUT_0
      EEPROM_WRITE(filamentrunout.logic_flag);
      EEPROM_WRITE(filamentrunout.pullup_flag);
    #endif

    //
    // Power Check
    //
    #if HAS_POWER_CHECK
      EEPROM_WRITE(powerManager.flag);
    #endif

    //
    // Z fade height
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
      EEPROM_WRITE(grid_max_x);                 // 1 byte
      EEPROM_WRITE(grid_max_y);                 // 1 byte
      EEPROM_WRITE(abl.bilinear_grid_spacing);  // 2 ints
      EEPROM_WRITE(abl.bilinear_start);         // 2 ints
      EEPROM_WRITE(abl.z_values);               // 9-256 floats
    #endif // AUTO_BED_LEVELING_BILINEAR

    //
    // Universal Bed Leveling
    //
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      const bool bedlevel_leveling_active = bedlevel.flag.leveling_active;
      EEPROM_WRITE(bedlevel_leveling_active);
      EEPROM_WRITE(ubl.storage_slot);
    #endif

    //
    // Probe data
    //
    #if HAS_BED_PROBE
      EEPROM_WRITE(probe.data);
    #endif

    //
    // ULTIPANEL
    //
    #if HAS_LCD_MENU
      EEPROM_WRITE(lcdui.preheat_hotend_temp);
      EEPROM_WRITE(lcdui.preheat_bed_temp);
      EEPROM_WRITE(lcdui.preheat_fan_speed);
    #endif

    //
    // Heaters
    //
    #if HEATER_COUNT > 0
      LOOP_HEATER() {
        EEPROM_WRITE(heaters[h].data);
        EEPROM_WRITE(heaters[h].pid);
        EEPROM_WRITE(heaters[h].sensor);
      }
    #endif

    //
    // PID add extrusion rate
    //
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      EEPROM_WRITE(tools.lpq_len);
    #endif

    //
    // DHT sensor
    //
    #if ENABLED(DHT_SENSOR)
      EEPROM_WRITE(dhtsensor.pin);
      EEPROM_WRITE(dhtsensor.type);
    #endif

    //
    // Fans
    //
    #if FAN_COUNT > 0
      LOOP_FAN() {
        EEPROM_WRITE(fans[f].data);
        #if ENABLED(TACHOMETRIC)
          EEPROM_WRITE(fans[f].tacho);
        #endif
      }
    #endif

    //
    // LCD contrast
    //
    #if HAS_LCD_CONTRAST
      EEPROM_WRITE(lcdui.contrast);
    #endif

    //
    // SD Restart
    //
    #if HAS_SD_RESTART
      EEPROM_WRITE(restart.enabled);
    #endif

    //
    // Servo angles
    //
    #if HAS_SERVOS
      LOOP_SERVO() EEPROM_WRITE(servo[s].angle);
    #endif

    //
    // Firmware Retraction
    //
    #if ENABLED(FWRETRACT)
      EEPROM_WRITE(fwretract.data);
      EEPROM_WRITE(fwretract.autoretract_enabled);
    #endif

    //
    // Volumetric & Filament Size
    //
    #if ENABLED(VOLUMETRIC_EXTRUSION)

      const bool volumetric_enabled = printer.isVolumetric();
      EEPROM_WRITE(volumetric_enabled);

      // Save filament sizes
      LOOP_EXTRUDER()
        EEPROM_WRITE(tools.filament_size[e]);

    #endif

    //
    // IDLE oozing
    //
    #if ENABLED(IDLE_OOZING_PREVENT)
      EEPROM_WRITE(printer.IDLE_OOZING_enabled);
    #endif

    //
    // Stepper
    //
    EEPROM_WRITE(stepper.direction_flag);
    EEPROM_WRITE(stepper.direction_delay);
    EEPROM_WRITE(stepper.minimum_pulse);
    EEPROM_WRITE(stepper.maximum_rate);

    //
    // Sound
    //
    EEPROM_WRITE(sound.mode);

    //
    // Alligator board
    //
    #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
      EEPROM_WRITE(externaldac.motor_current);
    #endif

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
      EEPROM_WRITE(advancedpause.data);
    #endif

    //
    // Save TMC2130 or TMC2208 Configuration, and placeholder values
    //
    #if HAS_TRINAMIC

      uint16_t  tmc_stepper_current[TMC_AXIS]   = { X_CURRENT, Y_CURRENT, Z_CURRENT, X2_CURRENT, Y2_CURRENT, Z2_CURRENT, Z3_CURRENT,
                                                    E0_CURRENT, E1_CURRENT, E2_CURRENT, E3_CURRENT, E4_CURRENT, E5_CURRENT },
                tmc_stepper_microstep[TMC_AXIS] = { X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS, X2_MICROSTEPS, Y2_MICROSTEPS, Z2_MICROSTEPS, Z3_MICROSTEPS,
                                                    E0_MICROSTEPS, E1_MICROSTEPS, E2_MICROSTEPS, E3_MICROSTEPS, E4_MICROSTEPS, E5_MICROSTEPS };

      LOOP_TMC() {
        MKTMC* st = tmc.driver_by_index(t);
        if (st) {
          tmc_stepper_current[t]    = st->getMilliamps();
          tmc_stepper_microstep[t]  = st->microsteps();
        }
      }

      EEPROM_WRITE(tmc_stepper_current);
      EEPROM_WRITE(tmc_stepper_microstep);

      //
      // Save TMC2130 or TMC2208 Hybrid Threshold, and placeholder values
      //
      #if ENABLED(HYBRID_THRESHOLD)
        tmc_hybrid_threshold_t tmc_hybrid_threshold = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        #if AXIS_HAS_TMC(X)
          tmc_hybrid_threshold.X = TMC_GET_PWMTHRS(X, X);
        #else
          tmc_hybrid_threshold.X = X_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(Y)
          tmc_hybrid_threshold.Y = TMC_GET_PWMTHRS(Y, Y);
        #else
          tmc_hybrid_threshold.Y = Y_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(Z)
          tmc_hybrid_threshold.Z = TMC_GET_PWMTHRS(Z, Z);
        #else
          tmc_hybrid_threshold.Z = Z_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(X2)
          tmc_hybrid_threshold.X2 = TMC_GET_PWMTHRS(X, X2);
        #else
          tmc_hybrid_threshold.X2 = X2_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(Y2)
          tmc_hybrid_threshold.Y2 = TMC_GET_PWMTHRS(Y, Y2);
        #else
          tmc_hybrid_threshold.Y2 = Y2_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(Z2)
          tmc_hybrid_threshold.Z2 = TMC_GET_PWMTHRS(Z, Z2);
        #else
          tmc_hybrid_threshold.Z2 = Z2_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(Z3)
          tmc_hybrid_threshold.Z3 = TMC_GET_PWMTHRS(Z, Z3);
        #else
          tmc_hybrid_threshold.Z3 = Z3_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(E0)
          tmc_hybrid_threshold.E0 = TMC_GET_PWMTHRS(E0, E0);
        #else
          tmc_hybrid_threshold.E0 = E0_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(E1)
          tmc_hybrid_threshold.E1 = TMC_GET_PWMTHRS(E1, E1);
        #else
          tmc_hybrid_threshold.E1 = E1_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(E2)
          tmc_hybrid_threshold.E2 = TMC_GET_PWMTHRS(E2, E2);
        #else
          tmc_hybrid_threshold.E2 = E2_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(E3)
          tmc_hybrid_threshold.E3 = TMC_GET_PWMTHRS(E3, E3);
        #else
          tmc_hybrid_threshold.E3 = E3_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(E4)
          tmc_hybrid_threshold.E4 = TMC_GET_PWMTHRS(E4, E4);
        #else
          tmc_hybrid_threshold.E4 = E4_HYBRID_THRESHOLD;
        #endif
        #if AXIS_HAS_TMC(E5)
          tmc_hybrid_threshold.E5 = TMC_GET_PWMTHRS(E5, E5)
        #else
          tmc_hybrid_threshold.E5 = E5_HYBRID_THRESHOLD;
        #endif
      #else // !HYBRID_THRESHOLD
        const tmc_hybrid_threshold_t tmc_hybrid_threshold = {
          .X  = 100, .Y  = 100, .Z  =   3,
          .X2 = 100, .Y2 = 100, .Z2 =   3, .Z3 =  3,
          .E0 =  30, .E1 =  30, .E2 =  30,
          .E3 =  30, .E4 =  30, .E5 =  30
        };
      #endif // |HYBRID_THRESHOLD
      EEPROM_WRITE(tmc_hybrid_threshold);

      //
      // TMC2130 StallGuard threshold
      //
      tmc_sgt_t tmc_sgt = { 0, 0, 0 };
      #if HAS_SENSORLESS
        #if X_HAS_SENSORLESS
          tmc_sgt.X = stepperX->sgt();
        #endif
        #if Y_HAS_SENSORLESS
          tmc_sgt.Y = stepperY->sgt();
        #endif
        #if Z_HAS_SENSORLESS
          tmc_sgt.Z = stepperZ->sgt();
        #endif
      #endif
      EEPROM_WRITE(tmc_sgt);

    #endif // HAS_TRINAMIC

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
        SERIAL_SMV(ECHO, "Settings Stored (", eeprom_size);
        SERIAL_MV(" bytes; crc ", final_crc);
        SERIAL_EM(")");
      #endif

      eeprom_error |= size_error(eeprom_size);
    }

    EEPROM_WRITE_FINISH();

    //
    // UBL Mesh
    //
    #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(UBL_SAVE_ACTIVE_ON_M500)
      if (ubl.storage_slot >= 0)
        store_mesh(ubl.storage_slot);
    #endif

    sound.feedback(!eeprom_error);

    return !eeprom_error;
  }

  /**
   * M501 - Load Configuration
   */
  bool EEPROM::_load() {

    uint16_t  working_crc = 0,
              stored_crc  = 0;

    char stored_ver[6];

    EEPROM_READ_START();

    EEPROM_READ_ALWAYS(stored_ver);
    EEPROM_READ_ALWAYS(stored_crc);

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
      reset();
      eeprom_error = true;
    }
    else {

      float dummy = 0;

      working_crc = 0; // Init to 0. Accumulated by EEPROM_READ

      //
      // Mechanics data
      //
      EEPROM_READ(mechanics.data);

      //
      // Hotend offset
      //
      EEPROM_READ(tools.hotend_offset);

      //
      // Endstops bit
      //
      EEPROM_READ(endstops.logic_flag);
      EEPROM_READ(endstops.pullup_flag);

      //
      // TWO or THREE Endstops adj
      //
      #if ENABLED(X_TWO_ENDSTOPS)
        EEPROM_READ(endstops.x2_endstop_adj);
      #endif
      #if ENABLED(Y_TWO_ENDSTOPS)
        EEPROM_READ(endstops.y2_endstop_adj);
      #endif
      #if ENABLED(Z_THREE_ENDSTOPS)
        EEPROM_READ(endstops.z2_endstop_adj);
        EEPROM_READ(endstops.z3_endstop_adj);
      #elif ENABLED(Z_TWO_ENDSTOPS)
        EEPROM_READ(endstops.z2_endstop_adj);
      #endif

      //
      // Filament Runout
      //
      #if HAS_FIL_RUNOUT_0
        EEPROM_READ(filamentrunout.logic_flag);
        EEPROM_READ(filamentrunout.pullup_flag);
      #endif

      //
      // Power Check
      //
      #if HAS_POWER_CHECK
        EEPROM_READ(powerManager.flag);
      #endif

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
        EEPROM_READ_ALWAYS(mesh_num_x);
        EEPROM_READ_ALWAYS(mesh_num_y);
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
        EEPROM_READ_ALWAYS(grid_max_x);            // 1 byte
        EEPROM_READ_ALWAYS(grid_max_y);            // 1 byte
        if (grid_max_x == GRID_MAX_POINTS_X && grid_max_y == GRID_MAX_POINTS_Y) {
          if (!validating) bedlevel.set_bed_leveling_enabled(false);
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

      //
      // Universal Bed Leveling
      //
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        bool bedlevel_leveling_active;
        EEPROM_READ(bedlevel_leveling_active);
        EEPROM_READ(ubl.storage_slot);
        bedlevel.flag.leveling_active = bedlevel_leveling_active;
      #endif

      //
      // Probe data
      //
      #if HAS_BED_PROBE
        EEPROM_READ(probe.data);
      #endif

      //
      // ULTIPANEL
      //
      #if HAS_LCD_MENU
        EEPROM_READ(lcdui.preheat_hotend_temp);
        EEPROM_READ(lcdui.preheat_bed_temp);
        EEPROM_READ(lcdui.preheat_fan_speed);
      #endif

      //
      // Heaters
      //
      #if HEATER_COUNT > 0
        LOOP_HEATER() {
          EEPROM_READ(heaters[h].data);
          EEPROM_READ(heaters[h].pid);
          EEPROM_READ(heaters[h].sensor);
        }
      #endif

      //
      // PID add extrusion rate
      //
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        EEPROM_READ(tools.lpq_len);
      #endif

      //
      // DHT sensor
      //
      #if ENABLED(DHT_SENSOR)
        EEPROM_READ(dhtsensor.pin);
        EEPROM_READ(dhtsensor.type);
      #endif

      //
      // Fans
      //
      #if FAN_COUNT > 0
        LOOP_FAN() {
          EEPROM_READ(fans[f].data);
          #if ENABLED(TACHOMETRIC)
            EEPROM_READ(fans[f].tacho);
          #endif
        }
      #endif

      //
      // LCD contrast
      //
      #if HAS_LCD_CONTRAST
        EEPROM_READ(lcdui.contrast);
      #endif

      //
      // SD Restart
      //
      #if HAS_SD_RESTART
        EEPROM_READ(restart.enabled);
      #endif

      //
      // Servo angles
      //
      #if HAS_SERVOS
        LOOP_SERVO() EEPROM_READ(servo[s].angle);
      #endif

      //
      // Firmware Retraction
      //
      #if ENABLED(FWRETRACT)
        EEPROM_READ(fwretract.data);
        EEPROM_READ(fwretract.autoretract_enabled);
      #endif

      //
      // Volumetric & Filament Size
      //
      #if ENABLED(VOLUMETRIC_EXTRUSION)

        bool volumetric_enabled;
        EEPROM_READ(volumetric_enabled);
        if (!validating) printer.setVolumetric(volumetric_enabled);

        LOOP_EXTRUDER()
          EEPROM_READ(tools.filament_size[e]);

      #endif

      //
      // IDLE oozing
      //
      #if ENABLED(IDLE_OOZING_PREVENT)
        EEPROM_READ(printer.IDLE_OOZING_enabled);
      #endif

      //
      // Stepper
      //
      EEPROM_READ(stepper.direction_flag);
      EEPROM_READ(stepper.direction_delay);
      EEPROM_READ(stepper.minimum_pulse);
      EEPROM_READ(stepper.maximum_rate);

      //
      // Sound
      //
      EEPROM_READ(sound.mode);

      //
      // Alligator board
      //
      #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
        EEPROM_READ(externaldac.motor_current);
      #endif

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
        EEPROM_READ(advancedpause.data);
      #endif

      if (!validating) reset_stepper_drivers();

      //
      // TMC2130 or TMC2208 Stepper Current
      //
      #if HAS_TRINAMIC

        uint16_t  tmc_stepper_current[TMC_AXIS],
                  tmc_stepper_microstep[TMC_AXIS];

        EEPROM_READ(tmc_stepper_current);
        EEPROM_READ(tmc_stepper_microstep);

        if (!validating) {
          LOOP_TMC() {
            MKTMC* st = tmc.driver_by_index(t);
            if (st) {
              st->rms_current(tmc_stepper_current[t]);
              st->microsteps(tmc_stepper_microstep[t]);
            }
          }
        }

        #define TMC_SET_PWMTHRS(P,ST) tmc.set_pwmthrs(stepper##ST, tmc_hybrid_threshold.ST, mechanics.data.axis_steps_per_mm[P##_AXIS])
        tmc_hybrid_threshold_t tmc_hybrid_threshold;
        EEPROM_READ(tmc_hybrid_threshold);
        #if ENABLED(HYBRID_THRESHOLD)
          if (!validating) {
            #if AXIS_HAS_TMC(X)
              TMC_SET_PWMTHRS(X, X);
            #endif
            #if AXIS_HAS_TMC(Y)
              TMC_SET_PWMTHRS(Y, Y);
            #endif
            #if AXIS_HAS_TMC(Z)
              TMC_SET_PWMTHRS(Z, Z);
            #endif
            #if AXIS_HAS_TMC(X2)
              TMC_SET_PWMTHRS(X, X2);
            #endif
            #if AXIS_HAS_TMC(Y2)
              TMC_SET_PWMTHRS(Y, Y2);
            #endif
            #if AXIS_HAS_TMC(Z2)
              TMC_SET_PWMTHRS(Z, Z2);
            #endif
            #if AXIS_HAS_TMC(Z3)
              TMC_SET_PWMTHRS(Z, Z3);
            #endif
            #if AXIS_HAS_TMC(E0)
              TMC_SET_PWMTHRS(E, E0);
            #endif
            #if AXIS_HAS_TMC(E1)
              TMC_SET_PWMTHRS(E, E1);
            #endif
            #if AXIS_HAS_TMC(E2)
              TMC_SET_PWMTHRS(E, E2);
            #endif
            #if AXIS_HAS_TMC(E3)
              TMC_SET_PWMTHRS(E, E3);
            #endif
            #if AXIS_HAS_TMC(E4)
              TMC_SET_PWMTHRS(E, E4);
            #endif
            #if AXIS_HAS_TMC(E4)
              TMC_SET_PWMTHRS(E, E5);
            #endif
          }
        #endif

        /*
         * TMC2130 Sensorless homing threshold.
         * X and X2 use the same value
         * Y and Y2 use the same value
         * Z, Z2 and Z3 use the same value
         */
        tmc_sgt_t tmc_sgt;
        EEPROM_READ(tmc_sgt);
        #if HAS_SENSORLESS
          if (!validating) {
            #if ENABLED(X_STALL_SENSITIVITY)
              #if AXIS_HAS_STALLGUARD(X)
                stepperX->sgt(tmc_sgt.X);
              #endif
              #if AXIS_HAS_STALLGUARD(X2)
                stepperX2->sgt(tmc_sgt.X);
              #endif
            #endif
            #if ENABLED(Y_STALL_SENSITIVITY)
              #if AXIS_HAS_STALLGUARD(Y)
                stepperY->sgt(tmc_sgt.Y);
              #endif
              #if AXIS_HAS_STALLGUARD(Y2)
                stepperY2->sgt(tmc_sgt.Y);
              #endif
            #endif
            #if ENABLED(Z_STALL_SENSITIVITY)
              #if AXIS_HAS_STALLGUARD(Z)
                stepperZ->sgt(tmc_sgt.Z);
              #endif
              #if AXIS_HAS_STALLGUARD(Z2)
                stepperZ2->sgt(tmc_sgt.Z);
              #endif
              #if AXIS_HAS_STALLGUARD(Z3)
                stepperZ3->sgt(tmc_sgt.Z);
              #endif
            #endif
          }
        #endif

      #endif // HAS_TRINAMIC

      eeprom_error = size_error(eeprom_index - (EEPROM_OFFSET));
      if (eeprom_error) {
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_MV("Index: ", int(eeprom_index - (EEPROM_OFFSET)));
          SERIAL_MV(" Size: ", datasize());
          SERIAL_EOL();
        #endif
      }
      else if (working_crc != stored_crc) {
        eeprom_error = true;
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_SMV(ER, "EEPROM CRC mismatch - (stored) ", stored_crc);
          SERIAL_MV(" != ", working_crc);
          SERIAL_EM(" (calculated)!");
        #endif
      }
      else if (!validating) {
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_ST(ECHO, version);
          SERIAL_MV(" Stored settings retrieved (", eeprom_index - (EEPROM_OFFSET));
          SERIAL_MV(" bytes; crc ", (uint32_t)working_crc);
          SERIAL_EM(")");
        #endif
      }

      if (!validating && !eeprom_error) post_process();

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        if (!validating) {
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
        }
      #endif

    }

    EEPROM_READ_FINISH();

    #if ENABLED(EEPROM_CHITCHAT)
      if (!validating) Print_Settings();
    #endif

    if (validating) sound.feedback(!eeprom_error);

    return !eeprom_error;
  }

  bool EEPROM::validate() {
    validating = true;
    const bool success = _load();
    validating = false;
    return success;
  }

  bool EEPROM::load() {
    if (validate()) return _load();
    reset();
    return false;
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

  bool EEPROM::store() { SERIAL_LM(ER, "EEPROM disabled"); return false; }

#endif // HAS_EEPROM

/**
 * M502 - Reset Configuration
 */
void EEPROM::reset() {

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

  // Reset sound mode
  sound.mode = SOUND_MODE_ON;

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    bedlevel.z_fade_height = 0.0f;
  #endif

  #if HAS_LEVELING
    bedlevel.reset();
  #endif

  #if HAS_BED_PROBE
    // Call Probe Factory parameters
    probe.factory_parameters();
  #endif

  #if HAS_LCD_MENU
    lcdui.preheat_hotend_temp[0] = PREHEAT_1_TEMP_HOTEND;
    lcdui.preheat_hotend_temp[1] = PREHEAT_2_TEMP_HOTEND;
    lcdui.preheat_hotend_temp[2] = PREHEAT_3_TEMP_HOTEND;
    lcdui.preheat_bed_temp[0] = PREHEAT_1_TEMP_BED;
    lcdui.preheat_bed_temp[1] = PREHEAT_2_TEMP_BED;
    lcdui.preheat_bed_temp[2] = PREHEAT_3_TEMP_BED;
    lcdui.preheat_fan_speed[0] = PREHEAT_1_FAN_SPEED;
    lcdui.preheat_fan_speed[1] = PREHEAT_2_FAN_SPEED;
    lcdui.preheat_fan_speed[2] = PREHEAT_3_FAN_SPEED;
  #endif

  #if HAS_LCD_CONTRAST
    lcdui.contrast = DEFAULT_LCD_CONTRAST;
  #endif

  #if HAS_SD_RESTART
    restart.enable(true);
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

    Heater        *heat;
    heater_data_t *data;
    pid_data_t    *pid;
    sensor_data_t *sens;

    #if HOTENDS > 0
      LOOP_HOTEND() {
        pid = &heaters[h].pid;
        pid->Kp  = pgm_read_float(&tmp1[h < COUNT(tmp1) ? h : COUNT(tmp1) - 1]);
        pid->Ki  = pgm_read_float(&tmp2[h < COUNT(tmp2) ? h : COUNT(tmp2) - 1]);
        pid->Kd  = pgm_read_float(&tmp3[h < COUNT(tmp3) ? h : COUNT(tmp3) - 1]);
        pid->Kc  = pgm_read_float(&tmp4[h < COUNT(tmp4) ? h : COUNT(tmp4) - 1]);
      }
    #endif

    #if HAS_HEATER_0
      // HOTEND 0
      heat  = &heaters[0];
      data  = &heat->data;
      sens  = &heat->sensor;
      pid   = &heat->pid;
      data->type            = IS_HOTEND;
      data->pin             = HEATER_0_PIN;
      data->ID              = 0;
      data->mintemp         = HEATER_0_MINTEMP;
      data->maxtemp         = HEATER_0_MAXTEMP;
      // Pid
      pid->DriveMin         = PID_DRIVE_MIN;
      pid->DriveMax         = PID_DRIVE_MAX;
      pid->Max              = PID_MAX;
      // Sensor
      sens->pin             = TEMP_0_PIN;
      sens->type            = TEMP_SENSOR_0;
      sens->r25             = HOT0_R25;
      sens->beta            = HOT0_BETA;
      sens->pullupR         = THERMISTOR_SERIES_RS;
      sens->shC             = 0;
      sens->adcLowOffset    = 0;
      sens->adcHighOffset   = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
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
      heat  = &heaters[1];
      data  = &heat->data;
      sens  = &heat->sensor;
      pid   = &heat->pid;
      data->type            = IS_HOTEND;
      data->pin             = HEATER_1_PIN;
      data->ID              = 1;
      data->mintemp         = HEATER_1_MINTEMP;
      data->maxtemp         = HEATER_1_MAXTEMP;
      // Pid
      pid->DriveMin         = PID_DRIVE_MIN;
      pid->DriveMax         = PID_DRIVE_MAX;
      pid->Max              = PID_MAX;
      // Sensor
      sens->pin             = TEMP_1_PIN;
      sens->type            = TEMP_SENSOR_1;
      sens->r25             = HOT1_R25;
      sens->beta            = HOT1_BETA;
      sens->pullupR         = THERMISTOR_SERIES_RS;
      sens->shC             = 0;
      sens->adcLowOffset    = 0;
      sens->adcHighOffset   = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
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
      heat  = &heaters[2];
      data  = &heat->data;
      sens  = &heat->sensor;
      pid   = &heat->pid;
      data->type            = IS_HOTEND;
      data->pin             = HEATER_2_PIN;
      data->ID              = 2;
      data->mintemp         = HEATER_2_MINTEMP;
      data->maxtemp         = HEATER_2_MAXTEMP;
      // Pid
      pid->DriveMin         = PID_DRIVE_MIN;
      pid->DriveMax         = PID_DRIVE_MAX;
      pid->Max              = PID_MAX;
      // Sensor
      sens->pin             = TEMP_2_PIN;
      sens->type            = TEMP_SENSOR_2;
      sens->r25             = HOT2_R25;
      sens->beta            = HOT2_BETA;
      sens->pullupR         = THERMISTOR_SERIES_RS;
      sens->shC             = 0;
      sens->adcLowOffset    = 0;
      sens->adcHighOffset   = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
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
      heat  = &heaters[3];
      data  = &heat->data;
      sens  = &heat->sensor;
      pid   = &heat->pid;
      data->type            = IS_HOTEND;
      data->pin             = HEATER_3_PIN;
      data->ID              = 3;
      data->mintemp         = HEATER_3_MINTEMP;
      data->maxtemp         = HEATER_3_MAXTEMP;
      // Pid
      pid->DriveMin         = PID_DRIVE_MIN;
      pid->DriveMax         = PID_DRIVE_MAX;
      pid->Max              = PID_MAX;
      // Sensor
      sens->pin             = TEMP_3_PIN;
      sens->type            = TEMP_SENSOR_3;
      sens->r25             = HOT3_R25;
      sens->beta            = HOT3_BETA;
      sens->pullupR         = THERMISTOR_SERIES_RS;
      sens->shC             = 0;
      sens->adcLowOffset    = 0;
      sens->adcHighOffset   = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
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
      heat  = &heaters[BED_INDEX];
      data  = &heat->data;
      sens  = &heat->sensor;
      pid   = &heat->pid;
      data->type            = IS_BED;
      data->pin             = HEATER_BED_PIN;
      data->ID              = BED_INDEX;
      data->mintemp         = BED_MINTEMP;
      data->maxtemp         = BED_MAXTEMP;
      // Pid
      pid->DriveMin         = PID_DRIVE_MIN;
      pid->DriveMax         = PID_DRIVE_MAX;
      pid->Max              = PID_MAX;
      pid->Kp               = DEFAULT_bedKp;
      pid->Ki               = DEFAULT_bedKi;
      pid->Kd               = DEFAULT_bedKd;
      // Sensor
      sens->pin             = TEMP_BED_PIN;
      sens->type            = TEMP_SENSOR_BED;
      sens->r25             = BED_R25;
      sens->beta            = BED_BETA;
      sens->pullupR         = THERMISTOR_SERIES_RS;
      sens->shC             = 0;
      sens->adcLowOffset    = 0;
      sens->adcHighOffset   = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
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
      heat  = &heaters[CHAMBER_INDEX];
      data  = &heat->data;
      sens  = &heat->sensor;
      pid   = &heat->pid;
      data->type            = IS_CHAMBER;
      data->pin             = HEATER_CHAMBER_PIN;
      data->ID              = CHAMBER_INDEX;
      data->mintemp         = CHAMBER_MINTEMP;
      data->maxtemp         = CHAMBER_MAXTEMP;
      // Pid
      pid->DriveMin         = PID_DRIVE_MIN;
      pid->DriveMax         = PID_DRIVE_MAX;
      pid->Max              = PID_MAX;
      pid->Kp               = DEFAULT_chamberKp;
      pid->Ki               = DEFAULT_chamberKi;
      pid->Kd               = DEFAULT_chamberKd;
      // Sensor
      sens->pin             = TEMP_CHAMBER_PIN;
      sens->type            = TEMP_SENSOR_CHAMBER;
      sens->r25             = CHAMBER_R25;
      sens->beta            = CHAMBER_BETA;
      sens->pullupR         = THERMISTOR_SERIES_RS;
      sens->shC             = 0;
      sens->adcLowOffset    = 0;
      sens->adcHighOffset   = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
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
      heat  = &heaters[COOLER_INDEX];
      data  = &heat->data;
      sens  = &heat->sensor;
      pid   = &heat->pid;
      data->type            = IS_COOLER;
      data->pin             = HEATER_COOLER_PIN;
      data->ID              = COOLER_INDEX;
      data->mintemp         = COOLER_MINTEMP;
      data->maxtemp         = COOLER_MAXTEMP;
      // Pid
      pid->DriveMin         = PID_DRIVE_MIN;
      pid->DriveMax         = PID_DRIVE_MAX;
      pid->Max              = PID_MAX;
      pid->Kp               = DEFAULT_coolerKp;
      pid->Ki               = DEFAULT_coolerKi;
      pid->Kd               = DEFAULT_coolerKd;
      // Sensor
      sens->pin             = TEMP_COOLER_PIN;
      sens->type            = TEMP_SENSOR_COOLER;
      sens->r25             = COOLER_R25;
      sens->beta            = COOLER_BETA;
      sens->pullupR         = THERMISTOR_SERIES_RS;
      sens->shC             = 0;
      sens->adcLowOffset    = 0;
      sens->adcHighOffset   = 0;
      #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
        sens->ad595_offset  = TEMP_SENSOR_AD595_OFFSET;
        sens->ad595_gain    = TEMP_SENSOR_AD595_GAIN;
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
    fan_data_t *fdata;

    LOOP_FAN() {
      fan   = &fans[f];
      fdata = &fan->data;
      fdata->ID                   = f;
      fdata->pin                  = (int8_t)pgm_read_byte(&tmp5[f]);
      fdata->freq                 = 250;
      fdata->min_Speed            = FAN_MIN_PWM;
      fdata->triggerTemperature   = HOTEND_AUTO_FAN_TEMPERATURE;
      fdata->autoMonitored        = 0;
      fdata->flag.all             = false;
      fan->setAutoMonitored((int8_t)pgm_read_byte(&tmp6[f]));
      fan->setHWInverted(FAN_INVERTED);
      #if ENABLED(TACHOMETRIC)
        fan->tacho.pin            = tacho_temp_pin[f];
      #endif
    }

  #endif

  #if HAS_FIL_RUNOUT_0
    filamentrunout.factory_parameters();
  #endif

  #if HAS_POWER_CHECK
    powerManager.factory_parameters();
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

  #if HAVE_DRV(TMC2130) || HAVE_DRV(TMC2208)
    tmc.current_init_to_defaults();
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
      advancedpause.data[e].unload_length = PAUSE_PARK_UNLOAD_LENGTH;
      advancedpause.data[e].load_length = PAUSE_PARK_FAST_LOAD_LENGTH;
    }
  #endif

  // Reset the watchdog
  watchdog.reset();

  post_process();

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

    endstops.print_parameters();

    #if HAS_LCD_MENU

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
      SERIAL_LM(CFG, "LCD Contrast");
      SERIAL_LMV(CFG, "  M250 C", lcdui.contrast);
    #endif

    #if HAS_SD_RESTART
      SERIAL_LM(CFG, "SD Restart Job");
      SERIAL_LMV(CFG, "  M413 S", int(restart.enabled));
    #endif

    #if HAS_SERVOS
      LOOP_SERVO() servo[s].print_parameters();
    #endif

    /**
     * Bed Leveling
     */
    #if HAS_LEVELING

      #if ENABLED(MESH_BED_LEVELING)
        SERIAL_LM(CFG, "Mesh Bed Leveling");
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        SERIAL_LM(CFG, "Unified Bed Leveling");
      #elif HAS_ABL
        SERIAL_LM(CFG, "Auto Bed Leveling");
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

    #if HAS_MULTI_ENDSTOP

      SERIAL_LM(CFG, "Endstop adjustment");
      SERIAL_SM(CFG, "  M666");
      #if ENABLED(X_TWO_ENDSTOPS)
        SERIAL_MV(" X", LINEAR_UNIT(endstops.x2_endstop_adj));
      #endif
      #if ENABLED(Y_TWO_ENDSTOPS)
        SERIAL_MV(" Y", LINEAR_UNIT(endstops.y2_endstop_adj));
      #endif
      #if ENABLED(Z_THREE_ENDSTOPS)
        SERIAL_EOL();
        SERIAL_LMV(CFG, "  M666 S2 Z", LINEAR_UNIT(endstops.z2_endstop_adj));
        SERIAL_SMV(CFG, "  M666 S3 Z", LINEAR_UNIT(endstops.z3_endstop_adj));
      #elif ENABLED(Z_TWO_ENDSTOPS)
        SERIAL_MV(" Z", LINEAR_UNIT(endstops.z2_endstop_adj));
      #endif
      SERIAL_EOL();

    #endif // [XYZ]_TWO_ENDSTOPS

    /**
     * Auto Bed Leveling
     */
    #if HAS_BED_PROBE
      SERIAL_SM(CFG, "Probe Offset");
      SERIAL_UNITS(true);
      SERIAL_SMV(CFG, "  M851 X", LINEAR_UNIT(probe.data.offset[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(probe.data.offset[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(probe.data.offset[Z_AXIS]), 3);
      SERIAL_EOL();
      SERIAL_LM(CFG, "Probe speed Fast and Slow [mm/min]");
      SERIAL_SMV(CFG, "  M851 F", probe.data.speed_fast);
      SERIAL_MV(" S", probe.data.speed_slow);
      SERIAL_EOL();
    #endif

    #if HAS_LCD_MENU
      SERIAL_LM(CFG, "Material heatup parameters");
      for (uint8_t i = 0; i < COUNT(lcdui.preheat_hotend_temp); i++) {
        SERIAL_SMV(CFG, "  M145 S", i);
        SERIAL_MV(" H", TEMP_UNIT(lcdui.preheat_hotend_temp[i]));
        SERIAL_MV(" B", TEMP_UNIT(lcdui.preheat_bed_temp[i]));
        SERIAL_MV(" F", lcdui.preheat_fan_speed[i]);
        SERIAL_EOL();
      }
    #endif // ULTIPANEL

    #if ENABLED(FWRETRACT)
      SERIAL_LM(CFG, "Retract: S<length> F<units/m> Z<lift>");
      SERIAL_SMV(CFG, "  M207 S", LINEAR_UNIT(fwretract.data.retract_length));
      SERIAL_MV(" W", LINEAR_UNIT(fwretract.data.swap_retract_length));
      SERIAL_MV(" F", MMS_TO_MMM(LINEAR_UNIT(fwretract.data.retract_feedrate_mm_s)));
      SERIAL_EMV(" Z", LINEAR_UNIT(fwretract.data.retract_zlift));

      SERIAL_LM(CFG, "Recover: S<length> F<units/m>");
      SERIAL_SMV(CFG, "  M208 S", LINEAR_UNIT(fwretract.data.retract_recover_length));
      SERIAL_MV(" W", LINEAR_UNIT(fwretract.data.swap_retract_recover_length));
      SERIAL_MV(" F", MMS_TO_MMM(LINEAR_UNIT(fwretract.data.retract_recover_feedrate_mm_s)));
      SERIAL_EMV(" R", MMS_TO_MMM(LINEAR_UNIT(fwretract.data.swap_retract_recover_feedrate_mm_s)));

      SERIAL_LM(CFG, "Auto-Retract: S=0 to disable, 1 to interpret E-only moves as retract/recover");
      SERIAL_LMV(CFG, "  M209 S", fwretract.autoretract_enabled ? 1 : 0);
    #endif // FWRETRACT

    #if ENABLED(VOLUMETRIC_EXTRUSION)

      /**
       * Volumetric extrusion M200
       */
      SERIAL_SM(CFG, "Filament settings");
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
    SERIAL_LM(CFG, "Stepper Direction");
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
    SERIAL_LM(CFG, "Stepper driver control");
    SERIAL_SMV(CFG, "  M569 D", stepper.direction_delay);
    SERIAL_MV(" P", stepper.minimum_pulse);
    SERIAL_MV(" R", stepper.maximum_rate);
    SERIAL_EOL();

    /**
     * Alligator current drivers M906
     */
    #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)

      SERIAL_LM(CFG, "Motor current (mA)");
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
      SERIAL_LM(CFG, "Stepper driver current (mA)");
      SERIAL_SM(CFG, "  M906");
      #if AXIS_HAS_TMC(X)
        SERIAL_MV(" X", stepperX->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(X2)
        SERIAL_MV(" I2 X", stepperX2->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(Y)
        SERIAL_MV(" Y", stepperY->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(Y2)
        SERIAL_MV(" I2 Y", stepperY2->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(Z)
        SERIAL_MV(" Z", stepperZ->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(Z2)
        SERIAL_MV(" I2 Z", stepperZ2->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(Z3)
        SERIAL_MV(" I3 Z", stepperZ3->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(E0)
        SERIAL_MV(" T0 E", stepperE0->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(E1)
        SERIAL_MV(" T1 E", stepperE1->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(E2)
        SERIAL_MV(" T2 E", stepperE2->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(E3)
        SERIAL_MV(" T3 E", stepperE3->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(E4)
        SERIAL_MV(" T4 E", stepperE4->getMilliamps());
      #endif
      #if AXIS_HAS_TMC(E5)
        SERIAL_MV(" T5 E", stepperE5->getMilliamps());
      #endif
      SERIAL_EOL();

      /**
       * TMC2130 or TMC2208 stepper driver microsteps
       */
      SERIAL_LM(CFG, "Stepper driver microsteps");
      SERIAL_SM(CFG, "  M350");
      #if AXIS_HAS_TMC(X)
        SERIAL_MV(" X", stepperX->microsteps());
      #endif
      #if AXIS_HAS_TMC(X2)
        SERIAL_MV(" I2 X", stepperX2->microsteps());
      #endif
      #if AXIS_HAS_TMC(Y)
        SERIAL_MV(" Y", stepperY->microsteps());
      #endif
      #if AXIS_HAS_TMC(Y2)
        SERIAL_MV(" I2 Y", stepperY2->microsteps());
      #endif
      #if AXIS_HAS_TMC(Z)
        SERIAL_MV(" Z", stepperZ->microsteps());
      #endif
      #if AXIS_HAS_TMC(Z2)
        SERIAL_MV(" I2 Z", stepperZ2->microsteps());
      #endif
      #if AXIS_HAS_TMC(Z3)
        SERIAL_MV(" I3 Z", stepperZ3->microsteps());
      #endif
      #if AXIS_HAS_TMC(E0)
        SERIAL_MV(" T0 E", stepperE0->microsteps());
      #endif
      #if AXIS_HAS_TMC(E1)
        SERIAL_MV(" T1 E", stepperE1->microsteps());
      #endif
      #if AXIS_HAS_TMC(E2)
        SERIAL_MV(" T2 E", stepperE2->microsteps());
      #endif
      #if AXIS_HAS_TMC(E3)
        SERIAL_MV(" T3 E", stepperE3->microsteps());
      #endif
      #if AXIS_HAS_TMC(E4)
        SERIAL_MV(" T4 E", stepperE4->microsteps());
      #endif
      #if AXIS_HAS_TMC(E5)
        SERIAL_MV(" T5 E", stepperE5->microsteps());
      #endif
      SERIAL_EOL();

      /**
       * TMC2130 or TMC2208 Hybrid Threshold
       */
      #if ENABLED(HYBRID_THRESHOLD)
        SERIAL_LM(CFG, "Hybrid Threshold");
        SERIAL_SM(CFG, "  M913");
        #if AXIS_HAS_TMC(X)
          SERIAL_MV(" X", TMC_GET_PWMTHRS(X, X));
        #endif
        #if AXIS_HAS_TMC(X2)
          SERIAL_MV(" I2 X", TMC_GET_PWMTHRS(X, X2));
        #endif
        #if AXIS_HAS_TMC(Y)
          SERIAL_MV(" Y", TMC_GET_PWMTHRS(Y, Y));
        #endif
        #if AXIS_HAS_TMC(Y2)
          SERIAL_MV(" I2 Y", TMC_GET_PWMTHRS(Y, Y2));
        #endif
        #if AXIS_HAS_TMC(Z)
          SERIAL_MV(" Z", TMC_GET_PWMTHRS(Z, Z));
        #endif
        #if AXIS_HAS_TMC(Z2)
          SERIAL_MV(" I2 Z", TMC_GET_PWMTHRS(Z, Z2));
        #endif
        #if AXIS_HAS_TMC(Z3)
          SERIAL_MV(" I3 Z", TMC_GET_PWMTHRS(Z, Z3));
        #endif
        #if AXIS_HAS_TMC(E0)
          SERIAL_MV(" T0 E", TMC_GET_PWMTHRS(E0, E0));
        #endif
        #if AXIS_HAS_TMC(E1)
          SERIAL_MV(" T1 E", TMC_GET_PWMTHRS(E1, E1));
        #endif
        #if AXIS_HAS_TMC(E2)
          SERIAL_MV(" T2 E", TMC_GET_PWMTHRS(E2, E2));
        #endif
        #if AXIS_HAS_TMC(E3)
          SERIAL_MV(" T3 E", TMC_GET_PWMTHRS(E3, E3));
        #endif
        #if AXIS_HAS_TMC(E4)
          SERIAL_MV(" T4 E", TMC_GET_PWMTHRS(E4, E4));
        #endif
        #if AXIS_HAS_TMC(E5)
          SERIAL_MV(" T5 E", TMC_GET_PWMTHRS(E5, E5));
        #endif
        SERIAL_EOL();
      #endif // HYBRID_THRESHOLD

      /**
       * TMC2130 StallGuard threshold
       */
      #if HAS_SENSORLESS
        SERIAL_LM(CFG, "TMC2130 StallGuard threshold:");
        SERIAL_SM(CFG, "  M914");
        #if X_HAS_SENSORLESS
          #if AXIS_HAS_STALLGUARD(X)
            SERIAL_MV(" X", stepperX->sgt());
          #endif
          #if AXIS_HAS_STALLGUARD(X2)
            SERIAL_MV(" I2 X", stepperX2->sgt());
          #endif
        #endif
        #if Y_HAS_SENSORLESS
          #if AXIS_HAS_STALLGUARD(Y)
            SERIAL_MV(" Y", stepperY->sgt());
          #endif
          #if AXIS_HAS_STALLGUARD(Y2)
            SERIAL_MV(" I2 Y", stepperY2->sgt());
          #endif
        #endif
        #if Z_HAS_SENSORLESS
          #if AXIS_HAS_STALLGUARD(Z)
            SERIAL_MV(" Z", stepperZ->sgt());
          #endif
          #if AXIS_HAS_STALLGUARD(Z2)
            SERIAL_MV(" I2 Z", stepperZ2->sgt());
          #endif
          #if AXIS_HAS_STALLGUARD(Z3)
            SERIAL_MV(" I3 Z", stepperZ3->sgt());
          #endif
        #endif
        SERIAL_EOL();
      #endif // HAS_SENSORLESS

    #endif // HAS_TRINAMIC

    /**
     * Linear Advance
     */
    #if ENABLED(LIN_ADVANCE)
      SERIAL_LM(CFG, "Linear Advance");
      SERIAL_LMV(CFG, "  M900 K", planner.extruder_advance_K);
    #endif

    /**
     * Hysteresis Feature
     */
    #if ENABLED(HYSTERESIS_FEATURE)
      SERIAL_LM(CFG, "Hysteresis Correction");
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
      SERIAL_LM(CFG, "Filament load/unload lengths");
      #if EXTRUDERS == 1
        SERIAL_SMV(CFG, "  M603 L", LINEAR_UNIT(advancedpause.data[0].load_length), 2);
        SERIAL_EMV(" U", LINEAR_UNIT(advancedpause.data[0].unload_length), 2);
      #else // EXTRUDERS != 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M603 T", (int)e);
          SERIAL_MV(" L", LINEAR_UNIT(advancedpause.data[e].load_length), 2);
          SERIAL_EMV(" U", LINEAR_UNIT(advancedpause.data[e].unload_length), 2);
        }
      #endif // EXTRUDERS != 1
    #endif // ADVANCED_PAUSE_FEATURE

    #if HAS_SD_SUPPORT
      card.print_settings();
    #endif

  }

#endif // !DISABLE_M503

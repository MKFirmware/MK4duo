/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

#pragma pack(push, 1)

/**
 * Current EEPROM Layout && Version
 *
 * Keep this data structure up to date so
 * EEPROM size is known at compile time!
 */
#define EEPROM_VERSION "MKV71"
#define EEPROM_OFFSET 100

typedef struct EepromDataStruct {

  char      version[6];   // MKVnn\0
  uint16_t  crc;          // Data Checksum

  //
  // Mechanics data
  //
  mechanics_data_t  mechanics_data;

  //
  // Endstop data
  //
  endstop_data_t    endstop_data;

  //
  // Stepper data
  //
  stepper_data_t    stepper_data;

  //
  // Tool data
  //
  tool_data_t       tool_data;

  //
  // Nozzle data
  //
  nozzle_data_t     nozzle_data;

  //
  // Sound data
  //
  sound_data_t      sound_data;

  //
  // Heaters data
  //
  #if HAS_HOTENDS
    heater_data_t   hotend_data[HOTENDS];
  #endif
  #if HAS_BEDS
    heater_data_t   bed_data[BEDS];
  #endif
  #if HAS_CHAMBERS
    heater_data_t   chamber_data[CHAMBERS];
  #endif
  #if HAS_COOLERS
    heater_data_t   cooler_data[COOLERS];
  #endif

  //
  // DHT sensor data
  //
  #if HAS_DHT
    dht_data_t      dht_data;
  #endif

  //
  // Fans data
  //
  #if HAS_FANS
    fan_data_t      fans_data[FAN_COUNT];
  #endif

  //
  // Filament Runout data
  //
  #if HAS_FILAMENT_SENSOR
    filament_data_t filrunout_data;
  #endif

  //
  // Power Check data
  //
  #if HAS_POWER_CHECK
    power_data_t    power_data;
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
    mbl_data_t      mbl_data;
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
  // Probe data
  //
  #if HAS_BED_PROBE
    probe_data_t    probe_data;
  #endif

  //
  // LCD menu
  //
  #if HAS_LCD_MENU
    #if HAS_HOTENDS
      int16_t       lcdui_preheat_hotend_temp[3];
    #endif
    #if HAS_BEDS  
      int16_t       lcdui_preheat_bed_temp[3];
    #endif
    #if HAS_CHAMBERS
      int16_t       lcdui_preheat_chamber_temp[3];
    #endif
    #if HAS_FANS
      int16_t       lcdui_preheat_fan_speed[3];
    #endif
  #endif

  //
  // LCD contrast
  //
  #if HAS_LCD_CONTRAST
    uint8_t         lcdui_contrast;
  #endif

  //
  // SD Restart
  //
  #if HAS_SD_RESTART
    bool            restart_enabled;
  #endif

  //
  // Servo angles
  //
  #if HAS_SERVOS
    int             servo_angles[NUM_SERVOS][2];
  #endif

  //
  // BLTOUCH
  //
  #if HAS_BLTOUCH
    bool bltouch_last_mode;
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
    bool            volumetric_enabled;
  #endif

  //
  // IDLE oozing
  //
  #if ENABLED(IDLE_OOZING_PREVENT)
    bool            IDLE_OOZING_enabled;
  #endif

  //
  // External DAC
  //
  #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
    uint16_t        motor_current[3 + DRIVER_EXTRUDERS];
  #endif

  //
  // Linear Advance
  //
  #if ENABLED(LIN_ADVANCE)
    float           planner_extruder_advance_K;
  #endif

  //
  // Hysteresis Feature
  //
  #if ENABLED(HYSTERESIS_FEATURE)
    hysteresis_data_t hysteresis_data;
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
    uint16_t  tmc_stepper_current[TMC_AXIS],
              tmc_stepper_microstep[TMC_AXIS];
    uint32_t  tmc_hybrid_threshold[TMC_AXIS];
    bool      tmc_stealth_enabled[TMC_AXIS];
    int16_t   tmc_sgt[XYZ];
  #endif

} eepromDataStruct;

EEPROM eeprom;

uint16_t EEPROM::datasize() { return sizeof(eepromDataStruct); }

/**
 * Post-process after Retrieve or Reset
 */
void EEPROM::post_process() {

  COPY_ARRAY(mechanics.stored_position[0], mechanics.current_position);

  // Recalculate pulse cycle
  HAL_calc_pulse_cycle();

  // steps per s2 needs to be updated to agree with units per s2
  planner.reset_acceleration_rates();

  // Make sure delta kinematics are updated before refreshing the
  // planner position so the stepper counts will be set correctly.
  #if MECH(DELTA)
    mechanics.recalc_delta_settings();
  #endif

  #if HAS_HOTENDS
    LOOP_HOTEND() hotends[h].init();
  #endif
  #if HAS_BEDS
    LOOP_BED() beds[h].init();
  #endif
  #if HAS_CHAMBERS
    LOOP_CHAMBER() chambers[h].init();
  #endif
  #if HAS_COOLERS
    LOOP_COOLER() coolers[h].init();
  #endif

  #if HAS_DHT
    dhtsensor.init();
  #endif

  #if HAS_FANS
    LOOP_FAN() fans[f].init();
  #endif

  #if ENABLED(VOLUMETRIC_EXTRUSION)
    tools.calculate_volumetric_multipliers();
  #else
    for (uint8_t i = COUNT(tools.e_factor); i--;)
      tools.refresh_e_factor(i);
  #endif

  // Software endstops depend on home_offset
  LOOP_XYZ(i) endstops.update_software_endstops((AxisEnum)i);

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
  #if HAS_FILAMENT_SENSOR
    filamentrunout.sensor.setup_pullup();
  #endif

  // Setup power check pullup
  #if HAS_POWER_CHECK
    powerManager.setup_pullup();
  #endif

  // Refresh steps_to_mm with the reciprocal of axis_steps_per_mm
  // and init stepper.count[], planner.position[] with current_position
  planner.refresh_positioning();

  if (memcmp(mechanics.stored_position[0], mechanics.current_position, sizeof(mechanics.stored_position[0])))
    mechanics.report_current_position();

}

#if HAS_EEPROM

  #define EEPROM_SKIP(VAR)        eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR)       memorystore.write_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ_ALWAYS(VAR) memorystore.read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ(VAR)        memorystore.read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc, !flag.validating)

  #define EEPROM_ASSERT(TST,ERR) do{ if (!(TST)) { SERIAL_LM(ER, ERR); flag.error = true; } }while(0)
  #define _FIELD_TEST(FIELD) \
    EEPROM_ASSERT( \
      flag.error || eeprom_index == offsetof(eepromDataStruct, FIELD) + EEPROM_OFFSET, \
      "Field " STRINGIFY(FIELD) " mismatch." \
    )

  const char version[6] = EEPROM_VERSION;

  eeprom_flag_t EEPROM::flag;

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

    char ver[6] = "ERROR";

    uint16_t working_crc = 0;

    int eeprom_index = EEPROM_OFFSET;

    flag.error = false;

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
    // Endstops data
    //
    EEPROM_WRITE(endstops.data);

    //
    // Stepper
    //
    EEPROM_WRITE(stepper.data);

    //
    // Tools Data
    //
    EEPROM_WRITE(tools.data);

    //
    // Nozzle Data
    //
    EEPROM_WRITE(nozzle.data);

    //
    // Sound
    //
    EEPROM_WRITE(sound.data);

    //
    // Heaters data
    //
    #if HAS_HOTENDS
      LOOP_HOTEND() EEPROM_WRITE(hotends[h].data);
    #endif
    #if HAS_BEDS
      LOOP_BED() EEPROM_WRITE(beds[h].data);
    #endif
    #if HAS_CHAMBERS
      LOOP_CHAMBER() EEPROM_WRITE(chambers[h].data);
    #endif
    #if HAS_COOLERS
      LOOP_COOLER() EEPROM_WRITE(coolers[h].data);
    #endif

    //
    // DHT sensor data
    //
    #if HAS_DHT
      EEPROM_WRITE(dhtsensor.data);
    #endif

    //
    // Fans data
    //
    #if HAS_FANS
      LOOP_FAN() EEPROM_WRITE(fans[f].data);
    #endif

    //
    // Filament Runout data
    //
    #if HAS_FILAMENT_SENSOR
      EEPROM_WRITE(filamentrunout.sensor.data);
    #endif

    //
    // PowerManager data
    //
    #if HAS_POWER_CHECK
      EEPROM_WRITE(powerManager.data);
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
        sizeof(mbl.data.z_values) == GRID_MAX_POINTS * sizeof(mbl.data.z_values[0][0]),
        "MBL Z array is the wrong size."
      );
      EEPROM_WRITE(mbl.data);
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
    // LCD menu
    //
    #if HAS_LCD_MENU
      #if HAS_HOTENDS
        EEPROM_WRITE(lcdui.preheat_hotend_temp);
      #endif
      #if HAS_BEDS
        EEPROM_WRITE(lcdui.preheat_bed_temp);
      #endif
      #if HAS_CHAMBERS
        EEPROM_WRITE(lcdui.preheat_chamber_temp);
      #endif
      #if HAS_FANS
        EEPROM_WRITE(lcdui.preheat_fan_speed);
      #endif
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
    // BLTOUCH
    //
    #if HAS_BLTOUCH
      EEPROM_WRITE(bltouch.last_mode);
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
    #endif

    //
    // IDLE oozing
    //
    #if ENABLED(IDLE_OOZING_PREVENT)
      EEPROM_WRITE(printer.IDLE_OOZING_enabled);
    #endif

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
      EEPROM_WRITE(hysteresis.data);
    #endif

    //
    // Advanced Pause data
    //
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      EEPROM_WRITE(advancedpause.data);
    #endif

    //
    // Save TMC2130 or TMC2208 Configuration, and placeholder values
    //
    #if HAS_TRINAMIC

      uint16_t  tmc_stepper_current[TMC_AXIS]   = { X_CURRENT, Y_CURRENT, Z_CURRENT, X_CURRENT, Y_CURRENT, Z_CURRENT, Z_CURRENT,
                                                    E0_CURRENT, E1_CURRENT, E2_CURRENT, E3_CURRENT, E4_CURRENT, E5_CURRENT },
                tmc_stepper_microstep[TMC_AXIS] = { X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS, X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS, Z_MICROSTEPS,
                                                    E0_MICROSTEPS, E1_MICROSTEPS, E2_MICROSTEPS, E3_MICROSTEPS, E4_MICROSTEPS, E5_MICROSTEPS };
      uint32_t  tmc_hybrid_threshold[TMC_AXIS]  = { X_HYBRID_THRESHOLD, Y_HYBRID_THRESHOLD, Z_HYBRID_THRESHOLD,
                                                    X_HYBRID_THRESHOLD, Y_HYBRID_THRESHOLD, Z_HYBRID_THRESHOLD, Z_HYBRID_THRESHOLD,
                                                    E0_HYBRID_THRESHOLD, E1_HYBRID_THRESHOLD, E2_HYBRID_THRESHOLD,
                                                    E3_HYBRID_THRESHOLD, E4_HYBRID_THRESHOLD, E5_HYBRID_THRESHOLD };
      bool      tmc_stealth_enabled[TMC_AXIS]   = { X_STEALTHCHOP, Y_STEALTHCHOP, Z_STEALTHCHOP, X_STEALTHCHOP, Y_STEALTHCHOP, Z_STEALTHCHOP, Z_STEALTHCHOP,
                                                    E0_STEALTHCHOP, E1_STEALTHCHOP, E2_STEALTHCHOP, E3_STEALTHCHOP, E4_STEALTHCHOP, E5_STEALTHCHOP };

      LOOP_TMC() {
        MKTMC* st = tmc.driver_by_index(t);
        if (st) {
          tmc_stepper_current[t]    = st->getMilliamps();
          tmc_stepper_microstep[t]  = st->microsteps();
          #if ENABLED(HYBRID_THRESHOLD)
            tmc_hybrid_threshold[t] = st->get_pwm_thrs();
          #endif
          #if TMC_HAS_STEALTHCHOP
            tmc_stealth_enabled[t]  = st->get_stealthChop_status();
          #endif
        }
      }

      EEPROM_WRITE(tmc_stepper_current);
      EEPROM_WRITE(tmc_stepper_microstep);
      EEPROM_WRITE(tmc_hybrid_threshold);
      EEPROM_WRITE(tmc_stealth_enabled);

      //
      // TMC2130 StallGuard threshold
      //
      int16_t tmc_sgt[XYZ] = { 0, 0, 0 };
      #if HAS_SENSORLESS
        #if X_HAS_SENSORLESS
          tmc_sgt[X_AXIS] = stepperX->homing_threshold();
        #endif
        #if Y_HAS_SENSORLESS
          tmc_sgt[Y_AXIS] = stepperY->homing_threshold();
        #endif
        #if Z_HAS_SENSORLESS
          tmc_sgt[Z_AXIS] = stepperZ->homing_threshold();
        #endif
      #endif
      EEPROM_WRITE(tmc_sgt);

    #endif // HAS_TRINAMIC

    //
    // Validate CRC and Data Size
    //
    if (!flag.error) {
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

      flag.error |= size_error(eeprom_size);
    }

    //
    // UBL Mesh
    //
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      if (ubl.storage_slot >= 0)
        store_mesh(ubl.storage_slot);
    #endif

    flag.error |= memorystore.access_write();

    sound.feedback(!flag.error);

    return !flag.error;
  }

  /**
   * M501 - Load Configuration
   */
  bool EEPROM::_load() {

    uint16_t  working_crc = 0,
              stored_crc  = 0;

    char stored_ver[6];

    int eeprom_index = EEPROM_OFFSET;

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
      flag.error = true;
    }
    else {

      working_crc = 0; // Init to 0. Accumulated by EEPROM_READ

      //
      // Mechanics data
      //
      EEPROM_READ(mechanics.data);

      //
      // Endstops data
      //
      EEPROM_READ(endstops.data);

      //
      // Stepper data
      //
      EEPROM_READ(stepper.data);

      //
      // Tools Data
      //
      EEPROM_READ(tools.data);

      //
      // Nozzle Data
      //
      EEPROM_READ(nozzle.data);

      //
      // Sound
      //
      EEPROM_READ(sound.data);

      //
      // Heaters data
      //
      #if HAS_HOTENDS
        LOOP_HOTEND() EEPROM_READ(hotends[h].data);
      #endif
      #if HAS_BEDS
        LOOP_BED() EEPROM_READ(beds[h].data);
      #endif
      #if HAS_CHAMBERS
        LOOP_CHAMBER() EEPROM_READ(chambers[h].data);
      #endif
      #if HAS_COOLERS
        LOOP_COOLER() EEPROM_READ(coolers[h].data);
      #endif

      //
      // DHT sensor data
      //
      #if HAS_DHT
        EEPROM_READ(dhtsensor.data);
      #endif

      //
      // Fans data
      //
      #if HAS_FANS
        LOOP_FAN() EEPROM_READ(fans[f].data);
      #endif

      //
      // Filament Runout data
      //
      #if HAS_FILAMENT_SENSOR
        EEPROM_READ(filamentrunout.sensor.data);
      #endif

      //
      // PowerManager data
      //
      #if HAS_POWER_CHECK
        EEPROM_READ(powerManager.data);
      #endif

      //
      // Z fade height
      //
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        EEPROM_READ(new_z_fade_height);
      #endif

      //
      // Mesh Bed Leveling
      //
      #if ENABLED(MESH_BED_LEVELING)
        EEPROM_READ(mbl.data);
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
          if (!flag.validating) bedlevel.set_bed_leveling_enabled(false);
          EEPROM_READ(abl.bilinear_grid_spacing); // 2 ints
          EEPROM_READ(abl.bilinear_start);        // 2 ints
          EEPROM_READ(abl.z_values);              // 9 to 256 floats
        }
        else { // EEPROM data is stale
          // Skip past disabled (or stale) Bilinear Grid data
          int bgs[2], bs[2];
          EEPROM_READ(bgs);
          EEPROM_READ(bs);
          float dummy = 0;
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
        if (!flag.validating)
          bedlevel.flag.leveling_active = bedlevel_leveling_active;
      #endif

      //
      // Probe data
      //
      #if HAS_BED_PROBE
        EEPROM_READ(probe.data);
      #endif

      //
      // LCD menu
      //
      #if HAS_LCD_MENU
        #if HAS_HOTENDS
          EEPROM_READ(lcdui.preheat_hotend_temp);
        #endif
        #if HAS_BEDS
          EEPROM_READ(lcdui.preheat_bed_temp);
        #endif
        #if HAS_CHAMBERS
          EEPROM_READ(lcdui.preheat_chamber_temp);
        #endif
        #if HAS_FANS
          EEPROM_READ(lcdui.preheat_fan_speed);
        #endif
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
      // BLTOUCH
      //
      #if HAS_BLTOUCH
        EEPROM_READ(bltouch.last_mode);
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
        if (!flag.validating) printer.setVolumetric(volumetric_enabled);
      #endif

      //
      // IDLE oozing
      //
      #if ENABLED(IDLE_OOZING_PREVENT)
        EEPROM_READ(printer.IDLE_OOZING_enabled);
      #endif

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
        EEPROM_READ(hysteresis.data);
      #endif

      //
      // Advanced Pause data
      //
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        EEPROM_READ(advancedpause.data);
      #endif

      if (!flag.validating) reset_stepper_drivers();

      //
      // TMC2130 or TMC2208 Stepper Current
      //
      #if HAS_TRINAMIC

        uint16_t  tmc_stepper_current[TMC_AXIS],
                  tmc_stepper_microstep[TMC_AXIS];
        uint32_t  tmc_hybrid_threshold[TMC_AXIS];
        bool      tmc_stealth_enabled[TMC_AXIS];

        EEPROM_READ(tmc_stepper_current);
        EEPROM_READ(tmc_stepper_microstep);
        EEPROM_READ(tmc_hybrid_threshold);
        EEPROM_READ(tmc_stealth_enabled);

        if (!flag.validating) {
          LOOP_TMC() {
            MKTMC* st = tmc.driver_by_index(t);
            if (st) {
              st->rms_current(tmc_stepper_current[t]);
              st->microsteps(tmc_stepper_microstep[t]);
              #if ENABLED(HYBRID_THRESHOLD)
                st->set_pwm_thrs(tmc_hybrid_threshold[t]);
              #endif
              #if TMC_HAS_STEALTHCHOP
                st->stealthChop_enabled = tmc_stealth_enabled[t];
                st->refresh_stepping_mode();
              #endif
            }
          }
        }

        /*
         * TMC2130 Sensorless homing threshold.
         * X and X2 use the same value
         * Y and Y2 use the same value
         * Z, Z2 and Z3 use the same value
         */
        int16_t tmc_sgt[XYZ];
        EEPROM_READ(tmc_sgt);
        #if HAS_SENSORLESS
          if (!flag.validating) {
            #if ENABLED(X_STALL_SENSITIVITY)
              #if AXIS_HAS_STALLGUARD(X)
                stepperX->homing_threshold(tmc_sgt[X_AXIS]);
              #endif
              #if AXIS_HAS_STALLGUARD(X2)
                stepperX2->homing_threshold(tmc_sgt[X_AXIS]);
              #endif
            #endif
            #if ENABLED(Y_STALL_SENSITIVITY)
              #if AXIS_HAS_STALLGUARD(Y)
                stepperY->homing_threshold(tmc_sgt[Y_AXIS]);
              #endif
              #if AXIS_HAS_STALLGUARD(Y2)
                stepperY2->homing_threshold(tmc_sgt[Y_AXIS]);
              #endif
            #endif
            #if ENABLED(Z_STALL_SENSITIVITY)
              #if AXIS_HAS_STALLGUARD(Z)
                stepperZ->homing_threshold(tmc_sgt[Z_AXIS]);
              #endif
              #if AXIS_HAS_STALLGUARD(Z2)
                stepperZ2->homing_threshold(tmc_sgt[Z_AXIS]);
              #endif
              #if AXIS_HAS_STALLGUARD(Z3)
                stepperZ3->homing_threshold(tmc_sgt[Z_AXIS]);
              #endif
            #endif
          }
        #endif

      #endif // HAS_TRINAMIC

      flag.error = size_error(eeprom_index - (EEPROM_OFFSET));
      if (flag.error) {
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_MV("Index: ", int(eeprom_index - (EEPROM_OFFSET)));
          SERIAL_MV(" Size: ", datasize());
          SERIAL_EOL();
        #endif
      }
      else if (working_crc != stored_crc) {
        flag.error = true;
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_SMV(ER, "EEPROM CRC mismatch - (stored) ", stored_crc);
          SERIAL_MV(" != ", working_crc);
          SERIAL_EM(" (calculated)!");
        #endif
      }
      else if (!flag.validating) {
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_ST(ECHO, version);
          SERIAL_MV(" Stored settings retrieved (", eeprom_index - (EEPROM_OFFSET));
          SERIAL_MV(" bytes; crc ", (uint32_t)working_crc);
          SERIAL_EM(")");
        #endif
      }

      if (!flag.validating && !flag.error) post_process();

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        if (!flag.validating) {

          ubl.report_state();

          if (!ubl.sanity_check()) {
            SERIAL_EOL();
            #if ENABLED(EEPROM_CHITCHAT)
              ubl.echo_name();
              SERIAL_EM(" initialized.");
            #endif
          }
          else {
            flag.error = true;
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

    #if ENABLED(EEPROM_CHITCHAT) && DISABLED(DISABLE_M503)
      if (!flag.validating) Print_Settings();
    #endif

    if (flag.validating) sound.feedback(!flag.error);

    return !flag.error;
  }

  bool EEPROM::validate() {
    flag.validating = true;
    const bool success = _load();
    flag.validating = false;
    return success;
  }

  bool EEPROM::load() {
    if (validate()) return _load();
    reset();
    #if ENABLED(EEPROM_AUTO_INIT)
      (void)store();
      SERIAL_EM("EEPROM Initialized");
    #endif
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

    uint16_t EEPROM::meshes_start_index() {
      return (datasize() + EEPROM_OFFSET + 32) & 0xFFF8;  // Pad the end of configuration data so it can float up
                                                          // or down a little bit without disrupting the mesh data
    }

    uint16_t EEPROM::calc_num_meshes() {
      return (meshes_end - meshes_start_index()) / sizeof(ubl.z_values);
    }

    int EEPROM::mesh_slot_offset(const int8_t slot) {
      return meshes_end - (slot + 1) * sizeof(ubl.z_values);
    }

    void EEPROM::store_mesh(const int8_t slot) {

      const int16_t a = calc_num_meshes();
      if (!WITHIN(slot, 0, a - 1)) {
        ubl_invalid_slot(a);
        DEBUG_MV("E2END=", (int)(memorystore.capacity() - 1));
        DEBUG_MV(" meshes_end=", (int)meshes_end);
        DEBUG_EMV(" slot=", slot);
        return;
      }

      uint16_t crc = 0;
      int pos = mesh_slot_offset(slot);

      const bool status = memorystore.write_data(pos, (uint8_t *)&ubl.z_values, sizeof(ubl.z_values), &crc);

      if (status) SERIAL_MSG("?Unable to save mesh data.\n");
      else        DEBUG_EMV("Mesh saved in slot ", slot);

    }

    void EEPROM::load_mesh(const int8_t slot, void * const into/*=NULL*/) {

      const int16_t a = calc_num_meshes();

      if (!WITHIN(slot, 0, a - 1)) {
        ubl_invalid_slot(a);
        return;
      }

      int pos = mesh_slot_offset(slot);
      uint16_t crc = 0;
      uint8_t * const dest = into ? (uint8_t*)into : (uint8_t*)&ubl.z_values;

      const bool status = memorystore.read_data(pos, dest, sizeof(ubl.z_values), &crc);

      if (status) SERIAL_MSG("?Unable to load mesh data.\n");
      else        DEBUG_EMV("Mesh loaded from slot ", slot);

    }

  #endif // AUTO_BED_LEVELING_UBL

#else // !HAS_EEPROM

  bool EEPROM::store() { SERIAL_LM(ER, "EEPROM disabled"); return false; }

#endif // HAS_EEPROM

/**
 * M502 - Reset Configuration
 */
void EEPROM::reset() {

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    new_z_fade_height = 0.0f;
  #endif

  // Call Mechanic Factory parameters
  mechanics.factory_parameters();

  // Call Stepper Factory parameters
  stepper.factory_parameters();

  // Call Planner Factory parameters
  planner.factory_parameters();

  // Call Endstop Factory parameters
  endstops.factory_parameters();

  // Call Tools Factory parameters
  tools.factory_parameters();

  // Call Nozzle Factory parameters
  nozzle.factory_parameters();

  // Call Temperature Factory parameters
  thermalManager.factory_parameters();

  // Call Printer Factory parameters
  printer.factory_parameters();

  // Call Sound Factory parameters
  sound.factory_parameters();

  #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
    externaldac.factory_parameters();
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    advancedpause.factory_parameters();
  #endif

  #if HAS_LEVELING
    bedlevel.factory_parameters();
  #endif

  #if ENABLED(MESH_BED_LEVELING)
    mbl.factory_parameters();
  #endif

  #if HAS_BED_PROBE
    probe.factory_parameters();
  #endif

  #if HAS_LCD
    lcdui.factory_parameters();
  #endif

  #if HAS_SD_RESTART
    restart.factory_parameters();
  #endif

  #if HAS_BLTOUCH
    bltouch.factory_parameters();
  #endif

  #if HAS_FILAMENT_SENSOR
    filamentrunout.sensor.factory_parameters();
  #endif

  #if HAS_POWER_CHECK
    powerManager.factory_parameters();
  #endif

  #if HAS_DHT
    dhtsensor.factory_parameters();
  #endif

  #if ENABLED(FWRETRACT)
    fwretract.factory_parameters();
  #endif

  #if ENABLED(HYSTERESIS_FEATURE)
    hysteresis.factory_parameters();
  #endif

  #if HAS_TRINAMIC
    tmc.factory_parameters();
  #endif

  reset_stepper_drivers();

  // Reset the watchdog
  watchdog.reset();

  post_process();

  SERIAL_LM(ECHO, "Factory Settings Loaded");

}

#if DISABLED(DISABLE_M503)

  inline void print_units(const bool colon) {
    SERIAL_STR(
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
    #if HAS_HOTENDS
      LOOP_HOTEND() {
        hotends[h].print_M305();
        hotends[h].print_M306();
        hotends[h].print_M301();
      }
    #endif
    #if HAS_BEDS
      LOOP_BED() {
        beds[h].print_M305();
        beds[h].print_M306();
        beds[h].print_M301();
      }
    #endif
    #if HAS_CHAMBERS
      LOOP_CHAMBER() {
        chambers[h].print_M305();
        chambers[h].print_M306();
        chambers[h].print_M301();
      }
    #endif
    #if HAS_COOLERS
      LOOP_COOLER() {
        coolers[h].print_M305();
        coolers[h].print_M306();
        coolers[h].print_M301();
      }
    #endif

    /**
     * Print dht parameters
     */
    #if HAS_DHT
      dhtsensor.print_M305();
    #endif

    /**
     * Print AD595 parameters
     */
    #if HAS_AD8495 || HAS_AD595
      LOOP_HOTEND() hotends[h].print_M595();
    #endif

    /**
     * Print Tools data
     */
    #if ENABLED(TOOL_CHANGE_FIL_SWAP)
      tools.print_M217();
    #endif

    /**
     * Print Nozzle data
     */
    #if ENABLED(NOZZLE_PARK_FEATURE) || EXTRUDERS > 1
      nozzle.print_M217();
    #endif

    /**
     * Print Hotends offsets parameters
     */
    #if HOTENDS > 1
      LOOP_HOTEND() nozzle.print_M218(h);
    #endif

    /**
     * Print Fans parameters
     */
    #if HAS_FANS
      LOOP_FAN() fans[f].print_M106();
    #endif

    endstops.print_parameters();

    #if HAS_LCD_MENU

      // Temperature units - for Ultipanel temperature options

      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        #define TEMP_UNIT(N) parser.to_temp_units(N)
        SERIAL_SM(CFG, "  M149 ");
        SERIAL_CHR(parser.temp_units_code());
        SERIAL_MSG(" ; Units in ");
        SERIAL_STR(parser.temp_units_name());
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
      LOOP_SERVO() servo[s].print_M281();
    #endif

    /**
     * Bed Leveling
     */
    #if HAS_LEVELING

      #if ENABLED(MESH_BED_LEVELING)
        SERIAL_LM(CFG, "Mesh Bed Leveling");
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        SERIAL_LM(CFG, "Unified Bed Leveling");
      #elif HAS_ABL_OR_UBL
        SERIAL_LM(CFG, "Auto Bed Leveling");
      #endif

      SERIAL_SMV(CFG, "  M420 S", bedlevel.leveling_is_valid() ? 1 : 0);
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        SERIAL_MV(" Z", LINEAR_UNIT(bedlevel.z_fade_height));
      #endif
      SERIAL_EOL();

      #if ENABLED(MESH_BED_LEVELING)

        if (bedlevel.leveling_is_valid()) {
          for (uint8_t iy = 0; iy < GRID_MAX_POINTS_Y; iy++) {
            for (uint8_t px = 0; px < GRID_MAX_POINTS_X; px++) {
              SERIAL_SMV(CFG, "  G29 S3 I", (int)px);
              SERIAL_MV(" J", (int)iy);
              SERIAL_EMV(" Z", LINEAR_UNIT(mbl.data.z_values[px][iy]), 3);
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
        SERIAL_MV(" X", LINEAR_UNIT(endstops.data.x2_endstop_adj));
      #endif
      #if ENABLED(Y_TWO_ENDSTOPS)
        SERIAL_MV(" Y", LINEAR_UNIT(endstops.data.y2_endstop_adj));
      #endif
      #if ENABLED(Z_THREE_ENDSTOPS)
        SERIAL_EOL();
        SERIAL_LMV(CFG, "  M666 S2 Z", LINEAR_UNIT(endstops.data.z2_endstop_adj));
        SERIAL_SMV(CFG, "  M666 S3 Z", LINEAR_UNIT(endstops.data.z3_endstop_adj));
      #elif ENABLED(Z_TWO_ENDSTOPS)
        SERIAL_MV(" Z", LINEAR_UNIT(endstops.data.z2_endstop_adj));
      #endif
      SERIAL_EOL();

    #endif // [XYZ]_TWO_ENDSTOPS

    /**
     * Auto Bed Leveling
     */
    #if HAS_BED_PROBE
      probe.print_M851();
    #endif

    #if HAS_LCD_MENU
      SERIAL_LM(CFG, "Material heatup parameters");
      for (uint8_t i = 0; i < COUNT(lcdui.preheat_hotend_temp); i++) {
        SERIAL_SMV(CFG, "  M145 S", i);
        #if HAS_HOTENDS
          SERIAL_MV(" H", TEMP_UNIT(lcdui.preheat_hotend_temp[i]));
        #endif
        #if HAS_BEDS
          SERIAL_MV(" B", TEMP_UNIT(lcdui.preheat_bed_temp[i]));
        #endif
        #if CHAMBER > 0
          SERIAL_MV(" C", TEMP_UNIT(lcdui.preheat_chamber_temp[i]));
        #endif
        #if HAS_FANS
          SERIAL_MV(" F", lcdui.preheat_fan_speed[i]);
        #endif
        SERIAL_EOL();
      }
    #endif // HAS_LCD_MENU

    #if ENABLED(FWRETRACT)
      SERIAL_LM(CFG, "Retract: S<length> F<units/m> W<swap lenght> Z<lift>");
      SERIAL_SMV(CFG, "  M207 S", LINEAR_UNIT(fwretract.data.retract_length));
      SERIAL_MV(" F", MMS_TO_MMM(LINEAR_UNIT(fwretract.data.retract_feedrate_mm_s)));
      SERIAL_MV(" W", LINEAR_UNIT(fwretract.data.swap_retract_length));
      SERIAL_EMV(" Z", LINEAR_UNIT(fwretract.data.retract_zlift));

      SERIAL_LM(CFG, "Recover: S<length> F<units/m> W<swap lenght> R<swap units/m>");
      SERIAL_SMV(CFG, "  M208 S", LINEAR_UNIT(fwretract.data.retract_recover_length));
      SERIAL_MV(" F", MMS_TO_MMM(LINEAR_UNIT(fwretract.data.retract_recover_feedrate_mm_s)));
      SERIAL_MV(" W", LINEAR_UNIT(fwretract.data.swap_retract_recover_length));
      SERIAL_EMV(" R", MMS_TO_MMM(LINEAR_UNIT(fwretract.data.swap_retract_recover_feedrate_mm_s)));

      SERIAL_LM(CFG, "Auto-Retract: S=0 to disable, 1 to interpret E-only moves as retract/recover");
      SERIAL_LMV(CFG, "  M209 S", fwretract.autoretract_enabled ? 1 : 0);
    #endif // FWRETRACT

    #if HAS_FILAMENT_SENSOR
      filamentrunout.print_M412();
    #endif

    #if ENABLED(VOLUMETRIC_EXTRUSION)
      tools.print_M200();
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
    SERIAL_SMV(CFG, "  M569 Q", stepper.data.quad_stepping);
    SERIAL_MV(" D", stepper.data.direction_delay);
    SERIAL_MV(" P", stepper.data.minimum_pulse);
    SERIAL_MV(" R", stepper.data.maximum_rate);
    SERIAL_EOL();

    /**
     * Alligator current drivers M906
     */
    #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)

      SERIAL_LM(CFG, "Stepper driver current (mA)");
      SERIAL_SM(CFG, "  M906");
      SERIAL_MV(" X", externaldac.motor_current[X_AXIS]);
      SERIAL_MV(" Y", externaldac.motor_current[Y_AXIS]);
      SERIAL_MV(" Z", externaldac.motor_current[Z_AXIS]);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", externaldac.motor_current[E_AXIS]);
      #endif
      SERIAL_EOL();
      #if DRIVER_EXTRUDERS > 1
        for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
          SERIAL_SM(CFG, "  M906");
          SERIAL_MV(" T", int(i));
          SERIAL_MV(" E", externaldac.motor_current[E_AXIS + i]);
          SERIAL_EOL();
        }
      #endif

    #endif // ALLIGATOR_R2 || ALLIGATOR_R3

    #if HAS_TRINAMIC

      // TMC2130 or TMC2208 stepper driver current
      tmc.print_M906();

      // TMC2130 or TMC2208 stepper driver microsteps
      tmc.print_M350();

      // TMC2130 or TMC2208 Hybrid Threshold
      tmc.print_M913();

      // TMC2130 StallGuard threshold
      tmc.print_M914();

      // TMC stepping mode
      tmc.print_M940();

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
      hysteresis.print_M99();
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

    print_job_counter.showStats();

  }

#endif // !DISABLE_M503

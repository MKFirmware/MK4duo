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
#include "sanitycheck.h"

// Check the integrity of data offsets.
// Can be disabled for production build.
//#define DEBUG_EEPROM_READWRITE

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
#define EEPROM_VERSION "MKV80"
#define EEPROM_OFFSET 100

typedef struct EepromDataStruct {

  char      version[6];   // MKVnn\0
  uint16_t  crc;          // Data Checksum

  //
  // ToolManager data
  //
  tool_data_t       tool_data;
  extruder_data_t   extruder_data[MAX_EXTRUDER];

  //
  // TempManager data
  //
  temp_data_t       temp_data;

  //
  // Mechanics data
  //
  mechanics_data_t  mechanics_data;

  //
  // Stepper data
  //
  stepper_data_t    stepper_data;

  //
  // Driver
  //
  driver_data_t     driver_data[MAX_DRIVER_XYZ];
  driver_data_t     driver_e_data[MAX_DRIVER_E];

  //
  // Endstop data
  //
  endstop_data_t    endstop_data;

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
    heater_data_t   hotend_data[MAX_HOTEND];
  #endif
  #if HAS_BEDS
    heater_data_t   bed_data[MAX_BED];
  #endif
  #if HAS_CHAMBERS
    heater_data_t   chamber_data[MAX_CHAMBER];
  #endif
  #if HAS_COOLERS
    heater_data_t   cooler_data[MAX_COOLER];
  #endif

  //
  // Fans data
  //
  fans_data_t     fans_data;
  fan_data_t      fan_data[MAX_FAN];

  //
  // DHT sensor data
  //
  #if HAS_DHT
    dht_data_t      dht_data;
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
    abl_data_t      abl_data;
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
  // LCD Language
  //
  #if HAS_LCD
    uint8_t lang;
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
    #if HAS_FAN
      uint8_t       lcdui_preheat_fan_speed[3];
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
    bool              bltouch_last_mode;
  #endif

  //
  // FWRETRACT
  //
  #if ENABLED(FWRETRACT)
    fwretract_data_t  fwretract_data;
    bool              autoretract_enabled;
  #endif

  //
  // IDLE oozing
  //
  #if ENABLED(IDLE_OOZING_PREVENT)
    bool            IDLE_OOZING_enabled;
  #endif

  //
  // Hysteresis Feature
  //
  #if ENABLED(HYSTERESIS_FEATURE)
    hysteresis_data_t hysteresis_data;
  #endif

  //
  // Trinamic
  //
  #if HAS_TRINAMIC
    uint16_t  tmc_stepper_current[MAX_DRIVER_XYZ],
              tmc_stepper_current_e[MAX_DRIVER_E],
              tmc_stepper_microstep[MAX_DRIVER_XYZ],
              tmc_stepper_microstep_e[MAX_DRIVER_E];
    uint32_t  tmc_hybrid_threshold[MAX_DRIVER_XYZ],
              tmc_hybrid_threshold_e[MAX_DRIVER_E];
    bool      tmc_stealth_enabled[MAX_DRIVER_XYZ],
              tmc_stealth_enabled_e[MAX_DRIVER_E];
    int16_t   tmc_sgt[XYZ];
  #endif

} eepromDataStruct;

EEPROM eeprom;

uint16_t EEPROM::datasize() { return sizeof(eepromDataStruct); }

/**
 * Post-process after Retrieve or Reset
 */
void EEPROM::post_process() {

  mechanics.stored_position[0] = mechanics.position;

  // Recalculate pulse cycle
  HAL_calc_pulse_cycle();

  // steps per s2 needs to be updated to agree with units per s2
  planner.reset_acceleration_rates();

  // Make sure delta kinematics are updated before refreshing the
  // planner position so the stepper counts will be set correctly.
  #if MECH(DELTA)
    mechanics.recalc_delta_settings();
  #endif

  tempManager.init();

  fanManager.init();

  #if HAS_DHT
    dhtsensor.init();
  #endif

  #if ENABLED(VOLUMETRIC_EXTRUSION)
    toolManager.calculate_volumetric_multipliers();
  #else
    LOOP_EXTRUDER() extruders[e]->refresh_e_factor();
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
  // and init stepper.count[], planner.position[] with position
  planner.refresh_positioning();

  if (mechanics.stored_position[0] != mechanics.position)
    mechanics.report_position();

}

#if HAS_EEPROM

  #define EEPROM_SKIP(VAR)        eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR)       memorystore.write_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ_ALWAYS(VAR) memorystore.read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ(VAR)        memorystore.read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc, !flag.validating)

  #if ENABLED(DEBUG_EEPROM_READWRITE)
    #define EEPROM_ASSERT(TST,ERR) do{ if (!(TST)) { SERIAL_LM(ER, ERR); flag.error = true; } }while(0)
    #define EEPROM_TEST(FIELD) \
      EEPROM_ASSERT( \
        flag.error || eeprom_index == offsetof(eepromDataStruct, FIELD) + EEPROM_OFFSET, \
        "Field " STRINGIFY(FIELD) " mismatch." \
      )
  #else
    #define EEPROM_TEST(FIELD)    NOOP
  #endif

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

    char ver[6]           = "ERROR";
    uint16_t working_crc  = 0;
    int eeprom_index      = EEPROM_OFFSET;

    driver_data_t driver_data[MAX_DRIVER_XYZ] = { { NoPin, NoPin, NoPin }, false };
    driver_data_t driver_e_data[MAX_DRIVER_E] = { { NoPin, NoPin, NoPin }, false };

    extruder_data_t extruder_data[MAX_EXTRUDER];
    heater_data_t   hotend_data[MAX_HOTEND];
    heater_data_t   bed_data[MAX_BED];
    heater_data_t   chamber_data[MAX_CHAMBER];
    heater_data_t   cooler_data[MAX_COOLER];
    fan_data_t      fan_data[MAX_FAN];

    if (memorystore.access_start()) {
      SERIAL_EM("No EEPROM.");
      return false;
    }

    flag.error = false;

    #if HAS_EEPROM_FLASH
      EEPROM_SKIP(ver);       // Flash doesn't allow rewriting without erase
    #else
      EEPROM_WRITE(ver);      // invalidate data first
    #endif
    EEPROM_SKIP(working_crc); // Skip the checksum slot

    working_crc = 0; // clear before first "real data"

    //
    // ToolManager data
    //
    EEPROM_TEST(tool_data);
    EEPROM_WRITE(toolManager.extruder);
    LOOP_EXTRUDER() if (extruders[e]) extruder_data[e] = extruders[e]->data;
    EEPROM_WRITE(extruder_data);

    //
    // TempManager data
    //
    EEPROM_TEST(temp_data);
    EEPROM_WRITE(tempManager.heater);

    // Mechanics data
    //
    EEPROM_TEST(mechanics_data);
    EEPROM_WRITE(mechanics.data);

    //
    // Stepper data
    //
    EEPROM_TEST(stepper_data);
    EEPROM_WRITE(stepper.data);

    //
    // Driver data
    //
    EEPROM_TEST(driver_data);
    LOOP_DRV_ALL_XYZ()  if (driver[d])    driver_data[d]    = driver[d]->data;
    LOOP_DRV_EXT()      if (driver.e[d])  driver_e_data[d]  = driver.e[d]->data;
    EEPROM_WRITE(driver_data);
    EEPROM_WRITE(driver_e_data);

    //
    // Endstops data
    //
    EEPROM_TEST(endstop_data);
    EEPROM_WRITE(endstops.data);

    //
    // Nozzle data
    //
    EEPROM_TEST(nozzle_data);
    EEPROM_WRITE(nozzle.data);

    //
    // Sound
    //
    EEPROM_TEST(sound_data);
    EEPROM_WRITE(sound.data);

    //
    // Heaters data
    //
    #if HAS_HOTENDS
      LOOP_HOTEND()   if (hotends[h])   hotend_data[h]  = hotends[h]->data;
      EEPROM_WRITE(hotend_data);
    #endif
    #if HAS_BEDS
      LOOP_BED()      if (beds[h])      bed_data[h]     = beds[h]->data;
      EEPROM_WRITE(bed_data);
    #endif
    #if HAS_CHAMBERS
      LOOP_CHAMBER()  if (chambers[h])  chamber_data[h] = chambers[h]->data;
      EEPROM_WRITE(chamber_data);
    #endif
    #if HAS_COOLERS
      LOOP_COOLER()   if (coolers[h])   cooler_data[h]  = coolers[h]->data;
      EEPROM_WRITE(cooler_data);
    #endif

    //
    // Fans data
    //
    EEPROM_TEST(fans_data);
    LOOP_FAN() if (fans[f]) fan_data[f] = fans[f]->data;
    EEPROM_WRITE(fanManager.data);
    EEPROM_WRITE(fan_data);

    //
    // DHT sensor data
    //
    #if HAS_DHT
      EEPROM_TEST(dht_data);
      EEPROM_WRITE(dhtsensor.data);
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
        sizeof(mbl.data.z_values) == (GRID_MAX_POINTS) * sizeof(mbl.data.z_values[0][0]),
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
        sizeof(abl.data.z_values) == (GRID_MAX_POINTS) * sizeof(abl.data.z_values[0][0]),
        "Bilinear Z array is the wrong size."
      );
      EEPROM_WRITE(abl.data);
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
      EEPROM_TEST(probe_data);
      EEPROM_WRITE(probe.data);
    #endif

    //
    // LCD Language
    //
    #if HAS_LCD
      EEPROM_WRITE(lcdui.lang);
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
      #if HAS_FAN
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
      EEPROM_TEST(restart_enabled);
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
      EEPROM_TEST(bltouch_last_mode);
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
    // IDLE oozing
    //
    #if ENABLED(IDLE_OOZING_PREVENT)
      EEPROM_WRITE(printer.IDLE_OOZING_enabled);
    #endif

    //
    // Hysteresis Feature
    //
    #if ENABLED(HYSTERESIS_FEATURE)
      EEPROM_WRITE(hysteresis.data);
    #endif

    //
    // Save Trinamic Driver Configuration, and placeholder values
    //
    #if HAS_TRINAMIC

      uint16_t  tmc_stepper_current[MAX_DRIVER_XYZ],
                tmc_stepper_current_e[MAX_DRIVER_E],
                tmc_stepper_microstep[MAX_DRIVER_XYZ],
                tmc_stepper_microstep_e[MAX_DRIVER_E];
      uint32_t  tmc_hybrid_threshold[MAX_DRIVER_XYZ],
                tmc_hybrid_threshold_e[MAX_DRIVER_E];
      bool      tmc_stealth_enabled[MAX_DRIVER_XYZ],
                tmc_stealth_enabled_e[MAX_DRIVER_E];

      LOOP_DRV_ALL_XYZ() {
        Driver* drv = driver[d];
        if (drv && drv->tmc) {
          tmc_stepper_current[d]    = drv->tmc->getMilliamps();
          tmc_stepper_microstep[d]  = drv->tmc->getMicrosteps();
          #if ENABLED(HYBRID_THRESHOLD)
            tmc_hybrid_threshold[d] = drv->tmc->get_pwm_thrs();
          #endif
          #if TMC_HAS_STEALTHCHOP
            tmc_stealth_enabled[d]  = drv->tmc->get_stealthChop_status();
          #endif
        }
      }
      LOOP_DRV_EXT() {
        Driver* drv = driver.e[d];
        if (drv && drv->tmc) {
          tmc_stepper_current_e[d]    = drv->tmc->getMilliamps();
          tmc_stepper_microstep_e[d]  = drv->tmc->getMicrosteps();
          #if ENABLED(HYBRID_THRESHOLD)
            tmc_hybrid_threshold_e[d] = drv->tmc->get_pwm_thrs_e();
          #endif
          #if TMC_HAS_STEALTHCHOP
            tmc_stealth_enabled_e[d]  = drv->tmc->get_stealthChop_status();
          #endif
        }
      }

      EEPROM_WRITE(tmc_stepper_current);
      EEPROM_WRITE(tmc_stepper_current_e);
      EEPROM_WRITE(tmc_stepper_microstep);
      EEPROM_WRITE(tmc_stepper_microstep_e);
      EEPROM_WRITE(tmc_hybrid_threshold);
      EEPROM_WRITE(tmc_hybrid_threshold_e);
      EEPROM_WRITE(tmc_stealth_enabled);
      EEPROM_WRITE(tmc_stealth_enabled_e);

      //
      // TMC2130 StallGuard threshold
      //
      int16_t tmc_sgt[XYZ] = { 0 };
      #if HAS_SENSORLESS
        #if X_HAS_SENSORLESS
          tmc_sgt[X_AXIS] = driver.x->tmc->homing_threshold();
        #endif
        #if Y_HAS_SENSORLESS
          tmc_sgt[Y_AXIS] = driver.y->tmc->homing_threshold();
        #endif
        #if Z_HAS_SENSORLESS
          tmc_sgt[Z_AXIS] = driver.z->tmc->homing_threshold();
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
   * M505 - Clear EEPROM and reset
   */
  void EEPROM::clear() {
    uint16_t temp_crc = 0;
    int eeprom_index = EEPROM_OFFSET;

    SERIAL_LM(ECHO, "Clear EEPROM and RESET!");

    while (eeprom_index <= EEPROM_SIZE)
      memorystore.write_data(eeprom_index, (uint8_t*)0XFF, 1, &temp_crc);

    // Reset Printer
    printer.setRunning(false);
    watchdog.enable(WDTO_15MS);
    while(1);
  }

  /**
   * M501 - Load Configuration
   */
  bool EEPROM::_load() {

    uint16_t  working_crc = 0,
              stored_crc  = 0;
    char      stored_ver[6];

    driver_data_t driver_data[MAX_DRIVER_XYZ] = { { NoPin, NoPin, NoPin }, false };
    driver_data_t driver_e_data[MAX_DRIVER_E] = { { NoPin, NoPin, NoPin }, false };

    extruder_data_t extruder_data[MAX_EXTRUDER];
    heater_data_t   hotend_data[MAX_HOTEND];
    heater_data_t   bed_data[MAX_BED];
    heater_data_t   chamber_data[MAX_CHAMBER];
    heater_data_t   cooler_data[MAX_COOLER];
    fan_data_t      fan_data[MAX_FAN];

    int eeprom_index = EEPROM_OFFSET;

    if (memorystore.access_start()) {
      SERIAL_EM("No EEPROM.");
      return false;
    }

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
      // ToolManager data
      //
      EEPROM_READ(toolManager.extruder);
      EEPROM_READ(extruder_data);
      if (!flag.validating) {
        toolManager.create_object();
        LOOP_EXTRUDER() if (extruders[e]) extruders[e]->data = extruder_data[e];
      }

      //
      // TempManager data
      //
      EEPROM_READ(tempManager.heater);
      if (!flag.validating) tempManager.create_object();

      //
      // Mechanics data
      //
      EEPROM_READ(mechanics.data);

      //
      // Stepper data
      //
      EEPROM_READ(stepper.data);

      //
      // Driver data
      //
      EEPROM_READ(driver_data);
      EEPROM_READ(driver_e_data);
      if (!flag.validating) {
        stepper.create_driver();  // Create driver stepper
        LOOP_DRV_ALL_XYZ()  if (driver[d])    driver[d]->data   = driver_data[d];
        LOOP_DRV_EXT()      if (driver.e[d])  driver.e[d]->data = driver_e_data[d];
      }

      //
      // Endstops data
      //
      EEPROM_READ(endstops.data);

      //
      // Nozzle data
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
        EEPROM_READ(hotend_data);
      #endif
      #if HAS_BEDS
        EEPROM_READ(bed_data);
      #endif
      #if HAS_CHAMBERS
        EEPROM_READ(chamber_data);
      #endif
      #if HAS_COOLERS
        EEPROM_READ(cooler_data);
      #endif
      if (!flag.validating) {
        #if HAS_HOTENDS
          LOOP_HOTEND()   if (hotends[h])   hotends[h]->data  = hotend_data[h];
        #endif
        #if HAS_BEDS
          LOOP_BED()      if (beds[h])      beds[h]->data     = bed_data[h];
        #endif
        #if HAS_CHAMBERS
          LOOP_CHAMBER()  if (chambers[h])  chambers[h]->data = chamber_data[h];
        #endif
        #if HAS_COOLERS
          LOOP_COOLER()   if (coolers[h])   coolers[h]->data  = cooler_data[h];
        #endif
      }

      //
      // Fans data
      //
      EEPROM_READ(fanManager.data);
      EEPROM_READ(fan_data);
      if (!flag.validating) {
        fanManager.create_object();
        LOOP_FAN() if (fans[f]) fans[f]->data = fan_data[f];
      }

      //
      // DHT sensor data
      //
      #if HAS_DHT
        EEPROM_READ(dhtsensor.data);
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
        EEPROM_READ(abl.data);
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
      // LCD Language
      //
      #if HAS_LCD
        EEPROM_READ(lcdui.lang);
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
        #if HAS_FAN
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
      // IDLE oozing
      //
      #if ENABLED(IDLE_OOZING_PREVENT)
        EEPROM_READ(printer.IDLE_OOZING_enabled);
      #endif

      //
      // Hysteresis Feature
      //
      #if ENABLED(HYSTERESIS_FEATURE)
        EEPROM_READ(hysteresis.data);
      #endif

      if (!flag.validating) stepper.reset_drivers();

      //
      // Trinamic Stepper data
      //
      #if HAS_TRINAMIC

        uint16_t  tmc_stepper_current[MAX_DRIVER_XYZ],
                  tmc_stepper_current_e[MAX_DRIVER_E],
                  tmc_stepper_microstep[MAX_DRIVER_XYZ],
                  tmc_stepper_microstep_e[MAX_DRIVER_E];
        uint32_t  tmc_hybrid_threshold[MAX_DRIVER_XYZ],
                  tmc_hybrid_threshold_e[MAX_DRIVER_E];
        bool      tmc_stealth_enabled[MAX_DRIVER_XYZ],
                  tmc_stealth_enabled_e[MAX_DRIVER_E];

        EEPROM_READ(tmc_stepper_current);
        EEPROM_READ(tmc_stepper_current_e);
        EEPROM_READ(tmc_stepper_microstep);
        EEPROM_READ(tmc_stepper_microstep_e);
        EEPROM_READ(tmc_hybrid_threshold);
        EEPROM_READ(tmc_hybrid_threshold_e);
        EEPROM_READ(tmc_stealth_enabled);
        EEPROM_READ(tmc_stealth_enabled_e);

        if (!flag.validating) {
          LOOP_DRV_ALL_XYZ() {
            Driver* drv = driver[d];
            if (drv && drv->tmc) {
              drv->tmc->rms_current(tmc_stepper_current[d]);
              drv->tmc->microsteps(tmc_stepper_microstep[d]);
              #if ENABLED(HYBRID_THRESHOLD)
                drv->tmc->set_pwm_thrs(tmc_hybrid_threshold[d]);
              #endif
              #if TMC_HAS_STEALTHCHOP
                drv->tmc->stealthChop_enabled = tmc_stealth_enabled[d];
                drv->tmc->refresh_stepping_mode();
              #endif
            }
          }
          LOOP_DRV_EXT() {
            Driver* drv = driver.e[d];
            if (drv && drv->tmc) {
              drv->tmc->rms_current(tmc_stepper_current_e[d]);
              drv->tmc->microsteps(tmc_stepper_microstep_e[d]);
              #if ENABLED(HYBRID_THRESHOLD)
                drv->tmc->set_pwm_thrs_e(tmc_hybrid_threshold_e[d]);
              #endif
              #if TMC_HAS_STEALTHCHOP
                drv->tmc->stealthChop_enabled = tmc_stealth_enabled_e[d];
                drv->tmc->refresh_stepping_mode();
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
                driver.x->tmc->homing_threshold(tmc_sgt[X_AXIS]);
              #endif
              #if AXIS_HAS_STALLGUARD(X2)
                driver.x2->tmc->homing_threshold(tmc_sgt[X_AXIS]);
              #endif
            #endif
            #if ENABLED(Y_STALL_SENSITIVITY)
              #if AXIS_HAS_STALLGUARD(Y)
                driver.y->tmc->homing_threshold(tmc_sgt[Y_AXIS]);
              #endif
              #if AXIS_HAS_STALLGUARD(Y2)
                driver.y2->tmc->homing_threshold(tmc_sgt[Y_AXIS]);
              #endif
            #endif
            #if ENABLED(Z_STALL_SENSITIVITY)
              #if AXIS_HAS_STALLGUARD(Z)
                driver.z->tmc->homing_threshold(tmc_sgt[Z_AXIS]);
              #endif
              #if AXIS_HAS_STALLGUARD(Z2)
                driver.z2->tmc->homing_threshold(tmc_sgt[Z_AXIS]);
              #endif
              #if AXIS_HAS_STALLGUARD(Z3)
                driver.z3->tmc->homing_threshold(tmc_sgt[Z_AXIS]);
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

  bool EEPROM::load() {
    if (validate()) return _load();
    reset();
    #if ENABLED(EEPROM_AUTO_INIT)
      (void)store();
      SERIAL_EM("EEPROM Initialized");
    #endif
    return false;
  }

  bool EEPROM::validate() {
    flag.validating = true;
    const bool success = _load();
    flag.validating = false;
    return success;
  }

  #if ENABLED(AUTO_BED_LEVELING_UBL)

    #if ENABLED(EEPROM_CHITCHAT)
      void ubl_invalid_slot(const int s) {
        SERIAL_EM("?Invalid slot.");
        SERIAL_VAL(s);
        SERIAL_EM(" mesh slots available.");
      }
    #else
      void ubl_invalid_slot(const int) { }
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

  bool eeprom_disabled() { SERIAL_LM(ER, "EEPROM disabled"); return false; }
  bool EEPROM::store() { return eeprom_disabled(); }
  void EEPROM::clear() { (void)eeprom_disabled(); }

#endif // HAS_EEPROM

/**
 * M502 - Reset Configuration
 */
void EEPROM::reset() {

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    new_z_fade_height = 0.0f;
  #endif

  // Call Printer Factory parameters
  printer.factory_parameters();

  // Call Stepper Factory parameters
  stepper.factory_parameters();

  // Call Tools Factory parameters
  toolManager.factory_parameters();

  // Call Temperature Factory parameters
  tempManager.factory_parameters();

  // Call Fans Factory parameters
  fanManager.factory_parameters();

  // Call Mechanic Factory parameters
  mechanics.factory_parameters();

  // Call Planner Factory parameters
  planner.factory_parameters();

  // Call Endstop Factory parameters
  endstops.factory_parameters();

  // Call Nozzle Factory parameters
  nozzle.factory_parameters();

  // Call Sound Factory parameters
  sound.factory_parameters();

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

  post_process();

  SERIAL_LM(ECHO, "Factory Settings Loaded");

  // Reset the watchdog
  watchdog.reset();

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
     * Print Number Extruder, Hotend, Bed, Chamber, Fan
     */
    printer.print_M353();

    /**
     * Print Hotends tools assignment
     */
    toolManager.print_M563();

    /**
     * Print Tool volumetric extrusion
     */
    #if ENABLED(VOLUMETRIC_EXTRUSION)
      toolManager.print_M200();
    #endif

    /**
     * Linear Advance
     */
    #if ENABLED(LIN_ADVANCE)
      toolManager.print_M900();
    #endif

    /**
     * Print heaters parameters
     */
    #if HAS_HOTENDS
      LOOP_HOTEND() {
        hotends[h]->print_M305();
        hotends[h]->print_M306();
        hotends[h]->print_M301();
      }
    #endif
    #if HAS_BEDS
      LOOP_BED() {
        beds[h]->print_M305();
        beds[h]->print_M306();
        beds[h]->print_M301();
      }
    #endif
    #if HAS_CHAMBERS
      LOOP_CHAMBER() {
        chambers[h]->print_M305();
        chambers[h]->print_M306();
        chambers[h]->print_M301();
      }
    #endif
    #if HAS_COOLERS
      LOOP_COOLER() {
        coolers[h]->print_M305();
        coolers[h]->print_M306();
        coolers[h]->print_M301();
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
      LOOP_HOTEND() hotends[h]->print_M595();
    #endif

    /**
     * Print Tool change filament swap
     */
    #if ENABLED(TOOL_CHANGE_FIL_SWAP)
      LOOP_EXTRUDER() extruders[e]->print_M217(e);
    #endif

    /**
     * Print Nozzle data
     */
    #if ENABLED(NOZZLE_PARK_FEATURE) || MAX_EXTRUDER > 1
      nozzle.print_M217();
    #endif

    /**
     * Print Hotends offsets parameters
     */
    #if MAX_HOTEND > 1
      nozzle.print_M218();
    #endif

    /**
     * Print Fans parameters
     */
    fanManager.print_parameters();

    /**
     * Print Endstops parameters
     */
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
              SERIAL_MV(" Z", LINEAR_UNIT(mbl.data.z_values[px][iy]), 5);
              SERIAL_EOL();
            }
          }
          SERIAL_LMV(CFG, "  G29 S4 Z", LINEAR_UNIT(mbl.data.z_offset), 5);
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
              SERIAL_MV(" Z", LINEAR_UNIT(abl.data.z_values[px][py]), 5);
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
        #if HAS_FAN
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

    /**
     * Driver Pins M352
     */
    stepper.print_M352();

    /**
     * Stepper driver control
     */
    stepper.print_M569();

    /**
     * Alligator current drivers M906
     */
    #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)

      SERIAL_LM(CFG, "Stepper driver current (mA)");
      SERIAL_SM(CFG, "  M906");
      SERIAL_MV(" X", driver.x->data.ma);
      SERIAL_MV(" Y", driver.y->data.ma);
      SERIAL_MV(" Z", driver.z->data.ma);
      SERIAL_EOL();
      LOOP_DRV_EXT() {
        SERIAL_SM(CFG, "  M906");
        SERIAL_MV(" T", int(d));
        SERIAL_MV(" E", driver.e[extruders[d]->get_driver()]->data.ma);
        SERIAL_EOL();
      }

    #endif // ALLIGATOR_R2 || ALLIGATOR_R3

    #if HAS_TRINAMIC

      // Trinamic stepper driver microsteps
      tmcManager.print_M350();

      // Trinamic stepper driver current
      tmcManager.print_M906();

      // Trinamic Hybrid Threshold
      tmcManager.print_M913();

      // TMC2130 StallGuard threshold
      tmcManager.print_M914();

      // Trinamic stepping mode
      tmcManager.print_M940();

    #endif // HAS_TRINAMIC

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
      LOOP_EXTRUDER() {
        SERIAL_SMV(CFG, "  M603 T", (int)e);
        SERIAL_MV(" L", LINEAR_UNIT(extruders[e]->data.load_length), 2);
        SERIAL_EMV(" U", LINEAR_UNIT(extruders[e]->data.unload_length), 2);
      }
    #endif // ADVANCED_PAUSE_FEATURE

    print_job_counter.showStats();

  }

#endif // !DISABLE_M503

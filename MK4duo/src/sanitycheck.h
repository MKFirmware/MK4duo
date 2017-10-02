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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

/**
 * Require gcc 4.7 or newer (first included with Arduino 1.8.2) for C++11 features.
 */
#if __cplusplus < 201103L
  #error "MK4duo requires C++11 support (gcc >= 4.7, Arduino IDE >= 1.8.2). Please upgrade your toolchain."
#endif

// Start check
#if DISABLED(SERIAL_PORT)
  #error DEPENDENCY ERROR: Missing setting SERIAL_PORT
#endif
#if DISABLED(BAUDRATE)
  #error DEPENDENCY ERROR: Missing setting BAUDRATE
#endif
#if DISABLED(STRING_CONFIG_H_AUTHOR)
  #define STRING_CONFIG_H_AUTHOR "(none, default config)"
#endif
#if DISABLED(MACHINE_UUID)
  #error DEPENDENCY ERROR: Missing setting MACHINE_UUID
#endif

// Board
#if DISABLED(MOTHERBOARD)
  #error DEPENDENCY ERROR: Missing setting MOTHERBOARD
#endif

// Mechanism
#if DISABLED(MECHANISM)
  #error DEPENDENCY ERROR: Missing setting MECHANISM
#endif

// Power supply
#if DISABLED(POWER_SUPPLY)
  #define POWER_SUPPLY 0
#endif

// Extruders
#if DISABLED(EXTRUDERS)
  #error DEPENDENCY ERROR: Missing setting EXTRUDERS
#endif
#if DISABLED(DRIVER_EXTRUDERS)
  #error DEPENDENCY ERROR: Missing setting DRIVER_EXTRUDERS
#endif

// Thermistor
#if DISABLED(TEMP_SENSOR_0)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_0
#endif
#if DISABLED(TEMP_SENSOR_1)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_1
#endif
#if DISABLED(TEMP_SENSOR_2)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_2
#endif
#if DISABLED(TEMP_SENSOR_3)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_3
#endif
#if DISABLED(TEMP_SENSOR_BED)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED
#endif
#if DISABLED(TEMP_SENSOR_CHAMBER)
  #error DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_CHAMBER
#endif
#if DISABLED(TEMP_SENSOR_COOLER)
  #error DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_COOLER
#endif
#if (THERMISTORHEATER_0 == 998) || (THERMISTORHEATER_1 == 998) || (THERMISTORHEATER_2 == 998) || (THERMISTORHEATER_3 == 998) || (THERMISTORBED == 998) || (THERMISTORCHAMBER == 998) || (THERMISTORCOOLER == 998) // User EXIST table
  #if DISABLED(DUMMY_THERMISTOR_998_VALUE)
    #define DUMMY_THERMISTOR_998_VALUE 25
  #endif
#endif
#if (THERMISTORHEATER_0 == 999) || (THERMISTORHEATER_1 == 999) || (THERMISTORHEATER_2 == 999) || (THERMISTORHEATER_3 == 999) || (THERMISTORBED == 999) || (THERMISTORCHAMBER == 999) || (THERMISTORCOOLER == 999)// User EXIST table
  #if DISABLED(DUMMY_THERMISTOR_999_VALUE)
    #define DUMMY_THERMISTOR_999_VALUE 25
  #endif
#endif

// Temperature
/**
 * Temperature defines
 */
#if ENABLED(TEMP_RESIDENCY_TIME)
  #if DISABLED(TEMP_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting TEMP_HYSTERESIS
  #endif
  #if DISABLED(TEMP_WINDOW)
    #error DEPENDENCY ERROR: Missing setting TEMP_WINDOW
  #endif
#endif
#if TEMP_SENSOR_0 != 0
  #if DISABLED(HEATER_0_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_0_MAXTEMP
  #endif
  #if DISABLED(HEATER_0_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_0_MINTEMP
  #endif
#endif
#if TEMP_SENSOR_1 != 0
  #if DISABLED(HEATER_1_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_1_MAXTEMP
  #endif
  #if DISABLED(HEATER_1_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_1_MINTEMP
  #endif
#endif
#if TEMP_SENSOR_2 != 0
  #if DISABLED(HEATER_2_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_2_MAXTEMP
  #endif
  #if DISABLED(HEATER_2_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_2_MINTEMP
  #endif
#endif
#if TEMP_SENSOR_3 != 0
  #if DISABLED(HEATER_3_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_3_MAXTEMP
  #endif
  #if DISABLED(HEATER_3_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_3_MINTEMP
  #endif
#endif
#if TEMP_SENSOR_BED != 0
  #if DISABLED(BED_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting BED_MAXTEMP
  #endif
  #if DISABLED(BED_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting BED_MINTEMP
  #endif
#endif
#if TEMP_SENSOR_CHAMBER != 0
  #if DISABLED(CHAMBER_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting CHAMBER_MAXTEMP
  #endif
  #if DISABLED(CHAMBER_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting CHAMBER_MINTEMP
  #endif
  #if !HAS_HEATER_CHAMBER
    #error DEPENDENCY ERROR: Cannot enable TEMP_SENSOR_CHAMBER and not HEATER_CHAMBER_PIN
  #endif
#endif
#if TEMP_SENSOR_COOLER != 0
  #if DISABLED(COOLER_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting COOLER_MAXTEMP
  #endif
  #if DISABLED(COOLER_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting COOLER_MINTEMP
  #endif
  #if !HAS_COOLER
    #error DEPENDENCY ERROR: Cannot enable TEMP_SENSOR_COOLER and not COOLER_PIN
  #endif
#endif
#if DISABLED(PREHEAT_1_TEMP_HOTEND)
  #error DEPENDENCY ERROR: Missing setting PREHEAT_1_TEMP_HOTEND
#endif
#if DISABLED(PREHEAT_1_TEMP_BED)
  #error DEPENDENCY ERROR: Missing setting PREHEAT_1_TEMP_BED
#endif
#if DISABLED(PREHEAT_1_FAN_SPEED)
  #error DEPENDENCY ERROR: Missing setting PREHEAT_1_FAN_SPEED
#endif
#if DISABLED(PREHEAT_2_TEMP_HOTEND)
  #error DEPENDENCY ERROR: Missing setting PREHEAT_2_TEMP_HOTEND
#endif
#if DISABLED(PREHEAT_2_TEMP_BED)
  #error DEPENDENCY ERROR: Missing setting PREHEAT_2_TEMP_BED
#endif
#if DISABLED(PREHEAT_2_FAN_SPEED)
  #error DEPENDENCY ERROR: Missing setting PREHEAT_2_FAN_SPEED
#endif
#if DISABLED(PREHEAT_3_TEMP_HOTEND)
  #error DEPENDENCY ERROR: Missing setting PREHEAT_3_TEMP_HOTEND
#endif
#if DISABLED(PREHEAT_3_TEMP_BED)
  #error DEPENDENCY ERROR: Missing setting PREHEAT_3_TEMP_BED
#endif
#if DISABLED(PREHEAT_3_FAN_SPEED)
  #error DEPENDENCY ERROR: Missing setting PREHEAT_3_FAN_SPEED
#endif

// Language
#if DISABLED(LCD_LANGUAGE)
  #error DEPENDENCY ERROR: Missing setting LCD_LANGUAGE
#endif

/// FEATURE

// Temperature
#if DISABLED(PID_MAX)
  #error DEPENDENCY ERROR: Missing setting PID_MAX
#endif
#if DISABLED(MAX_BED_POWER)
  #error DEPENDENCY ERROR: Missing setting MAX_BED_POWER
#endif
#if DISABLED(MAX_CHAMBER_POWER)
  #error DEPENDENCY ERROR: Missing setting MAX_CHAMBER_POWER
#endif
#if DISABLED(MAX_COOLER_POWER)
  #error DEPENDENCY ERROR: Missing setting MAX_COOLER_POWER
#endif
#if (PIDTEMP)
  #if DISABLED(PID_OPENLOOP) && DISABLED(PID_FUNCTIONAL_RANGE) 
    #error DEPENDENCY ERROR: Missing setting PID_FUNCTIONAL_RANGE
  #endif
  #if DISABLED(DEFAULT_Kp)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_Kp
  #endif
  #if DISABLED(DEFAULT_Ki)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_Ki
  #endif
  #if DISABLED(DEFAULT_Kd)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_Kd
  #endif
#endif
#if (PIDTEMPBED)
  #if !HAS_TEMP_BED
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED
  #endif
  #if DISABLED(DEFAULT_bedKp)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKp
  #endif
  #if DISABLED(DEFAULT_bedKi)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKi
  #endif
  #if DISABLED(DEFAULT_bedKd)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKd
  #endif
#endif
#if (PIDTEMPCHAMBER)
  #if !HAS_TEMP_CHAMBER
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_CHAMBER
  #endif
  #if DISABLED(DEFAULT_chamberKp)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_chamberKp
  #endif
  #if DISABLED(DEFAULT_chamberKi)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_chamberKi
  #endif
  #if DISABLED(DEFAULT_chamberKd)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_chamberKd
  #endif

#endif
#if (PIDTEMPCOOLER)
  #if !HAS_TEMP_COOLER
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_COOLER
  #endif
  #if DISABLED(DEFAULT_coolerKp)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_coolerKp
  #endif
  #if DISABLED(DEFAULT_coolerKi)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_coolerKi
  #endif
  #if DISABLED(DEFAULT_coolerKd)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_coolerKd
  #endif
#endif
#if ENABLED(BED_LIMIT_SWITCHING)
  #if DISABLED(BED_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting BED_HYSTERESIS
  #endif
  #if DISABLED(BED_CHECK_INTERVAL)
    #error DEPENDENCY ERROR: Missing setting BED_CHECK_INTERVAL
  #endif
#endif
#if ENABLED(CHAMBER_LIMIT_SWITCHING)
  #if DISABLED(CHAMBER_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting CHAMBER_HYSTERESIS
  #endif
  #if DISABLED(CHAMBER_CHECK_INTERVAL)
    #error DEPENDENCY ERROR: Missing setting CHAMBER_CHECK_INTERVAL
  #endif
#endif
#if ENABLED(COOLER_LIMIT_SWITCHING)
  #if DISABLED(COOLER_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting COOLER_HYSTERESIS
  #endif
  #if DISABLED(COOLER_CHECK_INTERVAL)
    #error DEPENDENCY ERROR: Missing setting COOLER_CHECK_INTERVAL
  #endif
#endif
#if THERMAL_PROTECTION_HOTENDS || THERMAL_PROTECTION_BED || THERMAL_PROTECTION_CHAMBER || THERMAL_PROTECTION_COOLER
  #if DISABLED(THERMAL_PROTECTION_PERIOD)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_PERIOD
  #endif
  #if DISABLED(THERMAL_PROTECTION_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_HYSTERESIS
  #endif
  #if DISABLED(WATCH_TEMP_PERIOD)
    #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_PERIOD
  #endif
  #if DISABLED(WATCH_TEMP_INCREASE)
    #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_INCREASE
  #endif
#endif

// Fan
#if ENABLED(CONTROLLERFAN)
  #if DISABLED(CONTROLLERFAN_SECS)
    #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SECS
  #endif
  #if DISABLED(CONTROLLERFAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SPEED
  #endif
  #if DISABLED(CONTROLLERFAN_MIN_SPEED)
    #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_MIN_SPEED
  #endif
#endif

#if ENABLED(HOTEND_AUTO_FAN)
  #if DISABLED(HOTEND_AUTO_FAN_TEMPERATURE)
    #error DEPENDENCY ERROR: Missing setting HOTEND_AUTO_FAN_TEMPERATURE
  #endif
  #if DISABLED(HOTEND_AUTO_FAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting HOTEND_AUTO_FAN_SPEED
  #endif
  #if DISABLED(HOTEND_AUTO_FAN_MIN_SPEED)
    #error DEPENDENCY ERROR: Missing setting HOTEND_AUTO_FAN_MIN_SPEED
  #endif
#endif

// Extruder
#if ENABLED(PREVENT_COLD_EXTRUSION)
  #if DISABLED(EXTRUDE_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting EXTRUDE_MINTEMP
  #endif
  #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
    #if DISABLED(EXTRUDE_MAXLENGTH)
      #error DEPENDENCY ERROR: Missing setting EXTRUDE_MAXLENGTH
    #endif
  #endif
#endif

#if ENABLED(COLOR_MIXING_EXTRUDER)
  #if EXTRUDERS > 1
    #error COLOR_MIXING_EXTRUDER supports plus one extruder.
  #endif
  #if MIXING_STEPPERS < 2
    #error You must set MIXING_STEPPERS >= 2 for a mixing extruder.
  #endif
  #if ENABLED(FILAMENT_SENSOR)
    #error COLOR_MIXING_EXTRUDER is incompatible with FILAMENT_SENSOR. Comment out this line to use it anyway.
  #endif
#endif

#if ENABLED(NPR2)
  #if DISABLED(COLOR_STEP)
    #error DEPENDENCY ERROR: Missing setting COLOR_STEP
  #endif
  #if DISABLED(COLOR_SLOWRATE)
    #error DEPENDENCY ERROR: Missing setting COLOR_SLOWRATE
  #endif
  #if DISABLED(COLOR_HOMERATE)
    #error DEPENDENCY ERROR: Missing setting COLOR_HOMERATE
  #endif
  #if DISABLED(MOTOR_ANGLE)
    #error DEPENDENCY ERROR: Missing setting MOTOR_ANGLE
  #endif
  #if DISABLED(DRIVER_MICROSTEP)
    #error DEPENDENCY ERROR: Missing setting DRIVER_MICROSTEP
  #endif
  #if DISABLED(CARTER_MOLTIPLICATOR)
    #error DEPENDENCY ERROR: Missing setting CARTER_MOLTIPLICATOR
  #endif
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  #if DISABLED(IDLE_OOZING_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_MINTEMP
  #endif
  #if DISABLED(IDLE_OOZING_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_FEEDRATE
  #endif
  #if DISABLED(IDLE_OOZING_SECONDS)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_SECONDS
  #endif
  #if DISABLED(IDLE_OOZING_LENGTH)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_LENGTH
  #endif
  #if DISABLED(IDLE_OOZING_RECOVER_LENGTH)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_LENGTH
  #endif
  #if DISABLED(IDLE_OOZING_RECOVER_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_FEEDRATE
  #endif
#endif

#if ENABLED(EXTRUDER_RUNOUT_PREVENT)
  #if DISABLED(EXTRUDER_RUNOUT_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_MINTEMP
  #endif
  #if DISABLED(EXTRUDER_RUNOUT_SECONDS)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SECONDS
  #endif
  #if DISABLED(EXTRUDER_RUNOUT_SPEED)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SPEED
  #endif
  #if DISABLED(EXTRUDER_RUNOUT_EXTRUDE)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_EXTRUDE
  #endif
#endif

#if ENABLED(EASY_LOAD)
  #if DISABLED(BOWDEN_LENGTH)
    #error DEPENDENCY ERROR: Missing setting BOWDEN_LENGTH
  #endif
  #if DISABLED(LCD_PURGE_LENGTH)
    #error DEPENDENCY ERROR: Missing setting LCD_PURGE_LENGTH
  #endif
  #if DISABLED(LCD_RETRACT_LENGTH)
    #error DEPENDENCY ERROR: Missing setting LCD_RETRACT_LENGTH
  #endif
  #if DISABLED(LCD_PURGE_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting LCD_PURGE_FEEDRATE
  #endif
  #if DISABLED(LCD_RETRACT_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting LCD_RETRACT_FEEDRATE
  #endif
  #if DISABLED(LCD_LOAD_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting LCD_LOAD_FEEDRATE
  #endif
  #if DISABLED(LCD_UNLOAD_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting LCD_UNLOAD_FEEDRATE
  #endif
#endif

/**
 * Advance Extrusion
 */
#if ENABLED(ADVANCE) && ENABLED(LIN_ADVANCE)
  #error You can enable ADVANCE or LIN_ADVANCE, but not both.
#endif
#if ENABLED(ADVANCE)
  #if DISABLED(EXTRUDER_ADVANCE_K)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_ADVANCE_K
  #endif
  #if DISABLED(D_FILAMENT)
    #error DEPENDENCY ERROR: Missing setting D_FILAMENT
  #endif
#endif

/**
 * Progress Bar
 */
#if ENABLED(LCD_PROGRESS_BAR)
  #if DISABLED(SDSUPPORT)
    #error "LCD_PROGRESS_BAR requires SDSUPPORT."
  #elif ENABLED(DOGLCD)
    #error "LCD_PROGRESS_BAR does not apply to graphical displays."
  #elif ENABLED(FILAMENT_LCD_DISPLAY)
    #error "LCD_PROGRESS_BAR and FILAMENT_LCD_DISPLAY are not fully compatible. Comment out this line to use both."
  #endif
#endif

/**
 * Delta requirements
 */
#if MECH(DELTA)
  #if ABL_GRID
    #if (GRID_MAX_POINTS_X & 1) == 0  || (GRID_MAX_POINTS_Y & 1) == 0
      #error "DELTA requires GRID_MAX_POINTS_X and GRID_MAX_POINTS_Y to be odd numbers."
    #elif GRID_MAX_POINTS_X < 3 || GRID_MAX_POINTS_Y < 3
      #error "DELTA requires GRID_MAX_POINTS_X and GRID_MAX_POINTS_Y to be 3 or higher."
    #endif
  #endif

  static_assert(1 >= 0
    #if ENABLED(DELTA_AUTO_CALIBRATION_1)
      +1
    #endif
    #if ENABLED(DELTA_AUTO_CALIBRATION_2)
      +1
    #endif
    , "Select only one of: DELTA_AUTO_CALIBRATION_1 or DELTA_AUTO_CALIBRATION_2"
  );
#endif

/**
 * Babystepping
 */
#if ENABLED(BABYSTEPPING)
  #if DISABLED(ULTRA_LCD)
    #error "BABYSTEPPING requires an LCD controller."
  #endif
  #if IS_SCARA
    #error "BABYSTEPPING is not implemented for SCARA yet."
  #endif
  #if MECH(DELTA) && ENABLED(BABYSTEP_XY)
    #error "BABYSTEPPING only implemented for Z axis on deltabots."
  #endif
#endif

/**
 * Allow only one bed leveling option to be defined
 */
static_assert(1 >= 0
  #if ENABLED(AUTO_BED_LEVELING_LINEAR)
    + 1
  #endif
  #if ENABLED(AUTO_BED_LEVELING_3POINT)
    + 1
  #endif
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    + 1
  #endif
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    + 1
  #endif
  #if ENABLED(MESH_BED_LEVELING)
    + 1
  #endif
  , "Select only one of: MESH_BED_LEVELING, AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT, AUTO_BED_LEVELING_BILINEAR or AUTO_BED_LEVELING_UBL."
);

/**
 * Bed Leveling Requirements
 */

#if ENABLED(AUTO_BED_LEVELING_UBL)

  /**
   * Unified Bed Leveling
   */

  #if IS_SCARA
    #error "AUTO_BED_LEVELING_UBL does not yet support SCARA printers."
  #elif DISABLED(EEPROM_SETTINGS)
    #error "AUTO_BED_LEVELING_UBL requires EEPROM_SETTINGS. Please update your configuration."
  #elif !WITHIN(GRID_MAX_POINTS_X, 3, 15) || !WITHIN(GRID_MAX_POINTS_Y, 3, 15)
    #error "GRID_MAX_POINTS_[XY] must be a whole number between 3 and 15."
  #endif
#endif

/**
 * Mesh Bed Leveling
 */
#if ENABLED(MESH_BED_LEVELING)
  #if IS_DELTA
    #error "MESH_BED_LEVELING does not yet support DELTA printers."
  #elif IS_SCARA
    #error "Only AUTO_BED_LEVELING_BILINEAR currently supports SCARA bed leveling."
  #elif GRID_MAX_POINTS_X > 9 || GRID_MAX_POINTS_Y > 9
    #error "GRID_MAX_POINTS_X and GRID_MAX_POINTS_Y must be less than 10."
  #endif
#endif

/**
 * Probes
 */
#if PROBE_SELECTED

  /**
   * Allow only one probe option to be defined
   */
  static_assert(1 >= 0
    #if ENABLED(PROBE_MANUALLY)
      + 1
    #endif
    #if ENABLED(FIX_MOUNTED_PROBE)
      + 1
    #endif
    #if HAS_Z_SERVO_PROBE && DISABLED(BLTOUCH)
      + 1
    #endif
    #if ENABLED(BLTOUCH)
      + 1
    #endif
    #if ENABLED(Z_PROBE_ALLEN_KEY)
      + 1
    #endif
    #if ENABLED(Z_PROBE_SLED)
      + 1
    #endif
    , "Please enable only one probe: FIX_MOUNTED_PROBE, Z Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, or Z_PROBE_SLED."
  );

  /**
   * Z_PROBE_SLED is incompatible with DELTA
   */
  #if ENABLED(Z_PROBE_SLED) && MECH(DELTA)
    #error "You cannot use Z_PROBE_SLED with DELTA."
  #endif

  /**
   * NUM_SERVOS is required for a Z servo probe
   */
  #if HAS_Z_SERVO_PROBE
    #ifndef NUM_SERVOS
      #error "You must set NUM_SERVOS for a Z servo probe (Z_ENDSTOP_SERVO_NR)."
    #elif Z_ENDSTOP_SERVO_NR >= NUM_SERVOS
      #error "Z_ENDSTOP_SERVO_NR must be less than NUM_SERVOS."
    #endif
  #endif

  /**
   * A probe needs a pin
   */
  #if DISABLED(PROBE_MANUALLY) && !PROBE_PIN_CONFIGURED
    #error "A probe needs a pin! Use Z_MIN_PIN or Z_PROBE_PIN."
  #endif

  /**
   * Make sure Z raise values are set
   */
  #if DISABLED(Z_PROBE_DEPLOY_HEIGHT)
    #error "You must define Z_PROBE_DEPLOY_HEIGHT in your configuration."
  #elif DISABLED(Z_PROBE_BETWEEN_HEIGHT)
    #error "You must define Z_PROBE_BETWEEN_HEIGHT in your configuration."
  #elif Z_PROBE_DEPLOY_HEIGHT < 0
    #error "Probes need Z_PROBE_DEPLOY_HEIGHT >= 0."
  #elif Z_PROBE_BETWEEN_HEIGHT < 0
    #error "Probes need Z_PROBE_BETWEEN_HEIGHT >= 0."
  #endif

  #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(PROBE_MANUALLY)
    #error "Unified Bed Leveling requires a probe: FIX_MOUNTED_PROBE, BLTOUCH, SOLENOID_PROBE, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z Servo."
  #endif

#else

  /**
   * Require some kind of probe for bed leveling and probe testing
   */
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    #error "Unified Bed Leveling requires a probe: FIX_MOUNTED_PROBE, BLTOUCH, SOLENOID_PROBE, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z Servo."
  #elif HAS_ABL
    #error "Auto Bed Leveling requires a probe! Define a PROBE_MANUALLY, Z Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
  #elif ENABLED(DELTA_AUTO_CALIBRATION_1)
    #error "DELTA_AUTO_CALIBRATION_1 requires a probe! Define a Z PROBE_MANUALLY, Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
  #elif ENABLED(DELTA_AUTO_CALIBRATION_2)
    #error "DELTA_AUTO_CALIBRATION_2 requires a probe! Define a Z PROBE_MANUALLY, Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
  #endif

#endif

#if (!HAS_BED_PROBE || ENABLED(PROBE_MANUALLY)) && ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
  #error "Z_MIN_PROBE_REPEATABILITY_TEST requires a probe! Define a Z Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
#endif

/**
 * Homing Bump
 */
#if X_HOME_BUMP_MM < 0 || Y_HOME_BUMP_MM < 0 || Z_HOME_BUMP_MM < 0
  #error "[XYZ]_HOME_BUMP_MM must be greater than or equal to 0."
#endif

/**
 * Check auto bed leveling sub-options, especially probe points
 */
#if ABL_GRID

  #if DISABLED(DELTA_PROBEABLE_RADIUS)
    // Be sure points are in the right order
    #if LEFT_PROBE_BED_POSITION > RIGHT_PROBE_BED_POSITION
      #error "LEFT_PROBE_BED_POSITION must be less than RIGHT_PROBE_BED_POSITION."
    #elif FRONT_PROBE_BED_POSITION > BACK_PROBE_BED_POSITION
      #error "FRONT_PROBE_BED_POSITION must be less than BACK_PROBE_BED_POSITION."
    #endif
  #endif
#endif // ABL_GRID

/**
 * ENABLE_LEVELING_FADE_HEIGHT requirements
 */
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT) && !HAS_LEVELING
  #error "ENABLE_LEVELING_FADE_HEIGHT requires Bed Level"
#endif

/**
 * LCD_BED_LEVELING requirements
 */
#if ENABLED(LCD_BED_LEVELING)
  #if !HAS_LCD
    #error "LCD_BED_LEVELING requires an LCD controller."
  #elif DISABLED(MESH_BED_LEVELING) && !(HAS_ABL && ENABLED(PROBE_MANUALLY))
    #error "LCD_BED_LEVELING requires MESH_BED_LEVELING or ABL and PROBE_MANUALLY."
  #endif
#endif

// Firmware Retract
#if ENABLED(FWRETRACT)
  #if DISABLED(MIN_AUTORETRACT)
    #error DEPENDENCY ERROR: Missing setting MIN_AUTORETRACT
  #endif
  #if DISABLED(MAX_AUTORETRACT)
    #error DEPENDENCY ERROR: Missing setting MAX_AUTORETRACT
  #endif
  #if DISABLED(RETRACT_LENGTH)
    #error DEPENDENCY ERROR: Missing setting RETRACT_LENGTH
  #endif
  #if DISABLED(RETRACT_LENGTH_SWAP)
    #error DEPENDENCY ERROR: Missing setting RETRACT_LENGTH_SWAP
  #endif
  #if DISABLED(RETRACT_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting RETRACT_FEEDRATE
  #endif
  #if DISABLED(RETRACT_ZLIFT)
    #error DEPENDENCY ERROR: Missing setting RETRACT_ZLIFT
  #endif
  #if DISABLED(RETRACT_RECOVER_LENGTH)
    #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH
  #endif
  #if DISABLED(RETRACT_RECOVER_LENGTH_SWAP)
    #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH_SWAP
  #endif
  #if DISABLED(RETRACT_RECOVER_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_FEEDRATE
  #endif
  #if DISABLED(RETRACT_RECOVER_FEEDRATE_SWAP)
    #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_FEEDRATE_SWAP
  #endif
#endif

#if ENABLED(DUAL_X_CARRIAGE)
  #if DISABLED(X2_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting X2_MIN_POS
  #endif
  #if DISABLED(X2_MAX_POS)
    #error DEPENDENCY ERROR: Missing setting X2_MAX_POS
  #endif
  #if DISABLED(X2_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting X2_HOME_DIR
  #endif
  #if DISABLED(X2_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting X2_HOME_POS
  #endif
  #if DISABLED(DEFAULT_DUAL_X_CARRIAGE_MODE)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_DUAL_X_CARRIAGE_MODE
  #endif
  #if DISABLED(TOOLCHANGE_PARK_ZLIFT)
    #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_PARK_ZLIFT
  #endif
  #if DISABLED(TOOLCHANGE_UNPARK_ZLIFT)
    #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_UNPARK_ZLIFT
  #endif
  #if DISABLED(DEFAULT_DUPLICATION_X_OFFSET)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_DUPLICATION_X_OFFSET
  #endif
#endif
#if ENABLED(Y_TWO_STEPPER)
  #if DISABLED(INVERT_Y2_VS_Y_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_Y2_VS_Y_DIR
  #endif
#endif

//sensors
#if ENABLED(FILAMENT_SENSOR)
  #if DISABLED(FILAMENT_SENSOR_EXTRUDER_NUM)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_SENSOR_EXTRUDER_NUM
  #endif
  #if DISABLED(MEASUREMENT_DELAY_CM)
    #error DEPENDENCY ERROR: Missing setting MEASUREMENT_DELAY_CM
  #endif
  #if DISABLED(DEFAULT_NOMINAL_FILAMENT_DIA)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_NOMINAL_FILAMENT_DIA 
  #endif
  #if DISABLED(MEASURED_UPPER_LIMIT)
    #error DEPENDENCY ERROR: Missing setting MEASURED_UPPER_LIMIT
  #endif
  #if DISABLED(MEASURED_LOWER_LIMIT)
    #error DEPENDENCY ERROR: Missing setting MEASURED_LOWER_LIMIT
  #endif
  #if DISABLED(MAX_MEASUREMENT_DELAY)
    #error DEPENDENCY ERROR: Missing setting MAX_MEASUREMENT_DELAY
  #endif
  #if DISABLED(DEFAULT_MEASURED_FILAMENT_DIA)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_MEASURED_FILAMENT_DIA
  #endif
#endif

/**
 * Filament Runout needs a pin and M600 command
 */
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #if DISABLED(FIL_RUNOUT_PIN_INVERTING)
    #error DEPENDENCY ERROR: Missing setting FIL_RUNOUT_PIN_INVERTING
  #elif DISABLED(FILAMENT_RUNOUT_SCRIPT)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_RUNOUT_SCRIPT 
  #elif DISABLED(ADVANCED_PAUSE_FEATURE)
    static_assert(NULL == strstr(FILAMENT_RUNOUT_SCRIPT, "M600"), "ADVANCED_PAUSE_FEATURE is required to use M600 with FILAMENT_RUNOUT_SENSOR.");
  #endif
#endif

/**
 * Advanced Pause
 */
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #if EXTRUDERS == 0
    #error "ADVANCED_PAUSE_FEATURE currently requires extruders."
  #endif
  #if !HAS_LCD
    #error "ADVANCED_PAUSE_FEATURE currently requires an LCD controller."
  #elif ENABLED(EXTRUDER_RUNOUT_PREVENT)
    #error "EXTRUDER_RUNOUT_PREVENT is incompatible with ADVANCED_PAUSE_FEATURE."
  #endif
  #if DISABLED(PAUSE_PARK_X_POS)
    #error DEPENDENCY ERROR: Missing setting PAUSE_PARK_X_POS
  #endif
  #if DISABLED(PAUSE_PARK_Y_POS)
    #error DEPENDENCY ERROR: Missing setting PAUSE_PARK_Y_POS
  #endif
  #if DISABLED(PAUSE_PARK_Z_ADD)
    #error DEPENDENCY ERROR: Missing setting PAUSE_PARK_Z_ADD
  #endif
  #if DISABLED(PAUSE_PARK_RETRACT_LENGTH)
    #error DEPENDENCY ERROR: Missing setting PAUSE_PARK_RETRACT_LENGTH
  #endif
  #if DISABLED(PAUSE_PARK_UNLOAD_LENGTH)
    #error DEPENDENCY ERROR: Missing setting PAUSE_PARK_UNLOAD_LENGTH
  #endif
  #if DISABLED(PAUSE_PARK_PRINTER_OFF)
    #error DEPENDENCY ERROR: Missing setting PAUSE_PARK_PRINTER_OFF
  #endif
#endif

#if ENABLED(POWER_CONSUMPTION)
  #if DISABLED(POWER_VOLTAGE)
    #error DEPENDENCY ERROR: Missing setting POWER_VOLTAGE
  #endif
  #if DISABLED(POWER_SENSITIVITY)
    #error DEPENDENCY ERROR: Missing setting POWER_SENSITIVITY
  #endif
  #if DISABLED(POWER_OFFSET)
    #error DEPENDENCY ERROR: Missing setting POWER_OFFSET 
  #endif
  #if DISABLED(POWER_ZERO)
    #error DEPENDENCY ERROR: Missing setting POWER_ZERO 
  #endif
  #if DISABLED(POWER_ERROR)
    #error DEPENDENCY ERROR: Missing setting POWER_ERROR 
  #endif
  #if DISABLED(POWER_EFFICIENCY)
    #error DEPENDENCY ERROR: Missing setting POWER_EFFICIENCY 
  #endif
#endif

//addon
#if HAS_SDSUPPORT
  #if DISABLED(SD_FINISHED_STEPPERRELEASE)
    #error DEPENDENCY ERROR: Missing setting SD_FINISHED_STEPPERRELEASE
  #endif
  #if DISABLED(SD_FINISHED_RELEASECOMMAND)
    #error DEPENDENCY ERROR: Missing setting SD_FINISHED_RELEASECOMMAND
  #endif
  #if ENABLED(SD_SETTINGS) && DISABLED(SD_CFG_SECONDS)
    #error DEPENDENCY ERROR: Missing setting SD_CFG_SECONDS
  #endif
#endif
#if ENABLED(SHOW_BOOTSCREEN)
  #if DISABLED(STRING_SPLASH_LINE1)
    #error DEPENDENCY ERROR: Missing setting STRING_SPLASH_LINE1
  #endif
  #if DISABLED(BOOTSCREEN_TIMEOUT)
    #error DEPENDENCY ERROR: Missing setting BOOTSCREEN_TIMEOUT
  #endif
#endif
#if ENABLED(ULTIPANEL)
  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    #if DISABLED(ENCODER_10X_STEPS_PER_SEC)
      #error DEPENDENCY ERROR: Missing setting ENCODER_10X_STEPS_PER_SEC
    #endif
    #if DISABLED(ENCODER_100X_STEPS_PER_SEC)
      #error DEPENDENCY ERROR: Missing setting ENCODER_100X_STEPS_PER_SEC
    #endif
  #endif
#endif
#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
  #if DISABLED(UI_VOLTAGE_LEVEL)
    #error DEPENDENCY ERROR: Missing setting UI_VOLTAGE_LEVEL
  #endif
#endif
#if ENABLED(REPRAPWORLD_KEYPAD)
  #if DISABLED(REPRAPWORLD_KEYPAD_MOVE_STEP)
    #error DEPENDENCY ERROR: Missing setting REPRAPWORLD_KEYPAD_MOVE_STEP
  #endif
#endif
#if ENABLED(ULTIPANEL)
  #if ENABLED(LCD_PROGRESS_BAR)
    #if DISABLED(PROGRESS_BAR_BAR_TIME)
      #error DEPENDENCY ERROR: Missing setting PROGRESS_BAR_BAR_TIME
    #endif
    #if DISABLED(PROGRESS_BAR_MSG_TIME)
      #error DEPENDENCY ERROR: Missing setting PROGRESS_BAR_MSG_TIME
    #endif
    #if DISABLED(PROGRESS_MSG_EXPIRE)
      #error DEPENDENCY ERROR: Missing setting PROGRESS_MSG_EXPIRE
    #endif
  #endif
#endif
#if ENABLED(CHDK)
  #if DISABLED(CHDK_DELAY)
    #error DEPENDENCY ERROR: Missing setting CHDK_DELAY
  #endif
#endif
//adv motion
#if ENABLED(USE_MICROSTEPS)
  #if DISABLED(MICROSTEP_MODES)
    #error DEPENDENCY ERROR: Missing setting MICROSTEP_MODES
  #endif
#endif
#if DISABLED(DEFAULT_STEPPER_DEACTIVE_TIME)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_STEPPER_DEACTIVE_TIME
#endif
#if ENABLED(STEPPER_HIGH_LOW)
  #if DISABLED(STEPPER_HIGH_LOW_DELAY)
    #error DEPENDENCY ERROR: Missing setting STEPPER_HIGH_LOW_DELAY
  #endif
#endif
#if ENABLED(DIGIPOT_I2C)
  #if DISABLED(DIGIPOT_I2C_NUM_CHANNELS)
    #error DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_NUM_CHANNELS
  #endif
  #if DISABLED(DIGIPOT_I2C_MOTOR_CURRENTS)
    #error DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_MOTOR_CURRENTS
  #endif
#endif
#if ENABLED(HAVE_TMCDRIVER)
  #if ENABLED(X_IS_TMC)
    #if DISABLED(X_CURRENT)
      #error DEPENDENCY ERROR: Missing setting X_CURRENT
    #endif
    #if DISABLED(X_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting X_SENSE_RESISTOR
    #endif
    #if DISABLED(X_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X_MICROSTEPS
    #endif
  #endif
  #if ENABLED(X2_IS_TMC)
    #if DISABLED(X2_CURRENT)
      #error DEPENDENCY ERROR: Missing setting X2_CURRENT
    #endif
    #if DISABLED(X2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting X2_SENSE_RESISTOR
    #endif
    #if DISABLED(X2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X2_MICROSTEPS
    #endif
  #endif
  #if ENABLED(Y_IS_TMC)
    #if DISABLED(Y_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Y_CURRENT
    #endif
    #if DISABLED(Y_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Y_SENSE_RESISTOR
    #endif
    #if DISABLED(Y_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y_MICROSTEPS
    #endif
  #endif
  #if ENABLED(Y2_IS_TMC)
    #if DISABLED(Y2_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Y2_CURRENT
    #endif
    #if DISABLED(Y2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Y2_SENSE_RESISTOR
    #endif
    #if DISABLED(Y2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y2_MICROSTEPS
    #endif
  #endif
  #if ENABLED(Z_IS_TMC)
    #if DISABLED(Z_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Z_CURRENT
    #endif
    #if DISABLED(Z_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Z_SENSE_RESISTOR
    #endif
    #if DISABLED(Z_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Z_MICROSTEPS
    #endif
  #endif
  #if ENABLED(Z2_IS_TMC)
    #if DISABLED(Z2_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Z2_MAX_CURRENT
    #endif
    #if DISABLED(Z2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Z2_SENSE_RESISTOR
    #endif
    #if DISABLED(Z2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Z2_MICROSTEPS
    #endif
  #endif
  #if ENABLED(E0_IS_TMC)
    #if DISABLED(E0_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E0_CURRENT
    #endif
    #if DISABLED(E0_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E0_SENSE_RESISTOR
    #endif
    #if DISABLED(E0_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E0_MICROSTEPS
    #endif
  #endif
  #if ENABLED(E1_IS_TMC)
    #if DISABLED(E1_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E1_CURRENT
    #endif
    #if DISABLED(E1_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E1_SENSE_RESISTOR
    #endif
    #if DISABLED(E1_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E1_MICROSTEPS
    #endif
  #endif
  #if ENABLED(E2_IS_TMC)
    #if DISABLED(E2_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E2_CURRENT
    #endif
    #if DISABLED(E2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E2_SENSE_RESISTOR
    #endif
    #if DISABLED(E2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E2_MICROSTEPS
    #endif
  #endif
  #if ENABLED(E3_IS_TMC)
    #if DISABLED(E3_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E3_CURRENT
    #endif
    #if DISABLED(E3_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E3_SENSE_RESISTOR
    #endif
    #if DISABLED(E3_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E3_MICROSTEPS
    #endif
  #endif
#endif
#if ENABLED(HAVE_L6470DRIVER)
  #if ENABLED(X_IS_L6470)
    #if DISABLED(X_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X_MICROSTEPS
    #endif
    #if DISABLED(X_K_VAL)
      #error DEPENDENCY ERROR: Missing setting X_K_VAL
    #endif
    #if DISABLED(X_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting X_OVERCURRENT
    #endif
    #if DISABLED(X_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting X_STALLCURRENT
    #endif
  #endif
  #if ENABLED(X2_IS_L6470)
    #if DISABLED(X2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X2_MICROSTEPS
    #endif
    #if DISABLED(X2_K_VAL)
      #error DEPENDENCY ERROR: Missing setting X2_K_VAL
    #endif
    #if DISABLED(X2_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting X2_OVERCURRENT
    #endif
    #if DISABLED(X2_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting X2_STALLCURRENT
    #endif
  #endif
  #if ENABLED(Y_IS_L6470)
    #if DISABLED(Y_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y_MICROSTEPS
    #endif
    #if DISABLED(Y_K_VAL)
      #error DEPENDENCY ERROR: Missing setting Y_K_VAL
    #endif
    #if DISABLED(Y_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting Y_OVERCURRENT
    #endif
    #if DISABLED(Y_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting Y_STALLCURRENT
    #endif
  #endif
  #if ENABLED(Y2_IS_L6470)
    #if DISABLED(Y2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y2_MICROSTEPS
    #endif
    #if DISABLED(Y2_K_VAL)
      #error DEPENDENCY ERROR: Missing setting Y2_K_VAL
    #endif
    #if DISABLED(Y2_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting Y2_OVERCURRENT
    #endif
    #if DISABLED(Y2_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting Y2_STALLCURRENT
    #endif
  #endif
  #if ENABLED(Z_IS_L6470)
    #if DISABLED(Z_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Z_MICROSTEPS
    #endif
    #if DISABLED(Z_K_VAL)
      #error DEPENDENCY ERROR: Missing setting Z_K_VAL
    #endif
    #if DISABLED(Z_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting Z_OVERCURRENT
    #endif
    #if DISABLED(Z_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting Z_STALLCURRENT
    #endif
  #endif
  #if ENABLED(Z2_IS_L6470)
    #if DISABLED(Z2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Z2_MICROSTEPS
    #endif
    #if DISABLED(Z2_K_VAL)
      #error DEPENDENCY ERROR: Missing setting Z2_K_VAL
    #endif
    #if DISABLED(Z2_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting Z2_OVERCURRENT
    #endif
    #if DISABLED(Z2_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting Z2_STALLCURRENT
    #endif
  #endif
  #if ENABLED(E0_IS_L6470)
    #if DISABLED(E0_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E0_MICROSTEPS
    #endif
    #if DISABLED(E0_K_VAL)
      #error DEPENDENCY ERROR: Missing setting E0_K_VAL
    #endif
    #if DISABLED(E0_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting E0_OVERCURRENT
    #endif
    #if DISABLED(E0_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting E0_STALLCURRENT
    #endif
  #endif
  #if ENABLED(E1_IS_L6470)
    #if DISABLED(E1_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E1_MICROSTEPS
    #endif
    #if DISABLED(E1_K_VAL)
      #error DEPENDENCY ERROR: Missing setting E1_K_VAL
    #endif
    #if DISABLED(E1_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting E1_OVERCURRENT
    #endif
    #if DISABLED(E1_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting E1_STALLCURRENT
    #endif
  #endif
  #if ENABLED(E2_IS_L6470)
    #if DISABLED(E2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E2_MICROSTEPS
    #endif
    #if DISABLED(E2_K_VAL)
      #error DEPENDENCY ERROR: Missing setting E2_K_VAL
    #endif
    #if DISABLED(E2_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting E2_OVERCURRENT
    #endif
    #if DISABLED(E2_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting E2_STALLCURRENT
    #endif
  #endif
  #if ENABLED(E3_IS_L6470)
    #if DISABLED(E3_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E3_MICROSTEPS
    #endif
    #if DISABLED(E3_K_VAL)
      #error DEPENDENCY ERROR: Missing setting E3_K_VAL
    #endif
    #if DISABLED(E3_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting E3_OVERCURRENT
    #endif
    #if DISABLED(E3_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting E3_STALLCURRENT
    #endif
  #endif
#endif
//buffer
#if DISABLED(BLOCK_BUFFER_SIZE)
  #error DEPENDENCY ERROR: Missing setting BLOCK_BUFFER_SIZE
#endif
#if DISABLED(MAX_CMD_SIZE)
  #error DEPENDENCY ERROR: Missing setting MAX_CMD_SIZE
#endif
#if DISABLED(BUFSIZE)
  #error DEPENDENCY ERROR: Missing setting BUFSIZE
#endif
#if DISABLED(NUM_POSITON_SLOTS)
  #error DEPENDENCY ERROR: Missing setting NUM_POSITON_SLOTS
#endif
#if DISABLED(MIN_STEPS_PER_SEGMENT)
  #error DEPENDENCY ERROR: Missing setting MIN_STEPS_PER_SEGMENT
#endif
#if DISABLED(DEFAULT_MINSEGMENTTIME)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_MINSEGMENTTIME
#endif
#if DISABLED(MM_PER_ARC_SEGMENT)
  #error DEPENDENCY ERROR: Missing setting MM_PER_ARC_SEGMENT
#endif
#if DISABLED(N_ARC_CORRECTION)
  #error DEPENDENCY ERROR: Missing setting N_ARC_CORRECTION
#endif

//Machines
#if DISABLED(X_MIN_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting X_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Y_MIN_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting Y_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z_MIN_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting Z_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z2_MIN_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting Z2_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z3_MIN_ENDSTOP_LOGIC) && !IS_KINEMATIC
  #error DEPENDENCY ERROR: Missing setting Z3_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z4_MIN_ENDSTOP_LOGIC) && !IS_KINEMATIC
  #error DEPENDENCY ERROR: Missing setting Z4_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(X_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting X_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Y_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Y_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z2_MAX_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting Z2_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z3_MAX_ENDSTOP_LOGIC) && !IS_KINEMATIC
  #error DEPENDENCY ERROR: Missing setting Z3_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z4_MAX_ENDSTOP_LOGIC) && !IS_KINEMATIC
  #error DEPENDENCY ERROR: Missing setting Z4_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z_PROBE_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z_PROBE_ENDSTOP_LOGIC
#endif
#if DISABLED(E_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting E_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(X_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting X_HOME_DIR
#endif
#if DISABLED(Y_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting Y_HOME_DIR
#endif
#if DISABLED(Z_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting Z_HOME_DIR
#endif
#if DISABLED(E_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting E_HOME_DIR
#endif
#if DISABLED(X_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting X_ENABLE_ON
#endif
#if DISABLED(Y_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting Y_ENABLE_ON
#endif
#if DISABLED(Z_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting Z_ENABLE_ON
#endif
#if DISABLED(E_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting E_ENABLE_ON
#endif
#if DISABLED(INVERT_X_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_X_STEP_PIN
#endif
#if DISABLED(INVERT_Y_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_Y_STEP_PIN
#endif
#if DISABLED(INVERT_Z_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_Z_STEP_PIN
#endif
#if DISABLED(INVERT_E_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_E_STEP_PIN
#endif
#if DISABLED(INVERT_X_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_X_DIR
#endif
#if DISABLED(INVERT_Y_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_Y_DIR
#endif
#if DISABLED(INVERT_Z_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_Z_DIR
#endif
#if DISABLED(INVERT_E0_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E0_DIR
#endif
#if DISABLED(INVERT_E1_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E1_DIR
#endif
#if DISABLED(INVERT_E2_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E2_DIR
#endif
#if DISABLED(INVERT_E3_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E3_DIR
#endif
#if DISABLED(DISABLE_X)
  #error DEPENDENCY ERROR: Missing setting DISABLE_X
#endif
#if DISABLED(DISABLE_Y)
  #error DEPENDENCY ERROR: Missing setting DISABLE_Y
#endif
#if DISABLED(DISABLE_Z)
  #error DEPENDENCY ERROR: Missing setting DISABLE_Z
#endif
#if DISABLED(DISABLE_E)
  #error DEPENDENCY ERROR: Missing setting DISABLE_E
#endif
#if DISABLED(X_MAX_POS)
  #error DEPENDENCY ERROR: Missing setting X_MAX_POS
#endif
#if DISABLED(X_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting X_MIN_POS
#endif
#if DISABLED(Y_MAX_POS)
  #error DEPENDENCY ERROR: Missing setting Y_MAX_POS
#endif
#if DISABLED(Y_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting Y_MIN_POS
#endif
#if DISABLED(Z_MAX_POS)
  #error DEPENDENCY ERROR: Missing setting Z_MAX_POS
#endif
#if DISABLED(Z_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting Z_MIN_POS
#endif
#if DISABLED(E_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting E_MIN_POS
#endif
#if DISABLED(AXIS_RELATIVE_MODES)
  #error DEPENDENCY ERROR: Missing setting AXIS_RELATIVE_MODES
#endif
#if DISABLED(DEFAULT_AXIS_STEPS_PER_UNIT)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_AXIS_STEPS_PER_UNIT
#endif
#if ENABLED(ULTIPANEL) && DISABLED(MANUAL_FEEDRATE)
  #error DEPENDENCY ERROR: Missing setting MANUAL_FEEDRATE
#endif
#if DISABLED(DEFAULT_MINTRAVELFEEDRATE)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_MINTRAVELFEEDRATE
#endif
#if DISABLED(MINIMUM_PLANNER_SPEED)
  #error DEPENDENCY ERROR: Missing setting MINIMUM_PLANNER_SPEED
#endif
#if DISABLED(DEFAULT_MAX_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_MAX_ACCELERATION
#endif
#if DISABLED(DEFAULT_RETRACT_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_RETRACT_ACCELERATION
#endif
#if DISABLED(DEFAULT_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_ACCELERATION
#endif
#if DISABLED(DEFAULT_TRAVEL_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_TRAVEL_ACCELERATION
#endif
#if DISABLED(DEFAULT_XJERK)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_XJERK
#endif
#if DISABLED(DEFAULT_YJERK)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_YJERK
#endif
#if DISABLED(DEFAULT_ZJERK)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_ZJERK
#endif
#if DISABLED(X_HOME_BUMP_MM)
  #error DEPENDENCY ERROR: Missing setting X_HOME_BUMP_MM
#endif
#if DISABLED(Y_HOME_BUMP_MM)
  #error DEPENDENCY ERROR: Missing setting Y_HOME_BUMP_MM
#endif
#if DISABLED(Z_HOME_BUMP_MM)
  #error DEPENDENCY ERROR: Missing setting Z_HOME_BUMP_MM
#endif
#if DISABLED(HOMING_BUMP_DIVISOR)
  #error DEPENDENCY ERROR: Missing setting HOMING_BUMP_DIVISOR
#endif

#if ENABLED(MANUAL_HOME_POSITIONS)
  #if DISABLED(MANUAL_X_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting MANUAL_X_HOME_POS
  #endif
  #if DISABLED(MANUAL_Y_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting MANUAL_Y_HOME_POS
  #endif
  #if DISABLED(MANUAL_Z_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting MANUAL_Z_HOME_POS
  #endif
#endif

#if IS_CORE
  #if DISABLED(CORE_FACTOR)
    #error DEPENDENCY ERROR: Missing setting CORE_FACTOR
  #endif
#endif

#if IS_SCARA
  #if DISABLED(SCARA_LINKAGE_1)
    #error DEPENDENCY ERROR: Missing setting SCARA_LINKAGE_1
  #endif
  #if DISABLED(SCARA_LINKAGE_2)
    #error DEPENDENCY ERROR: Missing setting SCARA_LINKAGE_2
  #endif
  #if DISABLED(SCARA_OFFSET_X)
    #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_X
  #endif
  #if DISABLED(SCARA_OFFSET_Y)
    #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_Y
  #endif
  #if DISABLED(THETA_HOMING_OFFSET)
    #error DEPENDENCY ERROR: Missing setting THETA_HOMING_OFFSET
  #endif
  #if DISABLED(PSI_HOMING_OFFSET)
    #error DEPENDENCY ERROR: Missing setting PSI_HOMING_OFFSET
  #endif
#endif

#if MECH(DELTA)
  #if DISABLED(DELTA_DIAGONAL_ROD)
    #error DEPENDENCY ERROR: Missing setting DELTA_DIAGONAL_ROD
  #endif
  #if DISABLED(DELTA_SMOOTH_ROD_OFFSET)
    #error DEPENDENCY ERROR: Missing setting DELTA_SMOOTH_ROD_OFFSET
  #endif
  #if DISABLED(DELTA_CARRIAGE_OFFSET)
    #error DEPENDENCY ERROR: Missing setting DELTA_CARRIAGE_OFFSET
  #endif
  #if DISABLED(DELTA_PRINTABLE_RADIUS)
    #error DEPENDENCY ERROR: Missing setting DELTA_PRINTABLE_RADIUS
  #endif
  #if DISABLED(TOWER_A_ENDSTOP_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_ENDSTOP_ADJ
  #endif
  #if DISABLED(TOWER_B_ENDSTOP_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_ENDSTOP_ADJ
  #endif
  #if DISABLED(TOWER_C_ENDSTOP_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_ENDSTOP_ADJ
  #endif
  #if DISABLED(TOWER_A_RADIUS_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_RADIUS_ADJ
  #endif
  #if DISABLED(TOWER_B_RADIUS_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_RADIUS_ADJ
  #endif
  #if DISABLED(TOWER_C_RADIUS_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_RADIUS_ADJ
  #endif
  #if DISABLED(TOWER_A_ANGLE_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_ANGLE_ADJ
  #endif
  #if DISABLED(TOWER_B_ANGLE_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_ANGLE_ADJ
  #endif
  #if DISABLED(TOWER_C_ANGLE_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_ANGLE_ADJ
  #endif
  #if DISABLED(TOWER_A_DIAGROD_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_DIAGROD_ADJ
  #endif
  #if DISABLED(TOWER_B_DIAGROD_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_DIAGROD_ADJ
  #endif
  #if DISABLED(TOWER_C_DIAGROD_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_DIAGROD_ADJ
  #endif
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #if DISABLED(XY_PROBE_SPEED)
      #error DEPENDENCY ERROR: Missing setting XY_PROBE_SPEED
    #endif
    #if DISABLED(Z_PROBE_SPEED)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_SPEED
    #endif
    #if DISABLED(X_PROBE_OFFSET_FROM_NOZZLE)
      #error DEPENDENCY ERROR: Missing setting X_PROBE_OFFSET_FROM_NOZZLE
    #endif
    #if DISABLED(Y_PROBE_OFFSET_FROM_NOZZLE)
      #error DEPENDENCY ERROR: Missing setting Y_PROBE_OFFSET_FROM_NOZZLE
    #endif
    #if DISABLED(Z_PROBE_OFFSET_FROM_NOZZLE)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_OFFSET_FROM_NOZZLE
    #endif
    #if DISABLED(Z_PROBE_DEPLOY_START_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_START_LOCATION
    #endif
    #if DISABLED(Z_PROBE_DEPLOY_END_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_END_LOCATION
    #endif
    #if DISABLED(Z_PROBE_RETRACT_START_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_START_LOCATION
    #endif
    #if DISABLED(Z_PROBE_RETRACT_END_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_END_LOCATION
    #endif
    #if DISABLED(Z_PROBE_BETWEEN_HEIGHT)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_BETWEEN_HEIGHT
    #endif
  #endif
#endif

/**
 * Board
 */
#if DISABLED(KNOWN_BOARD)
  #error DEPENDENCY ERROR: You have to set a valid MOTHERBOARD.
#endif

/**
 * Mechanics
 */
#if DISABLED(KNOWN_MECH)
  #error DEPENDENCY ERROR: You have to set a valid MECHANICS.
#endif

/**
 * Two or plus Z Stepper
 */
#if ENABLED(Z_TWO_STEPPER)
  #if ENABLED(Z_THREE_STEPPER) || ENABLED(Z_FOUR_STEPPER)
    #error "CONFLICT ERROR: You cannot have two Z stepper and three or four drivers".
  #endif
#elif ENABLED(Z_THREE_STEPPER) && ENABLED(Z_FOUR_STEPPER)
  #error "CONFLICT ERROR: You cannot have three Z stepper and four drivers".
#endif

/**
 * Extruder Runout Prevention
 */
#if DISABLED(PREVENT_COLD_EXTRUSION) && ENABLED(EXTRUDER_RUNOUT_PREVENT)
  #error DEPENDENCY ERROR: EXTRUDER_RUNOUT_PREVENT needs PREVENT_COLD_EXTRUSION
#endif
#if ENABLED(EXTRUDER_RUNOUT_PREVENT) && EXTRUDER_RUNOUT_MINTEMP < EXTRUDE_MINTEMP
  #error CONFLICT ERROR: EXTRUDER_RUNOUT_MINTEMP have to be greater than EXTRUDE_MINTEMP
#endif

/**
 * Idle oozing prevent with Extruder Runout Prevention
 */
#if ENABLED(EXTRUDER_RUNOUT_PREVENT) && ENABLED(IDLE_OOZING_PREVENT)
  #error CONFLICT ERROR: EXTRUDER_RUNOUT_PREVENT and IDLE_OOZING_PREVENT are incopatible. Please comment one of them.
#endif

/**
 * Idle oozing prevent
 */
#if DISABLED(PREVENT_COLD_EXTRUSION) && ENABLED(IDLE_OOZING_PREVENT)
  #error DEPENDENCY ERROR: IDLE_OOZING_MINTEMP needs PREVENT_COLD_EXTRUSION
#endif
#if ENABLED(IDLE_OOZING_PREVENT) && IDLE_OOZING_MINTEMP < EXTRUDE_MINTEMP
  #error CONFLICT ERROR: IDLE_OOZING_MINTEMP have to be greater than EXTRUDE_MINTEMP
#endif

/**
 * Limited number of servos
 */
#if NUM_SERVOS > 4
  #error CONFLICT ERROR: The maximum number of SERVOS in MKduo is 4.
#endif
#if ENABLED(ENABLE_SERVOS)
  #if NUM_SERVOS < 1
    #error CONFLICT ERROR: NUM_SERVOS has to be at least one if you enable ENABLE_SERVOS
  #endif
  #if Z_ENDSTOP_SERVO_NR >= 0
    #if Z_ENDSTOP_SERVO_NR >= NUM_SERVOS
      #error CONFLICT ERROR: Z_ENDSTOP_SERVO_NR must be smaller than NUM_SERVOS.
    #endif
  #endif
#endif

/**
 * Servo deactivation depends on servo endstops
 */
#if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE) && !HAS_Z_SERVO_PROBE
  #error DEPENDENCY ERROR: At least one of the Z_ENDSTOP_SERVO_NR is required for DEACTIVATE_SERVOS_AFTER_MOVE.
#endif

/**
 * Required LCD language
 */
#if DISABLED(DOGLCD) && ENABLED(ULTRA_LCD) && DISABLED(DISPLAY_CHARSET_HD44780)
  #error "You must set DISPLAY_CHARSET_HD44780 to JAPANESE, WESTERN or CYRILLIC for your LCD controller."
#endif

/**
 * ULTIPANEL encoder
 */
#if ENABLED(ULTIPANEL) && DISABLED(NEWPANEL) && DISABLED(SR_LCD_2W_NL) && DISABLED(SHIFT_CLK)
  #error DEPENDENCY ERROR: ULTIPANEL requires some kind of encoder.
#endif
#if ENCODER_PULSES_PER_STEP < 0
  #error "ENCODER_PULSES_PER_STEP should not be negative, use REVERSE_MENU_DIRECTION instead."
#endif

/**
 * Dual X Carriage requirements
 */
#if ENABLED(DUAL_X_CARRIAGE)
  #if EXTRUDERS == 1
    #error "DUAL_X_CARRIAGE requires 2 (or more) extruders."
  #elif MECH(COREXY) || MECH(COREXZ)
    #error "DUAL_X_CARRIAGE cannot be used with COREXY or COREXZ."
  #elif !HAS_X2_ENABLE || !HAS_X2_STEP || !HAS_X2_DIR
    #error "DUAL_X_CARRIAGE requires X2 stepper pins to be defined."
  #elif !HAS_X_MAX
    #error "DUAL_X_CARRIAGE requires use a X Max Endstop."
  #elif DISABLED(X2_HOME_POS) || DISABLED(X2_MIN_POS) || DISABLED(X2_MAX_POS)
    #error "DUAL_X_CARRIAGE requires X2_HOME_POS, X2_MIN_POS, and X2_MAX_POS."
  #elif X_HOME_DIR != -1 || X2_HOME_DIR != 1
    #error "DUAL_X_CARRIAGE requires X_HOME_DIR -1 and X2_HOME_DIR 1."
  #endif
#endif // DUAL_X_CARRIAGE

/**
 * Make sure auto fan pins don't conflict with the fan pin
 */
#if HAS_AUTO_FAN && HAS_FAN
  #if H0_AUTO_FAN_PIN == FAN0_PIN
    #error CONFLICT ERROR: You cannot set H0_AUTO_FAN_PIN equal to FAN0_PIN.
  #elif H1_AUTO_FAN_PIN == FAN0_PIN
    #error CONFLICT ERROR: You cannot set H1_AUTO_FAN_PIN equal to FAN0_PIN.
  #elif H2_AUTO_FAN_PIN == FAN0_PIN
    #error CONFLICT ERROR: You cannot set H2_AUTO_FAN_PIN equal to FAN0_PIN.
  #elif H3_AUTO_FAN_PIN == FAN0_PIN
    #error CONFLICT ERROR: You cannot set H3_AUTO_FAN_PIN equal to FAN0_PIN.
  #endif
#endif

#if HAS_FAN && CONTROLLERFAN_PIN == FAN0_PIN
  #error CONFLICT ERROR: You cannot set CONTROLLERFAN_PIN equal to FAN0_PIN.
#endif

/**
 * Test required HEATER defines
 */
#if HOTENDS > 3
  #if !HAS_HEATER_3
    #error DEPENDENCY ERROR: HEATER_3_PIN not EXIST for this board
  #endif
#elif HOTENDS > 2
  #if !HAS_HEATER_2
    #error DEPENDENCY ERROR: HEATER_2_PIN not EXIST for this board
  #endif
#elif HOTENDS > 1
  #if !HAS_HEATER_1
    #error DEPENDENCY ERROR: HEATER_1_PIN not EXIST for this board
  #endif
#elif HOTENDS > 0
  #if !HAS_HEATER_0
    #error DEPENDENCY ERROR: HEATER_0_PIN not EXIST for this board
  #endif
#endif

/**
 * SDSUPPORT test
 */
#if ENABLED(SD_SETTINGS) && DISABLED(SDSUPPORT)
  #error DEPENDENCY ERROR: You have to enable SDSUPPORT to use SD_SETTINGS
#endif

/**
 * EEPROM test
 */
#if DISABLED(SDSUPPORT) && ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_SD)
  #error DEPENDENCY ERROR: You have to enable SDSUPPORT to use EEPROM_SD
#endif

#if MECH(COREXZ) && ENABLED(Z_LATE_ENABLE)
  #error CONFLICT ERROR: "Z_LATE_ENABLE can't be used with COREXZ."
#endif

#if ENABLED(CHDK) || ENABLED(PHOTOGRAPH)
  #error CONFLICT ERROR: CHDK and PHOTOGRAPH are incompatible.
#endif

#if !PIN_EXISTS(X_STEP)
  #error DEPENDENCY ERROR: X_STEP_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(X_DIR)
  #error DEPENDENCY ERROR: X_DIR_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(X_ENABLE)
  #error DEPENDENCY ERROR: X_ENABLE_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Y_STEP)
  #error DEPENDENCY ERROR: Y_STEP_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Y_DIR)
  #error DEPENDENCY ERROR: Y_DIR_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Y_ENABLE)
  #error DEPENDENCY ERROR: Y_ENABLE_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Z_STEP)
  #error DEPENDENCY ERROR: Z_STEP_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Z_DIR)
  #error DEPENDENCY ERROR: Z_DIR_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Z_ENABLE)
  #error DEPENDENCY ERROR: Z_ENABLE_PIN is not defined for your board. You have to define it yourself.
#endif

#if DRIVER_EXTRUDERS > 0
  #if !PIN_EXISTS(E0_STEP)
    #error DEPENDENCY ERROR: E0_STEP_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(E0_DIR)
    #error DEPENDENCY ERROR: E0_DIR_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(E0_ENABLE)
    #error DEPENDENCY ERROR: E0_ENABLE_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if DRIVER_EXTRUDERS > 1
    #if !PIN_EXISTS(E1_STEP)
      #error DEPENDENCY ERROR: E1_STEP_PIN is not defined for your board. You have to define it yourself.
    #endif
    #if !PIN_EXISTS(E1_DIR)
      #error DEPENDENCY ERROR: E1_DIR_PIN is not defined for your board. You have to define it yourself.
    #endif
    #if !PIN_EXISTS(E1_ENABLE)
      #error DEPENDENCY ERROR: E1_ENABLE_PIN is not defined for your board. You have to define it yourself.
    #endif
    #if DRIVER_EXTRUDERS > 2
      #if !PIN_EXISTS(E2_STEP)
        #error DEPENDENCY ERROR: E2_STEP_PIN is not defined for your board. You have to define it yourself.
      #endif
      #if !PIN_EXISTS(E2_DIR)
        #error DEPENDENCY ERROR: E2_DIR_PIN is not defined for your board. You have to define it yourself.
      #endif
      #if !PIN_EXISTS(E2_ENABLE)
        #error DEPENDENCY ERROR: E2_ENABLE_PIN is not defined for your board. You have to define it yourself.
      #endif
      #if DRIVER_EXTRUDERS > 3
        #if !PIN_EXISTS(E3_STEP)
          #error DEPENDENCY ERROR: E3_STEP_PIN is not defined for your board. You have to define it yourself.
        #endif
        #if !PIN_EXISTS(E3_DIR)
          #error DEPENDENCY ERROR: E3_DIR_PIN is not defined for your board. You have to define it yourself.
        #endif
        #if !PIN_EXISTS(E3_ENABLE)
          #error DEPENDENCY ERROR: E3_ENABLE_PIN is not defined for your board. You have to define it yourself.
        #endif
        #if DRIVER_EXTRUDERS > 4
          #if !PIN_EXISTS(E4_STEP)
            #error DEPENDENCY ERROR: E4_STEP_PIN is not defined for your board. You have to define it yourself.
          #endif
          #if !PIN_EXISTS(E4_DIR)
            #error DEPENDENCY ERROR: E4_DIR_PIN is not defined for your board. You have to define it yourself.
          #endif
          #if !PIN_EXISTS(E4_ENABLE)
            #error DEPENDENCY ERROR: E4_ENABLE_PIN is not defined for your board. You have to define it yourself.
          #endif
          #if DRIVER_EXTRUDERS > 5
            #if !PIN_EXISTS(E5_STEP)
              #error DEPENDENCY ERROR: E5_STEP_PIN is not defined for your board. You have to define it yourself.
            #endif
            #if !PIN_EXISTS(E5_DIR)
              #error DEPENDENCY ERROR: E5_DIR_PIN is not defined for your board. You have to define it yourself.
            #endif
            #if !PIN_EXISTS(E5_ENABLE)
              #error DEPENDENCY ERROR: E5_ENABLE_PIN is not defined for your board. You have to define it yourself.
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
#endif

/**
 * Allow only multy tools option to be defined
 */
static_assert(1 >= 0
  #if ENABLED(NPR2)
    + 1
  #endif
  #if ENABLED(MKR4)
    + 1
  #endif
  #if ENABLED(MKR6)
    + 1
  #endif
  #if ENABLED(MKR12)
    + 1
  #endif
  #if ENABLED(MKSE6)
    + 1
  #endif
  , "Please enable only one Multy tools function: NPR2, MKR4, MKR6, MKR12 or MKSE6."
);

#if ENABLED(MKR4)
  #if   (EXTRUDERS == 2) && (DRIVER_EXTRUDERS == 1) && !PIN_EXISTS(E0E1_CHOICE)
    #error DEPENDENCY ERROR: You must set E0E1_CHOICE_PIN to a valid pin if you enable MKR4 with 2 extruder and 1 driver
  #elif (EXTRUDERS > 2) && (DRIVER_EXTRUDERS == 1)
    #error DEPENDENCY ERROR: For 3 or more extruder you must set 2 DRIVER_EXTRUDERS for MKR4 system
  #elif (EXTRUDERS > 2) && PIN_EXISTS(E0E1_CHOICE)
    #error DEPENDENCY ERROR: For 3 or more extruder you must not E0E1_CHOICE_PIN for MKR4 system
  #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS == 2) && !PIN_EXISTS(E0E2_CHOICE)
    #error DEPENDENCY ERROR: You must set E0E2_CHOICE_PIN to a valid pin if you enable MKR4 with 3 extruder and 1 driver
  #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(E0E2_CHOICE) || !PIN_EXISTS(E1E3_CHOICE))
    #error DEPENDENCY ERROR: You must set E0E2_CHOICE_PIN and E1E3_CHOICE_PIN to a valid pin if you enable MKR4 with 4 extruder and 2 driver
  #elif (EXTRUDERS > 4)
    #error DEPENDENCY ERROR: MKR4 support only max 4 extruder
  #elif DISABLED(SINGLENOZZLE)
    #error DEPENDENCY ERROR: You must enabled SINGLENOZZLE for MKR4 MULTI EXTRUDER
  #endif
#elif ENABLED(MKR6)
  #if   (EXTRUDERS == 2) && (DRIVER_EXTRUDERS == 1) && !PIN_EXISTS(EX1_CHOICE)
    #error DEPENDENCY ERROR: You must to set EX1_CHOICE_PIN to a valid pin if you enable MKR6 with 2 extruder and 1 driver
  #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS == 1) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR6 with 3 extruder and 1 driver
  #elif (EXTRUDERS >= 4) && (DRIVER_EXTRUDERS == 1)
    #error DEPENDENCY ERROR: For 4 or more extruder you must set 2 DRIVER_EXTRUDERS for MKR6 system
  #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR6 with 4 extruder and 2 driver
  #elif (EXTRUDERS == 5) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR6 with 5 extruder and 2 driver
  #elif (EXTRUDERS == 6) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR6 with 6 extruder and 2 driver
  #elif (EXTRUDERS > 6)
    #error DEPENDENCY ERROR: MKR6 support only max 6 extruder
  #elif DISABLED(SINGLENOZZLE)
    #error DEPENDENCY ERROR: You must enabled SINGLENOZZLE for MKR6 MULTI EXTRUDER
  #endif
#elif ENABLED(MKR12)
  #if   (EXTRUDERS >= 4) && (DRIVER_EXTRUDERS == 1)
    #error DEPENDENCY ERROR: For 4 or more extruder you must set more DRIVER_EXTRUDERS for MKR12 system
  #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 4 extruder and 2 driver
  #elif (EXTRUDERS == 5) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 5 extruder and 2 driver
  #elif (EXTRUDERS == 6) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 6 extruder and 2 driver
  #elif (EXTRUDERS >= 7) && (DRIVER_EXTRUDERS == 2)
    #error DEPENDENCY ERROR: For 7 or more extruder you must set more DRIVER_EXTRUDERS for MKR12 system
  #elif (EXTRUDERS == 7) && (DRIVER_EXTRUDERS == 3) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 7 extruder and 3 driver
  #elif (EXTRUDERS == 8) && (DRIVER_EXTRUDERS == 3) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 8 extruder and 3 driver
  #elif (EXTRUDERS == 9) && (DRIVER_EXTRUDERS == 3) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 9 extruder and 3 driver
  #elif (EXTRUDERS >= 10) && (DRIVER_EXTRUDERS == 3)
    #error DEPENDENCY ERROR: For 10 or more extruder you must set more DRIVER_EXTRUDERS for MKR12 system
  #elif (EXTRUDERS == 10) && (DRIVER_EXTRUDERS == 4) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 10 extruder and 4 driver
  #elif (EXTRUDERS == 11) && (DRIVER_EXTRUDERS == 4) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 11 extruder and 4 driver
  #elif (EXTRUDERS == 12) && (DRIVER_EXTRUDERS == 4) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 12 extruder and 4 driver
  #elif (EXTRUDERS > 12)
    #error DEPENDENCY ERROR: MKR12 support only max 12 extruder
  #elif DISABLED(SINGLENOZZLE)
    #error DEPENDENCY ERROR: You must enabled SINGLENOZZLE for MKR12 MULTI EXTRUDER
  #endif
#elif ENABLED(MKSE6)
  #if EXTRUDERS < 2
    #error DEPENDENCY ERROR: You must set EXTRUDERS > 1 for MKSE6 MULTI EXTRUDER
  #endif
  #if DRIVER_EXTRUDERS > 1
    #error DEPENDENCY ERROR: You must set DRIVER_EXTRUDERS = 1 for MKSE6 MULTI EXTRUDER
  #endif
  #if !HAS_SERVOS
    #error DEPENDENCY ERROR: You must enabled ENABLE_SERVOS and set NUM_SERVOS > 0 for MKSE6 MULTI EXTRUDER
  #endif
  #if DISABLED(SINGLENOZZLE)
    #error DEPENDENCY ERROR: You must enabled SINGLENOZZLE for MKSE6 MULTI EXTRUDER
  #endif
#endif

#if ENABLED(NPR2) && !PIN_EXISTS(E_MIN)
  #error DEPENDENCY ERROR: You have to set E_MIN_PIN to a valid pin if you enable NPR2
#endif

#if (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR)) && !HAS_SERVOS
  #error DEPENDENCY ERROR: You must enabled ENABLE_SERVOS and set NUM_SERVOS > 0 for DONDOLO MULTI EXTRUDER
#endif

#if ENABLED(DONDOLO_SINGLE_MOTOR) && ENABLED(DONDOLO_DUAL_MOTOR)
  #error DEPENDENCY ERROR: You must enabled only one for DONDOLO_SINGLE_MOTOR and DONDOLO_DUAL_MOTOR
#endif

#if (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR)) && EXTRUDERS != 2
  #error DEPENDENCY ERROR: You must set EXTRUDERS = 2 for DONDOLO
#endif

#if ENABLED(LASER)
  #if ENABLED(LASER_PERIPHERALS)
    #if !PIN_EXISTS(LASER_PERIPHERALS)
      #error DEPENDENCY ERROR: You have to set LASER_PERIPHERALS_PIN to a valid pin if you enable LASER_PERIPHERALS
    #endif
    #if !PIN_EXISTS(LASER_PERIPHERALS_STATUS)
      #error DEPENDENCY ERROR: You have to set LASER_PERIPHERALS_STATUS_PIN to a valid pin if you enable LASER_PERIPHERALS
    #endif
  #endif
  #if (DISABLED(LASER_CONTROL) || ((LASER_CONTROL != 1) && (LASER_CONTROL != 2)))
     #error DEPENDENCY ERROR: You have to set LASER_CONTROL to 1 or 2
  #else
    #if(LASER_CONTROL == 1)
      #if(!HAS_LASER_POWER)
        #error DEPENDENCY ERROR: You have to set LASER_PWR_PIN
      #endif
    #else
      #if(!HAS_LASER_POWER || !HAS_LASER_PWM)
        #error DEPENDENCY ERROR: You have to set LASER_PWR_PIN and LASER_PWM_PIN to a valid pin if you enable LASER
      #endif
    #endif
  #endif
#endif // ENABLED(LASER)

#if ENABLED(FILAMENT_RUNOUT_SENSOR) && !PIN_EXISTS(FIL_RUNOUT)
  #error DEPENDENCY ERROR: You have to set FIL_RUNOUT_PIN to a valid pin if you enable FILAMENT_RUNOUT_SENSOR
#endif

#if ENABLED(FILAMENT_SENSOR) && !PIN_EXISTS(FILWIDTH)
  #error DEPENDENCY ERROR: You have to set FILWIDTH_PIN to a valid pin if you enable FILAMENT_SENSOR
#endif

#if ENABLED(FLOWMETER_SENSOR) && !PIN_EXISTS(FLOWMETER)
  #error DEPENDENCY ERROR: You have to set FLOWMETER_PIN to a valid pin if you enable FLOWMETER_SENSOR
#endif

#if ENABLED(POWER_CONSUMPTION) && !PIN_EXISTS(POWER_CONSUMPTION)
  #error DEPENDENCY ERROR: You have to set POWER_CONSUMPTION_PIN to a valid pin if you enable POWER_CONSUMPTION
#endif

#if ENABLED(DOOR_OPEN) && !PIN_EXISTS(DOOR)
  #error DEPENDENCY ERROR: You have to set DOOR_PIN to a valid pin if you enable DOOR_OPEN
#endif

#if ENABLED(PHOTOGRAPH) && !PIN_EXISTS(PHOTOGRAPH)
  #error DEPENDENCY ERROR: You have to set PHOTOGRAPH_PIN to a valid pin if you enable PHOTOGRAPH
#endif

#if ENABLED(CHDK) && !PIN_EXISTS(CHDK)
  #error DEPENDENCY ERROR: You have to set CHDK_PIN to a valid pin if you enable CHDK
#endif

#if ENABLED(CONTROLLERFAN) && !PIN_EXISTS(CONTROLLERFAN)
  #error DEPENDENCY ERROR: You have to set CONTROLLERFAN_PIN to a valid pin if you enable CONTROLLERFAN
#endif

#if ENABLED(HOTEND_AUTO_FAN) && !PIN_EXISTS(H0_AUTO_FAN) && !PIN_EXISTS(H1_AUTO_FAN) && !PIN_EXISTS(H2_AUTO_FAN) && !PIN_EXISTS(H3_AUTO_FAN)
  #error DEPENDENCY ERROR: You have to set at least one HOTEND_?_AUTO_FAN_PIN to a valid pin if you enable HOTEND_AUTO_FAN
#endif

#if ENABLED(X2_IS_TMC) && (!PIN_EXISTS(X2_ENABLE) || !PIN_EXISTS(X2_STEP) || !PIN_EXISTS(X2_DIR))
  #error DEPENDENCY ERROR: You have to set X2_ENABLE_PIN, X2_STEP_PIN and X2_DIR_PIN to a valid pin if you enable X2_IS_TMC
#endif
  
#if ((ENABLED(ENABLE_SERVOS) && NUM_SERVOS > 0) && !(HAS_SERVO_0 || HAS_SERVO_1 || HAS_SERVO_2 || HAS_SERVO_3))
  #error DEPENDENCY ERROR: You have to set at least one SERVO?_PIN to a valid pin if you enable ENABLE_SERVOS
#endif

#if ENABLED(Z_PROBE_SLED) && !PIN_EXISTS(SLED)
  #error DEPENDENCY ERROR: You have to set SLED_PIN to a valid pin if you enable Z_PROBE_SLED
#endif

/**
 * G38 Probe Target
 */
#if ENABLED(G38_PROBE_TARGET)
  #if !HAS_BED_PROBE
    #error "G38_PROBE_TARGET requires a bed probe."
  #elif !IS_CARTESIAN
    #error "G38_PROBE_TARGET requires a Cartesian machine."
  #endif
#endif

/**
 * RGB_LED Requirements
 */
#define _RGB_TEST (PIN_EXISTS(RGB_LED_R) && PIN_EXISTS(RGB_LED_G) && PIN_EXISTS(RGB_LED_B))
#if ENABLED(RGB_LED)
  #if !_RGB_TEST
    #error "RGB_LED requires RGB_LED_R_PIN, RGB_LED_G_PIN, and RGB_LED_B_PIN."
  #elif ENABLED(RGBW_LED)
    #error "Please enable only one of RGB_LED and RGBW_LED."
  #endif
#elif ENABLED(RGBW_LED)
  #if !(_RGB_TEST && PIN_EXISTS(RGB_LED_W))
    #error "RGBW_LED requires RGB_LED_R_PIN, RGB_LED_G_PIN, RGB_LED_B_PIN, and RGB_LED_W_PIN."
  #endif
#elif ENABLED(NEOPIXEL_RGBW_LED)
  #if !(PIN_EXISTS(NEOPIXEL) && NEOPIXEL_PIXELS > 0)
    #error "NEOPIXEL_RGBW_LED requires NEOPIXEL_PIN and NEOPIXEL_PIXELS."
  #endif
#elif ENABLED(PRINTER_EVENT_LEDS) && DISABLED(BLINKM) && DISABLED(PCA9632) && !HAS_NEOPIXEL
  #error "PRINTER_EVENT_LEDS requires BLINKM, PCA9632, RGB_LED, RGBW_LED or NEOPIXEL_LED."
#endif

/**
 * Make sure only one display is enabled
 *
 * Note: BQ_LCD_SMART_CONTROLLER => REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
 *       REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER => REPRAP_DISCOUNT_SMART_CONTROLLER
 *       SAV_3DGLCD => U8GLIB_SH1106 => ULTIMAKERCONTROLLER
 *       miniVIKI => ULTIMAKERCONTROLLER
 *       VIKI2 => ULTIMAKERCONTROLLER
 *       ELB_FULL_GRAPHIC_CONTROLLER => ULTIMAKERCONTROLLER
 *       PANEL_ONE => ULTIMAKERCONTROLLER
 */
static_assert(1 >= 0
  #if ENABLED(ULTIMAKERCONTROLLER) \
      && DISABLED(SAV_3DGLCD) && DISABLED(miniVIKI) && DISABLED(VIKI2) \
      && DISABLED(ELB_FULL_GRAPHIC_CONTROLLER) && DISABLED(PANEL_ONE)
    + 1
  #endif
  #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) && DISABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    + 1
  #endif
  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER) && DISABLED(BQ_LCD_SMART_CONTROLLER)
    + 1
  #endif
  #if ENABLED(CARTESIO_UI)
    + 1
  #endif
  #if ENABLED(PANEL_ONE)
    + 1
  #endif
  #if ENABLED(MAKRPANEL)
    + 1
  #endif
  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
    + 1
  #endif
  #if ENABLED(VIKI2)
    + 1
  #endif
  #if ENABLED(miniVIKI)
    + 1
  #endif
  #if ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
    + 1
  #endif
  #if ENABLED(G3D_PANEL)
    + 1
  #endif
  #if ENABLED(MINIPANEL)
    + 1
  #endif
  #if ENABLED(REPRAPWORLD_KEYPAD) && DISABLED(CARTESIO_UI)
    + 1
  #endif
  #if ENABLED(RIGIDBOT_PANEL)
    + 1
  #endif
  #if ENABLED(RA_CONTROL_PANEL)
    + 1
  #endif
  #if ENABLED(LCD_I2C_SAINSMART_YWROBOT)
    + 1
  #endif
  #if ENABLED(LCM1602)
    + 1
  #endif
  #if ENABLED(LCD_I2C_PANELOLU2)
    + 1
  #endif
  #if ENABLED(LCD_I2C_VIKI)
    + 1
  #endif
  #if ENABLED(U8GLIB_SSD1306)
    + 1
  #endif
  #if ENABLED(SAV_3DLCD)
    + 1
  #endif
  #if ENABLED(BQ_LCD_SMART_CONTROLLER)
    + 1
  #endif
  #if ENABLED(SAV_3DGLCD)
    + 1
  #endif
  #if ENABLED(NEXTION)
    + 1
  #endif
  , "Please select no more than one LCD controller option."
);

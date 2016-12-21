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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

/**
 * Require gcc 4.7 or newer (first included with Arduino 1.6.8) for C++11 features.
 */
#if __cplusplus < 201103L
  #error "Marlin requires C++11 support (gcc >= 4.7, Arduino IDE >= 1.6.8). Please upgrade your toolchain."
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
  #if HASNT(HEATER_CHAMBER)
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
  #if HASNT(COOLER)
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
#if ENABLED(PIDTEMP)
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
#if ENABLED(PIDTEMPBED)
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
#if ENABLED(PIDTEMPCHAMBER)
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
#if ENABLED(PIDTEMPCOOLER)
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
#if ENABLED(THERMAL_PROTECTION_HOTENDS)
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
#if ENABLED(THERMAL_PROTECTION_BED)
  #if DISABLED(THERMAL_PROTECTION_BED_PERIOD)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_BED_PERIOD
  #endif
  #if DISABLED(THERMAL_PROTECTION_BED_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_BED_HYSTERESIS
  #endif
#endif
#if ENABLED(THERMAL_PROTECTION_COOLER)
  #if DISANLED(THERMAL_PROTECTION_COOLER_PERIOD)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_COOLER_PERIOD
  #endif
  #if DISABLED(THERMAL_PROTECTION_COOLER_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_COOLER_HYSTERESIS
  #endif
  #if ENABLED(THERMAL_PROTECTION_COOLER_WATCHDOG)
     #if DISABLED(WATCH_TEMP_COOLER_PERIOD)
       #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_COOLER_PERIOD
     #endif
     #if DISABLED(WATCH_TEMP_COOLER_DECREASE)
       #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_COOLER_DECREASE
     #endif
  #endif
#endif

// Fan
#if DISABLED(SOFT_PWM_SCALE)
  #error DEPENDENCY ERROR: Missing setting SOFT_PWM_SCALE
#endif

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
  #if DISABLED(EXTRUDER_RUNOUT_ESTEPS)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_ESTEPS
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

#if ENABLED(FILAMENT_CHANGE_FEATURE)
  #if DISABLED(FILAMENT_CHANGE_X_POS)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_X_POS
  #endif
  #if DISABLED(FILAMENT_CHANGE_Y_POS)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_Y_POS
  #endif
  #if DISABLED(FILAMENT_CHANGE_Z_ADD)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_Z_ADD
  #endif
  #if DISABLED(FILAMENT_CHANGE_RETRACT_LENGTH)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_RETRACT_LENGTH
  #endif
  #if DISABLED(FILAMENT_CHANGE_UNLOAD_LENGTH)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_UNLOAD_LENGTH
  #endif
  #if DISABLED(FILAMENT_CHANGE_PRINTER_OFF)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_PRINTER_OFF
  #endif
#endif

/**
 * Motion
 */
#if DISABLED(SOFTWARE_MIN_ENDSTOPS)
  #error DEPENDENCY ERROR: Missing setting SOFTWARE_MIN_ENDSTOPS
#endif
#if DISABLED(SOFTWARE_MAX_ENDSTOPS)
  #error DEPENDENCY ERROR: Missing setting SOFTWARE_MAX_ENDSTOPS
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
    #if (ABL_GRID_POINTS_X & 1) == 0 || (ABL_GRID_POINTS_Y & 1) == 0
      #error "DELTA requires ABL_GRID_POINTS_X and ABL_GRID_POINTS_Y to be odd numbers."
    #elif ABL_GRID_POINTS_X < 3
      #error "DELTA requires ABL_GRID_POINTS_X and ABL_GRID_POINTS_Y to be 3 or higher."
    #endif
  #endif

  #if ENABLED(AUTO_CALIBRATION_FEATURE) && ENABLED(AUTO_CALIBRATION_7_POINT)
    #error "Only one system Autocalibration must is defined."
  #endif
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
#if HAS(ABL)
  #define COUNT_LEV_1 0
  #if ENABLED(AUTO_BED_LEVELING_LINEAR)
    #define COUNT_LEV_2 INCREMENT(COUNT_LEV_1)
  #else
    #define COUNT_LEV_2 COUNT_LEV_1
  #endif
  #if ENABLED(AUTO_BED_LEVELING_3POINT)
    #define COUNT_LEV_3 INCREMENT(COUNT_LEV_2)
  #else
    #define COUNT_LEV_3 COUNT_LEV_2
  #endif
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    #define COUNT_LEV_4 INCREMENT(COUNT_LEV_3)
  #else
    #define COUNT_LEV_4 COUNT_LEV_3
  #endif
  #if ENABLED(MESH_BED_LEVELING)
    #define COUNT_LEV_5 INCREMENT(COUNT_LEV_4)
  #else
    #define COUNT_LEV_5 COUNT_LEV_4
  #endif
  #if COUNT_LEV_5 > 1
    #error "Select only one of: MESH_BED_LEVELING, AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT, or AUTO_BED_LEVELING_BILINEAR."
  #endif
#endif

/**
 * Mesh Bed Leveling
 */
#if ENABLED(MESH_BED_LEVELING)
  #if MECH(DELTA)
    #error "MESH_BED_LEVELING does not yet support DELTA printers."
  #elif MESH_NUM_X_POINTS > 9 || MESH_NUM_Y_POINTS > 9
    #error "MESH_NUM_X_POINTS and MESH_NUM_Y_POINTS must be less than 10."
  #endif
#elif ENABLED(MANUAL_BED_LEVELING)
  #error "MANUAL_BED_LEVELING only applies to MESH_BED_LEVELING."
#endif

/**
 * Probes
 */
#if PROBE_SELECTED

  /**
   * Allow only one probe option to be defined
   */
  #define COUNT_PROBE_1 0
  #if ENABLED(Z_PROBE_FIX_MOUNTED)
    #define COUNT_PROBE_2 INCREMENT(COUNT_PROBE_1)
  #else
    #define COUNT_PROBE_2 COUNT_PROBE_1
  #endif
  #if HAS_Z_SERVO_ENDSTOP && DISABLED(BLTOUCH)
    #define COUNT_PROBE_3 INCREMENT(COUNT_PROBE_2)
  #else
    #define COUNT_PROBE_3 COUNT_PROBE_2
  #endif
  #if ENABLED(BLTOUCH)
    #define COUNT_PROBE_4 INCREMENT(COUNT_PROBE_3)
  #else
    #define COUNT_PROBE_4 COUNT_PROBE_3
  #endif
  #if ENABLED(Z_PROBE_ALLEN_KEY)
    #define COUNT_PROBE_5 INCREMENT(COUNT_PROBE_4)
  #else
    #define COUNT_PROBE_5 COUNT_PROBE_4
  #endif
  #if ENABLED(Z_PROBE_SLED)
    #define COUNT_PROBE_6 INCREMENT(COUNT_PROBE_5)
  #else
    #define COUNT_PROBE_6 COUNT_PROBE_5
  #endif
  #if COUNT_PROBE_6 > 1
    #error "Please enable only one probe: Z_PROBE_FIX_MOUNTED, Z Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, or Z_PROBE_SLED."
  #endif

  /**
   * Z_PROBE_SLED is incompatible with DELTA
   */
  #if ENABLED(Z_PROBE_SLED) && MECH(DELTA)
    #error "You cannot use Z_PROBE_SLED with DELTA."
  #endif

  /**
   * NUM_SERVOS is required for a Z servo probe
   */
  #if HAS(Z_SERVO_ENDSTOP)
    #ifndef NUM_SERVOS
      #error "You must set NUM_SERVOS for a Z servo probe (Z_ENDSTOP_SERVO_NR)."
    #elif Z_ENDSTOP_SERVO_NR >= NUM_SERVOS
      #error "Z_ENDSTOP_SERVO_NR must be less than NUM_SERVOS."
    #endif
  #endif

  /**
   * A probe needs a pin
   */
  #if !PROBE_PIN_CONFIGURED
    #error "A probe needs a pin! Use Z_MIN_PIN or Z_PROBE_PIN."
  #endif

  /**
   * Make sure Z raise values are set
   */
  #if !defined(Z_PROBE_DEPLOY_HEIGHT)
    #error "You must define Z_PROBE_DEPLOY_HEIGHT in your configuration."
  #elif !defined(Z_PROBE_BETWEEN_HEIGHT)
    #error "You must define Z_PROBE_BETWEEN_HEIGHT in your configuration."
  #elif Z_PROBE_DEPLOY_HEIGHT < 0
    #error "Probes need Z_PROBE_DEPLOY_HEIGHT >= 0."
  #elif Z_PROBE_BETWEEN_HEIGHT < 0
    #error "Probes need Z_PROBE_BETWEEN_HEIGHT >= 0."
  #endif

#else

  /**
   * Require some kind of probe for bed leveling and probe testing
   */
  #if HAS(ABL) || ENABLED(AUTO_CALIBRATION_FEATURE) || ENABLED(AUTO_CALIBRATION_7_POINT)
    #error "Auto Bed Leveling or Auto Calibration requires a probe! Define a Z Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
  #elif ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
    #error "Z_MIN_PROBE_REPEATABILITY_TEST requires a probe! Define a Z Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
  #endif

#endif

/**
 * Homing Bump
 */
#if X_HOME_BUMP_MM < 0 || Y_HOME_BUMP_MM < 0 || Z_HOME_BUMP_MM < 0
  #error "[XYZ]_HOME_BUMP_MM must be greater than or equal to 0."
#endif

/**
 * Make sure Z_SAFE_HOMING point is reachable
 */
#if ENABLED(Z_SAFE_HOMING)
  #if Z_SAFE_HOMING_X_POINT < MIN_PROBE_X || Z_SAFE_HOMING_X_POINT > MAX_PROBE_X
    #if HAS_BED_PROBE
      #error "Z_SAFE_HOMING_X_POINT can't be reached by the Z probe."
    #else
      #error "Z_SAFE_HOMING_X_POINT can't be reached by the nozzle."
    #endif
  #elif Z_SAFE_HOMING_Y_POINT < MIN_PROBE_Y || Z_SAFE_HOMING_Y_POINT > MAX_PROBE_Y
    #if HAS_BED_PROBE
      #error "Z_SAFE_HOMING_Y_POINT can't be reached by the Z probe."
    #else
      #error "Z_SAFE_HOMING_Y_POINT can't be reached by the nozzle."
    #endif
  #endif
#endif // Z_SAFE_HOMING

/**
 * Auto Bed Leveling
 */
#if HAS(ABL)

  /**
   * Delta and SCARA have limited bed leveling options
   */
  #if DISABLED(AUTO_BED_LEVELING_BILINEAR)
    #if MECH(DELTA)
      #error "Only AUTO_BED_LEVELING_BILINEAR is supported for DELTA bed leveling."
    #elif IS_SCARA
      #error "Only AUTO_BED_LEVELING_BILINEAR is supported for SCARA bed leveling."
    #endif
  #endif

  /**
   * Check if Probe_Offset * Grid Points is greater than Probing Range
   */
  #if ABL_GRID

    #if DISABLED(DELTA_PROBEABLE_RADIUS)
      // Be sure points are in the right order
      #if LEFT_PROBE_BED_POSITION > RIGHT_PROBE_BED_POSITION
        #error "LEFT_PROBE_BED_POSITION must be less than RIGHT_PROBE_BED_POSITION."
      #elif FRONT_PROBE_BED_POSITION > BACK_PROBE_BED_POSITION
        #error "FRONT_PROBE_BED_POSITION must be less than BACK_PROBE_BED_POSITION."
      #endif
      // Make sure probing points are reachable
      #if LEFT_PROBE_BED_POSITION < MIN_PROBE_X
        #error "The given LEFT_PROBE_BED_POSITION can't be reached by the Z probe."
      #elif RIGHT_PROBE_BED_POSITION > MAX_PROBE_X
        #error "The given RIGHT_PROBE_BED_POSITION can't be reached by the Z probe."
      #elif FRONT_PROBE_BED_POSITION < MIN_PROBE_Y
        #error "The given FRONT_PROBE_BED_POSITION can't be reached by the Z probe."
      #elif BACK_PROBE_BED_POSITION > MAX_PROBE_Y
        #error "The given BACK_PROBE_BED_POSITION can't be reached by the Z probe."
      #endif
    #endif

  #else // !ABL_GRID

    // Check the triangulation points
    #if ABL_PROBE_PT_1_X < MIN_PROBE_X || ABL_PROBE_PT_1_X > MAX_PROBE_X
      #error "The given ABL_PROBE_PT_1_X can't be reached by the Z probe."
    #elif ABL_PROBE_PT_2_X < MIN_PROBE_X || ABL_PROBE_PT_2_X > MAX_PROBE_X
      #error "The given ABL_PROBE_PT_2_X can't be reached by the Z probe."
    #elif ABL_PROBE_PT_3_X < MIN_PROBE_X || ABL_PROBE_PT_3_X > MAX_PROBE_X
      #error "The given ABL_PROBE_PT_3_X can't be reached by the Z probe."
    #elif ABL_PROBE_PT_1_Y < MIN_PROBE_Y || ABL_PROBE_PT_1_Y > MAX_PROBE_Y
      #error "The given ABL_PROBE_PT_1_Y can't be reached by the Z probe."
    #elif ABL_PROBE_PT_2_Y < MIN_PROBE_Y || ABL_PROBE_PT_2_Y > MAX_PROBE_Y
      #error "The given ABL_PROBE_PT_2_Y can't be reached by the Z probe."
    #elif ABL_PROBE_PT_3_Y < MIN_PROBE_Y || ABL_PROBE_PT_3_Y > MAX_PROBE_Y
      #error "The given ABL_PROBE_PT_3_Y can't be reached by the Z probe."
    #endif

  #endif // !ABL_GRID

#endif // HAS_ABL

#if ENABLED(FWRETRACT)
  #if DISABLED(MIN_RETRACT)
    #error DEPENDENCY ERROR: Missing setting MIN_RETRACT
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
#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
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
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #if DISABLED(FIL_RUNOUT_PIN_INVERTING)
    #error DEPENDENCY ERROR: Missing setting FIL_RUNOUT_PIN_INVERTING
  #endif
  #if DISABLED(FILAMENT_RUNOUT_SCRIPT)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_RUNOUT_SCRIPT 
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
#if ENABLED(SDSUPPORT)
  #if DISABLED(SD_FINISHED_STEPPERRELEASE)
    #error DEPENDENCY ERROR: Missing setting SD_FINISHED_STEPPERRELEASE
  #endif
  #if DISABLED(SD_FINISHED_RELEASECOMMAND)
    #error DEPENDENCY ERROR: Missing setting SD_FINISHED_RELEASECOMMAND
  #endif
  #if ENABLED(SD_SETTINGS)
    #if DISABLED(SD_CFG_SECONDS)
      #error DEPENDENCY ERROR: Missing setting SD_CFG_SECONDS
    #endif
    #if DISABLED(CFG_SD_FILE)
      #error DEPENDENCY ERROR: Missing setting CFG_SD_FILE
    #endif
  #endif
#endif
#if DISABLED(DOGLCD) && ENABLED(ULTRA_LCD) && DISABLED(DISPLAY_CHARSET_HD44780)
  #error "You must set DISPLAY_CHARSET_HD44780 to JAPANESE, WESTERN or CYRILLIC for your LCD controller."
#endif
#if ENABLED(SHOW_BOOTSCREEN)
  #if DISABLED(STRING_SPLASH_LINE1)
    #error DEPENDENCY ERROR: Missing setting STRING_SPLASH_LINE1
  #endif
  #if DISABLED(SPLASH_SCREEN_DURATION)
    #error DEPENDENCY ERROR: Missing setting SPLASH_SCREEN_DURATION
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
    #if DISABLED(X_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting X_MAX_CURRENT
    #endif
    #if DISABLED(X_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting X_SENSE_RESISTOR
    #endif
    #if DISABLED(X_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X_MICROSTEPS
    #endif
  #endif
  #if ENABLED(X2_IS_TMC)
    #if DISABLED(X2_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting X2_MAX_CURRENT
    #endif
    #if DISABLED(X2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting X2_SENSE_RESISTOR
    #endif
    #if DISABLED(X2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X2_MICROSTEPS
    #endif
  #endif
  #if ENABLED(Y_IS_TMC)
    #if DISABLED(Y_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Y_MAX_CURRENT
    #endif
    #if DISABLED(Y_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Y_SENSE_RESISTOR
    #endif
    #if DISABLED(Y_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y_MICROSTEPS
    #endif
  #endif
  #if ENABLED(Y2_IS_TMC)
    #if DISABLED(Y2_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Y2_MAX_CURRENT
    #endif
    #if DISABLED(Y2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Y2_SENSE_RESISTOR
    #endif
    #if DISABLED(Y2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y2_MICROSTEPS
    #endif
  #endif
  #if ENABLED(Z_IS_TMC)
    #if DISABLED(Z_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Z_MAX_CURRENT
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
    #if DISABLED(E0_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E0_MAX_CURRENT
    #endif
    #if DISABLED(E0_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E0_SENSE_RESISTOR
    #endif
    #if DISABLED(E0_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E0_MICROSTEPS
    #endif
  #endif
  #if ENABLED(E1_IS_TMC)
    #if DISABLED(E1_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E1_MAX_CURRENT
    #endif
    #if DISABLED(E1_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E1_SENSE_RESISTOR
    #endif
    #if DISABLED(E1_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E1_MICROSTEPS
    #endif
  #endif
  #if ENABLED(E2_IS_TMC)
    #if DISABLED(E2_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E2_MAX_CURRENT
    #endif
    #if DISABLED(E2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E2_SENSE_RESISTOR
    #endif
    #if DISABLED(E2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E2_MICROSTEPS
    #endif
  #endif
  #if ENABLED(E3_IS_TMC)
    #if DISABLED(E3_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E3_MAX_CURRENT
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
#if DISABLED(X_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting X_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Y_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Y_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z2_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z2_MIN_ENDSTOP_LOGIC
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
#if DISABLED(Z2_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z2_MAX_ENDSTOP_LOGIC
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
  #if DISABLED(LINKAGE_1)
    #error DEPENDENCY ERROR: Missing setting LINKAGE_1
  #endif
  #if DISABLED(LINKAGE_2)
    #error DEPENDENCY ERROR: Missing setting LINKAGE_2
  #endif
  #if DISABLED(SCARA_OFFSET_X)
    #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_X
  #endif
  #if DISABLED(SCARA_OFFSET_Y)
    #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_Y
  #endif
  #if DISABLED(SCARA_RAD2DEG)
    #error DEPENDENCY ERROR: Missing setting SCARA_RAD2DEG
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
  #if DISABLED(DEFAULT_DELTA_RADIUS)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_DELTA_RADIUS
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
  #if DISABLED(TOWER_A_POSITION_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_POSITION_ADJ
  #endif
  #if DISABLED(TOWER_B_POSITION_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_POSITION_ADJ
  #endif
  #if DISABLED(TOWER_C_POSITION_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_POSITION_ADJ
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
    #if DISABLED(AUTOCALIBRATION_PRECISION)
      #error DEPENDENCY ERROR: Missing setting AUTOCALIBRATION_PRECISION
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
    #if DISABLED(AUTO_BED_LEVELING_GRID_POINTS)
      #error DEPENDENCY ERROR: Missing setting AUTO_BED_LEVELING_GRID_POINTS
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
 * Dual Stepper Drivers
 */
#if ENABLED(Z_DUAL_STEPPER_DRIVERS) && ENABLED(Y_DUAL_STEPPER_DRIVERS)
  #error CONFLICT ERROR: You cannot have dual stepper drivers for both Y and Z.
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
 * Options only for EXTRUDERS == 1
 */
#if EXTRUDERS > 1

  #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
    #error CONFLICT ERROR: EXTRUDERS must be 1 with TEMP_SENSOR_1_AS_REDUNDANT.
  #endif

  #if ENABLED(HEATERS_PARALLEL)
    #error CONFLICT ERROR: EXTRUDERS must be 1 with HEATERS_PARALLEL.
  #endif

#endif // EXTRUDERS > 1

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
#if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE) && HASNT(Z_SERVO_ENDSTOP)
  #error DEPENDENCY ERROR: At least one of the Z_ENDSTOP_SERVO_NR is required for DEACTIVATE_SERVOS_AFTER_MOVE.
#endif

/**
 * Required LCD language
 */
#if DISABLED(DOGLCD) && ENABLED(ULTRA_LCD) && DISABLED(DISPLAY_CHARSET_HD44780_JAPAN) && DISABLED(DISPLAY_CHARSET_HD44780_WESTERN) && DISABLED(DISPLAY_CHARSET_HD44780_CYRILLIC)
  #error DEPENDENCY ERROR: You must enable either DISPLAY_CHARSET_HD44780_JAPAN or DISPLAY_CHARSET_HD44780_WESTERN  or DISPLAY_CHARSET_HD44780_CYRILLIC for your LCD controller.
#endif

/**
 * Required LCD for FILAMENT_CHANGE_FEATURE
 */
#if ENABLED(FILAMENT_CHANGE_FEATURE) && DISABLED(ULTRA_LCD) && DISABLED(NEXTION)
  #error DEPENDENCY ERROR: You must have LCD in order to use FILAMENT_CHANGE_FEATURE
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
  #elif HASNT(X2_ENABLE) || HASNT(X2_STEP) || HASNT(X2_DIR)
    #error "DUAL_X_CARRIAGE requires X2 stepper pins to be defined."
  #elif HASNT(X_MAX)
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
#if HAS(AUTO_FAN) && HAS(FAN)
  #if H0_AUTO_FAN_PIN == FAN_PIN
    #error CONFLICT ERROR: You cannot set H0_AUTO_FAN_PIN equal to FAN_PIN.
  #elif H1_AUTO_FAN_PIN == FAN_PIN
    #error CONFLICT ERROR: You cannot set H1_AUTO_FAN_PIN equal to FAN_PIN.
  #elif H2_AUTO_FAN_PIN == FAN_PIN
    #error CONFLICT ERROR: You cannot set H2_AUTO_FAN_PIN equal to FAN_PIN.
  #elif H3_AUTO_FAN_PIN == FAN_PIN
    #error CONFLICT ERROR: You cannot set H3_AUTO_FAN_PIN equal to FAN_PIN.
  #endif
#endif

#if HAS(FAN) && CONTROLLERFAN_PIN == FAN_PIN
  #error CONFLICT ERROR: You cannot set CONTROLLERFAN_PIN equal to FAN_PIN.
#endif

/**
 * Test required HEATER defines
 */
#if HOTENDS > 3
  #if HASNT(HEATER_3)
    #error DEPENDENCY ERROR: HEATER_3_PIN not EXIST for this board
  #endif
#elif HOTENDS > 2
  #if HASNT(HEATER_2)
    #error DEPENDENCY ERROR: HEATER_2_PIN not EXIST for this board
  #endif
#elif HOTENDS > 1 || ENABLED(HEATERS_PARALLEL)
  #if HASNT(HEATER_1)
    #error DEPENDENCY ERROR: HEATER_1_PIN not EXIST for this board
  #endif
#elif HOTENDS > 0
  #if HASNT(HEATER_0)
    #error DEPENDENCY ERROR: HEATER_0_PIN not EXIST for this board
  #endif
#endif

#if DISABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
  #error DEPENDENCY ERROR: You have to enable SDSUPPORT to use SD_SETTINGS
#endif

#if MECH(COREXZ) && ENABLED(Z_LATE_ENABLE)
  #error CONFLICT ERROR: "Z_LATE_ENABLE can't be used with COREXZ."
#endif

#if ENABLED(POWER_CONSUMPTION) && !PIN_EXISTS(POWER_CONSUMPTION)
  #error DEPENDENCY ERROR: You have to set POWER_CONSUMPTION_PIN to a valid pin if you enable POWER_CONSUMPTION
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

#if ENABLED(MKR4) && ENABLED(MKR6)
  #error DEPENDENCY ERROR: You must set only one MKR system
#endif

#if ENABLED(MKR4)
  #if (EXTRUDERS == 2) && (DRIVER_EXTRUDERS == 1) && !PIN_EXISTS(E0E1_CHOICE)
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
  #endif 
#elif ENABLED(MKR6)
  #if (EXTRUDERS == 2) && (DRIVER_EXTRUDERS == 1) && !PIN_EXISTS(EX1_CHOICE)
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
  #endif
#endif

#if ENABLED(MKSE6)
  #if EXTRUDERS < 2
    #error DEPENDENCY ERROR: You must set EXTRUDERS > 1 for MKSE6 MULTI EXTRUDER
  #endif
  #if DRIVER_EXTRUDERS > 1
    #error DEPENDENCY ERROR: You must set DRIVER_EXTRUDERS = 1 for MKSE6 MULTI EXTRUDER
  #endif
  #if HASNT(SERVOS)
    #error DEPENDENCY ERROR: You must enabled ENABLE_SERVOS and set NUM_SERVOS > 0 for MKSE6 MULTI EXTRUDER
  #endif
  #if DISABLED(SINGLENOZZLE)
    #error DEPENDENCY ERROR: You must enabled SINGLENOZZLE for MKSE6 MULTI EXTRUDER
  #endif
#endif

#if ENABLED(NPR2) && !PIN_EXISTS(E_MIN)
  #error DEPENDENCY ERROR: You have to set E_MIN_PIN to a valid pin if you enable NPR2
#endif

#if (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR)) && HASNT(SERVOS)
  #error DEPENDENCY ERROR: You must enabled ENABLE_SERVOS and set NUM_SERVOS > 0 for DONDOLO MULTI EXTRUDER
#endif

#if ENABLED(DONDOLO_SINGLE_MOTOR) && ENABLED(DONDOLO_DUAL_MOTOR)
  #error DEPENDENCY ERROR: You must enabled only one for DONDOLO_SINGLE_MOTOR and DONDOLO_DUAL_MOTOR
#endif

#if (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR)) && EXTRUDERS != 2
  #error DEPENDENCY ERROR: You must set EXTRUDERS = 2 for DONDOLO
#endif

#if ENABLED(LASERBEAM) 
  #if (!ENABLED(LASER_REMAP_INTENSITY) && ENABLED(LASER_RASTER))
    #error DEPENDENCY ERROR: You have to set LASER_REMAP_INTENSITY with LASER_RASTER enabled
  #endif
  #if (!ENABLED(LASER_CONTROL) || ((LASER_CONTROL != 1) && (LASER_CONTROL != 2)))
     #error DEPENDENCY ERROR: You have to set LASER_CONTROL to 1 or 2
  #else
    #if(LASER_CONTROL == 1)
      #if( !PIN_EXISTS(LASER_PWR))
        #error DEPENDENCY ERROR: You have to set LASER_PWR_PIN
      #endif
    #else
      #if( !PIN_EXISTS(LASER_PWR) || !PIN_EXISTS(LASER_TTL))
        #error DEPENDENCY ERROR: You have to set LASER_PWR_PIN and LASER_TTL_PIN to a valid pin if you enable LASER
      #endif
    #endif
  #endif
  #if DISABLED(LASER_HAS_FOCUS)
    #error DEPENDENCY ERROR: Missing LASER_HAS_FOCUS setting
  #endif
#endif

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

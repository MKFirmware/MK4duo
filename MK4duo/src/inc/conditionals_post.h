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
 * conditionals_post.h
 * Defines that depend on configuration but are not editable.
 */

#ifndef _CONDITIONALS_POST_H_
#define _CONDITIONALS_POST_H_

/**
 * SAM3X8E
 */
#if ENABLED(ARDUINO_ARCH_SAM)
  #undef TX_BUFFER_SIZE
  #undef RX_BUFFER_SIZE
  #if ENABLED(M100_FREE_MEMORY_WATCHER)
    #undef M100_FREE_MEMORY_WATCHER
  #endif
  #define EXTENDED_CAPABILITIES_REPORT
#endif

/**
 * DELTA
 */
#if MECH(DELTA)
  #undef SLOWDOWN       // DELTA not needs SLOWDOWN
  #undef Z_SAFE_HOMING  // DELTA non needs Z_SAFE_HOMING

  // DELTA must have same valour for 3 axis Home Feedrate
  #define HOMING_FEEDRATE_X HOMING_FEEDRATE_XYZ
  #define HOMING_FEEDRATE_Y HOMING_FEEDRATE_XYZ
  #define HOMING_FEEDRATE_Z HOMING_FEEDRATE_XYZ

  // DELTA must have same valour for 3 axis endstop hits
  #define X_HOME_BUMP_MM XYZ_HOME_BUMP_MM
  #define Y_HOME_BUMP_MM XYZ_HOME_BUMP_MM
  #define Z_HOME_BUMP_MM XYZ_HOME_BUMP_MM
  #define HOMING_BUMP_DIVISOR {XYZ_BUMP_DIVISOR, XYZ_BUMP_DIVISOR, XYZ_BUMP_DIVISOR}

  // Effective horizontal distance bridged by diagonal push rods.
  #define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET - DELTA_EFFECTOR_OFFSET - DELTA_CARRIAGE_OFFSET)

  #if DISABLED(Z_PROBE_SPEED)
    #define Z_PROBE_SPEED     (HOMING_FEEDRATE_XYZ / 2)
  #endif

  #define Z_PROBE_SPEED_FAST  Z_PROBE_SPEED
  #define Z_PROBE_SPEED_SLOW  Z_PROBE_SPEED

  // Set the rectangle in which to probe
  #define DELTA_PROBEABLE_RADIUS     (DELTA_PRINTABLE_RADIUS - max(abs(X_PROBE_OFFSET_FROM_NOZZLE), abs(Y_PROBE_OFFSET_FROM_NOZZLE)))
  #define LEFT_PROBE_BED_POSITION   -(mechanics.delta_probe_radius)
  #define RIGHT_PROBE_BED_POSITION   (mechanics.delta_probe_radius)
  #define FRONT_PROBE_BED_POSITION  -(mechanics.delta_probe_radius)
  #define BACK_PROBE_BED_POSITION    (mechanics.delta_probe_radius)

  #define X_MIN_POS -(mechanics.delta_print_radius)
  #define X_MAX_POS  (mechanics.delta_print_radius)
  #define Y_MIN_POS -(mechanics.delta_print_radius)
  #define Y_MAX_POS  (mechanics.delta_print_radius)
  #define Z_MAX_POS  (mechanics.delta_height)
  #define Z_MIN_POS 0
  #define E_MIN_POS 0

  #define X_MIN_POS_ZERO (X_MIN_POS)
  #define Y_MIN_POS_ZERO (Y_MIN_POS)

  #define X_BED_SIZE ((DELTA_PRINTABLE_RADIUS) * 2)
  #define Y_BED_SIZE ((DELTA_PRINTABLE_RADIUS) * 2)

  #define UBL_PROBEABLE_RADIUS   (DELTA_PRINTABLE_RADIUS - MESH_INSET)
  #define UBL_MESH_MIN_X        -(UBL_PROBEABLE_RADIUS)
  #define UBL_MESH_MAX_X         (UBL_PROBEABLE_RADIUS)
  #define UBL_MESH_MIN_Y        -(UBL_PROBEABLE_RADIUS)
  #define UBL_MESH_MAX_Y         (UBL_PROBEABLE_RADIUS)

  #define PROBE_PT_1_X 0
  #define PROBE_PT_1_Y 0
  #define PROBE_PT_2_X 0
  #define PROBE_PT_2_Y 0
  #define PROBE_PT_3_X 0
  #define PROBE_PT_3_Y 0

  #if ENABLED(WORKSPACE_OFFSETS)
    #undef WORKSPACE_OFFSETS
  #endif

  #define HAS_DELTA_AUTO_CALIBRATION  (ENABLED(DELTA_AUTO_CALIBRATION_1) || ENABLED(DELTA_AUTO_CALIBRATION_2))

#else // !MECH(DELTA)

  #if ((X_MIN_POS) <= 0)
    #define X_MIN_POS_ZERO 0
  #else
    #define X_MIN_POS_ZERO (X_MIN_POS)
  #endif
  #if ((Y_MIN_POS) <= 0)
    #define Y_MIN_POS_ZERO 0
  #else
    #define Y_MIN_POS_ZERO (Y_MIN_POS)
  #endif

#endif // !MECH(DELTA)

/**
 * Axis lengths and center
 */
#define X_MAX_LENGTH (X_MAX_POS - (X_MIN_POS_ZERO))
#define Y_MAX_LENGTH (Y_MAX_POS - (Y_MIN_POS_ZERO))
#define Z_MAX_LENGTH (Z_MAX_POS - (Z_MIN_POS))

// Require 0,0 bed center for Delta and SCARA
#if IS_KINEMATIC
  #define BED_CENTER_AT_0_0
#endif

// Defined only if the sanity-check is bypassed
#ifndef X_BED_SIZE
  #define X_BED_SIZE X_MAX_LENGTH
#endif
#ifndef Y_BED_SIZE
  #define Y_BED_SIZE Y_MAX_LENGTH
#endif

// Define center values for future use
#ifdef BED_CENTER_AT_0_0
  #define X_CENTER 0
  #define Y_CENTER 0
#else
  #define X_CENTER ((X_BED_SIZE) / 2)
  #define Y_CENTER ((Y_BED_SIZE) / 2)
#endif
#define Z_CENTER ((Z_MIN_POS + Z_MAX_POS) / 2)

// Get the linear boundaries of the bed
#define X_MIN_BED (X_CENTER - (X_BED_SIZE) / 2)
#define X_MAX_BED (X_CENTER + (X_BED_SIZE) / 2)
#define Y_MIN_BED (Y_CENTER - (Y_BED_SIZE) / 2)
#define Y_MAX_BED (Y_CENTER + (Y_BED_SIZE) / 2)

/**
 * CoreXY, CoreXZ, and CoreYZ - and their reverse
 */
#if IS_CORE
  #if CORE_IS_XY
    #define CORE_AXIS_1 A_AXIS
    #define CORE_AXIS_2 B_AXIS
    #define NORMAL_AXIS Z_AXIS
  #elif CORE_IS_XZ
    #define CORE_AXIS_1 A_AXIS
    #define NORMAL_AXIS Y_AXIS
    #define CORE_AXIS_2 C_AXIS
  #elif CORE_IS_YZ
    #define NORMAL_AXIS X_AXIS
    #define CORE_AXIS_1 B_AXIS
    #define CORE_AXIS_2 C_AXIS
  #endif
  #if (MECH(COREXY) || MECH(COREXZ) || MECH(COREYZ))
    #define CORESIGN(n) (n)
  #else
    #define CORESIGN(n) (-(n))
  #endif
#endif

/**
 * SCARA cannot use SLOWDOWN and requires QUICKHOME
 */
#if IS_SCARA
  #undef SLOWDOWN
  #define QUICK_HOME
#endif

 /**
 * Set the home position based on settings or manual overrides
 */
#if ENABLED(MANUAL_X_HOME_POS)
  #define X_HOME_POS MANUAL_X_HOME_POS
#elif ENABLED(BED_CENTER_AT_0_0)
  #if MECH(DELTA)
    #define X_HOME_POS 0
  #else
    #define X_HOME_POS ((X_MAX_LENGTH) * (X_HOME_DIR) * 0.5)
  #endif
#else
  #if MECH(DELTA)
    #define X_HOME_POS (X_MIN_POS + (X_MAX_LENGTH) * 0.5)
  #else
    #define X_HOME_POS (X_HOME_DIR < 0 ? X_MIN_POS : X_MAX_POS)
  #endif
#endif

#if ENABLED(MANUAL_Y_HOME_POS)
  #define Y_HOME_POS MANUAL_Y_HOME_POS
#elif ENABLED(BED_CENTER_AT_0_0)
  #if MECH(DELTA)
    #define Y_HOME_POS 0
  #else
    #define Y_HOME_POS ((Y_MAX_LENGTH) * (Y_HOME_DIR) * 0.5)
  #endif
#else
  #if MECH(DELTA)
    #define Y_HOME_POS (Y_MIN_POS + (Y_MAX_LENGTH) * 0.5)
  #else
    #define Y_HOME_POS (Y_HOME_DIR < 0 ? Y_MIN_POS : Y_MAX_POS)
  #endif
#endif

#if ENABLED(MANUAL_Z_HOME_POS)
  #define Z_HOME_POS MANUAL_Z_HOME_POS
#else
  #define Z_HOME_POS (Z_HOME_DIR < 0 ? Z_MIN_POS : Z_MAX_POS)
#endif

/**
 * Auto Bed Leveling and Z Probe Repeatability Test
 */
#define HOMING_Z_WITH_PROBE (HAS_BED_PROBE && Z_HOME_DIR < 0 && DISABLED(Z_TWO_ENDSTOPS))

/**
 * Shorthand for pin tests, used wherever needed
 */

// EXTRUDERS
#define HAS_EXTRUDERS       (EXTRUDERS > 0)

// Steppers
#define HAS_X_ENABLE        (PIN_EXISTS(X_ENABLE))
#define HAS_X_DIR           (PIN_EXISTS(X_DIR))
#define HAS_X_STEP          (PIN_EXISTS(X_STEP))
#define HAS_X_MICROSTEPS    (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(X_MS1))

#define HAS_X2_ENABLE       (PIN_EXISTS(X2_ENABLE))
#define HAS_X2_DIR          (PIN_EXISTS(X2_DIR))
#define HAS_X2_STEP         (PIN_EXISTS(X2_STEP))

#define HAS_Y_ENABLE        (PIN_EXISTS(Y_ENABLE))
#define HAS_Y_DIR           (PIN_EXISTS(Y_DIR))
#define HAS_Y_STEP          (PIN_EXISTS(Y_STEP))
#define HAS_Y_MICROSTEPS    (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(Y_MS1))

#define HAS_Y2_ENABLE       (PIN_EXISTS(Y2_ENABLE))
#define HAS_Y2_DIR          (PIN_EXISTS(Y2_DIR))
#define HAS_Y2_STEP         (PIN_EXISTS(Y2_STEP))

#define HAS_Z_ENABLE        (PIN_EXISTS(Z_ENABLE))
#define HAS_Z_DIR           (PIN_EXISTS(Z_DIR))
#define HAS_Z_STEP          (PIN_EXISTS(Z_STEP))
#define HAS_Z_MICROSTEPS    (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(Z_MS1))

#define HAS_Z2_ENABLE       (PIN_EXISTS(Z2_ENABLE))
#define HAS_Z2_DIR          (PIN_EXISTS(Z2_DIR))
#define HAS_Z2_STEP         (PIN_EXISTS(Z2_STEP))

// Extruder steppers and solenoids
#define HAS_E0_ENABLE       (PIN_EXISTS(E0_ENABLE))
#define HAS_E0_DIR          (PIN_EXISTS(E0_DIR))
#define HAS_E0_STEP         (PIN_EXISTS(E0_STEP))
#define HAS_E0_MICROSTEPS   (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(E0_MS1))
#define HAS_SOLENOID_0      (PIN_EXISTS(SOL0))
#define HAS_E0_ENC          (PIN_EXISTS(E0_ENC))

#define HAS_E1_ENABLE       (PIN_EXISTS(E1_ENABLE))
#define HAS_E1_DIR          (PIN_EXISTS(E1_DIR))
#define HAS_E1_STEP         (PIN_EXISTS(E1_STEP))
#define HAS_E1_MICROSTEPS   (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(E1_MS1))
#define HAS_SOLENOID_1      (PIN_EXISTS(SOL1))
#define HAS_E1_ENC          (PIN_EXISTS(E1_ENC))

#define HAS_E2_ENABLE       (PIN_EXISTS(E2_ENABLE))
#define HAS_E2_DIR          (PIN_EXISTS(E2_DIR))
#define HAS_E2_STEP         (PIN_EXISTS(E2_STEP))
#define HAS_E2_MICROSTEPS   (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(E2_MS1))
#define HAS_SOLENOID_2      (PIN_EXISTS(SOL2))
#define HAS_E2_ENC          (PIN_EXISTS(E2_ENC))

#define HAS_E3_ENABLE       (PIN_EXISTS(E3_ENABLE))
#define HAS_E3_DIR          (PIN_EXISTS(E3_DIR))
#define HAS_E3_STEP         (PIN_EXISTS(E3_STEP))
#define HAS_E3_MICROSTEPS   (PIN_EXISTS(E3_MS1))
#define HAS_SOLENOID_3      (PIN_EXISTS(SOL3))
#define HAS_E3_ENC          (PIN_EXISTS(E3_ENC))

#define HAS_E4_ENABLE       (PIN_EXISTS(E4_ENABLE))
#define HAS_E4_DIR          (PIN_EXISTS(E4_DIR))
#define HAS_E4_STEP         (PIN_EXISTS(E4_STEP))
#define HAS_E4_MICROSTEPS   (PIN_EXISTS(E4_MS1))
#define HAS_SOLENOID_4      (PIN_EXISTS(SOL4))
#define HAS_E4_ENC          (PIN_EXISTS(E4_ENC))

#define HAS_E5_ENABLE       (PIN_EXISTS(E5_ENABLE))
#define HAS_E5_DIR          (PIN_EXISTS(E5_DIR))
#define HAS_E5_STEP         (PIN_EXISTS(E5_STEP))
#define HAS_E5_MICROSTEPS   (PIN_EXISTS(E5_MS1))
#define HAS_SOLENOID_5      (PIN_EXISTS(SOL5))
#define HAS_E5_ENC          (PIN_EXISTS(E5_ENC))

// Trinamic Stepper Drivers
#define HAS_TRINAMIC        (ENABLED(HAVE_TMC2130)  || ENABLED(HAVE_TMC2208)  || ENABLED(IS_TRAMS))
#define  X_IS_TRINAMIC      (ENABLED( X_IS_TMC2130) || ENABLED( X_IS_TMC2208) || ENABLED(IS_TRAMS))
#define X2_IS_TRINAMIC      (ENABLED(X2_IS_TMC2130) || ENABLED(X2_IS_TMC2208))
#define  Y_IS_TRINAMIC      (ENABLED( Y_IS_TMC2130) || ENABLED( Y_IS_TMC2208) || ENABLED(IS_TRAMS))
#define Y2_IS_TRINAMIC      (ENABLED(Y2_IS_TMC2130) || ENABLED(Y2_IS_TMC2208))
#define  Z_IS_TRINAMIC      (ENABLED( Z_IS_TMC2130) || ENABLED( Z_IS_TMC2208) || ENABLED(IS_TRAMS))
#define Z2_IS_TRINAMIC      (ENABLED(Z2_IS_TMC2130) || ENABLED(Z2_IS_TMC2208))
#define E0_IS_TRINAMIC      (ENABLED(E0_IS_TMC2130) || ENABLED(E0_IS_TMC2208) || ENABLED(IS_TRAMS))
#define E1_IS_TRINAMIC      (ENABLED(E1_IS_TMC2130) || ENABLED(E1_IS_TMC2208))
#define E2_IS_TRINAMIC      (ENABLED(E2_IS_TMC2130) || ENABLED(E2_IS_TMC2208))
#define E3_IS_TRINAMIC      (ENABLED(E3_IS_TMC2130) || ENABLED(E3_IS_TMC2208))
#define E4_IS_TRINAMIC      (ENABLED(E4_IS_TMC2130) || ENABLED(E4_IS_TMC2208))
#define E5_IS_TRINAMIC      (ENABLED(E5_IS_TMC2130) || ENABLED(E5_IS_TMC2208))

// Disable Z axis sensorless homing if a probe is used to home the Z axis
#if ENABLED(SENSORLESS_HOMING)
  #define X_SENSORLESS (ENABLED(X_IS_TMC2130) && ENABLED(X_HOMING_SENSITIVITY))
  #define Y_SENSORLESS (ENABLED(Y_IS_TMC2130) && ENABLED(Y_HOMING_SENSITIVITY))
  #define Z_SENSORLESS (ENABLED(Z_IS_TMC2130) && ENABLED(Z_HOMING_SENSITIVITY))
  #if HOMING_Z_WITH_PROBE
    #undef Z_HOMING_SENSITIVITY
  #endif
#endif

// Endstops and bed probe
#define HAS_X_MIN           (PIN_EXISTS(X_MIN))
#define HAS_X_MAX           (PIN_EXISTS(X_MAX))
#define HAS_Y_MIN           (PIN_EXISTS(Y_MIN))
#define HAS_Y_MAX           (PIN_EXISTS(Y_MAX))
#define HAS_Z_MIN           (PIN_EXISTS(Z_MIN))
#define HAS_Z_MAX           (PIN_EXISTS(Z_MAX))
#define HAS_X2_MIN          (PIN_EXISTS(X2_MIN))
#define HAS_Y2_MIN          (PIN_EXISTS(Y2_MIN))
#define HAS_Z2_MIN          (PIN_EXISTS(Z2_MIN))
#define HAS_X2_MAX          (PIN_EXISTS(X2_MAX))
#define HAS_Y2_MAX          (PIN_EXISTS(Y2_MAX))
#define HAS_Z2_MAX          (PIN_EXISTS(Z2_MAX))
#define HAS_Z_PROBE_PIN     (PIN_EXISTS(Z_PROBE))

// Utility
#define HAS_DOOR_OPEN       (ENABLED(DOOR_OPEN) && PIN_EXISTS(DOOR_OPEN))
#define HAS_POWER_CHECK     (ENABLED(POWER_CHECK) && PIN_EXISTS(POWER_CHECK))

// Thermistors
#define HAS_TEMP_0          (PIN_EXISTS(TEMP_0) && TEMP_SENSOR_0 != 0 && TEMP_SENSOR_0 >= -1)
#define HAS_TEMP_1          (PIN_EXISTS(TEMP_1) && TEMP_SENSOR_1 != 0 && TEMP_SENSOR_1 >= -1)
#define HAS_TEMP_2          (PIN_EXISTS(TEMP_2) && TEMP_SENSOR_2 != 0 && TEMP_SENSOR_2 >= -1)
#define HAS_TEMP_3          (PIN_EXISTS(TEMP_3) && TEMP_SENSOR_3 != 0 && TEMP_SENSOR_3 >= -1)
#define HAS_TEMP_HOTEND     (HAS_TEMP_0 || HEATER_USES_MAX)
#define HAS_TEMP_BED        (PIN_EXISTS(TEMP_BED) && TEMP_SENSOR_BED != 0 && TEMP_SENSOR_BED >= -1)
#define HAS_TEMP_CHAMBER    (PIN_EXISTS(TEMP_CHAMBER) && TEMP_SENSOR_CHAMBER != 0 && TEMP_SENSOR_CHAMBER >= -1)
#define HAS_TEMP_COOLER     (PIN_EXISTS(TEMP_COOLER) && TEMP_SENSOR_COOLER != 0 && TEMP_SENSOR_COOLER >= -1)
#define HAS_TEMP_HEATER     (HAS_TEMP_HOTEND || HAS_TEMP_BED || HAS_TEMP_CHAMBER || HAS_TEMP_COOLER)
#define HAS_MCU_TEMPERATURE (ENABLED(HAVE_MCU_TEMPERATURE))

// Thermocouples
#define HAS_MAX6675_SS      (PIN_EXISTS(MAX6675_SS))
#define HAS_MAX31855_SS0    (PIN_EXISTS(MAX31855_SS0))
#define HAS_MAX31855_SS1    (PIN_EXISTS(MAX31855_SS1))
#define HAS_MAX31855_SS2    (PIN_EXISTS(MAX31855_SS2))
#define HAS_MAX31855_SS3    (PIN_EXISTS(MAX31855_SS3))

// Heaters
#define HAS_HEATER_0        (HOTENDS > 0 && PIN_EXISTS(HEATER_0))
#define HAS_HEATER_1        (HOTENDS > 1 && PIN_EXISTS(HEATER_1))
#define HAS_HEATER_2        (HOTENDS > 2 && PIN_EXISTS(HEATER_2))
#define HAS_HEATER_3        (HOTENDS > 3 && PIN_EXISTS(HEATER_3))
#define HAS_HEATER_BED      (TEMP_SENSOR_BED != 0 && PIN_EXISTS(HEATER_BED))
#define HAS_HEATER_CHAMBER  (TEMP_SENSOR_CHAMBER != 0 && PIN_EXISTS(HEATER_CHAMBER))
#define HAS_HEATER_COOLER   (TEMP_SENSOR_COOLER != 0 && PIN_EXISTS(COOLER))

// Thermal protection
#define HAS_THERMALLY_PROTECTED_HOTEND  (HAS_TEMP_HOTEND && HAS_HEATER_0 && THERMAL_PROTECTION_HOTENDS)
#define HAS_THERMALLY_PROTECTED_BED     (HAS_TEMP_BED && HAS_HEATER_BED && THERMAL_PROTECTION_BED)
#define HAS_THERMALLY_PROTECTED_CHAMBER (HAS_TEMP_CHAMBER && HAS_HEATER_CHAMBER && THERMAL_PROTECTION_CHAMBER)
#define HAS_THERMALLY_PROTECTED_COOLER  (HAS_TEMP_COOLER && HAS_HEATER_COOLER && THERMAL_PROTECTION_COOLER)
#define HAS_THERMALLY_PROTECTED_HEATER  (HAS_THERMALLY_PROTECTED_HOTEND || HAS_THERMALLY_PROTECTED_BED || HAS_THERMALLY_PROTECTED_CHAMBER || HAS_THERMALLY_PROTECTED_COOLER)
#define WATCH_THE_HOTEND                (HAS_THERMALLY_PROTECTED_HOTEND   && WATCH_TEMP_PERIOD          > 0)
#define WATCH_THE_BED                   (HAS_THERMALLY_PROTECTED_BED      && WATCH_BED_TEMP_PERIOD      > 0)
#define WATCH_THE_CHAMBER               (HAS_THERMALLY_PROTECTED_CHAMBER  && WATCH_CHAMBER_TEMP_PERIOD  > 0)
#define WATCH_THE_COOLER                (HAS_THERMALLY_PROTECTED_COOLER   && WATCH_COOLER_TEMP_PERIOD   > 0)
#define WATCH_THE_HEATER                (WATCH_THE_HOTEND || WATCH_THE_BED || WATCH_THE_CHAMBER || WATCH_THE_COOLER)

// Other fans
#define HAS_FAN0            (PIN_EXISTS(FAN0))
#define HAS_FAN1            (PIN_EXISTS(FAN1))
#define HAS_FAN2            (PIN_EXISTS(FAN2))
#define HAS_FAN3            (PIN_EXISTS(FAN3))
#define HAS_FAN4            (PIN_EXISTS(FAN4))
#define HAS_FAN5            (PIN_EXISTS(FAN5))

// Servos
#define HAS_SERVO_0         (PIN_EXISTS(SERVO0))
#define HAS_SERVO_1         (PIN_EXISTS(SERVO1))
#define HAS_SERVO_2         (PIN_EXISTS(SERVO2))
#define HAS_SERVO_3         (PIN_EXISTS(SERVO3))
#define HAS_SERVOS          ((ENABLED(ENABLE_SERVOS) && NUM_SERVOS > 0) && (HAS_SERVO_0 || HAS_SERVO_1 || HAS_SERVO_2 || HAS_SERVO_3))

// Sensors
#define HAS_FILAMENT_SENSOR           (ENABLED(FILAMENT_SENSOR) && PIN_EXISTS(FILWIDTH))
#define HAS_FIL_RUNOUT                (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT0))
#define HAS_FIL_RUNOUT1               (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT1))
#define HAS_FIL_RUNOUT2               (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT2))
#define HAS_FIL_RUNOUT3               (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT3))
#define HAS_FIL_RUNOUT4               (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT4))
#define HAS_FIL_RUNOUT5               (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT5))
#define HAS_DAV_SYSTEM                (ENABLED(FILAMENT_RUNOUT_DAV_SYSTEM) && PIN_EXISTS(FIL_RUNOUT_DAV))
#define HAS_POWER_CONSUMPTION_SENSOR  (ENABLED(POWER_CONSUMPTION) && PIN_EXISTS(POWER_CONSUMPTION))

// User Interface
#define HAS_HOME            (PIN_EXISTS(HOME))
#define HAS_KILL            (PIN_EXISTS(KILL))
#define HAS_SUICIDE         (PIN_EXISTS(SUICIDE))
#define HAS_CHDK            (ENABLED(CHDK) && PIN_EXISTS(CHDK))
#define HAS_PHOTOGRAPH      (ENABLED(PHOTOGRAPH) && PIN_EXISTS(PHOTOGRAPH))
#define HAS_BUZZER          (PIN_EXISTS(BEEPER) || ENABLED(LCD_USE_I2C_BUZZER))
#define HAS_CASE_LIGHT      (ENABLED(CASE_LIGHT) && (PIN_EXISTS(CASE_LIGHT) || ENABLED(CASE_LIGHT_USE_NEOPIXEL)))

// Digital control
#define HAS_MICROSTEPS      (HAS_X_MICROSTEPS     \
                            || HAS_Y_MICROSTEPS   \
                            || HAS_Z_MICROSTEPS   \
                            || HAS_E0_MICROSTEPS  \
                            || HAS_E1_MICROSTEPS  \
                            || HAS_E2_MICROSTEPS  \
                            || HAS_E3_MICROSTEPS  \
                            || HAS_E4_MICROSTEPS  \
                            || HAS_E5_MICROSTEPS)
#define HAS_STEPPER_RESET   (PIN_EXISTS(STEPPER_RESET))
#define HAS_DIGIPOTSS       (PIN_EXISTS(DIGIPOTSS))
#define HAS_MOTOR_CURRENT_PWM_XY  (PIN_EXISTS(MOTOR_CURRENT_PWM_XY))
#define HAS_MOTOR_CURRENT_PWM     (PIN_EXISTS(MOTOR_CURRENT_PWM_XY) || PIN_EXISTS(MOTOR_CURRENT_PWM_Z) || PIN_EXISTS(MOTOR_CURRENT_PWM_E))

// Laser support
#define HAS_LASER_POWER     (PIN_EXISTS(LASER_PWR))
#define HAS_LASER_PWM       (PIN_EXISTS(LASER_PWM))

// CNC
#define HAS_CNCROUTER       (PIN_EXISTS(CNCROUTER))

// Multi Mode
#define HAS_MULTI_MODE      (ENABLED(LASER) || ENABLED(CNCROUTER) || ENABLED(MILLING) || ENABLED(PICK_AND_PLACE) || ENABLED(SOLDER) || ENABLED(PLOTTER))

// MK Multi tool system
#define HAS_MKMULTI_TOOLS   (ENABLED(MKSE6) || ENABLED(MKR4) || ENABLED(MKR6) || ENABLED(MKR12) || ENABLED(DONDOLO_SINGLE_MOTOR))

// MKR4 or MKR6 or MKR12
#define HAS_E0E1            (PIN_EXISTS(E0E1_CHOICE))
#define HAS_E0E2            (PIN_EXISTS(E0E2_CHOICE))
#define HAS_E1E3            (PIN_EXISTS(E1E3_CHOICE))
#define HAS_EX1             (PIN_EXISTS(EX1_CHOICE))
#define HAS_EX2             (PIN_EXISTS(EX2_CHOICE))

// Dondolo
#define HAS_DONDOLO         (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR))

// LCD
#define HAS_BTN_BACK        (PIN_EXISTS(BTN_BACK))

// EEPROM support
#if ENABLED(EEPROM_FLASH)
  #undef EEPROM_SPI
  #undef EEPROM_I2C
  #undef EEPROM_SD
#elif ENABLED(EEPROM_SD)
  #undef EEPROM_SPI
  #undef EEPROM_I2C
  #undef EEPROM_FLASH
#endif
#define HAS_EEPROM_SPI      (ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_SPI))
#define HAS_EEPROM_I2C      (ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_I2C))
#define HAS_EEPROM_SD       (ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_SD) && ENABLED(SDSUPPORT))
#define HAS_EEPROM_FLASH    (ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_FLASH))
#define HAS_EEPROM           ENABLED(EEPROM_SETTINGS)

// SD support
#define HAS_SDSUPPORT       (ENABLED(SDSUPPORT))
#if ENABLED(SDSUPPORT) && ENABLED(ARDUINO_ARCH_SAM)
  #undef SDCARD_SORT_ALPHA
  #undef SDSORT_LIMIT
  #undef SDSORT_GCODE
  #undef SDSORT_USES_RAM
  #undef SDSORT_USES_STACK
  #undef SDSORT_CACHE_NAMES
  #undef SDSORT_DYNAMIC_RAM
  #define SDCARD_SORT_ALPHA
  #define SDSORT_LIMIT 256
  #define SDSORT_GCODE true
  #define SDSORT_USES_RAM false
  #define SDSORT_USES_STACK false
  #define SDSORT_CACHE_NAMES false
  #define SDSORT_DYNAMIC_RAM false
#endif
#if ENABLED(SDCARD_SORT_ALPHA)
  #define HAS_FOLDER_SORTING (FOLDER_SORTING || ENABLED(SDSORT_GCODE))
#endif

// Extruder Encoder
#define HAS_EXT_ENCODER     (ENABLED(EXTRUDER_ENCODER_CONTROL) && (HAS_E0_ENC || HAS_E1_ENC || HAS_E2_ENC || HAS_E3_ENC || HAS_E4_ENC || HAS_E5_ENC))

// Other
#define HAS_Z_PROBE_SLED    (ENABLED(Z_PROBE_SLED) && PIN_EXISTS(SLED))

/**
 * Shorthand for filament sensor and power sensor for ultralcd.cpp, dogm_lcd_implementation.h, ultralcd_implementation_hitachi_HD44780.h
 */
#define HAS_LCD_FILAMENT_SENSOR (HAS_FILAMENT_SENSOR && ENABLED(FILAMENT_LCD_DISPLAY))
#define HAS_LCD_POWER_SENSOR    (HAS_POWER_CONSUMPTION_SENSOR && ENABLED(POWER_CONSUMPTION_LCD_DISPLAY))

/**
 * Trinamic Stepper Drivers
 */
#define HAS_TRINAMIC    (ENABLED(HAVE_TMC2130)  || ENABLED(HAVE_TMC2208)  || ENABLED(IS_TRAMS))
#define  X_IS_TRINAMIC  (ENABLED( X_IS_TMC2130) || ENABLED( X_IS_TMC2208) || ENABLED(IS_TRAMS))
#define X2_IS_TRINAMIC  (ENABLED(X2_IS_TMC2130) || ENABLED(X2_IS_TMC2208))
#define  Y_IS_TRINAMIC  (ENABLED( Y_IS_TMC2130) || ENABLED( Y_IS_TMC2208) || ENABLED(IS_TRAMS))
#define Y2_IS_TRINAMIC  (ENABLED(Y2_IS_TMC2130) || ENABLED(Y2_IS_TMC2208))
#define  Z_IS_TRINAMIC  (ENABLED( Z_IS_TMC2130) || ENABLED( Z_IS_TMC2208) || ENABLED(IS_TRAMS))
#define Z2_IS_TRINAMIC  (ENABLED(Z2_IS_TMC2130) || ENABLED(Z2_IS_TMC2208))
#define E0_IS_TRINAMIC  (ENABLED(E0_IS_TMC2130) || ENABLED(E0_IS_TMC2208) || ENABLED(IS_TRAMS))
#define E1_IS_TRINAMIC  (ENABLED(E1_IS_TMC2130) || ENABLED(E1_IS_TMC2208))
#define E2_IS_TRINAMIC  (ENABLED(E2_IS_TMC2130) || ENABLED(E2_IS_TMC2208))
#define E3_IS_TRINAMIC  (ENABLED(E3_IS_TMC2130) || ENABLED(E3_IS_TMC2208))
#define E4_IS_TRINAMIC  (ENABLED(E4_IS_TMC2130) || ENABLED(E4_IS_TMC2208))
#define E5_IS_TRINAMIC  (ENABLED(E5_IS_TMC2130) || ENABLED(E5_IS_TMC2208))

/**
 * ENDSTOPPULLUPS
 */
#if DISABLED(ENDSTOPPULLUP_XMIN)
  #define ENDSTOPPULLUP_XMIN    false
#endif
#if DISABLED(ENDSTOPPULLUP_YMIN)
  #define ENDSTOPPULLUP_YMIN    false
#endif
#if DISABLED(ENDSTOPPULLUP_ZMIN)
  #define ENDSTOPPULLUP_ZMIN    false
#endif
#if DISABLED(ENDSTOPPULLUP_XMAX)
  #define ENDSTOPPULLUP_XMAX    false
#endif
#if DISABLED(ENDSTOPPULLUP_YMAX)
  #define ENDSTOPPULLUP_YMAX    false
#endif
#if DISABLED(ENDSTOPPULLUP_ZMAX)
  #define ENDSTOPPULLUP_ZMAX    false
#endif
#if DISABLED(ENDSTOPPULLUP_X2MIN)
  #define ENDSTOPPULLUP_X2MIN   false
#endif
#if DISABLED(ENDSTOPPULLUP_Y2MIN)
  #define ENDSTOPPULLUP_Y2MIN   false
#endif
#if DISABLED(ENDSTOPPULLUP_Z2MIN)
  #define ENDSTOPPULLUP_Z2MIN   false
#endif
#if DISABLED(ENDSTOPPULLUP_X2MAX)
  #define ENDSTOPPULLUP_X2MAX   false
#endif
#if DISABLED(ENDSTOPPULLUP_Y2MAX)
  #define ENDSTOPPULLUP_Y2MAX   false
#endif
#if DISABLED(ENDSTOPPULLUP_Z2MAX)
  #define ENDSTOPPULLUP_Z2MAX   false
#endif
#if DISABLED(ENDSTOPPULLUP_ZPROBE)
  #define ENDSTOPPULLUP_ZPROBE  false
#endif

#if DISABLED(X_MIN_ENDSTOP_LOGIC)
  #define X_MIN_ENDSTOP_LOGIC   false
#endif
#if DISABLED(Y_MIN_ENDSTOP_LOGIC)
  #define Y_MIN_ENDSTOP_LOGIC   false
#endif
#if DISABLED(Z_MIN_ENDSTOP_LOGIC)
  #define Z_MIN_ENDSTOP_LOGIC   false
#endif
#if DISABLED(X_MAX_ENDSTOP_LOGIC)
  #define X_MAX_ENDSTOP_LOGIC   false
#endif
#if DISABLED(Y_MAX_ENDSTOP_LOGIC)
  #define Y_MAX_ENDSTOP_LOGIC   false
#endif
#if DISABLED(Z_MAX_ENDSTOP_LOGIC)
  #define Z_MAX_ENDSTOP_LOGIC   false
#endif
#if DISABLED(X2_MIN_ENDSTOP_LOGIC)
  #define X2_MIN_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Y2_MIN_ENDSTOP_LOGIC)
  #define Y2_MIN_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Z2_MIN_ENDSTOP_LOGIC)
  #define Z2_MIN_ENDSTOP_LOGIC  false
#endif
#if DISABLED(X2_MAX_ENDSTOP_LOGIC)
  #define X2_MAX_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Y2_MAX_ENDSTOP_LOGIC)
  #define Y2_MAX_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Z2_MAX_ENDSTOP_LOGIC)
  #define Z2_MAX_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Z_PROBE_ENDSTOP_LOGIC)
  #define Z_PROBE_ENDSTOP_LOGIC false
#endif

/**
 * The BLTouch Probe emulates a servo probe
 */
#if ENABLED(BLTOUCH)
  #if HAS_Z_PROBE_PIN
    #define TEST_BLTOUCH() (READ(Z_PROBE_PIN) != endstops.isLogic(Z_PROBE))
  #else
    #define TEST_BLTOUCH() (READ(Z_MIN_PIN) != endstops.isLogic(Z_MIN))
  #endif
#endif

/**
 * Set granular options based on the specific type of leveling
 */

#define UBL_DELTA             (ENABLED(AUTO_BED_LEVELING_UBL) && MECH(DELTA))
#define ABL_PLANAR            (ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_3POINT))
#define ABL_GRID              (ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_BILINEAR))
#define HAS_ABL               (ABL_PLANAR || ABL_GRID || ENABLED(AUTO_BED_LEVELING_UBL))
#define HAS_LEVELING          (HAS_ABL || ENABLED(MESH_BED_LEVELING))
#define HAS_AUTOLEVEL         (HAS_ABL && DISABLED(PROBE_MANUALLY))
#define OLD_ABL         (HAS_ABL && DISABLED(AUTO_BED_LEVELING_UBL))
#define HAS_MESH              (ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(MESH_BED_LEVELING))
#define PLANNER_LEVELING      (ABL_PLANAR || ABL_GRID || ENABLED(MESH_BED_LEVELING) || UBL_DELTA)
#define HAS_PROBING_PROCEDURE (HAS_ABL || ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST))

#if HAS_PROBING_PROCEDURE
  #define PROBE_BED_WIDTH     abs(RIGHT_PROBE_BED_POSITION - (LEFT_PROBE_BED_POSITION))
  #define PROBE_BED_HEIGHT    abs(BACK_PROBE_BED_POSITION - (FRONT_PROBE_BED_POSITION))
#endif

/**
 * Bed Probing rectangular bounds
 * These can be further constrained in code for Delta and SCARA
 */
#if IS_DELTA
  // Check for this in the code instead
  #define MIN_PROBE_X -(mechanics.delta_print_radius)
  #define MAX_PROBE_X  (mechanics.delta_print_radius)
  #define MIN_PROBE_Y -(mechanics.delta_print_radius)
  #define MAX_PROBE_Y  (mechanics.delta_print_radius)
#elif IS_SCARA
    #define SCARA_PRINTABLE_RADIUS (SCARA_LINKAGE_1 + SCARA_LINKAGE_2)
    #define MIN_PROBE_X (X_CENTER - SCARA_PRINTABLE_RADIUS)
    #define MAX_PROBE_X (X_CENTER + SCARA_PRINTABLE_RADIUS)
    #define MIN_PROBE_Y (Y_CENTER - SCARA_PRINTABLE_RADIUS)
    #define MAX_PROBE_Y (Y_CENTER + SCARA_PRINTABLE_RADIUS)
#else
  // Boundaries for probing based on set limits
  #define MIN_PROBE_X (max(X_MIN_POS, X_MIN_POS + probe.offset[X_AXIS]))
  #define MAX_PROBE_X (min(X_MAX_POS, X_MAX_POS + probe.offset[X_AXIS]))
  #define MIN_PROBE_Y (max(Y_MIN_POS, Y_MIN_POS + probe.offset[Y_AXIS]))
  #define MAX_PROBE_Y (min(Y_MAX_POS, Y_MAX_POS + probe.offset[Y_AXIS]))
#endif

/**
 * Set GRID MAX POINTS
 */
#if ENABLED(GRID_MAX_POINTS_X) && ENABLED(GRID_MAX_POINTS_Y)
  #define GRID_MAX_POINTS ((GRID_MAX_POINTS_X) * (GRID_MAX_POINTS_Y))
#endif

/**
 * Z probe repetitions
 */
#if DISABLED(Z_PROBE_REPETITIONS)
  #define Z_PROBE_REPETITIONS 1
#endif

/**
 * Sled Options
 */
#if ENABLED(Z_PROBE_SLED)
  #define Z_SAFE_HOMING
#endif

/**
 * Safe Homing Options
 */
#if ENABLED(Z_SAFE_HOMING)
  #if DISABLED(Z_SAFE_HOMING_X_POINT)
    #define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)
  #endif
  #if DISABLED(Z_SAFE_HOMING_Y_POINT)
    #define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)
  #endif
  #define X_TILT_FULCRUM Z_SAFE_HOMING_X_POINT
  #define Y_TILT_FULCRUM Z_SAFE_HOMING_Y_POINT
#else
  #define X_TILT_FULCRUM X_HOME_POS
  #define Y_TILT_FULCRUM Y_HOME_POS
#endif

/**
 * Host keep alive
 */
#if DISABLED(DEFAULT_KEEPALIVE_INTERVAL)
  #define DEFAULT_KEEPALIVE_INTERVAL 2
#endif

/**
 * DOUBLE_STEP_FREQUENCY for Arduino DUE or Mega
 */
#if ENABLED(ARDUINO_ARCH_SAM)
  #if ENABLED(LIN_ADVANCE)
    constexpr uint32_t DOUBLE_STEP_FREQUENCY = 60000; // 60KHz
  #else
    constexpr uint32_t DOUBLE_STEP_FREQUENCY = 80000; // 80Khz
  #endif
#else
  constexpr uint32_t DOUBLE_STEP_FREQUENCY = 10000;
#endif

/**
 * MAX_STEP_FREQUENCY differs for TOSHIBA
 */
#if ENABLED(CONFIG_STEPPERS_TOSHIBA)
  constexpr uint32_t MAX_STEP_FREQUENCY = DOUBLE_STEP_FREQUENCY;        // Max step frequency for Toshiba Stepper Controllers, 96kHz is close to maximum for an Arduino Due
#else
  constexpr uint32_t MAX_STEP_FREQUENCY = (DOUBLE_STEP_FREQUENCY * 4);  // Max step frequency for the Due is approx. 330kHz
#endif

// MS1 MS2 Stepper Driver Microstepping mode table
#define MICROSTEP1 LOW,LOW
#define MICROSTEP2 HIGH,LOW
#define MICROSTEP4 LOW,HIGH
#define MICROSTEP8 HIGH,HIGH
#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
  #define MICROSTEP16 LOW,LOW
  #define MICROSTEP32 HIGH,HIGH
#else
  #define MICROSTEP16 HIGH,HIGH
#endif

/**
 * SD DETECT
 */
#if ENABLED(SD_DISABLED_DETECT)
  #undef SD_DETECT_PIN
  #define SD_DETECT_PIN   -1
#endif
#if ENABLED(ULTIPANEL) && DISABLED(ELB_FULL_GRAPHIC_CONTROLLER)
  #undef SD_DETECT_INVERTED
#endif

/**
 * Power Signal Control Definitions
 * By default use Normal definition
 */
#if DISABLED(POWER_SUPPLY)
  #define POWER_SUPPLY 0
#endif
#if (POWER_SUPPLY == 1)     // 1 = ATX
  #define PS_ON_AWAKE  LOW
  #define PS_ON_ASLEEP HIGH
#elif (POWER_SUPPLY == 2)   // 2 = X-Box 360 203W
  #define PS_ON_AWAKE  HIGH
  #define PS_ON_ASLEEP LOW
#endif
#if DISABLED(DELAY_AFTER_POWER_ON)
  #define DELAY_AFTER_POWER_ON 5
#endif
#define HAS_POWER_SWITCH (POWER_SUPPLY > 0 && PIN_EXISTS(PS_ON))

/**
 * Temp Sensor defines
 */
#if TEMP_SENSOR_0 == -3
  #define SUPPORT_MAX31855
#elif TEMP_SENSOR_0 == -2
  #define SUPPORT_MAX6675
#elif TEMP_SENSOR_0 == -1
  #define HEATER_0_USES_AD595
#elif TEMP_SENSOR_0 > 0 && TEMP_SENSOR_0 < 10
  #define HEATER_0_USES_THERMISTOR
#elif TEMP_SENSOR_0 == 11
  #define HEATER_0_USES_DHT11
#elif TEMP_SENSOR_0 == 20
  #define THERMISTORHEATER_0 TEMP_SENSOR_0
  #define HEATER_0_USES_AMPLIFIER
#endif

#if TEMP_SENSOR_1 == -3
  #if DISABLED(SUPPORT_MAX31855)
    #define SUPPORT_MAX31855
  #endif
#elif TEMP_SENSOR_1 == -2
  #if DISABLED(SUPPORT_MAX6675)
    #define SUPPORT_MAX6675
  #endif
#elif TEMP_SENSOR_1 == -1
  #define HEATER_1_USES_AD595
#elif TEMP_SENSOR_1 > 0 && TEMP_SENSOR_1 < 10
  #define HEATER_1_USES_THERMISTOR
#elif TEMP_SENSOR_1 == 11
  #define HEATER_1_USES_DHT11
#elif TEMP_SENSOR_1 == 20
  #define THERMISTORHEATER_1 TEMP_SENSOR_1
  #define HEATER_1_USES_AMPLIFIER
#endif

#if TEMP_SENSOR_2 == -3
  #if DISABLED(SUPPORT_MAX31855)
    #define SUPPORT_MAX31855
  #endif
#elif TEMP_SENSOR_2 == -2
  #if DISABLED(SUPPORT_MAX6675)
    #define SUPPORT_MAX6675
  #endif
#elif TEMP_SENSOR_2 == -1
  #define HEATER_2_USES_AD595
#elif TEMP_SENSOR_2 > 0 && TEMP_SENSOR_2 < 10
  #define HEATER_2_USES_THERMISTOR
#elif TEMP_SENSOR_2 == 11
  #define HEATER_2_USES_DHT11
#elif TEMP_SENSOR_2 == 20
  #define THERMISTORHEATER_2 TEMP_SENSOR_2
  #define HEATER_2_USES_AMPLIFIER
#endif

#if TEMP_SENSOR_3 == -3
  #if DISABLED(SUPPORT_MAX31855)
    #define SUPPORT_MAX31855
  #endif
#elif TEMP_SENSOR_3 == -2
  #if DISABLED(SUPPORT_MAX6675)
    #define SUPPORT_MAX6675
  #endif
#elif TEMP_SENSOR_3 == -1
  #define HEATER_3_USES_AD595
#elif TEMP_SENSOR_3 > 0 && TEMP_SENSOR_3 < 10
  #define HEATER_3_USES_THERMISTOR
#elif TEMP_SENSOR_3 == 11
  #define HEATER_3_USES_DHT11
#elif TEMP_SENSOR_3 == 20
  #define THERMISTORHEATER_3 TEMP_SENSOR_3
  #define HEATER_3_USES_AMPLIFIER
#endif

#if TEMP_SENSOR_BED <= -2
  #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_BED"
#elif TEMP_SENSOR_BED == -1
  #define BED_USES_AD595
#elif TEMP_SENSOR_BED > 0 && TEMP_SENSOR_BED < 10
  #define BED_USES_THERMISTOR
#elif TEMP_SENSOR_BED == 11
  #define BED_USES_DHT11
#elif TEMP_SENSOR_BED == 20
  #define THERMISTORBED TEMP_SENSOR_BED
  #define BED_USES_AMPLIFIER
#endif

#if TEMP_SENSOR_CHAMBER <= -2
  #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_CHAMBER"
#elif TEMP_SENSOR_CHAMBER == -1
  #define CHAMBER_USES_AD595
#elif TEMP_SENSOR_CHAMBER > 0 && TEMP_SENSOR_CHAMBER < 10
  #define CHAMBER_USES_THERMISTOR
#elif TEMP_SENSOR_CHAMBER == 11
  #define CHAMBER_USES_DHT11
#elif TEMP_SENSOR_CHAMBER == 20
  #define THERMISTORCHAMBER TEMP_SENSOR_CHAMBER
  #define CHAMBER_USES_AMPLIFIER
#endif

#if TEMP_SENSOR_COOLER <= -2
  #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_COOLER"
#elif TEMP_SENSOR_COOLER == -1
  #define COOLER_USES_AD595
#elif TEMP_SENSOR_COOLER > 0 && TEMP_SENSOR_COOLER < 10
  #define COOLER_USES_THERMISTOR
#elif TEMP_SENSOR_COOLER == 11
  #define COOLER_USES_DHT11
#elif TEMP_SENSOR_COOLER == 20
  #define THERMISTORCOOLER TEMP_SENSOR_COOLER
  #define COOLER_USES_AMPLIFIER
#endif

#define HEATER_USES_AD595     (ENABLED(HEATER_0_USES_AD595) || ENABLED(HEATER_1_USES_AD595) || ENABLED(HEATER_2_USES_AD595) || ENABLED(HEATER_3_USES_AD595) || ENABLED(BED_USES_AD595) || ENABLED(CHAMBER_USES_AD595) || ENABLED(COOLER_USES_AD595))
#define HEATER_USES_MAX       (ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855))
#define HEATER_USES_AMPLIFIER (ENABLED(HEATER_0_USES_AMPLIFIER) || ENABLED(HEATER_1_USES_AMPLIFIER) || ENABLED(HEATER_2_USES_AMPLIFIER) || ENABLED(HEATER_3_USES_AMPLIFIER) || ENABLED(BED_USES_AMPLIFIER) || ENABLED(CHAMBER_USES_AMPLIFIER) || ENABLED(COOLER_USES_AMPLIFIER))

/**
 * Heaters
 */
#if HAS_HEATER_0
  #define HOT0_COUNT 1
  #define HOT0_INDEX 0
  #define HOT0_CHANNEL  HEATER_0_PIN
  #define HOT0_COMMA ,
#else
  #define HOT0_COUNT 0
  #define HOT0_INDEX
  #define HOT0_CHANNEL
  #define HOT0_COMMA
#endif
#if HAS_HEATER_1
  #define HOT1_COUNT 1
  #define HOT1_INDEX    HOT0_COUNT
  #define HOT1_CHANNEL  HOT0_COMMA HEATER_1_PIN
  #define HOT1_COMMA ,
#else
  #define HOT1_COUNT 0
  #define HOT1_INDEX
  #define HOT1_CHANNEL
  #define HOT1_COMMA    HOT0_COMMA
#endif
#if HAS_HEATER_2
  #define HOT2_COUNT 1
  #define HOT2_INDEX    HOT0_COUNT+HOT1_COUNT
  #define HOT2_CHANNEL  HOT1_COMMA HEATER_2_PIN
  #define HOT2_COMMA ,
#else
  #define HOT2_COUNT 0
  #define HOT2_INDEX
  #define HOT2_CHANNEL
  #define HOT2_COMMA    HOT1_COMMA
#endif
#if HAS_HEATER_3
  #define HOT3_COUNT 1
  #define HOT3_INDEX    HOT0_COUNT+HOT1_COUNT+HOT2_COUNT
  #define HOT3_CHANNEL  HOT2_COMMA HEATER_3_PIN
  #define HOT3_COMMA ,
#else
  #define HOT3_COUNT 0
  #define HOT3_INDEX
  #define HOT3_CHANNEL
  #define HOT3_COMMA    HOT2_COMMA
#endif
#if HAS_HEATER_BED
  #define BED_COUNT 1
  #define BED_INDEX     HOT0_COUNT+HOT1_COUNT+HOT2_COUNT+HOT3_COUNT
  #define BED_CHANNEL   HOT3_COMMA HEATER_BED_PIN
  #define BED_COMMA ,
#else
  #define BED_COUNT 0
  #define BED_INDEX
  #define BED_CHANNEL
  #define BED_COMMA     HOT3_COMMA
#endif
#if HAS_HEATER_CHAMBER
  #define CHAMBER_COUNT 1
  #define CHAMBER_INDEX   HOT0_COUNT+HOT1_COUNT+HOT2_COUNT+HOT3_COUNT+BED_COUNT
  #define CHAMBER_CHANNEL BED_COMMA HEATER_CHAMBER_PIN
  #define CHAMBER_COMMA ,
#else
  #define CHAMBER_COUNT 0
  #define CHAMBER_INDEX
  #define CHAMBER_CHANNEL
  #define CHAMBER_COMMA   BED_COMMA
#endif
#if HAS_HEATER_COOLER
  #define COOLER_COUNT 1
  #define COOLER_INDEX    HOT0_COUNT+HOT1_COUNT+HOT2_COUNT+HOT3_COUNT+BED_COUNT+CHAMBER_COUNT
  #define COOLER_CHANNEL  CHAMBER_COMMA HEATER_COOLER_PIN
  #define COOLER_COMMA ,
#else
  #define COOLER_COUNT 0
  #define COOLER_INDEX
  #define COOLER_CHANNEL
  #define COOLER_COMMA    CHAMBER_COMMA
#endif

#define HEATER_COUNT  (HOT0_COUNT+HOT1_COUNT+HOT2_COUNT+HOT3_COUNT+BED_COUNT+CHAMBER_COUNT+COOLER_COUNT)
#define HEATER_TYPE 4
#if HEATER_COUNT > 0
  #define HEATER_CHANNELS {HOT0_CHANNEL HOT1_CHANNEL HOT2_CHANNEL HOT3_CHANNEL BED_CHANNEL CHAMBER_CHANNEL COOLER_CHANNEL}
#else
  #define HEATER_CHANNELS { }
#endif

#if INVERTED_HEATER_PINS
  #define WRITE_HEATER(pin, value) WRITE(pin, !value)
#else
  #define WRITE_HEATER(pin, value) WRITE(pin, value)
#endif
#if INVERTED_BED_PIN
  #define WRITE_HEATER_BED(v) WRITE(HEATER_BED_PIN,!v)
#else
  #define WRITE_HEATER_BED(v) WRITE(HEATER_BED_PIN,v)
#endif
#if INVERTED_CHAMBER_PIN
  #define WRITE_HEATER_CHAMBER(v) WRITE(HEATER_CHAMBER_PIN,!v)
#else
  #define WRITE_HEATER_CHAMBER(v) WRITE(HEATER_CHAMBER_PIN,v)
#endif
#if INVERTED_COOLER_PIN
  #define WRITE_HEATER_COOLER(v) WRITE(COOLER_PIN,!v)
#else
  #define WRITE_HEATER_COOLER(v) WRITE(COOLER_PIN,v)
#endif

/**
 * FANS
 */
#if HAS_FAN0
  #define FAN0_COUNT 1
  #define FAN0_INDEX 0
  #define FAN0_CHANNEL FAN0_PIN
  #define FAN0_COMMA ,
#else
  #define FAN0_COUNT 0
  #define FAN0_INDEX
  #define FAN0_CHANNEL
  #define FAN0_COMMA
#endif

#if HAS_FAN1
  #define FAN1_COUNT 1
  #define FAN1_INDEX FAN0_COUNT
  #define FAN1_CHANNEL FAN0_COMMA FAN1_PIN
  #define FAN1_COMMA ,
#else
  #define FAN1_COUNT 0
  #define FAN1_INDEX
  #define FAN1_CHANNEL
  #define FAN1_COMMA FAN0_COMMA
#endif

#if HAS_FAN2
  #define FAN2_COUNT 1
  #define FAN2_INDEX FAN0_COUNT+FAN1_COUNT
  #define FAN2_CHANNEL FAN1_COMMA FAN2_PIN
  #define FAN2_COMMA ,
#else
  #define FAN2_COUNT 0
  #define FAN2_INDEX
  #define FAN2_CHANNEL
  #define FAN2_COMMA FAN1_COMMA
#endif

#if HAS_FAN3
  #define FAN3_COUNT 1
  #define FAN3_INDEX FAN0_COUNT+FAN1_COUNT+FAN2_COUNT
  #define FAN3_CHANNEL FAN2_COMMA FAN3_PIN
  #define FAN3_COMMA ,
#else
  #define FAN3_COUNT 0
  #define FAN3_INDEX
  #define FAN3_CHANNEL
  #define FAN3_COMMA FAN2_COMMA
#endif

#if HAS_FAN4
  #define FAN4_COUNT 1
  #define FAN4_INDEX FAN0_COUNT+FAN1_COUNT+FAN2_COUNT+FAN3_COUNT
  #define FAN4_CHANNEL FAN3_COMMA FAN4_PIN
  #define FAN4_COMMA ,
#else
  #define FAN4_COUNT 0
  #define FAN4_INDEX
  #define FAN4_CHANNEL
  #define FAN4_COMMA FAN3_COMMA
#endif

#if HAS_FAN5
  #define FAN5_COUNT 1
  #define FAN5_INDEX FAN0_COUNT+FAN1_COUNT+FAN2_COUNT+FAN3_COUNT+FAN4_COUNT
  #define FAN5_CHANNEL FAN4_COMMA FAN5_PIN
  #define FAN5_COMMA ,
#else
  #define FAN5_COUNT 0
  #define FAN5_INDEX
  #define FAN5_CHANNEL
  #define FAN5_COMMA FAN4_COMMA
#endif

#define FAN_COUNT       (FAN0_COUNT+FAN1_COUNT+FAN2_COUNT+FAN3_COUNT+FAN4_COUNT+FAN5_COUNT)

#if FAN_COUNT > 0
  #define FANS_CHANNELS {FAN0_CHANNEL FAN1_CHANNEL FAN2_CHANNEL FAN3_CHANNEL FAN4_CHANNEL FAN5_CHANNEL }
#else
  #define FANS_CHANNELS { }
#endif

#if ENABLED(INVERTED_FAN_PINS)
  #define FAN_INVERTED true
#else
  #define FAN_INVERTED false
#endif

/**
 * Extruder Encoder
 */
#if HAS_EXT_ENCODER
  #if ENABLED(INVERTED_ENCODER_PINS)
    #define READ_ENCODER(v) !READ(v)
  #else
    #define READ_ENCODER(v) READ(v)
  #endif
#endif

/**
 * Heater & Fan Pausing
 */
#if FAN_COUNT == 0
  #undef PROBING_FANS_OFF
#endif
#define QUIET_PROBING       (HAS_BED_PROBE && (ENABLED(PROBING_HEATERS_OFF) || ENABLED(PROBING_FANS_OFF)))
#define HEATER_IDLE_HANDLER (ENABLED(ADVANCED_PAUSE_FEATURE) || ENABLED(PROBING_HEATERS_OFF))

#if HAS_CNCROUTER
  #if ENABLED(INVERTED_CNCROUTER_PIN)
    #define WRITE_CNCROUTER(v) WRITE(CNCROUTER_PIN, !v)
  #else
    #define WRITE_CNCROUTER(v) WRITE(CNCROUTER_PIN, v)
  #endif
#endif

/**
 * Multiextruder with relè system
 */
#if ENABLED(MKR4) || ENABLED(MKR6) || ENABLED(MKR12)
  #if ENABLED(INVERTED_RELE_PINS)
    #define WRITE_RELE(pin, value) WRITE(pin, !value)
    #define OUT_WRITE_RELE(pin, value) OUT_WRITE(pin, !value)
  #else
    #define WRITE_RELE(pin, value) WRITE(pin, value)
    #define OUT_WRITE_RELE(pin, value) OUT_WRITE(pin, value)
  #endif
#endif

/**
 * Buzzer/Speaker
 */
#if ENABLED(LCD_USE_I2C_BUZZER)
  #if DISABLED(LCD_FEEDBACK_FREQUENCY_HZ)
    #define LCD_FEEDBACK_FREQUENCY_HZ 1000
  #endif
  #if DISABLED(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
    #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
  #endif
#else
  #if DISABLED(LCD_FEEDBACK_FREQUENCY_HZ)
    #define LCD_FEEDBACK_FREQUENCY_HZ 5000
  #endif
  #if DISABLED(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
    #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 10
  #endif
#endif

/**
 * VIKI2 and miniVIKI require DOGLCD_SCK and DOGLCD_MOSI to be defined.
 */
#if ENABLED(VIKI2) || ENABLED(miniVIKI)
  #ifndef DOGLCD_SCK
    #define DOGLCD_SCK  SCK_PIN
  #endif
  #ifndef DOGLCD_MOSI
    #define DOGLCD_MOSI MOSI_PIN
  #endif
#endif

/**
 * MIN_Z_HEIGHT_FOR_HOMING / Z_PROBE_BETWEEN_HEIGHT
 */
#if DISABLED(MIN_Z_HEIGHT_FOR_HOMING)
  #if DISABLED(Z_PROBE_BETWEEN_HEIGHT)
    #define MIN_Z_HEIGHT_FOR_HOMING 0
  #else
    #define MIN_Z_HEIGHT_FOR_HOMING Z_PROBE_BETWEEN_HEIGHT
  #endif
#endif
#if DISABLED(Z_PROBE_BETWEEN_HEIGHT)
  #define Z_PROBE_BETWEEN_HEIGHT MIN_Z_HEIGHT_FOR_HOMING
#endif
#if Z_PROBE_BETWEEN_HEIGHT > MIN_Z_HEIGHT_FOR_HOMING
  #define MANUAL_PROBE_HEIGHT Z_PROBE_BETWEEN_HEIGHT
#else
  #define MANUAL_PROBE_HEIGHT MIN_Z_HEIGHT_FOR_HOMING
#endif

/**
 * Servos
 */
#if HAS_SERVOS
  #if DISABLED(Z_ENDSTOP_SERVO_NR)
    #define Z_ENDSTOP_SERVO_NR -1
  #endif
#endif

/**
 * Set a flag for a servo probe
 */
#define HAS_Z_SERVO_PROBE (HAS_SERVOS && ENABLED(Z_ENDSTOP_SERVO_NR) && Z_ENDSTOP_SERVO_NR >= 0)

/**
 * Set a flag for any enabled probe
 */
#define PROBE_SELECTED        (ENABLED(PROBE_MANUALLY) || ENABLED(Z_PROBE_FIX_MOUNTED) || ENABLED(Z_PROBE_SLED) || ENABLED(Z_PROBE_ALLEN_KEY) || HAS_Z_SERVO_PROBE)
#define PROBE_PIN_CONFIGURED  (HAS_Z_PROBE_PIN || HAS_Z_MIN)
#define HAS_BED_PROBE         ((PROBE_SELECTED && PROBE_PIN_CONFIGURED) && DISABLED(PROBE_MANUALLY))

#if ENABLED(Z_PROBE_ALLEN_KEY)
  #define PROBE_IS_TRIGGERED_WHEN_STOWED_TEST
#endif

/**
 * Nextion Manual BED leveling
 */
#define HAS_NEXTION_MANUAL_BED (ENABLED(LCD_BED_LEVELING) && ENABLED(PROBE_MANUALLY) && ENABLED(NEXTION))

/**
 * Bed Probe dependencies
 */
#if HAS_BED_PROBE
  #if DISABLED(X_PROBE_OFFSET_FROM_NOZZLE)
    #define X_PROBE_OFFSET_FROM_NOZZLE 0
  #endif
  #if DISABLED(Y_PROBE_OFFSET_FROM_NOZZLE)
    #define Y_PROBE_OFFSET_FROM_NOZZLE 0
  #endif
  #if DISABLED(Z_PROBE_OFFSET_FROM_NOZZLE)
    #define Z_PROBE_OFFSET_FROM_NOZZLE 0
  #endif
  #if DISABLED(Z_PROBE_OFFSET_RANGE_MIN)
    #define Z_PROBE_OFFSET_RANGE_MIN -50
  #endif
  #if DISABLED(Z_PROBE_OFFSET_RANGE_MAX)
    #define Z_PROBE_OFFSET_RANGE_MAX 50
  #endif
  #if DISABLED(XY_PROBE_SPEED)
    #if ENABLED(HOMING_FEEDRATE_X)
      #define XY_PROBE_SPEED HOMING_FEEDRATE_X
    #elif define HOMING_FEEDRATE_XYZ
      #define XY_PROBE_SPEED HOMING_FEEDRATE_XYZ
    #else
      #define XY_PROBE_SPEED 3200
    #endif
  #endif
  #if Z_PROBE_BETWEEN_HEIGHT > Z_PROBE_DEPLOY_HEIGHT
    #define _Z_PROBE_DEPLOY_HEIGHT Z_PROBE_BETWEEN_HEIGHT
  #else
    #define _Z_PROBE_DEPLOY_HEIGHT Z_PROBE_DEPLOY_HEIGHT
  #endif
#else
  #undef X_PROBE_OFFSET_FROM_NOZZLE
  #undef Y_PROBE_OFFSET_FROM_NOZZLE
  #undef Z_PROBE_OFFSET_FROM_NOZZLE
  #define X_PROBE_OFFSET_FROM_NOZZLE 0
  #define Y_PROBE_OFFSET_FROM_NOZZLE 0
  #define Z_PROBE_OFFSET_FROM_NOZZLE 0
  #define _Z_PROBE_DEPLOY_HEIGHT (Z_MAX_POS / 2)
#endif

// Add commands that need sub-codes to this list
#define USE_GCODE_SUBCODES ENABLED(G38_PROBE_TARGET)

// MUVE 3D
#if MECH(MUVE3D) && ENABLED(PROJECTOR_PORT) && ENABLED(PROJECTOR_BAUDRATE)
  #if PROJECTOR_PORT == 1
    #define DLPSerial Serial1
  #elif PROJECTOR_PORT == 2
    #define DLPSerial Serial2
  #elif PROJECTOR_PORT == 3
    #define DLPSerial Serial3
  #else
    #define DLPSerial Serial2
  #endif
#endif

// PWM SPEED and MASK
#if DISABLED(HEATER_PWM_SPEED)
  #define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED < 0
  #define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED > 4
  #define HEATER_PWM_SPEED 4
#endif

#if HEATER_PWM_SPEED == 0
  #define HEATER_PWM_STEP 1
  #define HEATER_PWM_FREQ 15
  #define HEATER_PWM_MASK 255
#elif HEATER_PWM_SPEED == 1
  #define HEATER_PWM_STEP 2
  #define HEATER_PWM_FREQ 30
  #define HEATER_PWM_MASK 254
#elif HEATER_PWM_SPEED == 2
  #define HEATER_PWM_STEP 4
  #define HEATER_PWM_FREQ 61
  #define HEATER_PWM_MASK 252
#elif HEATER_PWM_SPEED == 3
  #define HEATER_PWM_STEP 8
  #define HEATER_PWM_FREQ 122
  #define HEATER_PWM_MASK 248
#elif HEATER_PWM_SPEED == 4
  #define HEATER_PWM_STEP 16
  #define HEATER_PWM_FREQ 244
  #define HEATER_PWM_MASK 240
#endif

#if DISABLED(FAN_PWM_SPEED)
  #define FAN_PWM_SPEED 0
#endif
#if FAN_PWM_SPEED < 0
  #define FAN_PWM_SPEED 0
#endif
#if FAN_PWM_SPEED > 4
  #define FAN_PWM_SPEED 4
#endif

#if FAN_PWM_SPEED == 0
  #define FAN_PWM_STEP 1
  #define FAN_PWM_FREQ 15
  #define FAN_PWM_MASK 255
#elif FAN_PWM_SPEED == 1
  #define FAN_PWM_STEP 2
  #define FAN_PWM_FREQ 30
  #define FAN_PWM_MASK 254
#elif FAN_PWM_SPEED == 2
  #define FAN_PWM_STEP 4
  #define FAN_PWM_FREQ 61
  #define FAN_PWM_MASK 252
#elif FAN_PWM_SPEED == 3
  #define FAN_PWM_STEP 8
  #define FAN_PWM_FREQ 122
  #define FAN_PWM_MASK 248
#elif FAN_PWM_SPEED == 4
  #define FAN_PWM_STEP 16
  #define FAN_PWM_FREQ 244
  #define FAN_PWM_MASK 240
#endif

// TEMPERATURE
#if (HOTENDS > 0 && HAS_TEMP_0)
  #define HOT0_ANALOG_INPUTS        1
  #define HOT0_ANALOG_INDEX         0
  #define HOT0_ANALOG_CHANNEL       TEMP_0_PIN
  #define HOT0_ANALOG_COMMA         ,
#else
  #define HOT0_ANALOG_INPUTS        0
  #define HOT0_ANALOG_INDEX         -1
  #define HOT0_ANALOG_CHANNEL
  #define HOT0_ANALOG_COMMA
#endif

#if (HOTENDS > 1 && HAS_TEMP_1)
  #define HOT1_ANALOG_INPUTS        1
  #define HOT1_ANALOG_INDEX         HOT0_ANALOG_INPUTS
  #define HOT1_ANALOG_CHANNEL       HOT0_ANALOG_COMMA TEMP_1_PIN
  #define HOT1_ANALOG_COMMA         ,
#else
  #define HOT1_ANALOG_INPUTS        0
  #define HOT1_ANALOG_INDEX         -1
  #define HOT1_ANALOG_CHANNEL
  #define HOT1_ANALOG_COMMA         HOT0_ANALOG_COMMA
#endif

#if (HOTENDS > 2 && HAS_TEMP_2)
  #define HOT2_ANALOG_INPUTS        1
  #define HOT2_ANALOG_INDEX         HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS
  #define HOT2_ANALOG_CHANNEL       HOT1_ANALOG_COMMA TEMP_2_PIN
  #define HOT2_ANALOG_COMMA         ,
#else
  #define HOT2_ANALOG_INPUTS        0
  #define HOT2_ANALOG_INDEX         -1
  #define HOT2_ANALOG_CHANNEL
  #define HOT2_ANALOG_COMMA         HOT1_ANALOG_COMMA
#endif

#if (HOTENDS > 3 && HAS_TEMP_3)
  #define HOT3_ANALOG_INPUTS        1
  #define HOT3_ANALOG_INDEX         HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS
  #define HOT3_ANALOG_CHANNEL       HOT2_ANALOG_COMMA TEMP_3_PIN
  #define HOT3_ANALOG_COMMA         ,
#else
  #define HOT3_ANALOG_INPUTS        0
  #define HOT3_ANALOG_INDEX         -1
  #define HOT3_ANALOG_CHANNEL
  #define HOT3_ANALOG_COMMA         HOT2_ANALOG_COMMA
#endif

#if HAS_TEMP_BED
  #define BED_ANALOG_INPUTS         1
  #define BED_ANALOG_INDEX          HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS
  #define BED_ANALOG_CHANNEL        HOT3_ANALOG_COMMA TEMP_BED_PIN
  #define BED_ANALOG_COMMA          ,
#else
  #define BED_ANALOG_INPUTS         0
  #define BED_ANALOG_INDEX          -1
  #define BED_ANALOG_CHANNEL
  #define BED_ANALOG_COMMA          HOT3_ANALOG_COMMA
#endif

#if HAS_TEMP_CHAMBER
  #define CHAMBER_ANALOG_INPUTS     1
  #define CHAMBER_ANALOG_INDEX      HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS+BED_ANALOG_INPUTS
  #define CHAMBER_ANALOG_CHANNEL    BED_ANALOG_COMMA TEMP_CHAMBER_PIN
  #define CHAMBER_ANALOG_COMMA      ,
#else
  #define CHAMBER_ANALOG_INPUTS     0
  #define CHAMBER_ANALOG_INDEX      -1
  #define CHAMBER_ANALOG_CHANNEL
  #define CHAMBER_ANALOG_COMMA      BED_ANALOG_COMMA
#endif

#if HAS_TEMP_COOLER
  #define COOLER_ANALOG_INPUTS      1
  #define COOLER_ANALOG_INDEX       HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS+BED_ANALOG_INPUTS+CHAMBER_ANALOG_INPUTS
  #define COOLER_ANALOG_CHANNEL     CHAMBER_ANALOG_COMMA TEMP_COOLER_PIN
  #define COOLER_ANALOG_COMMA       ,
#else
  #define COOLER_ANALOG_INPUTS      0
  #define COOLER_ANALOG_INDEX       -1
  #define COOLER_ANALOG_CHANNEL
  #define COOLER_ANALOG_COMMA       CHAMBER_ANALOG_COMMA
#endif

#if HAS_FILAMENT_SENSOR
  #define FILAMENT_ANALOG_INPUTS    1
  #define FILAMENT_ANALOG_INDEX     HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS+BED_ANALOG_INPUTS+CHAMBER_ANALOG_INPUTS+COOLER_ANALOG_INPUTS
  #define FILAMENT_ANALOG_CHANNEL   COOLER_ANALOG_COMMA FILWIDTH_PIN
  #define FILAMENT_ANALOG_COMMA     ,
#else
  #define FILAMENT_ANALOG_INPUTS    0
  #define FILAMENT_ANALOG_INDEX     -1
  #define FILAMENT_ANALOG_CHANNEL
  #define FILAMENT_ANALOG_COMMA     COOLER_ANALOG_COMMA
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  #define POWER_ANALOG_INPUTS       1
  #define POWER_ANALOG_INDEX        HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS+BED_ANALOG_INPUTS+CHAMBER_ANALOG_INPUTS+COOLER_ANALOG_INPUTS+FILAMENT_ANALOG_INPUTS
  #define POWER_ANALOG_CHANNEL      FILAMENT_ANALOG_COMMA POWER_CONSUMPTION_PIN
  #define POWER_ANALOG_COMMA        ,
#else
  #define POWER_ANALOG_INPUTS       0
  #define POWER_ANALOG_INDEX        -1
  #define POWER_ANALOG_CHANNEL
  #define POWER_ANALOG_COMMA        FILAMENT_ANALOG_COMMA
#endif

#if ENABLED(ADC_KEYPAD) // Keypad for ANET board
  #define ADC_KEYPAD_ANALOG_INPUTS  1
  #define ADC_KEYPAD_ANALOG_INDEX   HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS+BED_ANALOG_INPUTS+CHAMBER_ANALOG_INPUTS+COOLER_ANALOG_INPUTS+FILAMENT_ANALOG_INPUTS+POWER_ANALOG_INPUTS
  #define ADC_KEYPAD_ANALOG_CHANNEL POWER_ANALOG_COMMA ADC_KEYPAD_PIN
  #define ADC_KEYPAD_ANALOG_COMMA   ,
#else
  #define ADC_KEYPAD_ANALOG_INPUTS  0
  #define ADC_KEYPAD_ANALOG_INDEX   -1
  #define ADC_KEYPAD_ANALOG_CHANNEL
  #define ADC_KEYPAD_ANALOG_COMMA   POWER_ANALOG_COMMA
#endif

#if HAS_MCU_TEMPERATURE
  #define MCU_ANALOG_INPUTS         1
#else
  #define MCU_ANALOG_INPUTS         0
#endif

#define ANALOG_INPUTS (HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS+BED_ANALOG_INPUTS+CHAMBER_ANALOG_INPUTS+COOLER_ANALOG_INPUTS+FILAMENT_ANALOG_INPUTS+POWER_ANALOG_INPUTS+ADC_KEYPAD_ANALOG_INPUTS+MCU_ANALOG_INPUTS)
#if ANALOG_INPUTS > 0
  /** Channels are the MUX-part of ADMUX register */
  #define ANALOG_INPUT_CHANNELS {HOT0_ANALOG_CHANNEL HOT1_ANALOG_CHANNEL HOT2_ANALOG_CHANNEL HOT3_ANALOG_CHANNEL BED_ANALOG_CHANNEL CHAMBER_ANALOG_CHANNEL COOLER_ANALOG_CHANNEL FILAMENT_ANALOG_CHANNEL POWER_ANALOG_CHANNEL ADC_KEYPAD_ANALOG_CHANNEL}
#else
  #define ANALOG_INPUT_CHANNELS { }
#endif

#endif /* _CONDITIONALS_POST_H_ */

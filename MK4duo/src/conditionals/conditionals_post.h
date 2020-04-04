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
#pragma once

/**
 * conditionals_post.h
 * Defines that depend on configuration but are not editable.
 */

/**
 * CPU 32 BIT
 */
#if ENABLED(CPU_32_BIT)
  #if DISABLED(EXTENDED_CAPABILITIES_REPORT)
    #define EXTENDED_CAPABILITIES_REPORT
  #endif
  #if DISABLED(ENDSTOP_INTERRUPTS_FEATURE) && DISABLED(SPI_ENDSTOPS)
    #define ENDSTOP_INTERRUPTS_FEATURE
  #endif
#endif

/**
 * Stored Position
 */
#if DISABLED(NUM_POSITON_SLOTS)
  #define NUM_POSITON_SLOTS 1
#elif NUM_POSITON_SLOTS < 1
  #define NUM_POSITON_SLOTS 1
#endif

/**
 * DELTA
 */
#if MECH(DELTA)

  // BLTouch
  #if ENABLED(BLTOUCH) && DISABLED(BLTOUCH_HIGH_SPEED_MODE)
    #define BLTOUCH_HIGH_SPEED_MODE
  #endif

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

  #if DISABLED(Z_PROBE_SPEED_FAST)
    #define Z_PROBE_SPEED_FAST  HOMING_FEEDRATE_XYZ
  #endif
  #if DISABLED(Z_PROBE_SPEED_SLOW)
    #define Z_PROBE_SPEED_SLOW  (Z_PROBE_SPEED_FAST / 2)
  #endif

  // Set the rectangle in which to probe
  #define DELTA_PROBEABLE_RADIUS    (DELTA_PRINTABLE_RADIUS - HYPOT(X_PROBE_OFFSET_FROM_NOZZLE, Y_PROBE_OFFSET_FROM_NOZZLE))
  #define LEFT_PROBE_BED_POSITION   (-mechanics.data.probe_radius)
  #define RIGHT_PROBE_BED_POSITION  ( mechanics.data.probe_radius)
  #define FRONT_PROBE_BED_POSITION  (-mechanics.data.probe_radius)
  #define BACK_PROBE_BED_POSITION   ( mechanics.data.probe_radius)

  #define X_MIN_BED   (-mechanics.data.print_radius)
  #define X_MAX_BED   ( mechanics.data.print_radius)
  #define Y_MIN_BED   (-mechanics.data.print_radius)
  #define Y_MAX_BED   ( mechanics.data.print_radius)
  #define Z_MAX_BED   ( mechanics.data.height)
  #define Z_MIN_BED   0

  #define MESH_MIN_X  (-mechanics.data.probe_radius)
  #define MESH_MAX_X  ( mechanics.data.probe_radius)
  #define MESH_MIN_Y  (-mechanics.data.probe_radius)
  #define MESH_MAX_Y  ( mechanics.data.probe_radius)

  #define HAS_DELTA_AUTO_CALIBRATION  (ENABLED(DELTA_AUTO_CALIBRATION_1) || ENABLED(DELTA_AUTO_CALIBRATION_2))

  // DELTA should ignore Z_SAFE_HOMING, SLOWDOWN, WORKSPACE_OFFSETS and LEVEL_BED_CORNERS
  #undef Z_SAFE_HOMING
  #undef SLOWDOWN
  #undef WORKSPACE_OFFSETS
  #undef LEVEL_BED_CORNERS

  // DOGM SPI DELAY
  #if DISABLED(DOGM_SPI_DELAY_US)
    #define DOGM_SPI_DELAY_US 50
  #endif

#else // !MECH(DELTA)

  #define LEFT_PROBE_BED_POSITION   (MAX(X_CENTER - X_MAX_LENGTH / 2, x_min))
  #define RIGHT_PROBE_BED_POSITION  (MIN(probe_position_lf.x + X_MAX_LENGTH, x_max))
  #define FRONT_PROBE_BED_POSITION  (MAX(Y_CENTER - Y_MAX_LENGTH / 2, y_min))
  #define BACK_PROBE_BED_POSITION   (MIN(probe_position_lf.y + Y_MAX_LENGTH, y_max))

  #define X_MIN_BED   (mechanics.data.base_pos.min.x)
  #define X_MAX_BED   (mechanics.data.base_pos.max.x)
  #define Y_MIN_BED   (mechanics.data.base_pos.min.y)
  #define Y_MAX_BED   (mechanics.data.base_pos.max.y)
  #define Z_MIN_BED   (mechanics.data.base_pos.min.z)
  #define Z_MAX_BED   (mechanics.data.base_pos.max.z)

  #define MESH_MIN_X  (X_MIN_BED + (MIN_PROBE_EDGE))
  #define MESH_MAX_X  (X_MAX_BED - (MIN_PROBE_EDGE))
  #define MESH_MIN_Y  (Y_MIN_BED + (MIN_PROBE_EDGE))
  #define MESH_MAX_Y  (Y_MAX_BED - (MIN_PROBE_EDGE))

#endif // !MECH(DELTA)

/**
 * Require 0,0 bed center for Delta and SCARA
 */
#if IS_KINEMATIC
  #define BED_CENTER_AT_0_0
#endif

/**
 * Axis lengths and center
 */
#define X_MAX_LENGTH (X_MAX_BED - (X_MIN_BED))
#define Y_MAX_LENGTH (Y_MAX_BED - (Y_MIN_BED))
#define Z_MAX_LENGTH (Z_MAX_BED - (Z_MIN_BED))

/**
 * Define center values
 */
#if ENABLED(BED_CENTER_AT_0_0)
  #define X_CENTER 0
  #define Y_CENTER 0
#else
  #define X_CENTER ((X_MAX_LENGTH) / 2)
  #define Y_CENTER ((Y_MAX_LENGTH) / 2)
#endif
#define Z_CENTER ((Z_MIN_BED + Z_MAX_BED) / 2)

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
  #define X_HOME_POS  MANUAL_X_HOME_POS
#elif MECH(DELTA)
  #define X_HOME_POS  0
#else
  #define X_HOME_POS  (mechanics.get_homedir(X_AXIS) < 0 ? mechanics.data.base_pos.min.x : mechanics.data.base_pos.max.x)
#endif

#if ENABLED(MANUAL_Y_HOME_POS)
  #define Y_HOME_POS  MANUAL_Y_HOME_POS
#elif MECH(DELTA)
  #define Y_HOME_POS  0
#else
  #define Y_HOME_POS  (mechanics.get_homedir(Y_AXIS) < 0 ? mechanics.data.base_pos.min.y : mechanics.data.base_pos.max.y)
#endif

#if ENABLED(MANUAL_Z_HOME_POS)
  #define Z_HOME_POS  MANUAL_Z_HOME_POS
#elif MECH(DELTA)
  #define Z_HOME_POS  (mechanics.data.height)
#else
  #define Z_HOME_POS  (mechanics.get_homedir(Z_AXIS) < 0 ? mechanics.data.base_pos.min.z : mechanics.data.base_pos.max.z)
#endif

/**
 * Auto Bed Leveling and Z Probe Repeatability Test
 */
#define HOMING_Z_WITH_PROBE (HAS_BED_PROBE && Z_HOME_DIR < 0 && DISABLED(Z_TWO_ENDSTOPS))

/**
 * Sled Options
 */
#if ENABLED(PROBE_SLED)
  #define Z_SAFE_HOMING
#endif

/**
 * Safe Homing Options
 */
#if ENABLED(Z_SAFE_HOMING)
  #if DISABLED(Z_SAFE_HOMING_X_POINT)
    #define Z_SAFE_HOMING_X_POINT ((mechanics.data.base_pos.min.x + mechanics.data.base_pos.max.x) / 2)
  #endif
  #if DISABLED(Z_SAFE_HOMING_Y_POINT)
    #define Z_SAFE_HOMING_Y_POINT ((mechanics.data.base_pos.min.y + mechanics.data.base_pos.max.y) / 2)
  #endif
#endif

/**
 * Host keep alive
 */
#if DISABLED(DEFAULT_KEEPALIVE_INTERVAL)
  #define DEFAULT_KEEPALIVE_INTERVAL 2
#endif

/**
 * Extruder Encoder
 */
#if ENABLED(EXTRUDER_ENCODER_CONTROL) && FILAMENT_RUNOUT_DISTANCE_MM == 0
  #undef FILAMENT_RUNOUT_DISTANCE_MM
  #define FILAMENT_RUNOUT_DISTANCE_MM 5
#endif

/**
 * LCD Contrast for Graphical Displays
 */
#if ENABLED(CARTESIO_UI)
  #define _LCD_CONTRAST_MIN   60
  #define _LCD_CONTRAST_INIT  90
  #define _LCD_CONTRAST_MAX  140
#elif ENABLED(miniVIKI)
  #define _LCD_CONTRAST_MIN   75
  #define _LCD_CONTRAST_INIT  95
  #define _LCD_CONTRAST_MAX  115
#elif ENABLED(VIKI2)
  #define _LCD_CONTRAST_INIT 140
#elif ENABLED(WANHAO_D6_OLED)
  #define _LCD_CONTRAST_MIN   10
  #define _LCD_CONTRAST_INIT 100
  #define _LCD_CONTRAST_MAX  255
#elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
  #define _LCD_CONTRAST_MIN   90
  #define _LCD_CONTRAST_INIT 110
  #define _LCD_CONTRAST_MAX  130
#elif ENABLED(AZSMZ_12864)
  #define _LCD_CONTRAST_MIN  120
  #define _LCD_CONTRAST_INIT 190
#elif ENABLED(MKS_MINI_12864)
  #define _LCD_CONTRAST_INIT 150
#elif ENABLED(FYSETC_MINI_12864_X_X) || ENABLED(FYSETC_MINI_12864_1_2) || ENABLED(FYSETC_MINI_12864_2_0) || ENABLED(FYSETC_MINI_12864_2_1)
  #define _LCD_CONTRAST_INIT 220
#elif ENABLED(ULTI_CONTROLLER)
  #define _LCD_CONTRAST_INIT 127
  #define _LCD_CONTRAST_MAX  254
#elif ENABLED(MAKRPANEL) || ENABLED(MINIPANEL)
  #define _LCD_CONTRAST_INIT  17
#endif

#define HAS_LCD_CONTRAST ENABLED(_LCD_CONTRAST_INIT)
#if HAS_LCD_CONTRAST
  #if DISABLED(LCD_CONTRAST_MIN)
    #if ENABLED(_LCD_CONTRAST_MIN)
      #define LCD_CONTRAST_MIN  _LCD_CONTRAST_MIN
    #else
      #define LCD_CONTRAST_MIN  0
    #endif
  #endif
  #if DISABLED(LCD_CONTRAST_INIT)
    #define LCD_CONTRAST_INIT   _LCD_CONTRAST_INIT
  #endif
  #if DISABLED(LCD_CONTRAST_MAX)
    #if ENABLED(_LCD_CONTRAST_MAX)
      #define LCD_CONTRAST_MAX  _LCD_CONTRAST_MAX
    #else
      #define LCD_CONTRAST_MAX  63
    #endif
  #endif
#endif

/**
 * Stepper
 */
#if DISABLED(DOUBLE_QUAD_STEPPING)
  #define DOUBLE_QUAD_STEPPING true
#endif

// MS1 MS2 Stepper Driver Microstepping mode table
#define MICROSTEP1 LOW,LOW
#define MICROSTEP2 HIGH,LOW
#define MICROSTEP4 LOW,HIGH
#define MICROSTEP8 HIGH,HIGH
#if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
  #define MICROSTEP16 LOW,LOW
  #define MICROSTEP32 HIGH,HIGH
#else
  #define MICROSTEP16 HIGH,HIGH
#endif

/**
 * SPI_SPEED
 */
#if ENABLED(SD_HALF_SPEED)
  #define SPI_SPEED SPI_HALF_SPEED
#elif ENABLED(SD_QUARTER_SPEED)
  #define SPI_SPEED SPI_QUARTER_SPEED
#elif ENABLED(SD_EIGHTH_SPEED)
  #define SPI_SPEED SPI_EIGHTH_SPEED
#elif ENABLED(SD_SIXTEENTH_SPEED)
  #define SPI_SPEED SPI_SIXTEENTH_SPEED
#else
  #define SPI_SPEED SPI_FULL_SPEED
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
#elif (POWER_SUPPLY == 2)   // 2 = X-Box 360
  #define PS_ON_AWAKE  HIGH
  #define PS_ON_ASLEEP LOW
#endif
#if DISABLED(POWER_NAME)
  #if (POWER_SUPPLY == 1)
    #define POWER_NAME "ATX"      // ATX style
  #elif (POWER_SUPPLY == 2)
    #define POWER_NAME "XBox"     // X-Box 360
  #else
    #define POWER_NAME "Generic"  // No control
  #endif
#endif
#if DISABLED(DELAY_AFTER_POWER_ON)
  #define DELAY_AFTER_POWER_ON 5
#endif
#define HAS_POWER_SWITCH (POWER_SUPPLY > 0 && PIN_EXISTS(PS_ON))

/**
 * Temp Sensor defines
 */
#define HAS_MAX31855  PIN_EXISTS(MAX31855_SS0)
#define HAS_MAX6675   PIN_EXISTS(MAX6675_SS)
#define HAS_AD8495    ( TEMP_SENSOR_HE0 == -2       ||  TEMP_SENSOR_HE1 == -2       ||  TEMP_SENSOR_HE2 == -2                                     \
                    ||  TEMP_SENSOR_HE3 == -2       ||  TEMP_SENSOR_HE4 == -2       ||  TEMP_SENSOR_HE5 == -2                                     \
                    ||  TEMP_SENSOR_BED0 == -2      ||  TEMP_SENSOR_BED1 == -2      ||  TEMP_SENSOR_BED2 == -2      ||  TEMP_SENSOR_BED3 == -2    \
                    ||  TEMP_SENSOR_CHAMBER0 == -2  ||  TEMP_SENSOR_CHAMBER1 == -2  ||  TEMP_SENSOR_CHAMBER2 == -2  ||  TEMP_SENSOR_CHAMBER3 == -2)

#define HAS_AD595     ( TEMP_SENSOR_HE0 == -1       ||  TEMP_SENSOR_HE1 == -1       ||  TEMP_SENSOR_HE2 == -1                                     \
                    ||  TEMP_SENSOR_HE3 == -1       ||  TEMP_SENSOR_HE4 == -1       ||  TEMP_SENSOR_HE5 == -1                                     \
                    ||  TEMP_SENSOR_BED0 == -1      ||  TEMP_SENSOR_BED1 == -1      ||  TEMP_SENSOR_BED2 == -1      ||  TEMP_SENSOR_BED3 == -1    \
                    ||  TEMP_SENSOR_CHAMBER0 == -1  ||  TEMP_SENSOR_CHAMBER1 == -1  ||  TEMP_SENSOR_CHAMBER2 == -1  ||  TEMP_SENSOR_CHAMBER3 == -1)

#define HAS_AMPLIFIER ( TEMP_SENSOR_HE0 == 20       ||  TEMP_SENSOR_HE1 == 20       ||  TEMP_SENSOR_HE2 == 20                                     \
                    ||  TEMP_SENSOR_HE3 == 20       ||  TEMP_SENSOR_HE4 == 20       ||  TEMP_SENSOR_HE5 == 20                                     \
                    ||  TEMP_SENSOR_BED0 == 20      ||  TEMP_SENSOR_BED1 == 20      ||  TEMP_SENSOR_BED2 == 20      ||  TEMP_SENSOR_BED3 == 20    \
                    ||  TEMP_SENSOR_CHAMBER0 == 20  ||  TEMP_SENSOR_CHAMBER1 == 20  ||  TEMP_SENSOR_CHAMBER2 == 20  ||  TEMP_SENSOR_CHAMBER3 == 20)

#define HAS_DHT     ENABLED(DHT_SENSOR)

/**
 * Shorthand for pin tests, used wherever needed
 */

// Steppers
#define HAS_X_ENABLE        (PIN_EXISTS(X_ENABLE) || (AXIS_HAS_TMC(X) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_X_DIR           (PIN_EXISTS(X_DIR))
#define HAS_X_STEP          (PIN_EXISTS(X_STEP))
#define HAS_X_MICROSTEPS    (PIN_EXISTS(X_MS1))

#define HAS_X2_ENABLE       (PIN_EXISTS(X2_ENABLE) || (AXIS_HAS_TMC(X2) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_X2_DIR          (PIN_EXISTS(X2_DIR))
#define HAS_X2_STEP         (PIN_EXISTS(X2_STEP))

#define HAS_Y_ENABLE        (PIN_EXISTS(Y_ENABLE) || (AXIS_HAS_TMC(Y) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_Y_DIR           (PIN_EXISTS(Y_DIR))
#define HAS_Y_STEP          (PIN_EXISTS(Y_STEP))
#define HAS_Y_MICROSTEPS    (PIN_EXISTS(Y_MS1))

#define HAS_Y2_ENABLE       (PIN_EXISTS(Y2_ENABLE) || (AXIS_HAS_TMC(Y2) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_Y2_DIR          (PIN_EXISTS(Y2_DIR))
#define HAS_Y2_STEP         (PIN_EXISTS(Y2_STEP))

#define HAS_Z_ENABLE        (PIN_EXISTS(Z_ENABLE) || (AXIS_HAS_TMC(Z) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_Z_DIR           (PIN_EXISTS(Z_DIR))
#define HAS_Z_STEP          (PIN_EXISTS(Z_STEP))
#define HAS_Z_MICROSTEPS    (PIN_EXISTS(Z_MS1))

#define HAS_Z2_ENABLE       (PIN_EXISTS(Z2_ENABLE) || (AXIS_HAS_TMC(Z2) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_Z2_DIR          (PIN_EXISTS(Z2_DIR))
#define HAS_Z2_STEP         (PIN_EXISTS(Z2_STEP))

#define HAS_Z3_ENABLE       (PIN_EXISTS(Z3_ENABLE) || (AXIS_HAS_TMC(Z3) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_Z3_DIR          (PIN_EXISTS(Z3_DIR))
#define HAS_Z3_STEP         (PIN_EXISTS(Z3_STEP))

// Extruder steppers and solenoids
#define HAS_E0_ENABLE       (PIN_EXISTS(E0_ENABLE) || (AXIS_HAS_TMC(E0) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_E0_DIR          (PIN_EXISTS(E0_DIR))
#define HAS_E0_STEP         (PIN_EXISTS(E0_STEP))
#define HAS_E0_MICROSTEPS   (PIN_EXISTS(E0_MS1))
#define HAS_SOLENOID_0      (PIN_EXISTS(SOL0))

#define HAS_E1_ENABLE       (PIN_EXISTS(E1_ENABLE) || (AXIS_HAS_TMC(E1) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_E1_DIR          (PIN_EXISTS(E1_DIR))
#define HAS_E1_STEP         (PIN_EXISTS(E1_STEP))
#define HAS_E1_MICROSTEPS   (PIN_EXISTS(E1_MS1))
#define HAS_SOLENOID_1      (PIN_EXISTS(SOL1))

#define HAS_E2_ENABLE       (PIN_EXISTS(E2_ENABLE) || (AXIS_HAS_TMC(E2) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_E2_DIR          (PIN_EXISTS(E2_DIR))
#define HAS_E2_STEP         (PIN_EXISTS(E2_STEP))
#define HAS_E2_MICROSTEPS   (PIN_EXISTS(E2_MS1))
#define HAS_SOLENOID_2      (PIN_EXISTS(SOL2))

#define HAS_E3_ENABLE       (PIN_EXISTS(E3_ENABLE) || (AXIS_HAS_TMC(E3) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_E3_DIR          (PIN_EXISTS(E3_DIR))
#define HAS_E3_STEP         (PIN_EXISTS(E3_STEP))
#define HAS_E3_MICROSTEPS   (PIN_EXISTS(E3_MS1))
#define HAS_SOLENOID_3      (PIN_EXISTS(SOL3))

#define HAS_E4_ENABLE       (PIN_EXISTS(E4_ENABLE) || (AXIS_HAS_TMC(E4) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_E4_DIR          (PIN_EXISTS(E4_DIR))
#define HAS_E4_STEP         (PIN_EXISTS(E4_STEP))
#define HAS_E4_MICROSTEPS   (PIN_EXISTS(E4_MS1))
#define HAS_SOLENOID_4      (PIN_EXISTS(SOL4))

#define HAS_E5_ENABLE       (PIN_EXISTS(E5_ENABLE) || (AXIS_HAS_TMC(E5) && ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)))
#define HAS_E5_DIR          (PIN_EXISTS(E5_DIR))
#define HAS_E5_STEP         (PIN_EXISTS(E5_STEP))
#define HAS_E5_MICROSTEPS   (PIN_EXISTS(E5_MS1))
#define HAS_SOLENOID_5      (PIN_EXISTS(SOL5))

#define HAS_E_STEPPER_ENABLE  ( HAVE_E_DRV(TMC2660)                     \
  || ( E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != X_ENABLE_PIN   \
    && E0_ENABLE_PIN != Y_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN ) )

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
#define HAS_Z3_MIN          (PIN_EXISTS(Z3_MIN))
#define HAS_X2_MAX          (PIN_EXISTS(X2_MAX))
#define HAS_Y2_MAX          (PIN_EXISTS(Y2_MAX))
#define HAS_Z2_MAX          (PIN_EXISTS(Z2_MAX))
#define HAS_Z3_MAX          (PIN_EXISTS(Z3_MAX))
#define HAS_Z_PROBE_PIN     (PIN_EXISTS(Z_PROBE))

// Multi endstop
#define HAS_MULTI_ENDSTOP   (ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS) || ENABLED(Z_THREE_ENDSTOPS))

// Utility
#define HAS_DOOR_OPEN       (ENABLED(DOOR_OPEN_FEATURE) && PIN_EXISTS(DOOR_OPEN))
#define HAS_POWER_CHECK     (ENABLED(POWER_CHECK) && PIN_EXISTS(POWER_CHECK))

// Thermistors
#define HAS_TEMP_HE0        (TEMP_SENSOR_HE0 != 0)
#define HAS_TEMP_HE1        (TEMP_SENSOR_HE1 != 0)
#define HAS_TEMP_HE2        (TEMP_SENSOR_HE2 != 0)
#define HAS_TEMP_HE3        (TEMP_SENSOR_HE3 != 0)
#define HAS_TEMP_HE4        (TEMP_SENSOR_HE4 != 0)
#define HAS_TEMP_HE5        (TEMP_SENSOR_HE5 != 0)
#define HAS_TEMP_HOTEND     (HAS_TEMP_HE0 || HAS_MAX6675 || ENABLED(SUPPORT_MAX31855))
#define HAS_TEMP_BED0       (TEMP_SENSOR_BED0 != 0)
#define HAS_TEMP_BED1       (TEMP_SENSOR_BED1 != 0)
#define HAS_TEMP_BED2       (TEMP_SENSOR_BED2 != 0)
#define HAS_TEMP_BED3       (TEMP_SENSOR_BED3 != 0)
#define HAS_TEMP_CHAMBER0   (TEMP_SENSOR_CHAMBER0 != 0)
#define HAS_TEMP_CHAMBER1   (TEMP_SENSOR_CHAMBER1 != 0)
#define HAS_TEMP_CHAMBER2   (TEMP_SENSOR_CHAMBER2 != 0)
#define HAS_TEMP_CHAMBER3   (TEMP_SENSOR_CHAMBER3 != 0)
#define HAS_TEMP_COOLER     (TEMP_SENSOR_COOLER != 0)
#define HAS_MCU_TEMPERATURE (ENABLED(HAVE_MCU_TEMPERATURE))
#define HAS_VREF_MONITOR    (ENABLED(HAVE_VREF_MONITOR))

// Thermocouples
#define HAS_MAX6675_SS      (PIN_EXISTS(MAX6675_SS))
#define HAS_MAX31855_SS0    (PIN_EXISTS(MAX31855_SS0))
#define HAS_MAX31855_SS1    (PIN_EXISTS(MAX31855_SS1))
#define HAS_MAX31855_SS2    (PIN_EXISTS(MAX31855_SS2))
#define HAS_MAX31855_SS3    (PIN_EXISTS(MAX31855_SS3))

// Heaters
#define HAS_HEATER_HE0      (HOTENDS > 0 && PIN_EXISTS(HEATER_HE0))
#define HAS_HEATER_HE1      (HOTENDS > 1 && PIN_EXISTS(HEATER_HE1))
#define HAS_HEATER_HE2      (HOTENDS > 2 && PIN_EXISTS(HEATER_HE2))
#define HAS_HEATER_HE3      (HOTENDS > 3 && PIN_EXISTS(HEATER_HE3))
#define HAS_HEATER_HE4      (HOTENDS > 4 && PIN_EXISTS(HEATER_HE4))
#define HAS_HEATER_HE5      (HOTENDS > 5 && PIN_EXISTS(HEATER_HE5))
#define HAS_HEATER_BED0     (TEMP_SENSOR_BED0 != 0 && PIN_EXISTS(HEATER_BED0))
#define HAS_HEATER_BED1     (TEMP_SENSOR_BED1 != 0 && PIN_EXISTS(HEATER_BED1))
#define HAS_HEATER_BED2     (TEMP_SENSOR_BED1 != 0 && PIN_EXISTS(HEATER_BED1))
#define HAS_HEATER_BED3     (TEMP_SENSOR_BED2 != 0 && PIN_EXISTS(HEATER_BED2))
#define HAS_HEATER_CHAMBER0 (TEMP_SENSOR_CHAMBER0 != 0 && PIN_EXISTS(HEATER_CHAMBER0))
#define HAS_HEATER_CHAMBER1 (TEMP_SENSOR_CHAMBER1 != 0 && PIN_EXISTS(HEATER_CHAMBER1))
#define HAS_HEATER_CHAMBER2 (TEMP_SENSOR_CHAMBER2 != 0 && PIN_EXISTS(HEATER_CHAMBER2))
#define HAS_HEATER_CHAMBER3 (TEMP_SENSOR_CHAMBER3 != 0 && PIN_EXISTS(HEATER_CHAMBER3))
#define HAS_HEATER_COOLER   (TEMP_SENSOR_COOLER != 0 && PIN_EXISTS(HEATER_COOLER))

// Fans
#define HAS_FAN0            (PIN_EXISTS(FAN0))
#define HAS_FAN1            (PIN_EXISTS(FAN1))
#define HAS_FAN2            (PIN_EXISTS(FAN2))
#define HAS_FAN3            (PIN_EXISTS(FAN3))
#define HAS_FAN4            (PIN_EXISTS(FAN4))
#define HAS_FAN5            (PIN_EXISTS(FAN5))

// Servos
#define HAS_SERVO_0         (PIN_EXISTS(SERVO0) && NUM_SERVOS > 0)
#define HAS_SERVO_1         (PIN_EXISTS(SERVO1) && NUM_SERVOS > 1)
#define HAS_SERVO_2         (PIN_EXISTS(SERVO2) && NUM_SERVOS > 2)
#define HAS_SERVO_3         (PIN_EXISTS(SERVO3) && NUM_SERVOS > 3)
#define HAS_SERVOS          (ENABLED(ENABLE_SERVOS) && (HAS_SERVO_0 || HAS_SERVO_1 || HAS_SERVO_2 || HAS_SERVO_3))

// Sensors
#define HAS_FILAMENT_SENSOR           ENABLED(FILAMENT_RUNOUT_SENSOR)
#define HAS_FIL_RUNOUT_0              (HAS_FILAMENT_SENSOR && PIN_EXISTS(FIL_RUNOUT_0))
#define HAS_FIL_RUNOUT_1              (HAS_FILAMENT_SENSOR && PIN_EXISTS(FIL_RUNOUT_1))
#define HAS_FIL_RUNOUT_2              (HAS_FILAMENT_SENSOR && PIN_EXISTS(FIL_RUNOUT_2))
#define HAS_FIL_RUNOUT_3              (HAS_FILAMENT_SENSOR && PIN_EXISTS(FIL_RUNOUT_3))
#define HAS_FIL_RUNOUT_4              (HAS_FILAMENT_SENSOR && PIN_EXISTS(FIL_RUNOUT_4))
#define HAS_FIL_RUNOUT_5              (HAS_FILAMENT_SENSOR && PIN_EXISTS(FIL_RUNOUT_5))
#define HAS_DAV_SYSTEM                (ENABLED(FILAMENT_RUNOUT_DAV_SYSTEM) && PIN_EXISTS(FIL_RUNOUT_DAV))
#define HAS_LCD_FILAMENT_SENSOR       (ENABLED(FILAMENT_WIDTH_SENSOR) && ENABLED(FILAMENT_LCD_DISPLAY))
#define HAS_POWER_CONSUMPTION_SENSOR  (ENABLED(POWER_CONSUMPTION) && PIN_EXISTS(POWER_CONSUMPTION))
#define HAS_LCD_POWER_SENSOR          (HAS_POWER_CONSUMPTION_SENSOR && ENABLED(POWER_CONSUMPTION_LCD_DISPLAY))

// Service time 
#define HAS_SERVICE_TIMES             (ENABLED(SERVICE_TIME_1) || ENABLED(SERVICE_TIME_2) || ENABLED(SERVICE_TIME_3))

// User Interface
#define HAS_BTN_BACK        (PIN_EXISTS(BTN_BACK))
#define HAS_HOME            (PIN_EXISTS(HOME))
#define HAS_KILL            (PIN_EXISTS(KILL))
#define HAS_SUICIDE         (PIN_EXISTS(SUICIDE))
#define HAS_CHDK            (ENABLED(PHOTO_GCODE) && PIN_EXISTS(CHDK))
#define HAS_PHOTOGRAPH      (ENABLED(PHOTO_GCODE) && PIN_EXISTS(PHOTOGRAPH))
#define HAS_BUZZER          (PIN_EXISTS(BEEPER) || ENABLED(LCD_USE_I2C_BUZZER))
#define HAS_CASE_LIGHT      (ENABLED(CASE_LIGHT) && (PIN_EXISTS(CASE_LIGHT) || ENABLED(CASE_LIGHT_USE_NEOPIXEL)))
#define HAS_RESUME_CONTINUE (HAS_LCD || ENABLED(EMERGENCY_PARSER))

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
#define HAS_LASER_SPINDLE   (ENABLED(LASER) && ENABLED(LASER_FIRE_SPINDLE))
#define HAS_LASER_FIRE_G1   (ENABLED(LASER) && ENABLED(LASER_FIRE_G1))
#define HAS_LASER_FIRE_E    (ENABLED(LASER) && ENABLED(LASER_FIRE_E))

// CNC
#define HAS_CNCROUTER       (PIN_EXISTS(CNCROUTER))

// Multi Mode
#define HAS_MULTI_MODE      (ENABLED(LASER) || ENABLED(CNCROUTER) || ENABLED(MILLING) || ENABLED(PICK_AND_PLACE) || ENABLED(SOLDER) || ENABLED(PLOTTER))

// MK Multi tool system
#define HAS_MKMULTI_TOOLS   (ENABLED(MKSE6) || ENABLED(MKR4) || ENABLED(MKR6) || ENABLED(MKR12))

// MKR4 or MKR6 or MKR12
#define HAS_E0E1            (PIN_EXISTS(E0E1_CHOICE))
#define HAS_E0E2            (PIN_EXISTS(E0E2_CHOICE))
#define HAS_E1E3            (PIN_EXISTS(E1E3_CHOICE))
#define HAS_EX1             (PIN_EXISTS(EX1_CHOICE))
#define HAS_EX2             (PIN_EXISTS(EX2_CHOICE))

// Color Mixing_Extruder
#define HAS_COLOR_MIXING    (ENABLED(COLOR_MIXING_EXTRUDER))

// Prusa MMU2
#define HAS_MMU2            (ENABLED(PRUSA_MMU2))

// Dondolo
#define HAS_DONDOLO         (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR))

// Linear extruder system
#define HAS_LINEAR_EXTRUDER (!HAS_MKMULTI_TOOLS && !HAS_COLOR_MIXING && !HAS_MMU2 && !HAS_DONDOLO)

// RGB leds
#define HAS_COLOR_LEDS      (ENABLED(BLINKM) || ENABLED(RGB_LED) || ENABLED(RGBW_LED) || ENABLED(PCA9632) || ENABLED(NEOPIXEL_LED))
#define HAS_LEDS_OFF_FLAG   (ENABLED(PRINTER_EVENT_LEDS) && HAS_SD_SUPPORT && HAS_RESUME_CONTINUE)

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
#define HAS_EEPROM          (ENABLED(EEPROM_SETTINGS))  // Do not touch, AVR have not define anyone EEPROM.

// GAME MENU
#define HAS_GAMES           (ENABLED(GAME_BRICKOUT) || ENABLED(GAME_INVADERS) || ENABLED(GAME_SNAKE) || ENABLED(GAME_MAZE))
#define HAS_GAME_MENU       (1 < ENABLED(GAME_BRICKOUT) + ENABLED(GAME_INVADERS) + ENABLED(GAME_SNAKE) + ENABLED(GAME_MAZE))

// SD support
#define HAS_SD_SUPPORT      (ENABLED(SDSUPPORT) || ENABLED(USB_FLASH_DRIVE_SUPPORT))
#if HAS_SD_SUPPORT
  #if ENABLED(__AVR__)
    #define MAX_VFAT_ENTRIES (2)
    #define SD_MAX_FOLDER_DEPTH 2   // Maximum folder depth
  #else
    #define MAX_VFAT_ENTRIES (3)
    #define SD_MAX_FOLDER_DEPTH 5   // Maximum folder depth
  #endif
  #define FILENAME_LENGTH 13
  /** Total size of the buffer used to store the long filenames */
  #define LONG_FILENAME_LENGTH  (FILENAME_LENGTH * MAX_VFAT_ENTRIES + 1)
  #define MAX_PATH_NAME_LENGHT  (LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1)
  #define SHORT_FILENAME_LENGTH 14
  #define GENBY_SIZE 16
#else
  #undef SCROLL_LONG_FILENAMES
  #undef SDCARD_SORT_ALPHA
#endif
#if HAS_SD_SUPPORT && ENABLED(ARDUINO_ARCH_SAM)
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
  #define HAS_FOLDER_SORTING  (FOLDER_SORTING || ENABLED(SDSORT_GCODE))
#endif
#define HAS_SD_RESTART        (HAS_SD_SUPPORT && ENABLED(SD_RESTART_FILE))

// Other
#define HAS_Z_PROBE_SLED      (ENABLED(PROBE_SLED) && PIN_EXISTS(SLED))
#define HAS_SOFTWARE_ENDSTOPS (ENABLED(MIN_SOFTWARE_ENDSTOPS) || ENABLED(MAX_SOFTWARE_ENDSTOPS))

/**
 * PWM HEATER FREQUENCY
 */
#if DISABLED(HOTEND_PWM_FREQUENCY)
  #define HOTEND_PWM_FREQUENCY 1000
#endif
#if HOTEND_PWM_FREQUENCY > 1000
  #undef HOTEND_PWM_FREQUENCY
  #define HOTEND_PWM_FREQUENCY 1000
#endif
#if DISABLED(BED_PWM_FREQUENCY)
  #define BED_PWM_FREQUENCY 10
#endif
#if BED_PWM_FREQUENCY > 1000
  #undef BED_PWM_FREQUENCY
  #define BED_PWM_FREQUENCY 1000
#endif
#if DISABLED(CHAMBER_PWM_FREQUENCY)
  #define CHAMBER_PWM_FREQUENCY 10
#endif
#if CHAMBER_PWM_FREQUENCY > 1000
  #undef CHAMBER_PWM_FREQUENCY
  #define CHAMBER_PWM_FREQUENCY 1000
#endif
#if DISABLED(COOLER_PWM_FREQUENCY)
  #define COOLER_PWM_FREQUENCY 10
#endif
#if COOLER_PWM_FREQUENCY > 1000
  #undef COOLER_PWM_FREQUENCY
  #define COOLER_PWM_FREQUENCY 1000
#endif

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
#if DISABLED(ENDSTOPPULLUP_Z3MIN)
  #define ENDSTOPPULLUP_Z3MIN   false
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
#if DISABLED(ENDSTOPPULLUP_Z3MAX)
  #define ENDSTOPPULLUP_Z3MAX   false
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
#if DISABLED(Z3_MIN_ENDSTOP_LOGIC)
  #define Z3_MIN_ENDSTOP_LOGIC  false
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
#if DISABLED(Z3_MAX_ENDSTOP_LOGIC)
  #define Z3_MAX_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Z_PROBE_ENDSTOP_LOGIC)
  #define Z_PROBE_ENDSTOP_LOGIC false
#endif

/**
 * JERK or JUNCTION_DEVIATION
 */
#define HAS_CLASSIC_JERK        (IS_KINEMATIC || DISABLED(JUNCTION_DEVIATION))
#define HAS_CLASSIC_E_JERK      (DISABLED(LIN_ADVANCE) || DISABLED(JUNCTION_DEVIATION))
#define HAS_LINEAR_E_JERK       (ENABLED(LIN_ADVANCE) && ENABLED(JUNCTION_DEVIATION))
#define HAS_DIST_MM_ARG         (IS_KINEMATIC && ENABLED(JUNCTION_DEVIATION))

/**
 * Set granular options based on the specific type of leveling
 */
#define UBL_DELTA               (ENABLED(AUTO_BED_LEVELING_UBL) && MECH(DELTA))
#define ABL_PLANAR              (ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_3POINT))
#define ABL_GRID                (ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_BILINEAR))
#define OLD_ABL                 (ABL_PLANAR || ABL_GRID)
#define HAS_ABL_OR_UBL          (OLD_ABL || ENABLED(AUTO_BED_LEVELING_UBL))
#define HAS_LEVELING            (HAS_ABL_OR_UBL || ENABLED(MESH_BED_LEVELING))
#define HAS_AUTOLEVEL           (HAS_ABL_OR_UBL && DISABLED(PROBE_MANUALLY))
#define HAS_MESH                (ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(MESH_BED_LEVELING))
#define PLANNER_LEVELING        (HAS_LEVELING && DISABLED(AUTO_BED_LEVELING_UBL))
#define HAS_PROBING_PROCEDURE   (HAS_ABL_OR_UBL || ENABLED(PROBE_REPEATABILITY_TEST))
#define HAS_POSITION_MODIFIERS  (ENABLED(FWRETRACT) || HAS_LEVELING)

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #undef LCD_BED_LEVELING
#endif

/**
 * Position Float
 */
#define HAS_POSITION_FLOAT      (ENABLED(LIN_ADVANCE) || ENABLED(SCARA_FEEDRATE_SCALING) || HAS_GRADIENT_MIX)

/**
 * Bed Probing rectangular bounds
 * These can be further constrained in code for Delta and SCARA
 */
#if MECH(DELTA)
  #define MIN_PROBE_X -(mechanics.data.probe_radius)
  #define MAX_PROBE_X  (mechanics.data.probe_radius)
  #define MIN_PROBE_Y -(mechanics.data.probe_radius)
  #define MAX_PROBE_Y  (mechanics.data.probe_radius)
#elif IS_SCARA
  #define SCARA_PRINTABLE_RADIUS (SCARA_LINKAGE_1 + SCARA_LINKAGE_2)
  #define MIN_PROBE_X (X_CENTER - (SCARA_PRINTABLE_RADIUS) + (MIN_PROBE_EDGE))
  #define MAX_PROBE_X (Y_CENTER - (SCARA_PRINTABLE_RADIUS) + (MIN_PROBE_EDGE))
  #define MIN_PROBE_Y (X_CENTER + (SCARA_PRINTABLE_RADIUS) - (MIN_PROBE_EDGE))
  #define MAX_PROBE_Y (Y_CENTER + (SCARA_PRINTABLE_RADIUS) - (MIN_PROBE_EDGE))
#else
  // Boundaries for Cartesian probing based on bed limits
  #define MIN_PROBE_X (MAX(mechanics.data.base_pos.min.x + (MIN_PROBE_EDGE), mechanics.data.base_pos.min.x + probe.data.offset.x))
  #define MIN_PROBE_Y (MAX(mechanics.data.base_pos.min.y + (MIN_PROBE_EDGE), mechanics.data.base_pos.min.y + probe.data.offset.y))
  #define MAX_PROBE_X (MIN(mechanics.data.base_pos.max.x - (MIN_PROBE_EDGE), mechanics.data.base_pos.max.x + probe.data.offset.x))
  #define MAX_PROBE_Y (MIN(mechanics.data.base_pos.max.y - (MIN_PROBE_EDGE), mechanics.data.base_pos.max.y + probe.data.offset.y))
#endif

/**
 * Set GRID MAX POINTS
 */
#if ENABLED(GRID_MAX_POINTS_X) && ENABLED(GRID_MAX_POINTS_Y)
  #define GRID_MAX_POINTS   ((GRID_MAX_POINTS_X) * (GRID_MAX_POINTS_Y))
  #define GRID_LOOP(A,B)    LOOP_L_N(A, GRID_MAX_POINTS_X) LOOP_L_N(B, GRID_MAX_POINTS_Y)
#endif

/**
 * Z probe repetitions
 */
#if DISABLED(Z_PROBE_REPETITIONS)
  #define Z_PROBE_REPETITIONS 1
#endif

/**
 * Heaters Beds
 */
#if HAS_HEATER_BED3
  #define BEDS  4
#elif HAS_HEATER_BED2
  #define BEDS  3
#elif HAS_HEATER_BED1
  #define BEDS  2
#elif HAS_HEATER_BED0
  #define BEDS  1
#else
  #define BEDS  0
#endif

/**
 * Heaters Chambers
 */
#if HAS_HEATER_CHAMBER3
  #define CHAMBERS  4
#elif HAS_HEATER_CHAMBER2
  #define CHAMBERS  3
#elif HAS_HEATER_CHAMBER1
  #define CHAMBERS  2
#elif HAS_HEATER_CHAMBER0
  #define CHAMBERS  1
#else
  #define CHAMBERS  0
#endif

/**
 * Heaters Cooler
 */
#if HAS_HEATER_COOLER
  #define COOLERS  1
#else
  #define COOLERS  0
#endif

#define HEATER_COUNT  (HOTENDS+BEDS+CHAMBERS+COOLERS)
#define HAS_HEATER    (HEATER_COUNT > 0)

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
#define FANS_CHANNELS   { FAN0_CHANNEL FAN1_CHANNEL FAN2_CHANNEL FAN3_CHANNEL FAN4_CHANNEL FAN5_CHANNEL }

#if ENABLED(INVERTED_FAN_PINS)
  #define FAN_INVERTED true
#else
  #define FAN_INVERTED false
#endif

/**
 * Heater & Fan Pausing
 */
#if FAN_COUNT == 0
  #undef PROBING_FANS_OFF
#endif
#define QUIET_PROBING       (HAS_BED_PROBE && (ENABLED(PROBING_HEATERS_OFF) || ENABLED(PROBING_FANS_OFF)))

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
 * Make sure DOGLCD_SCK and DOGLCD_MOSI are defined.
 */
#if ENABLED(DOGLCD)
  #if DISABLED(DOGLCD_SCK)
    #define DOGLCD_SCK  SCK_PIN
  #endif
  #if DISABLED(DOGLCD_MOSI)
    #define DOGLCD_MOSI MOSI_PIN
  #endif
#endif

/**
 * MIN_Z_HEIGHT_FOR_HOMING / Z_PROBE_BETWEEN_HEIGHT
 */
#if DISABLED(MIN_Z_HEIGHT_FOR_HOMING)
  #define MIN_Z_HEIGHT_FOR_HOMING 0
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
  #if DISABLED(PROBE_SERVO_NR)
    #define PROBE_SERVO_NR -1
  #endif
#endif

/**
 * Set a flag for a servo probe
 */
#define HAS_Z_SERVO_PROBE     (HAS_SERVOS && ENABLED(PROBE_SERVO_NR) && PROBE_SERVO_NR >= 0)

/**
 * Set a flag for Probe Manually
 */
#define HAS_PROBE_MANUALLY    (ENABLED(PROBE_MANUALLY))

/**
 * Set a flag for Probe Fix
 */
#define HAS_PROBE_FIX         (ENABLED(PROBE_FIX_MOUNTED))

/**
 * Set a flag for BLTouch
 */
#define HAS_BLTOUCH           (ENABLED(BLTOUCH))

/**
 * Set a flag for Probe Sled
 */
#define HAS_SLED              (ENABLED(PROBE_SLED))

/**
 * Set a flag for Allen Key
 */
#define HAS_ALLEN_KEY         (ENABLED(PROBE_ALLEN_KEY))

/**
 * Set flags for enabled probes
 */
#define HAS_BED_PROBE         (HAS_PROBE_FIX || HAS_SLED || ENABLED(PROBE_ALLEN_KEY) || HAS_Z_SERVO_PROBE || ENABLED(PROBE_SENSORLESS))
#define PROBE_SELECTED        (HAS_BED_PROBE || HAS_PROBE_MANUALLY)
#define PROBE_PIN_CONFIGURED  (HAS_Z_PROBE_PIN || HAS_Z_MIN)

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
  #if DISABLED(Z_PROBE_AFTER_PROBING)
    #define Z_PROBE_AFTER_PROBING 0
  #endif
  #if DISABLED(Z_PROBE_LOW_POINT)
    #define Z_PROBE_LOW_POINT -2
  #endif
#else
  #undef X_PROBE_OFFSET_FROM_NOZZLE
  #undef Y_PROBE_OFFSET_FROM_NOZZLE
  #undef Z_PROBE_OFFSET_FROM_NOZZLE
  #define X_PROBE_OFFSET_FROM_NOZZLE 0
  #define Y_PROBE_OFFSET_FROM_NOZZLE 0
  #define Z_PROBE_OFFSET_FROM_NOZZLE 0
  #define _Z_PROBE_DEPLOY_HEIGHT (Z_MAX_BED / 2)
#endif

// Add commands that need sub-codes to this list
#define USE_GCODE_SUBCODES (ENABLED(G38_PROBE_TARGET) || HAS_SD_RESTART)

// HAS RESTART and MIN_Z_HEIGHT_FOR_HOMING
#if HAS_SD_RESTART && ENABLED(MIN_Z_HEIGHT_FOR_HOMING)
  #undef MIN_Z_HEIGHT_FOR_HOMING
  #define MIN_Z_HEIGHT_FOR_HOMING 0
#endif

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
#if DISABLED(SOFT_PWM_SPEED)
  #define SOFT_PWM_SPEED 0
#endif
#if SOFT_PWM_SPEED < 0
  #define SOFT_PWM_SPEED 0
#endif
#if SOFT_PWM_SPEED > 5
  #define SOFT_PWM_SPEED 5
#endif

#if SOFT_PWM_SPEED == 0
  #define SOFT_PWM_STEP 1
  #define SOFT_PWM_MASK 255
#elif SOFT_PWM_SPEED == 1
  #define SOFT_PWM_STEP 2
  #define SOFT_PWM_MASK 254
#elif SOFT_PWM_SPEED == 2
  #define SOFT_PWM_STEP 4
  #define SOFT_PWM_MASK 252
#elif SOFT_PWM_SPEED == 3
  #define SOFT_PWM_STEP 8
  #define SOFT_PWM_MASK 248
#elif SOFT_PWM_SPEED == 4
  #define SOFT_PWM_STEP 16
  #define SOFT_PWM_MASK 240
#elif SOFT_PWM_SPEED == 5
  #define SOFT_PWM_STEP 8
  #define SOFT_PWM_MASK 224
#endif

// TEMPERATURE
#if (HOTENDS > 0 && (PIN_EXISTS(TEMP_HE0) && TEMP_SENSOR_HE0 != 0 && TEMP_SENSOR_HE0 >= -2))
  #define HOT0_ANALOG_INPUTS        1
  #define HOT0_ANALOG_CHANNEL       TEMP_HE0_PIN
  #define HOT0_ANALOG_COMMA         ,
#else
  #define HOT0_ANALOG_INPUTS        0
  #define HOT0_ANALOG_CHANNEL
  #define HOT0_ANALOG_COMMA
#endif

#if (HOTENDS > 1 && (PIN_EXISTS(TEMP_HE1) && TEMP_SENSOR_HE1 != 0 && TEMP_SENSOR_HE1 >= -2))
  #define HOT1_ANALOG_INPUTS        1
  #define HOT1_ANALOG_CHANNEL       HOT0_ANALOG_COMMA TEMP_HE1_PIN
  #define HOT1_ANALOG_COMMA         ,
#else
  #define HOT1_ANALOG_INPUTS        0
  #define HOT1_ANALOG_CHANNEL
  #define HOT1_ANALOG_COMMA         HOT0_ANALOG_COMMA
#endif

#if (HOTENDS > 2 && (PIN_EXISTS(TEMP_HE2) && TEMP_SENSOR_HE2 != 0 && TEMP_SENSOR_HE2 >= -2))
  #define HOT2_ANALOG_INPUTS        1
  #define HOT2_ANALOG_CHANNEL       HOT1_ANALOG_COMMA TEMP_HE2_PIN
  #define HOT2_ANALOG_COMMA         ,
#else
  #define HOT2_ANALOG_INPUTS        0
  #define HOT2_ANALOG_CHANNEL
  #define HOT2_ANALOG_COMMA         HOT1_ANALOG_COMMA
#endif

#if (HOTENDS > 3 && (PIN_EXISTS(TEMP_HE3) && TEMP_SENSOR_HE3 != 0 && TEMP_SENSOR_HE3 >= -2))
  #define HOT3_ANALOG_INPUTS        1
  #define HOT3_ANALOG_CHANNEL       HOT2_ANALOG_COMMA TEMP_HE3_PIN
  #define HOT3_ANALOG_COMMA         ,
#else
  #define HOT3_ANALOG_INPUTS        0
  #define HOT3_ANALOG_CHANNEL
  #define HOT3_ANALOG_COMMA         HOT2_ANALOG_COMMA
#endif

#if (HOTENDS > 4 && (PIN_EXISTS(TEMP_HE4) && TEMP_SENSOR_HE4 != 0 && TEMP_SENSOR_HE4 >= -2))
  #define HOT4_ANALOG_INPUTS        1
  #define HOT4_ANALOG_CHANNEL       HOT3_ANALOG_COMMA TEMP_HE4_PIN
  #define HOT4_ANALOG_COMMA         ,
#else
  #define HOT4_ANALOG_INPUTS        0
  #define HOT4_ANALOG_CHANNEL
  #define HOT4_ANALOG_COMMA         HOT3_ANALOG_COMMA
#endif

#if (HOTENDS > 5 && (PIN_EXISTS(TEMP_HE5) && TEMP_SENSOR_HE5 != 0 && TEMP_SENSOR_HE5 >= -2))
  #define HOT5_ANALOG_INPUTS        1
  #define HOT5_ANALOG_CHANNEL       HOT4_ANALOG_COMMA TEMP_HE5_PIN
  #define HOT5_ANALOG_COMMA         ,
#else
  #define HOT5_ANALOG_INPUTS        0
  #define HOT5_ANALOG_CHANNEL
  #define HOT5_ANALOG_COMMA         HOT4_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_BED0) && TEMP_SENSOR_BED0 != 0 && TEMP_SENSOR_BED0 >= -2)
  #define BED0_ANALOG_INPUTS        1
  #define BED0_ANALOG_CHANNEL       HOT5_ANALOG_COMMA TEMP_BED0_PIN
  #define BED0_ANALOG_COMMA         ,
#else
  #define BED0_ANALOG_INPUTS        0
  #define BED0_ANALOG_CHANNEL
  #define BED0_ANALOG_COMMA         HOT5_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_BED1) && TEMP_SENSOR_BED1 != 0 && TEMP_SENSOR_BED1 >= -2)
  #define BED1_ANALOG_INPUTS        1
  #define BED1_ANALOG_CHANNEL       BED0_ANALOG_COMMA TEMP_BED1_PIN
  #define BED1_ANALOG_COMMA         ,
#else
  #define BED1_ANALOG_INPUTS        0
  #define BED1_ANALOG_CHANNEL
  #define BED1_ANALOG_COMMA         BED0_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_BED2) && TEMP_SENSOR_BED2 != 0 && TEMP_SENSOR_BED2 >= -2)
  #define BED2_ANALOG_INPUTS        1
  #define BED2_ANALOG_CHANNEL       BED1_ANALOG_COMMA TEMP_BED2_PIN
  #define BED2_ANALOG_COMMA         ,
#else
  #define BED2_ANALOG_INPUTS        0
  #define BED2_ANALOG_CHANNEL
  #define BED2_ANALOG_COMMA         BED1_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_BED3) && TEMP_SENSOR_BED3 != 0 && TEMP_SENSOR_BED3 >= -2)
  #define BED3_ANALOG_INPUTS        1
  #define BED3_ANALOG_CHANNEL       BED2_ANALOG_COMMA TEMP_BED3_PIN
  #define BED3_ANALOG_COMMA         ,
#else
  #define BED3_ANALOG_INPUTS        0
  #define BED3_ANALOG_CHANNEL
  #define BED3_ANALOG_COMMA         BED2_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_CHAMBER0) && TEMP_SENSOR_CHAMBER0 != 0 && TEMP_SENSOR_CHAMBER0 >= -2)
  #define CHAMBER0_ANALOG_INPUTS    1
  #define CHAMBER0_ANALOG_CHANNEL   BED3_ANALOG_COMMA TEMP_CHAMBER0_PIN
  #define CHAMBER0_ANALOG_COMMA     ,
#else
  #define CHAMBER0_ANALOG_INPUTS    0
  #define CHAMBER0_ANALOG_CHANNEL
  #define CHAMBER0_ANALOG_COMMA     BED3_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_CHAMBER1) && TEMP_SENSOR_CHAMBER1 != 0 && TEMP_SENSOR_CHAMBER1 >= -2)
  #define CHAMBER1_ANALOG_INPUTS    1
  #define CHAMBER1_ANALOG_CHANNEL   CHAMBER0_ANALOG_COMMA TEMP_CHAMBER1_PIN
  #define CHAMBER1_ANALOG_COMMA     ,
#else
  #define CHAMBER1_ANALOG_INPUTS    0
  #define CHAMBER1_ANALOG_CHANNEL
  #define CHAMBER1_ANALOG_COMMA     CHAMBER0_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_CHAMBER2) && TEMP_SENSOR_CHAMBER2 != 0 && TEMP_SENSOR_CHAMBER2 >= -2)
  #define CHAMBER2_ANALOG_INPUTS    1
  #define CHAMBER2_ANALOG_CHANNEL   CHAMBER1_ANALOG_COMMA TEMP_CHAMBER2_PIN
  #define CHAMBER2_ANALOG_COMMA     ,
#else
  #define CHAMBER2_ANALOG_INPUTS    0
  #define CHAMBER2_ANALOG_CHANNEL
  #define CHAMBER2_ANALOG_COMMA     CHAMBER1_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_CHAMBER3) && TEMP_SENSOR_CHAMBER3 != 0 && TEMP_SENSOR_CHAMBER3 >= -2)
  #define CHAMBER3_ANALOG_INPUTS    1
  #define CHAMBER3_ANALOG_CHANNEL   CHAMBER2_ANALOG_COMMA TEMP_CHAMBER3_PIN
  #define CHAMBER3_ANALOG_COMMA     ,
#else
  #define CHAMBER3_ANALOG_INPUTS    0
  #define CHAMBER3_ANALOG_CHANNEL
  #define CHAMBER3_ANALOG_COMMA     CHAMBER2_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_COOLER) && TEMP_SENSOR_COOLER != 0 && TEMP_SENSOR_COOLER >= -2)
  #define COOLER_ANALOG_INPUTS      1
  #define COOLER_ANALOG_CHANNEL     CHAMBER3_ANALOG_COMMA TEMP_COOLER_PIN
  #define COOLER_ANALOG_COMMA       ,
#else
  #define COOLER_ANALOG_INPUTS      0
  #define COOLER_ANALOG_CHANNEL
  #define COOLER_ANALOG_COMMA       CHAMBER3_ANALOG_COMMA
#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  #define FILAMENT_ANALOG_INPUTS    1
  #define FILAMENT_ANALOG_CHANNEL   COOLER_ANALOG_COMMA FILWIDTH_PIN
  #define FILAMENT_ANALOG_COMMA     ,
#else
  #define FILAMENT_ANALOG_INPUTS    0
  #define FILAMENT_ANALOG_CHANNEL
  #define FILAMENT_ANALOG_COMMA     COOLER_ANALOG_COMMA
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  #define POWER_ANALOG_INPUTS       1
  #define POWER_ANALOG_CHANNEL      FILAMENT_ANALOG_COMMA POWER_CONSUMPTION_PIN
  #define POWER_ANALOG_COMMA        ,
#else
  #define POWER_ANALOG_INPUTS       0
  #define POWER_ANALOG_CHANNEL
  #define POWER_ANALOG_COMMA        FILAMENT_ANALOG_COMMA
#endif

#if HAS_ADC_BUTTONS // Keypad for ANET board
  #define ADC_KEYPAD_ANALOG_INPUTS  1
  #define ADC_KEYPAD_ANALOG_CHANNEL POWER_ANALOG_COMMA ADC_KEYPAD_PIN
  #define ADC_KEYPAD_ANALOG_COMMA   ,
#else
  #define ADC_KEYPAD_ANALOG_INPUTS  0
  #define ADC_KEYPAD_ANALOG_CHANNEL
  #define ADC_KEYPAD_ANALOG_COMMA   POWER_ANALOG_COMMA
#endif

#if HAS_MCU_TEMPERATURE
  #define MCU_ANALOG_INPUTS         1
#else
  #define MCU_ANALOG_INPUTS         0
#endif

#define ANALOG_INPUTS ( HOT0_ANALOG_INPUTS      \
                      + HOT1_ANALOG_INPUTS      \
                      + HOT2_ANALOG_INPUTS      \
                      + HOT3_ANALOG_INPUTS      \
                      + HOT4_ANALOG_INPUTS      \
                      + HOT5_ANALOG_INPUTS      \
                      + BED0_ANALOG_INPUTS      \
                      + BED1_ANALOG_INPUTS      \
                      + BED2_ANALOG_INPUTS      \
                      + BED3_ANALOG_INPUTS      \
                      + CHAMBER0_ANALOG_INPUTS  \
                      + CHAMBER1_ANALOG_INPUTS  \
                      + CHAMBER2_ANALOG_INPUTS  \
                      + CHAMBER3_ANALOG_INPUTS  \
                      + COOLER_ANALOG_INPUTS    \
                      + FILAMENT_ANALOG_INPUTS  \
                      + POWER_ANALOG_INPUTS     \
                      + ADC_KEYPAD_ANALOG_INPUTS\
                      + MCU_ANALOG_INPUTS)

#if ANALOG_INPUTS > 0
  /** Channels are the MUX-part of ADMUX register */
  #define ANALOG_INPUT_CHANNELS { HOT0_ANALOG_CHANNEL     \
                                  HOT1_ANALOG_CHANNEL     \
                                  HOT2_ANALOG_CHANNEL     \
                                  HOT3_ANALOG_CHANNEL     \
                                  HOT4_ANALOG_CHANNEL     \
                                  HOT5_ANALOG_CHANNEL     \
                                  BED0_ANALOG_CHANNEL     \
                                  BED1_ANALOG_CHANNEL     \
                                  BED2_ANALOG_CHANNEL     \
                                  BED3_ANALOG_CHANNEL     \
                                  CHAMBER0_ANALOG_CHANNEL \
                                  CHAMBER1_ANALOG_CHANNEL \
                                  CHAMBER2_ANALOG_CHANNEL \
                                  CHAMBER3_ANALOG_CHANNEL \
                                  COOLER_ANALOG_CHANNEL   \
                                  FILAMENT_ANALOG_CHANNEL \
                                  POWER_ANALOG_CHANNEL    \
                                  ADC_KEYPAD_ANALOG_CHANNEL }
#else
  #define ANALOG_INPUT_CHANNELS { }
#endif

// Max consecutive low temp
#if DISABLED(MAX_CONSECUTIVE_LOW_TEMP)
  #define MAX_CONSECUTIVE_LOW_TEMP 2
#endif

// Calculate a default maximum stepper rate, if not supplied
#if DISABLED(MAXIMUM_STEPPER_RATE)
  #define MAXIMUM_STEPPER_RATE (500000UL)
#endif

/**
 * X STEPPER CUNT
 */
#if ENABLED(DUAL_X_CARRIAGE) || ENABLED(X_TWO_STEPPER_DRIVERS)
  #define X_STEPPER_COUNT 2
#else
  #define X_STEPPER_COUNT 1
#endif

/**
 * Y STEPPER CUNT
 */
#if ENABLED(Y_TWO_STEPPER_DRIVERS)
  #define Y_STEPPER_COUNT 2
#else
  #define Y_STEPPER_COUNT 1
#endif

/**
 * Z STEPPER COUNT
 */
#if ENABLED(Z_THREE_STEPPER_DRIVERS)
  #define Z_STEPPER_COUNT 3
#elif ENABLED(Z_TWO_STEPPER_DRIVERS)
  #define Z_STEPPER_COUNT 2
#else
  #define Z_STEPPER_COUNT 1
#endif

#define HAS_MULTY_STEPPER   (X_STEPPER_COUNT > 1 || Y_STEPPER_COUNT > 1 || Z_STEPPER_COUNT > 1)
#if HAS_MULTY_STEPPER
  #define MAX_DRIVER_XYZ    (XYZ + 4)
#else
  #define MAX_DRIVER_XYZ    (XYZ)
#endif
#define MAX_DRIVER          (MAX_DRIVER_XYZ + MAX_DRIVER_E)

/**
 * Define for max valor for Driver, Extruder, heater, fan
 */
#if HAS_LINEAR_EXTRUDER
  #ifdef __AVR__
    #define MAX_DRIVER_E     4
    #define MAX_EXTRUDER     4
    #define MAX_HOTEND       4
    #define MAX_BED          1
    #define MAX_CHAMBER      1
    #define MAX_COOLER       COOLERS
    #define MAX_FAN          4
  #else
    #define MAX_DRIVER_E     6
    #define MAX_EXTRUDER     6
    #define MAX_HOTEND       6
    #define MAX_BED          4
    #define MAX_CHAMBER      4
    #define MAX_COOLER       COOLERS
    #define MAX_FAN          6
  #endif
#else
  #ifdef __AVR__
    #define MAX_DRIVER_E    DRIVER_EXTRUDERS
    #define MAX_EXTRUDER    EXTRUDERS
    #define MAX_HOTEND      HOTENDS
    #define MAX_BED         1
    #define MAX_CHAMBER     1
    #define MAX_COOLER      COOLERS
    #define MAX_FAN         4
  #else
    #define MAX_DRIVER_E    DRIVER_EXTRUDERS
    #define MAX_EXTRUDER    EXTRUDERS
    #define MAX_HOTEND      HOTENDS
    #define MAX_BED         4
    #define MAX_CHAMBER     4
    #define MAX_COOLER      COOLERS
    #define MAX_FAN         6
  #endif
#endif // !HAS_LINEAR_EXTRUDER

#define HAS_HOTENDS         (MAX_HOTEND > 0)
#define HAS_BEDS            (MAX_BED > 0)
#define HAS_CHAMBERS        (MAX_CHAMBER > 0)
#define HAS_COOLERS         (MAX_COOLER > 0)
#define HAS_FAN             (MAX_FAN > 0)

/**
 * LCD define
 * Get LCD character width/height, which may be overridden by pins, configs, etc.
 */
#if DISABLED(LCD_WIDTH)
  #if HAS_GRAPHICAL_LCD
    #define LCD_WIDTH 21
  #elif ENABLED(ULTIPANEL)
    #define LCD_WIDTH 20
  #elif HAS_SPI_LCD
    #define LCD_WIDTH 16
  #endif
#endif
#if DISABLED(LCD_HEIGHT)
  #if HAS_GRAPHICAL_LCD
    #define LCD_HEIGHT 5
  #elif ENABLED(ULTIPANEL)
    #define LCD_HEIGHT 4
  #elif HAS_SPI_LCD
    #define LCD_HEIGHT 2
  #endif
#endif

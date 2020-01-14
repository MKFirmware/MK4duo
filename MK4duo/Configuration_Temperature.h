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
 * Configuration_Temperature.h
 *
 * This configuration file contains temperature settings.
 *
 * - Temperature Units support
 * - Thermistor type
 * - Temperature limits
 * - Automatic temperature
 * - Temperature status LEDs
 * - PWM Heater Frequency
 * - PID Settings - HOTEND
 * - PID Settings - BED
 * - PID Settings - CHAMBER
 * - PID Settings - COOLER
 * - Inverted PINS
 * - Thermal runaway protection
 * - Prevent cold extrusion
 * - Safety timer
 *
 */

/************************************************************************************
 ******************************** Temperature Units Support *************************
 ************************************************************************************
 *                                                                                  *
 * Enable TEMPERATURE UNITS SUPPORT for change unit with command Gcode M149         *
 *                                                                                  *
 ************************************************************************************/
//#define TEMPERATURE_UNITS_SUPPORT
/************************************************************************************/


/*****************************************************************************************************
 ************************************** Thermistor type **********************************************
 *****************************************************************************************************
 *                                                                                                   *
 * Please choose the one that matches your setup and set to TEMP SENSOR.                             *
 *                                                                                                   *
 *  -4 is thermocouple with MAX31855                                                                 *
 *  -3 is thermocouple with MAX6675                                                                  *
 *  -2 is thermocouple with AD8495                                                                   *
 *  -1 is thermocouple with AD595 or AD597                                                           *
 *   0 is not used                                                                                   *
 *   1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)                                 *
 *   2 is 100k thermistor - NTC3950 (4.7k pullup)                                                    *
 *   3 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)                                        *
 *   4 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)              *
 *   5 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)                                     *
 *   6 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)                                        *
 *   7 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)                                          *
 *   8 is 100k RS thermistor 198-961 (4.7k pullup)                                                   *
 *   9 User Sensor                                                                                   *
 *  11 DHT probe sensor DHT11, DHT12, DHT21 or DHT22 (ENABLE DHT SENSOR below)                       *
 *  20 is the PT100 circuit amplifier found in Ultimainboard V2.x and Wanhao D6                      *
 *                                                                                                   *
 *       Use these for Testing or Development purposes. NEVER for production machine.                *
 * 998 : Dummy Table that ALWAYS reads  25 degC or the temperature defined below.                    *
 * 999 : Dummy Table that ALWAYS reads 100 degC or the temperature defined below.                    *
 *                                                                                                   *
 *****************************************************************************************************/
#define TEMP_SENSOR_HE0       1
#define TEMP_SENSOR_HE1       0
#define TEMP_SENSOR_HE2       0
#define TEMP_SENSOR_HE3       0
#define TEMP_SENSOR_HE4       0
#define TEMP_SENSOR_HE5       0
#define TEMP_SENSOR_BED0      0
#define TEMP_SENSOR_BED1      0
#define TEMP_SENSOR_BED2      0
#define TEMP_SENSOR_BED3      0
#define TEMP_SENSOR_CHAMBER0  0
#define TEMP_SENSOR_CHAMBER1  0
#define TEMP_SENSOR_CHAMBER2  0
#define TEMP_SENSOR_CHAMBER3  0
#define TEMP_SENSOR_COOLER    0

// Thermistor series resistor value in Ohms (see on your board)
#define THERMISTOR_SERIES_RS 4700.0

// User Sensor
#define T9_NAME   "User Sensor"
#define T9_R25    100000.0  // Resistance in Ohms @ 25°C
#define T9_BETA     4036.0  // Beta Value (K)

// Enable this for support DHT sensor for temperature e Humidity DHT11, DHT21 or DHT22.
//#define DHT_SENSOR
// Set Type DHT 11 for DHT11, 12 for DHT12, 21 for DHT21, 22 for DHT22
#define DHT_TYPE 11

//These 2 defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
//The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

// Use it for Testing or Development purposes. NEVER for production machine.
#define DUMMY_THERMISTOR_998_VALUE  25
#define DUMMY_THERMISTOR_999_VALUE 100
/*****************************************************************************************/


/******************************************************************************************************
 ************************************** Temperature limits ********************************************
 ******************************************************************************************************/
// Temperature must be close to target for this long before M109-M190-M191-M192 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// When temperature exceeds max temp, your heater will be switched off.
// When temperature exceeds max temp, your cooler cannot be activaed.
// This feature exists to protect your hotend from overheating accidentally,
// but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HOTEND_0_MAXTEMP  275 // (degC)
#define HOTEND_1_MAXTEMP  275 // (degC)
#define HOTEND_2_MAXTEMP  275 // (degC)
#define HOTEND_3_MAXTEMP  275 // (degC)
#define HOTEND_4_MAXTEMP  275 // (degC)
#define HOTEND_5_MAXTEMP  275 // (degC)
#define BED_MAXTEMP       150 // (degC)
#define CHAMBER_MAXTEMP   100 // (degC)
#define COOLER_MAXTEMP     50 // (degC)

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// or, in case of cooler, it will switched off.
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HOTEND_0_MINTEMP  5 // (degC)
#define HOTEND_1_MINTEMP  5 // (degC)
#define HOTEND_2_MINTEMP  5 // (degC)
#define HOTEND_3_MINTEMP  5 // (degC)
#define HOTEND_4_MINTEMP  5 // (degC)
#define HOTEND_5_MINTEMP  5 // (degC)
#define BED_MINTEMP       5 // (degC)
#define CHAMBER_MINTEMP   5 // (degC)
#define COOLER_MINTEMP    5 // (degC)

// The number of consecutive low temperature errors that can occur
// before a min temp error is triggered. (Shouldn't be more than 10.)
#define MAX_CONSECUTIVE_LOW_TEMP 2

// Preheat Constants
#define PREHEAT_1_LABEL       "PLA"
#define PREHEAT_1_TEMP_HOTEND 190
#define PREHEAT_1_TEMP_BED     60
#define PREHEAT_1_TEMP_CHAMBER  0
#define PREHEAT_1_FAN_SPEED   255   // Insert Value between 0 and 255

#define PREHEAT_2_LABEL       "ABS"
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED    100
#define PREHEAT_2_TEMP_CHAMBER 50
#define PREHEAT_2_FAN_SPEED   255   // Insert Value between 0 and 255

#define PREHEAT_3_LABEL       "GUM"
#define PREHEAT_3_TEMP_HOTEND 230
#define PREHEAT_3_TEMP_BED     60
#define PREHEAT_3_TEMP_CHAMBER 50
#define PREHEAT_3_FAN_SPEED   255   // Insert Value between 0 and 255
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Automatic temperature **********************************
 *****************************************************************************************
 *                                                                                       *
 * The hotend target temperature is calculated by all the buffered lines of gcode.       *
 * The maximum buffered steps/sec of the extruder motor is called "se".                  *
 * Start autotemp mode with M109 S<mintemp> B<maxtemp> F<factor>                         *
 * The target temperature is set to mintemp+factor*se[steps/sec] and is limited by       *
 * mintemp and maxtemp. Turn this off by excuting M109 without F*                        *
 * Also, if the temperature is set to a value below mintemp, it will not be changed      *
 * by autotemp.                                                                          *
 * On an Ultimaker, some initial testing worked with M109 S215 B260 F1                   *
 * in the start.gcode                                                                    *
 *                                                                                       *
 *****************************************************************************************/
//#define AUTOTEMP
#define AUTOTEMP_OLDWEIGHT 0.98
/*****************************************************************************************/


/***********************************************************************
 ********************* Temperature status LEDs *************************
 ***********************************************************************
 *                                                                     *
 * Temperature status LEDs that display the hotend and bed             *
 * temperature.                                                        *
 * Otherwise the RED led is on. There is 1C hysteresis.                *
 *                                                                     *
 ***********************************************************************/
//#define TEMP_STAT_LEDS
/***********************************************************************/


/***********************************************************************
 ********************** PWM Heater Frequency ***************************
 ***********************************************************************
 *                                                                     *
 * PWM Heater Frequency only for Arduino DUE                           *
 * Max 1000 Hz                                                         *
 *                                                                     *
 ***********************************************************************/
#define HOTEND_PWM_FREQUENCY  250
#define BED_PWM_FREQUENCY     100
#define CHAMBER_PWM_FREQUENCY 100
#define COOLER_PWM_FREQUENCY  100
/***********************************************************************/


/***********************************************************************
 ********************** PID Settings - HOTEND **************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 *                                                                     *
 ***********************************************************************/
// Put to false following line to disable PID and enable bang-bang.
#define PIDTEMP true

#define POWER_MAX       255 // Limits current to nozzle;                                    255 = full current
#define POWER_DRIVE_MIN  40 // Limits min current to nozzle while PID/BANG_BANG is active;    0 = no current
#define POWER_DRIVE_MAX 230 // Limits max current to nozzle while PID/BANG_BANG is active;  255 = full current

#define HOTEND_HYSTERESIS 2       // (degC) range of +/- temperatures considered "close" to the target one
#define HOTEND_CHECK_INTERVAL 100 // ms between checks in bang-bang control

#define PID_AUTOTUNE_MENU // Add PID Autotune to the LCD "Temperature" menu to run M303 and apply the result.

// this adds an experimental additional term to the heating power, proportional to the extrusion speed.
// if Kc is chosen well, the additional required power due to increased melting should be compensated.
//#define PID_ADD_EXTRUSION_RATE
#define LPQ_MAX_LEN 50

//      HotEnd    {HE0,HE1,HE2,HE3,HE4,HE5}
#define HOTEND_Kp {40, 40, 40, 40, 40, 40}
#define HOTEND_Ki {07, 07, 07, 07, 07, 07}
#define HOTEND_Kd {60, 60, 60, 60, 60, 60}
#define HOTEND_Kc {100, 100, 100, 100, 100, 100} // Heating power = Kc * (e_speed)
/***********************************************************************/


/***********************************************************************
 ************************ PID Settings - BED ***************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 *                                                                     *
 ***********************************************************************/
// Put to false following line to disable PID and enable bang-bang.
#define PIDTEMPBED false

#define BED_POWER_MAX       255 // Limits current to bed;                                   255 = full current
#define BED_POWER_DRIVE_MIN  40 // Limits min current to bed while PID/BANG_BANG is active;   0 = no current
#define BED_POWER_DRIVE_MAX 230 // Limits max current to bed while PID/BANG_BANG is active; 255 = full current

#define BED_HYSTERESIS        2 // Only disable heating if T>target+BED HYSTERESIS and enable heating if T<target-BED HYSTERESIS
#define BED_CHECK_INTERVAL  500 // ms between checks in bang-bang control

//      BED     {BED0,BED1,BED2,BED3}
#define BED_Kp  {10,10,10,10}
#define BED_Ki  {1,1,1,1}
#define BED_Kd  {300,300,300,300}

// FIND YOUR OWN: "M303 H-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
/***********************************************************************/


/***********************************************************************
 ************************ PID Settings - CHAMBER ***********************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 *                                                                     *
 ***********************************************************************/
// Put to false following line to disable PID and enable bang-bang.
#define PIDTEMPCHAMBER false

#define CHAMBER_POWER_MAX       255 // Limits current to chamber;                                   255 = full current
#define CHAMBER_POWER_DRIVE_MIN  80 // Limits min current to chamber while PID/BANG_BANG is active;   0 = no current
#define CHAMBER_POWER_DRIVE_MAX 255 // Limits max current to chamber while PID/BANG_BANG is active; 255 = full current

#define CHAMBER_HYSTERESIS        2 // Only disable heating if T>target+CHAMBER HYSTERESIS and enable heating if T<target-CHAMBER HYSTERESIS
#define CHAMBER_CHECK_INTERVAL  500 // ms between checks in bang-bang control

//      CHAMBER     {CHAMBER0,CHAMBER1,CHAMBER2,CHAMBER3}
#define CHAMBER_Kp  {10,10,10,10}
#define CHAMBER_Ki  {1,1,1,1}
#define CHAMBER_Kd  {300,300,300,300}
/***********************************************************************/


/***********************************************************************
 ************************ PID Settings - COOLER ************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 *                                                                     *
 ***********************************************************************/
// Put to false following line to disable PID and enable bang-bang.
#define PIDTEMPCOOLER false

#define COOLER_POWER_MAX       255   // Limits current to chamber;                                    255 = full current
#define COOLER_POWER_DRIVE_MIN  80   // Limits min current to chamber while PID/BANG_BANG is active;    0 = no current
#define COOLER_POWER_DRIVE_MAX 255   // Limits max current to chamber while PID/BANG_BANG is active;  255 = full current

#define COOLER_HYSTERESIS        2 // only disable heating if T<target-COOLER_HYSTERESIS and enable heating if T>target+COOLER_HYSTERESIS
#define COOLER_CHECK_INTERVAL  500 // ms between checks in bang-bang control

#define COOLER_Kp  10
#define COOLER_Ki  1
#define COOLER_Kd  300
/***********************************************************************/


/********************************************************************************
 **************************** Inverted PINS *************************************
 ********************************************************************************
 *                                                                              *
 * For inverted logical Heater, Bed, Chamber or Cooler pins                     *
 *                                                                              *
 ********************************************************************************/
#define INVERTED_HEATER_PINS  false
#define INVERTED_BED_PIN      false
#define INVERTED_CHAMBER_PIN  false
#define INVERTED_COOLER_PIN   false
/********************************************************************************/


/**********************************************************************************
 ************************ Thermal runaway protection ******************************
 **********************************************************************************
 *                                                                                *
 * This protects your printer from damage and fire if a thermistor                *
 * falls out or temperature sensors fail in any way.                              *
 *                                                                                *
 * The issue: If a thermistor falls out or a temperature sensor fails,            *
 * MK4duo can no longer sense the actual temperature. Since a                     *
 * disconnected thermistor reads as a low temperature, the firmware               *
 * will keep the heater/cooler on.                                                *
 *                                                                                *
 * The solution: Once the temperature reaches the target, start                   *
 * observing. If the temperature stays too far below the                          *
 * target(hysteresis) for too long, the firmware will halt                        *
 * as a safety precaution.                                                        *
 *                                                                                *
 * Put THERMAL PROTECTION HOTENDS at true to enable this feature for all hotends. *
 * Put THERMAL PROTECTION BED at true to enable this feature for the heated bed.  *
 * Put THERMAL PROTECTION CHAMBER at true to enable this feature for the chamber. *
 * Put THERMAL PROTECTION COOLER at true to enable this feature for the cooler.   *
 *                                                                                *
 **********************************************************************************/
#define THERMAL_PROTECTION_HOTENDS  false
#define THERMAL_PROTECTION_BED      false
#define THERMAL_PROTECTION_CHAMBER  false
#define THERMAL_PROTECTION_COOLER   false

#define THERMAL_PROTECTION_PERIOD    40     // Seconds
#define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius

// If thermal protection hotends is true, this parameter adapt fan speed if temperature drops
//#define ADAPTIVE_FAN_SPEED

/**
 * When ever increases the target temperature the firmware will wait for the
 * WATCH TEMP PERIOD to expire, and if the temperature hasn't increased by WATCH TEMP INCREASE
 * degrees, the machine is halted, requiring a hard reset.
 *
 * If you get false positives for "Heating failed" increase WATCH TEMP PERIOD and/or decrease WATCH TEMP INCREASE
 * WATCH TEMP INCREASE should not be below 2.
 */
#define WATCH_HOTEND_PERIOD     20  // Seconds
#define WATCH_HOTEND_INCREASE    2  // Degrees Celsius

#define WATCH_BED_PERIOD        60  // Seconds
#define WATCH_BED_INCREASE       2  // Degrees Celsius

#define WATCH_CHAMBER_PERIOD    60  // Seconds
#define WATCH_CHAMBER_INCREASE   2  // Degrees Celsius

#define WATCH_COOLER_PERIOD     60  // Seconds
#define WATCH_COOLER_INCREASE    2  // Degrees Celsius
/********************************************************************************/


/***********************************************************************
 ************************ Prevent cold extrusion ***********************
 ***********************************************************************
 *                                                                     *
 * This option prevents extrusion if the temperature is                *
 * below EXTRUDE MINTEMP.                                              *
 * Add M302 to set the minimum extrusion temperature and/or turn       *
 * cold extrusion prevention on and off.                               *
 *                                                                     *
 *      IT IS HIGHLY RECOMMENDED TO LEAVE THIS OPTION ENABLED!         *
 *                                                                     *
 ***********************************************************************/
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170                 // Degree Celsius

// Prevent a single extrusion longer than EXTRUDE MAXLENGTH.
// Note: For Bowden Extruders make this large enough to allow load/unload.
//#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 400
/***********************************************************************/


/***********************************************************************
 **************************** Safety Timer *****************************
 ***********************************************************************
 *                                                                     *
 * Set safety timer expiration time in minutes, when not printing and  *
 * safety timer expires all heater are set to zero.                    *
 * Remeber put M530 S1 in your start gcode for start printing and      *
 * M530 S0 in end gcode for stop printing.                             *
 * M86 M[min] set safety timer expiration time                         *
 * M86 M0 will disable safety timer.                                   *
 *                                                                     *
 *      IT IS HIGHLY RECOMMENDED TO LEAVE THIS OPTION ENABLED!         *
 *                                                                     *
 ***********************************************************************/
#define SAFETYTIMER_TIME_MINS 30
/***********************************************************************/

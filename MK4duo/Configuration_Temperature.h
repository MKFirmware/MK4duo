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
 * Configuration_Temperature.h
 *
 * This configuration file contains temperature settings.
 *
 * - Temperature Units support
 * - Thermistor type
 * - Temperature limits
 * - Automatic temperature
 * - Temperature status LEDs
 * - PWM Heater Speed
 * - PID Settings - HOTEND
 * - PID Settings - BED
 * - PID Settings - CHAMBER
 * - PID Settings - COOLER
 * - Inverted PINS
 * - Thermal runaway protection
 * - Prevent cold extrusion
 *
 */

#ifndef _CONFIGURATION_TEMPERATURE_H_
#define _CONFIGURATION_TEMPERATURE_H_

/************************************************************************************
 ******************************** Temperature Units Support *************************
 ************************************************************************************
 *                                                                                  *
 * Enable TEMPERATURE UNITS SUPPORT for change unit width command Gcode M149        *
 *                                                                                  *
 ************************************************************************************/
//#define TEMPERATURE_UNITS_SUPPORT
/************************************************************************************/


/*****************************************************************************************************
 ************************************** Thermistor type **********************************************
 *****************************************************************************************************
 *                                                                                                   *
 * 4.7kohm PULLUP!                                                                                   *
 * This is a normal value, if you use a 1k pullup thermistor see below                               *
 * Please choose the one that matches your setup and set to TEMP_SENSOR_.                            *
 *                                                                                                   *
 * Temperature sensor settings (4.7kohm PULLUP):                                                     *
 *  -3 is thermocouple with MAX31855 (only Hotends)                                                  *
 *  -2 is thermocouple with MAX6675 (only Hotends)                                                   *
 *  -1 is thermocouple with AD595 or AD597                                                           *
 *   0 is not used                                                                                   *
 *   1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)                                 *
 *   2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)                                        *
 *   3 is Mendel-parts thermistor (4.7k pullup)                                                      *
 *   4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!     *
 *   5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)              *
 *   6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup) *
 *   7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)                                     *
 *  71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)                                     *
 *   8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)                                        *
 *   9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)                                          *
 *  10 is 100k RS thermistor 198-961 (4.7k pullup)                                                   *
 *  11 is 100k beta 3950 1% thermistor (4.7k pullup)                                                 *
 *  12 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)       *
 *  13 is 100k Hisens 3950 1% up to 300 degC for hotend "Simple ONE " & "Hotend "All In ONE"         *
 *  20 is the PT100 circuit found in the Ultimainboard V2.x                                          *
 *  60 is 100k Maker's Tool Works Kapton Bed Thermistor beta=3950                                    *
 *  66 is 4.7M High Temperature thermistor from Dyze Design                                          *
 *  70 is 100K thermistor found in the bq Hephestos 2                                                *
 *                                                                                                   *
 *    1k ohm pullup tables - This is atypical, and requires changing out the 4.7k pullup for 1k.     *
 *                           (but gives greater accuracy and more stable PID)                        *
 *  51 is 100k thermistor - EPCOS (1k pullup)                                                        *
 *  52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)                                          *
 *  55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)                *
 *                                                                                                   *
 *  1047 is Pt1000 with 4k7 pullup                                                                   *
 *  1010 is Pt1000 with 1k pullup (non standard)                                                     *
 *  147 is Pt100 with 4k7 pullup                                                                     *
 *  110 is Pt100 with 1k pullup (non standard)                                                       *
 *                                                                                                   *
 *         Use these for Testing or Development purposes. NEVER for production machine.              *
 *   998 : Dummy Table that ALWAYS reads 25 degC or the temperature defined below.                   *
 *   999 : Dummy Table that ALWAYS reads 100 degC or the temperature defined below.                  *
 *                                                                                                   *
 *****************************************************************************************************/
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 0
#define TEMP_SENSOR_CHAMBER 0
#define TEMP_SENSOR_COOLER 0

//These 2 defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
//The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

// Use it for Testing or Development purposes. NEVER for production machine.
#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 100

//Show Temperature ADC value
//The M105 command return, besides traditional information, the ADC value read from temperature sensors.
//#define SHOW_TEMP_ADC_VALUES
/*****************************************************************************************/


/******************************************************************************************************
 ************************************** Temperature limits ********************************************
 ******************************************************************************************************/
// Hotend temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// Bed temperature must be close to target for this long before M190 returns success
#define TEMP_BED_RESIDENCY_TIME 0   // (seconds)
#define TEMP_BED_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_BED_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// Chamber temperature must be close to target for this long before M190 returns success
#define TEMP_CHAMBER_RESIDENCY_TIME 0   // (seconds)
#define TEMP_CHAMBER_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_CHAMBER_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// Cooler temperature must be close to target for this long before M190 returns success
#define TEMP_COOLER_RESIDENCY_TIME 0    // (seconds)
#define TEMP_COOLER_HYSTERESIS 1        // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_COOLER_WINDOW     1        // (degC) Window around target to start the residency timer x degC early.

// When temperature exceeds max temp, your heater will be switched off.
// When temperature exceeds max temp, your cooler cannot be activaed.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275 // (degC)
#define HEATER_1_MAXTEMP 275 // (degC)
#define HEATER_2_MAXTEMP 275 // (degC)
#define HEATER_3_MAXTEMP 275 // (degC)
#define BED_MAXTEMP      150 // (degC)
#define CHAMBER_MAXTEMP  100 // (degC)
#define COOLER_MAXTEMP   35  // (degC) 

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// or, in case of cooler, it will switched off.
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5 // (degC)
#define HEATER_1_MINTEMP 5 // (degC)
#define HEATER_2_MINTEMP 5 // (degC)
#define HEATER_3_MINTEMP 5 // (degC)
#define BED_MINTEMP      5 // (degC)
#define CHAMBER_MINTEMP  5 // (degC)
#define COOLER_MINTEMP  10 // (degC) 

//Preheat Constants
#define PREHEAT_1_TEMP_HOTEND 190
#define PREHEAT_1_TEMP_BED     60
#define PREHEAT_1_FAN_SPEED   255   // Insert Value between 0 and 255

#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED    100
#define PREHEAT_2_FAN_SPEED   255   // Insert Value between 0 and 255

#define PREHEAT_3_TEMP_HOTEND 230
#define PREHEAT_3_TEMP_BED     60
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
#define AUTOTEMP
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
 ************************* PWM Heater Speed ****************************
 ***********************************************************************
 *                                                                     *
 * PWM Heater frequency and values                                     *
 *    0 -  15Hz 256 values                                             *
 *    1 -  30Hz 128 values                                             *
 *    2 -  61Hz  64 values                                             *
 *    3 - 122Hz  32 values                                             *
 *    4 - 244Hz  16 values                                             *
 *                                                                     *
 ***********************************************************************/
#define HEATER_PWM_SPEED 0
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

#define BANG_MAX  255       // Limits current to nozzle while in bang-bang mode; 255 = full current
#define PID_MIN     0       // Limits min current to nozzle while PID is active;   0 = no current
#define PID_MAX   255       // Limits max current to nozzle while PID is active; 255 = full current

//#define PID_AUTOTUNE_MENU // Add PID Autotune to the LCD "Temperature" menu to run M303 and apply the result.
//#define PID_DEBUG         // Sends debug data to the serial port.

// If the temperature difference between the target temperature and the actual temperature
// is more then PID FUNCTIONAL RANGE then the PID will be shut off and the heater will be set to min/max.
#define PID_FUNCTIONAL_RANGE 10

// this adds an experimental additional term to the heating power, proportional to the extrusion speed.
// if Kc is chosen well, the additional required power due to increased melting should be compensated.
//#define PID_ADD_EXTRUSION_RATE
#define LPQ_MAX_LEN 50

//           HotEnd{HE0,HE1,HE2,HE3}
#define DEFAULT_Kp {40, 40, 40, 40}     // Kp for H0, H1, H2, H3
#define DEFAULT_Ki {07, 07, 07, 07}     // Ki for H0, H1, H2, H3
#define DEFAULT_Kd {60, 60, 60, 60}     // Kd for H0, H1, H2, H3
#define DEFAULT_Kc {100, 100, 100, 100} // heating power = Kc * (e_speed)
/***********************************************************************/


/***********************************************************************
 ************************ PID Settings - BED ***************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 * Select PID or bang-bang with PIDTEMPBED.                            *
 * If bang-bang, BED LIMIT SWITCHING will enable hysteresis            *
 *                                                                     *
 ***********************************************************************/
// Put true to enable PID on the bed. It uses the same frequency PWM as the hotend.
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
#define PIDTEMPBED false

#define BED_HYSTERESIS        2 // Only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS (works only if BED_LIMIT_SWITCHING is enabled)
#define BED_CHECK_INTERVAL 5000 // ms between checks in bang-bang control

// This sets the max power delivered to the bed.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed,
// so you shouldn't use it unless you are OK with PWM on your bed. (see the comment on enabling PIDTEMPBED)
#define MIN_BED_POWER   0 // Limits duty cycle to bed;   0 = no current
#define MAX_BED_POWER 255 // Limits duty cycle to bed; 255 = full current

// 120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
// from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
#define DEFAULT_bedKp   10.00
#define DEFAULT_bedKi    0.1
#define DEFAULT_bedKd  300.0

// FIND YOUR OWN: "M303 H-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
/***********************************************************************/


/***********************************************************************
 ************************ PID Settings - CHAMBER ***********************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 * Select PID or bang-bang with PIDTEMPCHAMBER.                        *
 * If bang-bang, CHAMBER_LIMIT_SWITCHING will enable hysteresis        *
 *                                                                     *
 ***********************************************************************/
// Put true to enable PID on the chamber. It uses the same frequency PWM as the hotend.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use chamber PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
#define PIDTEMPCHAMBER false

#define CHAMBER_HYSTERESIS 2 //only disable heating if T>target+CHAMBER_HYSTERESIS and enable heating if T>target-CHAMBER_HYSTERESIS (works only if CHAMBER_LIMIT_SWITCHING is enabled)
#define CHAMBER_CHECK_INTERVAL 5000 //ms between checks in bang-bang control

// This sets the max power delivered to the chamber.
// all forms of chamber control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the chamber,
// so you shouldn't use it unless you are OK with PWM on your chamber.  (see the comment on enabling PIDTEMPCHAMBER)
#define MIN_CHAMBER_POWER   0 // Limits duty cycle to chamber;   0 = no current
#define MAX_CHAMBER_POWER 255 // Limits duty cycle to chamber; 255 = full current

// 120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
// from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
#define DEFAULT_chamberKp   10.00
#define DEFAULT_chamberKi    0.1
#define DEFAULT_chamberKd  300.0

// FIND YOUR OWN: "M303 E-2 C8 S90" to run autotune on the chamber at 90 degreesC for 8 cycles.
/***********************************************************************/


/***********************************************************************
 ************************ PID Settings - COOLER ************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 * Select PID or bang-bang with PIDTEMPCOOLER.                         *
 * If bang-bang, COOLER_LIMIT_SWITCHING will enable hysteresis         *
 *                                                                     *
 ***********************************************************************/
// Put true to enable PID on the cooler. It uses the same frequency PWM as the hotend.
// if you use a software PWM or the frequency you select if using an hardware PWM
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W cooler.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use cooler PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
#define PIDTEMPCOOLER false

// Enable fast PWM for cooler
//#define FAST_PWM_COOLER

#define COOLER_HYSTERESIS 2 //only disable heating if T<target-COOLER_HYSTERESIS and enable heating if T<target+COOLER_HYSTERESIS (works only if COOLER_LIMIT_SWITCHING is enabled)
#define COOLER_CHECK_INTERVAL 5000 //ms between checks in bang-bang control

// This sets the max power delivered to the cooler.
// all forms of cooler control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the cooler,
// so you shouldn't use it unless you are OK with PWM on your cooler.  (see the comment on enabling PIDTEMPCOOLER)
#define MIN_COOLER_POWER   0 // Limits duty cycle to cooler;   0 = no current
#define MAX_COOLER_POWER 255 // Limits duty cycle to cooler; 255 = full current

// 120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
// from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
#define DEFAULT_coolerKp 10.00
#define DEFAULT_coolerKi .023
#define DEFAULT_coolerKd 305.4

// FIND YOUR OWN: "M303 E-3 C8 S90" to run autotune on the cooler at 90 degreesC for 8 cycles.
/***********************************************************************/


/********************************************************************************
 **************************** Inverted PINS *************************************
 ********************************************************************************
 *                                                                              *
 * For inverted logical Heater, Bed, Chamber or Cooler pins                     *
 *                                                                              *
 ********************************************************************************/
#define INVERTED_HEATER_PINS false
#define INVERTED_BED_PIN false
#define INVERTED_CHAMBER_PIN false
#define INVERTED_COOLER_PIN false


/**********************************************************************************
 ************************ Thermal runaway protection ******************************
 **********************************************************************************
 *                                                                                *
 * This protects your printer from damage and fire if a thermistor                *
 * falls out or temperature sensors fail in any way.                              *
 *                                                                                *
 * The issue: If a thermistor falls out or a temperature sensor fails,            *
 * Marlin can no longer sense the actual temperature. Since a                     *
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
#define THERMAL_PROTECTION_HOTENDS false
#define THERMAL_PROTECTION_BED false
#define THERMAL_PROTECTION_CHAMBER false
#define THERMAL_PROTECTION_COOLER false

#define THERMAL_PROTECTION_PERIOD    40     // Seconds
#define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius

/**
 * When ever increases the target temperature the firmware will wait for the
 * WATCH TEMP PERIOD to expire, and if the temperature hasn't increased by WATCH TEMP INCREASE
 * degrees, the machine is halted, requiring a hard reset.
 *
 * If you get false positives for "Heating failed" increase WATCH TEMP PERIOD and/or decrease WATCH TEMP INCREASE
 * WATCH TEMP INCREASE should not be below 2.
 */
#define WATCH_TEMP_PERIOD  20               // Seconds
#define WATCH_TEMP_INCREASE 2               // Degrees Celsius
/********************************************************************************/


/***********************************************************************
 ************************ Prevent cold extrusion ***********************
 ***********************************************************************
 * This option prevents extrusion if the temperature is                *
 * below EXTRUDE_MINTEMP.                                              *
 * It also enables the M302 command to set the minimum extrusion       *
 * temperature or to allow moving the extruder regardless of the       *
 * hotend temperature.                                                 *
 * *** IT IS HIGHLY RECOMMENDED TO LEAVE THIS OPTION ENABLED! ***      *
 *                                                                     *
 ***********************************************************************/
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170                 // Degree Celsius

// This option prevents a single extrusion longer than EXTRUDE_MAXLENGTH.
//#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH + Y_MAX_LENGTH)
/***********************************************************************/

#endif /* _CONFIGURATION_TEMPERATURE_H_ */

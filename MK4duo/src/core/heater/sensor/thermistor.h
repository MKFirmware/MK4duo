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

#ifndef _THERMISTOR_H_
#define _THERMISTOR_H_

/**
 * Standard thermistors
 */

// 1 - EPCOS 100k
#define T1_NAME   "EPCOS 100K"
#define T1_R25    100000.0  // Resistance in Ohms @ 25°C
#define T1_BETA     4092.0  // Beta Value (K)

// 2 - NTC3950
#define T2_NAME   "NTC3950"
#define T2_R25    100000.0  // Resistance in Ohms @ 25°C
#define T2_BETA     3950.0  // Beta Value (K)

// 3 - ATC Semitec 204GT-2
#define T3_NAME   "ATC 204GT-2"
#define T3_R25    200000.0  // Resistance in Ohms @ 25°C
#define T3_BETA     4338.0  // Beta Value (K)

// 4 - ATC Semitec 104GT-2
#define T4_NAME   "ATC 104GT-2"
#define T4_R25    100000.0  // Resistance in Ohms @ 25°C
#define T4_BETA     4725.0  // Beta Value (K)

// 5 - Honeywell thermistor 135-104LAG-J01
#define T5_NAME   "HW 104LAG"
#define T5_R25    100000.0  // Resistance in Ohms @ 25°C
#define T5_BETA     3974.0  // Beta Value (K)

// 6 - Vishay NTCS0603E3104FXT
#define T6_NAME   "E3104FXT"
#define T6_R25    100000.0  // Resistance in Ohms @ 25°C
#define T6_BETA     4100.0  // Beta Value (K)

// 7 - GE Sensing AL03006-58.2K-97-G1
#define T7_NAME   "GE AL03006"
#define T7_R25    100000.0  // Resistance in Ohms @ 25°C
#define T7_BETA     3952.0  // Beta Value (K)

// 8 - RS Pro Thermistor 198-961
#define T8_NAME   "RS 198-961"
#define T8_R25    100000.0  // Resistance in Ohms @ 25°C
#define T8_BETA     3960.0  // Beta Value (K)

#define _THERMISTOR_NAME_IS(n)  T ## n ## _NAME
#define _THERMISTOR_R25_IS(n)   T ## n ## _R25
#define _THERMISTOR_BETA_IS(n)  T ## n ## _BETA

#define THERMISTOR_NAME_IS(n) _THERMISTOR_NAME_IS(n)
#define THERMISTOR_R25_IS(n)  _THERMISTOR_R25_IS(n)
#define THERMISTOR_BETA_IS(n) _THERMISTOR_BETA_IS(n)

#if ENABLED(HEATER_0_USES_THERMISTOR)
  #define HOT0_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_0)
  #define HOT0_R25  THERMISTOR_R25_IS(TEMP_SENSOR_0)
  #define HOT0_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_0)
#endif
#if ENABLED(HEATER_1_USES_THERMISTOR)
  #define HOT1_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_1)
  #define HOT1_R25  THERMISTOR_R25_IS(TEMP_SENSOR_1)
  #define HOT1_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_1)
#endif
#if ENABLED(HEATER_2_USES_THERMISTOR)
  #define HOT2_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_2)
  #define HOT2_R25  THERMISTOR_R25_IS(TEMP_SENSOR_2)
  #define HOT2_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_2)
#endif
#if ENABLED(HEATER_3_USES_THERMISTOR)
  #define HOT3_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_3)
  #define HOT3_R25  THERMISTOR_R25_IS(TEMP_SENSOR_3)
  #define HOT3_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_3)
#endif
#if ENABLED(BED_USES_THERMISTOR)
  #define BED_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_BED)
  #define BED_R25  THERMISTOR_R25_IS(TEMP_SENSOR_BED)
  #define BED_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_BED)
#endif
#if ENABLED(CHAMBER_USES_THERMISTOR)
  #define CHAMBER_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_CHAMBER)
  #define CHAMBER_R25  THERMISTOR_R25_IS(TEMP_SENSOR_CHAMBER)
  #define CHAMBER_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_CHAMBER)
#endif
#if ENABLED(COOLER_USES_THERMISTOR)
  #define COOLER_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_COOLER)
  #define COOLER_R25  THERMISTOR_R25_IS(TEMP_SENSOR_COOLER)
  #define COOLER_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_COOLER)
#endif

#if TEMP_SENSOR_0 == -3
  #define HOT0_NAME "MAX31855"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_0 == -2
  #define HOT0_NAME "MAX6675"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_0 == -1
  #define HOT0_NAME "AD595"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_0 == 11
  #define HOT0_NAME "DHT11"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_0 == 20
  #define HOT0_NAME "AMPLIFIER"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_0 == 998 || TEMP_SENSOR_0 == 999
  #define HOT0_NAME "DUMMY SENSOR"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#endif

#if TEMP_SENSOR_1 == -3
  #define HOT1_NAME "MAX31855"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_1 == -2
  #define HOT1_NAME "MAX6675"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_1 == -1
  #define HOT1_NAME "AD595"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_1 == 11
  #define HOT1_NAME "DHT11"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_1 == 20
  #define HOT1_NAME "AMPLIFIER"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_1 == 998 || TEMP_SENSOR_1 == 999
  #define HOT1_NAME "DUMMY SENSOR"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#endif

#if TEMP_SENSOR_2 == -3
  #define HOT2_NAME "MAX31855"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_2 == -2
  #define HOT2_NAME "MAX6675"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_2 == -1
  #define HOT2_NAME "AD595"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_2 == 11
  #define HOT2_NAME "DHT11"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_2 == 20
  #define HOT2_NAME "AMPLIFIER"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_2 == 998 || TEMP_SENSOR_2 == 999
  #define HOT2_NAME "DUMMY SENSOR"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#endif

#if TEMP_SENSOR_3 == -3
  #define HOT3_NAME "MAX31855"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_3 == -2
  #define HOT3_NAME "MAX6675"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_3 == -1
  #define HOT3_NAME "AD595"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_3 == 11
  #define HOT3_NAME "DHT11"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_3 == 20
  #define HOT3_NAME "AMPLIFIER"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_3 == 998 || TEMP_SENSOR_3 == 999
  #define HOT3_NAME "DUMMY SENSOR"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#endif

#if TEMP_SENSOR_BED == -3
  #define BED_NAME "MAX31855"
  #define BED_R25  0.0
  #define BED_BETA 0.0
#elif TEMP_SENSOR_BED == -2
  #define BED_NAME "MAX6675"
  #define BED_R25  0.0
  #define BED_BETA 0.0
#elif TEMP_SENSOR_BED == -1
  #define BED_NAME "AD595"
  #define BED_R25  0.0
  #define BED_BETA 0.0
#elif TEMP_SENSOR_BED == 11
  #define BED_NAME "DHT11"
  #define BED_R25  0.0
  #define BED_BETA 0.0
#elif TEMP_SENSOR_BED == 20
  #define BED_NAME "AMPLIFIER"
  #define BED_R25  0.0
  #define BED_BETA 0.0
#elif TEMP_SENSOR_BED == 998 || TEMP_SENSOR_BED == 999
  #define BED_NAME "DUMMY SENSOR"
  #define BED_R25  0.0
  #define BED_BETA 0.0
#endif

#if TEMP_SENSOR_CHAMBER == -3
  #define CHAMBER_NAME "MAX31855"
  #define CHAMBER_R25  0.0
  #define CHAMBER_BETA 0.0
#elif TEMP_SENSOR_CHAMBER == -2
  #define CHAMBER_NAME "MAX6675"
  #define CHAMBER_R25  0.0
  #define CHAMBER_BETA 0.0
#elif TEMP_SENSOR_CHAMBER == -1
  #define CHAMBER_NAME "AD595"
  #define CHAMBER_R25  0.0
  #define CHAMBER_BETA 0.0
#elif TEMP_SENSOR_CHAMBER == 11
  #define CHAMBER_NAME "DHT11"
  #define CHAMBER_R25  0.0
  #define CHAMBER_BETA 0.0
#elif TEMP_SENSOR_CHAMBER == 20
  #define CHAMBER_NAME "AMPLIFIER"
  #define CHAMBER_R25  0.0
  #define CHAMBER_BETA 0.0
#elif TEMP_SENSOR_CHAMBER == 998 || TEMP_SENSOR_CHAMBER == 999
  #define CHAMBER_NAME "DUMMY SENSOR"
  #define CHAMBER_R25  0.0
  #define CHAMBER_BETA 0.0
#endif

#if TEMP_SENSOR_COOLER == -3
  #define COOLER_NAME "MAX31855"
  #define COOLER_R25  0.0
  #define COOLER_BETA 0.0
#elif TEMP_SENSOR_COOLER == -2
  #define COOLER_NAME "MAX6675"
  #define COOLER_R25  0.0
  #define COOLER_BETA 0.0
#elif TEMP_SENSOR_COOLER == -1
  #define COOLER_NAME "AD595"
  #define COOLER_R25  0.0
  #define COOLER_BETA 0.0
#elif TEMP_SENSOR_COOLER == 11
  #define COOLER_NAME "DHT11"
  #define COOLER_R25  0.0
  #define COOLER_BETA 0.0
#elif TEMP_SENSOR_COOLER == 20
  #define COOLER_NAME "AMPLIFIER"
  #define COOLER_R25  0.0
  #define COOLER_BETA 0.0
#elif TEMP_SENSOR_COOLER == 998 || TEMP_SENSOR_COOLER == 999
  #define COOLER_NAME "DUMMY SENSOR"
  #define COOLER_R25  0.0
  #define COOLER_BETA 0.0
#endif

#if HEATER_USES_AMPLIFIER
  #include "thermistoramplifier.h"
#endif

#endif /* _THERMISTORTABLES_H_ */

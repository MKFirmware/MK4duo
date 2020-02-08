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

#define _THERMISTOR_NAME_IS(N)  T ## N ## _NAME
#define _THERMISTOR_R25_IS(N)   T ## N ## _R25
#define _THERMISTOR_BETA_IS(N)  T ## N ## _BETA

#define THERMISTOR_NAME_IS(N) _THERMISTOR_NAME_IS(N)
#define THERMISTOR_R25_IS(N)  _THERMISTOR_R25_IS(N)
#define THERMISTOR_BETA_IS(N) _THERMISTOR_BETA_IS(N)

#if TEMP_SENSOR_HE0 > 0 && TEMP_SENSOR_HE0 < 10
  #define HOT0_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_HE0)
  #define HOT0_R25  THERMISTOR_R25_IS(TEMP_SENSOR_HE0)
  #define HOT0_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_HE0)
#endif
#if TEMP_SENSOR_HE1 > 0 && TEMP_SENSOR_HE1 < 10
  #define HOT1_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_HE1)
  #define HOT1_R25  THERMISTOR_R25_IS(TEMP_SENSOR_HE1)
  #define HOT1_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_HE1)
#endif
#if TEMP_SENSOR_HE2 > 0 && TEMP_SENSOR_HE2 < 10
  #define HOT2_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_HE2)
  #define HOT2_R25  THERMISTOR_R25_IS(TEMP_SENSOR_HE2)
  #define HOT2_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_HE2)
#endif
#if TEMP_SENSOR_HE3 > 0 && TEMP_SENSOR_HE3 < 10
  #define HOT3_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_HE3)
  #define HOT3_R25  THERMISTOR_R25_IS(TEMP_SENSOR_HE3)
  #define HOT3_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_HE3)
#endif
#if TEMP_SENSOR_HE4 > 0 && TEMP_SENSOR_HE4 < 10
  #define HOT4_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_HE4)
  #define HOT4_R25  THERMISTOR_R25_IS(TEMP_SENSOR_HE4)
  #define HOT4_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_HE4)
#endif
#if TEMP_SENSOR_HE5 > 0 && TEMP_SENSOR_HE5 < 10
  #define HOT5_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_HE5)
  #define HOT5_R25  THERMISTOR_R25_IS(TEMP_SENSOR_HE5)
  #define HOT5_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_HE5)
#endif
#if TEMP_SENSOR_BED0 > 0 && TEMP_SENSOR_BED0 < 10
  #define BED0_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_BED0)
  #define BED0_R25  THERMISTOR_R25_IS(TEMP_SENSOR_BED0)
  #define BED0_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_BED0)
#endif
#if TEMP_SENSOR_BED1 > 0 && TEMP_SENSOR_BED1 < 10
  #define BED1_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_BED1)
  #define BED1_R25  THERMISTOR_R25_IS(TEMP_SENSOR_BED1)
  #define BED1_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_BED1)
#endif
#if TEMP_SENSOR_BED2 > 0 && TEMP_SENSOR_BED2 < 10
  #define BED2_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_BED2)
  #define BED2_R25  THERMISTOR_R25_IS(TEMP_SENSOR_BED2)
  #define BED2_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_BED2)
#endif
#if TEMP_SENSOR_BED3 > 0 && TEMP_SENSOR_BED3 < 10
  #define BED3_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_BED3)
  #define BED3_R25  THERMISTOR_R25_IS(TEMP_SENSOR_BED3)
  #define BED3_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_BED3)
#endif
#if TEMP_SENSOR_CHAMBER0 > 0 && TEMP_SENSOR_CHAMBER0 < 10
  #define CHAMBER0_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_CHAMBER0)
  #define CHAMBER0_R25  THERMISTOR_R25_IS(TEMP_SENSOR_CHAMBER0)
  #define CHAMBER0_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_CHAMBER0)
#endif
#if TEMP_SENSOR_CHAMBER1 > 0 && TEMP_SENSOR_CHAMBER1 < 10
  #define CHAMBER1_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_CHAMBER1)
  #define CHAMBER1_R25  THERMISTOR_R25_IS(TEMP_SENSOR_CHAMBER1)
  #define CHAMBER1_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_CHAMBER1)
#endif
#if TEMP_SENSOR_CHAMBER2 > 0 && TEMP_SENSOR_CHAMBER2 < 10
  #define CHAMBER2_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_CHAMBER2)
  #define CHAMBER2_R25  THERMISTOR_R25_IS(TEMP_SENSOR_CHAMBER2)
  #define CHAMBER2_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_CHAMBER2)
#endif
#if TEMP_SENSOR_CHAMBER3 > 0 && TEMP_SENSOR_CHAMBER3 < 10
  #define CHAMBER3_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_CHAMBER3)
  #define CHAMBER3_R25  THERMISTOR_R25_IS(TEMP_SENSOR_CHAMBER3)
  #define CHAMBER3_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_CHAMBER3)
#endif
#if TEMP_SENSOR_COOLER > 0 && TEMP_SENSOR_COOLER < 10
  #define COOLER_NAME THERMISTOR_NAME_IS(TEMP_SENSOR_COOLER)
  #define COOLER_R25  THERMISTOR_R25_IS(TEMP_SENSOR_COOLER)
  #define COOLER_BETA THERMISTOR_BETA_IS(TEMP_SENSOR_COOLER)
#endif

#if TEMP_SENSOR_HE0 == -4
  #define HOT0_NAME "MAX31855"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_HE0 == -3
  #define HOT0_NAME "MAX6675"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_HE0 == -2
  #define HOT0_NAME "AD8495"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_HE0 == -1
  #define HOT0_NAME "AD595"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_HE0 == 11
  #define HOT0_NAME "DHT11"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_HE0 == 20
  #define HOT0_NAME "AMPLIFIER"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_HE0 == 998 || TEMP_SENSOR_HE0 == 999
  #define HOT0_NAME "DUMMY SENSOR"
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#elif TEMP_SENSOR_HE0 == 0
  #define HOT0_NAME ""
  #define HOT0_R25  0.0
  #define HOT0_BETA 0.0
#endif

#if TEMP_SENSOR_HE1 == -4
  #define HOT1_NAME "MAX31855"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_HE1 == -3
  #define HOT1_NAME "MAX6675"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_HE1 == -2
  #define HOT1_NAME "AD8495"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_HE1 == -1
  #define HOT1_NAME "AD595"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_HE1 == 11
  #define HOT1_NAME "DHT11"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_HE1 == 20
  #define HOT1_NAME "AMPLIFIER"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_HE1 == 998 || TEMP_SENSOR_HE1 == 999
  #define HOT1_NAME "DUMMY SENSOR"
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#elif TEMP_SENSOR_HE1 == 0
  #define HOT1_NAME ""
  #define HOT1_R25  0.0
  #define HOT1_BETA 0.0
#endif

#if TEMP_SENSOR_HE2 == -4
  #define HOT2_NAME "MAX31855"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_HE2 == -3
  #define HOT2_NAME "MAX6675"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_HE2 == -2
  #define HOT2_NAME "AD8495"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_HE2 == -1
  #define HOT2_NAME "AD595"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_HE2 == 11
  #define HOT2_NAME "DHT11"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_HE2 == 20
  #define HOT2_NAME "AMPLIFIER"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_HE2 == 998 || TEMP_SENSOR_HE2 == 999
  #define HOT2_NAME "DUMMY SENSOR"
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#elif TEMP_SENSOR_HE2 == 0
  #define HOT2_NAME ""
  #define HOT2_R25  0.0
  #define HOT2_BETA 0.0
#endif

#if TEMP_SENSOR_HE3 == -4
  #define HOT3_NAME "MAX31855"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_HE3 == -3
  #define HOT3_NAME "MAX6675"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_HE3 == -2
  #define HOT3_NAME "AD8495"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_HE3 == -1
  #define HOT3_NAME "AD595"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_HE3 == 11
  #define HOT3_NAME "DHT11"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_HE3 == 20
  #define HOT3_NAME "AMPLIFIER"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_HE3 == 998 || TEMP_SENSOR_HE3 == 999
  #define HOT3_NAME "DUMMY SENSOR"
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#elif TEMP_SENSOR_HE3 == 0
  #define HOT3_NAME ""
  #define HOT3_R25  0.0
  #define HOT3_BETA 0.0
#endif

#if TEMP_SENSOR_HE4 == -4
  #define HOT4_NAME "MAX31855"
  #define HOT4_R25  0.0
  #define HOT4_BETA 0.0
#elif TEMP_SENSOR_HE4 == -3
  #define HOT4_NAME "MAX6675"
  #define HOT4_R25  0.0
  #define HOT4_BETA 0.0
#elif TEMP_SENSOR_HE4 == -2
  #define HOT4_NAME "AD8495"
  #define HOT4_R25  0.0
  #define HOT4_BETA 0.0
#elif TEMP_SENSOR_HE4 == -1
  #define HOT4_NAME "AD595"
  #define HOT4_R25  0.0
  #define HOT4_BETA 0.0
#elif TEMP_SENSOR_HE4 == 11
  #define HOT4_NAME "DHT11"
  #define HOT4_R25  0.0
  #define HOT4_BETA 0.0
#elif TEMP_SENSOR_HE4 == 20
  #define HOT4_NAME "AMPLIFIER"
  #define HOT4_R25  0.0
  #define HOT4_BETA 0.0
#elif TEMP_SENSOR_HE4 == 998 || TEMP_SENSOR_HE4 == 999
  #define HOT4_NAME "DUMMY SENSOR"
  #define HOT4_R25  0.0
  #define HOT4_BETA 0.0
#elif TEMP_SENSOR_HE4 == 0
  #define HOT4_NAME ""
  #define HOT4_R25  0.0
  #define HOT4_BETA 0.0
#endif

#if TEMP_SENSOR_HE5 == -4
  #define HOT5_NAME "MAX31855"
  #define HOT5_R25  0.0
  #define HOT5_BETA 0.0
#elif TEMP_SENSOR_HE5 == -3
  #define HOT5_NAME "MAX6675"
  #define HOT5_R25  0.0
  #define HOT5_BETA 0.0
#elif TEMP_SENSOR_HE5 == -2
  #define HOT5_NAME "AD8495"
  #define HOT5_R25  0.0
  #define HOT5_BETA 0.0
#elif TEMP_SENSOR_HE5 == -1
  #define HOT5_NAME "AD595"
  #define HOT5_R25  0.0
  #define HOT5_BETA 0.0
#elif TEMP_SENSOR_HE5 == 11
  #define HOT5_NAME "DHT11"
  #define HOT5_R25  0.0
  #define HOT5_BETA 0.0
#elif TEMP_SENSOR_HE5 == 20
  #define HOT5_NAME "AMPLIFIER"
  #define HOT5_R25  0.0
  #define HOT5_BETA 0.0
#elif TEMP_SENSOR_HE5 == 998 || TEMP_SENSOR_HE5 == 999
  #define HOT5_NAME "DUMMY SENSOR"
  #define HOT5_R25  0.0
  #define HOT5_BETA 0.0
#elif TEMP_SENSOR_HE5 == 0
  #define HOT5_NAME ""
  #define HOT5_R25  0.0
  #define HOT5_BETA 0.0
#endif

#if TEMP_SENSOR_BED0 == -4
  #define BED0_NAME "MAX31855"
  #define BED0_R25  0.0
  #define BED0_BETA 0.0
#elif TEMP_SENSOR_BED0 == -3
  #define BED0_NAME "MAX6675"
  #define BED0_R25  0.0
  #define BED0_BETA 0.0
#elif TEMP_SENSOR_BED0 == -2
  #define BED0_NAME "AD8495"
  #define BED0_R25  0.0
  #define BED0_BETA 0.0
#elif TEMP_SENSOR_BED0 == -1
  #define BED0_NAME "AD595"
  #define BED0_R25  0.0
  #define BED0_BETA 0.0
#elif TEMP_SENSOR_BED0 == 11
  #define BED0_NAME "DHT11"
  #define BED0_R25  0.0
  #define BED0_BETA 0.0
#elif TEMP_SENSOR_BED0 == 20
  #define BED0_NAME "AMPLIFIER"
  #define BED0_R25  0.0
  #define BED0_BETA 0.0
#elif TEMP_SENSOR_BED0 == 998 || TEMP_SENSOR_BED0 == 999
  #define BED0_NAME "DUMMY SENSOR"
  #define BED0_R25  0.0
  #define BED0_BETA 0.0
#elif TEMP_SENSOR_BED0 == 0
  #define BED0_NAME ""
  #define BED0_R25  0.0
  #define BED0_BETA 0.0
#endif

#if TEMP_SENSOR_BED1 == -4
  #define BED1_NAME "MAX31855"
  #define BED1_R25  0.0
  #define BED1_BETA 0.0
#elif TEMP_SENSOR_BED1 == -3
  #define BED1_NAME "MAX6675"
  #define BED1_R25  0.0
  #define BED1_BETA 0.0
#elif TEMP_SENSOR_BED1 == -2
  #define BED1_NAME "AD8495"
  #define BED1_R25  0.0
  #define BED1_BETA 0.0
#elif TEMP_SENSOR_BED1 == -1
  #define BED1_NAME "AD595"
  #define BED1_R25  0.0
  #define BED1_BETA 0.0
#elif TEMP_SENSOR_BED1 == 11
  #define BED1_NAME "DHT11"
  #define BED1_R25  0.0
  #define BED1_BETA 0.0
#elif TEMP_SENSOR_BED1 == 20
  #define BED1_NAME "AMPLIFIER"
  #define BED1_R25  0.0
  #define BED1_BETA 0.0
#elif TEMP_SENSOR_BED1 == 998 || TEMP_SENSOR_BED1 == 999
  #define BED1_NAME "DUMMY SENSOR"
  #define BED1_R25  0.0
  #define BED1_BETA 0.0
#endif

#if TEMP_SENSOR_BED2 == -4
  #define BED2_NAME "MAX31855"
  #define BED2_R25  0.0
  #define BED2_BETA 0.0
#elif TEMP_SENSOR_BED2 == -3
  #define BED2_NAME "MAX6675"
  #define BED2_R25  0.0
  #define BED2_BETA 0.0
#elif TEMP_SENSOR_BED2 == -2
  #define BED2_NAME "AD8495"
  #define BED2_R25  0.0
  #define BED2_BETA 0.0
#elif TEMP_SENSOR_BED2 == -1
  #define BED2_NAME "AD595"
  #define BED2_R25  0.0
  #define BED2_BETA 0.0
#elif TEMP_SENSOR_BED2 == 11
  #define BED2_NAME "DHT11"
  #define BED2_R25  0.0
  #define BED2_BETA 0.0
#elif TEMP_SENSOR_BED2 == 20
  #define BED2_NAME "AMPLIFIER"
  #define BED2_R25  0.0
  #define BED2_BETA 0.0
#elif TEMP_SENSOR_BED2 == 998 || TEMP_SENSOR_BED2 == 999
  #define BED2_NAME "DUMMY SENSOR"
  #define BED2_R25  0.0
  #define BED2_BETA 0.0
#endif

#if TEMP_SENSOR_BED3 == -4
  #define BED3_NAME "MAX31855"
  #define BED3_R25  0.0
  #define BED3_BETA 0.0
#elif TEMP_SENSOR_BED3 == -3
  #define BED3_NAME "MAX6675"
  #define BED3_R25  0.0
  #define BED3_BETA 0.0
#elif TEMP_SENSOR_BED3 == -2
  #define BED3_NAME "AD8495"
  #define BED3_R25  0.0
  #define BED3_BETA 0.0
#elif TEMP_SENSOR_BED3 == -1
  #define BED3_NAME "AD595"
  #define BED3_R25  0.0
  #define BED3_BETA 0.0
#elif TEMP_SENSOR_BED3 == 11
  #define BED3_NAME "DHT11"
  #define BED3_R25  0.0
  #define BED3_BETA 0.0
#elif TEMP_SENSOR_BED3 == 20
  #define BED3_NAME "AMPLIFIER"
  #define BED3_R25  0.0
  #define BED3_BETA 0.0
#elif TEMP_SENSOR_BED3 == 998 || TEMP_SENSOR_BED3 == 999
  #define BED3_NAME "DUMMY SENSOR"
  #define BED3_R25  0.0
  #define BED3_BETA 0.0
#endif

#if TEMP_SENSOR_CHAMBER0 == -4
  #define CHAMBER0_NAME "MAX31855"
  #define CHAMBER0_R25  0.0
  #define CHAMBER0_BETA 0.0
#elif TEMP_SENSOR_CHAMBER0 == -3
  #define CHAMBER0_NAME "MAX6675"
  #define CHAMBER0_R25  0.0
  #define CHAMBER0_BETA 0.0
#elif TEMP_SENSOR_CHAMBER0 == -2
  #define CHAMBER0_NAME "AD8495"
  #define CHAMBER0_R25  0.0
  #define CHAMBER0_BETA 0.0
#elif TEMP_SENSOR_CHAMBER0 == -1
  #define CHAMBER0_NAME "AD595"
  #define CHAMBER0_R25  0.0
  #define CHAMBER0_BETA 0.0
#elif TEMP_SENSOR_CHAMBER0 == 11
  #define CHAMBER0_NAME "DHT11"
  #define CHAMBER0_R25  0.0
  #define CHAMBER0_BETA 0.0
#elif TEMP_SENSOR_CHAMBER0 == 20
  #define CHAMBER0_NAME "AMPLIFIER"
  #define CHAMBER0_R25  0.0
  #define CHAMBER0_BETA 0.0
#elif TEMP_SENSOR_CHAMBER0 == 998 || TEMP_SENSOR_CHAMBER0 == 999
  #define CHAMBER0_NAME "DUMMY SENSOR"
  #define CHAMBER0_R25  0.0
  #define CHAMBER0_BETA 0.0
#elif TEMP_SENSOR_CHAMBER0 == 0
  #define CHAMBER0_NAME ""
  #define CHAMBER0_R25  0.0
  #define CHAMBER0_BETA 0.0
#endif

#if TEMP_SENSOR_CHAMBER1 == -4
  #define CHAMBER1_NAME "MAX31855"
  #define CHAMBER1_R25  0.0
  #define CHAMBER1_BETA 0.0
#elif TEMP_SENSOR_CHAMBER1 == -3
  #define CHAMBER1_NAME "MAX6675"
  #define CHAMBER1_R25  0.0
  #define CHAMBER1_BETA 0.0
#elif TEMP_SENSOR_CHAMBER1 == -2
  #define CHAMBER1_NAME "AD8495"
  #define CHAMBER1_R25  0.0
  #define CHAMBER1_BETA 0.0
#elif TEMP_SENSOR_CHAMBER1 == -1
  #define CHAMBER1_NAME "AD595"
  #define CHAMBER1_R25  0.0
  #define CHAMBER1_BETA 0.0
#elif TEMP_SENSOR_CHAMBER1 == 11
  #define CHAMBER1_NAME "DHT11"
  #define CHAMBER1_R25  0.0
  #define CHAMBER1_BETA 0.0
#elif TEMP_SENSOR_CHAMBER1 == 20
  #define CHAMBER1_NAME "AMPLIFIER"
  #define CHAMBER1_R25  0.0
  #define CHAMBER1_BETA 0.0
#elif TEMP_SENSOR_CHAMBER1 == 998 || TEMP_SENSOR_CHAMBER1 == 999
  #define CHAMBER1_NAME "DUMMY SENSOR"
  #define CHAMBER1_R25  0.0
  #define CHAMBER1_BETA 0.0
#endif

#if TEMP_SENSOR_CHAMBER2 == -4
  #define CHAMBER2_NAME "MAX31855"
  #define CHAMBER2_R25  0.0
  #define CHAMBER2_BETA 0.0
#elif TEMP_SENSOR_CHAMBER2 == -3
  #define CHAMBER2_NAME "MAX6675"
  #define CHAMBER2_R25  0.0
  #define CHAMBER2_BETA 0.0
#elif TEMP_SENSOR_CHAMBER2 == -2
  #define CHAMBER2_NAME "AD8495"
  #define CHAMBER2_R25  0.0
  #define CHAMBER2_BETA 0.0
#elif TEMP_SENSOR_CHAMBER2 == -1
  #define CHAMBER2_NAME "AD595"
  #define CHAMBER2_R25  0.0
  #define CHAMBER2_BETA 0.0
#elif TEMP_SENSOR_CHAMBER2 == 11
  #define CHAMBER2_NAME "DHT11"
  #define CHAMBER2_R25  0.0
  #define CHAMBER2_BETA 0.0
#elif TEMP_SENSOR_CHAMBER2 == 20
  #define CHAMBER2_NAME "AMPLIFIER"
  #define CHAMBER2_R25  0.0
  #define CHAMBER2_BETA 0.0
#elif TEMP_SENSOR_CHAMBER2 == 998 || TEMP_SENSOR_CHAMBER2 == 999
  #define CHAMBER2_NAME "DUMMY SENSOR"
  #define CHAMBER2_R25  0.0
  #define CHAMBER2_BETA 0.0
#endif

#if TEMP_SENSOR_CHAMBER3 == -4
  #define CHAMBER3_NAME "MAX31855"
  #define CHAMBER3_R25  0.0
  #define CHAMBER3_BETA 0.0
#elif TEMP_SENSOR_CHAMBER3 == -3
  #define CHAMBER3_NAME "MAX6675"
  #define CHAMBER3_R25  0.0
  #define CHAMBER3_BETA 0.0
#elif TEMP_SENSOR_CHAMBER3 == -2
  #define CHAMBER3_NAME "AD8495"
  #define CHAMBER3_R25  0.0
  #define CHAMBER3_BETA 0.0
#elif TEMP_SENSOR_CHAMBER3 == -1
  #define CHAMBER3_NAME "AD595"
  #define CHAMBER3_R25  0.0
  #define CHAMBER3_BETA 0.0
#elif TEMP_SENSOR_CHAMBER3 == 11
  #define CHAMBER3_NAME "DHT11"
  #define CHAMBER3_R25  0.0
  #define CHAMBER3_BETA 0.0
#elif TEMP_SENSOR_CHAMBER3 == 20
  #define CHAMBER3_NAME "AMPLIFIER"
  #define CHAMBER3_R25  0.0
  #define CHAMBER3_BETA 0.0
#elif TEMP_SENSOR_CHAMBER3 == 998 || TEMP_SENSOR_CHAMBER3 == 999
  #define CHAMBER3_NAME "DUMMY SENSOR"
  #define CHAMBER3_R25  0.0
  #define CHAMBER3_BETA 0.0
#endif

#if TEMP_SENSOR_COOLER == -4
  #define COOLER_NAME "MAX31855"
  #define COOLER_R25  0.0
  #define COOLER_BETA 0.0
#elif TEMP_SENSOR_COOLER == -3
  #define COOLER_NAME "MAX6675"
  #define COOLER_R25  0.0
  #define COOLER_BETA 0.0
#elif TEMP_SENSOR_COOLER == -2
  #define COOLER_NAME "AD8495"
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
#elif TEMP_SENSOR_COOLER == 0
  #define COOLER_NAME ""
  #define COOLER_R25  0.0
  #define COOLER_BETA 0.0
#endif

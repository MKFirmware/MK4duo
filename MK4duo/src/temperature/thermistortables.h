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

#ifndef THERMISTORTABLES_H_
#define THERMISTORTABLES_H_

#define ANY_THERMISTOR_IS(n) (THERMISTORHEATER_0 == n) || (THERMISTORHEATER_1 == n) || (THERMISTORHEATER_2 == n) || (THERMISTORHEATER_3 == n) || (THERMISTORBED == n) || (THERMISTORCHAMBER == n) || (THERMISTORCOOLER == n)

// Pt1000 and Pt100 handling
//
// Rt=R0*(1+a*T+b*T*T) [for T>0]
// a=3.9083E-3, b=-5.775E-7
#define PtA 3.9083E-3
#define PtB -5.775E-7
#define PtRt(T,R0) ((R0)*(1.0+(PtA)*(T)+(PtB)*(T)*(T)))
#define PtAdVal(T,R0,Rup) (short)(1024/(Rup/PtRt(T,R0)+1))
#define PtLine(T,R0,Rup) { PtAdVal(T,R0,Rup)*OVERSAMPLENR, T },

#if ANY_THERMISTOR_IS(1) // 100k bed thermistor
  #include "thermistortable/thermistortable_1.h"
#endif
#if ANY_THERMISTOR_IS(2) // 200k bed thermistor
  #include "thermistortable/thermistortable_2.h"
#endif
#if ANY_THERMISTOR_IS(3) // mendel-parts
  #include "thermistortable/thermistortable_3.h"
#endif
#if ANY_THERMISTOR_IS(4) // 10k thermistor
  #include "thermistortable/thermistortable_4.h"
#endif
#if ANY_THERMISTOR_IS(5) // 100k ParCan thermistor (104GT-2)
  #include "thermistortable/thermistortable_5.h"
#endif
#if ANY_THERMISTOR_IS(6) // 100k Epcos thermistor
  #include "thermistortable/thermistortable_6.h"
#endif
#if ANY_THERMISTOR_IS(7) // 100k Honeywell 135-104LAG-J01
  #include "thermistortable/thermistortable_7.h"
#endif
#if ANY_THERMISTOR_IS(71) // 100k Honeywell 135-104LAF-J01
  #include "thermistortable/thermistortable_71.h"
#endif
#if ANY_THERMISTOR_IS(8) // 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
  #include "thermistortable/thermistortable_8.h"
#endif
#if ANY_THERMISTOR_IS(9) // 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
  #include "thermistortable/thermistortable_9.h"
#endif
#if ANY_THERMISTOR_IS(10) // 100k RS thermistor 198-961 (4.7k pullup)
  #include "thermistortable/thermistortable_10.h"
#endif
#if ANY_THERMISTOR_IS(11) // QU-BD silicone bed QWG-104F-3950 thermistor
  #include "thermistortable/thermistortable_11.h"
#endif
#if ANY_THERMISTOR_IS(13) // Hisens thermistor B25/50 =3950 +/-1%
  #include "thermistortable/thermistortable_13.h"
#endif
#if ANY_THERMISTOR_IS(20) // PT100 with INA826 amp on Ultimaker v2.0 electronics
  #include "thermistortable/thermistortable_20.h"
#endif
#if ANY_THERMISTOR_IS(51) // 100k EPCOS (WITH 1kohm RESISTOR FOR PULLUP, R9 ON SANGUINOLOLU! NOT FOR 4.7kohm PULLUP! THIS IS NOT NORMAL!)
  #include "thermistortable/thermistortable_51.h"
#endif
#if ANY_THERMISTOR_IS(52) // 200k ATC Semitec 204GT-2 (WITH 1kohm RESISTOR FOR PULLUP, R9 ON SANGUINOLOLU! NOT FOR 4.7kohm PULLUP! THIS IS NOT NORMAL!)
  #include "thermistortable/thermistortable_52.h"
#endif
#if ANY_THERMISTOR_IS(55) // 100k ATC Semitec 104GT-2 (Used on ParCan) (WITH 1kohm RESISTOR FOR PULLUP, R9 ON SANGUINOLOLU! NOT FOR 4.7kohm PULLUP! THIS IS NOT NORMAL!)
  #include "thermistortable/thermistortable_55.h"
#endif
#if ANY_THERMISTOR_IS(60) // Maker's Tool Works Kapton Bed Thermistor
  #include "thermistortable/thermistortable_60.h"
#endif
#if ANY_THERMISTOR_IS(66) // DyzeDesign 500°C Thermistor
  #include "thermistortable/thermistortable_66.h"
#endif
#if ANY_THERMISTOR_IS(12) // 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
  #include "thermistortable/thermistortable_12.h"
#endif
#if ANY_THERMISTOR_IS(70) // bqh2 stock thermistor
  #include "thermistortable/thermistortable_70.h"
#endif
#if ANY_THERMISTOR_IS(110) // Pt100 with 1k0 pullup
  #include "thermistortable/thermistortable_110.h"
#endif
#if ANY_THERMISTOR_IS(147) // Pt100 with 4k7 pullup
  #include "thermistortable/thermistortable_147.h"
#endif
#if ANY_THERMISTOR_IS(1010) // Pt1000 with 1k0 pullup
  #include "thermistortable/thermistortable_1010.h"
#endif
#if ANY_THERMISTOR_IS(1047) // Pt1000 with 4k7 pullup
  #include "thermistortable/thermistortable_1047.h"
#endif
#if ANY_THERMISTOR_IS(998) // User-defined table 1
  #include "thermistortable/thermistortable_998.h"
#endif
#if ANY_THERMISTOR_IS(999) // User-defined table 2
  #include "thermistortable/thermistortable_999.h"
#endif

#define _TT_NAME(_N) temptable_ ## _N
#define TT_NAME(_N) _TT_NAME(_N)

#ifdef THERMISTORHEATER_0
  #define HEATER_0_TEMPTABLE TT_NAME(THERMISTORHEATER_0)
  #define HEATER_0_TEMPTABLE_LEN COUNT(HEATER_0_TEMPTABLE)
#elif ENABLED(HEATER_0_USES_THERMISTOR)
  #error "No heater 0 thermistor table specified"
#else
  #define HEATER_0_TEMPTABLE NULL
  #define HEATER_0_TEMPTABLE_LEN 0
#endif

#ifdef THERMISTORHEATER_1
  #define HEATER_1_TEMPTABLE TT_NAME(THERMISTORHEATER_1)
  #define HEATER_1_TEMPTABLE_LEN COUNT(HEATER_1_TEMPTABLE)
#elif ENABLED(HEATER_1_USES_THERMISTOR)
  #error "No heater 1 thermistor table specified"
#else
  #define HEATER_1_TEMPTABLE NULL
  #define HEATER_1_TEMPTABLE_LEN 0
#endif

#ifdef THERMISTORHEATER_2
  #define HEATER_2_TEMPTABLE TT_NAME(THERMISTORHEATER_2)
  #define HEATER_2_TEMPTABLE_LEN COUNT(HEATER_2_TEMPTABLE)
#elif ENABLED(HEATER_2_USES_THERMISTOR)
  #error "No heater 2 thermistor table specified"
#else
  #define HEATER_2_TEMPTABLE NULL
  #define HEATER_2_TEMPTABLE_LEN 0
#endif

#ifdef THERMISTORHEATER_3
  #define HEATER_3_TEMPTABLE TT_NAME(THERMISTORHEATER_3)
  #define HEATER_3_TEMPTABLE_LEN COUNT(HEATER_3_TEMPTABLE)
#elif ENABLED(HEATER_3_USES_THERMISTOR)
  #error "No heater 3 thermistor table specified"
#else
  #define HEATER_3_TEMPTABLE NULL
  #define HEATER_3_TEMPTABLE_LEN 0
#endif

#ifdef THERMISTORBED
  #define BEDTEMPTABLE TT_NAME(THERMISTORBED)
  #define BEDTEMPTABLE_LEN COUNT(BEDTEMPTABLE)
#elif ENABLED(BED_USES_THERMISTOR)
  #error "No bed thermistor table specified"
#endif

#ifdef THERMISTORCHAMBER
  #define CHAMBERTEMPTABLE TT_NAME(THERMISTORCHAMBER)
  #define CHAMBERTEMPTABLE_LEN COUNT(CHAMBERTEMPTABLE)
#elif ENABLED(CHAMBER_USES_THERMISTOR)
  #error No chamber thermistor table specified
#endif

#ifdef THERMISTORCOOLER
  #define COOLERTEMPTABLE TT_NAME(THERMISTORCOOLER)
  #define COOLERTEMPTABLE_LEN COUNT(COOLERTEMPTABLE)
#elif ENABLED(COOLER_USES_THERMISTOR)
  #error No Cooler thermistor table specified
#endif

// Set the high and low raw values for the heaters
// For thermistors the highest temperature results in the lowest ADC value
// For thermocouples the highest temperature results in the highest ADC value
#ifndef HEATER_0_RAW_HI_TEMP
  #ifdef HEATER_0_USES_THERMISTOR
    #define HEATER_0_RAW_HI_TEMP 0
    #define HEATER_0_RAW_LO_TEMP 16383
  #else
    #define HEATER_0_RAW_HI_TEMP 16383
    #define HEATER_0_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_1_RAW_HI_TEMP
  #ifdef HEATER_1_USES_THERMISTOR
    #define HEATER_1_RAW_HI_TEMP 0
    #define HEATER_1_RAW_LO_TEMP 16383
  #else
    #define HEATER_1_RAW_HI_TEMP 16383
    #define HEATER_1_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_2_RAW_HI_TEMP
  #ifdef HEATER_2_USES_THERMISTOR
    #define HEATER_2_RAW_HI_TEMP 0
    #define HEATER_2_RAW_LO_TEMP 16383
  #else
    #define HEATER_2_RAW_HI_TEMP 16383
    #define HEATER_2_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_3_RAW_HI_TEMP
  #ifdef HEATER_3_USES_THERMISTOR
    #define HEATER_3_RAW_HI_TEMP 0
    #define HEATER_3_RAW_LO_TEMP 16383
  #else
    #define HEATER_3_RAW_HI_TEMP 16383
    #define HEATER_3_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_BED_RAW_HI_TEMP
  #ifdef BED_USES_THERMISTOR
    #define HEATER_BED_RAW_HI_TEMP 0
    #define HEATER_BED_RAW_LO_TEMP 16383
  #else
    #define HEATER_BED_RAW_HI_TEMP 16383
    #define HEATER_BED_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_CHAMBER_RAW_HI_TEMP
  #ifdef CHAMBER_USES_THERMISTOR
    #define HEATER_CHAMBER_RAW_HI_TEMP 0
    #define HEATER_CHAMBER_RAW_LO_TEMP 16383
  #else
    #define HEATER_CHAMBER_RAW_HI_TEMP 16383
    #define HEATER_CHAMBER_RAW_LO_TEMP 0
  #endif
#endif
#ifndef COOLER_RAW_HI_TEMP
  #ifdef COOLER_USES_THERMISTOR
    #define COOLER_RAW_HI_TEMP 0 
    #define COOLER_RAW_LO_TEMP 16383
  #else
    #define COOLER_RAW_HI_TEMP 16383
    #define COOLER_RAW_LO_TEMP 0
  #endif
#endif

#endif // THERMISTORTABLES_H_

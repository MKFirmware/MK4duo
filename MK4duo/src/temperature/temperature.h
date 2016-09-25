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
  temperature.h - temperature controller
  Part of Marlin

  Copyright (c) 2011 Erik van der Zalm

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "thermistortables.h"

#if HOTENDS <= 1
  #define HOTEND_INDEX  0
#else
  #define HOTEND_INDEX  h
#endif

#if ENABLED(PIDTEMPBED) || ENABLED(PIDTEMP)  || ENABLED(PIDTEMPCHAMBER) || ENABLED(PIDTEMPCOOLER)
  #ifdef __SAM3X8E__
    #define PID_dT (((OVERSAMPLENR + 2) * 14.0)/ TEMP_FREQUENCY)
  #else
    #define PID_dT ((OVERSAMPLENR * 12.0)/(F_CPU / 64.0 / 256.0))
  #endif
#endif

// public functions
void tp_init();  //initialize the heating
void manage_temp_controller(); //it is critical that this is called periodically.

#if ENABLED(FILAMENT_SENSOR)
  // For converting raw Filament Width to milimeters
  float analog2widthFil();

  // For converting raw Filament Width to an extrusion ratio
  int widthFil_to_size_ratio();
#endif

#if HAS(POWER_CONSUMPTION_SENSOR)
  // For converting raw Power Consumption to watt
  float analog2voltage();
  float analog2current();
  float analog2power();
  float raw_analog2voltage();
  float analog2error(float current);
  float analog2efficiency(float watt);
#endif

// low level conversion routines
// do not use these routines and variables outside of temperature.cpp
extern int target_temperature[4];
extern float current_temperature[4];
extern int target_temperature_bed;
extern float current_temperature_bed;
extern int target_temperature_chamber;
extern float current_temperature_chamber;
extern int target_temperature_cooler;
extern float current_temperature_cooler;

#if ENABLED(SHOW_TEMP_ADC_VALUES)
  extern int current_temperature_raw[4];
  extern int current_temperature_bed_raw;
  extern int current_temperature_chamber_raw;
  extern int current_temperature_cooler_raw;
#endif

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  extern float redundant_temperature;
#endif

#if HAS(CONTROLLERFAN)
  extern uint8_t soft_pwm_bed;
#endif

#if ENABLED(PIDTEMP)
  extern float Kp[HOTENDS], Ki[HOTENDS], Kd[HOTENDS], Kc[HOTENDS];
  #define PID_PARAM(param, h) param[h] // use macro to point to array value
#endif

#if ENABLED(PIDTEMPBED)
  extern float bedKp, bedKi, bedKd;
#endif

#if ENABLED(PIDTEMPCHAMBER)
  extern float chamberKp, chamberKi, chamberKd;
#endif

#if ENABLED(PIDTEMPCOOLER)
  extern float coolerKp, coolerKi, coolerKd;
#endif

#if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED) || ENABLED(PIDTEMPCHAMBER) || ENABLED(PIDTEMPCOOLER)
  float scalePID_i(float i);
  float scalePID_d(float d);
  float unscalePID_i(float i);
  float unscalePID_d(float d);
#endif

#if ENABLED(BABYSTEPPING)
  extern volatile int babystepsTodo[3];
#endif

//high level conversion routines, for use outside of temperature.cpp
//inline so that there is no performance decrease.
//deg=degreeCelsius
FORCE_INLINE float degHotend(uint8_t h) {
  #if HOTENDS <= 1
    UNUSED(h);
  #endif
  return current_temperature[HOTEND_INDEX];
}

FORCE_INLINE float degBed() { return current_temperature_bed; }
FORCE_INLINE float degChamber() { return current_temperature_chamber; }
FORCE_INLINE float degCooler() { return current_temperature_cooler; }

#if ENABLED(SHOW_TEMP_ADC_VALUES)
  FORCE_INLINE float rawHotendTemp(uint8_t h) {
    #if HOTENDS <= 1
      UNUSED(h);
    #endif
    return current_temperature_raw[HOTEND_INDEX];
  }
  FORCE_INLINE float rawBedTemp() { return current_temperature_bed_raw; }
  FORCE_INLINE float rawChamberTemp() { return current_temperature_chamber_raw; }
  FORCE_INLINE float rawCoolerTemp() { return current_temperature_cooler_raw; }
#endif

FORCE_INLINE float degTargetHotend(uint8_t h) {
  #if HOTENDS <= 1
    UNUSED(h);
  #endif
  return target_temperature[HOTEND_INDEX];
}

FORCE_INLINE float degTargetBed() { return target_temperature_bed; }
FORCE_INLINE float degTargetChamber() { return target_temperature_chamber; }
FORCE_INLINE float degTargetCooler() { return target_temperature_cooler; }

#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  void start_watching_heater(int h = 0);
#endif

#if ENABLED(THERMAL_PROTECTION_BED)
  void start_watching_bed();
#endif

#if ENABLED(THERMAL_PROTECTION_CHAMBER)
  void start_watching_chamber();
#endif

#if ENABLED(THERMAL_PROTECTION_COOLER)
  void start_watching_cooler();
#endif

FORCE_INLINE void setTargetHotend(const float& celsius, uint8_t h) {
  #if HOTENDS <= 1
    UNUSED(h);
  #endif
  target_temperature[HOTEND_INDEX] = celsius;
  #if ENABLED(THERMAL_PROTECTION_HOTENDS)
    start_watching_heater(HOTEND_INDEX);
  #endif
}

FORCE_INLINE void setTargetBed(const float& celsius) {
  target_temperature_bed = celsius;
  #if ENABLED(THERMAL_PROTECTION_BED)
    start_watching_bed();
  #endif
}

FORCE_INLINE void setTargetChamber(const float& celsius) {
  target_temperature_chamber = celsius;
  #if ENABLED(THERMAL_PROTECTION_CHAMBER)
    start_watching_chamber();
  #endif
}

FORCE_INLINE void setTargetCooler(const float& celsius) {
  target_temperature_cooler = celsius;
  #if ENABLED(THERMAL_PROTECTION_COOLER)
    start_watching_cooler();
  #endif
}

FORCE_INLINE bool isHeatingHotend(uint8_t h) {
  #if HOTENDS <= 1
    UNUSED(h);
  #endif
  return target_temperature[HOTEND_INDEX] > current_temperature[HOTEND_INDEX];
}

FORCE_INLINE bool isHeatingBed() { return target_temperature_bed > current_temperature_bed; }
FORCE_INLINE bool isHeatingChamber() { return target_temperature_chamber > current_temperature_chamber; }
FORCE_INLINE bool isHeatingCooler() { return target_temperature_cooler > current_temperature_cooler; } 

FORCE_INLINE bool isCoolingHotend(uint8_t h) {
  #if HOTENDS <= 1
    UNUSED(h);
  #endif
  return target_temperature[HOTEND_INDEX] < current_temperature[HOTEND_INDEX];
}

FORCE_INLINE bool isCoolingBed() { return target_temperature_bed < current_temperature_bed; }
FORCE_INLINE bool isCoolingChamber() { return target_temperature_chamber < current_temperature_chamber; }
FORCE_INLINE bool isCoolingCooler() { return target_temperature_cooler < current_temperature_cooler; } 

#if ENABLED(PREVENT_COLD_EXTRUSION)
  extern float extrude_min_temp;
  extern bool allow_cold_extrude;
  FORCE_INLINE bool tooColdToExtrude(uint8_t h) {
    #if HOTENDS <= 1
      UNUSED(h);
    #endif
    return allow_cold_extrude ? false : degHotend(HOTEND_INDEX) < extrude_min_temp;
  }
#else
  FORCE_INLINE bool tooColdToExtrude(uint8_t h) { UNUSED(h); return false; }
#endif

#define HOTEND_ROUTINES(NR) \
  FORCE_INLINE float degHotend##NR() { return degHotend(NR); } \
  FORCE_INLINE float degTargetHotend##NR() { return degTargetHotend(NR); } \
  FORCE_INLINE void setTargetHotend##NR(const float c) { setTargetHotend(c, NR); } \
  FORCE_INLINE bool isHeatingHotend##NR() { return isHeatingHotend(NR); } \
  FORCE_INLINE bool isCoolingHotend##NR() { return isCoolingHotend(NR); }
HOTEND_ROUTINES(0);
#if HOTENDS > 1
  HOTEND_ROUTINES(1);
#else
  #define setTargetHotend1(c) do{}while(0)
#endif
#if HOTENDS > 2
  HOTEND_ROUTINES(2);
#else
  #define setTargetHotend2(c) do{}while(0)
#endif
#if HOTENDS > 3
  HOTEND_ROUTINES(3);
#else
  #define setTargetHotend3(c) do{}while(0)
#endif

int getHeaterPower(int heater);
int getBedPower();
int getChamberPower();
int getCoolerPower();
uint8_t getPwmCooler(bool soft);

void disable_all_heaters();
void disable_all_coolers();
void updatePID();

#if HAS(PID_HEATING) || HAS(PID_COOLING)
  void PID_autotune(float temp, int temp_controller, int ncycles, bool set_result = false);
#endif

void checkExtruderAutoFans();
extern void autotempShutdown();

#if ENABLED(BABYSTEPPING)
  extern void babystep_axis(AxisEnum axis, int distance)
#endif // BABYSTEPPING

#endif // TEMPERATURE_H

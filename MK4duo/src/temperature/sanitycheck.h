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

#ifndef _TEMPERATURE_SANITYCHECK_H_
#define _TEMPERATURE_SANITYCHECK_H_

// Temperature defines
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


// Temperature features
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

#endif /* _TEMPERATURE_SANITYCHECK_H_ */

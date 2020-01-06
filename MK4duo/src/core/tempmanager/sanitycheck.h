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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

// Temperature defines
#if ENABLED(TEMP_RESIDENCY_TIME)
  #if DISABLED(HOTEND_HYSTERESIS)
    #error "DEPENDENCY ERROR: Missing setting HOTEND_HYSTERESIS."
  #endif
  #if DISABLED(TEMP_WINDOW)
    #error "DEPENDENCY ERROR: Missing setting TEMP_WINDOW."
  #endif
#endif
#if TEMP_SENSOR_HE0 != 0
  #if DISABLED(HOTEND_0_MAXTEMP)
    #error "DEPENDENCY ERROR: Missing setting HOTEND_0_MAXTEMP."
  #endif
  #if DISABLED(HOTEND_0_MINTEMP)
    #error "DEPENDENCY ERROR: Missing setting HOTEND_0_MINTEMP."
  #endif
#endif
#if TEMP_SENSOR_HE1 != 0
  #if DISABLED(HOTEND_1_MAXTEMP)
    #error "DEPENDENCY ERROR: Missing setting HOTEND_1_MAXTEMP."
  #endif
  #if DISABLED(HOTEND_1_MINTEMP)
    #error "DEPENDENCY ERROR: Missing setting HOTEND_1_MINTEMP."
  #endif
#endif
#if TEMP_SENSOR_HE2 != 0
  #if DISABLED(HOTEND_2_MAXTEMP)
    #error "DEPENDENCY ERROR: Missing setting HOTEND_2_MAXTEMP."
  #endif
  #if DISABLED(HOTEND_2_MINTEMP)
    #error "DEPENDENCY ERROR: Missing setting HOTEND_2_MINTEMP."
  #endif
#endif
#if TEMP_SENSOR_HE3 != 0
  #if DISABLED(HOTEND_3_MAXTEMP)
    #error "DEPENDENCY ERROR: Missing setting HOTEND_3_MAXTEMP."
  #endif
  #if DISABLED(HOTEND_3_MINTEMP)
    #error "DEPENDENCY ERROR: Missing setting HOTEND_3_MINTEMP."
  #endif
#endif
#if TEMP_SENSOR_BED0 != 0
  #if DISABLED(BED_MAXTEMP)
    #error "DEPENDENCY ERROR: Missing setting BED_MAXTEMP."
  #endif
  #if DISABLED(BED_MINTEMP)
    #error "DEPENDENCY ERROR: Missing setting BED_MINTEMP."
  #endif
#endif
#if TEMP_SENSOR_CHAMBER0 != 0
  #if DISABLED(CHAMBER_MAXTEMP)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_MAXTEMP."
  #endif
  #if DISABLED(CHAMBER_MINTEMP)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_MINTEMP."
  #endif
  #if !HAS_HEATER_CHAMBER0
    #error "DEPENDENCY ERROR: Cannot enable TEMP_SENSOR_CHAMBER0 without HEATER_CHAMBER0_PIN."
  #endif
#endif
#if TEMP_SENSOR_COOLER != 0
  #if DISABLED(COOLER_MAXTEMP)
    #error "DEPENDENCY ERROR: Missing setting COOLER_MAXTEMP."
  #endif
  #if DISABLED(COOLER_MINTEMP)
    #error "DEPENDENCY ERROR: Missing setting COOLER_MINTEMP."
  #endif
  #if !HAS_HEATER_COOLER
    #error "DEPENDENCY ERROR: Cannot enable TEMP_SENSOR_COOLER without HEATER_COOLER_PIN."
  #endif
#endif
#if DISABLED(PREHEAT_1_TEMP_HOTEND)
  #error "DEPENDENCY ERROR: Missing setting PREHEAT_1_TEMP_HOTEND."
#endif
#if DISABLED(PREHEAT_1_TEMP_BED)
  #error "DEPENDENCY ERROR: Missing setting PREHEAT_1_TEMP_BED."
#endif
#if DISABLED(PREHEAT_1_FAN_SPEED)
  #error "DEPENDENCY ERROR: Missing setting PREHEAT_1_FAN_SPEED."
#endif
#if DISABLED(PREHEAT_2_TEMP_HOTEND)
  #error "DEPENDENCY ERROR: Missing setting PREHEAT_2_TEMP_HOTEND."
#endif
#if DISABLED(PREHEAT_2_TEMP_BED)
  #error "DEPENDENCY ERROR: Missing setting PREHEAT_2_TEMP_BED."
#endif
#if DISABLED(PREHEAT_2_FAN_SPEED)
  #error "DEPENDENCY ERROR: Missing setting PREHEAT_2_FAN_SPEED."
#endif
#if DISABLED(PREHEAT_3_TEMP_HOTEND)
  #error "DEPENDENCY ERROR: Missing setting PREHEAT_3_TEMP_HOTEND."
#endif
#if DISABLED(PREHEAT_3_TEMP_BED)
  #error "DEPENDENCY ERROR: Missing setting PREHEAT_3_TEMP_BED."
#endif
#if DISABLED(PREHEAT_3_FAN_SPEED)
  #error "DEPENDENCY ERROR: Missing setting PREHEAT_3_FAN_SPEED."
#endif

// Temperature features
#if DISABLED(POWER_MAX)
  #error "DEPENDENCY ERROR: Missing setting POWER_MAX."
#endif
#if DISABLED(POWER_DRIVE_MIN)
  #error "DEPENDENCY ERROR: Missing setting POWER_DRIVE_MIN."
#endif
#if DISABLED(POWER_DRIVE_MAX)
  #error "DEPENDENCY ERROR: Missing setting POWER_DRIVE_MAX."
#endif
#if DISABLED(HOTEND_Kp)
  #error "DEPENDENCY ERROR: Missing setting HOTEND_Kp."
#endif
#if DISABLED(HOTEND_Ki)
  #error "DEPENDENCY ERROR: Missing setting HOTEND_Ki."
#endif
#if DISABLED(HOTEND_Kd)
  #error "DEPENDENCY ERROR: Missing setting HOTEND_Kd."
#endif

#if HAS_TEMP_BED0
  #if DISABLED(BED_POWER_MAX)
    #error "DEPENDENCY ERROR: Missing setting BED_POWER_MAX."
  #endif
  #if DISABLED(BED_POWER_DRIVE_MIN)
    #error "DEPENDENCY ERROR: Missing setting BED_POWER_DRIVE_MIN."
  #endif
  #if DISABLED(BED_POWER_DRIVE_MAX)
    #error "DEPENDENCY ERROR: Missing setting BED_POWER_DRIVE_MAX."
  #endif
  #if DISABLED(BED_HYSTERESIS)
    #error "DEPENDENCY ERROR: Missing setting BED_HYSTERESIS."
  #endif
  #if DISABLED(BED_CHECK_INTERVAL)
    #error "DEPENDENCY ERROR: Missing setting BED_CHECK_INTERVAL."
  #endif
#endif
#if (PIDTEMPBED)
  #if !HAS_TEMP_BED0
    #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED0 for use PIDTEMPBED."
  #endif
  #if DISABLED(BED_Kp)
    #error "DEPENDENCY ERROR: Missing setting BED_Kp."
  #endif
  #if DISABLED(BED_Ki)
    #error "DEPENDENCY ERROR: Missing setting BED_Ki."
  #endif
  #if DISABLED(BED_Kd)
    #error "DEPENDENCY ERROR: Missing setting BED_Kd."
  #endif
#endif

#if HAS_TEMP_CHAMBER0
  #if DISABLED(CHAMBER_POWER_MAX)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_POWER_MAX."
  #endif
  #if DISABLED(CHAMBER_POWER_DRIVE_MIN)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_POWER_DRIVE_MIN."
  #endif
  #if DISABLED(CHAMBER_POWER_DRIVE_MAX)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_POWER_DRIVE_MAX."
  #endif
  #if DISABLED(CHAMBER_HYSTERESIS)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_HYSTERESIS."
  #endif
  #if DISABLED(CHAMBER_CHECK_INTERVAL)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_CHECK_INTERVAL."
  #endif
#endif
#if (PIDTEMPCHAMBER)
  #if !HAS_TEMP_CHAMBER0
    #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_CHAMBER0."
  #endif
  #if DISABLED(CHAMBER_Kp)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_Kp."
  #endif
  #if DISABLED(CHAMBER_Ki)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_Ki."
  #endif
  #if DISABLED(CHAMBER_Kd)
    #error "DEPENDENCY ERROR: Missing setting CHAMBER_Kd."
  #endif

#endif

#if HAS_TEMP_COOLER
  #if DISABLED(COOLER_POWER_MAX)
    #error "DEPENDENCY ERROR: Missing setting COOLER_POWER_MAX."
  #endif
  #if DISABLED(COOLER_POWER_DRIVE_MIN)
    #error "DEPENDENCY ERROR: Missing setting COOLER_POWER_DRIVE_MIN."
  #endif
  #if DISABLED(COOLER_POWER_DRIVE_MAX)
    #error "DEPENDENCY ERROR: Missing setting COOLER_POWER_DRIVE_MAX."
  #endif
  #if DISABLED(COOLER_HYSTERESIS)
    #error "DEPENDENCY ERROR: Missing setting COOLER_HYSTERESIS."
  #endif
  #if DISABLED(COOLER_CHECK_INTERVAL)
    #error "DEPENDENCY ERROR: Missing setting COOLER_CHECK_INTERVAL."
  #endif
#endif
#if (PIDTEMPCOOLER)
  #if !HAS_TEMP_COOLER
    #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_COOLER."
  #endif
  #if DISABLED(COOLER_Kp)
    #error "DEPENDENCY ERROR: Missing setting COOLER_Kp."
  #endif
  #if DISABLED(COOLER_Ki)
    #error "DEPENDENCY ERROR: Missing setting COOLER_Ki."
  #endif
  #if DISABLED(COOLER_Kd)
    #error "DEPENDENCY ERROR: Missing setting COOLER_Kd."
  #endif
#endif

#if THERMAL_PROTECTION_HOTENDS || THERMAL_PROTECTION_BED || THERMAL_PROTECTION_CHAMBER || THERMAL_PROTECTION_COOLER
  #if DISABLED(THERMAL_PROTECTION_PERIOD)
    #error "DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_PERIOD."
  #endif
  #if DISABLED(THERMAL_PROTECTION_HYSTERESIS)
    #error "DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_HYSTERESIS."
  #endif
  #if DISABLED(WATCH_HOTEND_PERIOD)
    #error "DEPENDENCY ERROR: Missing setting WATCH_HOTEND_PERIOD."
  #endif
  #if DISABLED(WATCH_HOTEND_INCREASE)
    #error "DEPENDENCY ERROR: Missing setting WATCH_HOTEND_INCREASE."
  #endif
#endif

/**
 * MK4duo supports only one DHT sensor
 */
#if ENABLED(DHT_SENSOR)
  static_assert(1 >= 0
    #if TEMP_SENSOR_HE0 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_HE1 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_HE2 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_HE3 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_HE4 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_HE5 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_BED0 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_BED1 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_BED2 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_BED3 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_CHAMBER0 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_CHAMBER1 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_CHAMBER2 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_CHAMBER3 == DHT_TYPE
      + 1
    #endif
    #if TEMP_SENSOR_COOLER == DHT_TYPE
      + 1
    #endif
    , "DEPENDENCY ERROR: only one DHT sensor is supported!"
  );
#endif

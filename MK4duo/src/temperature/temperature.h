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
 * temperature.h - temperature controller
 */

#ifndef _TEMPERATURE_H_
#define _TEMPERATURE_H_

#include "thermistortables.h"

class Temperature {

  public: /** Constructor */

    Temperature() {};

  public: /** Public Parameters */

    static volatile bool wait_for_heatup;

    #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
      static float    mcu_current_temperature,
                      mcu_highest_temperature,
                      mcu_lowest_temperature,
                      mcu_alarm_temperature;
      static int16_t  mcu_current_temperature_raw;
    #endif

    #if ENABLED(ADC_KEYPAD)
      static int16_t  current_ADCKey_raw;
    #endif

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static float    redundant_temperature;
    #endif

    #if (PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
        static int lpq_len;
    #endif

    #if HAS_EXTRUDERS && ENABLED(PREVENT_COLD_EXTRUSION)
      static bool allow_cold_extrude;
      static int16_t extrude_min_temp;
      static bool tooColdToExtrude(uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        return allow_cold_extrude ? false : heaters[HOTEND_INDEX].current_temperature < extrude_min_temp;
      }
    #else
      static bool tooColdToExtrude(uint8_t h) { UNUSED(h); return false; }
    #endif

    #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
      static uint8_t auto_report_temp_interval;
      static millis_t next_temp_report_ms;
    #endif

  private: /** Private Parameters */

    static float  temp_iState[HEATER_COUNT],
                  temp_dState[HEATER_COUNT][4],
                  temp_iState_min[HEATER_COUNT],
                  temp_iState_max[HEATER_COUNT],
                  pid_error[HEATER_COUNT];

    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      static float  cTerm[HOTENDS];
      static long   last_e_position,
                    lpq[LPQ_MAX_LEN];
      static int    lpq_ptr;
    #endif

    static uint8_t pid_pointer[HEATER_COUNT];

    static millis_t next_check_ms[HEATER_COUNT];

    #if ENABLED(FILAMENT_SENSOR)
      static int8_t   meas_shift_index;     // Index of a delayed sample in buffer
      static uint16_t current_raw_filwidth; // Measured filament diameter - one extruder only
    #endif

    #if HAS_AUTO_FAN
      static millis_t next_auto_fan_check_ms;
    #endif

    #if ENABLED(PROBING_HEATERS_OFF)
      static bool paused;
    #endif

    #if HEATER_IDLE_HANDLER
      static millis_t heater_idle_timeout_ms[HEATER_COUNT];
      static bool heater_idle_timeout_exceeded[HEATER_COUNT];
    #endif

  public: /** Public Function */

    void init();

    /**
     * Static (class) methods
     */
    static void wait_heater(const uint8_t h, bool no_wait_for_cooling=true);

    /**
     * Called from the Temperature ISR
     */
    static void set_current_temp_raw();

    /**
     * Call periodically to manage temp controller
     */
    static void manage_temp_controller();

    /**
     * Perform auto-tuning for hotend, bed, chamber or cooler in response to M303
     */
    static void PID_autotune(int8_t temp_controller, const float temp, int ncycles, bool storeValues=false);

    /**
     * Update the temp manager when PID values change
     */
    static void updatePID();

    /**
     * Switch off all heaters, set all target temperatures to 0
     */
    static void disable_all_heaters();

    #if WATCH_THE_HEATER
      static void start_watching(Heater *act);
    #endif

    #if HAS_FILAMENT_SENSOR
      static int widthFil_to_size_ratio(); // Convert raw Filament Width to an extrusion ratio
    #endif    

    #if ENABLED(PROBING_HEATERS_OFF)
      static void pause(const bool p);
      static bool is_paused() { return paused; }
    #endif

    #if HEATER_IDLE_HANDLER
      static void start_heater_idle_timer(const uint8_t h, const millis_t timeout_ms) {
        heater_idle_timeout_ms[h] = millis() + timeout_ms;
        heater_idle_timeout_exceeded[h] = false;
      }

      static void reset_heater_idle_timer(const uint8_t h) {
        heater_idle_timeout_ms[h] = 0;
        heater_idle_timeout_exceeded[h] = false;
        #if WATCH_THE_HOTEND
          if (h < HOTENDS) start_watching(&heaters[h]);
        #endif
        #if WATCH_THE_BED
          if (h == BED_INDEX) start_watching(&heaters[BED_INDEX]);
        #endif
      }

      static bool is_heater_idle(uint8_t h) {
        return heater_idle_timeout_exceeded[h];
      }
    #endif

    #if ENABLED(AUTO_REPORT_TEMPERATURES)
      static void auto_report_temperatures();
    #endif

    static void print_heaterstates();

  private:

    static void updateTemperaturesFromRawValues();

    static float analog2temp(const uint8_t h);

    #if HAS_FILAMENT_SENSOR
      static float analog2widthFil(); // Convert raw Filament Width to millimeters
    #endif

    #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
      static float analog2tempMCU(const int raw);
    #endif

    static uint8_t get_pid_output(const int8_t h);

    #if ENABLED(SUPPORT_MAX6675)
      static int16_t read_max6675(const Pin cs_pin, const int8_t h);
    #endif

    #if ENABLED(SUPPORT_MAX31855)
      static int16_t read_max31855(const Pin cs_pin);
    #endif

    static void checkExtruderAutoFans();

    static void _temp_error(const int8_t tc, const char * const serial_msg, const char * const lcd_msg);
    static void min_temp_error(const int8_t h);
    static void max_temp_error(const int8_t h);

    #if HAS_THERMALLY_PROTECTED_HEATER

      typedef enum TRState { TRInactive, TRFirstHeating, TRStable, TRRunaway } TRstate;

      static void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float target_temperature, uint8_t temp_controller_id, int period_seconds, int hysteresis_degc);

      static TRState thermal_runaway_state_machine[HEATER_COUNT];
      static millis_t thermal_runaway_timer[HEATER_COUNT];

    #endif // HAS_THERMALLY_PROTECTED_HEATER

    #if HEATER_COUNT > 0
      static void print_heater_state(const float &c, const int16_t &t,
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          const int16_t r,
        #endif
        const int8_t h
      );
    #endif

};

extern Temperature thermalManager;

#endif /* _TEMPERATURE_H_ */

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

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "thermistortables.h"

#if HOTENDS <= 1
  #define HOTEND_INDEX  0
  #define EXTRUDER_IDX  0
#else
  #define HOTEND_INDEX  h
  #define EXTRUDER_IDX  tools.active_extruder
#endif

class Temperature {

  public:

    static volatile bool  wait_for_heatup;

    #if HAS_TEMP_HOTEND
      static float      current_temperature[HOTENDS];
      static int16_t    current_temperature_raw[HOTENDS],
                        target_temperature[HOTENDS];
      static uint8_t    soft_pwm[HOTENDS];
    #endif
    
    #if HAS_TEMP_BED
      static float    current_temperature_bed;
      static int16_t  current_temperature_bed_raw,
                      target_temperature_bed;
      static uint8_t  soft_pwm_bed;
    #endif

    #if HAS_TEMP_CHAMBER
      static float    current_temperature_chamber;
      static int16_t  target_temperature_chamber,
                      current_temperature_chamber_raw;
      static uint8_t  soft_pwm_chamber;
    #endif

    #if HAS_TEMP_COOLER
      static float    current_temperature_cooler;
      static int16_t  target_temperature_cooler,
                      current_temperature_cooler_raw;
      static uint8_t  soft_pwm_cooler;
    #endif

    #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
      static float    current_temperature_mcu,
                      highest_temperature_mcu,
                      lowest_temperature_mcu,
                      alarm_temperature_mcu;
      static int16_t  current_temperature_mcu_raw;
    #endif

    #if ENABLED(ADC_KEYPAD)
      static int16_t  current_ADCKey_raw;
    #endif

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static float    redundant_temperature;
    #endif

    #if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED) || ENABLED(PIDTEMPCHAMBER) || ENABLED(PIDTEMPCOOLER)

      static float Kp[HOTENDS], Ki[HOTENDS], Kd[HOTENDS], Kc[HOTENDS];
      #define PID_PARAM(param, h) Temperature::param[h]

    #endif

    #if ENABLED(PIDTEMPBED)
      static float bedKp, bedKi, bedKd;
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      static float chamberKp, chamberKi, chamberKd;
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      static float coolerKp, coolerKi, coolerKd;
    #endif

    #if HAS(AUTO_FAN)
      static uint8_t autoFanSpeeds[HOTENDS];
    #endif

    #if ENABLED(BABYSTEPPING)
      static volatile int babystepsTodo[3];
    #endif

    #if WATCH_HOTENDS
      static uint16_t watch_target_temp[HOTENDS];
      static millis_t watch_heater_next_ms[HOTENDS];
    #endif

    #if WATCH_THE_BED
      static uint16_t watch_target_bed_temp;
      static millis_t watch_bed_next_ms;
    #endif

    #if WATCH_THE_CHAMBER
      static uint16_t watch_target_temp_chamber = 0;
      static millis_t watch_chamber_next_ms = 0;
    #endif

    #if WATCH_THE_COOLER
      static uint16_t watch_target_temp_cooler = 0;
      static millis_t watch_cooler_next_ms = 0;
    #endif

    #if HAS_EXTRUDERS && ENABLED(PREVENT_COLD_EXTRUSION)
      static bool allow_cold_extrude;
      static int16_t extrude_min_temp;
      static bool tooColdToExtrude(uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        return allow_cold_extrude ? false : degHotend(HOTEND_INDEX) < extrude_min_temp;
      }
    #else
      static bool tooColdToExtrude(uint8_t h) { UNUSED(h); return false; }
    #endif

    #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
      static uint8_t auto_report_temp_interval;
      static millis_t next_temp_report_ms;
    #endif

    #if HEATER_USES_AD595
      static float  ad595_offset[HOTENDS],
                    ad595_gain[HOTENDS];
    #endif

  private:

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static int redundant_temperature_raw;
      static float redundant_temperature;
    #endif

    #if ENABLED(PIDTEMP)
      static float  temp_iState[HOTENDS],
                    temp_dState[HOTENDS][4],
                    temp_iState_min[HOTENDS],
                    temp_iState_max[HOTENDS],
                    pid_error[HOTENDS];

      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        static float  cTerm[HOTENDS];
        static long   last_e_position,
                      lpq[LPQ_MAX_LEN];
        static int    lpq_ptr,
                      lpq_len;
      #endif

      static uint8_t pid_pointer[HOTENDS];
    #endif

    #if ENABLED(PIDTEMPBED)
      static float  temp_iState_bed,
                    temp_dState_bed[4],
                    temp_iState_bed_min,
                    temp_iState_bed_max,
                    pid_error_bed;

      static uint8_t pid_pointer_bed;
    #else
      static millis_t next_bed_check_ms;
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      static float  temp_iState_chamber,
                    temp_dState_chamber[4],
                    temp_iState_chamber_min,
                    temp_iState_chamber_max,
                    pid_error_chamber;

      static uint8_t pid_pointer_chamber;
    #else
      static millis_t next_chamber_check_ms;
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      static float  temp_iState_cooler,
                    temp_dState_cooler[4],
                    temp_iState_cooler_min,
                    temp_iState_cooler_max,
                    pid_error_cooler;

      static uint8_t pid_pointer_cooler;
    #else
      static millis_t next_cooler_check_ms;
    #endif

    // Init min and max temp with extreme values to prevent false errors during startup
    static int16_t  minttemp_raw[HOTENDS],
                    maxttemp_raw[HOTENDS],
                    minttemp[HOTENDS],
                    maxttemp[HOTENDS];

    #if ENABLED(MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
      static uint8_t consecutive_low_temperature_error[HOTENDS];
      #if HAS_TEMP_BED
        static uint8_t consecutive_bed_low_temperature_error;
      #endif
    #endif

    #if ENABLED(MILLISECONDS_PREHEAT_TIME)
      static millis_t preheat_end_time[HOTENDS];
    #endif

    #if ENABLED(BED_MINTEMP)
      static int16_t bed_minttemp_raw;
    #endif

    #if ENABLED(BED_MAXTEMP)
      static int16_t bed_maxttemp_raw;
    #endif

    #if ENABLED(CHAMBER_MINTEMP)
      static int16_t chamber_minttemp_raw;
    #endif

    #if ENABLED(CHAMBER_MAXTEMP)
      static int16_t chamber_maxttemp_raw;
    #endif

    #if ENABLED(COOLER_MINTEMP)
      static int16_t cooler_minttemp_raw;
    #endif

    #if ENABLED(COOLER_MAXTEMP)
      static int16_t cooler_maxttemp_raw;
    #endif

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
      static millis_t heater_idle_timeout_ms[HOTENDS];
      static bool heater_idle_timeout_exceeded[HOTENDS];
      #if HAS_TEMP_BED
        static millis_t bed_idle_timeout_ms;
        static bool bed_idle_timeout_exceeded;
      #endif
    #endif

  public:

    /**
     * Instance Methods
     */

    Temperature();

    void init();

    /**
     * Static (class) methods
     */
    #if HAS_TEMP_HOTEND
      static float analog2temp(const int raw, uint8_t h);
      static void wait_heater(bool no_wait_for_cooling=true);
    #endif
    #if HAS_TEMP_BED
      static float analog2tempBed(const int raw);
      static void wait_bed(bool no_wait_for_cooling=true);
    #endif
    #if HAS_TEMP_CHAMBER
      static float analog2tempChamber(const int raw);
      static void wait_chamber(bool no_wait_for_heating=true);
    #endif
    #if HAS_TEMP_COOLER
      static float analog2tempCooler(const int raw);
      static void wait_cooler(bool no_wait_for_heating=true);
    #endif
    #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
      static float analog2tempMCU(const int raw);
    #endif

    /**
     * Called from the Temperature ISR
     */
    static void set_current_temp_raw();

    /**
     * Call periodically to manage temp controller
     */
    static void manage_temp_controller();

    /**
     * Preheating hotends
     */
    #if HAS_TEMP_HOTEND
      #if ENABLED(MILLISECONDS_PREHEAT_TIME)
        static bool is_preheating(uint8_t h) {
          #if HOTENDS <= 1
            UNUSED(h);
          #endif
          return preheat_end_time[HOTEND_INDEX] && PENDING(millis(), preheat_end_time[HOTEND_INDEX]);
        }
        static void start_preheat_time(uint8_t h) {
          #if HOTENDS <= 1
            UNUSED(h);
          #endif
          preheat_end_time[HOTEND_INDEX] = millis() + MILLISECONDS_PREHEAT_TIME;
        }
        static void reset_preheat_time(uint8_t h) {
          #if HOTENDS <= 1
            UNUSED(h);
          #endif
          preheat_end_time[HOTEND_INDEX] = 0;
        }
      #else
        #define is_preheating(n) (false)
      #endif
    #endif

    #if HAS_FILAMENT_SENSOR
      static float analog2widthFil(); // Convert raw Filament Width to millimeters
      static int widthFil_to_size_ratio(); // Convert raw Filament Width to an extrusion ratio
    #endif

    // high level conversion routines, for use outside of temperature.cpp
    // inline so that there is no performance decrease.
    // deg=degreeCelsius

    #if HAS_TEMP_HOTEND
      static float degHotend(uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        return current_temperature[HOTEND_INDEX];
      }
    #else
      static float degHotend(uint8_t h) {
        UNUSED(h);
        return 0.0;
      }
    #endif
    #if HAS_TEMP_BED
      static float degBed() { return current_temperature_bed; }
    #else
      static float degBed() { return 0.0; }
    #endif
    #if HAS_TEMP_CHAMBER
      static float degChamber() { return current_temperature_chamber; }
    #endif
    #if HAS_TEMP_COOLER
      static float degCooler() { return current_temperature_cooler; }
    #endif

    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      #if HAS_TEMP_HOTEND
        static int16_t rawHotendTemp(uint8_t h) {
          #if HOTENDS <= 1
            UNUSED(h);
          #endif
          return current_temperature_raw[HOTEND_INDEX];
        }
      #endif
      #if HAS_TEMP_BED
        static int16_t rawBedTemp() { return current_temperature_bed_raw; }
      #endif
      #if HAS_TEMP_CHAMBER
        static int16_t rawChamberTemp() { return current_temperature_chamber_raw; }
      #endif
      #if HAS_TEMP_COOLER
        static int16_t rawCoolerTemp() { return current_temperature_cooler_raw; }
      #endif
      #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
        static float rawMCUTemp() { return current_temperature_mcu_raw; }
      #endif
    #endif

    #if HAS_TEMP_HOTEND
      static int16_t degTargetHotend(uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        return target_temperature[HOTEND_INDEX];
      }
    #else
      static int16_t degTargetHotend(uint8_t h) {
        UNUSED(h);
        return 0;
      }
    #endif
    #if HAS_TEMP_BED
      static int16_t degTargetBed() { return target_temperature_bed; }
    #else
      static int16_t degTargetBed() { return 0; }
    #endif
    #if HAS_TEMP_CHAMBER
      static int16_t degTargetChamber() { return target_temperature_chamber; }
    #endif
    #if HAS_TEMP_COOLER
      static int16_t degTargetCooler() { return target_temperature_cooler; }
    #endif

    #if WATCH_HOTENDS
      static void start_watching_heater(uint8_t h = 0);
    #endif
    #if WATCH_THE_BED
      static void start_watching_bed();
    #endif
    #if WATCH_THE_CHAMBER
      void start_watching_chamber();
    #endif
    #if WATCH_THE_COOLER
      void start_watching_cooler();
    #endif

    #if HAS_TEMP_HOTEND
      static void setTargetHotend(const int16_t celsius, uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        #if ENABLED(MILLISECONDS_PREHEAT_TIME)
          if (celsius == 0)
            reset_preheat_time(HOTEND_INDEX);
          else if (target_temperature[HOTEND_INDEX] == 0)
            start_preheat_time(HOTEND_INDEX);
        #endif
        target_temperature[HOTEND_INDEX] = celsius;
        #if WATCH_HOTENDS
          start_watching_heater(HOTEND_INDEX);
        #endif
      }
    #endif
    #if HAS_TEMP_BED
      static void setTargetBed(const int16_t celsius) {
        #if ENABLED(BED_MAXTEMP)
          target_temperature_bed = min(celsius, BED_MAXTEMP);
        #else
          target_temperature_bed = celsius;
        #endif
        #if WATCH_THE_BED
          start_watching_bed();
        #endif
      }
    #endif
    #if HAS_TEMP_CHAMBER
      static void setTargetChamber(const int16_t celsius) {
        #if ENABLED(CHAMBER_MAXTEMP)
          target_temperature_chamber = min(celsius, CHAMBER_MAXTEMP);
        #else
          target_temperature_chamber = celsius;
        #endif
        #if WATCH_THE_CHAMBER
          start_watching_chamber();
        #endif
      }
    #endif
    #if HAS_TEMP_COOLER
      static void setTargetCooler(const int16_t celsius) {
        #if ENABLED(COOLER_MAXTEMP)
          target_temperature_cooler = min(celsius, COOLER_MAXTEMP);
        #else
          target_temperature_cooler = celsius;
        #endif
        #if WATCH_THE_COOLER
          start_watching_cooler();
        #endif
      }
    #endif

    #if HAS_TEMP_HOTEND
      static bool isHeatingHotend(uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        return target_temperature[HOTEND_INDEX] > current_temperature[HOTEND_INDEX];
      }
    #else
      static bool isHeatingHotend(uint8_t h) {
        UNUSED(h);
        return false;
      }
    #endif
    #if HAS_TEMP_BED
      static bool isHeatingBed() { return target_temperature_bed > current_temperature_bed; }
    #else
      static bool isHeatingBed() { return false; }
    #endif
    #if HAS_TEMP_CHAMBER
      static bool isHeatingChamber() { return target_temperature_chamber > current_temperature_chamber; }
    #endif
    #if HAS_TEMP_COOLER
      static bool isHeatingCooler() { return target_temperature_cooler > current_temperature_cooler; } 
    #endif

    #if HAS_TEMP_HOTEND
      static bool isCoolingHotend(uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        return target_temperature[HOTEND_INDEX] < current_temperature[HOTEND_INDEX];
      }
    #endif
    #if HAS_TEMP_BED
      static bool isCoolingBed() { return target_temperature_bed < current_temperature_bed; }
    #endif
    #if HAS_TEMP_CHAMBER
      static bool isCoolingChamber() { return target_temperature_chamber < current_temperature_chamber; }
    #endif
    #if HAS_TEMP_COOLER
      static bool isCoolingCooler() { return target_temperature_cooler < current_temperature_cooler; } 
    #endif

    /**
     * The software PWM power
     */
    #if HAS_TEMP_HOTEND
      static int getHeaterPower(int h) { return soft_pwm[h]; }
    #endif
    #if HAS_TEMP_BED
      static int getBedPower() { return soft_pwm_bed; }
    #endif
    #if HAS_TEMP_CHAMBER
      static int getChamberPower() { return soft_pwm_chamber; }
    #endif
    #if HAS_TEMP_COOLER
      static int getCoolerPower() {
        #if ENABLED(FAST_PWM_COOLER)
          return fast_pwm_cooler;
        #else
          return soft_pwm_cooler;
        #endif
      }
      static uint8_t getPwmCooler(bool soft);
    #endif

    /**
     * Switch off all heaters, set all target temperatures to 0
     */
    static void disable_all_heaters();

    /**
     * Switch off all cooler, set all target temperatures to 0
     */
    #if HAS_TEMP_COOLER
      static void disable_all_coolers();
    #else
      inline void disable_all_coolers() {}
    #endif

    /**
     * Perform auto-tuning for hotend, bed, chamber or cooler in response to M303
     */
    #if HAS_PID_HEATING || HAS_PID_COOLING
      static void PID_autotune(const float temp, const int temp_controller, int ncycles, bool storeValues=false);
    #endif

    /**
     * Update the temp manager when PID values change
     */
    static void updatePID();

    #if ENABLED(BABYSTEPPING)

      static void babystep_axis(const AxisEnum axis, const int distance) {
        if (mechanics.axis_known_position[axis]) {
          #if IS_CORE
            #if ENABLED(BABYSTEP_XY)
              switch (axis) {
                case CORE_AXIS_1: // X on CoreXY and CoreXZ, Y on CoreYZ
                  babystepsTodo[CORE_AXIS_1] += distance * 2;
                  babystepsTodo[CORE_AXIS_2] += distance * 2;
                  break;
                case CORE_AXIS_2: // Y on CoreXY, Z on CoreXZ and CoreYZ
                  babystepsTodo[CORE_AXIS_1] += CORESIGN(distance * 2);
                  babystepsTodo[CORE_AXIS_2] -= CORESIGN(distance * 2);
                  break;
                case NORMAL_AXIS: // Z on CoreXY, Y on CoreXZ, X on CoreYZ
                  babystepsTodo[NORMAL_AXIS] += distance;
                  break;
              }
            #elif CORE_IS_XZ || CORE_IS_YZ
              // Only Z stepping needs to be handled here
              babystepsTodo[CORE_AXIS_1] += CORESIGN(distance * 2);
              babystepsTodo[CORE_AXIS_2] -= CORESIGN(distance * 2);
            #else
              babystepsTodo[Z_AXIS] += distance;
            #endif
          #else
            babystepsTodo[axis] += distance;
          #endif
        }
      }

    #endif // BABYSTEPPING

    #if ENABLED(PROBING_HEATERS_OFF)
      static void pause(const bool p);
      static bool is_paused() { return paused; }
    #endif

    #if HEATER_IDLE_HANDLER
      static void start_heater_idle_timer(uint8_t h, millis_t timeout_ms) {
        #if HOTENDS == 1
          UNUSED(h);
        #endif
        heater_idle_timeout_ms[HOTEND_INDEX] = millis() + timeout_ms;
        heater_idle_timeout_exceeded[HOTEND_INDEX] = false;
      }

      static void reset_heater_idle_timer(uint8_t h) {
        #if HOTENDS == 1
          UNUSED(h);
        #endif
        heater_idle_timeout_ms[HOTEND_INDEX] = 0;
        heater_idle_timeout_exceeded[HOTEND_INDEX] = false;
        #if WATCH_HOTENDS
          start_watching_heater(HOTEND_INDEX);
        #endif
      }

      static bool is_heater_idle(uint8_t h) {
        #if HOTENDS == 1
          UNUSED(h);
        #endif
        return heater_idle_timeout_exceeded[HOTEND_INDEX];
      }

      #if HAS_TEMP_BED
        static void start_bed_idle_timer(millis_t timeout_ms) {
          bed_idle_timeout_ms = millis() + timeout_ms;
          bed_idle_timeout_exceeded = false;
        }

        static void reset_bed_idle_timer() {
          bed_idle_timeout_ms = 0;
          bed_idle_timeout_exceeded = false;
          #if WATCH_THE_BED
            start_watching_bed();
          #endif
        }

        static bool is_bed_idle() {
          return bed_idle_timeout_exceeded;
        }
      #endif
    #endif

    #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
      static void auto_report_temperatures();
    #endif

    #if HAS_TEMP_HOTEND || HAS_TEMP_BED
      static void print_heaterstates();
    #endif

    #if HAS_TEMP_CHAMBER
      static void print_chamberstate();
    #endif

    #if HAS_TEMP_COOLER
      static void print_coolerstate();
    #endif

    #if ENABLED(ARDUINO_ARCH_SAM)&& !MB(RADDS)
      static void print_MCUstate();
    #endif

  private:

    static void updateTemperaturesFromRawValues();

    #if ENABLED(HEATER_0_USES_MAX6675)
      static int read_max6675();
    #endif

    static void checkExtruderAutoFans();

    static uint8_t get_pid_output(const int8_t h);

    #if ENABLED(PIDTEMPBED)
      static uint8_t get_pid_output_bed();
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      static uint8_t get_pid_output_chamber();
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      static uint8_t get_pid_output_cooler();
    #endif

    static void _temp_error(const int8_t tc, const char * const serial_msg, const char * const lcd_msg);
    static void min_temp_error(const int8_t h);
    static void max_temp_error(const int8_t h);

    #if ENABLED(THERMAL_PROTECTION_HOTENDS) || ENABLED(THERMAL_PROTECTION_BED) || ENABLED(THERMAL_PROTECTION_COOLER)

      typedef enum TRState { TRInactive, TRFirstHeating, TRStable, TRRunaway } TRstate;

      static void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float target_temperature, int temp_controller_id, int period_seconds, int hysteresis_degc);

      #if ENABLED(THERMAL_PROTECTION_HOTENDS)
        static TRState thermal_runaway_state_machine[HOTENDS];
        static millis_t thermal_runaway_timer[HOTENDS];
      #endif

      #if HAS_THERMALLY_PROTECTED_BED
        static TRState thermal_runaway_bed_state_machine;
        static millis_t thermal_runaway_bed_timer;
      #endif

      #if ENABLED(THERMAL_PROTECTION_CHAMBER) && HAS_TEMP_CHAMBER
        static TRState thermal_runaway_chamber_state_machine = TRReset;
        static millis_t thermal_runaway_chamber_timer;
      #endif

      #if ENABLED(THERMAL_PROTECTION_COOLER) && HAS_TEMP_COOLER
        static TRState thermal_runaway_cooler_state_machine = TRReset;
        static millis_t thermal_runaway_cooler_timer;
      #endif

    #endif // THERMAL_PROTECTION

    #if HAS_TEMP_HOTEND || HAS_TEMP_BED
      static void print_heater_state(const float &c, const int16_t &t,
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          const int16_t r,
        #endif
        const int8_t e=-2
      );
    #endif

};

extern Temperature thermalManager;

#endif // TEMPERATURE_H

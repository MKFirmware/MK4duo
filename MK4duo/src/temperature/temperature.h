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
 * temperature.h - temperature controller
 */

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "thermistortables.h"

#if HOTENDS <= 1
  #define HOTEND_LOOP() const int8_t h = 0;
  #define HOTEND_INDEX  0
  #define EXTRUDER_IDX  0
#else
  #define HOTEND_LOOP() for (int8_t h = 0; h < HOTENDS; h++)
  #define HOTEND_INDEX  h
  #define EXTRUDER_IDX  active_extruder
#endif

class Temperature {

  public:

    static float current_temperature[HOTENDS],
                 current_temperature_bed;
    static int   current_temperature_raw[HOTENDS],
                 target_temperature[HOTENDS],
                 current_temperature_bed_raw,
                 target_temperature_bed;

    #if HAS(TEMP_CHAMBER)
      static float current_temperature_chamber = 0.0;
      static int   target_temperature_chamber = 0,
                   current_temperature_chamber_raw = 0;
    #endif

    #if HAS(TEMP_COOLER)
      static float current_temperature_cooler = 0.0;
      static int   target_temperature_cooler = 0,
                   current_temperature_cooler_raw = 0;
    #endif

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static float redundant_temperature;
    #endif

    static uint8_t soft_pwm_bed;

    #if ENABLED(FAN_SOFT_PWM)
      static uint8_t fanSpeedSoftPwm;
    #endif

    #if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED) || ENABLED(PIDTEMPCHAMBER) || ENABLED(PIDTEMPCOOLER)
      #if ENABLED(ARDUINO_ARCH_SAM)
        #define PID_dT (((OVERSAMPLENR + 2) * 14.0) / TEMP_TIMER_FREQUENCY)
      #else
        #define PID_dT ((OVERSAMPLENR * 12.0) / TEMP_TIMER_FREQUENCY)
      #endif
    #endif

    #if ENABLED(PIDTEMP)
      static float Kp[HOTENDS], Ki[HOTENDS], Kd[HOTENDS], Kc[HOTENDS];
      #define PID_PARAM(param, h) Temperature::param[h]

      // Apply the scale factors to the PID values
      #define scalePID_i(i)   ( (i) * PID_dT )
      #define unscalePID_i(i) ( (i) / PID_dT )
      #define scalePID_d(d)   ( (d) / PID_dT )
      #define unscalePID_d(d) ( (d) * PID_dT )

    #endif

    #if ENABLED(PIDTEMPBED)
      static float bedKp, bedKi, bedKd;
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      extern float chamberKp, chamberKi, chamberKd;
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      extern float coolerKp, coolerKi, coolerKd;
    #endif

    #if ENABLED(BABYSTEPPING)
      static volatile int babystepsTodo[3];
    #endif

    #if ENABLED(THERMAL_PROTECTION_HOTENDS) && WATCH_TEMP_PERIOD > 0
      static int watch_target_temp[HOTENDS];
      static millis_t watch_heater_next_ms[HOTENDS];
    #endif

    #if ENABLED(THERMAL_PROTECTION_BED) && WATCH_BED_TEMP_PERIOD > 0
      static int watch_target_bed_temp;
      static millis_t watch_bed_next_ms;
    #endif

    #if ENABLED(THERMAL_PROTECTION_CHAMBER) && WATCH_CHAMBER_TEMP_PERIOD > 0
      int watch_target_temp_chamber = 0;
      millis_t watch_chamber_next_ms = 0;
    #endif

    #if ENABLED(THERMAL_PROTECTION_COOLER) && WATCH_COOLER_TEMP_PERIOD > 0
      int watch_target_temp_cooler = 0;
      millis_t watch_cooler_next_ms = 0;
    #endif

    #if ENABLED(PREVENT_COLD_EXTRUSION)
      static bool allow_cold_extrude;
      static float extrude_min_temp;
      static bool tooColdToExtrude(uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        return allow_cold_extrude ? false : degHotend(HOTEND_INDEX) < extrude_min_temp;
      }
    #else
      static bool tooColdToExtrude(uint8_t h) { UNUSED(h); return false; }
    #endif

  private:

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static int redundant_temperature_raw;
      static float redundant_temperature;
    #endif

    static volatile bool temp_meas_ready;

    #if ENABLED(PIDTEMP)
      static float temp_iState[HOTENDS],
                   temp_dState[HOTENDS],
                   pTerm[HOTENDS],
                   iTerm[HOTENDS],
                   dTerm[HOTENDS];

      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        static float cTerm[HOTENDS];
        static long last_e_position;
        static long lpq[LPQ_MAX_LEN];
        static int lpq_ptr;
      #endif

      static float pid_error[HOTENDS];
      static bool pid_reset[HOTENDS];
    #endif

    #if ENABLED(PIDTEMPBED)
      static float temp_iState_bed,
                   temp_dState_bed,
                   pTerm_bed,
                   iTerm_bed,
                   dTerm_bed,
                   pid_error_bed;
    #else
      static millis_t next_bed_check_ms;
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      static float temp_iState_chamber,
                   temp_dState_chamber,
                   pTerm_chamber,
                   iTerm_chamber,
                   dTerm_chamber,
                   pid_error_chamber;
    #else
      static millis_t next_chamber_check_ms;
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      static float temp_iState_cooler,
                   temp_dState_cooler,
                   pTerm_cooler,
                   iTerm_cooler,
                   dTerm_cooler,
                   pid_error_cooler;
    #else
      static millis_t next_cooler_check_ms;
    #endif

    static unsigned long raw_temp_value[4],
                         raw_temp_bed_value,
                         raw_temp_chamber_value,
                         raw_temp_cooler_value;

    // Init min and max temp with extreme values to prevent false errors during startup
    static int minttemp_raw[HOTENDS],
               maxttemp_raw[HOTENDS],
               minttemp[HOTENDS],
               maxttemp[HOTENDS];

    #if ENABLED(MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
      static int consecutive_low_temperature_error[HOTENDS];
    #endif

    #if ENABLED(MILLISECONDS_PREHEAT_TIME)
      static unsigned long preheat_end_time[HOTENDS];
    #endif

    #if ENABLED(BED_MINTEMP)
      static int bed_minttemp_raw;
    #endif

    #if ENABLED(BED_MAXTEMP)
      static int bed_maxttemp_raw;
    #endif

    #if ENABLED(CHAMBER_MINTEMP)
      static int chamber_minttemp_raw;
    #endif

    #if ENABLED(CHAMBER_MAXTEMP)
      static int chamber_maxttemp_raw;
    #endif

    #if ENABLED(COOLER_MINTEMP)
      static int cooler_minttemp_raw;
    #endif

    #if ENABLED(COOLER_MAXTEMP)
      static int cooler_maxttemp_raw;
    #endif

    #if ENABLED(ARDUINO_ARCH_SAM)
      // MEDIAN COUNT
      // For Smoother temperature
      // ONLY FOR DUE
      #define MEDIAN_COUNT 10

      static int max_temp[7],
                 min_temp[7];
      static uint8_t median_counter;
      static unsigned long raw_median_temp[7][MEDIAN_COUNT], sum;
    #endif

    #if ENABLED(FILAMENT_SENSOR)
      static int meas_shift_index;  // Index of a delayed sample in buffer
    #endif

    #if HAS(AUTO_FAN)
      static millis_t next_auto_fan_check_ms;
    #endif

    static uint8_t soft_pwm[HOTENDS];

    #if ENABLED(FAN_SOFT_PWM)
      static uint8_t soft_pwm_fan;
    #endif

    #if ENABLED(FILAMENT_SENSOR)
      static int current_raw_filwidth;  //Holds measured filament diameter - one extruder only
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
    static float analog2temp(int raw, uint8_t h);
    static float analog2tempBed(int raw);
    #if HAS(TEMP_CHAMBER)
      static float analog2tempChamber(int raw);
    #endif
    #if HAS(TEMP_COOLER)
      static float analog2tempCooler(int raw);
    #endif

    /**
     * Called from the Temperature ISR
     */
    static void isr();

    /**
     * Call periodically to manage temp controller
     */
    static void manage_temp_controller();

    /**
     * Preheating hotends
     */
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

    #if HAS(FILAMENT_SENSOR)
      static float analog2widthFil(); // Convert raw Filament Width to millimeters
      static int widthFil_to_size_ratio(); // Convert raw Filament Width to an extrusion ratio
    #endif

    #if HAS(POWER_CONSUMPTION_SENSOR)
      // For converting raw Power Consumption to watt
      static float analog2voltage(),
                   analog2current(),
                   analog2power(),
                   raw_analog2voltage(),
                   analog2error(float current),
                   analog2efficiency(float watt);
    #endif

    // high level conversion routines, for use outside of temperature.cpp
    // inline so that there is no performance decrease.
    // deg=degreeCelsius

    static float degHotend(uint8_t h) {
      #if HOTENDS <= 1
        UNUSED(h);
      #endif
      return current_temperature[HOTEND_INDEX];
    }
    static float degBed() { return current_temperature_bed; }
    #if HAS(TEMP_CHAMBER)
      static float degChamber() { return current_temperature_chamber; }
    #endif
    #if HAS(TEMP_COOLER)
      static float degCooler() { return current_temperature_cooler; }
    #endif

    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      static float rawHotendTemp(uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        return current_temperature_raw[HOTEND_INDEX];
      }
      static float rawBedTemp() { return current_temperature_bed_raw; }
      #if HAS(TEMP_CHAMBER)
        static float rawChamberTemp() { return current_temperature_chamber_raw; }
      #endif
      #if HAS(TEMP_COOLER)
        static float rawCoolerTemp() { return current_temperature_cooler_raw; }
      #endif
    #endif

    static float degTargetHotend(uint8_t h) {
      #if HOTENDS <= 1
        UNUSED(h);
      #endif
      return target_temperature[HOTEND_INDEX];
    }
    static float degTargetBed() { return target_temperature_bed; }
    #if HAS(TEMP_CHAMBER)
      static float degTargetChamber() { return target_temperature_chamber; }
    #endif
    #if HAS(TEMP_COOLER)
      static float degTargetCooler() { return target_temperature_cooler; }
    #endif

    #if ENABLED(THERMAL_PROTECTION_HOTENDS) && WATCH_TEMP_PERIOD > 0
      static void start_watching_heater(uint8_t h = 0);
    #endif

    #if ENABLED(THERMAL_PROTECTION_BED) && WATCH_BED_TEMP_PERIOD > 0
      static void start_watching_bed();
    #endif

    #if ENABLED(THERMAL_PROTECTION_CHAMBER) && WATCH_CHAMBER_TEMP_PERIOD > 0
      void start_watching_chamber();
    #endif

    #if ENABLED(THERMAL_PROTECTION_COOLER) && WATCH_COOLER_TEMP_PERIOD > 0
      void start_watching_cooler();
    #endif

    static void setTargetHotend(const float& celsius, uint8_t h) {
      #if HOTENDS <= 1
        UNUSED(h);
      #endif
      #if ENABLED(MILLISECONDS_PREHEAT_TIME)
        if (celsius == 0.0f)
          reset_preheat_time(HOTEND_INDEX);
        else if (target_temperature[HOTEND_INDEX] == 0.0f)
          start_preheat_time(HOTEND_INDEX);
      #endif
      target_temperature[HOTEND_INDEX] = celsius;
      #if ENABLED(THERMAL_PROTECTION_HOTENDS) && WATCH_TEMP_PERIOD > 0
        start_watching_heater(HOTEND_INDEX);
      #endif
    }

    static void setTargetBed(const float& celsius) {
      target_temperature_bed = celsius;
      #if ENABLED(THERMAL_PROTECTION_BED) && WATCH_BED_TEMP_PERIOD > 0
        start_watching_bed();
      #endif
    }

    #if HAS(TEMP_CHAMBER)
      static void setTargetChamber(const float& celsius) {
        target_temperature_chamber = celsius;
        #if ENABLED(THERMAL_PROTECTION_CHAMBER) && WATCH_CHAMBER_TEMP_PERIOD > 0
          start_watching_chamber();
        #endif
      }
    #endif

    #if HAS(TEMP_COOLER)
      static void setTargetCooler(const float& celsius) {
        target_temperature_cooler = celsius;
        #if ENABLED(THERMAL_PROTECTION_COOLER) && WATCH_COOLER_TEMP_PERIOD > 0
          start_watching_cooler();
        #endif
      }
    #endif

    static bool isHeatingHotend(uint8_t h) {
      #if HOTENDS <= 1
        UNUSED(h);
      #endif
      return target_temperature[HOTEND_INDEX] > current_temperature[HOTEND_INDEX];
    }
    static bool isHeatingBed() { return target_temperature_bed > current_temperature_bed; }
    #if HAS(TEMP_CHAMBER)
      static bool isHeatingChamber() { return target_temperature_chamber > current_temperature_chamber; }
    #endif
    #if HAS(TEMP_COOLER)
      static bool isHeatingCooler() { return target_temperature_cooler > current_temperature_cooler; } 
    #endif

    static bool isCoolingHotend(uint8_t h) {
      #if HOTENDS <= 1
        UNUSED(h);
      #endif
      return target_temperature[HOTEND_INDEX] < current_temperature[HOTEND_INDEX];
    }
    static bool isCoolingBed() { return target_temperature_bed < current_temperature_bed; }
    #if HAS(TEMP_CHAMBER)
      static bool isCoolingChamber() { return target_temperature_chamber < current_temperature_chamber; }
    #endif
    #if HAS(TEMP_COOLER)
      static bool isCoolingCooler() { return target_temperature_cooler < current_temperature_cooler; } 
    #endif

    /**
     * The software PWM power for a hotends
     */
    static int getHeaterPower(int h) { return soft_pwm[h]; }
    
    /**
     * The software PWM power for a bed
     */
    static int getBedPower() { return soft_pwm_bed; }

    #if HAS(TEMP_CHAMBER)
      static int getChamberPower() { return soft_pwm_chamber; }
    #endif

    #if HAS(TEMP_COOLER)
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
    #if HAS(TEMP_COOLER)
      static void disable_all_coolers();
    #else
      inline void disable_all_coolers() {}
    #endif

    /**
     * Perform auto-tuning for hotend, bed, chamber or cooler in response to M303
     */
    #if HAS(PID_HEATING) || HAS(PID_COOLING)
      static void PID_autotune(float temp, int temp_controller, int ncycles, bool storeValues=false);
    #endif

    /**
     * Update the temp manager when PID values change
     */
    static void updatePID();

    #if ENABLED(AUTOTEMP)
      static void autotempShutdown() {
        if (planner.autotemp_enabled) {
          planner.autotemp_enabled = false;
          if (degTargetHotend(EXTRUDER_IDX) > planner.autotemp_min)
            setTargetHotend(0, EXTRUDER_IDX);
        }
      }
    #endif

    #if ENABLED(BABYSTEPPING)

      static void babystep_axis(const AxisEnum axis, const int distance) {
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

    #endif // BABYSTEPPING

  private:

    #if ENABLED(ARDUINO_ARCH_SAM)
      static int calc_raw_temp_value(uint8_t temp_id);
      #if HAS(TEMP_BED)
        static int calc_raw_temp_bed_value();
      #endif
      #if HAS(TEMP_CHAMBER)
        static int calc_raw_temp_chamber_value();
      #endif
      #if HAS(TEMP_COOLER)
        static int calc_raw_temp_cooler_value();
      #endif
    #endif
    static void set_current_temp_raw();

    static void updateTemperaturesFromRawValues();

    #if ENABLED(HEATER_0_USES_MAX6675)
      static int read_max6675();
    #endif

    static void checkExtruderAutoFans();

    static float get_pid_output(int h);

    #if ENABLED(PIDTEMPBED)
      static float get_pid_output_bed();
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      float get_pid_output_chamber();
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      float get_pid_output_cooler();
    #endif

    static void _temp_error(int tc, const char* serial_msg, const char* lcd_msg);
    static void min_temp_error(int8_t h);
    static void max_temp_error(int8_t h);

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

      #if ENABLED(THERMAL_PROTECTION_CHAMBER) && HAS(TEMP_CHAMBER)
        static TRState thermal_runaway_chamber_state_machine = TRReset;
        static millis_t thermal_runaway_chamber_timer;
      #endif

      #if ENABLED(THERMAL_PROTECTION_COOLER) && HAS(TEMP_COOLER)
        static TRState thermal_runaway_cooler_state_machine = TRReset;
        static millis_t thermal_runaway_cooler_timer;
      #endif

    #endif // THERMAL_PROTECTION

    #if HAS(POWER_CONSUMPTION_SENSOR)
      int current_raw_powconsumption;
      static unsigned long raw_powconsumption_value;
    #endif
};

extern Temperature thermalManager;

#endif // TEMPERATURE_H

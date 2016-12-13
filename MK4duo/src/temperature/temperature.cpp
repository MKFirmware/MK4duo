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
 * temperature.cpp - temperature control
 */

#include "../../base.h"

#if ENABLED(K1) // Defined in Configuration_base.h in the PID settings
  #define K2 (1.0 - K1)
#endif

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  static void* heater_ttbl_map[2] = {(void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE };
  static uint8_t heater_ttbllen_map[2] = { HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN };
#else
  static void* heater_ttbl_map[HOTENDS] = ARRAY_BY_HOTENDS_N((void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE, (void*)HEATER_2_TEMPTABLE, (void*)HEATER_3_TEMPTABLE);
  static uint8_t heater_ttbllen_map[HOTENDS] = ARRAY_BY_HOTENDS_N(HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN, HEATER_3_TEMPTABLE_LEN);
#endif

Temperature thermalManager;

// public:

float Temperature::current_temperature[HOTENDS] = { 0.0 },
      Temperature::current_temperature_bed = 0.0;
int   Temperature::current_temperature_raw[HOTENDS] = { 0 },
      Temperature::target_temperature[HOTENDS] = { 0 },
      Temperature::current_temperature_bed_raw = 0,
      Temperature::target_temperature_bed = 0;

#if HAS(TEMP_CHAMBER)
  float Temperature::current_temperature_chamber = 0.0;
  int   Temperature::target_temperature_chamber = 0,
        Temperature::current_temperature_chamber_raw = 0;
#endif

#if HAS(TEMP_COOLER)
  float Temperature::current_temperature_cooler = 0.0;
  int   Temperature::target_temperature_cooler = 0,
        Temperature::current_temperature_cooler_raw = 0;
#endif

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  float Temperature::redundant_temperature = 0.0;
#endif

uint8_t Temperature::soft_pwm_bed;

#if HAS(TEMP_CHAMBER)
  uint8_t Temperature::soft_pwm_chamber;
#endif
#if HAS(TEMP_COOLER)
  uint8_t Temperature::soft_pwm_cooler;
#endif

#if ENABLED(FAN_SOFT_PWM)
  uint8_t Temperature::fanSpeedSoftPwm;
#endif

#if ENABLED(PIDTEMP)
  float Temperature::Kp[HOTENDS],
        Temperature::Ki[HOTENDS],
        Temperature::Kd[HOTENDS],
        Temperature::Kc[HOTENDS];
#endif

#if ENABLED(PIDTEMPBED)
  float Temperature::bedKp = DEFAULT_bedKp,
        Temperature::bedKi = ((DEFAULT_bedKi) * PID_dT),
        Temperature::bedKd = ((DEFAULT_bedKd) / PID_dT);
#endif

#if ENABLED(PIDTEMPCHAMBER)
  float Temperature::chamberKp = DEFAULT_chamberKp,
        Temperature::chamberKi = ((DEFAULT_chamberKi) * (PID_dT)),
        Temperature::chamberKd = ((DEFAULT_chamberKd) / (PID_dT));
#endif

#if ENABLED(PIDTEMPCOOLER)
  float Temperature::coolerKp = DEFAULT_coolerKp,
        Temperature::coolerKi = ((DEFAULT_coolerKi) * (PID_dT)),
        Temperature::coolerKd = ((DEFAULT_coolerKd) / (PID_dT));
#endif

#if ENABLED(BABYSTEPPING)
  volatile int Temperature::babystepsTodo[XYZ] = { 0 };
#endif

#if ENABLED(THERMAL_PROTECTION_HOTENDS) && WATCH_TEMP_PERIOD > 0
  int Temperature::watch_target_temp[HOTENDS] = { 0 };
  millis_t Temperature::watch_heater_next_ms[HOTENDS] = { 0 };
#endif

#if ENABLED(THERMAL_PROTECTION_BED) && WATCH_BED_TEMP_PERIOD > 0
  int Temperature::watch_target_bed_temp = 0;
  millis_t Temperature::watch_bed_next_ms = 0;
#endif

#if ENABLED(THERMAL_PROTECTION_CHAMBER) && WATCH_CHAMBER_TEMP_PERIOD > 0
  int Temperature::watch_target_temp_chamber = 0;
  millis_t Temperature::watch_chamber_next_ms = 0;
#endif

#if ENABLED(THERMAL_PROTECTION_COOLER) && WATCH_COOLER_TEMP_PERIOD > 0
  int Temperature::watch_target_temp_cooler = 0;
  millis_t Temperature::watch_cooler_next_ms = 0;
#endif

#if ENABLED(PREVENT_COLD_EXTRUSION)
  bool Temperature::allow_cold_extrude = false;
  float Temperature::extrude_min_temp = EXTRUDE_MINTEMP;
#endif

// private:

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  int Temperature::redundant_temperature_raw = 0;
  float Temperature::redundant_temperature = 0.0;
#endif

volatile bool Temperature::temp_meas_ready = false;

#if ENABLED(PIDTEMP)
  float Temperature::temp_iState[HOTENDS] = { 0 },
        Temperature::temp_dState[HOTENDS] = { 0 },
        Temperature::pTerm[HOTENDS],
        Temperature::iTerm[HOTENDS],
        Temperature::dTerm[HOTENDS];

  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    float Temperature::cTerm[HOTENDS];
    long Temperature::last_e_position;
    long Temperature::lpq[LPQ_MAX_LEN];
    int Temperature::lpq_ptr = 0;
  #endif

  float Temperature::pid_error[HOTENDS];
  bool Temperature::pid_reset[HOTENDS];
#endif

#if ENABLED(PIDTEMPBED)
  float Temperature::temp_iState_bed = { 0 },
        Temperature::temp_dState_bed = { 0 },
        Temperature::pTerm_bed,
        Temperature::iTerm_bed,
        Temperature::dTerm_bed,
        Temperature::pid_error_bed;
#else
  millis_t Temperature::next_bed_check_ms;
#endif

#if ENABLED(PIDTEMPCHAMBER)
  float Temperature::temp_iState_chamber = { 0 },
        Temperature::temp_dState_chamber = { 0 },
        Temperature::pTerm_chamber,
        Temperature::iTerm_chamber,
        Temperature::dTerm_chamber,
        Temperature::pid_error_chamber;
#else
  millis_t Temperature::next_chamber_check_ms;
#endif

#if ENABLED(PIDTEMPCOOLER)
  float Temperature::temp_iState_cooler = { 0 },
        Temperature::temp_dState_cooler = { 0 },
        Temperature::pTerm_cooler,
        Temperature::iTerm_cooler,
        Temperature::dTerm_cooler,
        Temperature::pid_error_cooler;
#else
  millis_t Temperature::next_cooler_check_ms;
#endif

unsigned long Temperature::raw_temp_value[4] = { 0 };
unsigned long Temperature::raw_temp_bed_value = 0;
#if HAS(TEMP_CHAMBER)
  unsigned long Temperature::raw_temp_chamber_value = 0;
#endif
#if HAS(TEMP_COOLER)
  unsigned long Temperature::raw_temp_cooler_value = 0;
#endif

// Init min and max temp with extreme values to prevent false errors during startup
int Temperature::minttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS_N(HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP, HEATER_3_RAW_LO_TEMP),
    Temperature::maxttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS_N(HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP, HEATER_3_RAW_HI_TEMP),
    Temperature::minttemp[HOTENDS] = { 0 },
    Temperature::maxttemp[HOTENDS] = ARRAY_BY_HOTENDS(16383);

#if ENABLED(MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
  int Temperature::consecutive_low_temperature_error[HOTENDS] = { 0 };
#endif

#if ENABLED(MILLISECONDS_PREHEAT_TIME)
  unsigned long Temperature::preheat_end_time[HOTENDS] = { 0 };
#endif

#if ENABLED(BED_MINTEMP)
  int Temperature::bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP;
#endif

#if ENABLED(BED_MAXTEMP)
  int Temperature::bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif

#if ENABLED(CHAMBER_MINTEMP)
  int Temperature::chamber_minttemp_raw = HEATER_CHAMBER_RAW_LO_TEMP;
#endif
#if ENABLED(CHAMBER_MAXTEMP)
  int Temperature::chamber_maxttemp_raw = HEATER_CHAMBER_RAW_HI_TEMP;
#endif
#if ENABLED(COOLER_MINTEMP)
  int Temperature::cooler_minttemp_raw = COOLER_RAW_LO_TEMP;
#endif
#if ENABLED(COOLER_MAXTEMP)
  int Temperature::cooler_maxttemp_raw = COOLER_RAW_HI_TEMP;
#endif

#if ENABLED(__SAM3X8E__)
  #define CORRECTION_FOR_RAW_TEMP 4
  #define RAW_MIN_TEMP_DEFAULT 123000
  #define RAW_MEDIAN_TEMP_DEFAULT 3600 * OVERSAMPLENR

  int Temperature::max_temp[7],
      Temperature::min_temp[7];
  uint8_t Temperature::median_counter;
  unsigned long Temperature::raw_median_temp[7][MEDIAN_COUNT],
                Temperature::sum;
#endif

#if ENABLED(FILAMENT_SENSOR)
  int Temperature::meas_shift_index;  // Index of a delayed sample in buffer
#endif

#if HAS(POWER_CONSUMPTION_SENSOR)
  int Temperature::current_raw_powconsumption = 0;  // Holds measured power consumption
  static unsigned long Temperature::raw_powconsumption_value = 0;
#endif

#if HAS(AUTO_FAN)
  millis_t Temperature::next_auto_fan_check_ms = 0;
#endif

uint8_t Temperature::soft_pwm[HOTENDS];

#if ENABLED(FAN_SOFT_PWM)
  uint8_t Temperature::soft_pwm_fan;
#endif

#if ENABLED(FILAMENT_SENSOR)
  int Temperature::current_raw_filwidth = 0;  //Holds measured filament diameter - one extruder only
#endif

#if HAS(PID_HEATING) || HAS(PID_COOLING)

  void Temperature::PID_autotune(float temp, int temp_controller, int ncycles, bool storeValues/*=false*/) {
    float currentTemp = 0.0;
    int cycles = 0;
    bool heating = true;

    millis_t temp_ms = millis(), t1 = temp_ms, t2 = temp_ms;
    long t_high = 0, t_low = 0;

    long bias, d;
    float Ku, Tu;
    float workKp = 0, workKi = 0, workKd = 0;
    float maxTemp = 0, minTemp = 10000;

    #if HAS(AUTO_FAN)
      next_auto_fan_check_ms = temp_ms + 2500UL;
    #endif

    if (temp_controller >=
      #if ENABLED(PIDTEMP)
        HOTENDS
      #else
        0
      #endif
      #if HASNT(TEMP_BED) && HASNT(TEMP_COOLER)
        || temp_controller < 0
      #elif HASNT(TEMP_BED)
        || (temp_controller == -1 && temp_controller < -3)
      #elif HASNT(TEMP_COOLER)
        || temp_controller < -2 
      #endif
    ) {
      SERIAL_LM(ER, MSG_PID_BAD_TEMP_CONTROLLER_NUM);
      return;
    }

    SERIAL_EM(MSG_PID_AUTOTUNE_START);
    if (temp_controller == -1) {
      SERIAL_M("BED");
    }
    else if(temp_controller == -2) {
      SERIAL_M("CHAMBER");
    }
    else if(temp_controller == -3) {
      SERIAL_M("COOLER");
    }
    else {
      SERIAL_MV("Hotend: ", temp_controller);
    }
    SERIAL_MV(" Temp: ", temp);
    SERIAL_MV(" Cycles: ", ncycles);
    if (storeValues)
      SERIAL_EM(" Apply result");
    else
      SERIAL_E;

    disable_all_heaters(); // switch off all heaters.
    #if HAS(TEMP_COOLER)
      disable_all_coolers(); // switch off all coolers.
    #endif

    #if HAS(TEMP_BED)
      if (temp_controller == -1) {
        bias = d = (MAX_BED_POWER) >> 1;
        soft_pwm_bed = (MAX_BED_POWER);
      }
    #endif
    #if HAS(TEMP_CHAMBER)
      if (temp_controller == -2) {
        bias = d = (MAX_CHAMBER_POWER) >> 1;
        soft_pwm_chamber = (MAX_CHAMBER_POWER);
      }
    #endif
    #if HAS(TEMP_COOLER)
      if (temp_controller == -3) {
        bias = d = (MAX_COOLER_POWER) >> 1;
        setPwmCooler(MAX_COOLER_POWER);
      }
    #endif

    if (temp_controller >= 0) {
      bias = d = (PID_MAX) >> 1;
      soft_pwm[temp_controller] = (PID_MAX);
    }

    wait_for_heatup = true;

    // PID Tuning loop
    while (wait_for_heatup) {

      millis_t ms = millis();

      if (temp_meas_ready) { // temp sample ready
        updateTemperaturesFromRawValues();

        #if HAS(TEMP_BED)
          if (temp_controller == -1)
            currentTemp = current_temperature_bed;
        #endif
        #if HAS(TEMP_CHAMBER)
          if (temp_controller == -2)
            currentTemp = current_temperature_chamber;
        #endif
        #if HAS(TEMP_COOLER)
          if (temp_controller == -3)
          currentTemp = current_temperature_cooler;
        #endif
        if (temp_controller >= 0)
          currentTemp = current_temperature[temp_controller];

        NOLESS(maxTemp, currentTemp);
        NOMORE(minTemp, currentTemp);

        #if HAS(AUTO_FAN)
          if (ELAPSED(ms, next_auto_fan_check_ms)) {
            checkExtruderAutoFans();
            next_auto_fan_check_ms = ms + 2500UL;
          }
        #endif

        if (heating && currentTemp > temp) {
          if (ELAPSED(ms, t2 + 5000UL)) {
            heating = false;

            #if HAS(TEMP_BED)
              if (temp_controller == -1)
                soft_pwm_bed = (bias - d);
            #endif
            #if HAS(TEMP_CHAMBER)
              if (temp_controller == -2)
                soft_pwm_chamber = (bias - d);
            #endif
            #if HAS(TEMP_COOLER)
              if (temp_controller == -3)
                setPwmCooler(bias - d);
            #endif
            if (temp_controller >= 0)
              soft_pwm[temp_controller] = (bias - d);

            t1 = ms;
            t_high = t1 - t2;

            #if HAS(TEMP_COOLER)
              if (temp_controller == -3)
                minTemp = temp;
              else
            #endif
              maxTemp = temp;
          }
        }

        if (!heating && currentTemp < temp) {
          if (ELAPSED(ms, t1 + 5000UL)) {
            heating = true;
            t2 = ms;
            t_low = t2 - t1;
            if (cycles > 0) {
              long max_pow;

              #if HAS(TEMP_BED)
                if (temp_controller == -1)
                  max_pow = MAX_BED_POWER;
              #endif
              #if HAS(TEMP_CHAMBER)
                if (temp_controller == -2)
                  max_pow = MAX_CHAMBER_POWER;
              #endif
              #if HAS(TEMP_COOLER)
                if (temp_controller == -3)
                  max_pow = MAX_COOLER_POWER;
              #endif
              if (temp_controller >= 0)
                max_pow = PID_MAX;

              bias += (d * (t_high - t_low)) / (t_low + t_high);
              bias = constrain(bias, 20, max_pow - 20);
              d = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;

              SERIAL_MV(MSG_BIAS, bias);
              SERIAL_MV(MSG_D, d);
              SERIAL_MV(MSG_T_MIN, minTemp);
              SERIAL_MV(MSG_T_MAX, maxTemp);
              if (cycles > 2) {
                Ku = (4.0 * d) / (M_PI * (maxTemp - minTemp));
                Tu = ((float)(t_low + t_high) * 0.001);
                SERIAL_MV(MSG_KU, Ku);
                SERIAL_EMV(MSG_TU, Tu);
                workKp = 0.6 * Ku;
                workKi = 2 * workKp / Tu;
                workKd = workKp * Tu * 0.125;
                
                SERIAL_EM(MSG_CLASSIC_PID);
                SERIAL_MV(MSG_KP, workKp);
                SERIAL_MV(MSG_KI, workKi);
                SERIAL_EMV(MSG_KD, workKd);
              }
            }

            #if ENABLED(PIDTEMP)
              if (temp_controller >= 0)
                soft_pwm[temp_controller] = (bias + d);
            #endif

            #if ENABLED(PIDTEMPBED)
              if (temp_controller == -1)
                soft_pwm_bed = (bias + d);
            #endif

            #if ENABLED(PIDTEMPCHAMBER)
              if (temp_controller == -2)
                soft_pwm_chamber = (bias + d);
            #endif

            #if ENABLED(PIDTEMPCOOLER)
              if (temp_controller == -3)
                setPwmCooler((bias + d));
            #endif

            cycles++;

            #if HAS(TEMP_COOLER)
              if (temp_controller == -3)
                maxTemp = temp;
              else
            #endif
              minTemp = temp;
          }
        }
      }
      #define MAX_OVERSHOOT_PID_AUTOTUNE 20
      if (currentTemp > temp + MAX_OVERSHOOT_PID_AUTOTUNE && temp_controller >= -2) {
        SERIAL_LM(ER, MSG_PID_TEMP_TOO_HIGH);
        return;
      }
      else if (currentTemp < temp + MAX_OVERSHOOT_PID_AUTOTUNE && temp_controller < -2) {
		    SERIAL_LM(ER, MSG_PID_TEMP_TOO_LOW);
		    return;
		  }
      // Every 1 seconds...
      if (ELAPSED(ms, temp_ms + 1000UL)) {
        #if HAS_TEMP_HOTEND || HAS_TEMP_BED
          print_heaterstates();
          SERIAL_E;
        #endif
        #if HAS(TEMP_CHAMBER)
          print_chamberstate();
          SERIAL_E;
        #endif
        #if HAS(TEMP_COOLER)
          print_coolerstate();
          SERIAL_E;
        #endif

        temp_ms = ms;
      }

      // Over 2 minutes?
      if (((ms - t1) + (ms - t2)) > (10L * 60L * 1000L * 2L)) {
        SERIAL_EM(MSG_PID_TIMEOUT);
        return;
      }
      if (cycles > ncycles) {
        SERIAL_EM(MSG_PID_AUTOTUNE_FINISHED);

        #if ENABLED(PIDTEMP)
          if (temp_controller >= 0) {
            SERIAL_MV(MSG_KP, workKp);
            SERIAL_MV(MSG_KI, workKi);
            SERIAL_EMV(MSG_KD, workKd);
            if (storeValues) {
              PID_PARAM(Kp, temp_controller) = workKp;
              PID_PARAM(Ki, temp_controller) = scalePID_i(workKi);
              PID_PARAM(Kd, temp_controller) = scalePID_d(workKd);
              updatePID();
            }
          }
        #endif

        #if ENABLED(PIDTEMPBED)
          if (temp_controller == -1) {
            SERIAL_EMV("#define DEFAULT_bedKp ", workKp);
            SERIAL_EMV("#define DEFAULT_bedKi ", workKi);
            SERIAL_EMV("#define DEFAULT_bedKd ", workKd);
            if (storeValues) {
              bedKp = workKp;
              bedKi = scalePID_i(workKi);
              bedKd = scalePID_d(workKd);
              updatePID();
            }
          }
        #endif

        #if ENABLED(PIDTEMPCOOLER)
          if (temp_controller == -3) {
            SERIAL_EMV("#define DEFAULT_coolerKp ", workKp);
            SERIAL_EMV("#define DEFAULT_coolerKi ", workKi);
            SERIAL_EMV("#define DEFAULT_coolerKd ", workKd);
            if (storeValues) {
              coolerKp = workKp;
              coolerKi = scalePID_i(workKi);
              coolerKd = scalePID_d(workKd);
              updatePID();
            }
          }
        #endif

        return;
      }
      lcd_update();
    }

    disable_all_heaters();

    #if HAS(TEMP_COOLER)
      disable_all_coolers();
    #endif
  }

#endif // HAS_PID_HEATING

/**
 * Class and Instance Methods
 */

Temperature::Temperature() { }

void Temperature::updatePID() {
  #if ENABLED(PIDTEMP)
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      last_e_position = 0;
    #endif
  #endif
}

#if HAS(AUTO_FAN)

  void Temperature::checkExtruderAutoFans() {
    const int8_t fanPin[] = { H0_AUTO_FAN_PIN, H1_AUTO_FAN_PIN, H2_AUTO_FAN_PIN, H3_AUTO_FAN_PIN };
    const int fanBit[] = {
                    0,
      AUTO_1_IS_0 ? 0 :               1,
      AUTO_2_IS_0 ? 0 : AUTO_2_IS_1 ? 1 :               2,
      AUTO_3_IS_0 ? 0 : AUTO_3_IS_1 ? 1 : AUTO_3_IS_2 ? 2 : 3
    };
    uint8_t fanState = 0;
 
    HOTEND_LOOP() {
      if (current_temperature[h] > HOTEND_AUTO_FAN_TEMPERATURE)
        SBI(fanState, fanBit[h]);
    }
 
    uint8_t fanDone = 0;
    for (uint8_t f = 0; f < COUNT(fanPin); f++) {
      int8_t pin = fanPin[f];
      if (pin >= 0 && !TEST(fanDone, fanBit[f])) {
        uint8_t newFanSpeed = TEST(fanState, fanBit[f]) ? HOTEND_AUTO_FAN_SPEED : 0;
        // this idiom allows both digital and PWM fan outputs (see M42 handling).
        digitalWrite(pin, newFanSpeed);
        analogWrite(pin, newFanSpeed);
        SBI(fanDone, fanBit[f]);
      }
    }
  }

#endif // HAS_AUTO_FAN

//
// Temperature Error Handlers
//
void Temperature::_temp_error(int tc, const char* serial_msg, const char* lcd_msg) {
  static bool killed = false;
  if (IsRunning()) {
    SERIAL_ST(ER, serial_msg);
    if (tc >= 0) {
      SERIAL_M(MSG_STOPPED_HEATER);
      SERIAL_EV((int)tc);
    }
    else if (tc == -1) {
      SERIAL_EM(MSG_STOPPED_BED);
    }
    else if (tc == -2) {
      SERIAL_EM(MSG_STOPPED_CHAMBER);
    }
    else
      SERIAL_EM(MSG_STOPPED_COOLER);
  }

  #if DISABLED(BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE)
    if (!killed) {
      Running = false;
      killed = true;
      kill(lcd_msg);
    }
    else {
      disable_all_heaters();
      #if HAS(TEMP_COOLER)
        disable_all_coolers();
      #endif
    }
  #endif
}

void Temperature::max_temp_error(int8_t h) {
  #if HAS(TEMP_BED)
    _temp_error(h, PSTR(MSG_T_MAXTEMP), h >= 0 ? PSTR(MSG_ERR_MAXTEMP) : PSTR(MSG_ERR_MAXTEMP_BED));
  #else
    _temp_error(HOTEND_INDEX, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP));
    #if HOTENDS <= 1
      UNUSED(h);
    #endif
  #endif
}
void Temperature::min_temp_error(int8_t h) {
  #if HAS(TEMP_BED)
    _temp_error(h, PSTR(MSG_T_MINTEMP), h >= 0 ? PSTR(MSG_ERR_MINTEMP) : PSTR(MSG_ERR_MINTEMP_BED));
  #else
    _temp_error(HOTEND_INDEX, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP));
    #if HOTENDS <= 1
      UNUSED(h);
    #endif
  #endif
}

float Temperature::get_pid_output(int h) {
  #if HOTENDS <= 1
    UNUSED(h);
    #define _HOTEND_TEST  true
  #else
    #define _HOTEND_TEST  h == active_extruder
  #endif

  float pid_output, current_temp;

  switch (HOTEND_INDEX) {
    case 0:
      current_temp = current_temperature[0]; break;
    #if HOTENDS > 1
      case 1:
        current_temp = current_temperature[1]; break;
      #if HOTENDS > 2
        case 2:
          current_temp = current_temperature[2]; break;
        #if HOTENDS > 3
          case 3:
            current_temp = current_temperature[3]; break;
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1
  }

  #if ENABLED(PIDTEMP)
    #if DISABLED(PID_OPENLOOP)
      pid_error[HOTEND_INDEX] = target_temperature[HOTEND_INDEX] - current_temp;
      dTerm[HOTEND_INDEX] = K2 * PID_PARAM(Kd, HOTEND_INDEX) * (current_temperature[HOTEND_INDEX] - temp_dState[HOTEND_INDEX]) + K1 * dTerm[HOTEND_INDEX];
      temp_dState[HOTEND_INDEX] = current_temperature[HOTEND_INDEX];
      if (pid_error[HOTEND_INDEX] > PID_FUNCTIONAL_RANGE) {
        pid_output = BANG_MAX;
        pid_reset[HOTEND_INDEX] = true;
      }
      else if (pid_error[HOTEND_INDEX] < -(PID_FUNCTIONAL_RANGE) || target_temperature[HOTEND_INDEX] == 0) {
        pid_output = 0;
        pid_reset[HOTEND_INDEX] = true;
      }
      else {
        if (pid_reset[HOTEND_INDEX]) {
          temp_iState[HOTEND_INDEX] = 0.0;
          pid_reset[HOTEND_INDEX] = false;
        }
        pTerm[HOTEND_INDEX] = PID_PARAM(Kp, HOTEND_INDEX) * pid_error[HOTEND_INDEX];
        temp_iState[HOTEND_INDEX] += pid_error[HOTEND_INDEX];
        iTerm[HOTEND_INDEX] = PID_PARAM(Ki, HOTEND_INDEX) * temp_iState[HOTEND_INDEX];

        pid_output = pTerm[HOTEND_INDEX] + iTerm[HOTEND_INDEX] - dTerm[HOTEND_INDEX];

        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          cTerm[HOTEND_INDEX] = 0;
          if (_HOTEND_TEST) {
            long e_position = stepper.position(E_AXIS);
            if (e_position > last_e_position) {
              lpq[lpq_ptr] = e_position - last_e_position;
              last_e_position = e_position;
            }
            else {
              lpq[lpq_ptr] = 0;
            }
            if (++lpq_ptr >= lpq_len) lpq_ptr = 0;
            cTerm[HOTEND_INDEX] = (lpq[lpq_ptr] * planner.steps_to_mm[E_AXIS]) * PID_PARAM(Kc, HOTEND_INDEX);
            pid_output += cTerm[HOTEND_INDEX];
          }
        #endif // PID_ADD_EXTRUSION_RATE

        if (pid_output > PID_MAX) {
          if (pid_error[HOTEND_INDEX] > 0) temp_iState[HOTEND_INDEX] -= pid_error[HOTEND_INDEX]; // conditional un-integration
          pid_output = PID_MAX;
        }
        else if (pid_output < 0) {
          if (pid_error[HOTEND_INDEX] < 0) temp_iState[HOTEND_INDEX] -= pid_error[HOTEND_INDEX]; // conditional un-integration
          pid_output = 0;
        }
      }
    #else
      pid_output = constrain(target_temperature[HOTEND_INDEX], 0, PID_MAX);
    #endif // PID_OPENLOOP

    #if ENABLED(PID_DEBUG)
      SERIAL_SMV(ECHO, MSG_PID_DEBUG, HOTEND_INDEX);
      SERIAL_MV(MSG_PID_DEBUG_INPUT, current_temperature[HOTEND_INDEX]);
      SERIAL_MV(MSG_PID_DEBUG_OUTPUT, pid_output);
      SERIAL_MV(MSG_PID_DEBUG_PTERM, pTerm[HOTEND_INDEX]);
      SERIAL_MV(MSG_PID_DEBUG_ITERM, iTerm[HOTEND_INDEX]);
      SERIAL_MV(MSG_PID_DEBUG_DTERM, dTerm[HOTEND_INDEX]);
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        SERIAL_ECHOPAIR(MSG_PID_DEBUG_CTERM, cTerm[HOTEND_INDEX]);
      #endif
      SERIAL_EOL;
    #endif // PID_DEBUG

  #else /* PID off */
    pid_output = (current_temperature[HOTEND_INDEX] < target_temperature[HOTEND_INDEX]) ? PID_MAX : 0;
  #endif

  return pid_output;
}

#if ENABLED(PIDTEMPBED)
  float Temperature::get_pid_output_bed() {
    float pid_output;
    #if DISABLED(PID_OPENLOOP)
      pid_error_bed = target_temperature_bed - current_temperature_bed;
      pTerm_bed = bedKp * pid_error_bed;
      temp_iState_bed += pid_error_bed;
      iTerm_bed = bedKi * temp_iState_bed;

      dTerm_bed = K2 * bedKd * (current_temperature_bed - temp_dState_bed) + K1 * dTerm_bed;
      temp_dState_bed = current_temperature_bed;

      pid_output = pTerm_bed + iTerm_bed - dTerm_bed;
      if (pid_output > MAX_BED_POWER) {
        if (pid_error_bed > 0) temp_iState_bed -= pid_error_bed; // conditional un-integration
        pid_output = MAX_BED_POWER;
      }
      else if (pid_output < 0) {
        if (pid_error_bed < 0) temp_iState_bed -= pid_error_bed; // conditional un-integration
        pid_output = 0;
      }
    #else
      pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
    #endif // PID_OPENLOOP

    #if ENABLED(PID_BED_DEBUG)
      SERIAL_SM(DEB ," PID_BED_DEBUG ");
      SERIAL_MV(": Input ", current_temperature_bed);
      SERIAL_MV(" Output ", pid_output);
      SERIAL_MV(" pTerm ", pTerm_bed);
      SERIAL_MV(" iTerm ", iTerm_bed);
      SERIAL_EMV(" dTerm ", dTerm_bed);
    #endif //PID_BED_DEBUG

    return pid_output;
  }
#endif //PIDTEMPBED

#if ENABLED(PIDTEMPCHAMBER)
  float Temperature::get_pid_output_chamber() {
    float pid_output;

    #if DISABLED(PID_OPENLOOP)
      pid_error_chamber = target_temperature_chamber - current_temperature_chamber;
      pTerm_chamber = chamberKp * pid_error_chamber;
      temp_iState_chamber += pid_error_chamber;
      iTerm_chamber = chamberKi * temp_iState_chamber;

      dTerm_chamber = K2 * chamberKd * (current_temperature_chamber - temp_dState_chamber) + K1 * dTerm_chamber;
      temp_dState_chamber = current_temperature_chamber;

      pid_output = pTerm_chamber + iTerm_chamber - dTerm_chamber;
      if (pid_output > MAX_CHAMBER_POWER) {
        if (pid_error_chamber > 0) temp_iState_chamber -= pid_error_chamber; // conditional un-integration
        pid_output = MAX_CHAMBER_POWER;
      }
      else if (pid_output < 0) {
        if (pid_error_chamber < 0) temp_iState_chamber -= pid_error_chamber; // conditional un-integration
        pid_output = 0;
      }
    #else
      pid_output = constrain(target_temperature_chamber, 0, MAX_CHAMBER_POWER);
    #endif // PID_OPENLOOP

    #if ENABLED(PID_CHAMBER_DEBUG)
      SERIAL_SM(DEB ," PID_CHAMBER_DEBUG ");
      SERIAL_MV(": Input ", current_temperature_chamber);
      SERIAL_MV(" Output ", pid_output);
      SERIAL_MV(" pTerm ", pTerm_chamber);
      SERIAL_MV(" iTerm ", iTerm_chamber);
      SERIAL_EMV(" dTerm ", dTerm_chamber);
    #endif // PID_CHAMBER_DEBUG

    return pid_output;
  }
#endif

#if ENABLED(PIDTEMPCOOLER)
  float Temperature::get_pid_output_cooler() {
    float pid_output;

    // We need this cause 0 is lower than our current temperature probably.
    if (target_temperature_cooler < COOLER_MINTEMP)
      return 0.0;

    #if DISABLED(PID_OPENLOOP)
      //pid_error_cooler = target_temperature_cooler - current_temperature_cooler;
      pid_error_cooler = current_temperature_cooler - target_temperature_cooler;
      pTerm_cooler = coolerKp * pid_error_cooler;
      temp_iState_cooler += pid_error_cooler;
      iTerm_cooler = coolerKi * temp_iState_cooler;

      //dTerm_cooler = K2 * coolerKd * (current_temperature_cooler - temp_dState_cooler) + K1 * dTerm_cooler;
      dTerm_cooler = K2 * coolerKd * (temp_dState_cooler - current_temperature_cooler) + K1 * dTerm_cooler;
      temp_dState_cooler = current_temperature_cooler;

      pid_output = pTerm_cooler + iTerm_cooler - dTerm_cooler;
      if (pid_output > MAX_COOLER_POWER) {
        if (pid_error_cooler > 0) temp_iState_cooler -= pid_error_cooler; // conditional un-integration
        pid_output = MAX_COOLER_POWER;
      }
      else if (pid_output < 0) {
        if (pid_error_cooler < 0) temp_iState_cooler -= pid_error_cooler; // conditional un-integration
        pid_output = 0;
      }
    #else
      pid_output = constrain(target_temperature_cooler, 0, MAX_COOLER_POWER);
    #endif // PID_OPENLOOP

    #if ENABLED(PID_COOLER_DEBUG)
      SERIAL_SM(DEB ," PID_COOLER_DEBUG ");
      SERIAL_MV(": Input ", current_temperature_cooler);
      SERIAL_MV(" Output ", pid_output);
      SERIAL_MV(" pTerm ", pTerm_cooler);
      SERIAL_MV(" iTerm ", iTerm_cooler);
      SERIAL_EMV(" dTerm ", dTerm_cooler);
    #endif //PID_COOLER_DEBUG

    return pid_output;
  }

#endif

/**
 * Manage heating activities for hotends, bed, chamber and cooler
 *  - Acquire updated temperature readings
 *    - Also resets the watchdog timer
 *  - Invoke thermal runaway protection
 *  - Manage extruder auto-fan
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */
void Temperature::manage_temp_controller() {

  if (!temp_meas_ready) return;

  updateTemperaturesFromRawValues(); // also resets the watchdog

  #if ENABLED(HEATER_0_USES_MAX6675)
    if (current_temperature[0] > min(HEATER_0_MAXTEMP, MAX6675_TMAX - 1)) max_temp_error(0);
    if (current_temperature[0] < max(HEATER_0_MINTEMP, MAX6675_TMIN + 0.01)) min_temp_error(0);
  #endif

  #if ENABLED(THERMAL_PROTECTION_HOTENDS) || ENABLED(THERMAL_PROTECTION_BED) || ENABLED(THERMAL_PROTECTION_CHAMBER) || ENABLED(THERMAL_PROTECTION_COOLER) || DISABLED(PIDTEMPBED) || DISABLED(PIDTEMPCHAMBER) || DISABLED(PIDTEMPCOOLER) || HAS(AUTO_FAN)
    millis_t ms = millis();
  #endif

  // Loop through all hotends
  HOTEND_LOOP() {

    #if ENABLED(THERMAL_PROTECTION_HOTENDS)
      thermal_runaway_protection(&thermal_runaway_state_machine[h], &thermal_runaway_timer[h], current_temperature[h], target_temperature[h], h, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
    #endif

    float pid_output = get_pid_output(h);

    // Check if temperature is within the correct range
    soft_pwm[h] = (current_temperature[h] > minttemp[h] || is_preheating(h)) && current_temperature[h] < maxttemp[h] ? (int)pid_output : 0;

    // Check if the temperature is failing to increase
    #if ENABLED(THERMAL_PROTECTION_HOTENDS) && WATCH_TEMP_PERIOD > 0

      // Is it time to check this extruder's heater?
      if (watch_heater_next_ms[h] && ELAPSED(ms, watch_heater_next_ms[h])) {
        // Has it failed to increase enough?
        if (degHotend(h) < watch_target_temp[h]) {
          // Stop!
          _temp_error(h, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
        }
        else {
          // Start again if the target is still far off
          start_watching_heater(h);
        }
      }

    #endif // THERMAL_PROTECTION_HOTENDS

    // Check if the temperature is failing to increase
    #if ENABLED(THERMAL_PROTECTION_BED) && WATCH_BED_TEMP_PERIOD > 0

      // Is it time to check the bed?
      if (watch_bed_next_ms && ELAPSED(ms, watch_bed_next_ms)) {
        // Has it failed to increase enough?
        if (degBed() < watch_target_bed_temp) {
          // Stop!
          _temp_error(-1, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
        }
        else {
          // Start again if the target is still far off
          start_watching_bed();
        }
      }

    #endif // THERMAL_PROTECTION_HOTENDS

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      if (fabs(current_temperature[0] - redundant_temperature) > MAX_REDUNDANT_TEMP_SENSOR_DIFF) {
        _temp_error(0, PSTR(MSG_REDUNDANCY), PSTR(MSG_ERR_REDUNDANT_TEMP));
      }
    #endif

  } // Hotends Loop

  #if HAS(AUTO_FAN)
    if (ELAPSED(ms, next_auto_fan_check_ms)) { // only need to check fan state very infrequently
      checkExtruderAutoFans();
      next_auto_fan_check_ms = ms + 2500UL;
    }
  #endif

  // Control the extruder rate based on the width sensor
  #if ENABLED(FILAMENT_SENSOR)
    if (filament_sensor) {
      meas_shift_index = filwidth_delay_index[0] - meas_delay_cm;
      if (meas_shift_index < 0) meas_shift_index += MAX_MEASUREMENT_DELAY + 1;  //loop around buffer if needed

      // Get the delayed info and add 100 to reconstitute to a percent of
      // the nominal filament diameter then square it to get an area
      meas_shift_index = constrain(meas_shift_index, 0, MAX_MEASUREMENT_DELAY);
      float vm = pow((measurement_delay[meas_shift_index] + 100.0) * 0.01, 2);
      NOLESS(vm, 0.01);
      volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM] = vm;
    }
  #endif //FILAMENT_SENSOR

  #if HAS(TEMP_BED) && DISABLED(PIDTEMPBED)
    if (PENDING(ms, next_bed_check_ms)) return;
    next_bed_check_ms = ms + BED_CHECK_INTERVAL;
  #endif

  #if HAS(TEMP_CHAMBER) && DISABLED(PIDTEMPCHAMBER)
    if (PENDING(ms, next_chamber_check_ms)) return;
    next_chamber_check_ms = ms + CHAMBER_CHECK_INTERVAL;
  #endif

  #if HAS(TEMP_COOLER) && DISABLED(PIDTEMPCOOLER)
    if (PENDING(ms, next_cooler_check_ms)) return;
    next_cooler_check_ms = ms + COOLER_CHECK_INTERVAL;
  #endif

  #if HAS(TEMP_BED)
    #if ENABLED(THERMAL_PROTECTION_BED)
      thermal_runaway_protection(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, -1, THERMAL_PROTECTION_BED_PERIOD, THERMAL_PROTECTION_BED_HYSTERESIS);
    #endif

    #if ENABLED(PIDTEMPBED)
      float pid_output = get_pid_output_bed();

      soft_pwm_bed = current_temperature_bed > BED_MINTEMP && current_temperature_bed < BED_MAXTEMP ? (int)pid_output >> 1 : 0;

    #elif ENABLED(BED_LIMIT_SWITCHING)
      // Check if temperature is within the correct band
      if (current_temperature_bed > BED_MINTEMP && current_temperature_bed < BED_MAXTEMP) {
        if (current_temperature_bed >= target_temperature_bed + BED_HYSTERESIS)
          soft_pwm_bed = 0;
        else if (current_temperature_bed <= target_temperature_bed - (BED_HYSTERESIS))
          soft_pwm_bed = MAX_BED_POWER >> 1;
      }
      else {
        soft_pwm_bed = 0;
        WRITE_HEATER_BED(LOW);
      }
    #else // !PIDTEMPBED && !BED_LIMIT_SWITCHING
      // Check if temperature is within the correct range
      if (current_temperature_bed > BED_MINTEMP && current_temperature_bed < BED_MAXTEMP) {
        soft_pwm_bed = current_temperature_bed < target_temperature_bed ? MAX_BED_POWER >> 1 : 0;
      }
      else {
        soft_pwm_bed = 0;
        WRITE_HEATER_BED(LOW);
      }
    #endif
  #endif // HAS(TEMP_BED)

  #if HAS(TEMP_CHAMBER)
    #if ENABLED(THERMAL_PROTECTION_CHAMBER)
      thermal_runaway_protection(&thermal_runaway_chamber_state_machine, &thermal_runaway_chamber_timer, current_temperature_chamber, target_temperature_chamber, -1, THERMAL_PROTECTION_CHAMBER_PERIOD, THERMAL_PROTECTION_CHAMBER_HYSTERESIS);
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      float pid_output = get_pid_output_chamber();

      soft_pwm_chamber = current_temperature_chamber > CHAMBER_MINTEMP && current_temperature_chamber < CHAMBER_MAXTEMP ? (int)pid_output >> 1 : 0;

    #elif ENABLED(CHAMBER_LIMIT_SWITCHING)
      // Check if temperature is within the correct band
      if (current_temperature_chamber > CHAMBER_MINTEMP && current_temperature_chamber < CHAMBER_MAXTEMP) {
        if (current_temperature_chamber >= target_temperature_chamber + CHAMBER_HYSTERESIS)
          soft_pwm_chamber = 0;
        else if (current_temperature_chamber <= target_temperature_chamber - CHAMBER_HYSTERESIS)
          soft_pwm_chamber = MAX_CHAMBER_POWER >> 1;
      }
      else {
        soft_pwm_chamber = 0;
        WRITE_HEATER_CHAMBER(LOW);
      }
    #else // !PIDTEMPCHAMBER && !CHAMBER_LIMIT_SWITCHING
      // Check if temperature is within the correct range
      if (current_temperature_chamber > CHAMBER_MINTEMP && current_temperature_chamber < CHAMBER_MAXTEMP) {
        soft_pwm_chamber = current_temperature_chamber < target_temperature_chamber ? MAX_CHAMBER_POWER >> 1 : 0;
      }
      else {
        soft_pwm_chamber = 0;
        WRITE_HEATER_CHAMBER(LOW);
      }
    #endif
  #endif // HAS(TEMP_CHAMBER)

  #if HAS(TEMP_COOLER)
    #if ENABLED(THERMAL_PROTECTION_COOLER)
      thermal_runaway_protection(&thermal_runaway_cooler_state_machine, &thermal_runaway_cooler_timer, current_temperature_cooler, target_temperature_cooler, -2, THERMAL_PROTECTION_COOLER_PERIOD, THERMAL_PROTECTION_COOLER_HYSTERESIS);
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      float pid_output = get_pid_output_cooler();

      setPwmCooler(current_temperature_cooler > COOLER_MINTEMP && current_temperature_cooler < COOLER_MAXTEMP ? (int)pid_output : 0);

    #elif ENABLED(COOLER_LIMIT_SWITCHING)
      // Check if temperature is within the correct band
      if (current_temperature_cooler > COOLER_MINTEMP && current_temperature_cooler < COOLER_MAXTEMP) {
        if (current_temperature_cooler >= target_temperature_cooler + COOLER_HYSTERESIS)
          setPwmCooler(MAX_COOLER_POWER);
        else if (current_temperature_cooler <= target_temperature_cooler - COOLER_HYSTERESIS)
          setPwmCooler(0);
      }
      else { 
        setPwmCooler(0);
        WRITE_COOLER(LOW);
      }
    #else // COOLER_LIMIT_SWITCHING
      // Check if temperature is within the correct range
      if (current_temperature_cooler > COOLER_MINTEMP && current_temperature_cooler < COOLER_MAXTEMP) {
        setPwmCooler(current_temperature_cooler > target_temperature_cooler ? MAX_COOLER_POWER  : 0);
      }
      else {
        setPwmCooler(0);
        WRITE_COOLER(LOW);
      }
    #endif
  #endif // HAS(TEMP_COOLER)
}

#define PGM_RD_W(x)   (short)pgm_read_word(&x)

// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
float Temperature::analog2temp(int raw, uint8_t h) {
  #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
    if (h > HOTENDS)
  #else
    if (h >= HOTENDS)
  #endif
    {
      SERIAL_SV(ER, (int)h);
      SERIAL_EM(MSG_INVALID_EXTRUDER_NUM);
      kill(PSTR(MSG_KILLED));
      return 0.0;
    }

  #if ENABLED(HEATER_0_USES_MAX6675)
    if (h == 0) return 0.25 * raw;
  #endif

  if (heater_ttbl_map[h] != NULL) {
    float celsius = 0;
    uint8_t i;
    short(*tt)[][2] = (short(*)[][2])(heater_ttbl_map[h]);

    for (i = 1; i < heater_ttbllen_map[h]; i++) {
      if (PGM_RD_W((*tt)[i][0]) > raw) {
        celsius = PGM_RD_W((*tt)[i - 1][1]) +
                  (raw - PGM_RD_W((*tt)[i - 1][0])) *
                  (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i - 1][1])) /
                  (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i - 1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[h]) celsius = PGM_RD_W((*tt)[i - 1][1]);

    return celsius;
  }

  #if HEATER_USES_AD595
    #if ENABLED(__SAM3X8E__)
      return ((raw * ((3.3 * 100.0) / 1024.0) / OVERSAMPLENR) * ad595_gain[h]) + ad595_offset[h];
    #else
      return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * ad595_gain[h]) + ad595_offset[h];
    #endif
  #else
    return 0;
  #endif
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
float Temperature::analog2tempBed(int raw) {
  #if ENABLED(BED_USES_THERMISTOR)
    float celsius = 0;
    byte i;

    for (i = 1; i < BEDTEMPTABLE_LEN; i++) {
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw) {
        celsius  = PGM_RD_W(BEDTEMPTABLE[i - 1][1]) +
                   (raw - PGM_RD_W(BEDTEMPTABLE[i - 1][0])) *
                   (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i - 1][1])) /
                   (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i - 1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i - 1][1]);

    return celsius;

  #elif ENABLED(BED_USES_AD595)
    #if ENABLED(__SAM3X8E__)
      return ((raw * ((3.3 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
    #else
      return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
    #endif
  #else
    UNUSED(raw);
    return 0;
  #endif
}

#if HAS(TEMP_CHAMBER)
  static float Temperature::analog2tempChamber(int raw) { 
    #if ENABLED(CHAMBER_USES_THERMISTOR)
      float celsius = 0;
      byte i;

      for (i = 1; i < CHAMBERTEMPTABLE_LEN; i++) {
        if (PGM_RD_W(CHAMBERTEMPTABLE[i][0]) > raw) {
          celsius  = PGM_RD_W(CHAMBERTEMPTABLE[i - 1][1]) +
                     (raw - PGM_RD_W(CHAMBERTEMPTABLE[i - 1][0])) *
                     (float)(PGM_RD_W(CHAMBERTEMPTABLE[i][1]) - PGM_RD_W(CHAMBERTEMPTABLE[i - 1][1])) /
                     (float)(PGM_RD_W(CHAMBERTEMPTABLE[i][0]) - PGM_RD_W(CHAMBERTEMPTABLE[i - 1][0]));
          break;
        }
      }

      // Overflow: Set to last value in the table
      if (i == CHAMBERTEMPTABLE_LEN) celsius = PGM_RD_W(CHAMBERTEMPTABLE[i - 1][1]);

      return celsius;

    #elif ENABLED(CHAMBER_USES_AD595)
      #if ENABLED(__SAM3X8E__)
        return ((raw * ((3.3 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
      #else
        return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
      #endif
    #else
      UNUSED(raw);
      return 0;
    #endif
  }
#endif

#if HAS(TEMP_COOLER)
  static float Temperature::analog2tempCooler(int raw) { 
    #if ENABLED(COOLER_USES_THERMISTOR)
      float celsius = 0;
      byte i;

      for (i = 1; i < COOLERTEMPTABLE_LEN; i++) {
        if (PGM_RD_W(COOLERTEMPTABLE[i][0]) > raw) {
          celsius  = PGM_RD_W(COOLERTEMPTABLE[i - 1][1]) +
                     (raw - PGM_RD_W(COOLERTEMPTABLE[i - 1][0])) *
                     (float)(PGM_RD_W(COOLERTEMPTABLE[i][1]) - PGM_RD_W(COOLERTEMPTABLE[i - 1][1])) /
                     (float)(PGM_RD_W(COOLERTEMPTABLE[i][0]) - PGM_RD_W(COOLERTEMPTABLE[i - 1][0]));
          break;
        }
      }

      // Overflow: Set to last value in the table
      if (i == COOLERTEMPTABLE_LEN) celsius = PGM_RD_W(COOLERTEMPTABLE[i - 1][1]);

      return celsius;

    #elif ENABLED(COOLER_USES_AD595)
      #if ENABLED(__SAM3X8E__)
        return ((raw * ((3.3 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
      #else
        return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
      #endif
    #else
      UNUSED(raw);
      return 0;
    #endif
  }
#endif

/**
 * Get the raw values into the actual temperatures.
 * The raw values are created in interrupt context,
 * and this function is called from normal context
 * as it would block the stepper routine.
 */
void Temperature::updateTemperaturesFromRawValues() {
  #if ENABLED(HEATER_0_USES_MAX6675)
    current_temperature_raw[0] = read_max6675();
  #endif
  HOTEND_LOOP() {
    current_temperature[h] = analog2temp(current_temperature_raw[h], h);
  }
  current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
  #if HAS(TEMP_CHAMBER)
    current_temperature_chamber = analog2tempChamber(current_temperature_chamber_raw);
  #endif
  #if HAS(TEMP_COOLER)
    current_temperature_cooler = analog2tempCooler(current_temperature_cooler_raw);
  #endif

  #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
    redundant_temperature = analog2temp(redundant_temperature_raw, 1);
  #endif
  #if ENABLED(FILAMENT_SENSOR)
    filament_width_meas = analog2widthFil();
  #endif
  #if HAS(POWER_CONSUMPTION_SENSOR)
    static millis_t last_update = millis();
    millis_t temp_last_update = millis();
    millis_t from_last_update = temp_last_update - last_update;
    static float watt_overflow = 0.0;
    power_consumption_meas = analog2power();
    /*SERIAL_MV("raw:", raw_analog2voltage(), 5);
    SERIAL_MV(" - V:", analog2voltage(), 5);
    SERIAL_MV(" - I:", analog2current(), 5);
    SERIAL_EMV(" - P:", analog2power(), 5);*/
    watt_overflow += (power_consumption_meas * from_last_update) / 3600000.0;
    if (watt_overflow >= 1.0) {
      power_consumption_hour++;
      watt_overflow--;
    }
    last_update = temp_last_update;
  #endif

  #if ENABLED(USE_WATCHDOG)
    // Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();
  #endif

  CRITICAL_SECTION_START;
  temp_meas_ready = false;
  CRITICAL_SECTION_END;
}


#if ENABLED(FILAMENT_SENSOR)

  // Convert raw Filament Width to millimeters
  float Temperature::analog2widthFil() {
    return current_raw_filwidth / 16383.0 * 5.0;
    //return current_raw_filwidth;
  }

  // Convert raw Filament Width to a ratio
  int Temperature::widthFil_to_size_ratio() {
    float temp = filament_width_meas;
    if (temp < MEASURED_LOWER_LIMIT) temp = filament_width_nominal;  //assume sensor cut out
    else NOMORE(temp, MEASURED_UPPER_LIMIT);
    return filament_width_nominal / temp * 100;
  }

#endif

#if HAS(POWER_CONSUMPTION_SENSOR)
  // Convert raw Power Consumption to watt
  float Temperature::raw_analog2voltage() {
    return (5.0 * current_raw_powconsumption) / (1023.0 * OVERSAMPLENR);
  }

  float Temperature::analog2voltage() {
    float power_zero_raw = (POWER_ZERO * 1023.0 * OVERSAMPLENR) / 5.0;
    float rel_raw_power = (current_raw_powconsumption < power_zero_raw) ? (2 * power_zero_raw - current_raw_powconsumption) : (current_raw_powconsumption);
    return ((5.0 * rel_raw_power) / (1023.0 * OVERSAMPLENR)) - POWER_ZERO;
  }

  float Temperature::analog2current() {
    float temp = analog2voltage() / POWER_SENSITIVITY;
    temp = (((100 - POWER_ERROR) / 100) * temp) - POWER_OFFSET;
    return temp > 0 ? temp : 0;
  }

  float Temperature::analog2power() {
    return (analog2current() * POWER_VOLTAGE * 100) /  POWER_EFFICIENCY;
  }

  float Temperature::analog2error(float current) {
    float temp1 = (analog2voltage() / POWER_SENSITIVITY - POWER_OFFSET) * POWER_VOLTAGE;
    if(temp1 <= 0) return 0.0;
    float temp2 = (current) * POWER_VOLTAGE;
    if(temp2 <= 0) return 0.0;
    return ((temp2/temp1) - 1) * 100;
  }

  float Temperature::analog2efficiency(float watt) {
    return (analog2current() * POWER_VOLTAGE * 100) / watt;
  }
#endif

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_temp_controller()
 */
void Temperature::init() {

  #if MB(RUMBA) && ((TEMP_SENSOR_0==-1)||(TEMP_SENSOR_1==-1)||(TEMP_SENSOR_2==-1)||(TEMP_SENSOR_BED==-1)||(TEMP_SENSOR_CHAMBER==-1)||(TEMP_SENSOR_COOLER==-1))
    //disable RUMBA JTAG in case the thermocouple extension is plugged on top of JTAG connector
    MCUCR = _BV(JTD);
    MCUCR = _BV(JTD);
  #endif

  // Finish init of mult hotend arrays
  HOTEND_LOOP() {
    // populate with the first value
    maxttemp[h] = maxttemp[0];
  }

  #if ENABLED(PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
    last_e_position = 0;
  #endif

  #if HAS(HEATER_0)
    SET_OUTPUT(HEATER_0_PIN);
  #endif
  #if HAS(HEATER_1)
    SET_OUTPUT(HEATER_1_PIN);
  #endif
  #if HAS(HEATER_2)
    SET_OUTPUT(HEATER_2_PIN);
  #endif
  #if HAS(HEATER_3)
    SET_OUTPUT(HEATER_3_PIN);
  #endif
  #if HAS(HEATER_BED)
    SET_OUTPUT(HEATER_BED_PIN);
  #endif
  #if HAS(HEATER_CHAMBER)
    SET_OUTPUT(HEATER_CHAMBER_PIN);
  #endif
  #if HAS(COOLER)
    SET_OUTPUT(COOLER_PIN);
    #if ENABLED(FAST_PWM_COOLER)
	    setPwmFrequency(COOLER_PIN, 2); // No prescaling. Pwm frequency = F_CPU/256/64
    #endif
  #endif
  #if HAS(FAN)
    SET_OUTPUT(FAN_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
    #if ENABLED(FAN_SOFT_PWM)
      soft_pwm_fan = fanSpeedSoftPwm >> 1;
    #endif
  #endif

  #if ENABLED(HEATER_0_USES_MAX6675)

    OUT_WRITE(SCK_PIN, LOW);
    OUT_WRITE(MOSI_PIN, HIGH);
    SET_INPUT(MISO_PIN);
    WRITE(MISO_PIN,1);
    OUT_WRITE(SDSS, HIGH);

    OUT_WRITE(MAX6675_SS, HIGH);

  #endif // HEATER_0_USES_MAX6675

  // Set analog inputs
  #if ENABLED(__SAM3X8E__)
    #define ANALOG_SELECT(pin) startAdcConversion(pinToAdcChannel(pin))
  #else
    #ifdef DIDR2
      #define ANALOG_SELECT(pin) do{ if (pin < 8) SBI(DIDR0, pin); else SBI(DIDR2, pin - 8); }while(0)
    #else
      #define ANALOG_SELECT(pin) do{ SBI(DIDR0, pin); }while(0)
    #endif
  #endif

  // Setup channels
  #if ENABLED(__SAM3X8E__)
    // ADC_MR_FREERUN_ON: Free Run Mode. It never waits for any trigger.
    ADC->ADC_MR |= ADC_MR_FREERUN_ON | ADC_MR_LOWRES_BITS_12;
  #else
    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIF) | 0x07;
    DIDR0 = 0;
    #ifdef DIDR2
      DIDR2 = 0;
    #endif
  #endif

  #if HAS(TEMP_0)
    ANALOG_SELECT(TEMP_0_PIN);
  #endif
  #if HAS(TEMP_1)
    ANALOG_SELECT(TEMP_1_PIN);
  #endif
  #if HAS(TEMP_2)
    ANALOG_SELECT(TEMP_2_PIN);
  #endif
  #if HAS(TEMP_3)
    ANALOG_SELECT(TEMP_3_PIN);
  #endif

  #if HAS(TEMP_BED)
    ANALOG_SELECT(TEMP_BED_PIN);
  #endif

  #if HAS(TEMP_CHAMBER)
    ANALOG_SELECT(TEMP_CHAMBER_PIN);
  #endif

  #if HAS(TEMP_COOLER)
    ANALOG_SELECT(TEMP_COOLER_PIN);
  #endif

  #if HAS(FILAMENT_SENSOR)
    ANALOG_SELECT(FILWIDTH_PIN);
  #endif

  #if HAS(POWER_CONSUMPTION_SENSOR)
    ANALOG_SELECT(POWER_CONSUMPTION_PIN);
  #endif

  #if HAS(AUTO_FAN_0)
    SET_OUTPUT(H0_AUTO_FAN_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(H0_AUTO_FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
  #endif
  #if HAS(AUTO_FAN_1) && !AUTO_1_IS_0
    SET_OUTPUT(H1_AUTO_FAN_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(H1_AUTO_FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
  #endif
  #if HAS(AUTO_FAN_2) && !AUTO_2_IS_0 && !AUTO_2_IS_1
    SET_OUTPUT(H2_AUTO_FAN_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(H2_AUTO_FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
  #endif
  #if HAS(AUTO_FAN_3) && !AUTO_3_IS_0 && !AUTO_3_IS_1 && !AUTO_3_IS_2
    SET_OUTPUT(H3_AUTO_FAN_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(H3_AUTO_FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
  #endif

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  #if ENABLED(__SAM3X8E__)
    HAL_temp_timer_start(TEMP_TIMER_NUM);
  #else
    OCR0B = 128;
  #endif

  ENABLE_TEMPERATURE_INTERRUPT();

  // Wait for temperature measurement to settle
  HAL::delayMilliseconds(250);

  #define TEMP_MIN_ROUTINE(NR) \
    minttemp[NR] = HEATER_ ## NR ## _MINTEMP; \
    while(analog2temp(minttemp_raw[NR], NR) < HEATER_ ## NR ## _MINTEMP) { \
      if (HEATER_ ## NR ## _RAW_LO_TEMP < HEATER_ ## NR ## _RAW_HI_TEMP) \
        minttemp_raw[NR] += OVERSAMPLENR; \
      else \
        minttemp_raw[NR] -= OVERSAMPLENR; \
    }
  #define TEMP_MAX_ROUTINE(NR) \
    maxttemp[NR] = HEATER_ ## NR ## _MAXTEMP; \
    while(analog2temp(maxttemp_raw[NR], NR) > HEATER_ ## NR ## _MAXTEMP) { \
      if (HEATER_ ## NR ## _RAW_LO_TEMP < HEATER_ ## NR ## _RAW_HI_TEMP) \
        maxttemp_raw[NR] -= OVERSAMPLENR; \
      else \
        maxttemp_raw[NR] += OVERSAMPLENR; \
    }

  #if ENABLED(HEATER_0_MINTEMP)
    TEMP_MIN_ROUTINE(0);
  #endif
  #if ENABLED(HEATER_0_MAXTEMP)
    TEMP_MAX_ROUTINE(0);
  #endif
  #if HOTENDS > 1
    #if ENABLED(HEATER_1_MINTEMP)
      TEMP_MIN_ROUTINE(1);
    #endif
    #if ENABLED(HEATER_1_MAXTEMP)
      TEMP_MAX_ROUTINE(1);
    #endif
    #if HOTENDS > 2
      #if ENABLED(HEATER_2_MINTEMP)
        TEMP_MIN_ROUTINE(2);
      #endif
      #if ENABLED(HEATER_2_MAXTEMP)
        TEMP_MAX_ROUTINE(2);
      #endif
      #if HOTENDS > 3
        #if ENABLED(HEATER_3_MINTEMP)
          TEMP_MIN_ROUTINE(3);
        #endif
        #if ENABLED(HEATER_3_MAXTEMP)
          TEMP_MAX_ROUTINE(3);
        #endif
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1

  #if ENABLED(BED_MINTEMP)
    while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_minttemp_raw += OVERSAMPLENR;
      #else
        bed_minttemp_raw -= OVERSAMPLENR;
      #endif
    }
  #endif //BED_MINTEMP
  #if ENABLED(BED_MAXTEMP)
    while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_maxttemp_raw -= OVERSAMPLENR;
      #else
        bed_maxttemp_raw += OVERSAMPLENR;
      #endif
    }
  #endif // BED_MAXTEMP

  #if ENABLED(CHAMBER_MINTEMP)
    while(analog2tempChamber(chamber_minttemp_raw) < CHAMBER_MINTEMP) {
      #if CHAMBER_RAW_LO_TEMP < HEATER_CHAMBER_RAW_HI_TEMP
        chamber_minttemp_raw += OVERSAMPLENR;
      #else
        chamber_minttemp_raw -= OVERSAMPLENR;
      #endif
    }
  #endif // CHAMBER_MINTEMP
  #if ENABLED(CHAMBER_MAXTEMP)
    while(analog2tempCooler(chamber_maxttemp_raw) > CHAMBER_MAXTEMP) {
      #if CHAMBER_RAW_LO_TEMP < CHAMBER_RAW_HI_TEMP
        chamber_maxttemp_raw -= OVERSAMPLENR;
      #else
        chamber_maxttemp_raw += OVERSAMPLENR;
      #endif
    }
  #endif // BED_MAXTEMP

  #if ENABLED(COOLER_MINTEMP)
    while(analog2tempCooler(cooler_minttemp_raw) < COOLER_MINTEMP) {
      #if COOLER_RAW_LO_TEMP < HEATER_COOLER_RAW_HI_TEMP
        cooler_minttemp_raw += OVERSAMPLENR;
      #else
        cooler_minttemp_raw -= OVERSAMPLENR;
      #endif
    }
  #endif // COOLER_MINTEMP
  #if ENABLED(COOLER_MAXTEMP)
    while(analog2tempCooler(cooler_maxttemp_raw) > COOLER_MAXTEMP) {
      #if COOLER_RAW_LO_TEMP < COOLER_RAW_HI_TEMP
        cooler_maxttemp_raw -= OVERSAMPLENR;
      #else
        cooler_maxttemp_raw += OVERSAMPLENR;
      #endif
    }
  #endif // COOLER_MAXTEMP
}

#if ENABLED(THERMAL_PROTECTION_HOTENDS) && WATCH_TEMP_PERIOD > 0
  /**
   * Start Heating Sanity Check for hotends that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set. (M104, M109)
   */
  void Temperature::start_watching_heater(uint8_t h) {
    #if HOTENDS <= 1
      UNUSED(h);
    #endif
    if (degHotend(HOTEND_INDEX) < degTargetHotend(HOTEND_INDEX) - (WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
      watch_target_temp[HOTEND_INDEX] = degHotend(HOTEND_INDEX) + WATCH_TEMP_INCREASE;
      watch_heater_next_ms[HOTEND_INDEX] = millis() + (WATCH_TEMP_PERIOD) * 1000UL;
    }
    else
      watch_heater_next_ms[HOTEND_INDEX] = 0;
  }
#endif

#if ENABLED(THERMAL_PROTECTION_BED) && WATCH_BED_TEMP_PERIOD > 0
  /**
   * Start Heating Sanity Check for hotends that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set. (M140, M190)
   */
  void Temperature::start_watching_bed() {
    if (degBed() < degTargetBed() - (WATCH_BED_TEMP_INCREASE + TEMP_BED_HYSTERESIS + 1)) {
      watch_target_bed_temp = degBed() + WATCH_BED_TEMP_INCREASE;
      watch_bed_next_ms = millis() + (WATCH_BED_TEMP_PERIOD) * 1000UL;
    }
    else
      watch_bed_next_ms = 0;
  }
#endif

#if ENABLED(THERMAL_PROTECTION_CHAMBER) && WATCH_CHAMBER_TEMP_PERIOD > 0
  /**
   * Start Cooling Sanity Check for chamber that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set. (M141)
   */
  void Temperature::start_watching_chamber() {
    if (degCooler() > degTargetCooler() - (WATCH_TEMP_CHAMBER_DECREASE - TEMP_CHAMBER_HYSTERESIS - 1)) {
      watch_target_temp_chamber = degChamber() - WATCH_CHAMBER_TEMP_DECREASE;
      watch_chamber_next_ms = millis() + WATCH_TEMP_CHAMBER_PERIOD * 1000UL;
    }
    else
      watch_chamber_next_ms = 0;
  }
#endif

#if ENABLED(THERMAL_PROTECTION_COOLER) && WATCH_COOLER_TEMP_PERIOD > 0
  /**
   * Start Cooling Sanity Check for hotends that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set. (M142)
   */
  void Temperature::start_watching_cooler() {
    if (degCooler() > degTargetCooler() - (WATCH_TEMP_COOLER_DECREASE - TEMP_COOLER_HYSTERESIS - 1)) {
      watch_target_temp_cooler = degCooler() - WATCH_COOLER_TEMP_DECREASE;
      watch_cooler_next_ms = millis() + WATCH_TEMP_COOLER_PERIOD * 1000UL;
    }
    else
      watch_cooler_next_ms = 0;
  }
#endif

#if ENABLED(THERMAL_PROTECTION_HOTENDS) || ENABLED(THERMAL_PROTECTION_BED) || ENABLED(THERMAL_PROTECTION_CHAMBER) || ENABLED(THERMAL_PROTECTION_COOLER)

  #if ENABLED(THERMAL_PROTECTION_HOTENDS)
    Temperature::TRState Temperature::thermal_runaway_state_machine[HOTENDS] = { TRInactive };
    millis_t Temperature::thermal_runaway_timer[HOTENDS] = { 0 };
  #endif

  #if HAS_THERMALLY_PROTECTED_BED
    Temperature::TRState Temperature::thermal_runaway_bed_state_machine = TRInactive;
    millis_t Temperature::thermal_runaway_bed_timer;
  #endif

  #if ENABLED(THERMAL_PROTECTION_CHAMBER) && HAS(TEMP_CHAMBER)
    Temperature::TRState Temperature::thermal_runaway_chamber_state_machine = TRReset;
    millis_t Temperature::thermal_runaway_chamber_timer;
  #endif

  #if ENABLED(THERMAL_PROTECTION_COOLER) && HAS(TEMP_COOLER)
    Temperature::TRState Temperature::thermal_runaway_cooler_state_machine = TRReset;
    millis_t Temperature::thermal_runaway_cooler_timer;
  #endif

  void Temperature::thermal_runaway_protection(Temperature::TRState* state, millis_t* timer, float temperature, float target_temperature, int temp_controller_id, int period_seconds, int hysteresis_degc) {

    static float tr_target_temperature[HOTENDS + 3] = { 0.0 };

    /*
        SERIAL_M("Thermal Thermal Runaway Running. Heater ID: ");
        if (temp_controller_id < 0) SERIAL_M("bed"); else SERIAL_V(temp_controller_id);
        SERIAL_MV(" ;  State:", *state);
        SERIAL_MV(" ;  Timer:", *timer);
        SERIAL_MV(" ;  Temperature:", temperature);
        SERIAL_EMV(" ;  Target Temp:", target_temperature);
    */

    int temp_controller_index;

    if(temp_controller_id >= 0)
      temp_controller_index = temp_controller_id;
    else if(temp_controller_id == -1)
      temp_controller_index = HOTENDS; // BED
    else if(temp_controller_id == -2)
      temp_controller_index = HOTENDS + 1; // CHAMBER
    else
      temp_controller_index = HOTENDS + 2; // COOLER

    // If the target temperature changes, restart
    if (tr_target_temperature[temp_controller_index] != target_temperature) {
      tr_target_temperature[temp_controller_index] = target_temperature;
      *state = target_temperature > 0 ? TRFirstHeating : TRInactive;
    }

    switch (*state) {
      // Inactive state waits for a target temperature to be set
      case TRInactive: break;
      // When first heating, wait for the temperature to be reached then go to Stable state
      case TRFirstHeating:
        if (temperature < tr_target_temperature[temp_controller_index]) break;
        *state = TRStable;
      // While the temperature is stable watch for a bad temperature
      case TRStable:
        if (temperature >= tr_target_temperature[temp_controller_index] - hysteresis_degc) {
          *timer = millis() + period_seconds * 1000UL;
          break;
        }
        else if (PENDING(millis(), *timer)) break;
        *state = TRRunaway;
      case TRRunaway:
        _temp_error(temp_controller_id, PSTR(MSG_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
    }
  }

#endif // THERMAL_PROTECTION_HOTENDS || THERMAL_PROTECTION_BED

void Temperature::disable_all_heaters() {
  HOTEND_LOOP() setTargetHotend(0, h);
  setTargetBed(0);

  #if HAS(TEMP_CHAMBER)
    setTargetChamber(0);
  #endif

  // If all heaters go down then for sure our print job has stopped
  print_job_counter.stop();

  #define DISABLE_HEATER(NR) { \
    setTargetHotend(0, NR); \
    soft_pwm[NR] = 0; \
    WRITE_HEATER_ ## NR (LOW); \
  }

  #if HAS(TEMP_0)
    target_temperature[0] = 0;
    soft_pwm[0] = 0;
    WRITE_HEATER_0P(LOW); // Should HEATERS_PARALLEL apply here? Then change to DISABLE_HEATER(0)
  #endif

  #if HOTENDS > 1 && HAS(TEMP_1)
    DISABLE_HEATER(1);
  #endif

  #if HOTENDS > 2 && HAS(TEMP_2)
    DISABLE_HEATER(2);
  #endif

  #if HOTENDS > 3 && HAS(TEMP_3)
    DISABLE_HEATER(3);
  #endif

  #if HAS(TEMP_BED)
    target_temperature_bed = 0;
    soft_pwm_bed = 0;
    #if HAS(HEATER_BED)
      WRITE_HEATER_BED(LOW);
    #endif
  #endif

  #if HAS(TEMP_CHAMBER)
    target_temperature_chamber = 0;
    soft_pwm_chamber = 0;
    #if HAS(HEATER_CHAMBER)
      WRITE_HEATER_CHAMBER(LOW);
    #endif
  #endif
}

#if HAS(TEMP_COOLER)
  void Temperature::disable_all_coolers() {
    setTargetCooler(0);

    // if cooler go down the print job is stopped 
    print_job_counter.stop();

    #if ENABLED(LASER)
      // No laser firing with no coolers running! (paranoia)
      laser_extinguish();
    #endif

    #if HAS(TEMP_COOLER)
      target_temperature_cooler = 0;
      setPwmCooler(0);
      #if HAS(COOLER) && !ENABLED(FAST_PWM_COOLER)
        WRITE_COOLER(LOW);
      #endif
    #endif
  }
#endif

#if ENABLED(HEATER_0_USES_MAX6675)

  #define MAX6675_HEAT_INTERVAL 250u

  #if ENABLED(MAX6675_IS_MAX31855)
    uint32_t max6675_temp = 2000;
    #define MAX6675_ERROR_MASK 7
    #define MAX6675_DISCARD_BITS 18
    #define MAX6675_SPEED_BITS (_BV(SPR1)) // clock รท 64
  #else
    uint16_t max6675_temp = 2000;
    #define MAX6675_ERROR_MASK 4
    #define MAX6675_DISCARD_BITS 3
    #define MAX6675_SPEED_BITS (_BV(SPR0)) // clock รท 16
  #endif

  int Temperature::read_max6675() {

    static millis_t next_max6675_ms = 0;

    millis_t ms = millis();

    if (PENDING(ms, next_max6675_ms)) return (int)max6675_temp;

    next_max6675_ms = ms + MAX6675_HEAT_INTERVAL;

    #if ENABLED(__SAM3X8E__)
      HAL::spiBegin();
      HAL::spiInit(2);
    #else
      CBI(
        #ifdef PRR
          PRR
        #elif defined(PRR0)
          PRR0
        #endif
          , PRSPI);
      SPCR = _BV(MSTR) | _BV(SPE) | _BV(SPR0);
    #endif

    HAL::digitalWrite(MAX6675_SS, 0);  // enable TT_MAX6675

    // ensure 100ns delay - a bit extra is fine
    #if ENABLED(__SAM3X8E__)
      HAL::delayMicroseconds(1);
    #else
      asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
      asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
    #endif

    // Read a big-endian temperature value
    max6675_temp = 0;
    for (uint8_t i = sizeof(max6675_temp); i--;) {
      #if ENABLED(__SAM3X8E__)
        max6675_temp |= HAL::spiReceive();
      #else
        SPDR = 0;
        for (;!TEST(SPSR, SPIF););
        max6675_temp |= SPDR;
      #endif
      if (i > 0) max6675_temp <<= 8; // shift left if not the last byte
    }

    HAL::digitalWrite(MAX6675_SS, 1); // disable TT_MAX6675

    if (max6675_temp & MAX6675_ERROR_MASK) {
      SERIAL_SM(ER, "Temp measurement error! ");
      #if MAX6675_ERROR_MASK == 7
        SERIAL_M("MAX31855 ");
        if (max6675_temp & 1)
          SERIAL_EM("Open Circuit");
        else if (max6675_temp & 2)
          SERIAL_EM("Short to GND");
        else if (max6675_temp & 4)
          SERIAL_EM("Short to VCC");
      #else
        SERIAL_EM("MAX6675");
      #endif
      max6675_temp = MAX6675_TMAX * 4; // thermocouple open
    }
    else {
      max6675_temp >>= MAX6675_DISCARD_BITS;
      #if ENABLED(MAX6675_IS_MAX31855)
        // Support negative temperature
        if (max6675_temp & 0x00002000) max6675_temp |= 0xffffc000;
      #endif
    }

    return (int)max6675_temp;
  }

#endif //HEATER_0_USES_MAX6675

/**
 * Get raw temperatures
 */
#if ENABLED(__SAM3X8E__)
  int Temperature::calc_raw_temp_value(uint8_t temp_id) {
    raw_median_temp[temp_id][median_counter] = (raw_temp_value[temp_id] - (min_temp[temp_id] + max_temp[temp_id]));
    sum = 0;
    for(int i = 0; i < MEDIAN_COUNT; i++) sum += raw_median_temp[temp_id][i];
    return (sum / MEDIAN_COUNT + CORRECTION_FOR_RAW_TEMP) >> 2;
  }

  #if HAS(TEMP_BED)
    int Temperature::calc_raw_temp_bed_value() {
      raw_median_temp[4][median_counter] = (raw_temp_bed_value - (min_temp[4] + max_temp[4]));
      sum = 0;
      for(int i = 0; i < MEDIAN_COUNT; i++) sum += raw_median_temp[4][i];
      return (sum / MEDIAN_COUNT + CORRECTION_FOR_RAW_TEMP) >> 2;
    }
  #endif

  #if HAS(TEMP_CHAMBER)
    int Temperature::calc_raw_temp_chamber_value() {
      raw_median_temp[5][median_counter] = (raw_temp_chamber_value - (min_temp[5] + max_temp[5]));
      sum = 0;
      for(int i = 0; i < MEDIAN_COUNT; i++) sum += raw_median_temp[5][i];
      return (sum / MEDIAN_COUNT + CORRECTION_FOR_RAW_TEMP) >> 2;
    }
  #endif

  #if HAS(TEMP_COOLER)
    int Temperature::calc_raw_temp_cooler_value() {
      raw_median_temp[6][median_counter] = (raw_temp_cooler_value - (min_temp[6] + max_temp[6]));
      sum = 0;
      for(int i = 0; i < MEDIAN_COUNT; i++) sum += raw_median_temp[6][i];
      return (sum / MEDIAN_COUNT + CORRECTION_FOR_RAW_TEMP) >> 2;
    }
  #endif
#endif

void Temperature::set_current_temp_raw() {
  #if HAS(TEMP_0) && DISABLED(HEATER_0_USES_MAX6675)
    #if ENABLED(__SAM3X8E__)
      current_temperature_raw[0] = calc_raw_temp_value(0);
    #else
      current_temperature_raw[0] = raw_temp_value[0];
    #endif
  #endif
  #if HAS(TEMP_1)
    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      #if ENABLED(__SAM3X8E__)
        redundant_temperature_raw = calc_raw_temp_value(1);
      #else
        redundant_temperature_raw = raw_temp_value[1];
      #endif
    #else
      #if ENABLED(__SAM3X8E__)
        current_temperature_raw[1] = calc_raw_temp_value(1);
      #else
        current_temperature_raw[1] = raw_temp_value[1];
      #endif
    #endif
    #if HAS(TEMP_2)
      #if ENABLED(__SAM3X8E__)
        current_temperature_raw[2] = calc_raw_temp_value(2);
      #else
        current_temperature_raw[2] = raw_temp_value[2];
      #endif
      #if HAS(TEMP_3)
        #if ENABLED(__SAM3X8E__)
          current_temperature_raw[3] = calc_raw_temp_value(3);
        #else
          current_temperature_raw[3] = raw_temp_value[3];
        #endif
      #endif
    #endif
  #endif

  #if ENABLED(__SAM3X8E__)
    #if HAS(TEMP_BED)
      current_temperature_bed_raw = calc_raw_temp_bed_value();
    #endif
    #if HAS(TEMP_CHAMBER)
      current_temperature_chamber_raw = calc_raw_temp_chamber_value();
    #endif
    #if HAS(TEMP_COOLER)
      current_temperature_cooler_raw = calc_raw_temp_cooler_value();
    #endif
  #else
    #if HAS(TEMP_BED)
      current_temperature_bed_raw = raw_temp_bed_value;
    #endif
    #if HAS(TEMP_CHAMBER)
      current_temperature_chamber_raw = raw_temp_chamber_value;
    #endif
    #if HAS(TEMP_COOLER)
      current_temperature_cooler_raw = raw_temp_cooler_value;
    #endif
  #endif

  #if HAS(POWER_CONSUMPTION_SENSOR)
    #if ENABLED(__SAM3X8E__)
      current_raw_powconsumption = calc_raw_powconsumption_value();
    #else
      current_raw_powconsumption = raw_powconsumption_value;
    #endif
  #endif

  #if ENABLED(__SAM3X8E__)
    // Reset min/max-holder
    for (uint8_t i = 0; i < 7; i++) {
      min_temp[i] = RAW_MIN_TEMP_DEFAULT;
      max_temp[i] = 0;
    }

    median_counter++;
    if (median_counter >= MEDIAN_COUNT) median_counter = 0;
  #endif

  temp_meas_ready = true;
}

#if ENABLED(PINS_DEBUGGING)
  /**
   * monitors endstops & Z probe for changes
   *
   * If a change is detected then the LED is toggled and
   * a message is sent out the serial port
   *
   * Yes, we could miss a rapid back & forth change but
   * that won't matter because this is all manual.
   *
   */
  void endstop_monitor() {
    static uint16_t old_endstop_bits_local = 0;
    static uint8_t local_LED_status = 0;
    uint16_t current_endstop_bits_local = 0;
    #if HAS_X_MIN
      if (READ(X_MIN_PIN)) SBI(current_endstop_bits_local, X_MIN);
    #endif
    #if HAS_X_MAX
      if (READ(X_MAX_PIN)) SBI(current_endstop_bits_local, X_MAX);
    #endif
    #if HAS_Y_MIN
      if (READ(Y_MIN_PIN)) SBI(current_endstop_bits_local, Y_MIN);
    #endif
    #if HAS_Y_MAX
      if (READ(Y_MAX_PIN)) SBI(current_endstop_bits_local, Y_MAX);
    #endif
    #if HAS_Z_MIN
      if (READ(Z_MIN_PIN)) SBI(current_endstop_bits_local, Z_MIN);
    #endif
    #if HAS_Z_MAX
      if (READ(Z_MAX_PIN)) SBI(current_endstop_bits_local, Z_MAX);
    #endif
    #if HAS_Z_PROBE_PIN
      if (READ(Z_PROBE_PIN)) SBI(current_endstop_bits_local, Z_PROBE);
    #endif
    #if HAS_Z2_MIN
      if (READ(Z2_MIN_PIN)) SBI(current_endstop_bits_local, Z2_MIN);
    #endif
    #if HAS_Z2_MAX
      if (READ(Z2_MAX_PIN)) SBI(current_endstop_bits_local, Z2_MAX);
    #endif

    uint16_t endstop_change = current_endstop_bits_local ^ old_endstop_bits_local;

    if (endstop_change) {
      #if HAS_X_MIN
        if (TEST(endstop_change, X_MIN)) SERIAL_MV("X_MIN:", !!TEST(current_endstop_bits_local, X_MIN));
      #endif
      #if HAS_X_MAX
        if (TEST(endstop_change, X_MAX)) SERIAL_MV("  X_MAX:", !!TEST(current_endstop_bits_local, X_MAX));
      #endif
      #if HAS_Y_MIN
        if (TEST(endstop_change, Y_MIN)) SERIAL_MV("  Y_MIN:", !!TEST(current_endstop_bits_local, Y_MIN));
      #endif
      #if HAS_Y_MAX
        if (TEST(endstop_change, Y_MAX)) SERIAL_MV("  Y_MAX:", !!TEST(current_endstop_bits_local, Y_MAX));
      #endif
      #if HAS_Z_MIN
        if (TEST(endstop_change, Z_MIN)) SERIAL_MV("  Z_MIN:", !!TEST(current_endstop_bits_local, Z_MIN));
      #endif
      #if HAS_Z_MAX
        if (TEST(endstop_change, Z_MAX)) SERIAL_MV("  Z_MAX:", !!TEST(current_endstop_bits_local, Z_MAX));
      #endif
      #if HAS_Z_PROBE_PIN
        if (TEST(endstop_change, Z_PROBE)) SERIAL_MV("  PROBE:", !!TEST(current_endstop_bits_local, Z_PROBE));
      #endif
      #if HAS_Z2_MIN
        if (TEST(endstop_change, Z2_MIN)) SERIAL_MV("  Z2_MIN:", !!TEST(current_endstop_bits_local, Z2_MIN));
      #endif
      #if HAS_Z2_MAX
        if (TEST(endstop_change, Z2_MAX)) SERIAL_MV("  Z2_MAX:", !!TEST(current_endstop_bits_local, Z2_MAX));
      #endif
      SERIAL_M("\n\n");
      analogWrite(LED_PIN, local_LED_status);
      local_LED_status ^= 255;
      old_endstop_bits_local = current_endstop_bits_local;
    }
  }
#endif // PINS_DEBUGGING

/**
 * Timer 0 is shared with millies so don't change the prescaler.
 *
 * This ISR uses the compare method so it runs at the base
 * frequency (16 MHz / 64 / 256 = 976.5625 Hz), but at the TCNT0 set
 * in OCR0B above (128 or halfway between OVFs).
 *
 *  - Manage PWM to all the heaters and fan
 *  - Update the raw temperature values
 *  - Check new temperature values for MIN/MAX errors
 *  - Step the babysteps value for each axis towards 0
 */
#if ENABLED(__SAM3X8E__)
  HAL_TEMP_TIMER_ISR { Temperature::isr(); }
#else
  ISR(TIMER0_COMPB_vect) { Temperature::isr(); }
#endif

void Temperature::isr() {
  // Allow UART and stepper ISRs
  DISABLE_TEMPERATURE_INTERRUPT(); // Disable Temperature ISR
  sei();

  static uint8_t temp_count = 0;
  static TempState temp_state = StartupDelay;
  static uint8_t pwm_count = _BV(SOFT_PWM_SCALE);

  #if ENABLED(__SAM3X8E__)
    static int temp_read = 0;
    static bool first_start = true;
  #endif

  // Static members for each heater
  #if ENABLED(SLOW_PWM_HEATERS)
    static uint8_t slow_pwm_count = 0;
    #define ISR_STATICS(n) \
      static uint8_t soft_pwm_ ## n; \
      static uint8_t state_heater_ ## n = 0; \
      static uint8_t state_timer_heater_ ## n = 0
  #else
    #define ISR_STATICS(n) static uint8_t soft_pwm_ ## n
  #endif

  // Statics per heater
  ISR_STATICS(0);
  #if HOTENDS > 1
    ISR_STATICS(1);
    #if HOTENDS > 2
      ISR_STATICS(2);
      #if HOTENDS > 3
        ISR_STATICS(3);
      #endif
    #endif
  #endif
  #if HAS_HEATER_BED
    ISR_STATICS(BED);
  #endif
  #if HAS(HEATER_CHAMBER)
    ISR_STATICS(CHAMBER);
  #endif
  #if HAS(COOLER) && !ENABLED(FAST_PWM_COOLER)
    ISR_STATICS(COOLER);
  #endif

  #if ENABLED(FILAMENT_SENSOR)
    static unsigned long raw_filwidth_value = 0;
  #endif

  #if ENABLED(__SAM3X8E__)
    // Initialize some variables only at start!
    if (first_start) {
 	    for (uint8_t i = 0; i < 7; i++) {
        for (int j = 0; j < MEDIAN_COUNT; j++) raw_median_temp[i][j] = RAW_MEDIAN_TEMP_DEFAULT;
        max_temp[i] = 0;
        min_temp[i] = RAW_MIN_TEMP_DEFAULT;
      }
      first_start = false;
      SERIAL_EM("First start for temperature finished.");
    }

    HAL_timer_isr_status (TEMP_TIMER_COUNTER, TEMP_TIMER_CHANNEL);
  #endif

  #if DISABLED(SLOW_PWM_HEATERS)
    /**
     * Standard PWM modulation
     */
    if (pwm_count == 0) {
      soft_pwm_0 = soft_pwm[0];
      WRITE_HEATER_0(soft_pwm_0 > 0 ? 1 : 0);
      #if HOTENDS > 1
        soft_pwm_1 = soft_pwm[1];
        WRITE_HEATER_1(soft_pwm_1 > 0 ? 1 : 0);
        #if HOTENDS > 2
          soft_pwm_2 = soft_pwm[2];
          WRITE_HEATER_2(soft_pwm_2 > 0 ? 1 : 0);
          #if HOTENDS > 3
            soft_pwm_3 = soft_pwm[3];
            WRITE_HEATER_3(soft_pwm_3 > 0 ? 1 : 0);
          #endif
        #endif
      #endif

      #if HAS_HEATER_BED
        soft_pwm_BED = soft_pwm_bed;
        WRITE_HEATER_BED(soft_pwm_BED > 0 ? 1 : 0);
      #endif

      #if HAS(HEATER_CHAMBER) && HAS(TEMP_CHAMBER)
        soft_pwm_CHAMBER = soft_pwm_chamber;
        WRITE_HEATER_CHAMBER(soft_pwm_CHAMBER > 0 ? 1 : 0);
      #endif

      #if HAS(COOLER) && !ENABLED(FAST_PWM_COOLER) && HAS(TEMP_COOLER)
        soft_pwm_COOLER = soft_pwm_cooler;
        WRITE_COOLER(soft_pwm_COOLER > 0 ? 1 : 0);
      #endif 

      #if ENABLED(FAN_SOFT_PWM)
        soft_pwm_fan = fanSpeedSoftPwm >> 1;
        WRITE_FAN(soft_pwm_fan > 0 ? 1 : 0);
      #endif
    }

    if (soft_pwm_0 < pwm_count) WRITE_HEATER_0(0);
    #if HOTENDS > 1
      if (soft_pwm_1 < pwm_count) WRITE_HEATER_1(0);
      #if HOTENDS > 2
        if (soft_pwm_2 < pwm_count) WRITE_HEATER_2(0);
        #if HOTENDS > 3
          if (soft_pwm_3 < pwm_count) WRITE_HEATER_3(0);
        #endif
      #endif
    #endif

    #if HAS_HEATER_BED
      if (soft_pwm_BED < pwm_count) WRITE_HEATER_BED(0);
    #endif

    #if HAS(HEATER_CHAMBER) && HAS(TEMP_CHAMBER)
      if (soft_pwm_CHAMBER < pwm_count) WRITE_HEATER_CHAMBER(0);
    #endif

    #if HAS(COOLER) && !ENABLED(FAST_PWM_COOLER) && HAS(TEMP_COOLER)
      if (soft_pwm_COOLER < pwm_count ) WRITE_COOLER(0);
    #endif

    #if ENABLED(FAN_SOFT_PWM)
      if (soft_pwm_fan < pwm_count) WRITE_FAN(0);
    #endif

    // SOFT_PWM_SCALE to frequency:
    //
    // 0: 16000000/64/256/128 =   7.6294 Hz
    // 1:                / 64 =  15.2588 Hz
    // 2:                / 32 =  30.5176 Hz
    // 3:                / 16 =  61.0352 Hz
    // 4:                /  8 = 122.0703 Hz
    // 5:                /  4 = 244.1406 Hz
    pwm_count += _BV(SOFT_PWM_SCALE);
    pwm_count &= 0x7F;

  #else // SLOW_PWM_HEATERS

    /**
     * SLOW PWM HEATERS
     *
     * For relay-driven heaters
     */
    #if DISABLED(MIN_STATE_TIME)
      #define MIN_STATE_TIME 16 // MIN_STATE_TIME * 65.5 = time in milliseconds
    #endif

    // Macros for Slow PWM timer logic
    #define _SLOW_PWM_ROUTINE(NR, src) \
      soft_pwm_ ## NR = src; \
      if (soft_pwm_ ## NR > 0) { \
        if (state_timer_heater_ ## NR == 0) { \
          if (state_heater_ ## NR == 0) state_timer_heater_ ## NR = MIN_STATE_TIME; \
          state_heater_ ## NR = 1; \
          WRITE_HEATER_ ## NR(1); \
        } \
      } \
      else { \
        if (state_timer_heater_ ## NR == 0) { \
          if (state_heater_ ## NR == 1) state_timer_heater_ ## NR = MIN_STATE_TIME; \
          state_heater_ ## NR = 0; \
          WRITE_HEATER_ ## NR(0); \
        } \
      }
    #define SLOW_PWM_ROUTINE(n) _SLOW_PWM_ROUTINE(n, soft_pwm[n])

    #define PWM_OFF_ROUTINE(NR) \
      if (soft_pwm_ ## NR < slow_pwm_count) { \
        if (state_timer_heater_ ## NR == 0) { \
          if (state_heater_ ## NR == 1) state_timer_heater_ ## NR = MIN_STATE_TIME; \
          state_heater_ ## NR = 0; \
          WRITE_HEATER_ ## NR (0); \
        } \
      }

    if (slow_pwm_count == 0) {

      SLOW_PWM_ROUTINE(0); // HOTEND 0
      #if HOTENDS > 1
        SLOW_PWM_ROUTINE(1); // HOTEND 1
        #if HOTENDS > 2
          SLOW_PWM_ROUTINE(2); // HOTEND 2
          #if HOTENDS > 3
            SLOW_PWM_ROUTINE(3); // HOTEND 3
          #endif
        #endif
      #endif

      #if HAS_HEATER_BED
        _SLOW_PWM_ROUTINE(BED, soft_pwm_bed); // BED
      #endif

      #if HAS(HEATER_CHAMBER)
        _SLOW_PWM_ROUTINE(CHAMBER, soft_pwm_chamber); // CHAMBER
      #endif

      #if HAS(COOLER) && !ENABLED(FAST_PWM_COOLER)
        _SLOW_PWM_ROUTINE(COOLER, soft_pwm_cooler); // COOLER
      #endif

    } // slow_pwm_count == 0

    PWM_OFF_ROUTINE(0); // HOTEND 0
    #if HOTENDS > 1
      PWM_OFF_ROUTINE(1); // HOTEND 1
      #if HOTENDS > 2
        PWM_OFF_ROUTINE(2); // HOTEND 2
        #if HOTENDS > 3
          PWM_OFF_ROUTINE(3); // HOTEND 3
        #endif
      #endif
    #endif

    #if HAS_HEATER_BED
      PWM_OFF_ROUTINE(BED); // BED
    #endif

    #if HAS(HEATER_CHAMBER)
      PWM_OFF_ROUTINE(CHAMBER); // CHAMBER
    #endif

    #if HAS(COOLER) && !ENABLED(FAST_PWM_COOLER)
      PWM_OFF_ROUTINE(COOLER); // COOLER
    #endif 

    #if ENABLED(FAN_SOFT_PWM)
      if (pwm_count == 0) {
        soft_pwm_fan = fanSpeedSoftPwm / 2;
        WRITE_FAN(soft_pwm_fan > 0 ? 1 : 0);
      }
      if (soft_pwm_fan < pwm_count) WRITE_FAN(0);
    #endif // FAN_SOFT_PWM

    // SOFT_PWM_SCALE to frequency:
    //
    // 0: 16000000/64/256/128 =   7.6294 Hz
    // 1:                / 64 =  15.2588 Hz
    // 2:                / 32 =  30.5176 Hz
    // 3:                / 16 =  61.0352 Hz
    // 4:                /  8 = 122.0703 Hz
    // 5:                /  4 = 244.1406 Hz
    pwm_count += _BV(SOFT_PWM_SCALE);
    pwm_count &= 0x7F;

    // increment slow_pwm_count only every 64 pwm_count (e.g., every 8s)
    if ((pwm_count % 64) == 0) {
      slow_pwm_count++;
      slow_pwm_count &= 0x7f;

      // EXTRUDER 0
      if (state_timer_heater_0 > 0) state_timer_heater_0--;
      #if HOTENDS > 1    // EXTRUDER 1
        if (state_timer_heater_1 > 0) state_timer_heater_1--;
        #if HOTENDS > 2    // EXTRUDER 2
          if (state_timer_heater_2 > 0) state_timer_heater_2--;
          #if HOTENDS > 3    // EXTRUDER 3
            if (state_timer_heater_3 > 0) state_timer_heater_3--;
          #endif
        #endif
      #endif

      #if HAS_HEATER_BED
        if (state_timer_heater_BED > 0) state_timer_heater_BED--;
      #endif

      #if HAS(HEATER_CHAMBER)
        if (state_timer_heater_CHAMBER > 0) state_timer_heater_CHAMBER--;
      #endif

      #if HAS(COOLER) && !ENABLED(FAST_PWM_COOLER)
        if(state_timer_heater_COOLER > 0) state_timer_heater_COOLER--;
      #endif 
    } // (pwm_count % 64) == 0

  #endif // SLOW_PWM_HEATERS

  #if ENABLED(__SAM3X8E__)
    #define SET_RAW_TEMP_VALUE(temp_id) temp_read = getAdcFreerun(pinToAdcChannel(TEMP_## temp_id ##_PIN)); \
      raw_temp_value[temp_id] += temp_read; \
      max_temp[temp_id] = max(max_temp[temp_id], temp_read); \
      min_temp[temp_id] = min(min_temp[temp_id], temp_read)

    #define SET_RAW_TEMP_BED_VALUE() temp_read = getAdcFreerun(pinToAdcChannel(TEMP_BED_PIN)); \
      raw_temp_bed_value += temp_read; \
      max_temp[4] = max(max_temp[4], temp_read); \
      min_temp[4] = min(min_temp[4], temp_read)

    #define SET_RAW_TEMP_CHAMBER_VALUE() temp_read = getAdcFreerun(pinToAdcChannel(TEMP_CHAMBER_PIN)); \
      raw_temp_chamber_value += temp_read; \
      max_temp[5] = max(max_temp[5], temp_read); \
      min_temp[5] = min(min_temp[5], temp_read)

    #define SET_RAW_TEMP_COOLER_VALUE() temp_read = getAdcFreerun(pinToAdcChannel(TEMP_COOLER_PIN)); \
      raw_temp_cooler_value += temp_read; \
      max_temp[6] = max(max_temp[6], temp_read); \
      min_temp[6] = min(min_temp[6], temp_read)
  #else
    #define SET_ADMUX_ADCSRA(pin) ADMUX = _BV(REFS0) | (pin & 0x07); SBI(ADCSRA, ADSC)
    #ifdef MUX5
      #define START_ADC(pin) if (pin > 7) ADCSRB = _BV(MUX5); else ADCSRB = 0; SET_ADMUX_ADCSRA(pin)
    #else
      #define START_ADC(pin) ADCSRB = 0; SET_ADMUX_ADCSRA(pin)
    #endif
  #endif

  // Prepare or measure a sensor, each one every 14th frame
  switch (temp_state) {
    case PrepareTemp_0:
      #if HAS(TEMP_0)
        #if ENABLED(__SAM3X8E__)
          // nothing todo for Due
        #else
          START_ADC(TEMP_0_PIN);
        #endif
      #endif
      lcd_buttons_update();
      temp_state = MeasureTemp_0;
      break;
    case MeasureTemp_0:
      #if HAS(TEMP_0)
        #if ENABLED(__SAM3X8E__)
          SET_RAW_TEMP_VALUE(0);
        #else
          raw_temp_value[0] += ADC;
        #endif
      #endif
      temp_state = PrepareTemp_BED;
      break;

    case PrepareTemp_BED:
      #if HAS(TEMP_BED)
        #if ENABLED(__SAM3X8E__)
          // nothing todo for Due
        #else
          START_ADC(TEMP_BED_PIN);
        #endif
      #endif
      lcd_buttons_update();
      temp_state = MeasureTemp_BED;
      break;
    case MeasureTemp_BED:
      #if HAS(TEMP_BED)
        #if ENABLED(__SAM3X8E__)
          SET_RAW_TEMP_BED_VALUE();
        #else
          raw_temp_bed_value += ADC;
        #endif
      #endif
      temp_state = PrepareTemp_1;
      break;

    case PrepareTemp_1:
      #if HAS(TEMP_1)
        #if ENABLED(__SAM3X8E__)
          // nothing todo for Due
        #else
          START_ADC(TEMP_1_PIN);
        #endif
      #endif
      lcd_buttons_update();
      temp_state = MeasureTemp_1;
      break;
    case MeasureTemp_1:
      #if HAS(TEMP_1)
        #if ENABLED(__SAM3X8E__)
          SET_RAW_TEMP_VALUE(1);
        #else
          raw_temp_value[1] += ADC;
        #endif
      #endif
      temp_state = PrepareTemp_2;
      break;

    case PrepareTemp_2:
      #if HAS(TEMP_2)
        #if ENABLED(__SAM3X8E__)
          // nothing todo for Due
        #else
          START_ADC(TEMP_2_PIN);
        #endif
      #endif
      lcd_buttons_update();
      temp_state = MeasureTemp_2;
      break;
    case MeasureTemp_2:
      #if HAS(TEMP_2)
        #if ENABLED(__SAM3X8E__)
          SET_RAW_TEMP_VALUE(2);
        #else
          raw_temp_value[2] += ADC;
        #endif
      #endif
      temp_state = PrepareTemp_3;
      break;

    case PrepareTemp_3:
      #if HAS(TEMP_3)
        #if ENABLED(__SAM3X8E__)
          // nothing todo for Due
        #else
          START_ADC(TEMP_3_PIN);
        #endif
      #endif
      lcd_buttons_update();
      temp_state = MeasureTemp_3;
      break;
    case MeasureTemp_3:
      #if HAS(TEMP_3)
        #if ENABLED(__SAM3X8E__)
          SET_RAW_TEMP_VALUE(3);
        #else
          raw_temp_value[3] += ADC;
        #endif
      #endif
      temp_state = PrepareTemp_CHAMBER;
      break;

    case PrepareTemp_CHAMBER:
      #if HAS(TEMP_CHAMBER)
        #if ENABLED(__SAM3X8E__)
          // nothing todo for Due
        #else
          START_ADC(TEMP_CHAMBER_PIN);
        #endif
      #endif
      lcd_buttons_update();
      temp_state = MeasureTemp_CHAMBER;
      break;
    case MeasureTemp_CHAMBER:
      #if HAS(TEMP_CHAMBER)
        #if ENABLED(__SAM3X8E__)
          SET_RAW_TEMP_CHAMBER_VALUE();
        #else
          raw_temp_chamber_value += ADC;
        #endif
      #endif
      temp_state = PrepareTemp_COOLER;
      break;

    case PrepareTemp_COOLER:
      #if HAS(TEMP_COOLER)
        #if ENABLED(__SAM3X8E__)
          // nothing todo for Due
        #else
          START_ADC(TEMP_COOLER_PIN);
        #endif
      #endif
      lcd_buttons_update();
      temp_state = MeasureTemp_COOLER;
      break;
    case MeasureTemp_COOLER:
      #if HAS(TEMP_COOLER)
        #if ENABLED(__SAM3X8E__)
          SET_RAW_TEMP_COOLER_VALUE();
        #else
          raw_temp_cooler_value += ADC;
        #endif
      #endif
      temp_state = Prepare_FILWIDTH;
      break;

    case Prepare_FILWIDTH:
      #if HAS(FILAMENT_SENSOR)
        #if ENABLED(__SAM3X8E__)
          // nothing todo for Due
        #else
          START_ADC(FILWIDTH_PIN);
        #endif
      #endif
      lcd_buttons_update();
      temp_state = Measure_FILWIDTH;
      break;
    case Measure_FILWIDTH:
      #if HAS(FILAMENT_SENSOR)
        // raw_filwidth_value += ADC;  //remove to use an IIR filter approach
        if (ADC > 102) { //check that ADC is reading a voltage > 0.5 volts, otherwise don't take in the data.
          raw_filwidth_value -= (raw_filwidth_value >> 7); //multiply raw_filwidth_value by 127/128
          raw_filwidth_value += ((unsigned long)ADC << 7); //add new ADC reading
        }
      #endif
      temp_state = Prepare_POWCONSUMPTION;
      break;

    case Prepare_POWCONSUMPTION:
      #if HAS(POWER_CONSUMPTION_SENSOR)
        #if ENABLED(__SAM3X8E__)
          // nothing todo for Due
        #else
          START_ADC(POWER_CONSUMPTION_PIN);
        #endif
      #endif
      lcd_buttons_update();
      temp_state = Measure_POWCONSUMPTION;
      break;
    case Measure_POWCONSUMPTION:
      #if HAS(POWER_CONSUMPTION_SENSOR)
        #if ENABLED(__SAM3X8E__)
          raw_powconsumption_value = analogRead(POWER_CONSUMPTION_PIN);
        #else
          raw_powconsumption_value += ADC;
        #endif
      #endif
      temp_state = PrepareTemp_0;
      temp_count++;
      break;

    case StartupDelay:
      temp_state = PrepareTemp_0;
      break;

    // default:
    //  SERIAL_LM(ER, MSG_TEMP_READ_ERROR);
    //  break;
  } // switch(temp_state)

  #if ENABLED(__SAM3X8E__)
    if (temp_count >= OVERSAMPLENR + 2) { // 14 * 16 * 1/(16000000/64/256)  = 164ms.
  #else
    if (temp_count >= OVERSAMPLENR) { // 10 * 16 * 1/(16000000/64/256)  = 164ms.
  #endif

    // Update the raw values if they've been read. Else we could be updating them during reading.
    if (!temp_meas_ready) set_current_temp_raw();

    // Filament Sensor - can be read any time since IIR filtering is used
    #if ENABLED(FILAMENT_SENSOR)
      current_raw_filwidth = raw_filwidth_value >> 10;  // Divide to get to 0-16384 range since we used 1/128 IIR filter approach
    #endif

    temp_count = 0;
    ZERO(raw_temp_value);
    raw_temp_bed_value = 0;

    #if HAS(TEMP_CHAMBER)
      raw_temp_chamber_value = 0;
    #endif
    #if HAS(TEMP_COOLER)
      raw_temp_cooler_value = 0;
    #endif

    #if HAS(POWER_CONSUMPTION_SENSOR)
      raw_powconsumption_value = 0;
    #endif

    int constexpr temp_dir[] = {
      #if ENABLED(HEATER_0_USES_MAX6675)
         0
      #elif HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
        -1
      #else
         1
      #endif
      #if HAS(TEMP_1) && HOTENDS > 1
        #if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
          , -1
        #else
          ,  1
        #endif
      #endif
      #if HAS(TEMP_2) && HOTENDS > 2
        #if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
          , -1
        #else
          ,  1
        #endif
      #endif
      #if HAS(TEMP_3) && HOTENDS > 3
        #if HEATER_3_RAW_LO_TEMP > HEATER_3_RAW_HI_TEMP
          , -1
        #else
          ,  1
        #endif
      #endif
    };

    for (uint8_t h = 0; h < COUNT(temp_dir); h++) {
      const int tdir = temp_dir[h], rawtemp = current_temperature_raw[h] * tdir;
      if (rawtemp > maxttemp_raw[h] * tdir && target_temperature[h] > 0.0f) max_temp_error(h);
      if (rawtemp < minttemp_raw[h] * tdir && !is_preheating(h) && target_temperature[h] > 0.0f) {
        #if ENABLED(MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
          if (++consecutive_low_temperature_error[h] >= MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
        #endif
            min_temp_error(h);
      }
      #if ENABLED(MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
        else
          consecutive_low_temperature_error[h] = 0;
      #endif
    }

    #if HAS_TEMP_BED
      #if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
        #define GEBED <=
      #else
        #define GEBED >=
      #endif
      if (current_temperature_bed_raw GEBED bed_maxttemp_raw) max_temp_error(-1);
      if (bed_minttemp_raw GEBED current_temperature_bed_raw && target_temperature_bed > 0.0f) min_temp_error(-1);
    #endif

    #if HAS(TEMP_CHAMBER)
      #if HEATER_CHAMBER_RAW_LO_TEMP > HEATER_CHAMBER_RAW_HI_TEMP
        #define GECHAMBER <=
      #else
        #define GECHAMBER >=
      #endif
      if (current_temperature_chamber_raw GECHAMBER chamber_maxttemp_raw) _temp_error(-2, MSG_T_MAXTEMP, PSTR(MSG_ERR_MAXTEMP_CHAMBER));
      if (chamber_minttemp_raw GECHAMBER current_temperature_chamber_raw && target_temperature_chamber > 0.0f) _temp_error(-2, MSG_T_MINTEMP, PSTR(MSG_ERR_MINTEMP_CHAMBER));
    #endif

    #if HAS(TEMP_COOLER)
      #if COOLER_RAW_LO_TEMP > COOLER_RAW_HI_TEMP
        #define GECOOLER <=
      #else
        #define GECOOLER >=
      #endif
      if (current_temperature_cooler_raw GECOOLER cooler_maxttemp_raw) _temp_error(-3, MSG_T_MAXTEMP, PSTR(MSG_ERR_MAXTEMP_COOLER));
      if (cooler_minttemp_raw GECOOLER current_temperature_cooler_raw && target_temperature_cooler > 0.0f) _temp_error(-3, MSG_T_MINTEMP, PSTR(MSG_ERR_MINTEMP_COOLER));
    #endif

  } // temp_count >= OVERSAMPLENR

  #if ENABLED(BABYSTEPPING)
    LOOP_XYZ(axis) {
      int curTodo = babystepsTodo[axis]; //get rid of volatile for performance

      if (curTodo > 0) {
        stepper.babystep((AxisEnum)axis,/*fwd*/true);
        babystepsTodo[axis]--; //fewer to do next time
      }
      else if (curTodo < 0) {
        stepper.babystep((AxisEnum)axis,/*fwd*/false);
        babystepsTodo[axis]++; //fewer to do next time
      }
    }
  #endif //BABYSTEPPING

  #if ENABLED(PINS_DEBUGGING)
    extern bool endstop_monitor_flag;
    // run the endstop monitor at 15Hz
    static uint8_t endstop_monitor_count = 16;  // offset this check from the others
    if (endstop_monitor_flag) {
      endstop_monitor_count += _BV(1);  //  15 Hz
      endstop_monitor_count &= 0x7F;
      if (!endstop_monitor_count) endstop_monitor();  // report changes in endstop status
    }
  #endif
  
  ENABLE_TEMPERATURE_INTERRUPT(); // re-enable Temperature ISR
}

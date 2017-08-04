/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  static void* heater_ttbl_map[2] = {(void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE };
  static uint8_t heater_ttbllen_map[2] = { HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN };
#elif HAS_TEMP_HOTEND
  static void* heater_ttbl_map[HOTENDS] = ARRAY_BY_HOTENDS_N((void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE, (void*)HEATER_2_TEMPTABLE, (void*)HEATER_3_TEMPTABLE);
  static uint8_t heater_ttbllen_map[HOTENDS] = ARRAY_BY_HOTENDS_N(HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN, HEATER_3_TEMPTABLE_LEN);
#endif

Temperature thermalManager;

// public:
volatile bool Temperature::wait_for_heatup = true;

#if HAS_TEMP_HOTEND
  float   Temperature::current_temperature[HOTENDS]     = { 0.0 };
  int16_t Temperature::current_temperature_raw[HOTENDS] = { 0 },
          Temperature::target_temperature[HOTENDS]      = { 0 };
#endif

#if HAS_TEMP_BED
  float   Temperature::current_temperature_bed          = 0.0;
  int16_t Temperature::current_temperature_bed_raw      = 0,
          Temperature::target_temperature_bed           = 0;
#endif

#if HAS_TEMP_CHAMBER
  float   Temperature::current_temperature_chamber      = 0.0;
  int16_t Temperature::target_temperature_chamber       = 0,
          Temperature::current_temperature_chamber_raw  = 0;
#endif

#if HAS_TEMP_COOLER
  float   Temperature::current_temperature_cooler     = 0.0;
  int16_t Temperature::target_temperature_cooler      = 0,
          Temperature::current_temperature_cooler_raw = 0;
#endif

#if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
  float   Temperature::current_temperature_mcu  = 0.0,
          Temperature::highest_temperature_mcu  = 0.0,
          Temperature::lowest_temperature_mcu   = 4096.0,
          Temperature::alarm_temperature_mcu    = 80.0;
  int16_t Temperature::current_temperature_mcu_raw;
#endif

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  float Temperature::redundant_temperature = 0.0;
#endif

#if HAS_TEMP_HOTEND
  uint8_t Temperature::soft_pwm[HOTENDS];
#endif
#if HAS_TEMP_BED
  uint8_t Temperature::soft_pwm_bed;
#endif
#if HAS_TEMP_CHAMBER
  uint8_t Temperature::soft_pwm_chamber;
#endif
#if HAS_TEMP_COOLER
  uint8_t Temperature::soft_pwm_cooler;
#endif

#if ENABLED(PIDTEMP)
  float Temperature::Kp[HOTENDS],
        Temperature::Ki[HOTENDS],
        Temperature::Kd[HOTENDS],
        Temperature::Kc[HOTENDS];
#endif

#if ENABLED(PIDTEMPBED)
  float Temperature::bedKp = DEFAULT_bedKp,
        Temperature::bedKi = DEFAULT_bedKi,
        Temperature::bedKd = DEFAULT_bedKd;
#endif

#if ENABLED(PIDTEMPCHAMBER)
  float Temperature::chamberKp = DEFAULT_chamberKp,
        Temperature::chamberKi = DEFAULT_chamberKi,
        Temperature::chamberKd = DEFAULT_chamberKd;
#endif

#if ENABLED(PIDTEMPCOOLER)
  float Temperature::coolerKp = DEFAULT_coolerKp,
        Temperature::coolerKi = DEFAULT_coolerKi,
        Temperature::coolerKd = DEFAULT_coolerKd;
#endif

#if HAS(AUTO_FAN)
  uint8_t Temperature::autoFanSpeeds[HOTENDS] = { 0 };
#endif

#if ENABLED(BABYSTEPPING)
  volatile int Temperature::babystepsTodo[XYZ] = { 0 };
#endif

#if WATCH_HOTENDS
  uint16_t Temperature::watch_target_temp[HOTENDS]    = { 0 };
  millis_t Temperature::watch_heater_next_ms[HOTENDS] = { 0 };
#endif

#if WATCH_THE_BED
  uint16_t Temperature::watch_target_bed_temp = 0;
  millis_t Temperature::watch_bed_next_ms     = 0;
#endif

#if WATCH_THE_CHAMBER
  uint16_t Temperature::watch_target_temp_chamber = 0;
  millis_t Temperature::watch_chamber_next_ms     = 0;
#endif

#if WATCH_THE_COOLER
  uint16_t Temperature::watch_target_temp_cooler  = 0;
  millis_t Temperature::watch_cooler_next_ms      = 0;
#endif

#if HAS_TEMP_HOTEND && ENABLED(PREVENT_COLD_EXTRUSION)
  bool    Temperature::allow_cold_extrude = false;
  int16_t Temperature::extrude_min_temp   = EXTRUDE_MINTEMP;
#endif

#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
  uint8_t   Temperature::auto_report_temp_interval  = 0;
  millis_t  Temperature::next_temp_report_ms        = 0;
#endif

#if HEATER_USES_AD595
  float Temperature::ad595_offset[HOTENDS]  = ARRAY_BY_HOTENDS(TEMP_SENSOR_AD595_OFFSET),
        Temperature::ad595_gain[HOTENDS]    = ARRAY_BY_HOTENDS(TEMP_SENSOR_AD595_GAIN);
#endif

// private:

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  int   Temperature::redundant_temperature_raw  = 0;
  float Temperature::redundant_temperature      = 0.0;
#endif

#if ENABLED(PIDTEMP)
  float Temperature::temp_iState[HOTENDS]     = { 0.0 },
        Temperature::temp_dState[HOTENDS][4]  = { 0.0 },
        Temperature::temp_iState_min[HOTENDS] = { 0.0 },
        Temperature::temp_iState_max[HOTENDS] = { 0.0 },
        Temperature::pid_error[HOTENDS];

  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    float Temperature::cTerm[HOTENDS];
    long  Temperature::last_e_position,
          Temperature::lpq[LPQ_MAX_LEN];
    int   Temperature::lpq_ptr = 0,
          Temperature::lpq_len = 20;
  #endif

  uint8_t Temperature::pid_pointer[HOTENDS] = { 0 };
#endif

#if ENABLED(PIDTEMPBED)
  float Temperature::temp_iState_bed = 0,
        Temperature::temp_dState_bed[4] = { 0 },
        Temperature::temp_iState_bed_min,
        Temperature::temp_iState_bed_max,
        Temperature::pid_error_bed;

  uint8_t Temperature::pid_pointer_bed = 0;
#else
  millis_t Temperature::next_bed_check_ms;
#endif

#if ENABLED(PIDTEMPCHAMBER)
  float Temperature::temp_iState_chamber = 0,
        Temperature::temp_dState_chamber[4] = { 0 },
        Temperature::temp_iState_chamber_min,
        Temperature::temp_iState_chamber_max,
        Temperature::pid_error_chamber;

  uint8_t Temperature::pid_pointer_chamber = 0;
#else
  millis_t Temperature::next_chamber_check_ms;
#endif

#if ENABLED(PIDTEMPCOOLER)
  float Temperature::temp_iState_cooler = 0,
        Temperature::temp_dState_cooler[4] = { 0 },
        Temperature::temp_iState_cooler_min,
        Temperature::temp_iState_cooler_max,
        Temperature::pid_error_cooler;

  uint8_t Temperature::pid_pointer_cooler = 0;
#else
  millis_t Temperature::next_cooler_check_ms;
#endif

// Init min and max temp with extreme values to prevent false errors during startup
int16_t Temperature::minttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS_N(HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP, HEATER_3_RAW_LO_TEMP),
        Temperature::maxttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS_N(HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP, HEATER_3_RAW_HI_TEMP),
        Temperature::minttemp[HOTENDS] = ARRAY_BY_HOTENDS(0),
        Temperature::maxttemp[HOTENDS] = ARRAY_BY_HOTENDS(16383);

#if ENABLED(MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
  uint8_t Temperature::consecutive_low_temperature_error[HOTENDS] = { 0 };
  #if HAS_TEMP_BED
    uint8_t Temperature::consecutive_bed_low_temperature_error = 0;
  #endif
#endif

#if ENABLED(MILLISECONDS_PREHEAT_TIME)
  millis_t Temperature::preheat_end_time[HOTENDS] = { 0 };
#endif

#if ENABLED(BED_MINTEMP)
  int16_t Temperature::bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP;
#endif

#if ENABLED(BED_MAXTEMP)
  int16_t Temperature::bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif

#if ENABLED(CHAMBER_MINTEMP)
  int16_t Temperature::chamber_minttemp_raw = HEATER_CHAMBER_RAW_LO_TEMP;
#endif
#if ENABLED(CHAMBER_MAXTEMP)
  int16_t Temperature::chamber_maxttemp_raw = HEATER_CHAMBER_RAW_HI_TEMP;
#endif
#if ENABLED(COOLER_MINTEMP)
  int16_t Temperature::cooler_minttemp_raw = COOLER_RAW_LO_TEMP;
#endif
#if ENABLED(COOLER_MAXTEMP)
  int16_t Temperature::cooler_maxttemp_raw = COOLER_RAW_HI_TEMP;
#endif

#if ENABLED(FILAMENT_SENSOR)
  int8_t    Temperature::meas_shift_index;          // Index of a delayed sample in buffer
  uint16_t  Temperature::current_raw_filwidth = 0;  // Measured filament diameter - one extruder only
#endif

#if HAS_AUTO_FAN
  millis_t Temperature::next_auto_fan_check_ms = 0;
#endif

#if ENABLED(PROBING_HEATERS_OFF)
  bool Temperature::paused;
#endif

#if HEATER_IDLE_HANDLER
  millis_t Temperature::heater_idle_timeout_ms[HOTENDS] = { 0 };
  bool Temperature::heater_idle_timeout_exceeded[HOTENDS] = { false };
  #if HAS_TEMP_BED
    millis_t Temperature::bed_idle_timeout_ms = 0;
    bool Temperature::bed_idle_timeout_exceeded = false;
  #endif
#endif

#if HAS_PID_HEATING || HAS_PID_COOLING

  void Temperature::PID_autotune(const float temp, const int temp_controller, int ncycles, bool storeValues/*=false*/) {

    float currentTemp = 0.0;
    int cycles = 0;
    bool heating = true;

    millis_t temp_ms = millis(), t1 = temp_ms, t2 = temp_ms;
    int32_t t_high = 0, t_low = 0;

    int32_t bias, d;
    float Ku, Tu;
    float workKp = 0, workKi = 0, workKd = 0;
    float maxTemp = 20, minTemp = 20;
    uint8_t pidMax = 0;

    NOLESS(ncycles, 5);
    NOMORE(ncycles, 20);

    #if HAS_AUTO_FAN
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
      SERIAL_MSG("BED");
    }
    else if(temp_controller == -2) {
      SERIAL_MSG("CHAMBER");
    }
    else if(temp_controller == -3) {
      SERIAL_MSG("COOLER");
    }
    else {
      SERIAL_MV("Hotend: ", temp_controller);
    }
    SERIAL_MV(" Temp: ", temp);
    SERIAL_MV(" Cycles: ", ncycles);
    if (storeValues)
      SERIAL_EM(" Apply result");
    else
      SERIAL_EOL();

    disable_all_heaters(); // switch off all heaters.
    #if HAS_TEMP_COOLER
      disable_all_coolers(); // switch off all coolers.
    #endif

    #if HAS_TEMP_BED
      if (temp_controller == -1) {
        pidMax = MAX_BED_POWER;
        soft_pwm_bed = (pidMax);
      }
    #endif
    #if HAS_TEMP_CHAMBER
      if (temp_controller == -2) {
        pidMax = MAX_CHAMBER_POWER;
        soft_pwm_chamber = (pidMax);
      }
    #endif
    #if HAS_TEMP_COOLER
      if (temp_controller == -3) {
        pidMax = MAX_COOLER_POWER;
        soft_pwm_cooler = (pidMax);
      }
    #endif
    if (temp_controller >= 0) {
      pidMax = PID_MAX;
      soft_pwm[temp_controller] = (pidMax);
    }

    bias = pidMax >> 1;
    d = pidMax >> 1;

    wait_for_heatup = true;

    // PID Tuning loop
    while (wait_for_heatup) {

      updateTemperaturesFromRawValues();
      millis_t ms = millis();

      #if HAS_TEMP_BED
        if (temp_controller == -1)
          currentTemp = current_temperature_bed;
      #endif
      #if HAS_TEMP_CHAMBER
        if (temp_controller == -2)
          currentTemp = current_temperature_chamber;
      #endif
      #if HAS_TEMP_COOLER
        if (temp_controller == -3)
        currentTemp = current_temperature_cooler;
      #endif
      if (temp_controller >= 0)
        currentTemp = current_temperature[temp_controller];

      NOLESS(maxTemp, currentTemp);
      NOMORE(minTemp, currentTemp);

      #if HAS_AUTO_FAN
        if (ELAPSED(ms, next_auto_fan_check_ms)) {
          checkExtruderAutoFans();
          next_auto_fan_check_ms = ms + 2500UL;
        }
      #endif

      if (heating && currentTemp > temp) {
        if (ELAPSED(ms, t2 + 2500UL)) {
          heating = false;

          #if HAS_TEMP_BED
            if (temp_controller == -1)
              soft_pwm_bed = (bias - d);
          #endif
          #if HAS_TEMP_CHAMBER
            if (temp_controller == -2)
              soft_pwm_chamber = (bias - d);
          #endif
          #if HAS_TEMP_COOLER
            if (temp_controller == -3)
              soft_pwm_cooler = (bias - d);
          #endif
          if (temp_controller >= 0)
            soft_pwm[temp_controller] = (bias - d);

          t1 = ms;
          t_high = t1 - t2;

          #if HAS_TEMP_COOLER
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

            bias += (d * (t_high - t_low)) / (t_low + t_high);
            bias = constrain(bias, 20, pidMax - 20);
            d = (bias > pidMax / 2) ? pidMax - 1 - bias : bias;

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
              soft_pwm_cooler = (bias + d);
          #endif

          cycles++;

          #if HAS_TEMP_COOLER
            if (temp_controller == -3)
              maxTemp = temp;
            else
          #endif
            minTemp = temp;
        }
      }

      #define MAX_OVERSHOOT_PID_AUTOTUNE 40
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
          SERIAL_EOL();
        #endif
        #if HAS_TEMP_CHAMBER
          print_chamberstate();
          SERIAL_EOL();
        #endif
        #if HAS_TEMP_COOLER
          print_coolerstate();
          SERIAL_EOL();
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
              PID_PARAM(Ki, temp_controller) = workKi;
              PID_PARAM(Kd, temp_controller) = workKd;
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
              bedKi = workKi;
              bedKd = workKd;
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
              coolerKi = workKi;
              coolerKd = workKd;
              updatePID();
            }
          }
        #endif

        return;
      }
    }

    disable_all_heaters();

    #if HAS_TEMP_COOLER
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
    LOOP_HOTEND() {
      temp_iState_min[HOTEND_INDEX] = (float)PID_MIN * 10.0f / PID_PARAM(Ki, HOTEND_INDEX);
      temp_iState_max[HOTEND_INDEX] = (float)PID_MAX * 10.0f / PID_PARAM(Ki, HOTEND_INDEX);
    }
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      last_e_position = 0;
    #endif
  #endif

  #if ENABLED(PIDTEMPBED)
    temp_iState_bed_min = (float)MIN_BED_POWER * 10.0f / bedKi;
    temp_iState_bed_max = (float)MAX_BED_POWER * 10.0f / bedKi;
  #endif

  #if ENABLED(PIDTEMPCHAMBER)
    temp_iState_chamber_min = (float)MIN_CHAMBER_POWER * 10.0f / chamberKi;
    temp_iState_chamber_max = (float)MAX_CHAMBER_POWER * 10.0f / chamberKi;
  #endif

  #if ENABLED(PIDTEMPCOOLER)
    temp_iState_cooler_min = (float)MIN_COOLER_POWER * 10.0f / coolerKi;
    temp_iState_cooler_max = (float)MAX_COOLER_POWER * 10.0f / coolerKi;
  #endif

}

#if HAS_AUTO_FAN

  void Temperature::checkExtruderAutoFans() {
    const int8_t fanPin[] = { H0_AUTO_FAN_PIN, H1_AUTO_FAN_PIN, H2_AUTO_FAN_PIN, H3_AUTO_FAN_PIN };
    const int fanBit[] = {
                    0,
      AUTO_1_IS_0 ? 0 :               1,
      AUTO_2_IS_0 ? 0 : AUTO_2_IS_1 ? 1 :               2,
      AUTO_3_IS_0 ? 0 : AUTO_3_IS_1 ? 1 : AUTO_3_IS_2 ? 2 : 3
    };
    uint8_t fanState = 0;
 
    LOOP_HOTEND() {
      if (current_temperature[h] > HOTEND_AUTO_FAN_TEMPERATURE)
        SBI(fanState, fanBit[h]);
    }
 
    uint8_t fanDone = 0;
    for (uint8_t f = 0; f < COUNT(fanPin); f++) {
      int8_t pin = fanPin[f];
      if (pin >= 0 && !TEST(fanDone, fanBit[f])) {
        autoFanSpeeds[f] = TEST(fanState, fanBit[f]) ? HOTEND_AUTO_FAN_SPEED : 0;
        // this idiom allows both digital and PWM fan outputs (see M42 handling).
        WRITE_AUTO_FAN(pin, autoFanSpeeds[f]);
        SBI(fanDone, fanBit[f]);
      }
    }
  }

#endif // HAS_AUTO_FAN

//
// Temperature Error Handlers
//
void Temperature::_temp_error(const int8_t tc, const char * const serial_msg, const char * const lcd_msg) {
  static bool killed = false;
  if (printer.IsRunning()) {
    SERIAL_ST(ER, serial_msg);
    SERIAL_MSG(MSG_STOPPED_HEATER);
    if (tc >= 0)
      SERIAL_EV((int)tc);
    #if HAS_TEMP_BED
      else if (tc == -1)
        SERIAL_EM(MSG_HEATER_BED);
    #endif
    #if HAS_TEMP_CHAMBER
      else if (tc == -2)
        SERIAL_EM(MSG_HEATER_CHAMBER);
    #endif
    #if HAS_TEMP_COOLER
      else if (tc == -3)
        SERIAL_EM(MSG_HEATER_COOLER);
    #endif
  }

  #if DISABLED(BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE)
    if (!killed) {
      printer.setRunning(false);
      killed = true;
      printer.kill(lcd_msg);
    }
    else {
      disable_all_heaters();
      #if HAS_TEMP_COOLER
        disable_all_coolers();
      #endif
    }
  #endif
}

void Temperature::max_temp_error(const int8_t h) {
  #if HAS_TEMP_BED
    _temp_error(h, PSTR(MSG_T_MAXTEMP), h >= 0 ? PSTR(MSG_ERR_MAXTEMP) : PSTR(MSG_ERR_MAXTEMP_BED));
  #else
    _temp_error(HOTEND_INDEX, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP));
    #if HOTENDS <= 1
      UNUSED(h);
    #endif
  #endif
}
void Temperature::min_temp_error(const int8_t h) {
  #if HAS_TEMP_BED
    _temp_error(h, PSTR(MSG_T_MINTEMP), h >= 0 ? PSTR(MSG_ERR_MINTEMP) : PSTR(MSG_ERR_MINTEMP_BED));
  #else
    _temp_error(HOTEND_INDEX, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP));
    #if HOTENDS <= 1
      UNUSED(h);
    #endif
  #endif
}

uint8_t Temperature::get_pid_output(const int8_t h) {
  #if HOTENDS <= 1
    UNUSED(h);
    #define _HOTEND_TEST  true
  #else
    #define _HOTEND_TEST  h == tools.active_extruder
  #endif

  uint8_t pid_output = 0;

  #if ENABLED(PIDTEMP)
    #if DISABLED(PID_OPENLOOP)
      temp_dState[HOTEND_INDEX][pid_pointer[HOTEND_INDEX]++] = current_temperature[HOTEND_INDEX];
      pid_pointer[HOTEND_INDEX] &= 3;
      float error = target_temperature[HOTEND_INDEX] - current_temperature[HOTEND_INDEX];
      #if HEATER_IDLE_HANDLER
        if (heater_idle_timeout_exceeded[HOTEND_INDEX]) {
          pid_output = 0;
          temp_iState[HOTEND_INDEX] = 0;
        }
        else
      #endif
      if (error > PID_FUNCTIONAL_RANGE) {
        pid_output = PID_MAX;
      }
      else if (error < -(PID_FUNCTIONAL_RANGE) || target_temperature[HOTEND_INDEX] == 0 
        #if HEATER_IDLE_HANDLER
          || heater_idle_timeout_exceeded[HOTEND_INDEX]
        #endif
      ) {
        pid_output = 0;
      }
      else {
        float pidTerm = PID_PARAM(Kp, HOTEND_INDEX) * error;
        temp_iState[HOTEND_INDEX] = constrain(temp_iState[HOTEND_INDEX] + error, temp_iState_min[HOTEND_INDEX], temp_iState_max[HOTEND_INDEX]);
        pidTerm += PID_PARAM(Ki, HOTEND_INDEX) * temp_iState[HOTEND_INDEX] * 0.1; // 0.1 = 10Hz
        float dgain = PID_PARAM(Kd, HOTEND_INDEX) * (temp_dState[HOTEND_INDEX][pid_pointer[HOTEND_INDEX]] - current_temperature[HOTEND_INDEX]) * 3.333f;
        pidTerm += dgain;

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
            cTerm[HOTEND_INDEX] = (lpq[lpq_ptr] * mechanics.steps_to_mm[E_AXIS]) * PID_PARAM(Kc, HOTEND_INDEX);
            pidTerm += cTerm[HOTEND_INDEX];
          }
        #endif // PID_ADD_EXTRUSION_RATE

        pid_output = constrain((int)pidTerm, 0, PID_MAX);

      }
    #else
      pid_output = constrain((int)target_temperature[HOTEND_INDEX], 0, PID_MAX);
    #endif // PID_OPENLOOP

    #if ENABLED(PID_DEBUG)
      SERIAL_SMV(ECHO, MSG_PID_DEBUG, HOTEND_INDEX);
      SERIAL_MV(MSG_PID_DEBUG_INPUT, current_temperature[HOTEND_INDEX]);
      SERIAL_EMV(MSG_PID_DEBUG_OUTPUT, pid_output);
    #endif // PID_DEBUG

  #elif HAS_TEMP_HOTEND /* PID off and have Hotend*/
    #if HEATER_IDLE_HANDLER
      if (heater_idle_timeout_exceeded[HOTEND_INDEX])
        pid_output = 0;
      else
    #endif
    pid_output = (current_temperature[HOTEND_INDEX] < target_temperature[HOTEND_INDEX]) ? PID_MAX : 0;
  #endif

  return pid_output;
}

#if ENABLED(PIDTEMPBED)
  uint8_t Temperature::get_pid_output_bed() {
    uint8_t pid_output = 0;
    #if DISABLED(PID_OPENLOOP)
      temp_dState_bed[pid_pointer_bed++] = current_temperature_bed;
      pid_pointer_bed &= 3;
      float error = target_temperature_bed - current_temperature_bed;
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        if (bed_idle_timeout_exceeded) {
          pid_output = 0;
          temp_iState_bed = 0;
        }
        else
      #endif
      if (error > PID_FUNCTIONAL_RANGE) {
        pid_output = MAX_BED_POWER;
      }
      else if (error < -(PID_FUNCTIONAL_RANGE) || target_temperature_bed == 0) {
        pid_output = 0;
      }
      else {
        float pidTerm = bedKp * error;
        temp_iState_bed = constrain(temp_iState_bed + error, temp_iState_bed_min, temp_iState_bed_max);
        pidTerm += bedKi * temp_iState_bed * 0.1;
        float dgain = bedKd * (temp_dState_bed[pid_pointer_bed] - current_temperature_bed) * 3.333f;
        pidTerm += dgain;

        pid_output = constrain((int)pidTerm, 0, MAX_BED_POWER);
      }
    #else
      pid_output = constrain((int)target_temperature_bed, 0, MAX_BED_POWER);
    #endif // PID_OPENLOOP

    #if ENABLED(PID_BED_DEBUG)
      SERIAL_SM(DEB ," PID_BED_DEBUG ");
      SERIAL_MV(MSG_PID_DEBUG_INPUT, current_temperature_bed);
      SERIAL_EMV(MSG_PID_DEBUG_OUTPUT, pid_output);
    #endif //PID_BED_DEBUG

    return pid_output;
  }
#endif //PIDTEMPBED

#if ENABLED(PIDTEMPCHAMBER)
  uint8_t Temperature::get_pid_output_chamber() {
    uint8_t pid_output;

    #if DISABLED(PID_OPENLOOP)
      temp_dState_chamber[pid_pointer_chamber++] = current_temperature_chamber;
      pid_pointer_chamber &= 3;
      float error = target_temperature_chamber - current_temperature_chamber;
      if (error > PID_FUNCTIONAL_RANGE) {
        pid_output = MAX_CHAMBER_POWER;
      }
      else if (error < -(PID_FUNCTIONAL_RANGE) || target_temperature_chamber == 0) {
        pid_output = 0;
      }
      else {
        float pidTerm = chamberKp * error;
        temp_iState_chamber = constrain(temp_iState_chamber + error, temp_iState_chamber_min, temp_iState_chamber_max);
        pidTerm += chamberKi * temp_iState_chamber * 0.1;
        float dgain = chamberKd * (temp_dState_chamber[pid_pointer_chamber] - current_temperature_chamber) * 3.333f;
        pidTerm += dgain;

        pid_output = constrain((int)pidTerm, 0, MAX_CHAMBER_POWER);
      }
    #else
      pid_output = constrain((int)target_temperature_chamber, 0, MAX_CHAMBER_POWER);
    #endif // PID_OPENLOOP

    #if ENABLED(PID_CHAMBER_DEBUG)
      SERIAL_SM(DEB ," PID_CHAMBER_DEBUG ");
      SERIAL_MV(MSG_PID_DEBUG_INPUT, current_temperature_chamber);
      SERIAL_EMV(MSG_PID_DEBUG_OUTPUT, pid_output);
    #endif // PID_CHAMBER_DEBUG

    return pid_output;
  }
#endif

#if ENABLED(PIDTEMPCOOLER)
  uint8_t Temperature::get_pid_output_cooler() {
    uint8_t pid_output;

    // We need this cause 0 is lower than our current temperature probably.
    if (target_temperature_cooler < COOLER_MINTEMP)
      return 0.0;

    #if DISABLED(PID_OPENLOOP)
      temp_dState_cooler[pid_pointer_cooler++] = current_temperature_cooler;
      pid_pointer_cooler &= 3;
      float error = target_temperature_cooler - current_temperature_cooler;
      if (error > PID_FUNCTIONAL_RANGE) {
        pid_output = MAX_COOLER_POWER;
      }
      else if (error < -(PID_FUNCTIONAL_RANGE) || target_temperature_cooler == 0) {
        pid_output = 0;
      }
      else {
        float pidTerm = coolerKp * error;
        temp_iState_cooler = constrain(temp_iState_cooler + error, temp_iState_cooler_min, temp_iState_cooler_max);
        pidTerm += coolerKi * temp_iState_cooler * 0.1;
        float dgain = coolerKd * (temp_dState_cooler[pid_pointer_cooler] - current_temperature_cooler) * 3.333f;
        pidTerm += dgain;

        pid_output = constrain((int)pidTerm, 0, MAX_COOLER_POWER);
      }
    #else
      pid_output = constrain((int)target_temperature_cooler, 0, MAX_COOLER_POWER);
    #endif // PID_OPENLOOP

    #if ENABLED(PID_COOLER_DEBUG)
      SERIAL_SM(DEB ," PID_COOLER_DEBUG ");
      SERIAL_MV(MSG_PID_DEBUG_INPUT, current_temperature_cooler);
      SERIAL_EMV(MSG_PID_DEBUG_OUTPUT, pid_output);
    #endif //PID_COOLER_DEBUG

    return pid_output;
  }

#endif

/**
 * Manage heating activities for hotends, bed, chamber and cooler
 *  - Is called every 100ms.
 *  - Acquire updated temperature readings
 *  - Also resets the watchdog timer
 *  - Invoke thermal runaway protection
 *  - Manage extruder auto-fan
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */
void Temperature::manage_temp_controller() {

  updateTemperaturesFromRawValues(); // also resets the watchdog

  #if ENABLED(HEATER_0_USES_MAX6675)
    if (current_temperature[0] > min(HEATER_0_MAXTEMP, MAX6675_TMAX - 1.0)) max_temp_error(0);
    if (current_temperature[0] < max(HEATER_0_MINTEMP, MAX6675_TMIN + .01)) min_temp_error(0);
  #endif

  #if WATCH_HOTENDS || WATCH_THE_BED || WATCH_THE_CHAMBER || WATCH_THE_COOLER || DISABLED(PIDTEMPBED) || DISABLED(PIDTEMPCHAMBER) || DISABLED(PIDTEMPCOOLER) || HAS_AUTO_FAN || HEATER_IDLE_HANDLER
    millis_t ms = millis();
  #endif

  #if HAS_TEMP_HOTEND

    LOOP_HOTEND() {

      #if HEATER_IDLE_HANDLER
        if (!heater_idle_timeout_exceeded[h] && heater_idle_timeout_ms[h] && ELAPSED(ms, heater_idle_timeout_ms[h]))
          heater_idle_timeout_exceeded[h] = true;
      #endif

      #if ENABLED(THERMAL_PROTECTION_HOTENDS)
        // Check for thermal runaway
        thermal_runaway_protection(&thermal_runaway_state_machine[h], &thermal_runaway_timer[h], current_temperature[h], target_temperature[h], h, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
      #endif

      soft_pwm[h] = (current_temperature[h] > minttemp[h] || is_preheating(h)) && current_temperature[h] < maxttemp[h] ? (int)get_pid_output(h) : 0;

      #if WATCH_HOTENDS
        // Make sure temperature is increasing
        if (watch_heater_next_ms[h] && ELAPSED(ms, watch_heater_next_ms[h])) {
          if (degHotend(h) < watch_target_temp[h])
            _temp_error(h, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
          else
            start_watching_heater(h); // Start again if the target is still far off
        }
      #endif

      #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
        // Make sure measured temperatures are close together
        if (FABS(current_temperature[0] - redundant_temperature) > MAX_REDUNDANT_TEMP_SENSOR_DIFF) {
          _temp_error(0, PSTR(MSG_REDUNDANCY), PSTR(MSG_ERR_REDUNDANT_TEMP));
        }
      #endif

    } // LOOP_HOTEND

  #endif

  #if HAS_AUTO_FAN
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
      meas_shift_index = constrain(meas_shift_index, 0, MAX_MEASUREMENT_DELAY);

      // Get the delayed info and add 100 to reconstitute to a percent of
      // the nominal filament diameter then square it to get an area
      const float vmroot = measurement_delay[meas_shift_index] * 0.01 + 1.0;
      tools.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM] = vmroot <= 0.1 ? 0.01 : sq(vmroot);
    }
  #endif // FILAMENT_SENSOR

  #if WATCH_THE_BED
    // Make sure temperature is increasing
    if (watch_bed_next_ms && ELAPSED(ms, watch_bed_next_ms)) {
      if (degBed() < watch_target_bed_temp)
        _temp_error(-1, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
      else
        start_watching_bed();
    }
  #endif

  #if HAS_TEMP_BED && DISABLED(PIDTEMPBED)
    if (PENDING(ms, next_bed_check_ms)) return;
    next_bed_check_ms = ms + BED_CHECK_INTERVAL;
  #endif

  #if HAS_TEMP_CHAMBER && DISABLED(PIDTEMPCHAMBER)
    if (PENDING(ms, next_chamber_check_ms)) return;
    next_chamber_check_ms = ms + CHAMBER_CHECK_INTERVAL;
  #endif

  #if HAS_TEMP_COOLER && DISABLED(PIDTEMPCOOLER)
    if (PENDING(ms, next_cooler_check_ms)) return;
    next_cooler_check_ms = ms + COOLER_CHECK_INTERVAL;
  #endif

  #if HAS_TEMP_BED

    #if HEATER_IDLE_HANDLER
      if (!bed_idle_timeout_exceeded && bed_idle_timeout_ms && ELAPSED(ms, bed_idle_timeout_ms))
        bed_idle_timeout_exceeded = true;
    #endif

    #if ENABLED(THERMAL_PROTECTION_BED)
      thermal_runaway_protection(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, -1, THERMAL_PROTECTION_BED_PERIOD, THERMAL_PROTECTION_BED_HYSTERESIS);
    #endif

    #if HEATER_IDLE_HANDLER
      if (bed_idle_timeout_exceeded) {
        soft_pwm_bed = 0;

        #if DISABLED(PIDTEMPBED)
          WRITE_HEATER_BED(LOW);
        #endif
      }
      else
    #endif
    {

      #if ENABLED(PIDTEMPBED)
        soft_pwm_bed = WITHIN(current_temperature_bed, BED_MINTEMP, BED_MAXTEMP) ? (int)get_pid_output_bed() : 0;
      #elif ENABLED(BED_LIMIT_SWITCHING)
        // Check if temperature is within the correct band
        if (WITHIN(current_temperature_bed, BED_MINTEMP, BED_MAXTEMP)) {
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
        if (WITHIN(current_temperature_bed, BED_MINTEMP, BED_MAXTEMP)) {
          soft_pwm_bed = current_temperature_bed < target_temperature_bed ? MAX_BED_POWER : 0;
        }
        else {
          soft_pwm_bed = 0;
          WRITE_HEATER_BED(LOW);
        }
      #endif
    }
  #endif // HAS_TEMP_BED

  #if HAS_TEMP_CHAMBER
    #if ENABLED(THERMAL_PROTECTION_CHAMBER)
      thermal_runaway_protection(&thermal_runaway_chamber_state_machine, &thermal_runaway_chamber_timer, current_temperature_chamber, target_temperature_chamber, -1, THERMAL_PROTECTION_CHAMBER_PERIOD, THERMAL_PROTECTION_CHAMBER_HYSTERESIS);
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      soft_pwm_chamber = WITHIN(current_temperature_chamber, CHAMBER_MINTEMP, CHAMBER_MAXTEMP) ? (int)get_pid_output_chamber() : 0;

    #elif ENABLED(CHAMBER_LIMIT_SWITCHING)
      // Check if temperature is within the correct band
      if (WITHIN(current_temperature_chamber, CHAMBER_MINTEMP, CHAMBER_MAXTEMP)) {
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
      if (WITHIN(current_temperature_chamber, CHAMBER_MINTEMP, CHAMBER_MAXTEMP)) {
        soft_pwm_chamber = current_temperature_chamber < target_temperature_chamber ? MAX_CHAMBER_POWER : 0;
      }
      else {
        soft_pwm_chamber = 0;
        WRITE_HEATER_CHAMBER(LOW);
      }
    #endif
  #endif // HAS_TEMP_CHAMBER

  #if HAS_TEMP_COOLER
    #if ENABLED(THERMAL_PROTECTION_COOLER)
      thermal_runaway_protection(&thermal_runaway_cooler_state_machine, &thermal_runaway_cooler_timer, current_temperature_cooler, target_temperature_cooler, -2, THERMAL_PROTECTION_COOLER_PERIOD, THERMAL_PROTECTION_COOLER_HYSTERESIS);
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      soft_pwm_cooler = WITHIN(current_temperature_cooler, COOLER_MINTEMP, COOLER_MAXTEMP) ? (int)get_pid_output_cooler() : 0;

    #elif ENABLED(COOLER_LIMIT_SWITCHING)
      // Check if temperature is within the correct band
      if (WITHIN(current_temperature_cooler, COOLER_MINTEMP, COOLER_MAXTEMP)) {
        if (current_temperature_cooler >= target_temperature_cooler + COOLER_HYSTERESIS)
          soft_pwm_cooler = MAX_COOLER_POWER >> 1;
        else if (current_temperature_cooler <= target_temperature_cooler - COOLER_HYSTERESIS)
          soft_pwm_cooler = 0;
      }
      else { 
        soft_pwm_cooler = 0;
        WRITE_COOLER(LOW);
      }
    #else // COOLER_LIMIT_SWITCHING
      // Check if temperature is within the correct range
      if (WITHIN(current_temperature_cooler, COOLER_MINTEMP, COOLER_MAXTEMP)) {
        soft_pwm_cooler = current_temperature_cooler > target_temperature_cooler ? MAX_COOLER_POWER : 0;
      }
      else {
        soft_pwm_cooler = 0;
        WRITE_COOLER(LOW);
      }
    #endif
  #endif // HAS_TEMP_COOLER
}

#define PGM_RD_W(x)   (short)pgm_read_word(&x)

#if HAS_TEMP_HOTEND

  // Derived from RepRap FiveD extruder::getTemperature()
  // For hot end temperature measurement.
  float Temperature::analog2temp(const int raw, uint8_t h) {

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      if (h > HOTENDS)
    #else
      if (h >= HOTENDS)
    #endif
      {
        SERIAL_SV(ER, (int)h);
        SERIAL_EM(MSG_INVALID_EXTRUDER_NUM);
        printer.kill(PSTR(MSG_KILLED));
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
      return ((raw * (((HAL_VOLTAGE_PIN) * 100.0) / 1024.0)) * ad595_gain[h]) + ad595_offset[h];
    #else
      return 0;
    #endif
  }

  #if DISABLED(MIN_COOLING_SLOPE_DEG)
    #define MIN_COOLING_SLOPE_DEG 1.50
  #endif
  #if DISABLED(MIN_COOLING_SLOPE_TIME)
    #define MIN_COOLING_SLOPE_TIME 60
  #endif

  void Temperature::wait_heater(bool no_wait_for_cooling/*=true*/) {

    #if TEMP_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is exactly on target
      #define TEMP_CONDITIONS (wants_to_cool ? isCoolingHotend(tools.target_extruder) : isHeatingHotend(tools.target_extruder))
    #endif

    float target_temp = -1.0, old_temp = 9999.0;
    bool wants_to_cool = false;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(NOT_BUSY);
    #endif

    #if ENABLED(PRINTER_EVENT_LEDS)
      const float start_temp = degHotend(tools.target_extruder);
      uint8_t old_blue = 0;
    #endif

    do {
      // Target temperature might be changed during the loop
      if (target_temp != degTargetHotend(tools.target_extruder))
        target_temp = degTargetHotend(tools.target_extruder);

      wants_to_cool = isCoolingHotend(tools.target_extruder);

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { // Print temp & remaining time every 1s while waiting
        next_temp_ms = now + 1000UL;
        print_heaterstates();
        #if TEMP_RESIDENCY_TIME > 0
          SERIAL_MSG(MSG_W);
          if (residency_start_ms)
            SERIAL_VAL(long((((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
          else
            SERIAL_CHR("?");
        #endif
        SERIAL_EOL();
      }

      printer.idle();
      commands.refresh_cmd_timeout(); // to prevent stepper.stepper_inactive_time from running out

      const float temp = degHotend(tools.target_extruder);

      #if ENABLED(PRINTER_EVENT_LEDS)
        // Gradually change LED strip from violet to red as nozzle heats up
        if (!wants_to_cool) {
          const uint8_t blue = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 255, 0);
          if (blue != old_blue) {
            old_blue = blue;
            printer.set_led_color(255, 0, blue
              #if ENABLED(NEOPIXEL_RGBW_LED)
                , 0, true
              #endif
            );
          }
        }
      #endif

      #if TEMP_RESIDENCY_TIME > 0

        float temp_diff = FABS(target_temp - temp);

        if (!residency_start_ms) {
          // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif

      // Prevent a wait-forever situation if R is misused i.e. M109 R0
      if (wants_to_cool) {
        // Break after MIN_COOLING_SLOPE_TIME seconds
        // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
        if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
          if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
          next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
          old_temp = temp;
        }
      }

    } while (wait_for_heatup && TEMP_CONDITIONS);

    if (wait_for_heatup) {
      LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
      #if ENABLED(PRINTER_EVENT_LEDS)
        #if ENABLED(RGBW_LED) || ENABLED(NEOPIXEL_RGBW_LED)
          printer.set_led_color(0, 0, 0, 255);  // Turn on the WHITE LED
        #else
          printer.set_led_color(255, 255, 255); // Set LEDs All On
        #endif
      #endif
    }

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(IN_HANDLER);
    #endif
  }

#endif

#if HAS_TEMP_BED

  // Derived from RepRap FiveD extruder::getTemperature()
  // For bed temperature measurement.
  float Temperature::analog2tempBed(const int raw) {
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
      return ((raw * (((HAL_VOLTAGE_PIN) * 100.0) / 1024.0)) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
    #else
      UNUSED(raw);
      return 0;
    #endif
  }

  #if DISABLED(MIN_COOLING_SLOPE_DEG_BED)
    #define MIN_COOLING_SLOPE_DEG_BED 1.50
  #endif
  #if DISABLED(MIN_COOLING_SLOPE_TIME_BED)
    #define MIN_COOLING_SLOPE_TIME_BED 60
  #endif

  void Temperature::wait_bed(bool no_wait_for_cooling/*=true*/) {

    #if TEMP_BED_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_BED_CONDITIONS (wants_to_cool ? isCoolingBed() : isHeatingBed())
    #endif

    float target_temp = -1.0, old_temp = 9999.0;
    bool wants_to_cool = false;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(NOT_BUSY);
    #endif

    tools.target_extruder = tools.active_extruder; // for print_heaterstates

    #if ENABLED(PRINTER_EVENT_LEDS)
      const float start_temp = degBed();
      uint8_t old_red = 255;
    #endif

    // Wait for temperature to come close enough
    do {
      // Target temperature might be changed during the loop
      if (target_temp != degTargetBed())
        target_temp = degTargetBed();

      wants_to_cool = isCoolingBed();

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { // Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_heaterstates();
        #if TEMP_BED_RESIDENCY_TIME > 0
          SERIAL_MSG(MSG_W);
          if (residency_start_ms)
            SERIAL_VAL(long((((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
          else
            SERIAL_CHR("?");
        #endif
        SERIAL_EOL();
      }

      printer.idle();
      commands.refresh_cmd_timeout(); // to prevent stepper.stepper_inactive_time from running out

      const float temp = degBed();

      #if ENABLED(PRINTER_EVENT_LEDS)
        // Gradually change LED strip from blue to violet as bed heats up
        if (!wants_to_cool) {
          const uint8_t red = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 0, 255);
          if (red != old_red) {
            old_red = red;
            printer.set_led_color(red, 0, 255
              #if ENABLED(NEOPIXEL_RGBW_LED)
                , 0, true
              #endif
            );
          }
        }
      #endif

      #if TEMP_BED_RESIDENCY_TIME > 0

        float temp_diff = FABS(target_temp - temp);

        if (!residency_start_ms) {
          // Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_BED_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif

      // Prevent a wait-forever situation if R is misused i.e. M190 R0
      if (wants_to_cool) {
        // Break after MIN_COOLING_SLOPE_TIME_BED seconds
        // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_BED
        if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
          if (old_temp - temp < MIN_COOLING_SLOPE_DEG_BED) break;
          next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME_BED;
          old_temp = temp;
        }
      }

    } while (wait_for_heatup && TEMP_BED_CONDITIONS);

    if (wait_for_heatup) LCD_MESSAGEPGM(MSG_BED_DONE);

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(IN_HANDLER);
    #endif
  }

#endif

#if HAS_TEMP_CHAMBER

  float Temperature::analog2tempChamber(const int raw) { 
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
      return ((raw * (((HAL_VOLTAGE_PIN) * 100.0) / 1024.0)) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
    #else
      UNUSED(raw);
      return 0;
    #endif
  }

  void Temperature::wait_chamber(bool no_wait_for_heating/*=true*/) {
    #if TEMP_CHAMBER_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_CHAMBER_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_CHAMBER_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_CHAMBER_CONDITIONS (wants_to_heat ? isHeatingChamber() : isCoolingChamber())
    #endif

    float target_temp = -1;
    bool wants_to_heat;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0;

    KEEPALIVE_STATE(WAIT_HEATER);

    // Wait for temperature to come close enough
    do {
      // Target temperature might be changed during the loop
      if (target_temp != degTargetChamber())
        target_temp = degTargetChamber();

      wants_to_heat = isHeatingChamber();

      // Exit if S<higher>, continue if S<lower>, R<higher>, or R<lower>
      if (no_wait_for_heating && wants_to_heat) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { // Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_chamberstate();
        #if TEMP_CHAMBER_RESIDENCY_TIME > 0
          SERIAL_MSG(MSG_W);
          if (residency_start_ms) {
            long rem = (((TEMP_CHAMBER_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
            SERIAL_EV(rem);
          }
          else {
            SERIAL_EM("?");
          }
        #else
          SERIAL_EOL();
        #endif
      }

      idle();
      commands.refresh_cmd_timeout(); // to prevent stepper.stepper_inactive_time from running out

      #if TEMP_CHAMBER_RESIDENCY_TIME > 0

        float temp_diff = FABS(target_temp - degTargetChamber());

        if (!residency_start_ms) {
          // Start the TEMP_CHAMBER_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_CHAMBER_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_CHAMBER_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif //TEMP_CHAMBER_RESIDENCY_TIME > 0

    } while (wait_for_heatup && TEMP_CHAMBER_CONDITIONS);
    LCD_MESSAGEPGM(MSG_CHAMBER_DONE);
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif

#if HAS_TEMP_COOLER

  float Temperature::analog2tempCooler(const int raw) { 
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
      return ((raw * (((HAL_VOLTAGE_PIN) * 100.0) / 1024.0)) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
    #else
      UNUSED(raw);
      return 0;
    #endif
  }

  void Temperature::wait_cooler(bool no_wait_for_heating/*=true*/) {
    #if TEMP_COOLER_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_COOLER_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_COOLER_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_COOLER_CONDITIONS (wants_to_heat ? isHeatingCooler() : isCoolingCooler())
    #endif

    float target_temp = -1;
    bool wants_to_heat;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0;

    KEEPALIVE_STATE(WAIT_HEATER);

    // Wait for temperature to come close enough
    do {
      // Target temperature might be changed during the loop
      if (target_temp != degTargetCooler())
        target_temp = degTargetCooler();

      wants_to_heat = isHeatingCooler();

      // Exit if S<higher>, continue if S<lower>, R<higher>, or R<lower>
      if (no_wait_for_heating && wants_to_heat) break;

      // Prevent a wait-forever situation if R is misused i.e. M190 C R50
      // Simply don't wait to heat a cooler over 25C
      if (wants_to_heat && target_temp > 25) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_coolerstate();
        #if TEMP_COOLER_RESIDENCY_TIME > 0
          SERIAL_MSG(MSG_W);
          if (residency_start_ms) {
            long rem = (((TEMP_COOLER_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
            SERIAL_EV(rem);
          }
          else {
            SERIAL_EM("?");
          }
        #else
          SERIAL_EOL();
        #endif
      }

      idle();
      commands.refresh_cmd_timeout(); // to prevent stepper.stepper_inactive_time from running out

      #if TEMP_COOLER_RESIDENCY_TIME > 0

        float temp_diff = FABS(target_temp - degTargetCooler());

        if (!residency_start_ms) {
          // Start the TEMP_COOLER_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_COOLER_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_COOLER_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif //TEMP_COOLER_RESIDENCY_TIME > 0

    } while (wait_for_heatup && TEMP_COOLER_CONDITIONS);
    LCD_MESSAGEPGM(MSG_COOLER_DONE);
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif

#if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)

  float Temperature::analog2tempMCU(const int raw) {
    float mcutemp = (float)raw * (3.3 / 4096.0);
    return (mcutemp - 0.8) * (1000.0 / 2.65) + 27.0; // + mcuTemperatureAdjust;			// accuracy at 27C is +/-45C
  }

#endif

#if HAS_TEMP_HOTEND || HAS_TEMP_BED

  void Temperature::print_heater_state(const float &c, const int16_t &t,
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      const int16_t r,
    #endif
    const int8_t e/*=-2*/
  ) {
    SERIAL_CHR(' ');
    SERIAL_CHR(
      #if HAS_TEMP_BED && HAS_TEMP_HOTEND
        e == -1 ? 'B' : 'T'
      #elif HAS_TEMP_HOTEND
        'T'
      #else
        'B'
      #endif
    );
    #if HOTENDS > 1
      if (e >= 0) SERIAL_CHR('0' + e);
    #endif
    SERIAL_CHR(':');
    SERIAL_VAL(c, 1);
    SERIAL_MV(" /" , t);
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV(" (", r);
      SERIAL_CHR(')');
    #endif
  }

  void Temperature::print_heaterstates() {
    #if HAS_TEMP_HOTEND
      print_heater_state(degHotend(tools.target_extruder), degTargetHotend(tools.target_extruder)
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          , rawHotendTemp(tools.target_extruder)
        #endif
      );
    #endif
    #if HAS_TEMP_BED
      print_heater_state(degBed(), degTargetBed(),
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          rawBedTemp(),
        #endif
        -1 // BED
      );
    #endif
    #if HOTENDS > 1
      LOOP_HOTEND() print_heater_state(degHotend(h), degTargetHotend(h),
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          rawHotendTemp(h),
        #endif
        h
      );
    #endif
    SERIAL_MV(MSG_AT ":", getHeaterPower(tools.target_extruder));
    #if HAS_TEMP_BED
      SERIAL_MV(MSG_BAT, getBedPower());
    #endif
    #if HOTENDS > 1
      LOOP_HOTEND() {
        SERIAL_MV(MSG_AT, h);
        SERIAL_CHR(':');
        SERIAL_VAL(getHeaterPower(h));
      }
    #endif
  }

#endif

#if HAS_TEMP_CHAMBER

  void Temperature::print_chamberstate() {
    SERIAL_MSG(" CHAMBER:");
    SERIAL_MV(MSG_C, degChamber(), 1);
    SERIAL_MV(" /", degTargetChamber());
    SERIAL_MSG(MSG_CAT);
    #if ENABLED(CHAMBER_WATTS)
      SERIAL_VAL(((CHAMBER_WATTS) * getChamberPower()) / 127.0);
      SERIAL_MSG("W");
    #else
      SERIAL_VAL(getChamberPower());
    #endif
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV(" ADC C:", degChamber(), 1);
      SERIAL_MV(" C->", rawChamberTemp());
    #endif
  }

#endif // HAS_TEMP_CHAMBER

#if HAS_TEMP_COOLER

  void Temperature::print_coolerstate() {
    SERIAL_MSG(" COOL:");
    SERIAL_MV(MSG_C, degCooler(), 1);
    SERIAL_MV(" /", degTargetCooler());
    SERIAL_MSG(MSG_CAT);
    #if ENABLED(COOLER_WATTS)
      SERIAL_VAL(((COOLER_WATTS) * getCoolerPower()) / 127.0);
      SERIAL_MSG("W");
    #else
      SERIAL_VAL(getCoolerPower());
    #endif
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV(" ADC C:", degCooler(), 1);
      SERIAL_MV(" C->", rawCoolerTemp());
    #endif
  }

#endif // HAS_TEMP_COOLER

#if ENABLED(ARDUINO_ARCH_SAM)&& !MB(RADDS)

  void Temperature::print_MCUstate() {
    SERIAL_MSG(" MCU: min");
    SERIAL_MV(MSG_C, lowest_temperature_mcu, 1);
    SERIAL_MSG(", current");
    SERIAL_MV(MSG_C, current_temperature_mcu, 1);
    SERIAL_MSG(", max");
    SERIAL_MV(MSG_C, highest_temperature_mcu, 1);
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV(" C->", rawMCUTemp());
    #endif
  }

#endif

#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)

  void Temperature::auto_report_temperatures() {
    if (auto_report_temp_interval && ELAPSED(millis(), next_temp_report_ms)) {
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
      print_heaterstates();

      #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
        print_MCUstate();
      #endif

      SERIAL_EOL();
    }
  }

#endif // AUTO_REPORT_TEMPERATURES

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
  #if HAS_TEMP_HOTEND
    LOOP_HOTEND()
      current_temperature[h] = analog2temp(current_temperature_raw[h], h);
  #endif
  #if HAS_TEMP_BED
    current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
  #endif
  #if HAS_TEMP_CHAMBER
    current_temperature_chamber = analog2tempChamber(current_temperature_chamber_raw);
  #endif
  #if HAS_TEMP_COOLER
    current_temperature_cooler = analog2tempCooler(current_temperature_cooler_raw);
  #endif

  #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
    redundant_temperature = analog2temp(redundant_temperature_raw, 1);
  #endif
  #if ENABLED(FILAMENT_SENSOR)
    filament_width_meas = analog2widthFil();
  #endif
  #if HAS_POWER_CONSUMPTION_SENSOR
    static millis_t last_update = millis();
    millis_t temp_last_update = millis();
    millis_t from_last_update = temp_last_update - last_update;
    static float watt_overflow = 0.0;
    powerManager.consumption_meas = powerManager.analog2power();
    /*SERIAL_MV("raw:", powerManager.raw_analog2voltage(), 5);
    SERIAL_MV(" - V:", powerManager.analog2voltage(), 5);
    SERIAL_MV(" - I:", powerManager.analog2current(), 5);
    SERIAL_EMV(" - P:", powerManager.analog2power(), 5);*/
    watt_overflow += (powerManager.consumption_meas * from_last_update) / 3600000.0;
    if (watt_overflow >= 1.0) {
      powerManager.consumption_hour++;
      watt_overflow--;
    }
    last_update = temp_last_update;
  #endif

  #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
    current_temperature_mcu = analog2tempMCU(current_temperature_mcu_raw);
    if (current_temperature_mcu > highest_temperature_mcu)
      highest_temperature_mcu= current_temperature_mcu;
    if (current_temperature_mcu < lowest_temperature_mcu && current_temperature_mcu != 0)
      lowest_temperature_mcu = current_temperature_mcu;
  #endif

  #if ENABLED(USE_WATCHDOG)
    // Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();
  #endif

}

#if ENABLED(FILAMENT_SENSOR)

  // Convert raw Filament Width to millimeters
  float Temperature::analog2widthFil() {
    return current_raw_filwidth * 5.0 * (1.0 / 16383.0);
    //return current_raw_filwidth;
  }

  // Convert raw Filament Width to a ratio
  int Temperature::widthFil_to_size_ratio() {
    float temp = filament_width_meas;
    if (temp < MEASURED_LOWER_LIMIT) temp = filament_width_nominal;  // assume sensor cut out
    else NOMORE(temp, MEASURED_UPPER_LIMIT);
    return filament_width_nominal / temp * 100;
  }

#endif

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_temp_controller()
 */
void Temperature::init() {

  #if MB(RUMBA) && ((TEMP_SENSOR_0==-1)||(TEMP_SENSOR_1==-1)||(TEMP_SENSOR_2==-1)||(TEMP_SENSOR_BED==-1)||(TEMP_SENSOR_CHAMBER==-1)||(TEMP_SENSOR_COOLER==-1))
    // disable RUMBA JTAG in case the thermocouple extension is plugged on top of JTAG connector
    MCUCR = _BV(JTD);
    MCUCR = _BV(JTD);
  #endif

  // Finish init of mult hotend arrays
  LOOP_HOTEND() maxttemp[h] = maxttemp[0];

  #if ENABLED(PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
    last_e_position = 0;
  #endif

  #if HAS_HEATER_0 && HAS_TEMP_0
    SET_OUTPUT(HEATER_0_PIN);
    WRITE_HEATER_0(LOW);
  #endif
  #if HAS_HEATER_1 && HAS_TEMP_1
    SET_OUTPUT(HEATER_1_PIN);
    WRITE_HEATER_1(LOW);
  #endif
  #if HAS_HEATER_2 && HAS_TEMP_2
    SET_OUTPUT(HEATER_2_PIN);
    WRITE_HEATER_2(LOW);
  #endif
  #if HAS_HEATER_3 && HAS_TEMP_3
    SET_OUTPUT(HEATER_3_PIN);
    WRITE_HEATER_3(LOW);
  #endif
  #if HAS_HEATER_BED && HAS_TEMP_BED
    SET_OUTPUT(HEATER_BED_PIN);
    WRITE_HEATER_BED(LOW);
  #endif
  #if HAS_HEATER_CHAMBER && HAS_TEMP_CHAMBER
    SET_OUTPUT(HEATER_CHAMBER_PIN);
    WRITE_HEATER_CHAMBER(LOW);
  #endif
  #if HAS_COOLER && HAS_TEMP_COOLER
    SET_OUTPUT(COOLER_PIN);
    WRITE_COOLER(LOW);
  #endif

  #if HAS_FAN0
    SET_OUTPUT(FAN_PIN);
    WRITE_FAN0(LOW);
  #endif
  #if HAS_FAN1
    SET_OUTPUT(FAN1_PIN);
    WRITE_FAN1(LOW);
  #endif
  #if HAS_FAN2
    SET_OUTPUT(FAN2_PIN);
    WRITE_FAN2(LOW);
  #endif
  #if HAS_FAN3
    SET_OUTPUT(FAN3_PIN);
    WRITE_FAN3(LOW);
  #endif

  #if ENABLED(HEATER_0_USES_MAX6675)

    OUT_WRITE(SCK_PIN, LOW);
    OUT_WRITE(MOSI_PIN, HIGH);
    SET_INPUT_PULLUP(MISO_PIN);
    OUT_WRITE(SS_PIN, HIGH);

    OUT_WRITE(MAX6675_SS, HIGH);

  #endif // HEATER_0_USES_MAX6675

  HAL::analogStart();

  #if HAS_AUTO_FAN_0
    SET_OUTPUT(H0_AUTO_FAN_PIN);
  #endif
  #if HAS_AUTO_FAN_1 && !AUTO_1_IS_0
    SET_OUTPUT(H1_AUTO_FAN_PIN);
  #endif
  #if HAS_AUTO_FAN_2 && !AUTO_2_IS_0 && !AUTO_2_IS_1
    SET_OUTPUT(H2_AUTO_FAN_PIN);
  #endif
  #if HAS_AUTO_FAN_3 && !AUTO_3_IS_0 && !AUTO_3_IS_1 && !AUTO_3_IS_2
    SET_OUTPUT(H3_AUTO_FAN_PIN);
  #endif

  // Use timer for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  HAL_TEMP_TIMER_START();
  ENABLE_TEMP_INTERRUPT();

  // Wait for temperature measurement to settle
  HAL::delayMilliseconds(250);

  #define TEMP_MIN_ROUTINE(NR) \
    minttemp[NR] = HEATER_ ##NR## _MINTEMP; \
    while (analog2temp(minttemp_raw[NR], NR) < HEATER_ ##NR## _MINTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        minttemp_raw[NR] += OVERSAMPLENR; \
      else \
        minttemp_raw[NR] -= OVERSAMPLENR; \
    }
  #define TEMP_MAX_ROUTINE(NR) \
    maxttemp[NR] = HEATER_ ##NR## _MAXTEMP; \
    while (analog2temp(maxttemp_raw[NR], NR) > HEATER_ ##NR## _MAXTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        maxttemp_raw[NR] -= OVERSAMPLENR; \
      else \
        maxttemp_raw[NR] += OVERSAMPLENR; \
    }

  #if HOTENDS > 0
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
  #endif // HOTENDS > 0

  #if ENABLED(BED_MINTEMP)
    while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_minttemp_raw += OVERSAMPLENR;
      #else
        bed_minttemp_raw -= OVERSAMPLENR;
      #endif
    }
  #endif // BED_MINTEMP
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
    while(analog2tempChamber(chamber_maxttemp_raw) > CHAMBER_MAXTEMP) {
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

  #if ENABLED(PROBING_HEATERS_OFF)
    paused = false;
  #endif
}

#if WATCH_HOTENDS
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

#if WATCH_THE_BED
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

#if WATCH_THE_CHAMBER
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

#if WATCH_THE_COOLER
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

  #if ENABLED(THERMAL_PROTECTION_CHAMBER) && HAS_TEMP_CHAMBER
    Temperature::TRState Temperature::thermal_runaway_chamber_state_machine = TRReset;
    millis_t Temperature::thermal_runaway_chamber_timer;
  #endif

  #if ENABLED(THERMAL_PROTECTION_COOLER) && HAS_TEMP_COOLER
    Temperature::TRState Temperature::thermal_runaway_cooler_state_machine = TRReset;
    millis_t Temperature::thermal_runaway_cooler_timer;
  #endif

  void Temperature::thermal_runaway_protection(Temperature::TRState* state, millis_t* timer, float temperature, float target_temperature, int temp_controller_id, int period_seconds, int hysteresis_degc) {

    static float tr_target_temperature[HOTENDS + 3] = { 0.0 };

    /*
        SERIAL_MSG("Thermal Thermal Runaway Running. Heater ID: ");
        if (temp_controller_id < 0) SERIAL_MSG("bed"); else SERIAL_VAL(temp_controller_id);
        SERIAL_MV(" ;  State:", *state);
        SERIAL_MV(" ;  Timer:", *timer);
        SERIAL_MV(" ;  Temperature:", temperature);
        SERIAL_EMV(" ;  Target Temp:", target_temperature);
    */

    int temp_controller_index;

    if (temp_controller_id >= 0)
      temp_controller_index = temp_controller_id;
    else if (temp_controller_id == -1)
      temp_controller_index = HOTENDS; // BED
    else if (temp_controller_id == -2)
      temp_controller_index = HOTENDS + 1; // CHAMBER
    else
      temp_controller_index = HOTENDS + 2; // COOLER

    #if HEATER_IDLE_HANDLER
      // If the heater idle timeout expires, restart
      if (temp_controller_id >= 0 && heater_idle_timeout_exceeded[temp_controller_id]) {
        *state = TRInactive;
        tr_target_temperature[temp_controller_index] = 0;
      }
      #if HAS_TEMP_BED
        else if (temp_controller_id == -1 && bed_idle_timeout_exceeded) {
          *state = TRInactive;
          tr_target_temperature[temp_controller_index] = 0;
        }
      #endif
      else
    #endif
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

  #if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)
    planner.autotemp_enabled = false;
  #endif

  #if HAS_TEMP_HOTEND
    LOOP_HOTEND() setTargetHotend(0, h);
  #endif
  #if HAS_TEMP_BED
    setTargetBed(0);
  #endif
  #if HAS_TEMP_CHAMBER
    setTargetChamber(0);
  #endif

  // Unpause and reset everything
  #if ENABLED(PROBING_HEATERS_OFF)
    pause(false);
  #endif

  // If all heaters go down then for sure our print job has stopped
  printer.print_job_counter.stop();

  #define DISABLE_HEATER(NR) { \
    setTargetHotend(0, NR); \
    soft_pwm[NR] = 0; \
    WRITE_HEATER_ ##NR (LOW); \
  }

  #if HAS_TEMP_HOTEND

    target_temperature[0] = 0;
    soft_pwm[0] = 0;
    WRITE_HEATER_0P(LOW); // Should HEATERS_PARALLEL apply here? Then change to DISABLE_HEATER(0)

    #if HOTENDS > 1 && HAS_TEMP_1
      DISABLE_HEATER(1);
    #endif

    #if HOTENDS > 2 && HAS_TEMP_2
      DISABLE_HEATER(2);
    #endif

    #if HOTENDS > 3 && HAS_TEMP_3
      DISABLE_HEATER(3);
    #endif

  #endif

  #if HAS_TEMP_BED
    target_temperature_bed = 0;
    soft_pwm_bed = 0;
    #if HAS_HEATER_BED
      WRITE_HEATER_BED(LOW);
    #endif
  #endif

  #if HAS_TEMP_CHAMBER
    target_temperature_chamber = 0;
    soft_pwm_chamber = 0;
    #if HAS_HEATER_CHAMBER
      WRITE_HEATER_CHAMBER(LOW);
    #endif
  #endif
}

#if HAS_TEMP_COOLER
  void Temperature::disable_all_coolers() {
    setTargetCooler(0);

    // if cooler go down the print job is stopped 
    printer.print_job_counter.stop();

    #if ENABLED(LASER)
      // No laser firing with no coolers running! (paranoia)
      laser.extinguish();
    #endif

    #if HAS_TEMP_COOLER
      target_temperature_cooler = 0;
      soft_pwm_chamber = 0;
      #if HAS_COOLER && !ENABLED(FAST_PWM_COOLER)
        WRITE_COOLER(LOW);
      #endif
    #endif
  }
#endif

#if ENABLED(PROBING_HEATERS_OFF)

  void Temperature::pause(const bool p) {
    if (p != paused) {
      paused = p;
      if (p) {
        LOOP_HOTEND() start_heater_idle_timer(h, 0); // timeout immediately
        #if HAS_TEMP_BED
          start_bed_idle_timer(0); // timeout immediately
        #endif
      }
      else {
        LOOP_HOTEND() reset_heater_idle_timer(h);
        #if HAS_TEMP_BED
          reset_bed_idle_timer();
        #endif
      }
    }
  }

#endif // PROBING_HEATERS_OFF

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

    spiBegin();
    spiInit(2);

    HAL::digitalWrite(MAX6675_SS, 0);  // enable TT_MAX6675

    // ensure 100ns delay - a bit extra is fine
    #if ENABLED(CPU_32_BIT)
      HAL::delayMicroseconds(1);
    #else
      asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
      asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
    #endif

    // Read a big-endian temperature value
    max6675_temp = 0;
    for (uint8_t i = sizeof(max6675_temp); i--;) {
      #if ENABLED(CPU_32_BIT)
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
        SERIAL_MSG("MAX31855 ");
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
        if (max6675_temp & 0x00002000) max6675_temp |= 0xFFFFC000;
      #endif
    }

    return (int)max6675_temp;
  }

#endif //HEATER_0_USES_MAX6675

void Temperature::set_current_temp_raw() {

  #if HAS_TEMP_HOTEND
    current_temperature_raw[0] = HAL::AnalogInputValues[HOT0_SENSOR_INDEX];
  #endif
  #if HAS_TEMP_1
    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      redundant_temperature_raw = HAL::AnalogInputValues[HOT1_SENSOR_INDEX];
    #else
      current_temperature_raw[1] = HAL::AnalogInputValues[HOT1_SENSOR_INDEX];
    #endif
    #if HAS_TEMP_2
      current_temperature_raw[2] = HAL::AnalogInputValues[HOT2_SENSOR_INDEX];
      #if HAS_TEMP_3
        current_temperature_raw[3] = HAL::AnalogInputValues[HOT3_SENSOR_INDEX];
      #endif
    #endif
  #endif

  #if HAS_TEMP_BED
    current_temperature_bed_raw = HAL::AnalogInputValues[BED_SENSOR_INDEX];
  #endif
  #if HAS_TEMP_CHAMBER
    current_temperature_chamber_raw = HAL::AnalogInputValues[CHAMBER_SENSOR_INDEX];
  #endif
  #if HAS_TEMP_COOLER
    current_temperature_cooler_raw = HAL::AnalogInputValues[COOLER_SENSOR_INDEX];
  #endif

  #if HAS_POWER_CONSUMPTION_SENSOR
    powerManager.current_raw_powconsumption = HAL::AnalogInputValues[POWER_SENSOR_INDEX];
  #endif

  #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
    current_temperature_mcu_raw = HAL::AnalogInputValues[MCU_SENSOR_INDEX];
  #endif

  #if HAS_TEMP_HOTEND

    int constexpr temp_dir[] = {
      #if ENABLED(HEATER_0_USES_MAX6675)
         0
      #elif HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
        -1
      #else
         1
      #endif
      #if HAS_TEMP_1 && HOTENDS > 1
        #if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
          , -1
        #else
          ,  1
        #endif
      #endif
      #if HAS_TEMP_2 && HOTENDS > 2
        #if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
          , -1
        #else
          ,  1
        #endif
      #endif
      #if HAS_TEMP_3 && HOTENDS > 3
        #if HEATER_3_RAW_LO_TEMP > HEATER_3_RAW_HI_TEMP
          , -1
        #else
          ,  1
        #endif
      #endif
    };

    for (uint8_t h = 0; h < COUNT(temp_dir); h++) {
      const int16_t tdir = temp_dir[h], rawtemp = current_temperature_raw[h] * tdir;
      if (rawtemp > maxttemp_raw[h] * tdir && target_temperature[h] > 0) max_temp_error(h);
      if (rawtemp < minttemp_raw[h] * tdir && !is_preheating(h) && target_temperature[h] > 0) {
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

  #endif

  #if HAS_TEMP_BED

    #if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
      #define GEBED <=
    #else
      #define GEBED >=
    #endif
    if (current_temperature_bed_raw GEBED bed_maxttemp_raw) max_temp_error(-1);
    if (bed_minttemp_raw GEBED current_temperature_bed_raw && target_temperature_bed > 0) {
      #if ENABLED(MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
        if (++consecutive_bed_low_temperature_error >= MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
      #endif
          min_temp_error(-1);
    }
    #if ENABLED(MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
      else
        consecutive_bed_low_temperature_error = 0;
    #endif

  #endif

  #if HAS_TEMP_CHAMBER

    #if HEATER_CHAMBER_RAW_LO_TEMP > HEATER_CHAMBER_RAW_HI_TEMP
      #define GECHAMBER <=
    #else
      #define GECHAMBER >=
    #endif
    if (current_temperature_chamber_raw GECHAMBER chamber_maxttemp_raw)
      _temp_error(-2, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP_CHAMBER));
    if (chamber_minttemp_raw GECHAMBER current_temperature_chamber_raw && target_temperature_chamber > 0)
      _temp_error(-2, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP_CHAMBER));

  #endif

  #if HAS_TEMP_COOLER

    #if COOLER_RAW_LO_TEMP > COOLER_RAW_HI_TEMP
      #define GECOOLER <=
    #else
      #define GECOOLER >=
    #endif
    if (current_temperature_cooler_raw GECOOLER cooler_maxttemp_raw)
      _temp_error(-3, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP_COOLER));
    if (cooler_minttemp_raw GECOOLER current_temperature_cooler_raw && target_temperature_cooler > 0)
      _temp_error(-3, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP_COOLER));

  #endif

}

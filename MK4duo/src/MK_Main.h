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

#ifndef _MK_MAIN_H
#define _MK_MAIN_H

#if IS_SCARA
  void forward_kinematics_SCARA(const float &a, const float &b);
  void inverse_kinematics(const float logical[XYZ]);
#endif

#if ENABLED(LIN_ADVANCE)
  extern int extruder_advance_k;
#endif

#if HEATER_USES_AD595
  extern float ad595_offset[HOTENDS];
  extern float ad595_gain[HOTENDS];
#endif

#if ENABLED(NPR2)
  extern uint8_t old_color; // old color for system NPR2
#endif

#if ENABLED(G38_PROBE_TARGET)
  extern bool G38_move,        // flag to tell the interrupt handler that a G38 command is being run
              G38_endstop_hit; // flag from the interrupt handler to indicate if the endstop went active
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  extern MK4duoBusyState busy_state;
  #define KEEPALIVE_STATE(n) do{ busy_state = n; }while(0)
#else
  #define KEEPALIVE_STATE(n) NOOP
#endif

#if ENABLED(BARICUDA)
  extern int baricuda_valve_pressure;
  extern int baricuda_e_to_p_pressure;
#endif

#if ENABLED(FILAMENT_SENSOR)
  extern bool     filament_sensor;          // Flag that filament sensor readings should control extrusion
  extern float    filament_width_nominal,   // Theoretical filament diameter i.e., 3.00 or 1.75
                  filament_width_meas;      // Measured filament diameter
  extern uint8_t  meas_delay_cm,            // Delay distance
                  measurement_delay[];      // Ring buffer to delay measurement
  extern int8_t   filwidth_delay_index[2];  // Ring buffer indexes. Used by planner, temperature, and main code
#endif

#if HAS(POWER_CONSUMPTION_SENSOR)
  extern float power_consumption_meas;          //holds the power consumption as accurately measured
  extern unsigned long power_consumption_hour;  //holds the power consumption per hour as accurately measured
  extern unsigned long startpower;
  extern unsigned long stoppower;
  extern float raw_analog2voltage();
  extern float analog2error(float current);
  extern float analog2efficiency(float watt);
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  extern bool IDLE_OOZING_enabled;
  extern void IDLE_OOZING_retract(bool retracting);
#endif

#if ENABLED(PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
  extern int lpq_len;
#endif

#if ENABLED(FWRETRACT)
  extern bool autoretract_enabled;
  extern bool retracted[EXTRUDERS]; // extruder[n].retracted
  extern float retract_length, retract_length_swap, retract_feedrate, retract_zlift;
  extern float retract_recover_length, retract_recover_length_swap, retract_recover_feedrate;
#endif

#if ENABLED(EASY_LOAD)
  extern bool allow_lengthy_extrude_once; // for load/unload
#endif

#if ENABLED(DIGIPOT_I2C)
  extern void digipot_i2c_set_current( int channel, float current );
  extern void digipot_i2c_init();
#endif

#if ENABLED(FLOWMETER_SENSOR)
  void print_flowratestate();
#endif

#if ENABLED(COLOR_MIXING_EXTRUDER)
  extern float mixing_factor[MIXING_STEPPERS];
#endif

#endif // _MK_MAIN_H

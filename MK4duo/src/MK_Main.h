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

void get_command();

void idle(
  #if ENABLED(ADVANCED_PAUSE_FEATURE) || ENABLED(CNCROUTER)
    bool no_stepper_sleep=false  // pass true to keep steppers from disabling on timeout
  #endif
);

void manage_inactivity(bool ignore_stepper_queue=false);

void FlushSerialRequestResend();
void ok_to_send();

#if IS_SCARA
  void forward_kinematics_SCARA(const float &a, const float &b);
  void inverse_kinematics(const float logical[XYZ]);
#endif

void home_all_axes();

void kill(const char *);
void Stop();
void quickstop_stepper();

extern MK4duoInterruptEvent interruptEvent;
void setInterruptEvent(const MK4duoInterruptEvent event);
void handle_Interrupt_Event();

#if HAS_EXT_ENCODER
  extern int32_t  encStepsSinceLastSignal[EXTRUDERS]; // when was the last signal
  extern uint8_t  encLastSignal[EXTRUDERS];           // what was the last signal
  extern int8_t   encLastDir[EXTRUDERS];
  extern int32_t  encLastChangeAt[EXTRUDERS],
                  encErrorSteps[EXTRUDERS];
#endif

extern uint8_t mk_debug_flags;

// Printer Mode
extern PrinterMode printer_mode;

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }

// Print status related
extern long   currentLayer,
              maxLayer; // -1 = unknown
extern char   printName[21]; // max. 20 chars + 0
extern float  progress;

// Count of commands in the queue
extern uint8_t commands_in_queue;

bool enqueue_and_echo_command(const char* cmd, bool say_ok=false);  // Add a single command to the end of the buffer. Return false on failure.
void enqueue_and_echo_commands_P(const char* cmd);                  // Set one or more commands to be prioritized over the next Serial/SD command.
void clear_command_queue();

void prepare_arc_move(char isclockwise);

extern millis_t previous_cmd_ms;
inline void refresh_cmd_timeout() { previous_cmd_ms = millis(); }

extern void setup_for_endstop_or_probe_move();
extern void clean_up_after_endstop_or_probe_move();

extern void safe_delay(millis_t ms);

extern bool volumetric_enabled;
extern int16_t  flow_percentage[EXTRUDERS],     // Extrusion factor for each extruder
                density_percentage[EXTRUDERS];  // Extrusion density factor for each extruder
extern float  filament_size[EXTRUDERS],         // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
              volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner

extern volatile bool wait_for_heatup;

extern const char axis_codes[NUM_AXIS];

#if HAS_RESUME_CONTINUE
  extern volatile bool wait_for_user;
#endif

// Hotend offset
extern float hotend_offset[XYZ][HOTENDS];

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

#if FAN_COUNT > 0
  extern int16_t fanSpeeds[FAN_COUNT];
  #if ENABLED(PROBING_FANS_OFF)
    extern bool fans_paused;
    extern int16_t paused_fanSpeeds[FAN_COUNT];
    extern void fans_pause(const bool p);
  #endif
#endif
#if HAS_CONTROLLERFAN
  extern uint8_t controller_fanSpeeds;
#endif
#if HAS(AUTO_FAN)
  extern uint8_t autoFanSpeeds[HOTENDS];
#endif
#if ENABLED(FAN_KICKSTART_TIME)
  extern uint8_t fanKickstart;
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

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  extern AdvancedPauseMenuResponse advanced_pause_menu_response;
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

// Print job timer
extern PrintCounter print_job_counter;

// Handling multiple extruders pins
extern uint8_t  active_extruder,
                previous_extruder,
                target_extruder,
                active_driver;

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
  extern float motor_current[3 + DRIVER_EXTRUDERS];
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

void calculate_volumetric_multipliers();

void tool_change(const uint8_t tmp_extruder, const float fr_mm_s = 0.0, bool no_move = false);

#if ENABLED(CNCROUTER)
  void tool_change_cnc(const uint8_t tool_id, bool wait = true, bool raise_z = true);
#endif

/**
 * SD Stop & Store location
 */
#if ENABLED(SDSUPPORT)
  void stopSDPrint(const bool store_location);
#endif

#endif // _MK_MAIN_H

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
 * printer.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _PRINTER_H_
#define _PRINTER_H_

extern const char axis_codes[NUM_AXIS];

class Printer {

  public: /** Constructor */

    Printer() {};

  public: /** Public Parameters */

    static bool Running,
                volumetric_enabled,
                pos_saved;

    static volatile bool  wait_for_heatup,
                          wait_for_user;

    // Extruder
    static uint8_t  active_extruder,
                    previous_extruder,
                    target_extruder,
                    active_driver;

    static uint8_t host_keepalive_interval;

    static float  resume_position[XYZE];
    static bool   move_away_flag;

    static bool relative_mode,
                axis_relative_modes[];

    static int16_t  flow_percentage[EXTRUDERS],       // Extrusion factor for each extruder
                    density_percentage[EXTRUDERS];    // Extrusion density factor for each extruder
    static float    filament_size[EXTRUDERS],         // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
                    volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner

    // Hotend offset
    static float  hotend_offset[XYZ][HOTENDS];

    static long   currentLayer,
                  maxLayer;       // -1 = unknown
    static char   printName[21];  // max. 20 chars + 0
    static float  progress;

    static millis_t max_inactive_time;

    static MK4duoInterruptEvent interruptEvent;
    static PrinterMode          mode;
    static PrintCounter         print_job_counter;

    #if HAS_SDSUPPORT
      static bool sd_print_paused;
    #endif

    #if FAN_COUNT > 0
      static int16_t fanSpeeds[FAN_COUNT];
      #if ENABLED(PROBING_FANS_OFF)
        static bool fans_paused;
        static int16_t paused_fanSpeeds[FAN_COUNT];
      #endif
    #endif
    #if HAS_CONTROLLERFAN
      static uint8_t controller_fanSpeeds;
    #endif
    #if ENABLED(FAN_KICKSTART_TIME)
      static uint8_t fanKickstart;
    #endif

    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      static AdvancedPauseMenuResponse advanced_pause_menu_response;
    #endif

    #if HAS_FIL_RUNOUT || HAS_EXT_ENCODER
      static bool filament_ran_out;
    #endif

    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      static float motor_current[3 + DRIVER_EXTRUDERS];
    #endif

    #if HAS_CASE_LIGHT
      static int case_light_brightness;
      static bool case_light_on;
    #endif

    #if HAS_EXT_ENCODER
      static int32_t  encStepsSinceLastSignal[EXTRUDERS]; // when was the last signal
      static uint8_t  encLastSignal[EXTRUDERS];           // what was the last signal
      static int8_t   encLastDir[EXTRUDERS];
      static int32_t  encLastChangeAt[EXTRUDERS],
                      encErrorSteps[EXTRUDERS];
    #endif

  public: /** Public Function */

    static void setup();

    static void safe_delay(millis_t ms);

    static void setup_for_endstop_or_probe_move();
    static void clean_up_after_endstop_or_probe_move();

    static void get_destination_from_command();
    static bool get_target_tool_from_command(const uint16_t code);

    static void kill(const char *);
    static void Stop();
    static void quickstop_stepper();

    static void calculate_volumetric_multipliers();

    static void idle(bool no_stepper_sleep=false);
    static void manage_inactivity(bool ignore_stepper_queue=false);
    static void setInterruptEvent(const MK4duoInterruptEvent event);
    static void handle_Interrupt_Event();

    static void tool_change(const uint8_t tmp_extruder, const float fr_mm_s=0.0, bool no_move=false);

    #if HAS_MKMULTI_TOOLS
      static void MK_multi_tool_change(const uint8_t &e);
    #endif

    #if ENABLED(CNCROUTER)
      static void tool_change_cnc(const uint8_t tool_id, bool wait=true, bool raise_z=true);
    #endif

    #if FAN_COUNT > 0 && ENABLED(PROBING_FANS_OFF)
      static void fans_pause(const bool p);
    #endif

    #if ENABLED(SDSUPPORT)
      static void stopSDPrint(const bool store_location);
    #endif

    #if HAS_TEMP_HOTEND
      static void wait_heater(bool no_wait_for_cooling=true);
    #endif
    #if HAS_TEMP_BED
      static void wait_bed(bool no_wait_for_cooling=true);
    #endif
    #if HAS_TEMP_CHAMBER
      static void wait_chamber(bool no_wait_for_heating=true);
    #endif
    #if HAS_TEMP_COOLER
      static void wait_cooler(bool no_wait_for_heating=true);
    #endif

    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      #if HAS_BUZZER
        static void filament_change_beep(const int8_t max_beep_count, const bool init=false);
      #endif
      static void ensure_safe_temperature();
      static bool pause_print(const float &retract, const float &retract2, const float &z_lift, const float &x_pos, const float &y_pos,
                              const float &unload_length=0, const int8_t max_beep_count=0, const bool show_lcd=false);
      static void wait_for_filament_reload(const int8_t max_beep_count=0);
      static void resume_print(const float &load_length=0, const float &initial_extrude_length=0, const int8_t max_beep_count=0);
    #endif

    #if HAS_CASE_LIGHT
      static void update_case_light();
    #endif

    #if HAS_COLOR_LEDS
      static void set_led_color(const uint8_t r, const uint8_t g, const uint8_t b
                                #if ENABLED(RGBW_LED)
                                  , const uint8_t w=0
                                #endif
      );
    #endif

    static bool pin_is_protected(uint8_t pin);

    static void suicide();

    FORCE_INLINE static void setRunning(const bool run) { Running = run; }
    FORCE_INLINE static bool IsRunning() { return  Running; }
    FORCE_INLINE static bool IsStopped() { return !Running; }

  private: /** Private Parameters */

  private: /** Private Function */

    #if HAS_FIL_RUNOUT
      static void setup_filrunoutpin();
    #endif

    static void setup_powerhold();

    #if HAS_STEPPER_RESET
      static void disableStepperDrivers();
      static void enableStepperDrivers();
    #endif

    #if HAS_CONTROLLERFAN
      static void controllerFan();
    #endif

    static float calculate_volumetric_multiplier(const float diameter);

    static void invalid_extruder_error(const uint8_t e);

    #if ENABLED(IDLE_OOZING_PREVENT)
      IDLE_OOZING_retract(bool retracting);
    #endif

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      static void host_keepalive();
    #endif

    #if ENABLED(TEMP_STAT_LEDS)
      static void handle_status_leds();
    #endif

    #if ENABLED(HAVE_TMC2130)
      static void checkOverTemp();
    #endif

};

extern Printer printer;

// Define runplan for move axes
#if IS_KINEMATIC
  #define RUNPLAN(RATE_MM_S) planner.buffer_line_kinematic(mechanics.destination, RATE_MM_S, printer.active_extruder);
#else
  #define RUNPLAN(RATE_MM_S) mechanics.line_to_destination(RATE_MM_S);
#endif

#endif /* _PRINTER_H_ */

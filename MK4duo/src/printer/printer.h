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
 * printer.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _PRINTER_H_
#define _PRINTER_H_

enum PrinterMode {
  PRINTER_MODE_FFF,           // M450 S0 or M451
  PRINTER_MODE_LASER,         // M450 S1 or M452
  PRINTER_MODE_CNC,           // M450 S2 or M453
  PRINTER_MODE_PICKER,        // M450 S3 or M454
  PRINTER_MODE_SOLDER,        // M450 S4
  PRINTER_MODE_PLOTTER,
  PRINTER_MODE_COUNT
};

enum MK4duoInterruptEvent {
  INTERRUPT_EVENT_NONE,
  INTERRUPT_EVENT_FIL_RUNOUT,
  INTERRUPT_EVENT_DAV_SYSTEM,
  INTERRUPT_EVENT_ENC_DETECT
};

extern const char axis_codes[NUM_AXIS];

class Printer {

  public: /** Constructor */

    Printer() {}

  public: /** Public Parameters */

    static bool Running,
                pos_saved;

    static volatile bool  wait_for_heatup,
                          wait_for_user;

    static uint8_t host_keepalive_interval;

    static bool relative_mode,
                axis_relative_modes[];

    static long   currentLayer,
                  maxLayer;       // -1 = unknown
    static char   printName[21];  // max. 20 chars + 0
    static float  progress;

    static millis_t max_inactive_time;

    static MK4duoInterruptEvent interruptEvent;
    static PrinterMode          mode;

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      /**
       * States for managing MK4duo and host communication
       * MK4duo sends messages if blocked or busy
       */
      enum MK4duoBusyState {
        NOT_BUSY,           // Not in a handler
        IN_HANDLER,         // Processing a GCode
        IN_PROCESS,         // Known to be blocking command input (as in G29)
        WAIT_HEATER,        // Wait heater
        PAUSED_FOR_USER,    // Blocking pending any input
        PAUSED_FOR_INPUT,   // Blocking pending text input (concept)
        DOOR_OPEN           // Door open
      };
      static MK4duoBusyState busy_state;
      #define KEEPALIVE_STATE(n) do{ printer.busy_state = printer.n; }while(0)
    #else
      #define KEEPALIVE_STATE(n) NOOP
    #endif

    #if HAS_FIL_RUNOUT || HAS_EXT_ENCODER
      static bool filament_ran_out;
    #endif

    #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
      static float motor_current[3 + DRIVER_EXTRUDERS];
    #endif

    #if ENABLED(RFID_MODULE)
      static uint32_t Spool_ID[EXTRUDERS];
      static bool     RFID_ON,
                      Spool_must_read[EXTRUDERS],
                      Spool_must_write[EXTRUDERS];
    #endif

    #if ENABLED(NPR2)
      static uint8_t old_color; // old color for system NPR2
    #endif

    #if ENABLED(G38_PROBE_TARGET)
      static bool G38_move,        // flag to tell the interrupt handler that a G38 command is being run
                  G38_endstop_hit; // flag from the interrupt handler to indicate if the endstop went active
    #endif

    #if ENABLED(BARICUDA)
      static int baricuda_valve_pressure;
      static int baricuda_e_to_p_pressure;
    #endif

    #if ENABLED(EASY_LOAD)
      static bool allow_lengthy_extrude_once; // for load/unload
    #endif

    #if ENABLED(IDLE_OOZING_PREVENT)
      static bool IDLE_OOZING_enabled;
    #endif

    #if HAS_CHDK
      static millis_t chdkHigh;
      static bool chdkActive;
    #endif

  private: /** Private Parameters */

    #if ENABLED(IDLE_OOZING_PREVENT)
      static millis_t axis_last_activity;
      static bool     IDLE_OOZING_retracted[EXTRUDERS];
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

    #if ENABLED(SDSUPPORT)
      static void stopSDPrint(const bool store_location);
    #endif

    #if HAS_COLOR_LEDS
      static void set_led_color(const uint8_t r, const uint8_t g, const uint8_t b
                                #if ENABLED(RGBW_LED) || ENABLED(NEOPIXEL_RGBW_LED)
                                  , const uint8_t w=0
                                #endif
                                #if HAS_NEOPIXEL
                                  , bool isSequence=false
                                #endif
      );
    #endif

    static bool pin_is_protected(uint8_t pin);

    static void suicide();

    FORCE_INLINE static void setRunning(const bool run) { Running = run; }
    FORCE_INLINE static bool IsRunning() { return  Running; }
    FORCE_INLINE static bool IsStopped() { return !Running; }

  private: /** Private Function */

    #if HAS_FIL_RUNOUT
      static void setup_filrunoutpin();
    #endif

    static void setup_powerhold();

    static float calculate_volumetric_multiplier(const float diameter);

    #if ENABLED(IDLE_OOZING_PREVENT)
      static void IDLE_OOZING_retract(bool retracting);
    #endif

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      static void host_keepalive();
    #endif

    #if HAS_NEOPIXEL
      static void set_neopixel_color(const uint32_t color);
      static void setup_neopixel();
    #endif

    #if ENABLED(TEMP_STAT_LEDS)
      static void handle_status_leds();
    #endif

    #if ENABLED(HAVE_TMC2130)
      static void checkOverTemp();
    #endif

};

extern Printer printer;

#endif /* _PRINTER_H_ */

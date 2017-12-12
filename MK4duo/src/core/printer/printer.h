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

#define DEBUG_NONE                        0
#define DEBUG_ECHO                        1
#define DEBUG_INFO                        2
#define DEBUG_ERROR                       4
#define DEBUG_DRYRUN                      8
#define DEBUG_COMMUNICATION               16
#define DEBUG_LEVELING                    32
#define DEBUG_MESH_ADJUST                 64
#define PRINTER_FLAG1_RUNNING             1
#define PRINTER_FLAG1_POS_SAVED           2
#define PRINTER_FLAG1_RELATIVE_MODE       4
#define PRINTER_FLAG1_WAIT_FOR_USER       8
#define PRINTER_FLAG1_WAIT_FOR_HEATUP     16
#define PRINTER_FLAG1_FILAMENT_OUT        32
#define PRINTER_FLAG1_ALLOW_COLD_EXTRUDE  64
#define PRINTER_FLAG1_AUTOREPORT_TEMP     128

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
  INTERRUPT_EVENT_ENC_DETECT
};

extern const char axis_codes[NUM_AXIS];

class Printer {

  public: /** Constructor */

    Printer() {}

  public: /** Public Parameters */

    static bool     axis_relative_modes[];

    static long     currentLayer,
                    maxLayer;       // -1 = unknown

    static char     printName[21];  // max. 20 chars + 0

    static uint8_t  progress,
                    host_keepalive_interval;

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

    static uint8_t  mk_debug_flag,  // For debug
                    mk_1_flag;      // For various

    #if ENABLED(IDLE_OOZING_PREVENT)
      static millis_t axis_last_activity;
      static bool     IDLE_OOZING_retracted[EXTRUDERS];
    #endif

  public: /** Public Function */

    static void setup();  // Main setup
    static void loop();   // Main loop

    static void safe_delay(millis_t ms);

    static void setup_for_endstop_or_probe_move();
    static void clean_up_after_endstop_or_probe_move();

    static void kill(const char *);
    static void Stop();

    static void idle(bool no_stepper_sleep=false);
    static void setInterruptEvent(const MK4duoInterruptEvent event);

    static bool pin_is_protected(const Pin pin);

    static void suicide();

    static char GetStatusCharacter();

    #if ENABLED(IDLE_OOZING_PREVENT)
      static void IDLE_OOZING_retract(bool retracting);
    #endif

    // Flags function
    static void setDebugLevel(const uint8_t newLevel);
    FORCE_INLINE static uint8_t getDebugFlags()   { return mk_debug_flag; }
    FORCE_INLINE static bool debugEcho()          { return mk_debug_flag & DEBUG_ECHO; }
    FORCE_INLINE static bool debugInfo()          { return mk_debug_flag & DEBUG_INFO; }
    FORCE_INLINE static bool debugError()         { return mk_debug_flag & DEBUG_ERROR; }
    FORCE_INLINE static bool debugDryrun()        { return mk_debug_flag & DEBUG_DRYRUN; }
    FORCE_INLINE static bool debugCommunication() { return mk_debug_flag & DEBUG_COMMUNICATION; }
    FORCE_INLINE static bool debugLeveling()      { return mk_debug_flag & DEBUG_LEVELING; }
    FORCE_INLINE static bool debugMesh()          { return mk_debug_flag & DEBUG_MESH_ADJUST; }

    FORCE_INLINE static bool debugFlag(const uint8_t flag) {
      return (mk_debug_flag & flag);
    }
    FORCE_INLINE static void debugSet(const uint8_t flag) {
      setDebugLevel(mk_debug_flag | flag);
    }
    FORCE_INLINE static void debugReset(const uint8_t flag) {
      setDebugLevel(mk_debug_flag & ~flag);
    }

    FORCE_INLINE static void setRunning(const bool run) {
      mk_1_flag = (run ? mk_1_flag | PRINTER_FLAG1_RUNNING : mk_1_flag & ~PRINTER_FLAG1_RUNNING);
    }
    FORCE_INLINE static bool IsRunning()  { return mk_1_flag & PRINTER_FLAG1_RUNNING; }

    FORCE_INLINE static void setPosSaved(const bool val) {
      mk_1_flag = (val ? mk_1_flag | PRINTER_FLAG1_POS_SAVED : mk_1_flag & ~PRINTER_FLAG1_POS_SAVED);
    }
    FORCE_INLINE static bool isPosSaved() { return mk_1_flag & PRINTER_FLAG1_POS_SAVED; }

    FORCE_INLINE static void setRelativeMode(const bool val) {
      mk_1_flag = (val ? mk_1_flag | PRINTER_FLAG1_RELATIVE_MODE : mk_1_flag & ~PRINTER_FLAG1_RELATIVE_MODE);
    }
    FORCE_INLINE static bool isRelativeMode() { return mk_1_flag & PRINTER_FLAG1_RELATIVE_MODE; }

    FORCE_INLINE static void setWaitForUser(const bool val) {
      mk_1_flag = (val ? mk_1_flag | PRINTER_FLAG1_WAIT_FOR_USER : mk_1_flag & ~PRINTER_FLAG1_WAIT_FOR_USER);
    }
    FORCE_INLINE static bool isWaitForUser() { return mk_1_flag & PRINTER_FLAG1_WAIT_FOR_USER; }

    FORCE_INLINE static void setWaitForHeatUp(const bool val) {
      mk_1_flag = (val ? mk_1_flag | PRINTER_FLAG1_WAIT_FOR_HEATUP : mk_1_flag & ~PRINTER_FLAG1_WAIT_FOR_HEATUP);
    }
    FORCE_INLINE static bool isWaitForHeatUp() { return mk_1_flag & PRINTER_FLAG1_WAIT_FOR_HEATUP; }

    FORCE_INLINE static void setFilamentOut(const bool val) {
      mk_1_flag = (val ? mk_1_flag | PRINTER_FLAG1_FILAMENT_OUT : mk_1_flag & ~PRINTER_FLAG1_FILAMENT_OUT);
    }
    FORCE_INLINE static bool isFilamentOut() { return mk_1_flag & PRINTER_FLAG1_FILAMENT_OUT; }

    FORCE_INLINE static void setAllowColdExtrude(const bool val) {
      mk_1_flag = (val ? mk_1_flag | PRINTER_FLAG1_ALLOW_COLD_EXTRUDE : mk_1_flag & ~PRINTER_FLAG1_ALLOW_COLD_EXTRUDE);
    }
    FORCE_INLINE static bool isAllowColdExtrude() { return mk_1_flag & PRINTER_FLAG1_ALLOW_COLD_EXTRUDE; }

    FORCE_INLINE static void setAutoreportTemp(const bool val) {
      mk_1_flag = (val ? mk_1_flag | PRINTER_FLAG1_AUTOREPORT_TEMP : mk_1_flag & ~PRINTER_FLAG1_AUTOREPORT_TEMP);
    }
    FORCE_INLINE static bool isAutoreportTemp() { return mk_1_flag & PRINTER_FLAG1_AUTOREPORT_TEMP; }

  private: /** Private Function */

    static void manage_inactivity(bool ignore_stepper_queue=false);
    static void handle_Interrupt_Event();

    static void setup_powerhold();

    static void bracket_probe_move(const bool before);

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      static void host_keepalive();
    #endif

    #if ENABLED(TEMP_STAT_LEDS)
      static void handle_status_leds();
    #endif

};

extern Printer printer;

#endif /* _PRINTER_H_ */

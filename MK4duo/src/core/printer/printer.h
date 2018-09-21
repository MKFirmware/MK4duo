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

constexpr uint8_t debug_echo                = 1;
constexpr uint8_t debug_info                = 2;
constexpr uint8_t debug_error               = 4;
constexpr uint8_t debug_dryrun              = 8;
constexpr uint8_t debug_communication       = 16;
constexpr uint8_t debug_feature             = 32;
constexpr uint8_t debug_mesh_adjust         = 64;
constexpr uint8_t debug_simulation          = 128;

enum VariousBits : char {
  bit_running,
  bit_printing,
  bit_pos_saved,
  bit_relative_mode,
  bit_volumetric_enabled,
  bit_wait_for_user,
  bit_wait_for_heatup,
  bit_allow_cold_extrude,
  bit_autoreport_temp,
  bit_autoreport_sd,
  bit_suspend_autoreport,
  bit_abort_sd_printing,
  bit_filament_out,
  bit_g38_move
};

enum PrinterMode : char {
  PRINTER_MODE_FFF,           // M450 S0 or M451
  PRINTER_MODE_LASER,         // M450 S1 or M452
  PRINTER_MODE_CNC,           // M450 S2 or M453
  PRINTER_MODE_PICKER,        // M450 S3 or M454
  PRINTER_MODE_SOLDER,        // M450 S4
  PRINTER_MODE_PLOTTER,
  PRINTER_MODE_COUNT
};

enum MK4duoInterruptEvent : char {
  INTERRUPT_EVENT_NONE,
  INTERRUPT_EVENT_FIL_RUNOUT,
  INTERRUPT_EVENT_ENC_DETECT
};

/**
 * States for managing MK4duo and host communication
 * MK4duo sends messages if blocked or busy
 */
enum MK4duoBusyState : char {
  NotBusy,          // Not in a handler
  InHandler,        // Processing a GCode
  InProcess,        // Known to be blocking command input (as in G29)
  WaitHeater,       // Wait heater
  PausedforUser,    // Blocking pending any input
  PausedforInput,   // Blocking pending text input
  DoorOpen          // Door open
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

    static uint8_t  progress;

    static watch_t  max_inactivity_watch,
                    move_watch;

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      static watch_t  host_keepalive_watch;
    #endif

    static MK4duoInterruptEvent interruptEvent;
    static PrinterMode          mode;

    #if ENABLED(RFID_MODULE)
      static uint32_t Spool_ID[EXTRUDERS];
      static bool     RFID_ON,
                      Spool_must_read[EXTRUDERS],
                      Spool_must_write[EXTRUDERS];
    #endif

    #if ENABLED(BARICUDA)
      static int baricuda_valve_pressure;
      static int baricuda_e_to_p_pressure;
    #endif

    #if ENABLED(IDLE_OOZING_PREVENT)
      static bool IDLE_OOZING_enabled;
    #endif

    #if HAS_CHDK
      static watch_t chdk_watch;
      static bool chdkActive;
    #endif

  private: /** Private Parameters */

    static uint8_t  mk_debug_flag,    // For debug
                    mk_home_flag;     // For Homed

    static uint16_t mk_various_flag;  // For various

    #if ENABLED(IDLE_OOZING_PREVENT)
      static millis_t axis_last_activity;
      static bool     IDLE_OOZING_retracted[EXTRUDERS];
    #endif

  public: /** Public Function */

    static void setup();  // Main setup
    static void loop();   // Main loop

    static void check_periodical_actions();
    static void safe_delay(millis_t ms);

    static void setup_for_endstop_or_probe_move();
    static void clean_up_after_endstop_or_probe_move();

    static void quickstop_stepper();

    static void kill(const char *);
    static void Stop();

    static void idle(const bool ignore_stepper_queue=false);
    static void setInterruptEvent(const MK4duoInterruptEvent event);

    static bool pin_is_protected(const pin_t pin);

    static void suicide();

    #if ENABLED(IDLE_OOZING_PREVENT)
      static void IDLE_OOZING_retract(bool retracting);
    #endif

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      static void keepalive(const MK4duoBusyState state);
    #else
      FORCE_INLINE static void keepalive(const MK4duoBusyState state) { UNUSED(state); }
    #endif

    // Flag Debug function
    static void setDebugLevel(const uint8_t newLevel);
    FORCE_INLINE static uint8_t getDebugFlags()   { return mk_debug_flag; }
    FORCE_INLINE static bool debugEcho()          { return mk_debug_flag & debug_echo; }
    FORCE_INLINE static bool debugInfo()          { return mk_debug_flag & debug_info; }
    FORCE_INLINE static bool debugError()         { return mk_debug_flag & debug_error; }
    FORCE_INLINE static bool debugDryrun()        { return mk_debug_flag & debug_dryrun; }
    FORCE_INLINE static bool debugCommunication() { return mk_debug_flag & debug_communication; }
    FORCE_INLINE static bool debugFeature()       { return mk_debug_flag & debug_feature; }
    FORCE_INLINE static bool debugMesh()          { return mk_debug_flag & debug_mesh_adjust; }
    FORCE_INLINE static bool debugSimulation()    { return mk_debug_flag & debug_simulation; }

    FORCE_INLINE static bool debugFlag(const uint8_t flag) {
      return (mk_debug_flag & flag);
    }
    FORCE_INLINE static void debugSet(const uint8_t flag) {
      setDebugLevel(mk_debug_flag | flag);
    }
    FORCE_INLINE static void debugReset(const uint8_t flag) {
      setDebugLevel(mk_debug_flag & ~flag);
    }

    // Flag1 Home function
    FORCE_INLINE static void setXHomed(const bool onoff) {
      SET_BIT(mk_home_flag, _BV(X_AXIS), onoff);
    }
    FORCE_INLINE static bool isXHomed() { return TEST(mk_home_flag, _BV(X_AXIS)); }

    FORCE_INLINE static void setYHomed(const bool onoff) {
      SET_BIT(mk_home_flag, _BV(Y_AXIS), onoff);
    }
    FORCE_INLINE static bool isYHomed() { return TEST(mk_home_flag, _BV(Y_AXIS)); }

    FORCE_INLINE static void setZHomed(const bool onoff) {
      SET_BIT(mk_home_flag, _BV(Z_AXIS), onoff);
    }
    FORCE_INLINE static bool isZHomed() { return TEST(mk_home_flag, _BV(Z_AXIS)); }

    FORCE_INLINE static void setAxisHomed(const AxisEnum axis, const bool onoff) {
      switch (axis) {
        case X_AXIS: setXHomed(onoff); break;
        case Y_AXIS: setYHomed(onoff); break;
        case Z_AXIS: setZHomed(onoff); break;
      }
    }
    FORCE_INLINE static bool isAxisHomed(const AxisEnum axis) {
      switch (axis) {
        case X_AXIS: return isXHomed(); break;
        case Y_AXIS: return isYHomed(); break;
        case Z_AXIS: return isZHomed(); break;
      }
    }

    FORCE_INLINE static void unsetHomedAll() {
      CBI(mk_home_flag, _BV(X_AXIS));
      CBI(mk_home_flag, _BV(Y_AXIS));
      CBI(mk_home_flag, _BV(Z_AXIS));
    }
    FORCE_INLINE static bool isHomedAll() { return isXHomed() && isYHomed() && isZHomed(); }

    // Flag1 Various function
    FORCE_INLINE static void setRunning(const bool onoff) {
      SET_BIT(mk_various_flag, bit_running, onoff);
    }
    FORCE_INLINE static bool isRunning() { return TEST(mk_various_flag, bit_running); }

    FORCE_INLINE static void setPosSaved(const bool onoff) {
      SET_BIT(mk_various_flag, bit_pos_saved, onoff);
    }
    FORCE_INLINE static bool isPosSaved() { return TEST(mk_various_flag, bit_pos_saved); }

    FORCE_INLINE static void setRelativeMode(const bool onoff) {
      SET_BIT(mk_various_flag, bit_relative_mode, onoff);
    }
    FORCE_INLINE static bool isRelativeMode() { return TEST(mk_various_flag, bit_relative_mode); }

    FORCE_INLINE static void setVolumetric(const bool onoff) {
      SET_BIT(mk_various_flag, bit_volumetric_enabled, onoff);
    }
    FORCE_INLINE static bool isVolumetric() { return TEST(mk_various_flag, bit_volumetric_enabled); }

    FORCE_INLINE static void setWaitForUser(const bool onoff) {
      SET_BIT(mk_various_flag, bit_wait_for_user, onoff);
    }
    FORCE_INLINE static bool isWaitForUser() { return TEST(mk_various_flag, bit_wait_for_user); }

    FORCE_INLINE static void setWaitForHeatUp(const bool onoff) {
      SET_BIT(mk_various_flag, bit_wait_for_heatup, onoff);
    }
    FORCE_INLINE static bool isWaitForHeatUp() { return TEST(mk_various_flag, bit_wait_for_heatup); }

    FORCE_INLINE static void setAllowColdExtrude(const bool onoff) {
      SET_BIT(mk_various_flag, bit_allow_cold_extrude, onoff);
    }
    FORCE_INLINE static bool isAllowColdExtrude() { return TEST(mk_various_flag, bit_allow_cold_extrude); }

    FORCE_INLINE static void setAutoreportTemp(const bool onoff) {
      SET_BIT(mk_various_flag, bit_autoreport_temp, onoff);
    }
    FORCE_INLINE static bool isAutoreportTemp() { return TEST(mk_various_flag, bit_autoreport_temp); }

    FORCE_INLINE static void setAutoreportSD(const bool onoff) {
      SET_BIT(mk_various_flag, bit_autoreport_sd, onoff);
    }
    FORCE_INLINE static bool isAutoreportSD() { return TEST(mk_various_flag, bit_autoreport_sd); }

    FORCE_INLINE static void setSuspendAutoreport(const bool onoff) {
      SET_BIT(mk_various_flag, bit_suspend_autoreport, onoff);
    }
    FORCE_INLINE static bool isSuspendAutoreport() { return TEST(mk_various_flag, bit_suspend_autoreport); }

    FORCE_INLINE static void setAbortSDprinting(const bool onoff) {
      SET_BIT(mk_various_flag, bit_abort_sd_printing, onoff);
    }
    FORCE_INLINE static bool isAbortSDprinting() { return TEST(mk_various_flag, bit_abort_sd_printing); }

    FORCE_INLINE static void setFilamentOut(const bool onoff) {
      SET_BIT(mk_various_flag, bit_filament_out, onoff);
    }
    FORCE_INLINE static bool isFilamentOut() { return TEST(mk_various_flag, bit_filament_out); }

    FORCE_INLINE static void setG38Move(const bool onoff) {
      SET_BIT(mk_various_flag, bit_running, onoff);
      mk_various_flag = (onoff ? mk_various_flag | bit_g38_move : mk_various_flag & ~bit_g38_move);
    }
    FORCE_INLINE static bool IsG38Move() { return mk_various_flag & bit_g38_move; }

    FORCE_INLINE static bool reset_flag() { mk_home_flag = 0; mk_various_flag = 0; }

  private: /** Private Function */

    static void setup_pinout();

    static void handle_interrupt_events();

    static void handle_safety_watch();

    static void bracket_probe_move(const bool before);

    #if ENABLED(TEMP_STAT_LEDS)
      static void handle_status_leds();
    #endif

};

extern Printer printer;

#endif /* _PRINTER_H_ */

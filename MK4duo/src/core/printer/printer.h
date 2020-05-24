/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
#pragma once

/**
 * printer.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

union debug_flag_t {
  uint8_t all;
  struct {
    bool  echo          : 1;
    bool  info          : 1;
    bool  errors        : 1;
    bool  dryrun        : 1;
    bool  communication : 1;
    bool  feature       : 1;
    bool  mesh          : 1;
    bool  simulation    : 1;
  };
  debug_flag_t() { all = 0x00; }
};

union various_flag_t {
  uint16_t all;
  struct {
    bool  Running           : 1;
    bool  PosSaved          : 1;
    bool  WaitForUser       : 1;
    bool  WaitForHeatUp     : 1;
    bool  AllowColdExtrude  : 1;
    bool  AutoreportTemp    : 1;
    bool  SuspendAutoreport : 1;
    bool  G38Move           : 1;
    bool  statistics_loaded : 1;
    bool  RFID              : 1;
    bool  bit10             : 1;
    bool  bit11             : 1;
    bool  bit12             : 1;
    bool  bit13             : 1;
    bool  bit14             : 1;
    bool  bit15             : 1;
  };
  various_flag_t() { all = 0x0000; }
};

class Printer {

  public: /** Constructor */

    Printer() {}

  public: /** Public Parameters */

    static debug_flag_t   debug_flag;   // For debug
    static various_flag_t various_flag; // For various

    static int16_t  currentLayer,
                    maxLayer;       // -1 = unknown

    static char     printName[21];  // max. 20 chars + 0

    static uint8_t  progress;

    static uint16_t safety_time,
                    max_inactive_time,
                    move_time;

    static long_timer_t max_inactivity_timer,
                        move_timer;

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      static BusyStateEnum busy_state;
      static uint8_t  host_keepalive_time;
      #define PRINTER_KEEPALIVE(N)  REMEMBER(_KA_, printer.busy_state, N)
    #else
      #define PRINTER_KEEPALIVE(N)  NOOP
    #endif

    static PrinterModeEnum mode;

    #if ENABLED(BARICUDA)
      static int baricuda_valve_pressure;
      static int baricuda_e_to_p_pressure;
    #endif

    #if HAS_CHDK
      static short_timer_t chdk_timer;
    #endif

  public: /** Public Function */

    static void setup_pinout();
    static void factory_parameters();

    static void check_periodical_actions();
    static void safe_delay(millis_l ms);

    static void quickstop_stepper();

    static void kill(PGM_P const lcd_msg=nullptr, const bool steppers_off=false);
    static void minikill(const bool steppers_off=false);

    static void stop();

    static void zero_fan_speed();

    static void idle(const bool no_stepper_sleep=false);
    inline static void idle_no_sleep() { idle(true); }

    static bool isPrinting();
    static bool isPaused();

    static bool pin_is_protected(const pin_t pin);

    static void print_M353();

    #if HAS_SD_SUPPORT
      static void abort_sd_printing();
      static void finish_sd_printing();
    #endif

    #if HAS_RESUME_CONTINUE
      void wait_for_user_response(millis_l ms=0, const bool no_sleep=false);
    #endif

    #if HAS_SUICIDE
      static void suicide();
    #endif

    #if HAS_KILL
      #ifndef KILL_PIN_STATE
        #define KILL_PIN_STATE LOW
      #endif
      inline static bool kill_state() { return READ(KILL_PIN) == KILL_PIN_STATE; }
    #endif

    FORCE_INLINE static void reset_move_timer() { move_timer.start(); }

    // Flag Debug function
    static void setDebugLevel(const uint8_t newLevel);
    FORCE_INLINE static uint8_t getDebugFlags()   { return debug_flag.all; }
    FORCE_INLINE static bool debugEcho()          { return debug_flag.echo; }
    FORCE_INLINE static bool debugInfo()          { return debug_flag.info; }
    FORCE_INLINE static bool debugError()         { return debug_flag.errors; }
    FORCE_INLINE static bool debugDryrun()        { return debug_flag.dryrun; }
    FORCE_INLINE static bool debugCommunication() { return debug_flag.communication; }
    FORCE_INLINE static bool debugFeature()       { return debug_flag.feature; }
    FORCE_INLINE static bool debugMesh()          { return debug_flag.mesh; }
    FORCE_INLINE static bool debugSimulation()    { return debug_flag.simulation; }

    FORCE_INLINE static bool debugFlag(const uint8_t flag) {
      return (debug_flag.all & flag);
    }
    FORCE_INLINE static void debugSet(const uint8_t flag) {
      setDebugLevel(debug_flag.all | flag);
    }
    FORCE_INLINE static void debugReset(const uint8_t flag) {
      setDebugLevel(debug_flag.all & ~flag);
    }

    // Various flag bit 0 Running
    FORCE_INLINE static void setRunning(const bool onoff) { various_flag.Running = onoff; }
    FORCE_INLINE static bool isRunning() { return various_flag.Running; }
    FORCE_INLINE static bool isStopped() { return !isRunning(); }

    // Various flag bit 1 PosSaved
    FORCE_INLINE static void setPosSaved(const bool onoff) { various_flag.PosSaved = onoff; }
    FORCE_INLINE static bool isPosSaved() { return various_flag.PosSaved; }

    // Various flag bit 3 WaitForUser
    FORCE_INLINE static void setWaitForUser(const bool onoff) { various_flag.WaitForUser = onoff; }
    FORCE_INLINE static bool isWaitForUser() { return various_flag.WaitForUser; }

    // Various flag bit 4 WaitForHeatUp
    FORCE_INLINE static void setWaitForHeatUp(const bool onoff) { various_flag.WaitForHeatUp = onoff; }
    FORCE_INLINE static bool isWaitForHeatUp() { return various_flag.WaitForHeatUp; }

    // Various flag bit 5 AllowColdExtrude
    FORCE_INLINE static void setAllowColdExtrude(const bool onoff) { various_flag.AllowColdExtrude = onoff; }
    FORCE_INLINE static bool isAllowColdExtrude() { return various_flag.AllowColdExtrude; }

    // Various flag bit 6 AutoreportTemp
    FORCE_INLINE static void setAutoreportTemp(const bool onoff) { various_flag.AutoreportTemp = onoff; }
    FORCE_INLINE static bool isAutoreportTemp() { return various_flag.AutoreportTemp; }

    // Various flag bit 7 SuspendAutoreport
    FORCE_INLINE static void setSuspendAutoreport(const bool onoff) { various_flag.SuspendAutoreport = onoff; }
    FORCE_INLINE static bool isSuspendAutoreport() { return various_flag.SuspendAutoreport; }

    // Various flag bit 8 G38Move
    FORCE_INLINE static void setG38Move(const bool onoff) { various_flag.G38Move = onoff; }
    FORCE_INLINE static bool IsG38Move() { return various_flag.G38Move; }

    // Various flag bit 9 Statistics loaded
    FORCE_INLINE static void setStatisticsLoaded(const bool onoff) { various_flag.statistics_loaded = onoff; }
    FORCE_INLINE static bool IsStatisticsLoaded() { return various_flag.statistics_loaded; }

    // Various flag bit 10 RFID_ON
    FORCE_INLINE static void setRfid(const bool onoff) { various_flag.RFID = onoff; }
    FORCE_INLINE static bool IsRfid() { return various_flag.RFID; }

  private: /** Private Function */

    static void handle_safety_watch();

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      static void host_keepalive_tick();
    #endif

    #if ENABLED(TEMP_STAT_LEDS)
      static void handle_status_leds();
    #endif

};

extern Printer printer;

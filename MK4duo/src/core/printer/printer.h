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
#pragma once

/**
 * printer.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

union flagdebug_t {
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
  flagdebug_t() { all = false; }
};

union flagVarious_t {
  uint16_t all;
  struct {
    bool  Running           : 1;
    bool  PosSaved          : 1;
    bool  RelativeMode      : 1;
    bool  Volumetric        : 1;
    bool  WaitForUser       : 1;
    bool  WaitForHeatUp     : 1;
    bool  AllowColdExtrude  : 1;
    bool  AutoreportTemp    : 1;
    bool  SuspendAutoreport : 1;
    bool  FilamentOut       : 1;
    bool  G38Move           : 1;
    bool  statistics_loaded : 1;
    bool  bit12             : 1;
    bool  bit13             : 1;
    bool  bit14             : 1;
    bool  bit15             : 1;
  };
  flagVarious_t() { all = 0; }
};

extern const char axis_codes[NUM_AXIS];

class Printer {

  public: /** Constructor */

    Printer() {}

  public: /** Public Parameters */

    static flagdebug_t    debug_flag;   // For debug
    static flagVarious_t  various_flag; // For various

    static bool     axis_relative_modes[];

    static int16_t  currentLayer,
                    maxLayer;       // -1 = unknown

    static char     printName[21];  // max. 20 chars + 0

    static uint8_t  progress;

    static watch_t  max_inactivity_watch,
                    move_watch;

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      static watch_t  host_keepalive_watch;
    #endif

    static InterruptEventEnum interruptEvent;
    static PrinterModeEnum    mode;

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

    static void kill(PGM_P const lcd_msg=NULL);
    static void minikill();

    static void Stop();

    static void idle(const bool ignore_stepper_queue=false);
    static void setInterruptEvent(const InterruptEventEnum event);

    static bool isPrinting();
    static bool isPaused();

    static bool pin_is_protected(const pin_t pin);

    #if HAS_SUICIDE
      static void suicide();
    #endif

    #if ENABLED(IDLE_OOZING_PREVENT)
      static void IDLE_OOZING_retract(bool retracting);
    #endif

    #if ENABLED(HOST_KEEPALIVE_FEATURE)
      static void keepalive(const BusyStateEnum state);
    #else
      FORCE_INLINE static void keepalive(const BusyStateEnum state) { UNUSED(state); }
    #endif

    FORCE_INLINE static void zero_fan_speed() {
      #if FAN_COUNT > 0
        LOOP_FAN() fans[f].Speed = 0;
      #endif
    }

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

    // Various flag bit 2 RelativeMode
    FORCE_INLINE static void setRelativeMode(const bool onoff) { various_flag.RelativeMode = onoff; }
    FORCE_INLINE static bool isRelativeMode() { return various_flag.RelativeMode; }

    // Various flag bit 3 Volumetric
    FORCE_INLINE static void setVolumetric(const bool onoff) { various_flag.Volumetric = onoff; }
    FORCE_INLINE static bool isVolumetric() { return various_flag.Volumetric; }

    // Various flag bit 4 WaitForUser
    FORCE_INLINE static void setWaitForUser(const bool onoff) { various_flag.WaitForUser = onoff; }
    FORCE_INLINE static bool isWaitForUser() { return various_flag.WaitForUser; }

    // Various flag bit 5 WaitForHeatUp
    FORCE_INLINE static void setWaitForHeatUp(const bool onoff) { various_flag.WaitForHeatUp = onoff; }
    FORCE_INLINE static bool isWaitForHeatUp() { return various_flag.WaitForHeatUp; }

    // Various flag bit 6 AllowColdExtrude
    FORCE_INLINE static void setAllowColdExtrude(const bool onoff) { various_flag.AllowColdExtrude = onoff; }
    FORCE_INLINE static bool isAllowColdExtrude() { return various_flag.AllowColdExtrude; }

    // Various flag bit 7 AutoreportTemp
    FORCE_INLINE static void setAutoreportTemp(const bool onoff) { various_flag.AutoreportTemp = onoff; }
    FORCE_INLINE static bool isAutoreportTemp() { return various_flag.AutoreportTemp; }

    // Various flag bit 8 SuspendAutoreport
    FORCE_INLINE static void setSuspendAutoreport(const bool onoff) { various_flag.SuspendAutoreport = onoff; }
    FORCE_INLINE static bool isSuspendAutoreport() { return various_flag.SuspendAutoreport; }

    // Various flag bit 9 FilamentOut
    FORCE_INLINE static void setFilamentOut(const bool onoff) { various_flag.FilamentOut = onoff; }
    FORCE_INLINE static bool isFilamentOut() { return various_flag.FilamentOut; }

    // Various flag bit 10 G38Move
    FORCE_INLINE static void setG38Move(const bool onoff) { various_flag.G38Move = onoff; }
    FORCE_INLINE static bool IsG38Move() { return various_flag.G38Move; }

    // Various flag bit 11 Statistics loaded
    FORCE_INLINE static void setStatisticsLoaded(const bool onoff) { various_flag.statistics_loaded = onoff; }
    FORCE_INLINE static bool IsStatisticsLoaded() { return various_flag.statistics_loaded; }

    FORCE_INLINE static bool reset_flag() { various_flag.all = 0; }

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

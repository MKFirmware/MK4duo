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

    static flagbyte_t debug_flag,   // For debug
                      home_flag;    // For Homed

    static flagword_t various_flag; // For various

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
    static void setInterruptEvent(const MK4duoInterruptEvent event);

    static bool isPrinting();
    static bool pin_is_protected(const pin_t pin);

    #if HAS_SUICIDE
      static void suicide();
    #endif

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
    FORCE_INLINE static uint8_t getDebugFlags()   { return debug_flag._byte; }
    FORCE_INLINE static bool debugEcho()          { return debug_flag._byte & debug_echo; }
    FORCE_INLINE static bool debugInfo()          { return debug_flag._byte & debug_info; }
    FORCE_INLINE static bool debugError()         { return debug_flag._byte & debug_error; }
    FORCE_INLINE static bool debugDryrun()        { return debug_flag._byte & debug_dryrun; }
    FORCE_INLINE static bool debugCommunication() { return debug_flag._byte & debug_communication; }
    FORCE_INLINE static bool debugFeature()       { return debug_flag._byte & debug_feature; }
    FORCE_INLINE static bool debugMesh()          { return debug_flag._byte & debug_mesh_adjust; }
    FORCE_INLINE static bool debugSimulation()    { return debug_flag._byte & debug_simulation; }

    FORCE_INLINE static bool debugFlag(const uint8_t flag) {
      return (debug_flag._byte & flag);
    }
    FORCE_INLINE static void debugSet(const uint8_t flag) {
      setDebugLevel(debug_flag._byte | flag);
    }
    FORCE_INLINE static void debugReset(const uint8_t flag) {
      setDebugLevel(debug_flag._byte & ~flag);
    }

    // Home flag bit 0 X homed
    FORCE_INLINE static void setXHomed(const bool onoff) { home_flag.bit0 = onoff; }
    FORCE_INLINE static bool isXHomed() { return home_flag.bit0; }

    // Home flag bit 1 Y homed
    FORCE_INLINE static void setYHomed(const bool onoff) { home_flag.bit1 = onoff; }
    FORCE_INLINE static bool isYHomed() { return home_flag.bit1; }

    // Home flag bit 2 Z homed
    FORCE_INLINE static void setZHomed(const bool onoff) { home_flag.bit2 = onoff; }
    FORCE_INLINE static bool isZHomed() { return home_flag.bit2; }

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

    FORCE_INLINE static void unsetHomedAll() { home_flag._byte = 0; }
    FORCE_INLINE static bool isHomedAll() { return isXHomed() && isYHomed() && isZHomed(); }

    // Various flag bit 0 Running
    FORCE_INLINE static void setRunning(const bool onoff) { various_flag.bit0 = onoff; }
    FORCE_INLINE static bool isRunning() { return various_flag.bit0; }
    FORCE_INLINE static bool isStopped() { return !isRunning(); }

    // Various flag bit 1 PosSaved
    FORCE_INLINE static void setPosSaved(const bool onoff) { various_flag.bit1 = onoff; }
    FORCE_INLINE static bool isPosSaved() { return various_flag.bit1; }

    // Various flag bit 2 RelativeMode
    FORCE_INLINE static void setRelativeMode(const bool onoff) { various_flag.bit2 = onoff; }
    FORCE_INLINE static bool isRelativeMode() { return various_flag.bit2; }

    // Various flag bit 3 Volumetric
    FORCE_INLINE static void setVolumetric(const bool onoff) { various_flag.bit3 = onoff; }
    FORCE_INLINE static bool isVolumetric() { return various_flag.bit3; }

    // Various flag bit 4 WaitForUser
    FORCE_INLINE static void setWaitForUser(const bool onoff) { various_flag.bit4 = onoff; }
    FORCE_INLINE static bool isWaitForUser() { return various_flag.bit4; }

    // Various flag bit 5 WaitForHeatUp
    FORCE_INLINE static void setWaitForHeatUp(const bool onoff) { various_flag.bit5 = onoff; }
    FORCE_INLINE static bool isWaitForHeatUp() { return various_flag.bit5; }

    // Various flag bit 6 AllowColdExtrude
    FORCE_INLINE static void setAllowColdExtrude(const bool onoff) { various_flag.bit6 = onoff; }
    FORCE_INLINE static bool isAllowColdExtrude() { return various_flag.bit6; }

    // Various flag bit 7 AutoreportTemp
    FORCE_INLINE static void setAutoreportTemp(const bool onoff) { various_flag.bit7 = onoff; }
    FORCE_INLINE static bool isAutoreportTemp() { return various_flag.bit7; }

    // Various flag bit 8 SuspendAutoreport
    FORCE_INLINE static void setSuspendAutoreport(const bool onoff) { various_flag.bit8 = onoff; }
    FORCE_INLINE static bool isSuspendAutoreport() { return various_flag.bit8; }

    // Various flag bit 9 FilamentOut
    FORCE_INLINE static void setFilamentOut(const bool onoff) { various_flag.bit9 = onoff; }
    FORCE_INLINE static bool isFilamentOut() { return various_flag.bit9; }

    // Various flag bit 10 G38Move
    FORCE_INLINE static void setG38Move(const bool onoff) { various_flag.bit10 = onoff; }
    FORCE_INLINE static bool IsG38Move() { return various_flag.bit10; }

    FORCE_INLINE static bool reset_flag() { home_flag._byte = 0; various_flag._word = 0; }

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

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

constexpr const uint8_t debug_echo                = 1;
constexpr const uint8_t debug_info                = 2;
constexpr const uint8_t debug_error               = 4;
constexpr const uint8_t debug_dryrun              = 8;
constexpr const uint8_t debug_communication       = 16;
constexpr const uint8_t debug_leveling            = 32;
constexpr const uint8_t debug_mesh_adjust         = 64;
constexpr const uint8_t debug_simulation          = 128;
constexpr const uint8_t flag1_running             = 1;
constexpr const uint8_t flag1_printing            = 2;
constexpr const uint8_t flag1_endstop_enabled     = 4;
constexpr const uint8_t flag1_endstop_globally    = 8;
constexpr const uint8_t flag1_soft_endstop        = 16;
constexpr const uint8_t flag1_probe_endstop       = 32;
constexpr const uint8_t flag1_g38_move            = 64;
constexpr const uint8_t flag1_g38_endstop_hit     = 128;
constexpr const uint8_t flag2_pos_saved           = 1;
constexpr const uint8_t flag2_relative_mode       = 2;
constexpr const uint8_t flag2_volumetric_enabled  = 4;
constexpr const uint8_t flag2_wait_for_user       = 8;
constexpr const uint8_t flag2_wait_for_heatup     = 16;
constexpr const uint8_t flag2_filament_out        = 32;
constexpr const uint8_t flag2_allow_cold_extrude  = 64;
constexpr const uint8_t flag2_autoreport_temp     = 128;
constexpr const uint8_t flag3_all_homed           = 1;
constexpr const uint8_t flag3_x_homed             = 2;
constexpr const uint8_t flag3_y_homed             = 4;
constexpr const uint8_t flag3_z_homed             = 8;
constexpr const uint8_t flag3_x_known_position    = 16;
constexpr const uint8_t flag3_y_known_position    = 32;
constexpr const uint8_t flag3_z_known_position    = 64;

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
                    mk_1_flag,      // For Endstop
                    mk_2_flag,      // For various
                    mk_3_flag;      // For Homed

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

    static bool pin_is_protected(const pin_t pin);

    static void suicide();

    static char GetStatusCharacter();

    #if ENABLED(IDLE_OOZING_PREVENT)
      static void IDLE_OOZING_retract(bool retracting);
    #endif

    // Flags function
    static void setDebugLevel(const uint8_t newLevel);
    FORCE_INLINE static uint8_t getDebugFlags()   { return mk_debug_flag; }
    FORCE_INLINE static bool debugEcho()          { return mk_debug_flag & debug_echo; }
    FORCE_INLINE static bool debugInfo()          { return mk_debug_flag & debug_info; }
    FORCE_INLINE static bool debugError()         { return mk_debug_flag & debug_error; }
    FORCE_INLINE static bool debugDryrun()        { return mk_debug_flag & debug_dryrun; }
    FORCE_INLINE static bool debugCommunication() { return mk_debug_flag & debug_communication; }
    FORCE_INLINE static bool debugLeveling()      { return mk_debug_flag & debug_leveling; }
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

    FORCE_INLINE static void setRunning(const bool run) {
      mk_1_flag = (run ? mk_1_flag | flag1_running : mk_1_flag & ~flag1_running);
    }
    FORCE_INLINE static bool IsRunning() { return mk_1_flag & flag1_running; }

    FORCE_INLINE static void setEndstopEnabled(const bool onoff) {
      mk_1_flag = (onoff ? mk_1_flag | flag1_endstop_enabled : mk_1_flag & ~flag1_endstop_enabled);
    }
    FORCE_INLINE static bool IsEndstopEnabled() { return mk_1_flag & flag1_endstop_enabled; }

    FORCE_INLINE static void setEndstopGlobally(const bool onoff) {
      mk_1_flag = (onoff ? mk_1_flag | flag1_endstop_globally : mk_1_flag & ~flag1_endstop_globally);
      setEndstopEnabled(onoff);
    }
    FORCE_INLINE static bool IsEndstopGlobally() { return mk_1_flag & flag1_endstop_globally; }

    // Disable / Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    FORCE_INLINE static void setNotHoming() { setEndstopEnabled(IsEndstopGlobally()); }

    FORCE_INLINE static void setSoftEndstop(const bool onoff) {
      mk_1_flag = (onoff ? mk_1_flag | flag1_soft_endstop : mk_1_flag & ~flag1_soft_endstop);
    }
    FORCE_INLINE static bool IsSoftEndstop() { return mk_1_flag & flag1_soft_endstop; }

    FORCE_INLINE static void setProbeEndstop(const bool onoff) {
      mk_1_flag = (onoff ? mk_1_flag | flag1_probe_endstop : mk_1_flag & ~flag1_probe_endstop);
    }
    FORCE_INLINE static bool IsProbeEndstop() { return mk_1_flag & flag1_probe_endstop; }

    FORCE_INLINE static void setG38Move(const bool onoff) {
      mk_1_flag = (onoff ? mk_1_flag | flag1_g38_move : mk_1_flag & ~flag1_g38_move);
    }
    FORCE_INLINE static bool IsG38Move() { return mk_1_flag & flag1_g38_move; }

    FORCE_INLINE static void setG38EndstopHit(const bool onoff) {
      mk_1_flag = (onoff ? mk_1_flag | flag1_g38_endstop_hit : mk_1_flag & ~flag1_g38_endstop_hit);
    }
    FORCE_INLINE static bool IsG38EndstopHit() { return mk_1_flag & flag1_g38_endstop_hit; }

    FORCE_INLINE static void setPosSaved(const bool onoff) {
      mk_2_flag = (onoff ? mk_2_flag | flag2_pos_saved : mk_2_flag & ~flag2_pos_saved);
    }
    FORCE_INLINE static bool isPosSaved() { return mk_2_flag & flag2_pos_saved; }

    FORCE_INLINE static void setRelativeMode(const bool onoff) {
      mk_2_flag = (onoff ? mk_2_flag | flag2_relative_mode : mk_2_flag & ~flag2_relative_mode);
    }
    FORCE_INLINE static bool isRelativeMode() { return mk_2_flag & flag2_relative_mode; }

    FORCE_INLINE static void setVolumetric(const bool onoff) {
      mk_2_flag = (onoff ? mk_2_flag | flag2_volumetric_enabled : mk_2_flag & ~flag2_volumetric_enabled);
    }
    FORCE_INLINE static bool isVolumetric() { return mk_2_flag & flag2_volumetric_enabled; }

    FORCE_INLINE static void setWaitForUser(const bool onoff) {
      mk_2_flag = (onoff ? mk_2_flag | flag2_wait_for_user : mk_2_flag & ~flag2_wait_for_user);
    }
    FORCE_INLINE static bool isWaitForUser() { return mk_2_flag & flag2_wait_for_user; }

    FORCE_INLINE static void setWaitForHeatUp(const bool onoff) {
      mk_2_flag = (onoff ? mk_2_flag | flag2_wait_for_heatup : mk_2_flag & ~flag2_wait_for_heatup);
    }
    FORCE_INLINE static bool isWaitForHeatUp() { return mk_2_flag & flag2_wait_for_heatup; }

    FORCE_INLINE static void setFilamentOut(const bool onoff) {
      mk_2_flag = (onoff ? mk_2_flag | flag2_filament_out : mk_2_flag & ~flag2_filament_out);
    }
    FORCE_INLINE static bool isFilamentOut() { return mk_2_flag & flag2_filament_out; }

    FORCE_INLINE static void setAllowColdExtrude(const bool onoff) {
      mk_2_flag = (onoff ? mk_2_flag | flag2_allow_cold_extrude : mk_2_flag & ~flag2_allow_cold_extrude);
    }
    FORCE_INLINE static bool isAllowColdExtrude() { return mk_2_flag & flag2_allow_cold_extrude; }

    FORCE_INLINE static void setAutoreportTemp(const bool onoff) {
      mk_2_flag = (onoff ? mk_2_flag | flag2_autoreport_temp : mk_2_flag & ~flag2_autoreport_temp);
    }
    FORCE_INLINE static bool isAutoreportTemp() { return mk_2_flag & flag2_autoreport_temp; }

    FORCE_INLINE static void unsetHomedAll() {
      mk_3_flag &= ~flag3_all_homed;
      mk_3_flag &= ~(flag3_x_homed | flag3_y_homed | flag3_z_homed);
    }
    FORCE_INLINE static void updateHomedAll() {
      bool onoff = isXHomed() && isYHomed() && isZHomed();
      mk_3_flag = (onoff ? mk_3_flag | flag3_all_homed : mk_3_flag & ~flag3_all_homed);
    }
    FORCE_INLINE static bool isHomedAll() { return mk_3_flag & flag3_all_homed; }

    FORCE_INLINE static void setXHomed(const bool onoff) {
      mk_3_flag = (onoff ? mk_3_flag | flag3_x_homed : mk_3_flag & ~flag3_x_homed);
      updateHomedAll();
    }
    FORCE_INLINE static bool isXHomed() { return mk_3_flag & flag3_x_homed; }

    FORCE_INLINE static void setYHomed(const bool onoff) {
      mk_3_flag = (onoff ? mk_3_flag | flag3_y_homed : mk_3_flag & ~flag3_y_homed);
      updateHomedAll();
    }
    FORCE_INLINE static bool isYHomed() { return mk_3_flag & flag3_y_homed; }

    FORCE_INLINE static void setZHomed(const bool onoff) {
      mk_3_flag = (onoff ? mk_3_flag | flag3_z_homed : mk_3_flag & ~flag3_z_homed);
      updateHomedAll();
    }
    FORCE_INLINE static bool isZHomed() { return mk_3_flag & flag3_z_homed; }

    FORCE_INLINE static void setAxisHomed(const AxisEnum axis, const bool onoff) {
      switch (axis) {
        case X_AXIS: setXHomed(onoff); break;
        case Y_AXIS: setYHomed(onoff); break;
        case Z_AXIS: setZHomed(onoff); break;
      }
      updateHomedAll();
    }
    FORCE_INLINE static bool isAxisHomed(const AxisEnum axis) {
      switch (axis) {
        case X_AXIS: return isXHomed(); break;
        case Y_AXIS: return isYHomed(); break;
        case Z_AXIS: return isZHomed(); break;
      }
    }

    FORCE_INLINE static void setXKnownPosition(const bool onoff) {
      mk_3_flag = (onoff ? mk_3_flag | flag3_x_known_position : mk_3_flag & ~flag3_x_known_position);
    }
    FORCE_INLINE static bool isXKnownPosition() { return mk_3_flag & flag3_x_known_position; }

    FORCE_INLINE static void setYKnownPosition(const bool onoff) {
      mk_3_flag = (onoff ? mk_3_flag | flag3_y_known_position : mk_3_flag & ~flag3_y_known_position);
    }
    FORCE_INLINE static bool isYKnownPosition() { return mk_3_flag & flag3_y_known_position; }

    FORCE_INLINE static void setZKnownPosition(const bool onoff) {
      mk_3_flag = (onoff ? mk_3_flag | flag3_z_known_position : mk_3_flag & ~flag3_z_known_position);
    }
    FORCE_INLINE static bool isZKnownPosition() { return mk_3_flag & flag3_z_known_position; }

    FORCE_INLINE static void setAxisKnownPosition(const AxisEnum axis, const bool onoff) {
      switch (axis) {
        case X_AXIS: setXKnownPosition(onoff); break;
        case Y_AXIS: setYKnownPosition(onoff); break;
        case Z_AXIS: setZKnownPosition(onoff); break;
      }
    }
    FORCE_INLINE static bool isAxisKnownPosition(const AxisEnum axis) {
      switch (axis) {
        case X_AXIS: return isXKnownPosition(); break;
        case Y_AXIS: return isYKnownPosition(); break;
        case Z_AXIS: return isZKnownPosition(); break;
      }
    }

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

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
 * filamentrunout.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if HAS_FILAMENT_SENSOR

//#define FILAMENT_RUNOUT_SENSOR_DEBUG

union filament_flag_t {
  uint8_t all;
  struct {
    bool  enabled       : 1;
    bool  ran_out       : 1;
    bool  host_handling : 1;
    bool  bit3          : 1;
    bool  bit4          : 1;
    bool  bit5          : 1;
    bool  bit6          : 1;
    bool  bit7          : 1;
  };
  filament_flag_t() { all = 0x00; }
};

// Struct Filament Runout data
typedef struct {
  filament_flag_t flag;
  uint8_t         logic_flag,
                  pullup_flag;
  #if FILAMENT_RUNOUT_DISTANCE_MM > 0
    float         runout_distance_mm;
  #endif
} filament_data_t;

template<class RESPONSE_T, class SENSOR_T>
class FilamentRunoutBase {

  public: /** Public Parameters */

    typedef RESPONSE_T  response_t;
    typedef SENSOR_T    sensor_t;
    static  sensor_t    sensor;
    static  response_t  response;

  public: /** Public Function */

    static inline void init() {
      sensor.init();
      reset();
    }

    static inline void reset() {
      sensor.setFilamentOut(false);
      response.reset();
    }

    // Call this method when filament is present,
    // so the response can reset its counter.
    static inline void filament_present(const uint8_t extruder) {
      response.filament_present(extruder);
    }

    #if FILAMENT_RUNOUT_DISTANCE_MM > 0
      static inline float& runout_distance() { return sensor.data.runout_distance_mm; }
      static inline void set_runout_distance(const float &mm) { sensor.data.runout_distance_mm = mm; }
    #endif

    // Handle a block completion. RunoutResponseDelayed uses this to
    // add up the length of filament moved while the filament is out.
    static inline void block_completed(const block_t* const b) {
      if (sensor.isEnabled()) {
        response.block_completed(b);
        sensor.block_completed(b);
      }
    }

    // Give the response a chance to update its counter.
    static inline void spin() {
      if (sensor.isEnabled() && !sensor.isFilamentOut() && printer.isPrinting()) {
        #if FILAMENT_RUNOUT_DISTANCE_MM > 0
          cli(); // Prevent RunoutResponseDelayed::block_completed from accumulating here
        #endif
        response.run();
        sensor.run();
        const bool ran_out = response.has_run_out();
        #if FILAMENT_RUNOUT_DISTANCE_MM > 0
          sei();
        #endif
        if (ran_out) {
          sensor.setFilamentOut(true);
          event_runout();
          planner.synchronize();
        }
      }
    }

    static inline void print_M412() {
      SERIAL_LM(CFG, "Filament runout: S<enable> H<Host control> D<Distanze (mm)>");
      SERIAL_SM(CFG, "  M412");
      SERIAL_MV(" S", sensor.isEnabled());
      SERIAL_MV(" H", sensor.isHostHandling());
      #if FILAMENT_RUNOUT_DISTANCE_MM > 0
        SERIAL_MV(" D", runout_distance());
      #endif
      SERIAL_EOL();
    }

  private: /** Private Function */

    static void event_runout() {

      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        if (advancedpause.did_pause_print) return;
      #endif

      const char tool = DIGIT(toolManager.extruder.active);
      host_action.prompt_begin(PROMPT_FILAMENT_RUNOUT, PSTR("Filament Runout T"), tool);
      host_action.prompt_show();

      const bool run_runout_script = !sensor.isHostHandling();

      if (run_runout_script
        && ( strstr(FILAMENT_RUNOUT_SCRIPT, "M600")
          || strstr(FILAMENT_RUNOUT_SCRIPT, "M125")
          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            || strstr(FILAMENT_RUNOUT_SCRIPT, "M25")
          #endif
        )
      ) {
        host_action.paused(false);
      }
      else
        host_action.pause(false);

      SERIAL_MSG(" filament_runout T");
      SERIAL_CHR(tool);
      SERIAL_EOL();

      if (run_runout_script)
        commands.inject_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
    }

};

class FilamentSensorBase {

  public: /** Public Parameters */

    static filament_data_t data;

  public: /** Public Function */

    /**
     * Initialize the filrunout pins
     */
    static void init();

    /**
     * Initialize Factory parameters
     */
    static void factory_parameters();

    /**
     * Setup Pullup
     */
    static void setup_pullup();

    /**
     * Print logical and pullup
     */
    static void report();

    // Return a bitmask of runout pin states
    static inline uint8_t read() {
      return (
        (READ(FIL_RUNOUT_0_PIN) ^ isLogic(FIL_RUNOUT_0) ? _BV(0) : 0)
        #if HAS_FIL_RUNOUT_1
          | (READ(FIL_RUNOUT_1_PIN) ^ isLogic(FIL_RUNOUT_1) ? _BV(1) : 0)
          #if HAS_FIL_RUNOUT_2
            | (READ(FIL_RUNOUT_2_PIN) ^ isLogic(FIL_RUNOUT_2) ? _BV(2) : 0)
            #if HAS_FIL_RUNOUT_3
              | (READ(FIL_RUNOUT_3_PIN) ^ isLogic(FIL_RUNOUT_3) ? _BV(3) : 0)
              #if HAS_FIL_RUNOUT_4
                | (READ(FIL_RUNOUT_4_PIN) ^ isLogic(FIL_RUNOUT_4) ? _BV(4) : 0)
                #if HAS_FIL_RUNOUT_5
                  | (READ(FIL_RUNOUT_5_PIN) ^ isLogic(FIL_RUNOUT_5) ? _BV(5) : 0)
                #endif
              #endif
            #endif
          #endif
        #endif
      );
    }

    FORCE_INLINE static void setEnabled(const bool onoff) { data.flag.enabled = onoff; }
    FORCE_INLINE static bool isEnabled() { return data.flag.enabled; }
    FORCE_INLINE static void setFilamentOut(const bool onoff) { data.flag.ran_out = onoff; }
    FORCE_INLINE static bool isFilamentOut() { return data.flag.ran_out; }
    FORCE_INLINE static void setHostHandling(const bool onoff) { data.flag.host_handling = onoff; }
    FORCE_INLINE static bool isHostHandling() { return data.flag.host_handling; }

    FORCE_INLINE static void setLogic(const FilRunoutEnum filrunout, const bool logic) {
      SET_BIT_TO(data.logic_flag, filrunout, logic);
    }
    FORCE_INLINE static bool isLogic(const FilRunoutEnum filrunout) { return TEST(data.logic_flag, filrunout); }

    FORCE_INLINE static void setPullup(const FilRunoutEnum filrunout, const bool pullup) {
      SET_BIT_TO(data.pullup_flag, filrunout, pullup);
    }
    FORCE_INLINE static bool isPullup(const FilRunoutEnum filrunout) { return TEST(data.pullup_flag, filrunout); }

  protected: /** Protected Function */

    static void filament_present(const uint8_t extruder);

};

#if ENABLED(EXTRUDER_ENCODER_CONTROL)

  /**
   * This sensor uses a magnetic encoder disc and a Hall effect
   * sensor (or a slotted disc and optical sensor). The state
   * will toggle between 0 and 1 on filament movement. It can detect
   * filament runout and stripouts or jams.
   */
  class FilamentSensorEncoder : public FilamentSensorBase {

    private: /** Private Parameters */

      static uint8_t motion_detected;

    public: /** Public Function */

      static inline void block_completed(const block_t* const b) {
        // If the sensor wheel has moved since the last call to
        // this method reset the runout counter for the active extruder.
        if (TEST(motion_detected, b->active_extruder))
          filament_present(b->active_extruder);

        // Clear motion triggers for next block
        motion_detected = 0;
      }

      static inline void run() {
        static uint8_t old_state;
        const uint8_t new_state = read(),
                      change    = old_state ^ new_state;

        old_state = new_state;

        #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
          if (change) {
            SERIAL_MSG("Motion detected:E");
            for (uint8_t e = 0; e < EXTRUDERS; e++)
              if (TEST(change, e)) { SERIAL_CHR(' '); SERIAL_CHR('0' + e); }
            SERIAL_EOL();
          }
        #endif

        motion_detected |= change;
      }

  };

#else // DISABLED(EXTRUDER_ENCODER_CONTROL)

  /**
   * This is a simple endstop switch in the path of the filament.
   * It can detect filament runout, but not stripouts or jams.
   */
  class FilamentSensorSwitch : public FilamentSensorBase {

    public: /** Public Function */

      static inline void block_completed(const block_t* const b) { UNUSED(b); }

      static inline void run() {
        const bool out = read_runout_state(toolManager.extruder.active);
        if (!out) filament_present(toolManager.extruder.active);
        #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
          static bool was_out = false;
          if (out != was_out) {
            was_out = out;
            SERIAL_MSG("Filament ");
            SERIAL_STR(out ? PSTR("OUT\n") : PSTR("IN\n"));
          }
        #endif
      }

    private: /** Private Function */

      static inline bool read_runout_state(const uint8_t extruder) {
        const uint8_t runout_states = read();
        #if !HAS_FIL_RUNOUT_1
          UNUSED(extruder);
          return runout_states;                     // A single sensor applying to all extruders
        #else
          #if ENABLED(DUAL_X_CARRIAGE)
            if (mechanics.dual_x_carriage_mode == DXC_DUPLICATION_MODE || mechanics.dual_x_carriage_mode == DXC_MIRRORED_MODE)
              return runout_states;                 // Any extruder
            else if (mechanics.extruder_duplication_enabled)
              return runout_states;                 // Any extruder
            else
          #endif
              return TEST(runout_states, extruder); // Specific extruder
        #endif
      }

  };


#endif // DISABLED(EXTRUDER_ENCODER_CONTROL)

#if FILAMENT_RUNOUT_DISTANCE_MM > 0

  // RunoutResponseDelayed triggers a runout event only if the length
  // of filament specified by FILAMENT_RUNOUT_DISTANCE_MM has been fed
  // during a runout condition.
  class RunoutResponseDelayed {
 
    private: /** Private Parameters */

      static volatile float runout_mm_countdown[EXTRUDERS];

    public: /** Public Function */

      static inline void reset() {
        LOOP_EXTRUDER() filament_present(e);
      }

      static inline void run() {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
          static short_timer_t nex_timer(millis());
          if (nex_timer.expired(1000)) {
            LOOP_EXTRUDER() {
              SERIAL_STR(e ? PSTR(", ") : PSTR("Remaining mm: "));
              SERIAL_VAL(runout_mm_countdown[e]);
            }
            SERIAL_EOL();
          }
        #endif
      }

      static inline bool has_run_out() {
        return runout_mm_countdown[toolManager.extruder.active] < 0;
      }

      static void filament_present(const uint8_t extruder);

      static inline void block_completed(const block_t* const block) {
        if (block->steps.x || block->steps.y || block->steps.z
          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            || advancedpause.did_pause_print // Allow pause purge move to re-trigger runout state
          #endif
        ) {
          // Only trigger on extrusion with XYZ movement to allow filament change and retract/recover.
          const uint8_t e = block->active_extruder;
          const int32_t steps = block->steps.e;
          runout_mm_countdown[e] -= (TEST(block->direction_bits, E_AXIS) ? -steps : steps) * extruders[e]->steps_to_mm;
        }
      }

  };

#else // !FILAMENT_RUNOUT_DISTANCE_MM

  // RunoutResponseDebounced triggers a runout event after a runout
  // condition has been detected FILAMENT_RUNOUT_THRESHOLD times in a row.

  class RunoutResponseDebounced {

    private: /** Private Parameters */

      static int8_t runout_count;

    public: /** Public Function */

      static inline void reset()                                  { runout_count = FILAMENT_RUNOUT_THRESHOLD; }
      static inline void run()                                    { if (runout_count >= 0) runout_count--; }
      static inline bool has_run_out()                            { return runout_count < 0; }
      static inline void block_completed(const block_t* const b)  { UNUSED(b); }
      static inline void filament_present(const uint8_t extruder) { runout_count = FILAMENT_RUNOUT_THRESHOLD; UNUSED(extruder); }

  };

#endif // !FILAMENT_RUNOUT_DISTANCE_MM

#if FILAMENT_RUNOUT_DISTANCE_MM > 0
  #if ENABLED(EXTRUDER_ENCODER_CONTROL)
    typedef FilamentRunoutBase<RunoutResponseDelayed, FilamentSensorEncoder>  FilamentRunout;
  #else
    typedef FilamentRunoutBase<RunoutResponseDelayed, FilamentSensorSwitch>   FilamentRunout;
  #endif
#else
  typedef FilamentRunoutBase<RunoutResponseDebounced, FilamentSensorSwitch>   FilamentRunout;
#endif

extern FilamentRunout filamentrunout;

#endif // HAS_FILAMENT_SENSOR

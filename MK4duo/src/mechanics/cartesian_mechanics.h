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
 * cartesian_mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _CARTESIAN_MECHANICS_H_
#define _CARTESIAN_MECHANICS_H_

#if IS_CARTESIAN

  class Cartesian_Mechanics {

    public: /** Constructor */

      Cartesian_Mechanics() {};

    public: /** Public Parameters */

      /**
       * Feedrate, min, max, travel
       */
      float min_feedrate_mm_s,
            max_feedrate_mm_s[XYZE_N],        // Max speeds in mm per second
            min_travel_feedrate_mm_s;

      /**
       * Step per unit
       */
      float axis_steps_per_mm[XYZE_N],
            steps_to_mm[XYZE_N];

      /**
       * Acceleration and Jerk
       */
      float     acceleration,                         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
                retract_acceleration[EXTRUDERS],      // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
                travel_acceleration,                  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
                max_jerk[XYZE_N];                     // The largest speed change requiring no acceleration
      uint32_t  max_acceleration_steps_per_s2[XYZE_N],
                max_acceleration_mm_per_s2[XYZE_N];   // Use M201 to override by software

      /**
       * Min segment time
       */
      millis_t  min_segment_time;

      /**
       * Cartesian Current Position
       *   Used to track the logical position as moves are queued.
       *   Used by 'line_to_current_position' to do a move after changing it.
       *   Used by 'sync_plan_position' to update 'planner.position'.
       */
      float current_position[XYZE];

      /**
       * Cartesian Stored Position
       *   Used to save logical position as moves are queued.
       *   Used by G60 for stored.
       *   Used by G61 for move to.
       */
      float stored_position[NUM_POSITON_SLOTS][XYZE];

      /**
       * Cartesian position
       */
      float cartesian_position[XYZ];

      /**
       * Cartesian Destination
       *   A temporary position, usually applied to 'current_position'.
       *   Set with 'gcode_get_destination' or 'set_destination_to_current'.
       *   'line_to_destination' sets 'current_position' to 'destination'.
       */
      float destination[XYZE];

      /**
       * axis_homed
       *   Flags that each linear axis was homed.
       *   XYZ on cartesian.
       *
       * axis_known_position
       *   Flags that the position is known in each linear axis. Set when homed.
       *   Cleared whenever a stepper powers off, potentially losing its position.
       */
      bool axis_homed[XYZ], axis_known_position[XYZ];

      /**
       * Workspace Offset
       */
      #if ENABLED(WORKSPACE_OFFSETS)
        // The distance that XYZ has been offset by G92. Reset by G28.
        float position_shift[XYZ] = { 0 };

        // This offset is added to the configured home position.
        // Set by M206, M428, or menu item. Saved to EEPROM.
        float home_offset[XYZ] = { 0 };

        // The above two are combined to save on computes
        float workspace_offset[XYZ] = { 0 };
      #endif

      /**
       * Feed rates are often configured with mm/m
       * but the planner and stepper like mm/s units.
       */
      float feedrate_mm_s             = MMM_TO_MMS(1500.0),
            saved_feedrate_mm_s       = MMM_TO_MMS(1500.0);
      int   feedrate_percentage       = 100,
            saved_feedrate_percentage = 100;

      /**
       * Homing feed rates are often configured with mm/m
       * but the planner and stepper like mm/s units.
       */
      const float homing_feedrate_mm_s[XYZ] = { MMM_TO_MMS(HOMING_FEEDRATE_X), MMM_TO_MMS(HOMING_FEEDRATE_Y), MMM_TO_MMS(HOMING_FEEDRATE_Z) },
                  base_max_pos[XYZ]         = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS },
                  base_min_pos[XYZ]         = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
                  base_home_pos[XYZ]        = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS },
                  max_length[XYZ]           = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH },
                  home_bump_mm[XYZ]         = { X_HOME_BUMP_MM, Y_HOME_BUMP_MM, Z_HOME_BUMP_MM };

      const signed char home_dir[XYZ]       = { X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR };

      #if ENABLED(DUAL_X_CARRIAGE)
        DualXMode dual_x_carriage_mode          = DEFAULT_DUAL_X_CARRIAGE_MODE;
        float     inactive_hotend_x_pos         = X2_MAX_POS,                   // used in mode 0 & 1
                  raised_parked_position[NUM_AXIS],                             // used in mode 1
                  duplicate_hotend_x_offset     = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
        int16_t   duplicate_hotend_temp_offset  = 0;                            // used in mode 2
        millis_t  delayed_move_time             = 0;                            // used in mode 1
        bool      active_hotend_parked          = false,                        // used in mode 1 & 2
                  hotend_duplication_enabled    = false;                        // used in mode 2
      #endif

    public: /** Public Function */

      /**
       * Initialize Cartesian parameters
       */
      void Init();

      /**
       * Get the position (mm) of an axis based on stepper position(s)
       */
      float get_axis_position_mm(AxisEnum axis);

      /**
       * Set the planner.position and individual stepper positions.
       * Used by G92, G28, G29, and other procedures.
       *
       * Multiplies by axis_steps_per_mm[] and does necessary conversion
       *
       * Clears previous speed values.
       */
      void _set_position_mm(const float &a, const float &b, const float &c, const float &e);
      void set_position_mm(const AxisEnum axis, const float &v);
      void set_position_mm(ARG_X, ARG_Y, ARG_Z, const float &e);
      void set_position_mm_kinematic(const float position[NUM_AXIS]);
      FORCE_INLINE void set_z_position_mm(const float &z) { set_position_mm(Z_AXIS, z); }
      FORCE_INLINE void set_e_position_mm(const float &e) { set_position_mm(AxisEnum(E_AXIS), e); }

      /**
       * Get the stepper positions in the cartesian_position[] array.
       *
       * The result is in the current coordinate space with
       * leveling applied. The coordinates need to be run through
       * unapply_leveling to obtain the "ideal" coordinates
       * suitable for current_position, etc.
       */
      void get_cartesian_from_steppers();

      /**
       * Set the current_position for an axis based on
       * the stepper positions, removing any leveling that
       * may have been applied.
       */
      void set_current_from_steppers_for_axis(const AxisEnum axis);

      /**
       * Set current to destination and set destination to current
       */
      FORCE_INLINE void set_current_to_destination() { COPY_ARRAY(current_position, destination); }
      FORCE_INLINE void set_destination_to_current() { COPY_ARRAY(destination, current_position); }

      /**
       * line_to_current_position
       * Move the planner to the current position from wherever it last moved
       * (or from wherever it has been told it is located).
       */
      void line_to_current_position();

      /**
       * line_to_destination
       * Move the planner to the position stored in the destination array, which is
       * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
       */
      void line_to_destination(float fr_mm_s);
      void line_to_destination();

      /**
       * Prepare a single move and get ready for the next one
       * If Mesh Bed Leveling is enabled, perform a mesh move.
       */
      void prepare_move_to_destination();

      /**
       *  Plan a move to (X, Y, Z) and set the current_position
       *  The final current_position may not be the one that was requested
       */
      void do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s = 0.0);
      void do_blocking_move_to_x(const float &lx, const float &fr_mm_s = 0.0);
      void do_blocking_move_to_z(const float &lz, const float &fr_mm_s = 0.0);
      void do_blocking_move_to_xy(const float &lx, const float &ly, const float &fr_mm_s = 0.0);

      /**
       * sync_plan_position
       *
       * Set the planner/stepper positions directly from current_position with
       * no kinematic translation. Used for homing axes and cartesian/core syncing.
       */
      void sync_plan_position();
      void sync_plan_position_e();

      void reset_acceleration_rates();
      void refresh_positioning();

      /**
       * Home Cartesian
       */
      void Home(const bool always_home_all);

      /**
       * Set an axis' current position to its home position (after homing).
       *
       * For Cartesian robots this applies one-to-one when an
       * individual axis has been homed.
       *
       * Callers must sync the planner position after calling this!
       */
      void set_axis_is_at_home(const AxisEnum axis);

      bool axis_unhomed_error(const bool x=true, const bool y=true, const bool z=true);
      bool position_is_reachable_raw_xy(const float &rx, const float &ry);
      bool position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry);
      bool position_is_reachable_by_probe_xy(const float &lx, const float &ly);
      bool position_is_reachable_xy(const float &lx, const float &ly);

      #if ENABLED(DUAL_X_CARRIAGE)
        float x_home_pos(const int extruder);
      #endif

      #if ENABLED(HYSTERESIS)
        void set_hysteresis_axis(uint8_t axis, float mm);
        void report_hysteresis();
        void insert_hysteresis_correction(const float x, const float y, const float z, const float e);
      #endif

      #if ENABLED(ZWOBBLE)
        #define STEPS_IN_ZLUT 50
        #define ZWOBBLE_MIN_Z 0.1
        // minimum distance within which two distances in mm are considered equal
        #define TOLERANCE_MM 0.01
        #define DISTANCE(_A,_B) abs((_A) - (_B))
        #define EQUAL_WITHIN_TOLERANCE(_A, _B) (DISTANCE(_A, _B) < TOLERANCE_MM)
        #define TWOPI 6.28318530718
        #define ZACTUAL_IS_SCALED(_I) (zwobble_zLut[_I][1] < 0)
        #define ZACTUAL(_I) (zwobble_zLut[_I][1] < 0 ? -zwobble_zLut[_I][1] * m_zwobble_scalingFactor : zwobble_zLut[_I][1])
        #define ZROD(_I) zwobble_zLut[_I][0]
        #define SET_ZACTUAL(_I,_V) zwobble_zLut[_I][1] = _V
        #define SET_ZROD(_I,_V) zwobble_zLut[_I][0] = _V
        void report_zwobble();
        void insert_zwobble_correction(const float targetZ);
        void set_zwobble_amplitude(float _amplitude);
        void set_zwobble_period(float _period);
        void set_zwobble_phase(float _phase);
        void set_zwobble_sample(float zRod, float zActual);
        void set_zwobble_scaledsample(float zRod, float zScaledLength);
        void set_zwobble_scalingfactor(float zActualPerScaledLength);
      #endif

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        void print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z);
        void print_xyz(const char* prefix, const char* suffix, const float xyz[]);
        #if HAS_ABL
          void print_xyz(const char* prefix, const char* suffix, const vector_3 &xyz);
        #endif
      #endif

    private: /** Private Parameters */

      #if ENABLED(HYSTERESIS)
        float   m_hysteresis_axis_shift[XYZE],
                m_hysteresis_mm[XYZE];
        long    m_hysteresis_steps[XYZE];
        uint8_t m_hysteresis_prev_direction_bits,
                m_hysteresis_bits;
      #endif

      #if ENABLED(ZWOBBLE)
        float   m_zwobble_amplitude, m_zwobble_puls, m_zwobble_phase,
                zwobble_zLut[STEPS_IN_ZLUT][2],
                zwobble_lastZ, zwobble_lastZRod,
                m_zwobble_scalingFactor;
        bool    m_zwobble_consistent,
                m_zwobble_sinusoidal;
        int     zwobble_lutSize;
      #endif

    private: /** Private Function */

      /**
       * Home an individual linear axis
       */
      void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0);

      /**
       *  Home axis
       */
      void homeaxis(const AxisEnum axis);

      /**
       * Prepare a linear move in a dual X axis setup
       */
      #if ENABLED(DUAL_X_CARRIAGE)
        bool  prepare_move_to_destination_dualx();
        int   x_home_dir(const int extruder) { return extruder ? X2_HOME_DIR : X_HOME_DIR; }
      #endif

      #if ENABLED(QUICK_HOME)
        void quick_home_xy();
      #endif

      #if ENABLED(Z_SAFE_HOMING)
        void home_z_safely();
      #endif

      #if ENABLED(DOUBLE_Z_HOMING)
        void double_home_z();
      #endif

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        void bilinear_line_to_destination(float fr_mm_s, uint16_t x_splits=0xFFFF, uint16_t y_splits=0xFFFF);
      #endif

      #if ENABLED(MESH_BED_LEVELING)
        /**
         * Prepare a mesh-leveled linear move in a Cartesian setup,
         * splitting the move where it crosses mesh borders.
         */
        void mesh_line_to_destination(float fr_mm_s, uint8_t x_splits = 0xFF, uint8_t y_splits = 0xFF);
      #endif

      #if ENABLED(HYSTERESIS)
        void set_hysteresis(float x_mm, float y_mm, float z_mm, float e_mm);
        void calc_hysteresis_steps();
        uint8_t calc_direction_bits(const long *position, const long *target);
        uint8_t calc_move_bits(const long *position, const long *target);
      #endif

      #if ENABLED(ZWOBBLE)
        void  calculateLut();
        void  initLinearLut();
        void  insertInLut(float, float);
        float findInLut(float);
        float findZRod(float);
        bool  areParametersConsistent();
      #endif

      /**
       * Some planner shorthand inline functions
       */
      float get_homing_bump_feedrate(const AxisEnum axis);

  };

  extern Cartesian_Mechanics Mechanics;

#endif // IS_CARTESIAN

#endif // _CARTESIAN_MECHANICS_H_

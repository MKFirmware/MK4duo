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
 * scara_mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _SCARA_MECHANICS_H_
#define _SCARA_MECHANICS_H_

#if IS_SCARA

  class Scara_Mechanics : public Mechanics {

    public: /** Constructor */

      Scara_Mechanics() {}

    public: /** Public Parameters */

      // Float constants for SCARA calculations
      const float L1 = SCARA_LINKAGE_1, L2 = SCARA_LINKAGE_2,
                  L1_2 = sq(float(L1)), L1_2_2 = 2.0 * L1_2,
                  L2_2 = sq(float(L2));

      float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND,
            delta[ABC];

    public: /** Public Function */

      /**
       * Initialize Scara parameters
       */
      void Init();

      /**
       * Report current position to host
       */
      void report_current_position() override;
      void report_current_position_detail() override;
      
      void get_cartesian_from_steppers() override;

      /**
       * Prepare a linear move in a SCARA setup.
       *
       * This calls planner.buffer_line several times, adding
       * small incremental moves for SCARA.
       */
      bool prepare_move_to_destination_mech_specific();

      /**
       * Home an individual linear axis
       */
      void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0) override;

      /**
       * Calculate delta, start a line, and set current_position to destination
       */
      void prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0);

      /**
       * Set an axis' current position to its home position (after homing).
       *
       * SCARA should wait until all XY homing is done before setting the XY
       * current_position to home, because neither X nor Y is at home until
       * both are at home. Z can however be homed individually.
       *
       * Callers must sync the planner position after calling this!
       */
      void set_axis_is_at_home(const AxisEnum axis);

      void set_position_mm_kinematic(const float position[NUM_AXIS]);
      void sync_plan_position_mech_specific();

      void do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s/*=0.0*/) override;
      bool position_is_reachable(const float &rx, const float &ry) override;
      bool position_is_reachable_by_probe(const float &rx, const float &ry) override;

      #if MECH(MORGAN_SCARA)
        bool move_to_cal(uint8_t delta_a, uint8_t delta_b);
        void forward_kinematics_SCARA(const float &a, const float &b);
        void inverse_kinematics_SCARA(const float logical[XYZ]);
      #endif

    private: /** Private Function */

      /**
       *  Home axis
       */
      void homeaxis(const AxisEnum axis);

  };

  extern Scara_Mechanics mechanics;

#endif // IS_SCARA

#endif /* _SCARA_MECHANICS_H_ */

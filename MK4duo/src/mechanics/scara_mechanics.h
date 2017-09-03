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

      Scara_Mechanics() {};

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
       * Prepare a linear move in a SCARA setup.
       *
       * This calls planner.buffer_line several times, adding
       * small incremental moves for SCARA.
       */
      bool prepare_move_to_destination_mech_specific() override;

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

      void do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s/*=0.0*/) override;
      bool position_is_reachable_raw_xy(const float &rx, const float &ry) override;
      bool position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry) override;

      #if MECH(MORGAN_SCARA)
        bool move_to_cal(uint8_t delta_a, uint8_t delta_b);
        void forward_kinematics_SCARA(const float &a, const float &b);
      #endif

    private: /** Private Parameters */

    private: /** Private Function */

  };

  extern Scara_Mechanics mechanics;

#endif // IS_SCARA

#endif /* _SCARA_MECHANICS_H_ */

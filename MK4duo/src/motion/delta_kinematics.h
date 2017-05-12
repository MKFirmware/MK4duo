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

#ifndef _DELTA_PARAMETERS_H_
#define _DELTA_PARAMETERS_H_

#if MECH(DELTA)

  class DeltaKinematics {

    public:

      /**
       * Constructor
       */
      DeltaKinematics() {};

      /**
       * Public Delta parameters
       */
      float diagonal_rod          = DELTA_DIAGONAL_ROD,
            delta_radius          = DEFAULT_DELTA_RADIUS,
            segments_per_second   = DELTA_SEGMENTS_PER_SECOND,
            print_radius          = DELTA_PRINTABLE_RADIUS,
            probe_radius          = DELTA_PROBEABLE_RADIUS,
            delta_height          = DELTA_HEIGHT,
            clip_start_height     = DELTA_HEIGHT,
            diagonal_rod_adj[ABC] = { TOWER_A_DIAGROD_ADJ, TOWER_B_DIAGROD_ADJ, TOWER_C_DIAGROD_ADJ },
            endstop_adj[ABC]      = { TOWER_A_ENDSTOP_ADJ, TOWER_B_ENDSTOP_ADJ, TOWER_C_ENDSTOP_ADJ },
            tower_radius_adj[ABC] = { TOWER_A_RADIUS_ADJ, TOWER_B_RADIUS_ADJ, TOWER_C_RADIUS_ADJ },
            tower_pos_adj[ABC]    = { TOWER_A_POSITION_ADJ, TOWER_B_POSITION_ADJ, TOWER_C_POSITION_ADJ },
            max_length[ABC]       = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH };

      /**
       * Initialize Delta parameters
       */
      void Init();

      #if ENABLED(DELTA_AUTO_CALIBRATION_1)
        float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);
        void Adjust(const uint8_t numFactors, const float v[]);
        void Convert_endstop_adj();
      #endif

      void InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[ABC]);
      void InverseTransform(const float point[ABC], float cartesian[ABC]) { InverseTransform(point[A_AXIS], point[B_AXIS], point[C_AXIS], cartesian); }
      void Transform(const float logical[XYZ]);
      void Recalc();
      void Set_clip_start_height();
      bool IsReachable(const float dx, const float dy);

    private:

      #if ENABLED(DELTA_AUTO_CALIBRATION_1)
        void NormaliseEndstopAdjustments();
      #endif

      #if ENABLED(DELTA_FAST_SQRT) && DISABLED(MATH_USE_HAL)
        float Q_rsqrt(float number);
      #endif

      // Derived values
      float delta_diagonal_rod_2[ABC],  // Diagonal rod 2
            towerX[ABC],                // The X coordinate of each tower
            towerY[ABC],                // The Y coordinate of each tower
            homed_Height,
            printRadiusSquared,
            Xbc, Xca, Xab, Ybc, Yca, Yab,
            coreFa, coreFb, coreFc,
            Q, Q2, D2;

  };

  extern DeltaKinematics deltaParams;

#endif

#endif // _DELTA_PARAMETERS_H_

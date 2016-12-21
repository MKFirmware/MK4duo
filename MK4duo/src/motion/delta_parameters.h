/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

  class DeltaParameters {

    public:

      DeltaParameters() { Init(); }

      // Core parameters
      float diagonal_rod,             // Original Diagonl Rod lenght
            diagonal_rod_adj[ABC],    // Diagonal Rod adjustments
            radius,                   // Original Delta radius
            endstop_adj[ABC],         // Endstop adjustments
            tower_adj[6],             // Tower adjustments
            segments_per_second       = DELTA_SEGMENTS_PER_SECOND,
            print_Radius              = DELTA_PRINTABLE_RADIUS,
            probe_Radius              = DELTA_PROBEABLE_RADIUS,
            base_min_pos[ABC]         = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
            base_max_pos[ABC]         = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS },
            base_home_pos[ABC]        = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS },
            max_length[ABC]           = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH },
            clip_start_height         = Z_MAX_POS;

      // Function
      void Init();
      float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);
      void Adjust(const uint8_t numFactors, const float v[]);
      void forward_kinematics_DELTA(const float Ha, const float Hb, const float Hc, float machinePos[ABC]);
      void forward_kinematics_DELTA(const float point[ABC], float machinePos[ABC]) { forward_kinematics_DELTA(point[A_AXIS], point[B_AXIS], point[C_AXIS], machinePos); }
      void inverse_kinematics_DELTA(const float logical[XYZ]);
      void Recalc_delta_constants();

      void Set_clip_start_height();

    private:

      const float degreesToRadians = M_PI / 180.0;
      const float radiansToDegrees = 180.0 / M_PI;

      #if ENABLED(DELTA_FAST_SQRT)
        float Q_rsqrt(float number);
      #endif

      // Derived values
      float delta_diagonal_rod_2[ABC]; // Diagonal rod 2
      float towerX[ABC];      // The X coordinate of each tower
      float towerY[ABC];      // The Y coordinate of each tower
      float Xbc, Xca, Xab, Ybc, Yca, Yab;
      float coreFa, coreFb, coreFc;
      float Q, Q2, D2;

  };

  extern DeltaParameters deltaParams;

#endif
#endif // _DELTA_PARAMETERS_H_

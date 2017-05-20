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

#include "../../base.h"

#if MECH(DELTA)

  DeltaKinematics deltaParams;

  void DeltaKinematics::Init() {
    delta_radius              = DEFAULT_DELTA_RADIUS;
    diagonal_rod              = DELTA_DIAGONAL_ROD;
    segments_per_second       = DELTA_SEGMENTS_PER_SECOND;
    print_radius              = DELTA_PRINTABLE_RADIUS;
    probe_radius              = DELTA_PRINTABLE_RADIUS - max(abs(X_PROBE_OFFSET_FROM_NOZZLE), abs(Y_PROBE_OFFSET_FROM_NOZZLE));
    delta_height              = DELTA_HEIGHT;
    endstop_adj[A_AXIS]       = TOWER_A_ENDSTOP_ADJ;
    endstop_adj[B_AXIS]       = TOWER_B_ENDSTOP_ADJ;
    endstop_adj[C_AXIS]       = TOWER_C_ENDSTOP_ADJ;
    tower_radius_adj[A_AXIS]  = TOWER_A_RADIUS_ADJ;
    tower_radius_adj[B_AXIS]  = TOWER_B_RADIUS_ADJ;
    tower_radius_adj[C_AXIS]  = TOWER_C_RADIUS_ADJ;
    tower_pos_adj[A_AXIS]     = TOWER_A_POSITION_ADJ;
    tower_pos_adj[B_AXIS]     = TOWER_B_POSITION_ADJ;
    tower_pos_adj[C_AXIS]     = TOWER_C_POSITION_ADJ;
    diagonal_rod_adj[A_AXIS]  = TOWER_A_DIAGROD_ADJ;
    diagonal_rod_adj[B_AXIS]  = TOWER_B_DIAGROD_ADJ;
    diagonal_rod_adj[C_AXIS]  = TOWER_C_DIAGROD_ADJ;
    clip_start_height         = Z_MAX_POS;
  }

  #if ENABLED(DELTA_AUTO_CALIBRATION_1)
  
    // Compute the derivative of height with respect to a parameter at the specified motor endpoints.
    // 'deriv' indicates the parameter as follows:
    // 0, 1, 2 = X, Y, Z tower endstop adjustments
    // 3 = delta radius
    // 4 = X tower correction
    // 5 = Y tower correction
    // 6 = diagonal_rod rod length
    // 7, 8 = X tilt, Y tilt. We scale these by the printable radius to get sensible values in the range -1..1
    float DeltaKinematics::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) {
      const float perturb = 0.2;			// perturbation amount in mm or degrees
      DeltaKinematics hiParams(*this), loParams(*this);

      switch(deriv) {
        case 0:
        case 1:
        case 2:
          // Endstop corrections
          break;

        case 3:
          hiParams.delta_radius += perturb;
          loParams.delta_radius -= perturb;
          break;

        case 4:
          hiParams.tower_radius_adj[A_AXIS] += perturb;
          loParams.tower_radius_adj[A_AXIS] -= perturb;
          break;

        case 5:
          hiParams.tower_radius_adj[B_AXIS] += perturb;
          loParams.tower_radius_adj[B_AXIS] -= perturb;
          break;

        case 6:
          hiParams.diagonal_rod += perturb;
          loParams.diagonal_rod -= perturb;
          break;
      }

      hiParams.Recalc();
      loParams.Recalc();

      float newPos[ABC];

      hiParams.InverseTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc, newPos);
      const float zHi = newPos[C_AXIS];

      loParams.InverseTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc, newPos);
      const float zLo = newPos[C_AXIS];

      return ((float)zHi - (float)zLo) / (2 * perturb);
    }

    // Perform 3, 4, 6, 7 - factor adjustment.
    // The input vector contains the following parameters in this order:
    //  X, Y and Z endstop adjustments
    //  Delta radius
    //  X tower position adjustment
    //  Y tower position adjustment
    //  Diagonal rod length adjustment
    void DeltaKinematics::Adjust(const uint8_t numFactors, const float v[]) {

      const float oldHeightA = homed_Height + endstop_adj[A_AXIS];

      // Update endstop adjustments
      endstop_adj[A_AXIS] += v[0];
      endstop_adj[B_AXIS] += v[1];
      endstop_adj[C_AXIS] += v[2];
      NormaliseEndstopAdjustments();

      if (numFactors >= 4) {
        delta_radius += v[3];

        if (numFactors >= 6) {
          tower_radius_adj[A_AXIS] += v[4];
          tower_radius_adj[B_AXIS] += v[5];

          if (numFactors == 7) diagonal_rod += v[6];

        }
      }

      Recalc();
      const float heightError = homed_Height + endstop_adj[A_AXIS] - oldHeightA - v[0];
      delta_height -= heightError;
      homed_Height -= heightError;

    }

    // Convert endstop_adj
    void DeltaKinematics::Convert_endstop_adj() {
      LOOP_XYZ(i) endstop_adj[i] *= -1;
    }

    // Normalize Endstop
    void DeltaKinematics::NormaliseEndstopAdjustments() {
      const float min_endstop = MIN3(endstop_adj[A_AXIS], endstop_adj[B_AXIS], endstop_adj[C_AXIS]);
      LOOP_XYZ(i) endstop_adj[i] -= min_endstop;
      delta_height += min_endstop;
      homed_Height += min_endstop;
    }

  #endif // DELTA_AUTO_CALIBRATION_1

  /**
   * Delta InverseTransform
   *
   * See the Wikipedia article "Trilateration"
   * https://en.wikipedia.org/wiki/Trilateration
   *
   * Establish a new coordinate system in the plane of the
   * three carriage points. This system has its origin at
   * tower1, with tower2 on the X axis. Tower3 is in the X-Y
   * plane with a Z component of zero.
   * We will define unit vectors in this coordinate system
   * in our original coordinate system. Then when we calculate
   * the Xnew, Ynew and Znew values, we can translate back into
   * the original system by moving along those unit vectors
   * by the corresponding values.
   *
   * Variable names matched to Mk4duo, c-version, and avoid the
   * use of any vector library.
   *
   * by Andreas Hardtung 2016-06-07
   * based on a Java function from "Delta Robot Kinematics V3"
   * by Steve Graves
   *
   * The result is stored in the cartesian[] array.
   */
  void DeltaKinematics::InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[ABC]) {

    const float Fa = coreFa + sq(Ha);
    const float Fb = coreFb + sq(Hb);
    const float Fc = coreFc + sq(Hc);

    // Setup PQRSU such that x = -(S - uz)/P, y = (P - Rz)/Q
    const float P = (Xbc * Fa) + (Xca * Fb) + (Xab * Fc);
    const float S = (Ybc * Fa) + (Yca * Fb) + (Yab * Fc);

    const float R = 2 * ((Xbc * Ha) + (Xca * Hb) + (Xab * Hc));
    const float U = 2 * ((Ybc * Ha) + (Yca * Hb) + (Yab * Hc));

    const float R2 = sq(R), U2 = sq(U);

    const float A = U2 + R2 + Q2;
    const float minusHalfB = S * U + P * R + Ha * Q2 + towerX[A_AXIS] * U * Q - towerY[A_AXIS] * R * Q;
    const float C = sq(S + towerX[A_AXIS] * Q) + sq(P - towerY[A_AXIS] * Q) + (sq(Ha) - D2) * Q2;

    const float z = (minusHalfB - sqrtf(sq(minusHalfB) - A * C)) / A;

    cartesian[X_AXIS] = (U * z - S) / Q;
    cartesian[Y_AXIS] = (P - R * z) / Q;
    cartesian[Z_AXIS] = z;
  }

  void DeltaKinematics::Recalc() {

    LOOP_XY(i) {
      soft_endstop_min[i] = -print_radius;
      soft_endstop_max[i] = print_radius;
    }
    soft_endstop_max[Z_AXIS]  = delta_height;
    probe_radius              = print_radius - max(abs(X_PROBE_OFFSET_FROM_NOZZLE), abs(Y_PROBE_OFFSET_FROM_NOZZLE));

    delta_diagonal_rod_2[A_AXIS] = sq(diagonal_rod + diagonal_rod_adj[A_AXIS]);
    delta_diagonal_rod_2[B_AXIS] = sq(diagonal_rod + diagonal_rod_adj[B_AXIS]);
    delta_diagonal_rod_2[C_AXIS] = sq(diagonal_rod + diagonal_rod_adj[C_AXIS]);

    // Effective X/Y positions of the three vertical towers.
    towerX[A_AXIS] = -((delta_radius + tower_pos_adj[A_AXIS]) * cos(RADIANS(30 + tower_radius_adj[A_AXIS]))); // front left tower
    towerY[A_AXIS] = -((delta_radius + tower_pos_adj[A_AXIS]) * sin(RADIANS(30 + tower_radius_adj[A_AXIS]))); 
    towerX[B_AXIS] = +((delta_radius + tower_pos_adj[B_AXIS]) * cos(RADIANS(30 - tower_radius_adj[B_AXIS]))); // front right tower
    towerY[B_AXIS] = -((delta_radius + tower_pos_adj[B_AXIS]) * sin(RADIANS(30 - tower_radius_adj[B_AXIS]))); 
    towerX[C_AXIS] = -((delta_radius + tower_pos_adj[C_AXIS]) * sin(RADIANS(     tower_radius_adj[C_AXIS]))); // back middle tower
    towerY[C_AXIS] = +((delta_radius + tower_pos_adj[C_AXIS]) * cos(RADIANS(     tower_radius_adj[C_AXIS]))); 

    Xbc = towerX[C_AXIS] - towerX[B_AXIS];
    Xca = towerX[A_AXIS] - towerX[C_AXIS];
    Xab = towerX[B_AXIS] - towerX[A_AXIS];
    Ybc = towerY[C_AXIS] - towerY[B_AXIS];
    Yca = towerY[A_AXIS] - towerY[C_AXIS];
    Yab = towerY[B_AXIS] - towerY[A_AXIS];
    coreFa = HYPOT2(towerX[A_AXIS], towerY[A_AXIS]);
    coreFb = HYPOT2(towerX[B_AXIS], towerY[B_AXIS]);
    coreFc = HYPOT2(towerX[C_AXIS], towerY[C_AXIS]);
    Q = 2 * (Xca * Yab - Xab * Yca);
    Q2 = sq(Q);
    D2 = sq(diagonal_rod);

    const float tempHeight = diagonal_rod;		// any sensible height will do here, probably even zero
    float cartesian[ABC];
    InverseTransform(tempHeight, tempHeight, tempHeight, cartesian);
    homed_Height = delta_height + tempHeight - cartesian[Z_AXIS];
    printRadiusSquared = sq(print_radius);

    Set_clip_start_height();

  }

  #if ENABLED(DELTA_FAST_SQRT) && DISABLED(MATH_USE_HAL)
    /**
     * Fast inverse SQRT from Quake III Arena
     * See: https://en.wikipedia.org/wiki/Fast_inverse_square_root
     */
    float DeltaKinematics::Q_rsqrt(float number) {
      long i;
      float x2, y;
      const float threehalfs = 1.5f;
      x2 = number * 0.5f;
      y  = number;
      i  = * ( long * ) &y;                         // evil floating point bit level hacking
      i  = 0x5F3759DF - ( i >> 1 );
      y  = * ( float * ) &i;
      y  = y * ( threehalfs - ( x2 * y * y ) );     // 1st iteration
      // y  = y * ( threehalfs - ( x2 * y * y ) );  // 2nd iteration, this can be removed
      return y;
    }

    #define _SQRT(n) (1.0f / Q_rsqrt(n))

  #else

    #define _SQRT(n) SQRT(n)

  #endif

  /**
   * Delta Inverse Kinematics
   *
   * Calculate the tower positions for a given logical
   * position, storing the result in the delta[] array.
   *
   * This is an expensive calculation, requiring 3 square
   * roots per segmented linear move, and strains the limits
   * of a Mega2560 with a Graphical Display.
   */
  void DeltaKinematics::Transform(const float logical[XYZ]) {

    delta[A_AXIS] = logical[Z_AXIS] + _SQRT(delta_diagonal_rod_2[A_AXIS] - sq(logical[A_AXIS] - towerX[A_AXIS]) - sq(logical[B_AXIS] - towerY[A_AXIS]));
    delta[B_AXIS] = logical[Z_AXIS] + _SQRT(delta_diagonal_rod_2[B_AXIS] - sq(logical[A_AXIS] - towerX[B_AXIS]) - sq(logical[B_AXIS] - towerY[B_AXIS]));
    delta[C_AXIS] = logical[Z_AXIS] + _SQRT(delta_diagonal_rod_2[C_AXIS] - sq(logical[A_AXIS] - towerX[C_AXIS]) - sq(logical[B_AXIS] - towerY[C_AXIS]));

    /*
    SERIAL_MV("cartesian X:", logical[X_AXIS]);
    SERIAL_MV(" Y:", logical[Y_AXIS]);
    SERIAL_EMV(" Z:", logical[Z_AXIS]);
    SERIAL_MV("delta A:", delta[A_AXIS]);
    SERIAL_MV(" B:", delta[B_AXIS]);
    SERIAL_EMV(" C:", delta[C_AXIS]);
    */
  }

  void DeltaKinematics::Set_clip_start_height() {
    float cartesian[XYZ] = { 0, 0, 0 };
    Transform(cartesian);
    float distance = delta[A_AXIS];
    cartesian[Y_AXIS] = print_radius;
    Transform(cartesian);
    clip_start_height = soft_endstop_max[Z_AXIS] - abs(distance - delta[A_AXIS]);
  }

  bool DeltaKinematics::IsReachable(const float dx, const float dy) {
    return HYPOT2(dx, dy) <= printRadiusSquared;
  }

#endif

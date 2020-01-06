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
 * Incremental Least Squares Best Fit By Roxy and Ed Williams
 *
 * This algorithm is high speed and has a very small code footprint.
 * Its results are identical to both the Iterative Least-Squares published
 * earlier by Roxy and the QR_SOLVE solution. If used in place of QR_SOLVE
 * it saves roughly 10K of program memory.   And even better...  the data
 * fed into the algorithm does not need to all be present at the same time.
 * A point can be probed and its values fed into the algorithm and then discarded.
 *
 */

#if ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(Z_STEPPER_ALIGN_KNOWN_STEPPER_POSITIONS)

struct linear_fit_data {
  float xbar, ybar, zbar,
        x2bar, y2bar, z2bar,
        xybar, xzbar, yzbar,
        max_absx, max_absy,
        A, B, D, N;
};

void inline incremental_LSF_reset(struct linear_fit_data *lsf) {
  memset(lsf, 0, sizeof(linear_fit_data));
}

void inline incremental_WLSF(struct linear_fit_data *lsf, const float &x, const float &y, const float &z, const float &w) {
  // weight each accumulator by factor w, including the "number" of samples
  // (analagous to calling inc_LSF twice with same values to weight it by 2X)
  const float wx = w * x, wy = w * y, wz = w * z;
  lsf->xbar  += wx;
  lsf->ybar  += wy;
  lsf->zbar  += wz;
  lsf->x2bar += wx * x;
  lsf->y2bar += wy * y;
  lsf->z2bar += wz * z;
  lsf->xybar += wx * y;
  lsf->xzbar += wx * z;
  lsf->yzbar += wy * z;
  lsf->N     += w;
  lsf->max_absx = MAX(ABS(wx), lsf->max_absx);
  lsf->max_absy = MAX(ABS(wy), lsf->max_absy);
}
inline void incremental_WLSF(struct linear_fit_data *lsf, const xy_pos_t &pos, const float &z, const float &w) {
  incremental_WLSF(lsf, pos.x, pos.y, z, w);
}

void inline incremental_LSF(struct linear_fit_data *lsf, const float &x, const float &y, const float &z) {
  lsf->xbar += x;
  lsf->ybar += y;
  lsf->zbar += z;
  lsf->x2bar += sq(x);
  lsf->y2bar += sq(y);
  lsf->z2bar += sq(z);
  lsf->xybar += x * y;
  lsf->xzbar += x * z;
  lsf->yzbar += y * z;
  lsf->max_absx = MAX(ABS(x), lsf->max_absx);
  lsf->max_absy = MAX(ABS(y), lsf->max_absy);
  lsf->N += 1.0;
}
inline void incremental_LSF(struct linear_fit_data *lsf, const xy_pos_t &pos, const float &z) {
  incremental_LSF(lsf, pos.x, pos.y, z);
}

int finish_incremental_LSF(struct linear_fit_data *);

#endif // ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(Z_STEPPER_ALIGN_KNOWN_STEPPER_POSITIONS)

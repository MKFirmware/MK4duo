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

/**
 * gcode.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(DELTA_AUTO_CALIBRATION_1)

#define CODE_G33

/**
 * Probe a point
 */
static float calibration_probe(const xyz_pos_t &pos, const bool stow=false) {
  return probe.check_at_point(pos.x, pos.y, stow ? PROBE_PT_STOW : PROBE_PT_RAISE, 4);
}

static void ac_home() {
  endstops.setEnabled(true);
  mechanics.home(false);
  endstops.setNotHoming();
}

static void ac_setup() {

  if (toolManager.extruder.total > 1) toolManager.change(0, true);

  planner.synchronize();
  mechanics.setup_for_endstop_or_probe_move();

  #if HAS_LEVELING
    bedlevel.reset(); // After full calibration bed-level data is no longer valid
  #endif
}

static void ac_cleanup() {
  #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
    mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
  #endif

  STOW_PROBE();
  mechanics.clean_up_after_endstop_or_probe_move();

  if (toolManager.extruder.total > 1) toolManager.change(toolManager.extruder.previous, true);

}

// Homed height
float homed_height;
static void calc_homed_height() {
  const float tempHeight = mechanics.data.diagonal_rod;		// any sensible height will do here, probably even zero
  abc_pos_t cartesian;
  mechanics.InverseTransform(tempHeight, tempHeight, tempHeight, cartesian);
  homed_height = mechanics.data.height + tempHeight - cartesian.z;
}

// Convert data.endstop_adj
static void Convert_endstop_adj() {
  mechanics.data.endstop_adj *= -1;
}

// Normalize Endstop
static void NormaliseEndstopAdjustments() {
  const float min_endstop = MIN(mechanics.data.endstop_adj.a, mechanics.data.endstop_adj.b, mechanics.data.endstop_adj.c);
  mechanics.data.endstop_adj -= min_endstop;
  mechanics.data.height += min_endstop;
  homed_height += min_endstop;
}

// Perform 3, 4, 6, 7 - factor adjustment.
// The input vector contains the following parameters in this order:
//  X, Y and Z endstop adjustments
//  Delta radius
//  X tower position adjustment
//  Y tower position adjustment
//  Diagonal rod length adjustment
static void Adjust(const uint8_t numFactors, const float v[]) {

  const float oldHeightA = homed_height + mechanics.data.endstop_adj.a;

  // Update endstop adjustments
  mechanics.data.endstop_adj.a += v[0];
  mechanics.data.endstop_adj.b += v[1];
  mechanics.data.endstop_adj.c += v[2];
  NormaliseEndstopAdjustments();

  if (numFactors >= 4) {
    mechanics.data.radius += v[3];

    if (numFactors >= 6) {
      mechanics.data.tower_angle_adj.a += v[4];
      mechanics.data.tower_angle_adj.b += v[5];

      if (numFactors == 7) mechanics.data.diagonal_rod += v[6];

    }
  }

  mechanics.recalc_delta_settings();
  calc_homed_height();
  const float heightError = homed_height + mechanics.data.endstop_adj.a - oldHeightA - v[0];
  mechanics.data.height -= heightError;
  homed_height -= heightError;

}

// Compute the derivative of height with respect to a parameter at the specified motor endpoints.
// 'deriv' indicates the parameter as follows:
// 0, 1, 2 = X, Y, Z tower endstop adjustments
// 3 = delta radius
// 4 = X tower correction
// 5 = Y tower correction
// 6 = data.diagonal_rod rod length
// 7, 8 = X tilt, Y tilt. We scale these by the printable radius to get sensible values in the range -1..1
static float compute_derivative(const uint16_t deriv, const abc_pos_t &hpos) {
  constexpr float perturb = 0.2;      // perturbation amount in mm or degrees
  float zHi = 0.0f,
        zLo = 0.0f;
  abc_float_t newPos  = { 0.0f, 0.0f, 0.0f };

  switch (deriv) {
    case 0:
    case 1:
    case 2:
      // Endstop corrections
      mechanics.InverseTransform((deriv == 0) ? hpos.a + perturb : hpos.a, (deriv == 1) ? hpos.b + perturb : hpos.b, (deriv == 2) ? hpos.c + perturb : hpos.c, newPos);
      zHi = newPos.c;
      mechanics.InverseTransform((deriv == 0) ? hpos.a - perturb : hpos.a, (deriv == 1) ? hpos.b - perturb : hpos.b, (deriv == 2) ? hpos.c - perturb : hpos.c, newPos);
      zLo = newPos.c;
      break;

    case 3: {
      const float old_delta_radius = mechanics.data.radius;

      // Calc High parameters
      mechanics.data.radius += perturb;
      mechanics.recalc_delta_settings();
      mechanics.InverseTransform(hpos, newPos);
      zHi = newPos.c;

      // Reset Delta Radius
      mechanics.data.radius = old_delta_radius;

      // Calc Low parameters
      mechanics.data.radius -= perturb;
      mechanics.recalc_delta_settings();
      mechanics.InverseTransform(hpos, newPos);
      zLo = newPos.c;

      // Reset Delta Radius
      mechanics.data.radius = old_delta_radius;
      break;
    }

    case 4: {
      const float old_delta_tower_angle_adj = mechanics.data.tower_angle_adj.a;

      // Calc High parameters
      mechanics.data.tower_angle_adj.a += perturb;
      mechanics.recalc_delta_settings();
      mechanics.InverseTransform(hpos, newPos);
      zHi = newPos.c;

      // Reset Delta tower Alpha angle adj 
      mechanics.data.tower_angle_adj.a = old_delta_tower_angle_adj;

      // Calc Low parameters
      mechanics.data.tower_angle_adj.a -= perturb;
      mechanics.recalc_delta_settings();
      mechanics.InverseTransform(hpos, newPos);
      zLo = newPos.c;

      // Reset Delta tower Alpha angle adj 
      mechanics.data.tower_angle_adj.a = old_delta_tower_angle_adj;
      break;
    }

    case 5: {
      const float old_delta_tower_angle_adj = mechanics.data.tower_angle_adj.b;

      // Calc High parameters
      mechanics.data.tower_angle_adj.b += perturb;
      mechanics.recalc_delta_settings();
      mechanics.InverseTransform(hpos, newPos);
      zHi = newPos.c;

      // Reset Delta tower Beta angle adj 
      mechanics.data.tower_angle_adj.b = old_delta_tower_angle_adj;

      // Calc Low parameters
      mechanics.data.tower_angle_adj.b -= perturb;
      mechanics.recalc_delta_settings();
      mechanics.InverseTransform(hpos, newPos);
      zLo = newPos.c;

      // Reset Delta tower Beta angle adj 
      mechanics.data.tower_angle_adj.b = old_delta_tower_angle_adj;
      break;
    }

    case 6: {
      const float old_delta_diagonal_rod = mechanics.data.diagonal_rod;

      // Calc High parameters
      mechanics.data.diagonal_rod += perturb;
      mechanics.recalc_delta_settings();
      mechanics.InverseTransform(hpos, newPos);
      zHi = newPos.c;

      // Reset Delta Diagonal Rod
      mechanics.data.diagonal_rod = old_delta_diagonal_rod;

      // Calc Low parameters
      mechanics.data.diagonal_rod -= perturb;
      mechanics.recalc_delta_settings();
      mechanics.InverseTransform(hpos, newPos);
      zLo = newPos.c;

      // Reset Delta Diagonal Rod
      mechanics.data.diagonal_rod = old_delta_diagonal_rod;
      break;
    }
  }

  mechanics.recalc_delta_settings();

  return (zHi - zLo) / (2.0f * perturb);
}

/**
 * Delta AutoCalibration Algorithm of Minor Squares based on DC42 RepRapFirmware 7 points
 * Usage:
 *    G33 <Fn> <Pn> <Q>
 *      F = Num Factors 3 or 4 or 6 or 7
 *        The input vector contains the following parameters in this order:
 *          X, Y and Z endstop adjustments
 *          Delta radius
 *          X tower position adjustment and Y tower position adjustment
 *          Diagonal rod length adjustment
 *      P = Num probe points 7 or 10
 */
inline void gcode_G33() {

  constexpr uint8_t MaxCalibrationPoints  = 10,
                NperifericalPoints    = 6,
                NinternalPoints       = 3,
                MaxnumFactors         = 7;

  uint8_t iteration = 0;

  float   initialSumOfSquares,
          expectedRmsError;

  xyz_pos_t BedProbePoints[MaxCalibrationPoints];

  char    rply[50];

  const uint8_t numFactors = parser.intval('F', DELTA_AUTO_CALIBRATION_1_DEFAULT_FACTOR);
  if (!WITHIN(numFactors, 3, 7)) {
    SERIAL_EM("?(F)actors is implausible (3 to 7).");
    return;
  }

  uint8_t probe_points  = parser.intval('P', DELTA_AUTO_CALIBRATION_1_DEFAULT_POINTS);
  if (probe_points <= 7) probe_points = 7;
  else probe_points = 10;

  const bool g33_debug = parser.boolval('D');

  SERIAL_MV("Starting Auto Calibration ", probe_points);
  SERIAL_MV(" points and ", numFactors);
  SERIAL_MSG(" Factors");
  if (g33_debug) SERIAL_MSG(" Debug on");
  SERIAL_EOL();
  LCD_MESSAGEPGM(MSG_DELTA_AUTO_CALIBRATE);

  ac_setup();

  if (!mechanics.isHomedAll()) ac_home();

  DEPLOY_PROBE();

  calc_homed_height();

  for (uint8_t probe_index = 0; probe_index < NperifericalPoints; probe_index++) {
    BedProbePoints[probe_index].x = mechanics.data.probe_radius * SIN((2 * M_PI * probe_index) / float(NperifericalPoints));
    BedProbePoints[probe_index].y = mechanics.data.probe_radius * COS((2 * M_PI * probe_index) / float(NperifericalPoints));
    BedProbePoints[probe_index].z = calibration_probe(BedProbePoints[probe_index]);
    if (isnan(BedProbePoints[probe_index].z)) return ac_cleanup();
  }

  if (probe_points == 10) {
    for (uint8_t index = 0; index < NinternalPoints; index++) {
      const uint8_t probe_index = index + NperifericalPoints;
      BedProbePoints[probe_index].x = (mechanics.data.probe_radius / 2) * SIN((2 * M_PI * index) / float(NinternalPoints));
      BedProbePoints[probe_index].y = (mechanics.data.probe_radius / 2) * COS((2 * M_PI * index) / float(NinternalPoints));
      BedProbePoints[probe_index].z = calibration_probe(BedProbePoints[probe_index]);
      if (isnan(BedProbePoints[probe_index].z)) return ac_cleanup();
    }
  }

  BedProbePoints[probe_points - 1].x = 0.0f;
  BedProbePoints[probe_points - 1].y = 0.0f;
  BedProbePoints[probe_points - 1].z = calibration_probe(BedProbePoints[probe_points - 1], true);
  if (isnan(BedProbePoints[probe_points - 1].z)) return ac_cleanup();

  // convert data.endstop_adj;
  Convert_endstop_adj();

  abc_float_t probeMotorPositions[MaxCalibrationPoints];
  float corrections[MaxCalibrationPoints];

  initialSumOfSquares = 0.0;

  // Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
  for (uint8_t i = 0; i < probe_points; ++i) {
    corrections[i] = 0.0;
    abc_float_t machinePos = { BedProbePoints[i].x, BedProbePoints[i].y, 0.0f };
    mechanics.Transform(machinePos);
    probeMotorPositions[i] = mechanics.delta;
    initialSumOfSquares += sq(BedProbePoints[i].z);
  }

  // Do 1 or more Newton-Raphson iterations
  do {

    // Build a Nx9 matrix of derivatives
    FixedMatrix<float, MaxCalibrationPoints, MaxnumFactors> derivativeMatrix;

    for (uint8_t i = 0; i < probe_points; i++) {
      for (uint8_t j = 0; j < numFactors; j++) {
        derivativeMatrix(i, j) =
          compute_derivative(j, probeMotorPositions[i]);
      }
    }

    // Debug Derivative matrix
    if (g33_debug) {
      SERIAL_EM("Derivative matrix");
      for (uint8_t i = 0; i < probe_points; i++) {
        for (uint8_t j = 0; j < numFactors; j++) {
          sprintf_P(rply, PSTR("%7.4f%c"), (double)derivativeMatrix(i, j), (j == numFactors - 1) ? '\n' : ' ');
          SERIAL_STR(rply);
        }
      }
    }

    // Now build the normal equations for least squares fitting
    FixedMatrix<float, MaxnumFactors, MaxnumFactors + 1> normalMatrix;
    for (uint8_t i = 0; i < numFactors; i++) {
      for (uint8_t j = 0; j < numFactors; j++) {
        float temp = derivativeMatrix(0, i) * derivativeMatrix(0, j);
        for (uint8_t k = 1; k < probe_points; k++) {
          temp += derivativeMatrix(k, i) * derivativeMatrix(k, j);
        }
        normalMatrix(i, j) = temp;
      }
      float temp = -1 * derivativeMatrix(0, i) * (BedProbePoints[0].z + corrections[0]);
      for (uint8_t k = 1; k < probe_points; k++) {
        temp += -1 * derivativeMatrix(k, i) * (BedProbePoints[k].z + corrections[k]);
      }
      normalMatrix(i, numFactors) = temp;
    }

    // Debug Normal matrix
    if (g33_debug) {
      SERIAL_EM("Normal matrix");
      for (uint8_t i = 0; i < numFactors; i++) {
        for (uint8_t j = 0; j < numFactors + 1; j++) {
          sprintf_P(rply, PSTR("%7.4f%c"), (double)normalMatrix(i, j), (j == numFactors) ? '\n' : ' ');
          SERIAL_STR(rply);
        }
      }
    }

    if (!normalMatrix.GaussJordan(numFactors, numFactors + 1)) {
			SERIAL_EM("Unable to calculate calibration parameters. Please reduce probe radius.");
			return;
		}

    float solution[numFactors];
    for (uint8_t i = 0; i < numFactors; ++i)
			solution[i] = normalMatrix(i, numFactors);

    // Debug Solved matrix, solution and residuals
    if (g33_debug) {
      SERIAL_EM("Solved matrix");
      for (uint8_t i = 0; i < numFactors; i++) {
        for (uint8_t j = 0; j < numFactors + 1; j++) {
          sprintf_P(rply, PSTR("%7.4f%c"), (double)normalMatrix(i, j), (j == numFactors) ? '\n' : ' ');
          SERIAL_STR(rply);
        }
      }
      SERIAL_MSG("Solution :");
      for (uint8_t i = 0; i < numFactors; i++) {
        sprintf_P(rply, PSTR(" %7.4f"), (double)solution[i]);
        SERIAL_STR(rply);
      }
      SERIAL_EOL();
      // Calculate and display the residuals
      SERIAL_MSG("Residuals:");
      for (uint8_t i = 0; i < probe_points; ++i) {
        float residual = BedProbePoints[i].z;
        for (uint8_t j = 0; j < numFactors; j++)
          residual += solution[j] * derivativeMatrix(i, j);
        sprintf_P(rply, PSTR(" %7.4f"), (double)residual);
        SERIAL_STR(rply);
      }
      SERIAL_EOL();
    }

    Adjust(numFactors, solution);

    // Calculate the expected probe heights using the new parameters
    float expectedResiduals[MaxCalibrationPoints];
    float sumOfSquares = 0.0f;

    for (int8_t i = 0; i < probe_points; i++) {
      probeMotorPositions[i].a += solution[A_AXIS];
      probeMotorPositions[i].b += solution[B_AXIS];
      probeMotorPositions[i].c += solution[C_AXIS];
      abc_pos_t newPosition;
      mechanics.InverseTransform(probeMotorPositions[i], newPosition);
      corrections[i] = newPosition.z;
      expectedResiduals[i] = BedProbePoints[i].z + newPosition.z;
      sumOfSquares += sq(expectedResiduals[i]);
    }

    expectedRmsError = SQRT((float)(sumOfSquares / probe_points));

    ++iteration;
  } while (iteration < 2);

  // convert data.endstop_adj;
  Convert_endstop_adj();

  SERIAL_MV("Calibrated ", numFactors);
  SERIAL_MV(" factors using ", probe_points);
  SERIAL_MV(" points, deviation before ", SQRT(initialSumOfSquares / probe_points), 4);
  SERIAL_MV(" after ", expectedRmsError, 4);
  SERIAL_EOL();

  mechanics.recalc_delta_settings();

  ac_home();

  const float measured_z = probe.check_at_point(0.0f, 0.0f, PROBE_PT_RAISE, 0);
  mechanics.data.height -= measured_z;
  mechanics.recalc_delta_settings();

  SERIAL_MV("Endstops X", mechanics.data.endstop_adj.a, 3);
  SERIAL_MV(" Y", mechanics.data.endstop_adj.b, 3);
  SERIAL_MV(" Z", mechanics.data.endstop_adj.c, 3);
  SERIAL_MV(" height ", mechanics.data.height, 3);
  SERIAL_MV(" diagonal rod ", mechanics.data.diagonal_rod, 3);
  SERIAL_MV(" delta radius ", mechanics.data.radius, 3);
  SERIAL_MV(" Towers angle correction I", mechanics.data.tower_angle_adj.a, 2);
  SERIAL_MV(" J", mechanics.data.tower_angle_adj.b, 2);
  SERIAL_MV(" K", mechanics.data.tower_angle_adj.c, 2);
  SERIAL_EOL();

  ac_cleanup();

}

#endif // ENABLED(DELTA_AUTO_CALIBRATION_1)

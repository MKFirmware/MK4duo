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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(DELTA_AUTO_CALIBRATION_1)

  #define CODE_G33

  void Calibration_cleanup(
    #if HOTENDS > 1
      const uint8_t old_tool_index
    #endif
  ) {
    #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
      mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
    #endif
    STOW_PROBE();
    printer.clean_up_after_endstop_or_probe_move();
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif
  }

  // Homed height
  float homed_height;
  void Calc_homed_height() {
    const float tempHeight = mechanics.data.diagonal_rod;		// any sensible height will do here, probably even zero
    float cartesian[ABC];
    mechanics.InverseTransform(tempHeight, tempHeight, tempHeight, cartesian);
    homed_height = mechanics.data.height + tempHeight - cartesian[Z_AXIS];
  }

  // Convert data.endstop_adj
  void Convert_endstop_adj() {
    LOOP_XYZ(i) mechanics.data.endstop_adj[i] *= -1;
  }

  // Normalize Endstop
  void NormaliseEndstopAdjustments() {
    const float min_endstop = MIN(mechanics.data.endstop_adj[A_AXIS], mechanics.data.endstop_adj[B_AXIS], mechanics.data.endstop_adj[C_AXIS]);
    LOOP_XYZ(i) mechanics.data.endstop_adj[i] -= min_endstop;
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
  void Adjust(const uint8_t numFactors, const float v[]) {

    const float oldHeightA = homed_height + mechanics.data.endstop_adj[A_AXIS];

    // Update endstop adjustments
    mechanics.data.endstop_adj[A_AXIS] += v[0];
    mechanics.data.endstop_adj[B_AXIS] += v[1];
    mechanics.data.endstop_adj[C_AXIS] += v[2];
    NormaliseEndstopAdjustments();

    if (numFactors >= 4) {
      mechanics.data.radius += v[3];

      if (numFactors >= 6) {
        mechanics.data.tower_angle_adj[A_AXIS] += v[4];
        mechanics.data.tower_angle_adj[B_AXIS] += v[5];

        if (numFactors == 7) mechanics.data.diagonal_rod += v[6];

      }
    }

    mechanics.recalc_delta_settings();
    Calc_homed_height();
    const float heightError = homed_height + mechanics.data.endstop_adj[A_AXIS] - oldHeightA - v[0];
    mechanics.data.height -= heightError;
    homed_height -= heightError;

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
  inline void gcode_G33(void) {

    const uint8_t MaxCalibrationPoints  = 10,
                  MaxnumFactors         = 7;

    uint8_t iteration = 0;

    float   xBedProbePoints[MaxCalibrationPoints],
            yBedProbePoints[MaxCalibrationPoints],
            zBedProbePoints[MaxCalibrationPoints],
            initialSumOfSquares,
            expectedRmsError;

    char    rply[50];

    const uint8_t numFactors = parser.intval('F', 7);
    if (!WITHIN(numFactors, 3, 7)) {
      SERIAL_EM("?(F)actors is implausible (3 to 7).");
      return;
    }

    const uint8_t probe_points  = parser.intval('P', 7);
    if (!WITHIN(probe_points, 1, 10)) {
      SERIAL_EM("?(P)oints is implausible (1 to 10).");
      return;
    }

    const bool g33_debug = parser.boolval('D');

    SERIAL_MV("Starting Auto Calibration ", probe_points);
    SERIAL_MV(" points and ", numFactors);
    SERIAL_MSG(" Factors");
    if (g33_debug) SERIAL_MSG(" Debug on");
    SERIAL_EOL();
    LCD_MESSAGEPGM(MSG_DELTA_AUTO_CALIBRATE);

    planner.synchronize();

    #if HAS_LEVELING
      bedlevel.reset(); // After calibration bed-level data is no longer valid
    #endif

    #if HOTENDS > 1
      const uint8_t old_tool_index = tools.active_extruder;
      tools.change(0, 0, true);
      #define CALIBRATION_CLEANUP() Calibration_cleanup(old_tool_index)
    #else
      #define CALIBRATION_CLEANUP() Calibration_cleanup()
    #endif

    printer.setup_for_endstop_or_probe_move();

    if (!mechanics.isHomedAll()) {
      endstops.setEnabled(true);
      mechanics.home();
      endstops.setNotHoming();
    }

    DEPLOY_PROBE();

    Calc_homed_height();

    for (uint8_t probe_index = 0; probe_index < 6; probe_index++) {
      xBedProbePoints[probe_index] = mechanics.data.probe_radius * SIN((2 * M_PI * probe_index) / 6);
      yBedProbePoints[probe_index] = mechanics.data.probe_radius * COS((2 * M_PI * probe_index) / 6);
      zBedProbePoints[probe_index] = probe.check_pt(xBedProbePoints[probe_index], yBedProbePoints[probe_index], PROBE_PT_RAISE, 4);
      if (isnan(zBedProbePoints[probe_index])) return CALIBRATION_CLEANUP();
    }
    if (probe_points >= 10) {
      for (uint8_t probe_index = 6; probe_index < 9; probe_index++) {
        xBedProbePoints[probe_index] = (mechanics.data.probe_radius / 2) * SIN((2 * M_PI * (probe_index - 6)) / 3);
        yBedProbePoints[probe_index] = (mechanics.data.probe_radius / 2) * COS((2 * M_PI * (probe_index - 6)) / 3);
        zBedProbePoints[probe_index] = probe.check_pt(xBedProbePoints[probe_index], yBedProbePoints[probe_index], PROBE_PT_RAISE, 4);
        if (isnan(zBedProbePoints[probe_index])) return CALIBRATION_CLEANUP();
      }
      xBedProbePoints[9] = 0.0;
      yBedProbePoints[9] = 0.0;
      zBedProbePoints[9] = probe.check_pt(0.0, 0.0, PROBE_PT_STOW, 4);
      if (isnan(zBedProbePoints[9])) return CALIBRATION_CLEANUP();
    }
    else {
      xBedProbePoints[6] = 0.0;
      yBedProbePoints[6] = 0.0;
      zBedProbePoints[6] = probe.check_pt(0.0, 0.0, PROBE_PT_STOW, 4);
      if (isnan(zBedProbePoints[6])) return CALIBRATION_CLEANUP();
    }

    // convert data.endstop_adj;
    Convert_endstop_adj();

    FixedMatrix<float, MaxCalibrationPoints, ABC> probeMotorPositions;
    float corrections[MaxCalibrationPoints];

    initialSumOfSquares = 0.0;

    // Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
    for (uint8_t i = 0; i < probe_points; ++i) {
      corrections[i] = 0.0;
      float machinePos[ABC] = { xBedProbePoints[i], yBedProbePoints[i], 0.0 };

      mechanics.Transform(machinePos);

      for (uint8_t axis = 0; axis < ABC; axis++)
        probeMotorPositions(i, axis) = mechanics.delta[axis];

      initialSumOfSquares += sq(zBedProbePoints[i]);
    }

    // Do 1 or more Newton-Raphson iterations
    do {

      // Build a Nx9 matrix of derivatives
      FixedMatrix<float, MaxCalibrationPoints, MaxnumFactors> derivativeMatrix;

      for (uint8_t i = 0; i < probe_points; i++) {
        for (uint8_t j = 0; j < numFactors; j++) {
          derivativeMatrix(i, j) =
            mechanics.ComputeDerivative(j, probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS));
        }
      }

      // Debug Derivative matrix
      if (g33_debug) {
        SERIAL_EM("Derivative matrix");
        for (uint8_t i = 0; i < probe_points; i++) {
          for (uint8_t j = 0; j < numFactors; j++) {
            sprintf_P(rply, PSTR("%7.4f%c"), (double)derivativeMatrix(i, j), (j == numFactors - 1) ? '\n' : ' ');
            SERIAL_PS(rply);
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
        float temp = -1 * derivativeMatrix(0, i) * (zBedProbePoints[0] + corrections[0]);
        for (uint8_t k = 1; k < probe_points; k++) {
          temp += -1 * derivativeMatrix(k, i) * (zBedProbePoints[k] + corrections[k]);
        }
        normalMatrix(i, numFactors) = temp;
      }

      // Debug Normal matrix
      if (g33_debug) {
        SERIAL_EM("Normal matrix");
        for (uint8_t i = 0; i < numFactors; i++) {
          for (uint8_t j = 0; j < numFactors + 1; j++) {
            sprintf_P(rply, PSTR("%7.4f%c"), (double)normalMatrix(i, j), (j == numFactors) ? '\n' : ' ');
            SERIAL_PS(rply);
          }
        }
      }

      float solution[numFactors];
      normalMatrix.GaussJordan(solution, numFactors);

      // Debug Solved matrix, solution and residuals
      if (g33_debug) {
        SERIAL_EM("Solved matrix");
        for (uint8_t i = 0; i < numFactors; i++) {
          for (uint8_t j = 0; j < numFactors + 1; j++) {
            sprintf_P(rply, PSTR("%7.4f%c"), (double)normalMatrix(i, j), (j == numFactors) ? '\n' : ' ');
            SERIAL_PS(rply);
          }
        }
        SERIAL_MSG("Solution :");
        for (uint8_t i = 0; i < numFactors; i++) {
          sprintf_P(rply, PSTR(" %7.4f"), (double)solution[i]);
          SERIAL_PS(rply);
        }
        SERIAL_EOL();
        // Calculate and display the residuals
        SERIAL_MSG("Residuals:");
        for (uint8_t i = 0; i < probe_points; ++i) {
          float residual = zBedProbePoints[i];
          for (uint8_t j = 0; j < numFactors; j++)
            residual += solution[j] * derivativeMatrix(i, j);
          sprintf_P(rply, PSTR(" %7.4f"), (double)residual);
          SERIAL_PS(rply);
        }
        SERIAL_EOL();
      }

      Adjust(numFactors, solution);

      // Calculate the expected probe heights using the new parameters
      float expectedResiduals[MaxCalibrationPoints];
      float sumOfSquares = 0.0;

      for (int8_t i = 0; i < probe_points; i++) {
        LOOP_XYZ(axis) probeMotorPositions(i, axis) += solution[axis];
        float newPosition[ABC];
        mechanics.InverseTransform(probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS), newPosition);
        corrections[i] = newPosition[Z_AXIS];
        expectedResiduals[i] = zBedProbePoints[i] + newPosition[Z_AXIS];
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

    endstops.setEnabled(true);
    mechanics.home();
    endstops.setNotHoming();

    const float measured_z = probe.check_pt(0, 0, PROBE_PT_RAISE, 0);
    mechanics.data.height -= measured_z;
    mechanics.recalc_delta_settings();

    SERIAL_MV("Endstops X", mechanics.data.endstop_adj[A_AXIS], 3);
    SERIAL_MV(" Y", mechanics.data.endstop_adj[B_AXIS], 3);
    SERIAL_MV(" Z", mechanics.data.endstop_adj[C_AXIS], 3);
    SERIAL_MV(" height ", mechanics.data.height, 3);
    SERIAL_MV(" diagonal rod ", mechanics.data.diagonal_rod, 3);
    SERIAL_MV(" delta radius ", mechanics.data.radius, 3);
    SERIAL_MV(" Towers angle correction I", mechanics.data.tower_angle_adj[A_AXIS], 2);
    SERIAL_MV(" J", mechanics.data.tower_angle_adj[B_AXIS], 2);
    SERIAL_MV(" K", mechanics.data.tower_angle_adj[C_AXIS], 2);
    SERIAL_EOL();

    CALIBRATION_CLEANUP();

  }

#endif // ENABLED(DELTA_AUTO_CALIBRATION_1)

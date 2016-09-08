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

/**
 * cartesian_correction.h
 * A class that manages hysteresis by inserting extra planner.buffer_line when necessary
 * A class that manages ZWobble
 *
 * Copyright (c) 2012 Neil James Martin
 * Copyright (c) 2013 Francesco Santini
 * Copyright (c) 2016 MagoKimbra
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _CARTESIAN_CORRECTION_H
  #define _CARTESIAN_CORRECTION_H

  #ifdef HYSTERESIS
    //===========================================================================
    class Hysteresis {
    public:
      Hysteresis(float x_mm, float y_mm, float z_mm, float e_mm);

      void Set(float x_mm, float y_mm, float z_mm, float e_mm);
      void SetAxis(uint8_t axis, float mm);
      void ReportToSerial();
      void InsertCorrection(const float x, const float y, const float z, const float e);

    private:
      void      calcSteps();
      float     m_hysteresis_mm[NUM_AXIS];
      long      m_hysteresis_steps[NUM_AXIS];
      uint8_t   m_prev_direction_bits;
      uint8_t   m_hysteresis_bits;
    };

    //===========================================================================
    extern Hysteresis hysteresis;
  #endif // HYSTERESIS

  #ifdef ZWOBBLE
    #define STEPS_IN_ZLUT 50
    #define ZWOBBLE_MIN_Z 0.1
    //===========================================================================
    class ZWobble {
    public:
      ZWobble(float _amplitude, float _period, float _phase);

      void Set(float _amplitude, float _period, float _phase);
      void ReportToSerial();
      void InsertCorrection(const float targetZ);

      void setAmplitude(float _amplitude);
      void setPeriod(float _period);
      void setPhase(float _phase);
      void setSample(float zRod, float zActual);
      void setScaledSample(float zRod, float zScaledLength);
      void setScalingFactor(float zActualPerScaledLength);

    private:
      float     m_amplitude, m_puls, m_phase;
      bool      m_consistent;

      int       lutSize;
      float     zLut[STEPS_IN_ZLUT][2];
      void      calculateLut();
      void      initLinearLut();
      void      insertInLut(float, float);
      float     findInLut(float);
      float     findZRod(float);
      bool      areParametersConsistent();

      float     lastZ, lastZRod;
      float     m_scalingFactor;
      bool      m_sinusoidal;

    };

    //===========================================================================
    extern ZWobble zwobble;
  #endif // ZWOBBLE

#endif // _CARTESIAN_CORRECTION_H

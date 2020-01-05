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

template<class T>
class MathMatrix {

  public: /** Destructor */

    virtual ~MathMatrix() { }

  public: /** Public Function */

    virtual uint8_t rows()                              const = 0;
    virtual uint8_t cols()                              const = 0;
    virtual T& operator()       (uint8_t r, uint8_t c)        = 0;
    virtual const T& operator() (uint8_t r, uint8_t c)  const = 0;

};

template<class T, uint8_t ROWS, uint8_t COLS>
class FixedMatrix : public MathMatrix<T> {

  private: /** Private Parameters */

    T data[ROWS][COLS];

  public: /** Public Function */

    uint8_t rows() const override { return ROWS; }
    uint8_t cols() const override { return COLS; }

    T& operator() (uint8_t r, uint8_t c) override { return data[r][c]; }

    const T& operator() (uint8_t r, uint8_t c) const override { return data[r][c]; }

    void SwapRows(uint8_t i, uint8_t j, uint8_t numCols = COLS);

    bool GaussJordan(uint8_t numRows, uint8_t numCols);

    T* GetRow(uint8_t r) { return data[r]; }

    const T* GetRow(uint8_t r) const { return data[r]; }

    void Fill(T val);
};

template<class T, uint8_t ROWS, uint8_t COLS>
inline void FixedMatrix<T, ROWS, COLS>::SwapRows(uint8_t i, uint8_t j, uint8_t numCols) {
  if (i != j) {
    for (uint8_t k = 0; k < numCols; ++k) {
      T temp = data[i][k];
      data[i][k] = data[j][k];
			data[j][k] = temp;
    }
  }
}

// Perform Gauss-Jordan elimination on a N x (N+M) matrix. Return true if successful, false if not possible.
template<class T, uint8_t ROWS, uint8_t COLS>
bool FixedMatrix<T, ROWS, COLS>::GaussJordan(uint8_t numRows, uint8_t numCols) {
  for (uint8_t i = 0; i < numRows; ++i) {
    // Swap the rows around for stable Gauss-Jordan elimination
    float vmax = fabsf(data[i][i]);
    for (uint8_t j = i + 1; j < numRows; ++j) {
      const float rmax = fabsf(data[j][i]);
      if (rmax > vmax) {
        SwapRows(i, j, numCols);
        vmax = rmax;
      }
    }

    // Use row i to eliminate the element in the ith column from previous and subsequent rows
    const T v = data[i][i];
    if (v == (T)0.0) return false;

    for (uint8_t j = 0; j < i; ++j) {
      const T factor = data[j][i] / v;
      data[j][i] = (T)0.0;
      for (uint8_t k = i + 1; k < numCols; ++k)
        data[j][k] -= data[i][k] * factor;
    }

    for (uint8_t j = i + 1; j < numRows; ++j) {
      const T factor = data[j][i] / v;
      data[j][i] = (T)0.0;
      for (uint8_t k = i + 1; k < numCols; ++k)
        data[j][k] -= data[i][k] * factor;
    }
  }

  for (uint8_t r = 0; r < numRows; ++r) {
    const T val = data[r][r];
    for (uint8_t c = numRows; c < numCols; ++c) data[r][c] /= val;
    data[r][r] = (T)1.0;
  }

  return true;
}

// Set all elements to a specified value
template<class T, uint8_t ROWS, uint8_t COLS>
void FixedMatrix<T, ROWS, COLS>::Fill(T val) {
	for (uint8_t i = 0; i < ROWS; ++i) {
		for (uint8_t j = 0; j < COLS; ++j)
			data[i][j] = val;
	}
}

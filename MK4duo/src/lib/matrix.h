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

    T data[ROWS * COLS];

  public: /** Public Function */

    uint8_t rows() const override { return ROWS; }
    uint8_t cols() const override { return COLS; }

    T& operator() (uint8_t r, uint8_t c) override { return data[r * COLS + c]; }

    const T& operator() (uint8_t r, uint8_t c) const override { return data[r * COLS + c]; }

    void SwapRows(uint8_t i, uint8_t j, uint8_t numCols = COLS);

    void GaussJordan(T *solution, uint8_t numRows);

    T* GetRow(uint8_t r) { return data + (r * COLS); }

    const T* GetRow(uint8_t r) const { return data + (r * COLS); }

};

template<class T, uint8_t ROWS, uint8_t COLS>
inline void FixedMatrix<T, ROWS, COLS>::SwapRows(uint8_t i, uint8_t j, uint8_t numCols) {
  if (i != j) {
    for (uint8_t k = i; k < numCols; ++k) {
      T temp = (*this)(i, k);
      (*this)(i, k) = (*this)(j, k);
      (*this)(j, k) = temp;
    }
  }
}

// Perform Gauss-Jordan elimination on a N x (N+1) matrix.
// Returns a pointer to the solution vector.
template<class T, uint8_t ROWS, uint8_t COLS>
void FixedMatrix<T, ROWS, COLS>::GaussJordan(T *solution, uint8_t numRows) {
  for (uint8_t i = 0; i < numRows; ++i) {
    // Swap the rows around for stable Gauss-Jordan elimination
    float vmax = fabsf((*this)(i, i));
    for (uint8_t j = i + 1; j < numRows; ++j) {
      const float rmax = fabsf((*this)(j, i));
      if (rmax > vmax) {
        SwapRows(i, j);
        vmax = rmax;
      }
    }

    T v = (*this)(i, i);
    for (uint8_t j = 0; j < i; ++j) {
      T factor = (*this)(j, i) / v;
      (*this)(j, i) = 0.0;
      for (uint8_t k = i + 1; k <= numRows; ++k)
        (*this)(j, k) -= (*this)(i, k) * factor;
    }

    for (uint8_t j = i + 1; j < numRows; ++j) {
      T factor = (*this)(j, i) / v;
      (*this)(j, i) = 0.0;
      for (uint8_t k = i + 1; k <= numRows; ++k)
        (*this)(j, k) -= (*this)(i, k) * factor;
    }
  }

  for (uint8_t i = 0; i < numRows; ++i)
    solution[i] = (*this)(i, numRows) / (*this)(i, i);

}

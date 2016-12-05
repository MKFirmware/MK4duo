#ifndef __MATRIX_MATH_H__
#define __MATRIX_MATH_H__

namespace mm {
  template <typename T, uint8_t rows, uint8_t cols>
    class MathMatrix {
      public:

        T& get(const uint8_t& row, const uint8_t& col) {
          return data[row][col];
        }

        void set(const uint8_t& row, const uint8_t& col, T value) {
          data[row][col] = value;
        }

        T& operator()(const uint8_t& row, const uint8_t& col) {
          return this->data[row][col];
        }

        const T& operator()(const uint8_t& row, const uint8_t& col) const {
          return this->data[row][col];
        }

        void SwapRows(uint8_t i, uint8_t j, uint8_t numCols = cols);

        void GaussJordan(T *solution, uint8_t numRows);

      private:
        T data[rows][cols];

    };

    // Swap 2 rows of a matrix
    template<class T, uint8_t rows, uint8_t cols> inline void MathMatrix<T, rows, cols>::SwapRows(uint8_t i, uint8_t j, uint8_t numCols) {
      if (i != j) {
        for (uint8_t k = 0; k < numCols; k++) {
          T temp = (*this)(i, k);
          (*this)(i, k) = (*this)(j, k);
          (*this)(j, k) = temp;
        }
      }
    }

    // Perform Gauss-Jordan elimination on a N x (N+1) matrix.
    // Returns a pointer to the solution vector.
    template<class T, uint8_t rows, uint8_t cols> void MathMatrix<T, rows, cols>::GaussJordan(T *solution, uint8_t numRows) {
      for (uint8_t i = 0; i < numRows; i++) {
        // Swap the rows around for stable Gauss-Jordan elimination
        float vmax = abs((*this)(i, i));
        for (uint8_t j = i + 1; j < numRows; j++) {
          const float rmax = abs((*this)(j, i));
          if (rmax > vmax) {
            SwapRows(i, j, numRows + 1);
            vmax = rmax;
          }
        }

        // Use row i to eliminate the ith element from previous and subsequent rows
        float v = (*this)(i, i);
        for (uint8_t j = 0; j < i; j++)	{
          float factor = (*this)(j, i) / v;
          (*this)(j, i) = 0.0;
          for (uint8_t k = i + 1; k <= numRows; k++) {
            (*this)(j, k) -= (*this)(i, k) * factor;
          }
        }

        for (uint8_t j = i + 1; j < numRows; j++) {
          float factor = (*this)(j, i)/v;
          (*this)(j, i) = 0.0;
          for (uint8_t k = i + 1; k <= numRows; k++) {
            (*this)(j, k) -= (*this)(i, k) * factor;
          }
        }
      }

      for (uint8_t i = 0; i < numRows; i++)	{
        solution[i] = (*this)(i, numRows) / (*this)(i, i);
      }
    }
}
#endif

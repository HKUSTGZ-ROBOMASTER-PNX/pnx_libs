#include "matrix.hpp"

#include "numeric.hpp"

#include <cstdlib>

namespace matrix {

void mat_init(Mat* matrix, uint8_t rows, uint8_t cols) {
  if (matrix == nullptr) {
    return;
  }
  matrix->numRows = rows;
  matrix->numCols = cols;
  matrix->pData = static_cast<float*>(std::malloc(rows * cols * sizeof(float)));
}

float* normalize_3d(float* vector) {
  if (vector == nullptr) {
    return nullptr;
  }
  const float length =
      numeric::sqrt_f(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  if (length == 0.0f) {
    return vector;
  }
  vector[0] /= length;
  vector[1] /= length;
  vector[2] /= length;
  return vector;
}

float norm_3d(const float* vector) {
  if (vector == nullptr) {
    return 0.0f;
  }
  return numeric::sqrt_f(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

void cross_3d(const float* v1, const float* v2, float* result) {
  if (v1 == nullptr || v2 == nullptr || result == nullptr) {
    return;
  }
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = v1[2] * v2[0] - v1[0] * v2[2];
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

float dot_3d(const float* v1, const float* v2) {
  if (v1 == nullptr || v2 == nullptr) {
    return 0.0f;
  }
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

}  // namespace matrix

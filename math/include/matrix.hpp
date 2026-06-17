#pragma once

#include <cstdint>

#include "arm_math.h"

namespace matrix {

using Mat = arm_matrix_instance_f32;

constexpr auto kMatAdd = arm_mat_add_f32;
constexpr auto kMatSubtract = arm_mat_sub_f32;
constexpr auto kMatMultiply = arm_mat_mult_f32;
constexpr auto kMatTranspose = arm_mat_trans_f32;
constexpr auto kMatInverse = arm_mat_inverse_f32;

void mat_init(Mat* matrix, uint8_t rows, uint8_t cols);
float* normalize_3d(float* vector);
float norm_3d(const float* vector);
void cross_3d(const float* v1, const float* v2, float* result);
float dot_3d(const float* v1, const float* v2);

}  // namespace matrix

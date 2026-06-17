#pragma once

#include <cstdint>

#include "arm_math.h"

namespace numeric {

inline float sin_f(float x) { return arm_sin_f32(x); }
inline float cos_f(float x) { return arm_cos_f32(x); }

constexpr float kPi = 3.14159265358979f;
constexpr float kPiX2 = 6.283185307f;
constexpr float kGravity = 9.78f;
constexpr float kDegreeToRad = kPi / 180.0f;
constexpr float kRadToDegree = 180.0f / kPi;

float calculate_shortest_angle(float target_rad, float current_rad);
float loop_float_constrain(float input, float min_value, float max_value);
float float_constrain(float input, float min_value, float max_value);
float limit_abs(float input, float max_value);
float limit_abs_smaller(float input, float max_value, float constraint);
uint32_t convert_to_fixed(float value, float min_value, float precision);
float convert_from_fixed(uint32_t value, float min_value, float precision);
float inv_sqrt(float x);
float sqrt_f(float x);
float abs_f(float x);
float sign_f(float value);
float float_deadband(float value, float min_value, float max_value);
int16_t int16_constrain(int16_t value, int16_t min_value, int16_t max_value);
float theta_format(float angle_deg);
int float_rounding(float raw);
float average_filter(float new_data, float* buffer, uint8_t length);
int float_to_uint(float value, float min_value, float max_value, int bits);
float uint_to_float(int value, float min_value, float max_value, int bits);

}  // namespace numeric

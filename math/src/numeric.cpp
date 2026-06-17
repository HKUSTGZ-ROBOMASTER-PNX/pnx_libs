#include "numeric.hpp"

#include <cmath>
#include <cstdlib>

namespace numeric {

float calculate_shortest_angle(float target_rad, float current_rad) {
  float delta = target_rad - current_rad;
  while (delta > kPi) {
    delta -= kPiX2;
  }
  while (delta < -kPi) {
    delta += kPiX2;
  }
  return delta;
}

float loop_float_constrain(float input, float min_value, float max_value) {
  if (max_value < min_value) {
    return input;
  }

  const float range = max_value - min_value;
  if (range == 0.0f) {
    return min_value;
  }

  float normalized = input - min_value;
  normalized = std::fmod(normalized, range);
  if (normalized < 0.0f) {
    normalized += range;
  }
  return min_value + normalized;
}

float float_constrain(float input, float min_value, float max_value) {
  if (max_value < min_value) {
    return input;
  }
  if (input > max_value) {
    return max_value;
  }
  if (input < min_value) {
    return min_value;
  }
  return input;
}

float limit_abs(float input, float max_value) {
  if (input > max_value) {
    return max_value;
  }
  if (input < -max_value) {
    return -max_value;
  }
  return input;
}

float limit_abs_smaller(float input, float max_value, float constraint) {
  if (input > max_value) {
    return constraint;
  }
  if (input < -max_value) {
    return -constraint;
  }
  return input;
}

uint32_t convert_to_fixed(float value, float min_value, float precision) {
  return static_cast<uint32_t>((value - min_value) / precision);
}

float convert_from_fixed(uint32_t value, float min_value, float precision) {
  return static_cast<float>(value) * precision + min_value;
}

float inv_sqrt(float x) {
  float xhalf = 0.5f * x;
  int i = *reinterpret_cast<int*>(&x);
  i = 0x5f3759df - (i >> 1);
  x = *reinterpret_cast<float*>(&i);
  x = x * (1.5f - xhalf * x * x);
  return x;
}

float sqrt_f(float x) {
  if (x <= 0.0f) {
    return 0.0f;
  }

  float y = x / 2.0f;
  float delta = 0.0f;
  const float max_error = x * 0.001f;

  do {
    delta = (y * y) - x;
    y -= delta / (2.0f * y);
  } while (delta > max_error || delta < -max_error);

  return y;
}

float abs_f(float x) { return x > 0.0f ? x : -x; }

float sign_f(float value) { return value >= 0.0f ? 1.0f : -1.0f; }

float float_deadband(float value, float min_value, float max_value) {
  if (value < max_value && value > min_value) {
    return 0.0f;
  }
  return value;
}

int16_t int16_constrain(int16_t value, int16_t min_value, int16_t max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

float theta_format(float angle_deg) {
  return loop_float_constrain(angle_deg, -180.0f, 180.0f);
}

int float_rounding(float raw) {
  const int integer = static_cast<int>(raw);
  const float decimal = raw - static_cast<float>(integer);
  if (decimal > 0.5f) {
    return integer + 1;
  }
  return integer;
}

float average_filter(float new_data, float* buffer, uint8_t length) {
  if (buffer == nullptr || length == 0) {
    return new_data;
  }

  for (uint8_t i = length - 1; i > 0; --i) {
    buffer[i] = buffer[i - 1];
  }
  buffer[0] = new_data;

  float sum = 0.0f;
  for (uint8_t i = 0; i < length; ++i) {
    sum += buffer[i];
  }
  return sum / static_cast<float>(length);
}

int float_to_uint(float value, float min_value, float max_value, int bits) {
  const float span = max_value - min_value;
  return static_cast<int>((value - min_value) * static_cast<float>((1 << bits) - 1) / span);
}

float uint_to_float(int value, float min_value, float max_value, int bits) {
  const float span = max_value - min_value;
  return static_cast<float>(value) * span / static_cast<float>((1 << bits) - 1) + min_value;
}

}  // namespace numeric

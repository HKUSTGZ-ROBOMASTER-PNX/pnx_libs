#pragma once

#include <cstdint>

namespace pid {

enum Mode : uint8_t {
  Position = 0x01,
  Delta = 0x02,
  TrapezoidIntegral = 0x04,
  ChangingIntegralRate = 0x08,
  IntegralSeparation = 0x10,
  DerivativeOnMeasurement = 0x20,
  IntegralLimit = 0x40,
  DerivativeIncomplete = 0x80,
};

class Pid {
 public:
  Pid(float kp, float ki, float kd, float max_out, float max_i_out, uint8_t mode = Position);

  void tuning(float kp, float ki, float kd);
  void update_result(float velocity = 0.0f);
  void clear();

  uint8_t mode_ = Position;

  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;

  float scalar_a_ = 0.0f;
  float scalar_b_ = 0.0f;

  float tau_ = 0.0f;

  float fdf_ = 0.0f;
  float ref_ = 0.0f;
  float fdb_ = 0.0f;
  float last_fdb_ = 0.0f;
  float last_ref_ = 0.0f;
  float err_[3]{};

  float p_result_ = 0.0f;
  float i_result_ = 0.0f;
  float d_result_ = 0.0f;
  float i_term_ = 0.0f;
  float result_ = 0.0f;

  float max_out_ = 0.0f;
  float max_i_out_ = 0.0f;

  float deadband_ = 0.0f;

  int error_count_ = 0;
  int right_count_ = 0;

  bool motor_blocked_ = false;
  bool motor_normal_ = true;
};

}  // namespace pid

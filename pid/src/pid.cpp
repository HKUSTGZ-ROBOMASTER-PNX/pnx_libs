#include "pid.hpp"

#include "numeric.hpp"

#include <cmath>

namespace pid {
namespace {

void apply_trapezoid_integral(Pid* pid) {
  pid->i_term_ = pid->ki_ * ((pid->err_[0] + pid->err_[1]) / 2.0f);
}

void apply_derivative_on_measurement(Pid* pid) {
  pid->d_result_ = pid->kd_ * (pid->last_fdb_ - pid->fdb_);
}

void apply_integral_separation(Pid* pid) {
  if (pid->err_[0] * pid->i_result_ > 0.0f) {
    if (std::fabs(pid->err_[0]) <= pid->max_out_) {
      return;
    }
    pid->i_term_ = 0.0f;
  }
}

void apply_changing_integral_rate(Pid* pid) {
  if (pid->err_[0] * pid->i_result_ > 0.0f) {
    if (std::fabs(pid->err_[0]) <= pid->scalar_b_) {
      return;
    }
    if (std::fabs(pid->err_[0]) <= (pid->scalar_a_ + pid->scalar_b_)) {
      pid->i_term_ *= (pid->scalar_a_ - std::fabs(pid->err_[0]) + pid->scalar_b_) / pid->scalar_a_;
      return;
    }
    pid->i_term_ = 0.0f;
  }
}

void apply_integral_limit(Pid* pid) {
  const float temp_i_out = pid->i_result_ + pid->i_term_;
  const float temp_output = pid->p_result_ + pid->i_result_ + pid->d_result_;
  if (std::fabs(temp_output) > pid->max_out_) {
    if (pid->err_[0] * pid->i_result_ > 0.0f) {
      pid->i_term_ = 0.0f;
    }
  }

  if (temp_i_out > pid->max_i_out_) {
    pid->i_term_ = 0.0f;
    pid->i_result_ = pid->max_i_out_;
  }
  if (temp_i_out < -pid->max_i_out_) {
    pid->i_term_ = 0.0f;
    pid->i_result_ = -pid->max_i_out_;
  }
}

}  // namespace

Pid::Pid(float kp, float ki, float kd, float max_out, float max_i_out, uint8_t mode)
    : mode_(mode),
      kp_(kp),
      ki_(ki),
      kd_(kd),
      max_out_(max_out),
      max_i_out_(max_i_out) {
  fdb_ = last_fdb_ = 0.0f;
  ref_ = last_ref_ = 0.0f;
  err_[0] = err_[1] = err_[2] = 0.0f;
  scalar_a_ = scalar_b_ = 0.0f;
  fdf_ = 0.0f;
  p_result_ = i_result_ = d_result_ = 0.0f;
  i_term_ = 0.0f;
  tau_ = 0.0f;
  result_ = 0.0f;
  motor_blocked_ = false;
  motor_normal_ = true;
  error_count_ = right_count_ = 0;
  deadband_ = 0.0f;
}

void Pid::tuning(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void Pid::update_result(float velocity) {
  err_[2] = err_[1];
  err_[1] = err_[0];
  err_[0] = ref_ - fdb_;

  p_result_ = kp_ * err_[0];
  i_term_ = ki_ * err_[0];

  if (mode_ == Position) {
    d_result_ = kd_ * (err_[0] - err_[1]);
  } else if (mode_ == Delta) {
    d_result_ = kd_ * velocity;
  }

  if ((mode_ & TrapezoidIntegral) != 0U) {
    apply_trapezoid_integral(this);
  }
  if ((mode_ & DerivativeOnMeasurement) != 0U) {
    apply_derivative_on_measurement(this);
  }
  if ((mode_ & IntegralSeparation) != 0U) {
    apply_integral_separation(this);
  }
  if ((mode_ & ChangingIntegralRate) != 0U) {
    apply_changing_integral_rate(this);
  }
  if ((mode_ & IntegralLimit) != 0U) {
    apply_integral_limit(this);
  }

  i_result_ += i_term_;
  i_result_ = numeric::limit_abs(i_result_, max_i_out_);

  last_fdb_ = fdb_;
  result_ = p_result_ + i_result_ + d_result_;
  result_ = numeric::limit_abs(result_, max_out_);
}

void Pid::clear() {
  last_fdb_ = fdb_ = 0.0f;
  err_[0] = err_[1] = err_[2] = 0.0f;
  p_result_ = i_result_ = d_result_ = result_ = 0.0f;
  ref_ = fdb_ = 0.0f;
  i_result_ = 0.0f;
  p_result_ = 0.0f;
  d_result_ = 0.0f;
  scalar_a_ = 1.0f;
  scalar_b_ = 1.0f;
}

}  // namespace pid

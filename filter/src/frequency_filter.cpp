#include "frequency_filter.hpp"

#include "numeric.hpp"

namespace filter {

IirFilter::IirFilter(float omega_n, float zeta, float dt) {
  const float k = 2.0f / dt;
  const float wn2 = omega_n * omega_n;
  const float den = k * k + 2.0f * zeta * omega_n * k + wn2;

  coeff_.resize(5);
  coeff_[0] = wn2 / den;
  coeff_[1] = 2.0f * coeff_[0];
  coeff_[2] = coeff_[0];
  coeff_[3] = -(2.0f * wn2 - 2.0f * k * k) / den;
  coeff_[4] = -(k * k - 2.0f * zeta * omega_n * k + wn2) / den;

  gain_ = 1.0f;
  arm_biquad_cascade_df1_init_f32(&section_, 1, coeff_.data(), buffer_);
}

float IirFilter::update(float input) const {
  float result = 0.0f;
  arm_biquad_cascade_df1_f32(&section_, &input, &result, 1);
  return result * gain_;
}

void IirFilter::reset() {
  for (float& sample : buffer_) {
    sample = 0.0f;
  }
}

FirFilter::FirFilter(uint8_t order, float constrain_low, float constrain_high, FilterMode mode,
                     float freq_low, float freq_high) {
  order_ = order;
  constrain_low_ = constrain_low;
  constrain_high_ = constrain_high;
  filter_mode_ = mode;
  freq_low_ = freq_low;
  freq_high_ = freq_high;

  system_function_.assign(order + 1U, 0.0f);
  input_signal_.assign(order + 1U, 0.0f);

  float system_function_sum = 0.0f;
  const float omega_low = 2.0f * numeric::kPi * freq_low_ / sample_rate_hz_;
  const float omega_high = 2.0f * numeric::kPi * freq_high_ / sample_rate_hz_;

  switch (filter_mode_) {
    case FilterMode::LowPass:
      for (uint8_t i = 0; i < order + 1U; ++i) {
        system_function_[i] =
            omega_low / numeric::kPi *
            numeric::sin_f((static_cast<float>(i) - order / 2.0f) * omega_low);
      }
      break;
    case FilterMode::HighPass:
      for (uint8_t i = 0; i < order + 1U; ++i) {
        system_function_[i] =
            numeric::sin_f((static_cast<float>(i) - order / 2.0f) * numeric::kPi) -
            omega_high / numeric::kPi *
                numeric::sin_f((static_cast<float>(i) - order / 2.0f) * omega_high);
      }
      break;
    case FilterMode::BandPass:
      for (uint8_t i = 0; i < order + 1U; ++i) {
        system_function_[i] =
            omega_high / numeric::kPi *
                numeric::sin_f((static_cast<float>(i) - order / 2.0f) * omega_high) -
            omega_low / numeric::kPi *
                numeric::sin_f((static_cast<float>(i) - order / 2.0f) * omega_low);
      }
      break;
    case FilterMode::BandStop:
      for (uint8_t i = 0; i < order + 1U; ++i) {
        system_function_[i] =
            numeric::sin_f((static_cast<float>(i) - order / 2.0f) * numeric::kPi) +
            omega_low / numeric::kPi *
                numeric::sin_f((static_cast<float>(i) - order / 2.0f) * omega_low) -
            omega_high / numeric::kPi *
                numeric::sin_f((static_cast<float>(i) - order / 2.0f) * omega_high);
      }
      break;
  }

  for (uint8_t i = 0; i < order + 1U; ++i) {
    system_function_sum += system_function_[i];
  }
  for (uint8_t i = 0; i < order + 1U; ++i) {
    system_function_[i] /= system_function_sum;
  }
}

void FirFilter::set_now(float value) {
  float now_value = value;
  if (constrain_low_ != 0.0f || constrain_high_ != 0.0f) {
    now_value = numeric::float_constrain(value, constrain_low_, constrain_high_);
  }

  input_signal_[signal_flag_] = now_value;
  signal_flag_++;
  if (signal_flag_ == order_ + 1U) {
    signal_flag_ = 0;
  }
}

float FirFilter::update() const {
  float result = 0.0f;
  for (uint8_t i = 0; i < order_ + 1U; ++i) {
    result += system_function_[i] * input_signal_[(signal_flag_ + i) % (order_ + 1U)];
  }
  return result;
}

}  // namespace filter

#pragma once

#include <cstdint>
#include <vector>

#include "arm_math.h"

namespace filter {

constexpr float kDefaultSamplingFrequency = 1000.0f;

enum class FilterMode {
  LowPass = 0,
  HighPass,
  BandPass,
  BandStop,
};

class FirFilter {
 public:
  FirFilter(uint8_t order = 50, float constrain_low = 0.0f, float constrain_high = 0.0f,
            FilterMode mode = FilterMode::LowPass, float freq_low = 0.0f,
            float freq_high = kDefaultSamplingFrequency / 2.0f);

  void set_now(float value);
  float update() const;

  uint32_t order_ = 50;
  uint8_t signal_flag_ = 0;

  float constrain_low_ = 0.0f;
  float constrain_high_ = 0.0f;
  FilterMode filter_mode_ = FilterMode::LowPass;
  float freq_low_ = 0.0f;
  float freq_high_ = 0.0f;
  float sample_rate_hz_ = kDefaultSamplingFrequency;

  std::vector<float> system_function_;
  std::vector<float> input_signal_;
};

class IirFilter {
 public:
  explicit IirFilter(float omega_n, float zeta, float dt);

  float update(float input) const;
  void reset();

 private:
  float gain_ = 1.0f;
  std::vector<float> coeff_;
  arm_biquad_casd_df1_inst_f32 section_{};
  mutable float buffer_[8]{};
};

}  // namespace filter

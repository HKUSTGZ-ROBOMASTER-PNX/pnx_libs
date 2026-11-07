#pragma once

#include "biquad.hpp"
#include "constants.hpp"
#include <cmath>

namespace filter
{

struct iir
{
    // Second-order low-pass filter.
    // Default damping: Butterworth (zeta = 1 / sqrt(2)).
    static biquad low_pass(float cutoff_hz, float sample_rate_hz, float zeta = 0.70710678f)
    {
        biquad result{};

        if (!std::isfinite(cutoff_hz) || !std::isfinite(sample_rate_hz) ||
            !std::isfinite(zeta) || sample_rate_hz <= 0.0f || cutoff_hz <= 0.0f ||
            cutoff_hz >= sample_rate_hz / 2.0f || zeta <= 0.0f)
        {
            return result; // Bypass
        }

        const float k = std::tan(math::pi * cutoff_hz / sample_rate_hz);
        const float norm = 1.0f / (1.0f + 2.0f * zeta * k + k * k);

        result.b0 = result.b2 = k * k * norm;
        result.b1 = 2.0f * result.b0;
        result.a1 = 2.0f * (1.0f - k * k) * norm;
        result.a2 = -(1.0f - 2.0f * zeta * k + k * k) * norm;

        return result;
    }

    // Second-order high-pass filter.
    // Default damping: Butterworth.
    static biquad high_pass(float cutoff_hz, float sample_rate_hz, float zeta = 0.70710678f)
    {
        biquad result{};

        if (!std::isfinite(cutoff_hz) || !std::isfinite(sample_rate_hz) ||
            !std::isfinite(zeta) || sample_rate_hz <= 0.0f || cutoff_hz <= 0.0f ||
            cutoff_hz >= sample_rate_hz / 2.0f || zeta <= 0.0f)
        {
            return result; // Bypass
        }

        const float k = std::tan(math::pi * cutoff_hz / sample_rate_hz);
        const float norm = 1.0f / (1.0f + 2.0f * zeta * k + k * k);

        result.b0 = result.b2 = norm;
        result.b1 = -2.0f * norm;
        result.a1 = 2.0f * (1.0f - k * k) * norm;
        result.a2 = -(1.0f - 2.0f * zeta * k + k * k) * norm;

        return result;
    }

    // Second-order notch filter.
    // bandwidth_hz uses the legacy Q = center / bandwidth.
    static biquad notch(float center_hz, float bandwidth_hz, float sample_rate_hz)
    {
        biquad result{};

        if (!std::isfinite(center_hz) || !std::isfinite(bandwidth_hz) ||
            !std::isfinite(sample_rate_hz) || sample_rate_hz <= 0.0f || center_hz <= 0.0f ||
            center_hz >= sample_rate_hz / 2.0f || bandwidth_hz <= 0.0f)
        {
            return result; // Bypass
        }

        const float q = center_hz / bandwidth_hz;
        const float omega = 2.0f * math::pi * center_hz / sample_rate_hz;

        const float c = std::cos(omega);
        const float alpha = std::sin(omega) / (2.0f * q);
        const float norm = 1.0f / (1.0f + alpha);

        result.b0 = result.b2 = norm;
        result.b1 = -2.0f * c * norm;
        result.a1 = 2.0f * c * norm;
        result.a2 = -(1.0f - alpha) * norm;

        return result;
    }
};

} // namespace filter

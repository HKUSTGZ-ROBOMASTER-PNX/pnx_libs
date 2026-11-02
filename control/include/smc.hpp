#pragma once

#include <cmath>
#include <cstdint>

#include "constrains.hpp"

namespace control
{

enum class smc_mode : std::uint8_t
{
    exponential,
    power,
    terminal,
    velocity,
    error_integral,
};

struct smc_config
{
    smc_mode mode = smc_mode::exponential;

    float inertia = 0.0f;        // J
    float reaching_gain = 0.0f;  // K
    float surface_gain = 0.0f;   // c, or c1 for error_integral
    float integral_gain = 0.0f;  // c2 for error_integral
    float terminal_gain = 0.0f;  // beta
    std::uint16_t terminal_p = 3; // Positive odd integer, p > q.
    std::uint16_t terminal_q = 1; // Positive odd integer.

    // The source controller used epsilon for all three meanings. They are separate here so
    // tuning one part of the reaching law does not silently change the other two.
    float switching_gain = 0.0f;
    float boundary_layer = 0.0f;
    float power_exponent = 1.0f;

    float saturation_limit = 1.0f;
    float output_limit = 0.0f;
    float position_deadband = 0.0f;
    float velocity_deadband = 0.0f;
    float position_integral_limit = 2000.0f;
    float velocity_integral_limit = 2000.0f;
};

struct smc_state
{
    float target = 0.0f;
    float target_rate = 0.0f;
    float target_acceleration = 0.0f;
    float position = 0.0f;
    float velocity = 0.0f;
    float position_error = 0.0f;
    float velocity_error = 0.0f;
    float position_integral = 0.0f;
    float velocity_integral = 0.0f;
    float surface = 0.0f;
    float output = 0.0f;
    bool initialized = false;
    bool valid = false;
};

class smc
{
public:
    smc() = default;

    explicit smc(const smc_config& config) noexcept
    {
        (void)configure(config);
    }

    bool configure(const smc_config& config) noexcept
    {
        if (!valid_config(config))
        {
            return false;
        }

        config_ = config;
        configured_ = true;
        reset();
        return true;
    }

    // Position modes use error = feedback - target, matching the source controller.
    // dt is the elapsed time in seconds.
    float update_position(float target, float position, float velocity, float dt) noexcept
    {
        if (!configured_ || config_.mode == smc_mode::velocity || !valid_sample(target, dt) ||
            !std::isfinite(position) || !std::isfinite(velocity))
        {
            return invalidate(true);
        }

        if (!update_target(target, dt))
        {
            return invalidate(true);
        }
        state_.position = position;
        state_.velocity = velocity;
        state_.position_error = position - target;
        state_.velocity_error = velocity - state_.target_rate;

        const bool settled =
            std::fabs(state_.position_error) <= config_.position_deadband &&
            std::fabs(state_.velocity_error) <= config_.velocity_deadband;
        if (settled &&
            (config_.mode != smc_mode::terminal || state_.target_acceleration == 0.0f))
        {
            return finish(0.0f, 0.0f);
        }
        if (config_.mode == smc_mode::error_integral)
        {
            state_.position_integral = math::limit_abs(
                state_.position_integral + state_.position_error * dt,
                config_.position_integral_limit);
        }

        switch (config_.mode)
        {
            case smc_mode::exponential:
                return calculate_exponential();
            case smc_mode::power:
                return calculate_power();
            case smc_mode::terminal:
                return calculate_terminal();
            case smc_mode::error_integral:
                return calculate_error_integral();
            case smc_mode::velocity:
                break;
        }
        return invalidate(true);
    }

    // Velocity mode treats target as velocity and target_rate as its acceleration.
    float update_velocity(float target, float velocity, float dt) noexcept
    {
        if (!configured_ || config_.mode != smc_mode::velocity || !valid_sample(target, dt) ||
            !std::isfinite(velocity))
        {
            return invalidate(true);
        }

        if (!update_target(target, dt))
        {
            return invalidate(true);
        }
        state_.velocity = velocity;
        state_.velocity_error = velocity - target;
        state_.velocity_integral = math::limit_abs(
            state_.velocity_integral + state_.velocity_error * dt,
            config_.velocity_integral_limit);

        state_.surface =
            state_.velocity_error + config_.surface_gain * state_.velocity_integral;
        const float output = config_.inertia *
                             (state_.target_rate - config_.surface_gain * state_.velocity_error -
                                 config_.reaching_gain * state_.surface -
                                 config_.switching_gain * saturation(state_.surface));
        return finish(state_.surface, output);
    }

    // Preserve the configuration while clearing all dynamic state.
    void reset() noexcept
    {
        state_ = {};
    }

    void reset_integrals() noexcept
    {
        state_.position_integral = 0.0f;
        state_.velocity_integral = 0.0f;
    }

    bool configured() const noexcept
    {
        return configured_;
    }

    const smc_config& configuration() const noexcept
    {
        return config_;
    }

    const smc_state& state() const noexcept
    {
        return state_;
    }

    float output() const noexcept
    {
        return state_.output;
    }

private:
    static bool finite(float value) noexcept
    {
        return std::isfinite(value);
    }

    static bool valid_mode(smc_mode mode) noexcept
    {
        switch (mode)
        {
            case smc_mode::exponential:
            case smc_mode::power:
            case smc_mode::terminal:
            case smc_mode::velocity:
            case smc_mode::error_integral:
                return true;
        }
        return false;
    }

    static bool valid_config(const smc_config& config) noexcept
    {
        const bool finite_values =
            finite(config.inertia) && finite(config.reaching_gain) &&
            finite(config.surface_gain) && finite(config.integral_gain) &&
            finite(config.terminal_gain) && finite(config.switching_gain) &&
            finite(config.boundary_layer) && finite(config.power_exponent) &&
            finite(config.saturation_limit) && finite(config.output_limit) &&
            finite(config.position_deadband) && finite(config.velocity_deadband) &&
            finite(config.position_integral_limit) && finite(config.velocity_integral_limit);
        if (!valid_mode(config.mode) || !finite_values || config.inertia <= 0.0f ||
            config.reaching_gain < 0.0f || config.surface_gain < 0.0f ||
            config.integral_gain < 0.0f || config.terminal_gain < 0.0f ||
            config.switching_gain < 0.0f || config.boundary_layer < 0.0f ||
            config.saturation_limit <= 0.0f || config.output_limit <= 0.0f ||
            config.position_deadband < 0.0f || config.velocity_deadband < 0.0f ||
            config.position_integral_limit < 0.0f || config.velocity_integral_limit < 0.0f)
        {
            return false;
        }
        if (config.mode == smc_mode::power &&
            (config.power_exponent <= 0.0f || config.power_exponent >= 1.0f))
        {
            return false;
        }
        if (config.mode == smc_mode::terminal &&
            (config.terminal_q == 0 || config.terminal_p <= config.terminal_q ||
                config.terminal_p % 2 == 0 || config.terminal_q % 2 == 0 ||
                config.terminal_gain <= 0.0f || config.position_deadband <= 0.0f))
        {
            return false;
        }
        return true;
    }

    static bool valid_sample(float target, float dt) noexcept
    {
        return std::isfinite(target) && std::isfinite(dt) && dt > 0.0f;
    }

    static float sign(float value) noexcept
    {
        if (value > 0.0f)
        {
            return 1.0f;
        }
        if (value < 0.0f)
        {
            return -1.0f;
        }
        return 0.0f;
    }

    bool update_target(float target, float dt) noexcept
    {
        if (!state_.initialized)
        {
            state_.target = target;
            state_.target_rate = 0.0f;
            state_.target_acceleration = 0.0f;
            state_.initialized = true;
            return true;
        }

        const float target_rate = (target - state_.target) / dt;
        const float target_acceleration = (target_rate - state_.target_rate) / dt;
        if (!std::isfinite(target_rate) || !std::isfinite(target_acceleration))
        {
            return false;
        }

        state_.target = target;
        state_.target_rate = target_rate;
        state_.target_acceleration = target_acceleration;
        return true;
    }

    float saturation(float surface) const noexcept
    {
        if (config_.boundary_layer == 0.0f)
        {
            return config_.saturation_limit * sign(surface);
        }

        const float scaled = surface / config_.boundary_layer;
        return math::clamp(
            scaled, -config_.saturation_limit, config_.saturation_limit);
    }

    float calculate_exponential() noexcept
    {
        const float surface =
            config_.surface_gain * state_.position_error + state_.velocity_error;
        const float output = config_.inertia *
                             (-config_.surface_gain * state_.velocity_error -
                                 config_.reaching_gain * surface -
                                 config_.switching_gain * saturation(surface));
        return finish(surface, output);
    }

    float calculate_power() noexcept
    {
        const float surface =
            config_.surface_gain * state_.position_error + state_.velocity_error;
        const float output =
            config_.inertia *
            (-config_.surface_gain * state_.velocity_error -
                config_.reaching_gain * surface -
                config_.reaching_gain * std::pow(std::fabs(surface), config_.power_exponent) *
                    saturation(surface));
        return finish(surface, output);
    }

    float calculate_terminal() noexcept
    {
        const float p = static_cast<float>(config_.terminal_p);
        const float q = static_cast<float>(config_.terminal_q);
        const float absolute_error = std::fabs(state_.position_error);
        const float position_power =
            sign(state_.position_error) * std::pow(absolute_error, q / p);
        const float regularized_error =
            absolute_error < config_.position_deadband ? config_.position_deadband : absolute_error;
        const float error_ratio = std::pow(regularized_error, q / p - 1.0f);
        const float surface =
            config_.terminal_gain * position_power + state_.velocity_error;
        const float output =
            config_.inertia *
            (state_.target_acceleration - config_.reaching_gain * surface -
                config_.switching_gain * saturation(surface) -
                state_.velocity_error * (q * config_.terminal_gain / p) * error_ratio);
        return finish(surface, output);
    }

    float calculate_error_integral() noexcept
    {
        const float surface = config_.surface_gain * state_.position_error +
                              state_.velocity_error +
                              config_.integral_gain * state_.position_integral;
        const float output = config_.inertia *
                             (-config_.surface_gain * state_.velocity_error -
                                 config_.integral_gain * state_.position_error -
                                 config_.reaching_gain * surface -
                                 config_.switching_gain * saturation(surface));
        return finish(surface, output);
    }

    float finish(float surface, float output) noexcept
    {
        if (!std::isfinite(surface) || !std::isfinite(output))
        {
            return invalidate();
        }

        state_.surface = surface;
        state_.output = math::limit_abs(output, config_.output_limit);
        state_.valid = true;
        return state_.output;
    }

    float invalidate(bool reset_target = false) noexcept
    {
        if (reset_target)
        {
            state_.initialized = false;
            state_.target_rate = 0.0f;
            state_.target_acceleration = 0.0f;
        }
        state_.surface = 0.0f;
        state_.output = 0.0f;
        state_.valid = false;
        return 0.0f;
    }

    smc_config config_{};
    smc_state state_{};
    bool configured_ = false;
};

} // namespace control

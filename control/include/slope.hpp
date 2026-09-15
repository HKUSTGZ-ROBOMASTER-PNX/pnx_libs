#pragma once

#include <cmath>

namespace control
{

// Call-count based slew limiter. One update advances at most one configured step.
class slope
{
public:
    constexpr slope() = default;
    constexpr slope(float value, float step)
        : value_(value), up_step_(step), down_step_(step)
    {
    }

    float update(float target) noexcept
    {
        const float delta = target - value_;
        if ((delta >= 0.0f && delta < up_step_) ||
            (delta < 0.0f && delta > -down_step_))
        {
            value_ = target;
            reached_ = true;
        }
        else
        {
            reached_ = false;
            value_ += delta < 0.0f ? -down_step_ : up_step_;
        }
        return value_;
    }

    void set_value(float value) noexcept { value_ = value; }
    void set_step(float step) noexcept { up_step_ = down_step_ = std::fabs(step); }
    void set_up_step(float step) noexcept { up_step_ = std::fabs(step); }
    void set_down_step(float step) noexcept { down_step_ = std::fabs(step); }

    [[nodiscard]] float value() const noexcept { return value_; }
    [[nodiscard]] bool reached() const noexcept { return reached_; }

private:
    float value_ = 0.0f;
    float up_step_ = 0.0f;
    float down_step_ = 0.0f;
    bool reached_ = false;
};

} // namespace control

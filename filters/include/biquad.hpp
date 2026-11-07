#pragma once

namespace filter
{
// Direct-form I state; a1/a2 are signed feedback coefficients.
struct biquad_state
{
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;

    float update(float input, float b0, float b1, float b2, float a1, float a2)
    {
        const float output = b0 * input + b1 * x1 + b2 * x2 + a1 * y1 + a2 * y2;
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        return output;
    }
};

struct biquad
{
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    biquad_state state{};

    float update(float input)
    {
        return state.update(input, b0, b1, b2, a1, a2);
    }

    void reset()
    {
        state = {};
    }
};
} // namespace filter

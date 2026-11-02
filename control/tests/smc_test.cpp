#include "smc.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace
{

void require(bool value, const char* message)
{
    if (!value)
    {
        std::fprintf(stderr, "SMC test failed: %s\n", message);
        std::exit(EXIT_FAILURE);
    }
}

bool near(float actual, float expected, float tolerance = 0.0001f)
{
    return std::fabs(actual - expected) <= tolerance * (1.0f + std::fabs(expected));
}

control::smc_config config(control::smc_mode mode)
{
    control::smc_config result;
    result.mode = mode;
    result.inertia = 2.0f;
    result.reaching_gain = 3.0f;
    result.surface_gain = 4.0f;
    result.integral_gain = 2.0f;
    result.terminal_gain = 2.0f;
    result.terminal_p = 3;
    result.terminal_q = 1;
    result.switching_gain = 5.0f;
    result.boundary_layer = 10.0f;
    result.power_exponent = 0.5f;
    result.saturation_limit = 1.0f;
    result.output_limit = 1000.0f;
    result.position_deadband = mode == control::smc_mode::terminal ? 0.001f : 0.0f;
    result.velocity_deadband = 0.0f;
    return result;
}

void test_exponential_formula()
{
    control::smc controller{config(control::smc_mode::exponential)};
    require(controller.configured(), "exponential configuration accepted");

    const float output = controller.update_position(0.0f, 1.0f, 2.0f, 0.5f);
    require(near(controller.state().surface, 6.0f), "exponential surface formula");
    require(near(output, -58.0f), "exponential output formula");
}

void test_power_formula()
{
    control::smc controller{config(control::smc_mode::power)};
    require(controller.configured(), "power configuration accepted");

    const float output = controller.update_position(0.0f, 0.5f, 2.0f, 0.5f);
    require(near(controller.state().surface, 4.0f), "power surface formula");
    require(near(output, -44.8f), "power output formula");
}

void test_terminal_formula()
{
    control::smc controller{config(control::smc_mode::terminal)};
    require(controller.configured(), "terminal configuration accepted");

    const float output = controller.update_position(0.0f, 8.0f, 1.0f, 0.5f);
    require(near(controller.state().surface, 5.0f), "terminal surface formula");
    require(near(output, -35.333333f), "terminal output formula");
}

void test_velocity_formula()
{
    control::smc controller{config(control::smc_mode::velocity)};
    require(controller.configured(), "velocity configuration accepted");

    (void)controller.update_velocity(0.0f, 0.0f, 0.5f);
    const float output = controller.update_velocity(1.0f, 3.0f, 0.5f);
    require(near(controller.state().target_rate, 2.0f), "velocity target derivative");
    require(near(controller.state().velocity_integral, 1.0f), "velocity error integral");
    require(near(controller.state().surface, 6.0f), "velocity surface formula");
    require(near(output, -54.0f), "velocity output formula");
}

void test_error_integral_formula()
{
    control::smc controller{config(control::smc_mode::error_integral)};
    require(controller.configured(), "error-integral configuration accepted");

    const float output = controller.update_position(0.0f, 1.0f, 2.0f, 0.5f);
    require(near(controller.state().position_integral, 0.5f),
        "position error integral");
    require(near(controller.state().surface, 7.0f), "error-integral surface formula");
    require(near(output, -69.0f), "error-integral output formula");
}

void test_configuration_rejection()
{
    control::smc controller;
    require(!controller.configure({}), "zero default configuration rejected");
    require(!controller.configured(), "rejected initial configuration stays unconfigured");

    const auto valid = config(control::smc_mode::exponential);
    require(controller.configure(valid), "valid configuration accepted");

    auto invalid = valid;
    invalid.mode = static_cast<control::smc_mode>(0xff);
    require(!controller.configure(invalid), "unknown mode rejected");

    invalid = valid;
    invalid.inertia = std::numeric_limits<float>::quiet_NaN();
    require(!controller.configure(invalid), "NaN configuration rejected");

    invalid = config(control::smc_mode::power);
    invalid.power_exponent = 1.0f;
    require(!controller.configure(invalid), "invalid power exponent rejected");

    invalid = config(control::smc_mode::terminal);
    invalid.terminal_p = 2;
    require(!controller.configure(invalid), "even terminal exponent rejected");

    invalid = config(control::smc_mode::terminal);
    invalid.position_deadband = 0.0f;
    require(!controller.configure(invalid), "terminal mode without regularization rejected");

    require(controller.configured() &&
                controller.configuration().mode == control::smc_mode::exponential,
        "configuration failure preserves the accepted configuration");
}

void test_output_limit()
{
    auto limited = config(control::smc_mode::exponential);
    limited.output_limit = 10.0f;
    control::smc controller{limited};

    require(near(controller.update_position(0.0f, 1.0f, 2.0f, 0.5f), -10.0f),
        "negative output limit");
    controller.reset();
    require(near(controller.update_position(0.0f, -1.0f, -2.0f, 0.5f), 10.0f),
        "positive output limit");
}

void test_saturation_limit()
{
    auto continuous = config(control::smc_mode::exponential);
    continuous.inertia = 1.0f;
    continuous.reaching_gain = 0.0f;
    continuous.surface_gain = 0.0f;
    continuous.switching_gain = 1.0f;
    continuous.boundary_layer = 2.0f;
    continuous.saturation_limit = 0.5f;
    control::smc controller{continuous};

    require(near(controller.update_position(0.0f, 0.0f, 0.9f, 0.1f), -0.45f),
        "boundary layer remains linear below the saturation limit");
    require(near(controller.update_position(0.0f, 0.0f, 1.1f, 0.1f), -0.5f),
        "boundary layer clamps continuously at the configured limit");

    auto discontinuous = continuous;
    discontinuous.boundary_layer = 0.0f;
    require(controller.configure(discontinuous), "zero-width boundary layer accepted");
    require(near(controller.update_position(0.0f, 0.0f, 0.1f, 0.1f), -0.5f),
        "zero-width boundary layer uses the configured sign amplitude");
}

void test_position_and_velocity_deadband()
{
    auto deadband = config(control::smc_mode::exponential);
    deadband.position_deadband = 0.1f;
    deadband.velocity_deadband = 0.2f;
    control::smc controller{deadband};

    require(near(controller.update_position(0.0f, 0.05f, 0.1f, 0.1f), 0.0f),
        "both errors inside deadband produce zero output");
    require(controller.state().valid && near(controller.state().surface, 0.0f),
        "deadband result is a valid zero output");

    controller.reset();
    require(!near(controller.update_position(0.0f, 0.05f, 0.3f, 0.1f), 0.0f),
        "velocity outside deadband still controls");

    controller.reset();
    require(!near(controller.update_position(0.0f, 0.2f, 0.1f, 0.1f), 0.0f),
        "position outside deadband still controls");
}

void test_terminal_zero_error_with_velocity()
{
    auto terminal = config(control::smc_mode::terminal);
    terminal.position_deadband = 0.1f;
    terminal.velocity_deadband = 0.1f;
    control::smc controller{terminal};

    const float output = controller.update_position(0.0f, 0.0f, 1.0f, 0.1f);
    require(controller.state().valid && std::isfinite(output),
        "terminal zero-position-error result is finite");
    require(near(controller.state().surface, 1.0f),
        "terminal zero-position-error surface retains velocity error");
    require(output < 0.0f && !near(output, 0.0f),
        "terminal zero-position-error case still brakes nonzero velocity");
}

void test_terminal_feedforward_inside_deadband()
{
    auto terminal = config(control::smc_mode::terminal);
    terminal.position_deadband = 0.1f;
    terminal.velocity_deadband = 0.1f;
    control::smc controller{terminal};

    (void)controller.update_position(0.0f, 0.0f, 0.0f, 1.0f);
    const float output = controller.update_position(1.0f, 1.0f, 1.0f, 1.0f);
    require(near(controller.state().target_acceleration, 1.0f),
        "terminal target acceleration established inside deadband");
    require(near(output, 2.0f),
        "terminal deadband preserves target-acceleration feedforward");
}

void test_invalid_sample_resets_target_difference()
{
    control::smc controller{config(control::smc_mode::velocity)};
    (void)controller.update_velocity(0.0f, 0.0f, 0.1f);
    (void)controller.update_velocity(1.0f, 0.0f, 0.1f);
    require(near(controller.state().target_rate, 10.0f), "target difference established");

    const float nan = std::numeric_limits<float>::quiet_NaN();
    require(near(controller.update_velocity(2.0f, nan, 0.1f), 0.0f),
        "invalid velocity produces zero output");
    require(!controller.state().valid && !controller.state().initialized,
        "invalid velocity resets target history");

    (void)controller.update_velocity(3.0f, 0.0f, 0.1f);
    require(controller.state().valid && near(controller.state().target_rate, 0.0f) &&
                near(controller.state().target_acceleration, 0.0f),
        "first sample after invalid input reseeds target derivatives");
}

void test_nan_and_zero_dt()
{
    control::smc controller{config(control::smc_mode::exponential)};
    (void)controller.update_position(0.0f, 1.0f, 0.0f, 0.1f);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    require(near(controller.update_position(nan, 0.0f, 0.0f, 0.1f), 0.0f) &&
                !controller.state().valid && !controller.state().initialized,
        "NaN target invalidates output and target history");

    (void)controller.update_position(0.0f, 0.0f, 0.0f, 0.1f);
    require(near(controller.update_position(0.0f, 0.0f, 0.0f, 0.0f), 0.0f) &&
                !controller.state().valid && !controller.state().initialized,
        "zero dt invalidates output and target history");
}

} // namespace

int main()
{
    test_exponential_formula();
    test_power_formula();
    test_terminal_formula();
    test_velocity_formula();
    test_error_integral_formula();
    test_configuration_rejection();
    test_output_limit();
    test_saturation_limit();
    test_position_and_velocity_deadband();
    test_terminal_zero_error_with_velocity();
    test_terminal_feedforward_inside_deadband();
    test_invalid_sample_resets_target_difference();
    test_nan_and_zero_dt();
    std::puts("SMC formula, validation and safety tests passed");
}

#include <gtest/gtest.h>

#include "../include/powertrain_system.h"
#include "../include/powertrain/powertrain_unit.h"
#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <cmath>

namespace {
    struct DriverRig {
        PowertrainSystem system;
        powertrain::PowertrainUnit unit;

        void build(double pedalTimeConstant) {
            PowertrainSystem::Parameters params;
            params.pedalTimeConstant = pedalTimeConstant;
            params.clutchTimeConstant = 0.0;

            unit.initialize(
                powertrain::EngineControlUnit::Parameters(),
                powertrain::TransmissionControlUnit::Parameters());

            system.initialize(params);
            system.setController(&unit);
        }

        double settle(double dt, double seconds) {
            system.getDriverInputs().accelerator = 0.0;
            system.conditionInputs(dt);

            system.getDriverInputs().accelerator = 1.0;

            const int steps = static_cast<int>(std::round(seconds / dt));
            for (int i = 0; i < steps; ++i) system.conditionInputs(dt);

            return system.getConditionedInputs().accelerator;
        }
    };
}

TEST(DriverModelTests, TheStepResponseFollowsElapsedTimeNotStepCount) {
    const auto after = [](double dt) {
        DriverRig rig;
        rig.build(1.0 / 60.0);

        return rig.settle(dt, 0.05);
    };

    const double half = after(5e-4);
    const double one = after(1e-3);
    const double two = after(2e-3);

    const double continuous = 1.0 - std::exp(-0.05 / (1.0 / 60.0));

    EXPECT_LT(std::abs(half - two), 0.01);
    EXPECT_NEAR(half, continuous, 0.01);
    EXPECT_NEAR(one, continuous, 0.01);
    EXPECT_NEAR(two, continuous, 0.01);
}

TEST(DriverModelTests, TheSameElapsedTimeGivesTheSamePedalAtAnyControlRate) {
    const auto after = [](double dt, double seconds) {
        DriverRig rig;
        rig.build(0.05);

        return rig.settle(dt, seconds);
    };

    EXPECT_NEAR(after(1e-3, 0.1), after(2.5e-4, 0.1), 5e-3);
    EXPECT_NEAR(after(1e-3, 0.2), after(2.5e-4, 0.2), 5e-3);
}

TEST(DriverModelTests, AZeroTimeConstantPassesThePedalStraightThrough) {
    DriverRig rig;
    rig.build(0.0);

    rig.system.getDriverInputs().accelerator = 0.0;
    rig.system.conditionInputs(1e-3);

    rig.system.getDriverInputs().accelerator = 0.9;
    rig.system.conditionInputs(1e-3);

    EXPECT_NEAR(rig.system.getConditionedInputs().accelerator, 0.9, 1e-12);
}

TEST(DriverModelTests, ThePedalApproachesTheCommandMonotonically) {
    DriverRig rig;
    rig.build(0.05);

    rig.system.getDriverInputs().accelerator = 0.0;
    rig.system.conditionInputs(1e-3);
    rig.system.getDriverInputs().accelerator = 1.0;

    double previous = 0.0;
    for (int i = 0; i < 2000; ++i) {
        rig.system.conditionInputs(1e-3);
        const double now = rig.system.getConditionedInputs().accelerator;
        ASSERT_GE(now, previous - 1e-12);
        ASSERT_LE(now, 1.0 + 1e-12);
        previous = now;
    }

    EXPECT_GT(previous, 0.99);
}

TEST(DriverModelTests, TheOtherInputsPassThroughUntouched) {
    DriverRig rig;
    rig.build(0.05);

    rig.system.getDriverInputs().brake = 0.7;
    rig.system.getDriverInputs().gatePosition = 3;
    rig.system.getDriverInputs().manualMode = true;
    rig.system.conditionInputs(1e-3);
    rig.system.conditionInputs(1e-3);

    EXPECT_NEAR(rig.system.getConditionedInputs().brake, 0.7, 1e-12);
    EXPECT_EQ(rig.system.getConditionedInputs().gatePosition, 3);
    EXPECT_TRUE(rig.system.getConditionedInputs().manualMode);
}

TEST(DriverModelTests, TheConstantsAreReachableThroughTheRegistry) {
    config::ParameterRegistry registry;

    PowertrainSystem system;
    system.initialize(PowertrainSystem::Parameters());
    system.registerParameters(&registry);

    ASSERT_TRUE(registry.contains("driver.pedal_time_constant"));
    ASSERT_TRUE(registry.contains("driver.clutch_time_constant"));
    ASSERT_TRUE(registry.contains("driver.clutch_pedal_rate"));
    ASSERT_TRUE(registry.contains("throttle.open_rate"));

    ASSERT_TRUE(registry.set("driver.pedal_time_constant", 0.2));
    EXPECT_NEAR(system.getParameters().pedalTimeConstant, 0.2, 1e-12);
}

TEST(DriverModelTests, TheDefaultReproducesTheOldFrameFilterAtSixtyHertz) {
    DriverRig rig;
    rig.build(PowertrainSystem::Parameters().pedalTimeConstant);

    const double afterOneFrame = rig.settle(1e-4, 1.0 / 60.0);

    EXPECT_NEAR(afterOneFrame, 0.5, 5e-3);
}

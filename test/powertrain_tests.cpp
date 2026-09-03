#include <gtest/gtest.h>

#include "../include/powertrain/passthrough_controller.h"
#include "../include/external_throttle.h"
#include "../include/direct_throttle_linkage.h"
#include "../include/engine.h"
#include "../include/config/parameter_registry.h"

#include <cmath>

TEST(PassthroughControllerTests, ReproducesTheDirectThrottleLinkage) {
    const double gammas[] = { 0.5, 1.0, 2.0, 3.7 };
    const double pedals[] = { 0.0, 0.1, 0.25, 0.5, 0.75, 0.99, 1.0 };

    for (double gamma : gammas) {
        Engine engine;

        DirectThrottleLinkage::Parameters linkageParams;
        linkageParams.gamma = gamma;

        DirectThrottleLinkage linkage;
        linkage.initialize(linkageParams);

        powertrain::PassthroughController::Parameters controllerParams;
        controllerParams.throttleGamma = gamma;

        powertrain::PassthroughController controller;
        controller.initialize(controllerParams);

        ExternalThrottle external;

        for (double pedal : pedals) {
            linkage.setSpeedControl(pedal);
            linkage.update(1e-3, &engine);
            const double reference = engine.getThrottle();

            powertrain::PowertrainState state;
            powertrain::DriverInputs inputs;
            powertrain::ActuatorCommands commands;
            inputs.accelerator = pedal;

            controller.update(1e-3, state, inputs, &commands);
            external.setPlatePosition(commands.throttlePlate);
            external.update(1e-3, &engine);

            EXPECT_NEAR(engine.getThrottle(), reference, 1e-12)
                << "gamma=" << gamma << " pedal=" << pedal;
        }
    }
}

TEST(ExternalThrottleTests, PlateOneIsWideOpen) {
    Engine engine;
    ExternalThrottle throttle;

    throttle.setPlatePosition(1.0);
    throttle.update(1e-3, &engine);
    EXPECT_NEAR(engine.getThrottle(), 0.0, 1e-12);

    throttle.setPlatePosition(0.0);
    throttle.update(1e-3, &engine);
    EXPECT_NEAR(engine.getThrottle(), 1.0, 1e-12);
}

TEST(ExternalThrottleTests, PlatePositionIsClamped) {
    ExternalThrottle throttle;

    throttle.setPlatePosition(5.0);
    EXPECT_NEAR(throttle.getPlatePosition(), 1.0, 1e-12);

    throttle.setPlatePosition(-5.0);
    EXPECT_NEAR(throttle.getPlatePosition(), 0.0, 1e-12);
}

TEST(PassthroughControllerTests, ShiftRequestsAreEdgeTriggered) {
    powertrain::PassthroughController controller;
    controller.initialize(powertrain::PassthroughController::Parameters());

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    state.gear = 0;
    state.gearCount = 6;
    inputs.shiftUpRequest = true;

    controller.update(1e-3, state, inputs, &commands);
    EXPECT_EQ(commands.targetGear, 1);

    state.gear = 1;
    controller.update(1e-3, state, inputs, &commands);
    EXPECT_EQ(commands.targetGear, 1);

    inputs.shiftUpRequest = false;
    controller.update(1e-3, state, inputs, &commands);
    EXPECT_EQ(commands.targetGear, 1);

    inputs.shiftUpRequest = true;
    controller.update(1e-3, state, inputs, &commands);
    EXPECT_EQ(commands.targetGear, 2);
}

TEST(PassthroughControllerTests, GearRequestsStayInRange) {
    powertrain::PassthroughController controller;
    controller.initialize(powertrain::PassthroughController::Parameters());

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    state.gearCount = 3;
    state.gear = 2;
    inputs.shiftUpRequest = true;
    controller.update(1e-3, state, inputs, &commands);
    EXPECT_EQ(commands.targetGear, 2);

    controller.reset();
    state.gear = -1;
    inputs.shiftUpRequest = false;
    inputs.shiftDownRequest = true;
    controller.update(1e-3, state, inputs, &commands);
    EXPECT_EQ(commands.targetGear, -1);
}

TEST(PassthroughControllerTests, ClutchPedalInvertsToClutchPressure) {
    powertrain::PassthroughController controller;
    controller.initialize(powertrain::PassthroughController::Parameters());

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.clutchPedal = 0.0;
    controller.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.clutchPressure[0], 1.0, 1e-12);

    inputs.clutchPedal = 1.0;
    controller.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.clutchPressure[0], 0.0, 1e-12);
}

TEST(PassthroughControllerTests, ExposesItsGammaThroughTheRegistry) {
    config::ParameterRegistry registry;

    powertrain::PassthroughController::Parameters params;
    params.throttleGamma = 2.0;

    powertrain::PassthroughController controller;
    controller.initialize(params);
    controller.registerParameters(&registry, "");

    ASSERT_TRUE(registry.contains("passthrough.throttle_gamma"));

    ASSERT_TRUE(registry.set("passthrough.throttle_gamma", 1.0));

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;
    inputs.accelerator = 0.5;

    controller.update(1e-3, state, inputs, &commands);

    EXPECT_NEAR(commands.throttlePlate, 0.5, 1e-12);
}

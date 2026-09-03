#include <gtest/gtest.h>

#include "../include/powertrain/engine_control_unit.h"
#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <cmath>

namespace {
    powertrain::EngineControlUnit::Parameters ecuParameters() {
        powertrain::EngineControlUnit::Parameters params;
        params.referenceTorque = units::torque(200.0, units::Nm);
        params.revLimit = units::rpm(7000.0);
        params.softLimitBand = units::rpm(300.0);
        params.idleSpeedWarm = units::rpm(800.0);
        params.idleSpeedCold = units::rpm(1300.0);

        return params;
    }

    powertrain::PowertrainState warmIdleState() {
        powertrain::PowertrainState state;
        state.coolantTemperature = units::celcius(90.0);
        state.oilTemperature = units::celcius(90.0);
        state.engineSpeed = units::rpm(800.0);
        state.engineRpm = 800.0;
        state.engineRunning = true;
        state.gear = -1;

        return state;
    }
}

TEST(EngineControlUnitTests, IdleTargetFallsAsTheEngineWarmsUp) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    const double cold = ecu.idleSpeedAt(units::celcius(-10.0));
    const double lukewarm = ecu.idleSpeedAt(units::celcius(35.0));
    const double warm = ecu.idleSpeedAt(units::celcius(90.0));

    EXPECT_GT(cold, lukewarm);
    EXPECT_GT(lukewarm, warm);
    EXPECT_NEAR(cold, units::rpm(1300.0), 1e-9);
    EXPECT_NEAR(warm, units::rpm(800.0), 1e-9);
}

TEST(EngineControlUnitTests, IdleTargetIsMonotonicInTemperature) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    double previous = ecu.idleSpeedAt(units::celcius(-40.0));
    for (double t = -40.0; t <= 120.0; t += 2.0) {
        const double current = ecu.idleSpeedAt(units::celcius(t));
        EXPECT_LE(current, previous + 1e-12) << "t=" << t;
        previous = current;
    }
}

TEST(EngineControlUnitTests, PedalScalesTheTorqueRequestAgainstWhatIsAvailable) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    powertrain::PowertrainState state = warmIdleState();
    state.engineSpeed = units::rpm(3000.0);

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 1.0;
    ecu.update(1e-3, state, inputs, &commands);
    const double full = ecu.getDriverTorqueRequest();

    inputs.accelerator = 0.5;
    ecu.update(1e-3, state, inputs, &commands);
    const double half = ecu.getDriverTorqueRequest();

    EXPECT_NEAR(full, ecu.maxTorqueAt(state.engineSpeed), 1e-9);
    EXPECT_NEAR(half, 0.5 * full, 1e-9);
}

TEST(EngineControlUnitTests, ColdEngineCapsTheDriverRequest) {
    powertrain::EngineControlUnit::Parameters params = ecuParameters();
    params.coldStartTorqueCap = 0.5;

    powertrain::EngineControlUnit ecu;
    ecu.initialize(params);

    powertrain::DriverInputs inputs;
    inputs.accelerator = 1.0;
    powertrain::ActuatorCommands commands;

    powertrain::PowertrainState cold = warmIdleState();
    cold.coolantTemperature = params.coldTemperature;
    cold.engineSpeed = units::rpm(3000.0);

    ecu.update(1e-3, cold, inputs, &commands);
    const double coldRequest = ecu.getDriverTorqueRequest();

    powertrain::PowertrainState warm = cold;
    warm.coolantTemperature = params.warmTemperature;

    ecu.reset();
    ecu.update(1e-3, warm, inputs, &commands);
    const double warmRequest = ecu.getDriverTorqueRequest();

    EXPECT_NEAR(coldRequest, 0.5 * warmRequest, 1e-9);
}

TEST(EngineControlUnitTests, ColdEngineEnrichesAndRetardsTiming) {
    powertrain::EngineControlUnit::Parameters params = ecuParameters();
    params.coldStartEnrichment = 1.6;
    params.coldStartTimingRetard = units::angle(8.0, units::deg);

    powertrain::EngineControlUnit ecu;
    ecu.initialize(params);

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    powertrain::PowertrainState cold = warmIdleState();
    cold.coolantTemperature = params.coldTemperature;
    ecu.update(1e-3, cold, inputs, &commands);

    EXPECT_NEAR(commands.fuelEnrichment, 1.6, 1e-9);
    EXPECT_NEAR(commands.timingOffset, -units::angle(8.0, units::deg), 1e-9);

    powertrain::PowertrainState warm = warmIdleState();
    ecu.update(1e-3, warm, inputs, &commands);

    EXPECT_NEAR(commands.fuelEnrichment, 1.0, 1e-9);
    EXPECT_NEAR(commands.timingOffset, 0.0, 1e-9);
}

TEST(EngineControlUnitTests, SoftLimiterRampsIgnitionCutBeforeTheLimit) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    powertrain::PowertrainState state = warmIdleState();
    powertrain::DriverInputs inputs;
    inputs.accelerator = 1.0;
    powertrain::ActuatorCommands commands;

    state.engineSpeed = units::rpm(6600.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.ignitionCutFraction, 0.0, 1e-9);

    state.engineSpeed = units::rpm(6850.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.ignitionCutFraction, 0.5, 1e-6);

    state.engineSpeed = units::rpm(7000.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.ignitionCutFraction, 1.0, 1e-9);
}

TEST(EngineControlUnitTests, HardLimiterCutsFuelAboveTheLimit) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    powertrain::PowertrainState state = warmIdleState();
    powertrain::DriverInputs inputs;
    inputs.accelerator = 1.0;
    powertrain::ActuatorCommands commands;

    state.engineSpeed = units::rpm(7000.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.fuelCutFraction, 0.0, 1e-9);

    state.engineSpeed = units::rpm(7400.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.fuelCutFraction, 1.0, 1e-9);
}

TEST(EngineControlUnitTests, OverrunFuelCutHasHysteresis) {
    powertrain::EngineControlUnit::Parameters params = ecuParameters();
    params.overrunCutSpeed = units::rpm(2000.0);
    params.overrunResumeSpeed = units::rpm(1400.0);

    powertrain::EngineControlUnit ecu;
    ecu.initialize(params);

    powertrain::PowertrainState state = warmIdleState();
    state.gear = 2;

    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.0;
    powertrain::ActuatorCommands commands;

    state.engineSpeed = units::rpm(2500.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.fuelCutFraction, 1.0, 1e-9);

    state.engineSpeed = units::rpm(1700.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.fuelCutFraction, 1.0, 1e-9);

    state.engineSpeed = units::rpm(1300.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.fuelCutFraction, 0.0, 1e-9);
}

TEST(EngineControlUnitTests, OverrunCutReleasesWhenThePedalIsTouched) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    powertrain::PowertrainState state = warmIdleState();
    state.gear = 3;
    state.engineSpeed = units::rpm(3000.0);

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 0.0;
    ecu.update(1e-3, state, inputs, &commands);
    ASSERT_NEAR(commands.fuelCutFraction, 1.0, 1e-9);

    inputs.accelerator = 0.3;
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.fuelCutFraction, 0.0, 1e-9);
}

TEST(EngineControlUnitTests, NoOverrunCutInNeutral) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    powertrain::PowertrainState state = warmIdleState();
    state.gear = -1;
    state.engineSpeed = units::rpm(3000.0);

    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.0;
    powertrain::ActuatorCommands commands;

    ecu.update(1e-3, state, inputs, &commands);

    EXPECT_NEAR(commands.fuelCutFraction, 0.0, 1e-9);
}

TEST(EngineControlUnitTests, IdleGovernorHoldsTheTargetAgainstALoad) {
    powertrain::EngineControlUnit::Parameters params = ecuParameters();
    params.idleController.kp = 0.01;
    params.idleController.ki = 0.08;

    powertrain::EngineControlUnit ecu;
    ecu.initialize(params);

    powertrain::PowertrainState state = warmIdleState();
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    const double target = units::rpm(800.0);
    const double inertia = 0.35;
    const double load = units::torque(18.0, units::Nm);
    const double dt = 1e-3;

    state.engineSpeed = units::rpm(600.0);

    for (int i = 0; i < 400000; ++i) {
        ecu.update(dt, state, inputs, &commands);

        const double delivered =
            commands.throttlePlate * ecu.maxTorqueAt(state.engineSpeed);
        state.indicatedTorque = delivered;
        state.engineSpeed += ((delivered - load) / inertia) * dt;
        state.engineSpeed = std::max(state.engineSpeed, units::rpm(100.0));
    }

    EXPECT_NEAR(units::toRpm(state.engineSpeed), units::toRpm(target), 25.0);
}

TEST(EngineControlUnitTests, TransmissionTorqueReductionLowersTheRequest) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    powertrain::PowertrainState state = warmIdleState();
    state.engineSpeed = units::rpm(3000.0);

    powertrain::DriverInputs inputs;
    inputs.accelerator = 1.0;
    powertrain::ActuatorCommands commands;

    for (int i = 0; i < 2000; ++i) ecu.update(1e-3, state, inputs, &commands);
    const double unreduced = ecu.getTorqueRequest();

    powertrain::PowertrainBus request;
    request.torqueReductionRequest = 0.6;
    request.interventionType = powertrain::TorqueIntervention::Spark;
    ecu.setTransmissionRequests(request);

    for (int i = 0; i < 2000; ++i) ecu.update(1e-3, state, inputs, &commands);
    const double reduced = ecu.getTorqueRequest();

    EXPECT_LT(reduced, unreduced * 0.5);
    EXPECT_NEAR(commands.ignitionCutFraction, 0.6, 1e-9);
}

TEST(EngineControlUnitTests, TransmissionSpeedRequestRaisesTheIdleTarget) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    powertrain::PowertrainState state = warmIdleState();
    state.engineSpeed = units::rpm(1500.0);

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    ecu.update(1e-3, state, inputs, &commands);
    const double withoutBlip = commands.throttlePlate;

    powertrain::PowertrainBus request;
    request.speedRequestActive = true;
    request.speedRequest = units::rpm(3000.0);
    ecu.setTransmissionRequests(request);

    ecu.update(1e-3, state, inputs, &commands);

    EXPECT_GT(commands.throttlePlate, withoutBlip);
}

TEST(EngineControlUnitTests, IgnitionKeyOffShutsTheEngineDown) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    powertrain::PowertrainState state = warmIdleState();
    powertrain::DriverInputs inputs;
    inputs.ignitionKey = false;
    inputs.accelerator = 1.0;
    powertrain::ActuatorCommands commands;

    ecu.update(1e-3, state, inputs, &commands);

    EXPECT_FALSE(commands.ignitionEnabled);
    EXPECT_NEAR(commands.throttlePlate, 0.0, 1e-12);
    EXPECT_NEAR(commands.fuelCutFraction, 1.0, 1e-12);
    EXPECT_EQ(ecu.getEngineState(), powertrain::EngineState::Off);
}

TEST(EngineControlUnitTests, StarterRunsOnlyBelowCrankingSpeed) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    powertrain::PowertrainState state = warmIdleState();
    powertrain::DriverInputs inputs;
    inputs.starterRequest = true;
    powertrain::ActuatorCommands commands;

    state.engineSpeed = units::rpm(0.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_TRUE(commands.starterEnabled);
    EXPECT_EQ(ecu.getEngineState(), powertrain::EngineState::Cranking);

    state.engineSpeed = units::rpm(800.0);
    ecu.update(1e-3, state, inputs, &commands);
    EXPECT_FALSE(commands.starterEnabled);
}

TEST(EngineControlUnitTests, ThrottleMapInvertsTheTorqueCurve) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());

    const double speed = units::rpm(3000.0);
    const double available = ecu.maxTorqueAt(speed);

    EXPECT_NEAR(ecu.getThrottleMap().sample(speed, 0.0), 0.0, 1e-9);
    EXPECT_NEAR(ecu.getThrottleMap().sample(speed, available), 1.0, 0.05);
}

TEST(EngineControlUnitTests, GainsAreReachableThroughTheRegistry) {
    config::ParameterRegistry registry;

    powertrain::EngineControlUnit ecu;
    ecu.initialize(ecuParameters());
    ecu.registerParameters(&registry, "");

    ASSERT_TRUE(registry.contains("ecu.idle.pid.kp"));
    ASSERT_TRUE(registry.contains("ecu.limiter.rev_limit"));
    ASSERT_TRUE(registry.contains("ecu.throttle_map"));
    EXPECT_TRUE(registry.isAdaptive("ecu.throttle_map"));

    ASSERT_TRUE(registry.set("ecu.limiter.rev_limit", units::rpm(5000.0)));
    EXPECT_NEAR(ecu.getParameters().revLimit, units::rpm(5000.0), 1e-9);
}

#include <gtest/gtest.h>

#include "../include/powertrain/transmission_control_unit.h"
#include "../include/powertrain/selector_gate.h"
#include "../include/transmission.h"
#include "../include/vehicle.h"
#include "../include/units.h"

#include <cmath>

namespace {
    powertrain::TransmissionControlUnit::Parameters rangeParameters() {
        powertrain::TransmissionControlUnit::Parameters params;
        params.gearCount = 6;
        params.minGearTime = 0.5;
        params.brakeInterlock = true;

        return params;
    }

    powertrain::PowertrainState stateAt(double speed) {
        powertrain::PowertrainState state;
        state.coolantTemperature = units::celcius(90.0);
        state.engineRunning = true;
        state.engineSpeed = units::rpm(900.0);
        state.gear = -1;
        state.gearCount = 6;
        state.vehicleSpeed = speed;

        return state;
    }

    void step(
        powertrain::TransmissionControlUnit &tcu,
        powertrain::PowertrainState &state,
        powertrain::DriverInputs &inputs,
        powertrain::ActuatorCommands &commands,
        int steps = 1)
    {
        for (int i = 0; i < steps; ++i) {
            tcu.update(1e-3, state, inputs, &commands);
            state.gear = commands.targetGear;
            state.engagement = commands.engagement;
            state.gatePosition = commands.gatePosition;
            state.parkLockEngaged = commands.parkLock;
        }
    }

    void select(
        powertrain::TransmissionControlUnit &tcu,
        powertrain::PowertrainState &state,
        powertrain::DriverInputs &inputs,
        powertrain::ActuatorCommands &commands,
        int position)
    {
        inputs.gatePosition = position;

        for (int i = 0; i < tcu.getGate().getCount() + 2; ++i) {
            const int before = tcu.getGatePosition();
            step(tcu, state, inputs, commands);
            if (tcu.getGatePosition() == before) break;
        }
    }
}

TEST(SelectorGateTests, TheGateOnlyMovesOneStepAtATime) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(rangeParameters());

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.brake = 1.0;
    ASSERT_EQ(tcu.getPosition().name, "D");

    inputs.gatePosition = 0;

    step(tcu, state, inputs, commands);
    EXPECT_EQ(tcu.getPosition().name, "N");

    step(tcu, state, inputs, commands);
    EXPECT_EQ(tcu.getPosition().name, "R");

    step(tcu, state, inputs, commands);
    EXPECT_EQ(tcu.getPosition().name, "P");

    step(tcu, state, inputs, commands);
    EXPECT_EQ(tcu.getPosition().name, "P");
}

TEST(SelectorGateTests, ReverseIsRefusedAboveTheReverseSpeed) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(rangeParameters());

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    select(tcu, state, inputs, commands, 2);
    ASSERT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Neutral);

    state.vehicleSpeed = 12.0;
    select(tcu, state, inputs, commands, 1);

    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Neutral);
    EXPECT_TRUE(tcu.wasPositionRefused());

    state.vehicleSpeed = 0.5;
    step(tcu, state, inputs, commands);
    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Reverse);
    EXPECT_FALSE(tcu.wasPositionRefused());
}

TEST(SelectorGateTests, ParkIsRefusedWhileRolling) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(rangeParameters());

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.brake = 1.0;
    select(tcu, state, inputs, commands, 1);
    ASSERT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Reverse);

    state.vehicleSpeed = 2.0;
    select(tcu, state, inputs, commands, 0);
    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Reverse);

    state.vehicleSpeed = 0.1;
    step(tcu, state, inputs, commands);
    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Park);
}

TEST(SelectorGateTests, LeavingParkNeedsTheBrake) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(rangeParameters());

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.brake = 1.0;
    select(tcu, state, inputs, commands, 0);
    ASSERT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Park);

    inputs.brake = 0.0;
    select(tcu, state, inputs, commands, 3);
    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Park);
    EXPECT_TRUE(tcu.wasPositionRefused());

    inputs.brake = 1.0;
    step(tcu, state, inputs, commands);
    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Reverse);
}

TEST(SelectorGateTests, ParkAndNeutralCommandTheParkLockAndOpenTheClutch) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(rangeParameters());

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.brake = 1.0;
    inputs.accelerator = 1.0;
    select(tcu, state, inputs, commands, 0);

    EXPECT_EQ(commands.engagement, powertrain::GateEngagement::Park);
    EXPECT_TRUE(commands.parkLock);
    EXPECT_NEAR(commands.clutchPressure[0], 0.0, 1e-12);
    EXPECT_EQ(commands.targetGear, -1);

    select(tcu, state, inputs, commands, 2);

    EXPECT_EQ(commands.engagement, powertrain::GateEngagement::Neutral);
    EXPECT_FALSE(commands.parkLock);
    EXPECT_NEAR(commands.clutchPressure[0], 0.0, 1e-12);
}

TEST(SelectorGateTests, ReverseNeverEngagesAForwardGear) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(rangeParameters());

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    select(tcu, state, inputs, commands, 1);
    ASSERT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Reverse);

    inputs.accelerator = 1.0;
    step(tcu, state, inputs, commands, 2000);

    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Reverse);
    EXPECT_EQ(commands.targetGear, -1);
    EXPECT_GT(commands.clutchPressure[0], 0.0);
}

TEST(SelectorGateTests, DriveStillSchedulesForwardGears) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(rangeParameters());

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    select(tcu, state, inputs, commands, 3);
    ASSERT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Forward);

    inputs.accelerator = 0.6;
    step(tcu, state, inputs, commands, 2000);

    EXPECT_EQ(commands.targetGear, 0);
    EXPECT_GT(commands.clutchPressure[0], 0.0);
}

TEST(SelectorGateTests, AGearboxWithoutRangesStaysInNeutral) {
    powertrain::TransmissionControlUnit::Parameters params = rangeParameters();

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(params);

    powertrain::GearboxCapabilities capabilities;
    capabilities.supportsRange = false;
    tcu.configureGearbox(capabilities);

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.brake = 1.0;
    select(tcu, state, inputs, commands, 0);

    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Neutral);
}

TEST(SelectorGateTests, TheLegacyGearboxRefusesParkAndReverse) {
    static const double ratios[] = { 3.6, 2.1, 1.4, 1.0, 0.8, 0.7 };

    Transmission::Parameters params;
    params.GearCount = 6;
    params.GearRatios = ratios;
    params.MaxClutchTorque = units::torque(1000.0, units::Nm);
    params.GearboxType = Transmission::Type::Legacy;

    Transmission legacy;
    legacy.initialize(params);

    EXPECT_FALSE(legacy.supportsEngagement());

    legacy.setEngagement(powertrain::GateEngagement::Park);
    EXPECT_EQ(legacy.getEngagement(), powertrain::GateEngagement::Neutral);

    legacy.setEngagement(powertrain::GateEngagement::Reverse);
    EXPECT_EQ(legacy.getEngagement(), powertrain::GateEngagement::Neutral);

    legacy.setEngagement(powertrain::GateEngagement::Forward);
    EXPECT_EQ(legacy.getEngagement(), powertrain::GateEngagement::Forward);
}

TEST(SelectorGateTests, ReverseGivesTheClutchANegativeRatio) {
    static const double ratios[] = { 3.6, 2.1, 1.4, 1.0, 0.8, 0.7 };

    Transmission::Parameters params;
    params.GearCount = 6;
    params.GearRatios = ratios;
    params.MaxClutchTorque = units::torque(1000.0, units::Nm);
    params.GearboxType = Transmission::Type::Manual;
    params.ReverseRatio = 3.2;

    Vehicle::Parameters vehicleParams;
    vehicleParams.mass = 1500.0;
    vehicleParams.dragCoefficient = 0.3;
    vehicleParams.crossSectionArea = 2.2;
    vehicleParams.diffRatio = 3.4;
    vehicleParams.tireRadius = 0.3;
    vehicleParams.rollingResistance = 200.0;

    Vehicle vehicle;
    vehicle.initialize(vehicleParams);

    Transmission gearbox;
    gearbox.initialize(params);

    atg_scs::RigidBody driveline;
    driveline.reset();
    gearbox.bind(&driveline, &vehicle, nullptr);

    gearbox.setEngagement(powertrain::GateEngagement::Forward);
    gearbox.changeGear(0);
    gearbox.setClutchPressure(0, 1.0);
    gearbox.update(1e-3);
    EXPECT_NEAR(gearbox.getClutchRatio(0), 3.6, 1e-12);

    gearbox.setEngagement(powertrain::GateEngagement::Reverse);
    gearbox.update(1e-3);
    EXPECT_NEAR(gearbox.getClutchRatio(0), -3.2, 1e-12);
    EXPECT_GT(gearbox.getClutchCapacity(0), 0.0);

    gearbox.setEngagement(powertrain::GateEngagement::Park);
    gearbox.update(1e-3);
    EXPECT_NEAR(gearbox.getClutchCapacity(0), 0.0, 1e-12);
}

TEST(SelectorGateTests, AFreelyNamedGateBehavesLikeAThrustLever) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(rangeParameters());

    powertrain::SelectorGate &gate = tcu.getGate();
    gate.clear();

    powertrain::GatePosition rev;
    rev.name = "REV";
    rev.engagement = powertrain::GateEngagement::Reverse;
    rev.maxEntrySpeed = 1.0;
    rev.mode = "reverse_thrust";
    gate.add(rev);

    powertrain::GatePosition idle;
    idle.name = "IDLE";
    idle.engagement = powertrain::GateEngagement::Neutral;
    gate.add(idle);

    for (const char *name : { "CLB", "MCT", "TOGA" }) {
        powertrain::GatePosition detent;
        detent.name = name;
        detent.engagement = powertrain::GateEngagement::Forward;
        detent.mode = name;
        gate.add(detent);
    }

    tcu.reset();

    EXPECT_EQ(gate.getCount(), 5);
    EXPECT_EQ(gate.find("TOGA"), 4);
    EXPECT_EQ(gate.find("N"), -1);

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    select(tcu, state, inputs, commands, gate.find("TOGA"));

    EXPECT_EQ(tcu.getPosition().name, "TOGA");
    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Forward);
    EXPECT_EQ(tcu.getRequestedMode(), "TOGA");

    select(tcu, state, inputs, commands, gate.find("CLB"));
    EXPECT_EQ(tcu.getPosition().name, "CLB");
    EXPECT_EQ(tcu.getRequestedMode(), "CLB");

    state.vehicleSpeed = 40.0;
    select(tcu, state, inputs, commands, gate.find("REV"));

    EXPECT_EQ(tcu.getPosition().name, "IDLE") << "reverse thrust engaged at speed";
    EXPECT_TRUE(tcu.wasPositionRefused());

    state.vehicleSpeed = 0.4;
    step(tcu, state, inputs, commands, 4);
    EXPECT_EQ(tcu.getPosition().name, "REV");
    EXPECT_EQ(tcu.getEngagement(), powertrain::GateEngagement::Reverse);
}

TEST(SelectorGateTests, AGateWithoutParkNeverCommandsTheParkLock) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(rangeParameters());

    powertrain::SelectorGate &gate = tcu.getGate();
    gate.clear();

    powertrain::GatePosition idle;
    idle.name = "IDLE";
    idle.engagement = powertrain::GateEngagement::Neutral;
    gate.add(idle);

    powertrain::GatePosition run;
    run.name = "RUN";
    run.engagement = powertrain::GateEngagement::Forward;
    gate.add(run);

    tcu.reset();

    powertrain::PowertrainState state = stateAt(0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    for (int i = 0; i < gate.getCount(); ++i) {
        inputs.gatePosition = i;
        step(tcu, state, inputs, commands, 4);
        EXPECT_FALSE(commands.parkLock) << gate.get(i).name;
    }
}

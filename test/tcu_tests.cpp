#include <gtest/gtest.h>

#include "../include/powertrain/transmission_control_unit.h"
#include "../include/powertrain/powertrain_unit.h"
#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <cmath>
#include <vector>

namespace {
    powertrain::TransmissionControlUnit::Parameters tcuParameters() {
        powertrain::TransmissionControlUnit::Parameters params;
        params.gearCount = 6;
        params.minGearTime = 0.5;

        return params;
    }

    powertrain::PowertrainState drivingState(int gear, double speed) {
        powertrain::PowertrainState state;
        state.coolantTemperature = units::celcius(90.0);
        state.engineRunning = true;
        state.gear = gear;
        state.gearCount = 6;
        state.vehicleSpeed = speed;
        state.engineSpeed = units::rpm(2000.0);

        return state;
    }

    void settle(
        powertrain::TransmissionControlUnit &tcu,
        powertrain::PowertrainState &state,
        powertrain::DriverInputs &inputs,
        powertrain::ActuatorCommands &commands,
        double duration)
    {
        const double dt = 1e-3;
        const int steps = static_cast<int>(duration / dt);

        for (int i = 0; i < steps; ++i) {
            tcu.update(dt, state, inputs, &commands);
            state.gear = commands.targetGear;
        }
    }
}

TEST(TransmissionControlUnitTests, EngineSpeedForGearFollowsTheRatios) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    const double speed = 20.0;
    const double first = tcu.engineSpeedForGear(0, speed);
    const double sixth = tcu.engineSpeedForGear(5, speed);

    EXPECT_GT(first, sixth);
    EXPECT_NEAR(
        first / sixth,
        tcuParameters().gearRatios[0] / tcuParameters().gearRatios[5],
        1e-9);
}

TEST(TransmissionControlUnitTests, UpshiftsAsSpeedRises) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    const int low = tcu.scheduleGear(0, 0.3, 5.0);
    const int high = tcu.scheduleGear(0, 0.3, 60.0);

    EXPECT_EQ(low, 0);
    EXPECT_EQ(high, 1);
}

TEST(TransmissionControlUnitTests, HigherPedalDelaysTheUpshift) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    double lightThreshold = 0.0;
    double heavyThreshold = 0.0;

    for (double speed = 1.0; speed < 120.0; speed += 0.1) {
        if (lightThreshold == 0.0 && tcu.scheduleGear(1, 0.15, speed) == 2) {
            lightThreshold = speed;
        }
        if (heavyThreshold == 0.0 && tcu.scheduleGear(1, 0.95, speed) == 2) {
            heavyThreshold = speed;
        }
    }

    ASSERT_GT(lightThreshold, 0.0);
    ASSERT_GT(heavyThreshold, 0.0);
    EXPECT_GT(heavyThreshold, lightThreshold);
}

TEST(TransmissionControlUnitTests, ShiftSequenceRunsInOrder) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    powertrain::PowertrainState state = drivingState(0, 60.0);
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.3;
    powertrain::ActuatorCommands commands;

    std::vector<powertrain::ShiftState> seen;
    const double dt = 1e-3;

    for (int i = 0; i < 3000; ++i) {
        tcu.update(dt, state, inputs, &commands);
        state.gear = commands.targetGear;

        if (seen.empty() || seen.back() != tcu.getShiftState()) {
            seen.push_back(tcu.getShiftState());
        }
    }

    ASSERT_GE(seen.size(), 6u);
    EXPECT_EQ(seen[0], powertrain::ShiftState::Idle);
    EXPECT_EQ(seen[1], powertrain::ShiftState::TorqueReduction);
    EXPECT_EQ(seen[2], powertrain::ShiftState::ClutchRelease);
    EXPECT_EQ(seen[3], powertrain::ShiftState::GearChange);
    EXPECT_EQ(seen[4], powertrain::ShiftState::ClutchEngage);
    EXPECT_EQ(seen[5], powertrain::ShiftState::Idle);
}

TEST(TransmissionControlUnitTests, ClutchIsOpenWhileTheGearChanges) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    powertrain::PowertrainState state = drivingState(0, 60.0);
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.3;
    powertrain::ActuatorCommands commands;

    const double dt = 1e-3;
    bool sawGearChange = false;

    for (int i = 0; i < 3000; ++i) {
        tcu.update(dt, state, inputs, &commands);
        state.gear = commands.targetGear;

        if (tcu.getShiftState() == powertrain::ShiftState::GearChange) {
            sawGearChange = true;
            EXPECT_NEAR(commands.clutchPressure[0], 0.0, 1e-12);
        }
    }

    EXPECT_TRUE(sawGearChange);
}

TEST(TransmissionControlUnitTests, TorqueReductionIsRequestedThenReleased) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    powertrain::PowertrainState state = drivingState(0, 60.0);
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.3;
    powertrain::ActuatorCommands commands;

    const double dt = 1e-3;
    double peak = 0.0;
    bool started = false;

    for (int i = 0; i < 5000; ++i) {
        tcu.update(dt, state, inputs, &commands);
        state.gear = commands.targetGear;
        peak = std::max(peak, tcu.getBus().torqueReductionRequest);

        if (tcu.isShifting()) started = true;
        else if (started) break;
    }

    ASSERT_TRUE(started);
    EXPECT_NEAR(peak, tcuParameters().shiftTorqueReduction, 1e-9);
    EXPECT_NEAR(tcu.getBus().torqueReductionRequest, 0.0, 1e-12);
    EXPECT_FALSE(tcu.isShifting());
}

TEST(TransmissionControlUnitTests, DownshiftAsksTheEngineToMatchSpeed) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    powertrain::PowertrainState state = drivingState(3, 12.0);
    state.engineSpeed = tcu.engineSpeedForGear(3, 12.0);

    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.2;
    powertrain::ActuatorCommands commands;

    const double beforeDownshift = state.engineSpeed;
    const double dt = 1e-3;
    bool sawSpeedRequest = false;
    double requested = 0.0;

    for (int i = 0; i < 4000; ++i) {
        tcu.update(dt, state, inputs, &commands);
        state.gear = commands.targetGear;

        if (tcu.getBus().speedRequestActive) {
            sawSpeedRequest = true;
            requested = tcu.getBus().speedRequest;
        }
    }

    ASSERT_TRUE(sawSpeedRequest);
    EXPECT_GT(requested, beforeDownshift);
    EXPECT_NEAR(requested, tcu.engineSpeedForGear(2, 12.0), 1e-9);
}

TEST(TransmissionControlUnitTests, MinimumGearTimePreventsHunting) {
    powertrain::TransmissionControlUnit::Parameters params = tcuParameters();
    params.minGearTime = 2.0;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(params);

    powertrain::PowertrainState state = drivingState(0, 60.0);
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.3;
    powertrain::ActuatorCommands commands;

    const double dt = 1e-3;
    int shifts = 0;
    int lastGear = state.gear;

    for (int i = 0; i < 4000; ++i) {
        tcu.update(dt, state, inputs, &commands);
        state.gear = commands.targetGear;

        if (state.gear != lastGear) {
            ++shifts;
            lastGear = state.gear;
        }
    }

    EXPECT_EQ(shifts, 1);
}

TEST(TransmissionControlUnitTests, ManualModeIgnoresTheSchedule) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    powertrain::PowertrainState state = drivingState(0, 60.0);
    powertrain::DriverInputs inputs;
    inputs.manualMode = true;
    inputs.accelerator = 0.3;
    powertrain::ActuatorCommands commands;

    settle(tcu, state, inputs, commands, 3.0);

    EXPECT_EQ(state.gear, 0);
}

TEST(TransmissionControlUnitTests, KickdownSelectsALowerGear) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    const int cruising = tcu.scheduleGear(4, 0.2, 30.0);
    const int kickdown = tcu.scheduleGear(4, 1.0, 30.0);

    EXPECT_LT(kickdown, cruising);
}

TEST(TransmissionControlUnitTests, LaunchControlsSlipThenLocksUp) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    powertrain::PowertrainState state = drivingState(0, 0.0);
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.6;
    powertrain::ActuatorCommands commands;

    state.clutchSlipSpeed[0] = units::rpm(2000.0);
    for (int i = 0; i < 500; ++i) tcu.update(1e-3, state, inputs, &commands);
    const double slipping = commands.clutchPressure[0];

    state.clutchSlipSpeed[0] = units::rpm(10.0);
    state.vehicleSpeed = 10.0;
    tcu.update(1e-3, state, inputs, &commands);

    EXPECT_LT(slipping, 1.0);
    EXPECT_NEAR(commands.clutchPressure[0], 1.0, 1e-9);
}

TEST(TransmissionControlUnitTests, NeutralKeepsTheClutchOpen) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    powertrain::PowertrainState state = drivingState(-1, 0.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    tcu.update(1e-3, state, inputs, &commands);

    EXPECT_EQ(commands.targetGear, -1);
    EXPECT_NEAR(commands.clutchPressure[0], 0.0, 1e-12);
}

TEST(TransmissionControlUnitTests, DualClutchOverlapsWithoutOpeningTheDriveline) {
    powertrain::TransmissionControlUnit::Parameters params = tcuParameters();
    params.supportsPreselect = true;
    params.requiresTorqueInterrupt = false;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(params);

    powertrain::PowertrainState state = drivingState(0, 60.0);
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.3;
    powertrain::ActuatorCommands commands;

    const double dt = 1e-3;
    bool sawOverlap = false;
    double minimumTotal = 1.0;

    for (int i = 0; i < 3000; ++i) {
        tcu.update(dt, state, inputs, &commands);
        state.gear = commands.targetGear;

        if (tcu.getShiftState() == powertrain::ShiftState::ClutchOverlap) {
            sawOverlap = true;
            minimumTotal = std::min(
                minimumTotal,
                commands.clutchPressure[0] + commands.clutchPressure[1]);
        }
    }

    EXPECT_TRUE(sawOverlap);
    EXPECT_NEAR(minimumTotal, 1.0, 1e-9);
}

TEST(TransmissionControlUnitTests, ParametersAreReachableThroughTheRegistry) {
    config::ParameterRegistry registry;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());
    tcu.registerParameters(&registry, "");

    ASSERT_TRUE(registry.contains("tcu.shift.min_gear_time"));
    ASSERT_TRUE(registry.contains("tcu.upshift_map"));
    EXPECT_TRUE(registry.isAdaptive("tcu.upshift_map"));

    ASSERT_TRUE(registry.set("tcu.shift.min_gear_time", 1.75));
    EXPECT_NEAR(tcu.getParameters().minGearTime, 1.75, 1e-9);
}

TEST(PowertrainUnitTests, EngineHonoursTheTransmissionTorqueRequest) {
    powertrain::PowertrainUnit unit;
    unit.initialize(
        powertrain::EngineControlUnit::Parameters(),
        tcuParameters());

    powertrain::PowertrainState state = drivingState(0, 60.0);
    state.engineSpeed = units::rpm(4000.0);

    powertrain::DriverInputs inputs;
    inputs.accelerator = 1.0;
    powertrain::ActuatorCommands commands;

    const double dt = 1e-3;
    double peakCut = 0.0;
    bool started = false;

    for (int i = 0; i < 5000; ++i) {
        unit.update(dt, state, inputs, &commands);
        state.gear = commands.targetGear;
        peakCut = std::max(peakCut, commands.ignitionCutFraction);

        if (unit.getTransmissionControlUnit().isShifting()) started = true;
        else if (started) break;
    }

    ASSERT_TRUE(started);
    EXPECT_GT(peakCut, 0.5);
    EXPECT_NEAR(commands.ignitionCutFraction, 0.0, 1e-9);
}

TEST(PowertrainUnitTests, TransmissionCommandsSurviveTheEngineUpdate) {
    powertrain::PowertrainUnit unit;
    unit.initialize(
        powertrain::EngineControlUnit::Parameters(),
        tcuParameters());

    powertrain::PowertrainState state = drivingState(2, 30.0);
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.4;
    powertrain::ActuatorCommands commands;

    unit.update(1e-3, state, inputs, &commands);

    EXPECT_EQ(commands.targetGear, 2);
    EXPECT_GT(commands.clutchPressure[0], 0.0);
}

TEST(TransmissionControlUnitTests, TheClutchPedalIsIgnoredWithoutDriverAuthority) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParameters());

    powertrain::PowertrainState state = drivingState(2, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 0.4;
    inputs.clutchPedal = 1.0;
    settle(tcu, state, inputs, commands, 1.0);

    EXPECT_GT(commands.clutchPressure[0], 0.9);
}

TEST(TransmissionControlUnitTests, TheClutchPedalCapsThePressureWithDriverAuthority) {
    powertrain::TransmissionControlUnit::Parameters params = tcuParameters();
    params.driverClutchAuthority = true;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(params);

    powertrain::PowertrainState state = drivingState(2, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 0.4;
    inputs.clutchPedal = 0.0;
    settle(tcu, state, inputs, commands, 1.0);
    const double released = commands.clutchPressure[0];

    inputs.clutchPedal = 1.0;
    tcu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.clutchPressure[0], 0.0, 1e-12);

    inputs.clutchPedal = 0.6;
    tcu.update(1e-3, state, inputs, &commands);
    EXPECT_NEAR(commands.clutchPressure[0], 0.4, 1e-12);

    EXPECT_GT(released, 0.9);
}

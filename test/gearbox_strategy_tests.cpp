#include <gtest/gtest.h>

#include "../include/powertrain/transmission_control_unit.h"
#include "../include/powertrain/powertrain_unit.h"
#include "../include/adaptation/adaptation_manager.h"
#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <cmath>

namespace {
    powertrain::TransmissionControlUnit::Parameters dctParameters() {
        powertrain::TransmissionControlUnit::Parameters params;
        params.gearCount = 6;
        params.minGearTime = 0.2;
        params.supportsPreselect = true;
        params.requiresTorqueInterrupt = false;

        return params;
    }

    powertrain::TransmissionControlUnit::Parameters converterParameters() {
        powertrain::TransmissionControlUnit::Parameters params;
        params.gearCount = 6;
        params.minGearTime = 0.2;
        params.hasLaunchDevice = true;
        params.requiresTorqueInterrupt = false;

        return params;
    }

    powertrain::TransmissionControlUnit::Parameters amtParameters() {
        powertrain::TransmissionControlUnit::Parameters params;
        params.gearCount = 6;
        params.minGearTime = 0.2;
        params.requiresTorqueInterrupt = true;

        return params;
    }

    powertrain::PowertrainState drivingState(int gear, double speed) {
        powertrain::PowertrainState state;
        state.coolantTemperature = units::celcius(90.0);
        state.engineRunning = true;
        state.engineSpeed = units::rpm(2500.0);
        state.gear = gear;
        state.gearCount = 6;
        state.vehicleSpeed = speed;

        return state;
    }

    void step(
        powertrain::TransmissionControlUnit &tcu,
        powertrain::PowertrainState &state,
        powertrain::DriverInputs &inputs,
        powertrain::ActuatorCommands &commands,
        int steps)
    {
        for (int i = 0; i < steps; ++i) {
            tcu.update(1e-3, state, inputs, &commands);
            state.gear = commands.targetGear;
            state.clutchPressure[0] = commands.clutchPressure[0];
            state.clutchPressure[1] = commands.clutchPressure[1];
            state.lockupPressure = commands.lockupPressure;
        }
    }
}

// --- DCT -----------------------------------------------------------------

TEST(DualClutchTests, OddAndEvenGearsSitOnDifferentClutches) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(dctParameters());

    for (int gear = 0; gear < 6; ++gear) {
        EXPECT_EQ(tcu.clutchForGear(gear), gear % 2) << "gear " << gear;
    }

    EXPECT_NE(tcu.clutchForGear(0), tcu.clutchForGear(1));
    EXPECT_EQ(tcu.clutchForGear(0), tcu.clutchForGear(2));
}

TEST(DualClutchTests, TheIdleClutchStagesANeighbourNotTheCurrentGear) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(dctParameters());

    powertrain::PowertrainState state = drivingState(2, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 0.5;
    step(tcu, state, inputs, commands, 20);

    const int active = tcu.getActiveClutch();
    const int idle = (active + 1) % 2;

    EXPECT_EQ(commands.clutchGear[active], 2);
    EXPECT_NE(commands.clutchGear[idle], 2) << "idle clutch holds the current gear";
    EXPECT_GE(commands.clutchGear[idle], 0);
    EXPECT_EQ(std::abs(commands.clutchGear[idle] - 2), 1) << "staged gear is not a neighbour";
}

TEST(DualClutchTests, TheRolesSwapAfterAShift) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(dctParameters());

    powertrain::PowertrainState state = drivingState(2, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 0.9;
    step(tcu, state, inputs, commands, 20);
    const int before = tcu.getActiveClutch();

    step(tcu, state, inputs, commands, 4000);

    EXPECT_NE(tcu.getActiveClutch(), before) << "the same clutch carried both gears";
    EXPECT_EQ(tcu.getActiveClutch(), tcu.clutchForGear(tcu.getTargetGear()));
}

TEST(DualClutchTests, TorqueNeverCollapsesDuringTheOverlap) {
    powertrain::TransmissionControlUnit::Parameters params = dctParameters();
    params.overlapHold = 0.2;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(params);

    powertrain::PowertrainState state = drivingState(2, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 0.9;
    step(tcu, state, inputs, commands, 20);

    bool sawOverlap = false;
    double lowest = 1.0;

    for (int i = 0; i < 4000; ++i) {
        step(tcu, state, inputs, commands, 1);

        if (tcu.getShiftState() == powertrain::ShiftState::ClutchOverlap) {
            sawOverlap = true;
            lowest = std::min(
                lowest,
                commands.clutchPressure[0] + commands.clutchPressure[1]);
        }
    }

    ASSERT_TRUE(sawOverlap);
    EXPECT_GT(lowest, 0.5) << "both clutches were open at the same time";
}

TEST(DualClutchTests, ASkipShiftFallsBackToTorqueInterrupt) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(dctParameters());

    powertrain::PowertrainState state = drivingState(2, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;
    step(tcu, state, inputs, commands, 400);

    tcu.update(1e-3, state, inputs, &commands);
    ASSERT_EQ(tcu.getShiftState(), powertrain::ShiftState::Idle);

    tcu.beginShiftForTest(3);
    EXPECT_EQ(tcu.getShiftState(), powertrain::ShiftState::ClutchOverlap)
        << "a neighbour shift should overlap";

    tcu.reset();
    step(tcu, state, inputs, commands, 400);
    tcu.beginShiftForTest(4);
    EXPECT_EQ(tcu.getShiftState(), powertrain::ShiftState::TorqueReduction)
        << "a same-shaft shift must release the clutch";
}

TEST(DualClutchTests, ADualClutchShiftTrainsTheLearningProfile) {
    powertrain::PowertrainUnit unit;
    unit.initialize(
        powertrain::EngineControlUnit::Parameters(), dctParameters());

    adaptation::AdaptationManager adaptation;
    adaptation.initialize(adaptation::AdaptationManager::Parameters());
    adaptation.attach(
        &unit.getEngineControlUnit(), &unit.getTransmissionControlUnit());

    powertrain::PowertrainState state = drivingState(2, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 0.9;

    for (int i = 0; i < 6000; ++i) {
        unit.update(1e-3, state, inputs, &commands);
        state.gear = commands.targetGear;
        state.clutchSlipSpeed[0] = units::rpm(200.0);
        adaptation.update(1e-3, state, unit.getBus());
    }

    EXPECT_GT(
        unit.getTransmissionControlUnit().getEngageProfile().getIterationCount(), 0)
        << "a dual clutch never trained its engage profile";
}

TEST(DualClutchTests, BackToBackShiftsBeatTheTorqueInterruptPath) {
    powertrain::PowertrainState state = drivingState(0, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;

    powertrain::TransmissionControlUnit dct;
    dct.initialize(dctParameters());
    step(dct, state, inputs, commands, 400);

    int overlapTicks = 0;
    dct.beginShiftForTest(1);
    while (dct.isShifting() && overlapTicks < 20000) {
        step(dct, state, inputs, commands, 1);
        ++overlapTicks;
    }

    step(dct, state, inputs, commands, 400);
    dct.beginShiftForTest(2);
    while (dct.isShifting() && overlapTicks < 20000) {
        step(dct, state, inputs, commands, 1);
        ++overlapTicks;
    }

    powertrain::TransmissionControlUnit amt;
    amt.initialize(amtParameters());

    state = drivingState(0, 20.0);
    step(amt, state, inputs, commands, 400);

    int interruptTicks = 0;
    amt.beginShiftForTest(1);
    while (amt.isShifting() && interruptTicks < 20000) {
        step(amt, state, inputs, commands, 1);
        ++interruptTicks;
    }

    step(amt, state, inputs, commands, 400);
    amt.beginShiftForTest(2);
    while (amt.isShifting() && interruptTicks < 20000) {
        step(amt, state, inputs, commands, 1);
        ++interruptTicks;
    }

    EXPECT_LT(overlapTicks, interruptTicks)
        << "the dual clutch was no faster than a torque interrupt";
}

// --- Torque converter ----------------------------------------------------

TEST(TorqueConverterStrategyTests, LockupOpensBelowTheScheduleAndClosesAbove) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(converterParameters());

    powertrain::PowertrainState state = drivingState(2, 2.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;
    inputs.accelerator = 0.3;
    state.converterSlip = units::rpm(400.0);

    step(tcu, state, inputs, commands, 50);
    EXPECT_NEAR(commands.lockupPressure, 0.0, 1e-12) << "locked up while crawling";

    state.vehicleSpeed = 45.0;
    step(tcu, state, inputs, commands, 500);
    EXPECT_GT(commands.lockupPressure, 0.0) << "never engaged at speed";
}

TEST(TorqueConverterStrategyTests, TheLockupPressureRisesSmoothly) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(converterParameters());

    powertrain::PowertrainState state = drivingState(2, 45.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;
    inputs.accelerator = 0.3;
    state.converterSlip = units::rpm(400.0);

    double previous = 0.0;
    double biggestStep = 0.0;

    for (int i = 0; i < 500; ++i) {
        step(tcu, state, inputs, commands, 1);
        biggestStep = std::max(biggestStep, std::abs(commands.lockupPressure - previous));
        previous = commands.lockupPressure;
    }

    EXPECT_LT(biggestStep, 0.1) << "the lockup slammed shut";
    EXPECT_GT(previous, 0.0);
}

TEST(TorqueConverterStrategyTests, KickdownReleasesTheLockup) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(converterParameters());

    powertrain::PowertrainState state = drivingState(2, 45.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;
    inputs.accelerator = 0.3;
    state.converterSlip = units::rpm(400.0);
    step(tcu, state, inputs, commands, 500);
    ASSERT_GT(commands.lockupPressure, 0.0);

    inputs.accelerator = 1.0;
    step(tcu, state, inputs, commands, 5);

    EXPECT_NEAR(commands.lockupPressure, 0.0, 1e-12);
}

TEST(TorqueConverterStrategyTests, ASmallSlipLocksItFully) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(converterParameters());

    powertrain::PowertrainState state = drivingState(2, 45.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;
    inputs.accelerator = 0.3;
    state.converterSlip = units::rpm(5.0);

    step(tcu, state, inputs, commands, 1000);
    EXPECT_NEAR(commands.lockupPressure, 1.0, 1e-9);
}

TEST(TorqueConverterStrategyTests, TheScheduleIsReachableFromTheRegistry) {
    config::ParameterRegistry registry;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(converterParameters());
    tcu.registerParameters(&registry, "");

    for (const char *path : {
        "tcu.lockup_map",
        "tcu.lockup.slip_target",
        "tcu.lockup.lock_slip",
        "tcu.lockup.pid.kp",
        "tcu.lockup.pid.ki" })
    {
        EXPECT_TRUE(registry.contains(path)) << path;
    }
}

TEST(TorqueConverterStrategyTests, AShiftReleasesTheLockup) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(converterParameters());

    powertrain::PowertrainState state = drivingState(2, 45.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;
    inputs.accelerator = 0.3;
    state.converterSlip = units::rpm(400.0);

    step(tcu, state, inputs, commands, 1000);
    ASSERT_GT(commands.lockupPressure, 0.0);

    tcu.beginShiftForTest(3);
    ASSERT_TRUE(tcu.isShifting());

    double highest = 0.0;
    while (tcu.isShifting()) {
        step(tcu, state, inputs, commands, 1);
        if (tcu.isShifting()) highest = std::max(highest, commands.lockupPressure);
    }

    EXPECT_NEAR(highest, 0.0, 1e-12) << "the converter stayed locked through a shift";
}

TEST(TorqueConverterStrategyTests, MovingTheScheduleMovesTheLockupPoint) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(converterParameters());

    control::Map2d &map = tcu.getLockupMap();
    const double threshold = map.sample(0.3, 2.0);

    powertrain::PowertrainState state = drivingState(2, threshold * 0.5);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;
    inputs.accelerator = 0.3;
    state.converterSlip = units::rpm(400.0);

    step(tcu, state, inputs, commands, 500);
    ASSERT_NEAR(commands.lockupPressure, 0.0, 1e-12) << "locked below the schedule";

    for (int i = 0; i < map.getXCount(); ++i) {
        for (int j = 0; j < map.getYCount(); ++j) {
            map.setValue(i, j, threshold * 0.25);
        }
    }

    step(tcu, state, inputs, commands, 500);
    EXPECT_GT(commands.lockupPressure, 0.0) << "the lowered schedule did not take effect";
}

// --- AMT -----------------------------------------------------------------

TEST(RobotisedManualTests, TheGapUsesTheDeepCutAndEngagementTheShallowOne) {
    powertrain::TransmissionControlUnit::Parameters params = amtParameters();
    params.shiftTorqueCut = 1.0;
    params.shiftTorqueReduction = 0.5;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(params);

    powertrain::PowertrainState state = drivingState(2, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 0.9;
    step(tcu, state, inputs, commands, 20);

    double gapCut = 0.0;
    double engageCut = 1.0;

    for (int i = 0; i < 4000; ++i) {
        step(tcu, state, inputs, commands, 1);

        const powertrain::ShiftState shiftState = tcu.getShiftState();
        if (shiftState == powertrain::ShiftState::GearChange) {
            gapCut = std::max(gapCut, tcu.getBus().torqueReductionRequest);
        }
        else if (shiftState == powertrain::ShiftState::ClutchEngage) {
            engageCut = std::min(engageCut, tcu.getBus().torqueReductionRequest);
        }
    }

    EXPECT_NEAR(gapCut, 1.0, 1e-9) << "the gap did not use the deep cut";
    EXPECT_LT(engageCut, 1.0);
}

TEST(RobotisedManualTests, ADownshiftAsksTheEngineForASpeed) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(amtParameters());

    powertrain::PowertrainState state = drivingState(3, 12.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;
    step(tcu, state, inputs, commands, 400);

    inputs.shiftDownRequest = true;
    step(tcu, state, inputs, commands, 2);
    inputs.shiftDownRequest = false;

    bool sawSpeedRequest = false;
    for (int i = 0; i < 4000; ++i) {
        step(tcu, state, inputs, commands, 1);
        if (tcu.getBus().speedRequestActive && tcu.getBus().speedRequest > 0.0) {
            sawSpeedRequest = true;
        }
    }

    EXPECT_TRUE(sawSpeedRequest) << "no blip was requested on a downshift";
}

TEST(RobotisedManualTests, TheSpeedMatchToleranceIsTunable) {
    config::ParameterRegistry registry;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(amtParameters());
    tcu.registerParameters(&registry, "");

    ASSERT_TRUE(registry.contains("tcu.shift.speed_match_tolerance"));
    ASSERT_TRUE(registry.set("tcu.shift.speed_match_tolerance", units::rpm(300.0)));
    EXPECT_NEAR(
        tcu.getParameters().speedMatchTolerance, units::rpm(300.0), 1e-9);
}

// --- Gangzahl und Schaltplan ---------------------------------------------

namespace {
    powertrain::GearboxCapabilities eightSpeed(const double *ratios) {
        powertrain::GearboxCapabilities caps;
        caps.gearCount = 8;
        caps.gearRatios = ratios;

        return caps;
    }
}

TEST(GearScheduleTests, TheGearboxGearCountReachesTheShiftMaps) {
    static const double ratios[8] =
        { 4.70, 3.10, 2.10, 1.60, 1.20, 1.00, 0.85, 0.67 };

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(powertrain::TransmissionControlUnit::Parameters());

    ASSERT_EQ(tcu.getUpshiftMap().getYCount(), 6);

    tcu.configureGearbox(eightSpeed(ratios));

    EXPECT_EQ(tcu.getParameters().gearCount, 8);
    EXPECT_EQ(tcu.getUpshiftMap().getYCount(), 8);
    EXPECT_EQ(tcu.getDownshiftMap().getYCount(), 8);
    EXPECT_EQ(tcu.getLockupMap().getYCount(), 8);

    EXPECT_GT(
        tcu.getUpshiftMap().sample(0.5, 7.0),
        tcu.getUpshiftMap().sample(0.5, 5.0))
        << "the top gears share one schedule row";
}

TEST(GearScheduleTests, TheScheduleFollowsTheGearboxRatiosNotTheDefaults) {
    static const double tall[8] =
        { 4.70, 3.10, 2.10, 1.60, 1.20, 1.00, 0.85, 0.67 };
    static const double shorter[8] =
        { 9.40, 6.20, 4.20, 3.20, 2.40, 2.00, 1.70, 1.34 };

    powertrain::TransmissionControlUnit tallUnit;
    tallUnit.initialize(powertrain::TransmissionControlUnit::Parameters());
    tallUnit.configureGearbox(eightSpeed(tall));

    powertrain::TransmissionControlUnit shortUnit;
    shortUnit.initialize(powertrain::TransmissionControlUnit::Parameters());
    shortUnit.configureGearbox(eightSpeed(shorter));

    EXPECT_NEAR(
        tallUnit.getUpshiftMap().sample(0.5, 3.0),
        2.0 * shortUnit.getUpshiftMap().sample(0.5, 3.0),
        1e-6) << "halving every ratio did not halve the shift speed";
}

TEST(GearScheduleTests, AnAuthoredMapSurvivesTheGearboxSync) {
    static const double ratios[8] =
        { 4.70, 3.10, 2.10, 1.60, 1.20, 1.00, 0.85, 0.67 };

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(powertrain::TransmissionControlUnit::Parameters());

    control::Map2d &map = tcu.getUpshiftMap();
    for (int i = 0; i < map.getXCount(); ++i) {
        for (int g = 0; g < map.getYCount(); ++g) {
            map.setValue(i, g, 11.0 + g);
        }
    }

    tcu.markAuthoredMaps(true, false, false);
    tcu.configureGearbox(eightSpeed(ratios));

    EXPECT_EQ(tcu.getUpshiftMap().getYCount(), 8);
    EXPECT_NEAR(tcu.getUpshiftMap().sample(0.5, 0.0), 11.0, 1e-9)
        << "the scripted schedule was overwritten";
    EXPECT_NEAR(tcu.getUpshiftMap().sample(0.5, 5.0), 16.0, 1e-9);

    EXPECT_GT(tcu.getDownshiftMap().sample(0.5, 7.0), 0.0)
        << "the unscripted map was not rebuilt for the new gears";
}

// --- Schaltcharakter ------------------------------------------------------

namespace {
    double overlapRise(
        powertrain::TransmissionControlUnit &tcu,
        powertrain::PowertrainState &state,
        powertrain::DriverInputs &inputs,
        powertrain::ActuatorCommands &commands,
        double *peakStep)
    {
        step(tcu, state, inputs, commands, 400);
        tcu.beginShiftForTest(2);

        const int oncomingClutch = tcu.clutchForGear(2);

        double previous = 0.0;
        double biggest = 0.0;
        int ticks = 0;

        while (tcu.getShiftState() == powertrain::ShiftState::ClutchOverlap
            && ticks < 20000)
        {
            step(tcu, state, inputs, commands, 1);
            const double oncoming = commands.clutchPressure[oncomingClutch];
            biggest = std::max(biggest, std::abs(oncoming - previous));
            previous = oncoming;
            ++ticks;
        }

        *peakStep = biggest;

        return static_cast<double>(ticks);
    }
}

TEST(ShiftShapeTests, TheDefaultShapeIsTheLinearRamp) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(dctParameters());

    const control::Map2d &shape = tcu.getOverlapShape();
    ASSERT_TRUE(shape.isInitialized());

    for (double t = 0.0; t <= 1.0; t += 0.125) {
        EXPECT_NEAR(shape.sample(t, 0.5), t, 1e-9) << "phase " << t;
    }
}

TEST(ShiftShapeTests, ASteepShapeEngagesHarderThanTheLinearRamp) {
    powertrain::PowertrainState state = drivingState(1, 20.0);
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.manualMode = true;
    inputs.accelerator = 0.5;

    powertrain::TransmissionControlUnit soft;
    soft.initialize(dctParameters());

    double softStep = 0.0;
    overlapRise(soft, state, inputs, commands, &softStep);

    powertrain::TransmissionControlUnit crisp;
    crisp.initialize(dctParameters());

    control::Map2d &crispShape = crisp.getOverlapShape();
    for (int j = 0; j < crispShape.getYCount(); ++j) {
        for (int i = 0; i < crispShape.getXCount(); ++i) {
            const double t = crispShape.getXAxis(i);
            crispShape.setValue(i, j, std::min(1.0, t * 4.0));
        }
    }

    state = drivingState(1, 20.0);
    double crispStep = 0.0;
    overlapRise(crisp, state, inputs, commands, &crispStep);

    EXPECT_GT(crispStep, softStep * 2.5)
        << "the steep shape engaged no harder than the linear ramp";
}

TEST(ShiftShapeTests, TheShapeIsPedalDependent) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(dctParameters());

    control::Map2d &shape = tcu.getOverlapShape();
    for (int j = 0; j < shape.getYCount(); ++j) {
        const double pedal = shape.getYAxis(j);

        for (int i = 0; i < shape.getXCount(); ++i) {
            const double t = shape.getXAxis(i);
            shape.setValue(i, j, (pedal > 0.5) ? std::min(1.0, t * 3.0) : t * t);
        }
    }

    EXPECT_GT(shape.sample(0.25, 1.0), shape.sample(0.25, 0.0))
        << "the pedal axis has no effect on the shape";
}

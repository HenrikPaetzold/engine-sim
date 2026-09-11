#include <gtest/gtest.h>

#include "../include/powertrain/manoeuvre.h"
#include "../include/units.h"

#include <cmath>

namespace {
    powertrain::Setpoint at(double time, double accelerator) {
        powertrain::Setpoint setpoint;
        setpoint.time = time;
        setpoint.accelerator = accelerator;

        return setpoint;
    }

    powertrain::Manoeuvre rampThenHold() {
        powertrain::Manoeuvre manoeuvre;
        manoeuvre.add(at(0.0, 0.0));
        manoeuvre.add(at(2.0, 1.0));
        manoeuvre.add(at(4.0, 1.0));
        manoeuvre.sort();

        return manoeuvre;
    }
}

TEST(ManoeuvreTests, ThePedalRampsBetweenSetpoints) {
    const powertrain::Manoeuvre manoeuvre = rampThenHold();

    EXPECT_NEAR(manoeuvre.sample(0.0).accelerator, 0.0, 1e-9);
    EXPECT_NEAR(manoeuvre.sample(0.5).accelerator, 0.25, 1e-9);
    EXPECT_NEAR(manoeuvre.sample(1.0).accelerator, 0.5, 1e-9);
    EXPECT_NEAR(manoeuvre.sample(2.0).accelerator, 1.0, 1e-9);
    EXPECT_NEAR(manoeuvre.sample(3.0).accelerator, 1.0, 1e-9);
}

TEST(ManoeuvreTests, TheGearIsHeldAndNeverInterpolated) {
    powertrain::Manoeuvre manoeuvre;

    powertrain::Setpoint third = at(0.0, 0.5);
    third.selectedGear = 3;
    third.gatePosition = 2;
    manoeuvre.add(third);

    powertrain::Setpoint fourth = at(1.0, 0.5);
    fourth.selectedGear = 4;
    fourth.gatePosition = 3;
    manoeuvre.add(fourth);
    manoeuvre.sort();

    for (double t = 0.0; t < 1.0; t += 0.1) {
        EXPECT_EQ(manoeuvre.sample(t).selectedGear, 3) << t;
        EXPECT_EQ(manoeuvre.sample(t).gatePosition, 2) << t;
    }

    EXPECT_EQ(manoeuvre.sample(1.0).selectedGear, 4);
    EXPECT_EQ(manoeuvre.sample(1.0).gatePosition, 3);
}

TEST(ManoeuvreTests, TheSetpointsAreSortedByTimeRegardlessOfOrder) {
    powertrain::Manoeuvre manoeuvre;
    manoeuvre.add(at(4.0, 1.0));
    manoeuvre.add(at(0.0, 0.0));
    manoeuvre.add(at(2.0, 0.5));
    manoeuvre.sort();

    EXPECT_NEAR(manoeuvre.get(0).time, 0.0, 1e-9);
    EXPECT_NEAR(manoeuvre.get(1).time, 2.0, 1e-9);
    EXPECT_NEAR(manoeuvre.get(2).time, 4.0, 1e-9);
    EXPECT_NEAR(manoeuvre.sample(1.0).accelerator, 0.25, 1e-9);
}

TEST(ManoeuvreTests, BeforeAndAfterTheEndsTheValuesAreHeldFlat) {
    const powertrain::Manoeuvre manoeuvre = rampThenHold();

    EXPECT_NEAR(manoeuvre.sample(-5.0).accelerator, 0.0, 1e-9);
    EXPECT_NEAR(manoeuvre.sample(99.0).accelerator, 1.0, 1e-9);
}

TEST(ManoeuvreTests, AnEmptyManoeuvreYieldsTheNeutralDriverInputs) {
    powertrain::Manoeuvre manoeuvre;

    EXPECT_TRUE(manoeuvre.isEmpty());
    EXPECT_EQ(manoeuvre.getDuration(), 0.0);
    EXPECT_NEAR(manoeuvre.sample(1.0).accelerator, 0.0, 1e-9);
}

TEST(ManoeuvrePlayerTests, ThePlayerDrivesTheInputsWhileItRuns) {
    const powertrain::Manoeuvre manoeuvre = rampThenHold();

    powertrain::ManoeuvrePlayer player;
    player.setManoeuvre(&manoeuvre);

    powertrain::DriverInputs inputs;
    EXPECT_FALSE(player.update(10.0, &inputs));

    player.start(10.0);
    ASSERT_TRUE(player.isRunning());

    EXPECT_TRUE(player.update(11.0, &inputs));
    EXPECT_NEAR(inputs.accelerator, 0.5, 1e-9);

    EXPECT_TRUE(player.update(12.0, &inputs));
    EXPECT_NEAR(inputs.accelerator, 1.0, 1e-9);
}

TEST(ManoeuvrePlayerTests, ThePlayerStopsItselfAtTheEnd) {
    const powertrain::Manoeuvre manoeuvre = rampThenHold();

    powertrain::ManoeuvrePlayer player;
    player.setManoeuvre(&manoeuvre);
    player.start(0.0);

    powertrain::DriverInputs inputs;
    EXPECT_TRUE(player.update(4.0, &inputs));
    EXPECT_NEAR(player.getProgress(), 1.0, 1e-9);

    EXPECT_FALSE(player.update(4.1, &inputs));
    EXPECT_FALSE(player.isRunning());
}

TEST(ManoeuvrePlayerTests, AShiftRequestIsPulsedForExactlyOneTick) {
    powertrain::Manoeuvre manoeuvre;
    manoeuvre.add(at(0.0, 0.5));

    powertrain::Setpoint shift = at(1.0, 0.5);
    shift.shiftUp = true;
    manoeuvre.add(shift);

    manoeuvre.add(at(3.0, 0.5));
    manoeuvre.sort();

    powertrain::ManoeuvrePlayer player;
    player.setManoeuvre(&manoeuvre);
    player.start(0.0);

    powertrain::DriverInputs inputs;
    int raised = 0;
    for (double t = 0.0; t <= 3.0; t += 0.01) {
        if (player.update(t, &inputs) && inputs.shiftUpRequest) ++raised;
    }

    EXPECT_EQ(raised, 1) << "the shift request must rise once, not be held";
}

TEST(ManoeuvrePlayerTests, TheClutchDefaultMeansTheFootIsOffThePedal) {
    powertrain::Setpoint setpoint;

    EXPECT_EQ(setpoint.clutchPedal, 0.0)
        << "clutchPedal is inverted: 1.0 opens the clutch";
}

TEST(ManoeuvrePlayerTests, AFreshPlayerWithoutAManoeuvreNeverRuns) {
    powertrain::ManoeuvrePlayer player;
    player.start(0.0);

    EXPECT_FALSE(player.isRunning());

    powertrain::DriverInputs inputs;
    EXPECT_FALSE(player.update(1.0, &inputs));
}

// --- record mode ----------------------------------------------------------

namespace {
    powertrain::DriverInputs drive(double accelerator) {
        powertrain::DriverInputs inputs;
        inputs.accelerator = accelerator;
        inputs.clutchPedal = 0.0;
        inputs.gatePosition = 3;

        return inputs;
    }

    void recordRamp(powertrain::ManoeuvreRecorder *recorder, double duration) {
        recorder->start(0.0);
        for (double t = 0.0; t <= duration; t += 0.001) {
            recorder->update(t, drive(std::min(1.0, t / duration)));
        }
        recorder->stop();
    }
}

TEST(ManoeuvreRecorderTests, TheRecorderSamplesAtItsIntervalNotEveryTick) {
    powertrain::ManoeuvreRecorder recorder;
    recorder.setInterval(0.05);

    recordRamp(&recorder, 1.0);

    EXPECT_GT(recorder.getCount(), 15);
    EXPECT_LT(recorder.getCount(), 40)
        << "a 1 kHz tick rate must not produce 1000 samples per second";
}

TEST(ManoeuvreRecorderTests, AStraightRampThinsDownToItsEnds) {
    powertrain::ManoeuvreRecorder recorder;
    recorder.setInterval(0.05);
    recorder.setTolerance(0.02);

    recordRamp(&recorder, 2.0);

    powertrain::Manoeuvre manoeuvre;
    recorder.thin(&manoeuvre);

    EXPECT_LE(manoeuvre.getCount(), 4)
        << "a straight line needs no intermediate setpoints";
    EXPECT_GE(manoeuvre.getCount(), 2);
    EXPECT_NEAR(manoeuvre.sample(1.0).accelerator, 0.5, 0.03);
}

TEST(ManoeuvreRecorderTests, EveryDiscreteChangeSurvivesTheThinning) {
    powertrain::ManoeuvreRecorder recorder;
    recorder.setInterval(0.05);
    recorder.start(0.0);

    for (double t = 0.0; t <= 2.0; t += 0.001) {
        powertrain::DriverInputs inputs = drive(0.5);
        if (t >= 1.0 && t < 1.002) inputs.shiftUpRequest = true;
        if (t >= 1.5) inputs.gatePosition = 2;
        recorder.update(t, inputs);
    }
    recorder.stop();

    powertrain::Manoeuvre manoeuvre;
    recorder.thin(&manoeuvre);

    bool sawShift = false;
    bool sawGate = false;
    for (int i = 0; i < manoeuvre.getCount(); ++i) {
        if (manoeuvre.get(i).shiftUp) sawShift = true;
        if (manoeuvre.get(i).gatePosition == 2) sawGate = true;
    }

    EXPECT_TRUE(sawShift) << "a shift request must never be thinned away";
    EXPECT_TRUE(sawGate) << "a gate change must never be thinned away";
}

TEST(ManoeuvreRecorderTests, TheThinnedShapeStaysWithinTheTolerance) {
    powertrain::ManoeuvreRecorder recorder;
    recorder.setInterval(0.02);
    recorder.setTolerance(0.05);

    recorder.start(0.0);
    for (double t = 0.0; t <= 4.0; t += 0.001) {
        recorder.update(t, drive(0.5 + 0.5 * std::sin(t * 2.0)));
    }
    recorder.stop();

    powertrain::Manoeuvre manoeuvre;
    recorder.thin(&manoeuvre);

    for (double t = 0.0; t <= 4.0; t += 0.01) {
        const double expected = 0.5 + 0.5 * std::sin(t * 2.0);
        EXPECT_NEAR(manoeuvre.sample(t).accelerator, expected, 0.06) << t;
    }
}

TEST(ManoeuvreRecorderTests, TheExportedScriptUsesNoScientificNotation) {
    powertrain::ManoeuvreRecorder recorder;
    recorder.setInterval(0.001);

    recorder.start(0.0);
    for (double t = 0.0; t <= 0.01; t += 0.001) {
        recorder.update(t, drive(t * 0.0001));
    }
    recorder.stop();

    const std::string script = recorder.toScript("tiny");

    EXPECT_EQ(script.find("e+"), std::string::npos) << script;
    EXPECT_EQ(script.find("e-"), std::string::npos) << script;
}

TEST(ManoeuvreRecorderTests, AnEmptyRecordingStillProducesAValidHeader) {
    powertrain::ManoeuvreRecorder recorder;

    const std::string script = recorder.toScript("nothing");

    EXPECT_NE(script.find("add_manoeuvre("), std::string::npos);
    EXPECT_NE(script.find("manoeuvre(name: \"nothing\")"), std::string::npos);
}

// --- findings from the read-through -----------------------------------------

TEST(ManoeuvrePlayerTests, AShiftSurvivesATickThatSkipsOverItsSetpoint) {
    powertrain::Manoeuvre manoeuvre;
    manoeuvre.add(at(0.0, 0.5));

    powertrain::Setpoint shift = at(1.0, 0.5);
    shift.shiftUp = true;
    manoeuvre.add(shift);

    manoeuvre.add(at(1.001, 0.5));
    manoeuvre.add(at(3.0, 0.5));
    manoeuvre.sort();

    powertrain::ManoeuvrePlayer player;
    player.setManoeuvre(&manoeuvre);
    player.start(0.0);

    powertrain::DriverInputs inputs;
    player.update(0.5, &inputs);

    ASSERT_TRUE(player.update(1.5, &inputs))
        << "this tick crosses both the shift setpoint and the one after it";

    EXPECT_TRUE(inputs.shiftUpRequest)
        << "a shift request must not be lost when one tick skips its setpoint";
}

TEST(ManoeuvrePlayerTests, TwoShiftsCrossedInOneTickStillRaiseTheRequestOnce) {
    powertrain::Manoeuvre manoeuvre;
    manoeuvre.add(at(0.0, 0.5));

    powertrain::Setpoint up = at(1.0, 0.5);
    up.shiftUp = true;
    manoeuvre.add(up);

    powertrain::Setpoint down = at(1.001, 0.5);
    down.shiftDown = true;
    manoeuvre.add(down);

    manoeuvre.add(at(3.0, 0.5));
    manoeuvre.sort();

    powertrain::ManoeuvrePlayer player;
    player.setManoeuvre(&manoeuvre);
    player.start(0.0);

    powertrain::DriverInputs inputs;
    player.update(0.5, &inputs);
    player.update(1.5, &inputs);

    EXPECT_TRUE(inputs.shiftUpRequest);
    EXPECT_TRUE(inputs.shiftDownRequest);

    ASSERT_TRUE(player.update(1.6, &inputs));
    EXPECT_FALSE(inputs.shiftUpRequest) << "and it must fall again on the next tick";
    EXPECT_FALSE(inputs.shiftDownRequest);
}

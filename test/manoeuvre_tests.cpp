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

#include <gtest/gtest.h>

#include "../include/ignition_module.h"
#include "../include/intake.h"
#include "../include/units.h"

#include <cmath>

TEST(IgnitionCutTests, ZeroFractionFiresEveryEvent) {
    IgnitionModule ignition;
    ignition.setCutFraction(0.0);

    for (int i = 0; i < 100; ++i) EXPECT_TRUE(ignition.consumeCutDecision());
}

TEST(IgnitionCutTests, FullFractionCutsEveryEvent) {
    IgnitionModule ignition;
    ignition.setCutFraction(1.0);

    for (int i = 0; i < 100; ++i) EXPECT_FALSE(ignition.consumeCutDecision());
}

TEST(IgnitionCutTests, PartialFractionMatchesTheRequestedRate) {
    for (double fraction : { 0.1, 0.25, 0.5, 0.75, 0.9 }) {
        IgnitionModule ignition;
        ignition.setCutFraction(fraction);

        const int events = 10000;
        int cuts = 0;
        for (int i = 0; i < events; ++i) {
            if (!ignition.consumeCutDecision()) ++cuts;
        }

        EXPECT_NEAR(static_cast<double>(cuts) / events, fraction, 1e-3)
            << "fraction=" << fraction;
    }
}

TEST(IgnitionCutTests, HalfFractionAlternates) {
    IgnitionModule ignition;
    ignition.setCutFraction(0.5);

    EXPECT_TRUE(ignition.consumeCutDecision());
    EXPECT_FALSE(ignition.consumeCutDecision());
    EXPECT_TRUE(ignition.consumeCutDecision());
    EXPECT_FALSE(ignition.consumeCutDecision());
}

TEST(IgnitionCutTests, FractionIsClamped) {
    IgnitionModule ignition;

    ignition.setCutFraction(5.0);
    EXPECT_NEAR(ignition.getCutFraction(), 1.0, 1e-12);

    ignition.setCutFraction(-5.0);
    EXPECT_NEAR(ignition.getCutFraction(), 0.0, 1e-12);
}

TEST(IntakeTests, FuelCutRemovesFuelFromTheCharge) {
    GasSystem::Mix mix;
    mix.p_fuel = 0.2;
    mix.p_inert = 0.6;
    mix.p_o2 = 0.2;

    const GasSystem::Mix cut = Intake::scaleFuel(mix, 0.0);

    EXPECT_NEAR(cut.p_fuel, 0.0, 1e-12);
    EXPECT_NEAR(cut.p_inert + cut.p_o2, 1.0, 1e-12);
}

TEST(IntakeTests, FuelScalingKeepsMoleFractionsNormalised) {
    GasSystem::Mix mix;
    mix.p_fuel = 0.2;
    mix.p_inert = 0.6;
    mix.p_o2 = 0.2;

    for (double factor : { 0.25, 0.5, 1.0, 1.5, 3.0 }) {
        const GasSystem::Mix scaled = Intake::scaleFuel(mix, factor);
        const double total = scaled.p_fuel + scaled.p_inert + scaled.p_o2;

        EXPECT_NEAR(total, 1.0, 1e-12) << "factor=" << factor;
    }
}

TEST(IntakeTests, EnrichmentRaisesTheFuelFraction) {
    GasSystem::Mix mix;
    mix.p_fuel = 0.2;
    mix.p_inert = 0.6;
    mix.p_o2 = 0.2;

    const GasSystem::Mix lean = Intake::scaleFuel(mix, 0.5);
    const GasSystem::Mix rich = Intake::scaleFuel(mix, 2.0);

    EXPECT_LT(lean.p_fuel, mix.p_fuel);
    EXPECT_GT(rich.p_fuel, mix.p_fuel);
}

TEST(IntakeTests, UnityFactorIsANoOp) {
    GasSystem::Mix mix;
    mix.p_fuel = 0.2;
    mix.p_inert = 0.6;
    mix.p_o2 = 0.2;

    const GasSystem::Mix scaled = Intake::scaleFuel(mix, 1.0);

    EXPECT_NEAR(scaled.p_fuel, mix.p_fuel, 1e-12);
    EXPECT_NEAR(scaled.p_inert, mix.p_inert, 1e-12);
    EXPECT_NEAR(scaled.p_o2, mix.p_o2, 1e-12);
}

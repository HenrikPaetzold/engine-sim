#include <gtest/gtest.h>

#include "../include/thermal_model.h"
#include "../include/ignition_module.h"
#include "../include/intake.h"
#include "../include/units.h"

#include <cmath>

namespace {
    ThermalModel::Parameters simpleThermalParameters() {
        ThermalModel::Parameters params;
        params.blockThermalMass = 1000.0;
        params.oilThermalMass = 500.0;
        params.blockToOilConductance = 10.0;
        params.radiatorConductance = 20.0;
        params.oilToAmbientConductance = 5.0;
        params.speedCoolingCoefficient = 0.0;
        params.thermostatOpenTemperature = units::celcius(85.0);
        params.thermostatFullTemperature = units::celcius(100.0);
        params.ambientTemperature = units::celcius(20.0);

        return params;
    }
}

TEST(ThermalModelTests, StartsAtAmbient) {
    ThermalModel model;
    model.initialize(simpleThermalParameters());

    EXPECT_NEAR(model.getBlockTemperature(), units::celcius(20.0), 1e-9);
    EXPECT_NEAR(model.getOilTemperature(), units::celcius(20.0), 1e-9);
}

TEST(ThermalModelTests, WarmUpIsMonotonic) {
    ThermalModel model;
    model.initialize(simpleThermalParameters());

    const double dt = 0.01;
    double previous = model.getBlockTemperature();

    for (int i = 0; i < 20000; ++i) {
        model.addHeat(2000.0 * dt);
        model.update(dt, 0.0);

        EXPECT_GE(model.getBlockTemperature(), previous - 1e-9);
        previous = model.getBlockTemperature();
    }

    EXPECT_GT(model.getBlockTemperature(), units::celcius(50.0));
}

TEST(ThermalModelTests, OilLagsBehindTheBlock) {
    ThermalModel model;
    model.initialize(simpleThermalParameters());

    const double dt = 0.01;
    for (int i = 0; i < 2000; ++i) {
        model.addHeat(2000.0 * dt);
        model.update(dt, 0.0);
    }

    EXPECT_GT(model.getBlockTemperature(), model.getOilTemperature());
}

TEST(ThermalModelTests, ThermostatStaysShutWhileCold) {
    ThermalModel model;
    model.initialize(simpleThermalParameters());

    EXPECT_NEAR(model.thermostatOpening(), 0.0, 1e-12);

    model.setBlockTemperature(units::celcius(85.0));
    EXPECT_NEAR(model.thermostatOpening(), 0.0, 1e-12);

    model.setBlockTemperature(units::celcius(92.5));
    EXPECT_NEAR(model.thermostatOpening(), 0.5, 1e-9);

    model.setBlockTemperature(units::celcius(120.0));
    EXPECT_NEAR(model.thermostatOpening(), 1.0, 1e-12);
}

TEST(ThermalModelTests, CoolingWithoutHeatMatchesTheAnalyticSolution) {
    ThermalModel::Parameters params = simpleThermalParameters();
    params.blockToOilConductance = 0.0;
    params.oilToAmbientConductance = 0.0;
    params.thermostatOpenTemperature = units::celcius(-100.0);
    params.thermostatFullTemperature = units::celcius(-100.0);

    ThermalModel model;
    model.initialize(params);

    const double start = units::celcius(120.0);
    model.setBlockTemperature(start);

    const double dt = 1e-3;
    const double duration = 20.0;
    const int steps = static_cast<int>(duration / dt);

    for (int i = 0; i < steps; ++i) model.update(dt, 0.0);

    const double tau = params.blockThermalMass / params.radiatorConductance;
    const double expected =
        params.ambientTemperature
        + (start - params.ambientTemperature) * std::exp(-duration / tau);

    EXPECT_NEAR(model.getBlockTemperature(), expected, 0.05);
}

TEST(ThermalModelTests, SteadyStateBalancesHeatAgainstCooling) {
    ThermalModel::Parameters params = simpleThermalParameters();
    params.blockToOilConductance = 0.0;
    params.oilToAmbientConductance = 0.0;
    params.thermostatOpenTemperature = units::celcius(-100.0);
    params.thermostatFullTemperature = units::celcius(-100.0);

    ThermalModel model;
    model.initialize(params);

    const double heatPower = 400.0;
    const double dt = 1e-3;

    for (int i = 0; i < 2000000; ++i) {
        model.addHeat(heatPower * dt);
        model.update(dt, 0.0);
    }

    const double expected =
        params.ambientTemperature + heatPower / params.radiatorConductance;

    EXPECT_NEAR(model.getBlockTemperature(), expected, 0.05);
}

TEST(ThermalModelTests, VehicleSpeedIncreasesCooling) {
    ThermalModel::Parameters params = simpleThermalParameters();
    params.speedCoolingCoefficient = 1.0;
    params.thermostatOpenTemperature = units::celcius(-100.0);
    params.thermostatFullTemperature = units::celcius(-100.0);

    ThermalModel standing;
    ThermalModel moving;
    standing.initialize(params);
    moving.initialize(params);

    standing.setBlockTemperature(units::celcius(100.0));
    moving.setBlockTemperature(units::celcius(100.0));

    const double dt = 1e-3;
    for (int i = 0; i < 10000; ++i) {
        standing.update(dt, 0.0);
        moving.update(dt, 30.0);
    }

    EXPECT_LT(moving.getBlockTemperature(), standing.getBlockTemperature());
}

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

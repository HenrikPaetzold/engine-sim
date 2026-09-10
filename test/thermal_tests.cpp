#include <gtest/gtest.h>

#include "../include/thermal_model.h"
#include "../include/engine.h"
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

    void buildSingleCylinder(Engine *engine) {
        Engine::Parameters params{};
        params.cylinderBanks = 1;
        params.cylinderCount = 1;
        params.crankshaftCount = 1;
        params.exhaustSystemCount = 1;
        params.intakeCount = 1;
        params.throttle = nullptr;

        engine->initialize(params);
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

TEST(WallTemperatureTests, TheThermalModelDoesNotTouchTheCylinderWallOnItsOwn) {
    Engine engine;
    buildSingleCylinder(&engine);

    ASSERT_NEAR(engine.getChamber(0)->m_wallTemperature, units::celcius(90.0), 1e-9);
    ASSERT_LT(engine.getThermalModel().getBlockTemperature(), units::celcius(30.0));

    for (int i = 0; i < 1000; ++i) engine.updateThermal(1e-3, 0.0);

    EXPECT_NEAR(engine.getChamber(0)->m_wallTemperature, units::celcius(90.0), 1e-9)
        << "a script without a powertrain no longer burns against a 90 C wall";

    engine.destroy();
}

TEST(WallTemperatureTests, TheCouplingStillArrivesWhenItIsAskedFor) {
    Engine engine;
    buildSingleCylinder(&engine);

    for (int i = 0; i < 1000; ++i) engine.updateThermal(1e-3, 0.0);
    engine.applyWallTemperature();

    EXPECT_NEAR(
        engine.getChamber(0)->m_wallTemperature,
        engine.getThermalModel().getBlockTemperature(),
        1e-9);

    engine.destroy();
}

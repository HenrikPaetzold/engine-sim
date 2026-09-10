#include <gtest/gtest.h>

#include "../include/engine_friction.h"
#include "../include/thermal_model.h"
#include "../include/combustion_chamber.h"
#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <cmath>

namespace {
    EngineFriction::Parameters oilParameters() {
        EngineFriction::Parameters params;
        params.vogelK = 0.123;
        params.vogelB = 948.0;
        params.vogelTheta = 160.0;
        params.referenceTemperature = units::celcius(100.0);

        return params;
    }

    EngineFriction::Parameters chenFlynnParameters() {
        EngineFriction::Parameters params;
        params.constantFmep = units::pressure(0.3, units::bar);
        params.speedFactor = units::pressure(0.05, units::bar);
        params.speedSquaredFactor = units::pressure(0.002, units::bar);

        return params;
    }
}

TEST(OilViscosityTests, ColdOilIsFarMoreViscousThanHotOil) {
    EngineFriction friction;
    friction.initialize(oilParameters());

    const double cold = friction.viscosity(units::celcius(20.0));
    const double hot = friction.viscosity(units::celcius(100.0));

    EXPECT_GT(cold, hot * 10.0);
    EXPECT_NEAR(hot, 10.5, 1.5);
    EXPECT_NEAR(cold, 152.0, 25.0);
}

TEST(OilViscosityTests, TheRatioIsOneAtTheReferenceTemperature) {
    EngineFriction friction;
    friction.initialize(oilParameters());

    EXPECT_NEAR(friction.viscosityRatio(units::celcius(100.0)), 1.0, 1e-9);
}

TEST(OilViscosityTests, TheRatioFallsMonotonicallyWithTemperature) {
    EngineFriction friction;
    friction.initialize(oilParameters());

    double previous = friction.viscosityRatio(units::celcius(0.0));
    for (double t = 10.0; t <= 200.0; t += 10.0) {
        const double current = friction.viscosityRatio(units::celcius(t));
        EXPECT_LT(current, previous) << t;
        previous = current;
    }
}

TEST(OilViscosityTests, WithoutTheVogelSlopeTheRatioStaysAtOne) {
    EngineFriction friction;
    friction.initialize(EngineFriction::Parameters());

    for (double t = -20.0; t <= 200.0; t += 20.0) {
        EXPECT_EQ(friction.viscosityRatio(units::celcius(t)), 1.0) << t;
    }
}

TEST(ChenFlynnTests, TheDefaultCoefficientsProduceNoFrictionTorque) {
    EngineFriction friction;
    friction.initialize(EngineFriction::Parameters());

    for (double rpm = 0.0; rpm <= 8000.0; rpm += 500.0) {
        EXPECT_EQ(
            friction.crankTorque(
                units::rpm(rpm), 0.086, units::volume(2.0, units::L), 1.0),
            0.0) << rpm;
    }
}

TEST(ChenFlynnTests, TheFrictionTorqueRisesWithEngineSpeed) {
    EngineFriction friction;
    friction.initialize(chenFlynnParameters());

    const double stroke = 0.086;
    const double displacement = units::volume(2.0, units::L);

    double previous = friction.crankTorque(
        units::rpm(500.0), stroke, displacement, 1.0);

    for (double rpm = 1000.0; rpm <= 7000.0; rpm += 500.0) {
        const double current =
            friction.crankTorque(units::rpm(rpm), stroke, displacement, 1.0);
        EXPECT_GT(current, previous) << rpm;
        previous = current;
    }
}

TEST(ChenFlynnTests, TheTorqueFollowsTheFmepDefinition) {
    EngineFriction friction;
    friction.initialize(chenFlynnParameters());

    const double stroke = 0.086;
    const double displacement = units::volume(2.0, units::L);
    const double speed = units::rpm(3000.0);

    const double pressure = friction.fmep(speed, stroke, 1.0);
    const double expected = pressure * displacement / (4.0 * 3.14159265358979323846);

    EXPECT_NEAR(
        friction.crankTorque(speed, stroke, displacement, 1.0),
        expected,
        1e-9);
}

TEST(ChenFlynnTests, TheMeanPistonSpeedFollowsStrokeAndSpeed) {
    EngineFriction friction;
    friction.initialize(EngineFriction::Parameters());

    EXPECT_NEAR(friction.meanPistonSpeed(units::rpm(3000.0), 0.086), 8.6, 0.01);
}

TEST(ChenFlynnTests, ColdOilRaisesTheHydrodynamicTerm) {
    EngineFriction friction;
    EngineFriction::Parameters params = chenFlynnParameters();
    params.constantFmep = 0.0;
    params.speedSquaredFactor = 0.0;
    friction.initialize(params);

    const double warm = friction.fmep(units::rpm(3000.0), 0.086, 1.0);
    const double cold = friction.fmep(units::rpm(3000.0), 0.086, 14.0);

    EXPECT_NEAR(cold, warm * 14.0, warm * 0.01);
}

TEST(ChenFlynnTests, ThePeakPressureTermTracksTheCylinderPressure) {
    EngineFriction friction;
    EngineFriction::Parameters params;
    params.peakPressureFactor = 0.01;
    params.peakPressureDecay = 0.5;
    friction.initialize(params);

    EXPECT_EQ(friction.fmep(units::rpm(3000.0), 0.086, 1.0), 0.0);

    friction.updatePeakPressure(0.001, units::pressure(60.0, units::bar));

    EXPECT_NEAR(
        friction.fmep(units::rpm(3000.0), 0.086, 1.0),
        0.01 * units::pressure(60.0, units::bar),
        1.0);
}

TEST(ChenFlynnTests, ThePeakPressureDecaysWhenTheCylinderIsQuiet) {
    EngineFriction friction;
    EngineFriction::Parameters params;
    params.peakPressureDecay = 0.5;
    friction.initialize(params);

    friction.updatePeakPressure(0.001, units::pressure(60.0, units::bar));
    const double peak = friction.getPeakPressure();

    for (int i = 0; i < 1000; ++i) friction.updatePeakPressure(0.001, 0.0);

    EXPECT_LT(friction.getPeakPressure(), peak * 0.2);
}

TEST(FrictionHeatTests, OilHeatRaisesTheOilTemperatureAlone) {
    ThermalModel model;
    ThermalModel::Parameters params;
    params.blockThermalMass = 1000.0;
    params.oilThermalMass = 500.0;
    params.blockToOilConductance = 0.0;
    params.radiatorConductance = 0.0;
    params.oilToAmbientConductance = 0.0;
    params.oilCoolerConductance = 0.0;
    params.speedCoolingCoefficient = 0.0;
    params.initialBlockTemperature = units::celcius(20.0);
    params.initialOilTemperature = units::celcius(20.0);
    model.initialize(params);

    const double blockBefore = model.getBlockTemperature();

    model.addOilHeat(5000.0);
    model.update(1.0, 0.0);

    EXPECT_NEAR(model.getOilTemperature(), units::celcius(30.0), 1e-6);
    EXPECT_NEAR(model.getBlockTemperature(), blockBefore, 1e-9);
}

TEST(FrictionHeatTests, TheOilCoolerRemovesHeatOnlyAboveItsThermostat) {
    ThermalModel::Parameters params;
    params.blockThermalMass = 1000.0;
    params.oilThermalMass = 500.0;
    params.blockToOilConductance = 0.0;
    params.radiatorConductance = 0.0;
    params.oilToAmbientConductance = 0.0;
    params.oilCoolerConductance = 100.0;
    params.speedCoolingCoefficient = 0.0;
    params.ambientTemperature = units::celcius(20.0);
    params.oilThermostatOpenTemperature = units::celcius(90.0);
    params.oilThermostatFullTemperature = units::celcius(105.0);
    params.initialBlockTemperature = units::celcius(20.0);

    ThermalModel closed;
    params.initialOilTemperature = units::celcius(60.0);
    closed.initialize(params);
    const double before = closed.getOilTemperature();
    closed.update(1.0, 0.0);
    EXPECT_NEAR(closed.getOilTemperature(), before, 1e-9);

    ThermalModel open;
    params.initialOilTemperature = units::celcius(120.0);
    open.initialize(params);
    const double hotBefore = open.getOilTemperature();
    open.update(1.0, 0.0);
    EXPECT_LT(open.getOilTemperature(), hotBefore - 1.0);
}

TEST(FrictionHeatTests, TheOilCoolerIsOffByDefault) {
    ThermalModel::Parameters params;
    EXPECT_EQ(params.oilCoolerConductance, 0.0);

    ThermalModel model;
    params.blockThermalMass = 1000.0;
    params.oilThermalMass = 500.0;
    params.blockToOilConductance = 0.0;
    params.radiatorConductance = 0.0;
    params.oilToAmbientConductance = 0.0;
    params.speedCoolingCoefficient = 0.0;
    params.initialBlockTemperature = units::celcius(20.0);
    params.initialOilTemperature = units::celcius(150.0);
    model.initialize(params);

    const double before = model.getOilTemperature();
    model.update(1.0, 0.0);

    EXPECT_NEAR(model.getOilTemperature(), before, 1e-9);
}

TEST(FrictionHeatTests, TheOilCoolerScalesWithRoadSpeed) {
    ThermalModel::Parameters params;
    params.blockThermalMass = 1000.0;
    params.oilThermalMass = 500.0;
    params.blockToOilConductance = 0.0;
    params.radiatorConductance = 0.0;
    params.oilToAmbientConductance = 0.0;
    params.oilCoolerConductance = 10.0;
    params.speedCoolingCoefficient = 1.0;
    params.ambientTemperature = units::celcius(20.0);
    params.oilThermostatOpenTemperature = units::celcius(90.0);
    params.oilThermostatFullTemperature = units::celcius(105.0);
    params.initialBlockTemperature = units::celcius(20.0);
    params.initialOilTemperature = units::celcius(120.0);

    ThermalModel parked;
    parked.initialize(params);
    parked.update(0.1, 0.0);

    ThermalModel moving;
    moving.initialize(params);
    moving.update(0.1, 30.0);

    EXPECT_LT(moving.getOilTemperature(), parked.getOilTemperature() - 0.5);
}

namespace {
    class FrictionRig : public CombustionChamber {
    public:
        void setFriction(const FrictionModelParams &params) {
            m_frictionModel = params;
        }
    };

    CombustionChamber::FrictionModelParams viscousOnly() {
        CombustionChamber::FrictionModelParams params;
        params.frictionCoeff = 0.0;
        params.breakawayFriction = 0.0;
        params.breakawayFrictionVelocity = 0.1;
        params.viscousFrictionCoefficient = 20.0;
        params.boundaryExponent = 0.0;

        return params;
    }
}

TEST(CylinderFrictionTests, TheViscousTermScalesWithTheViscosityRatio) {
    FrictionRig rig;
    rig.setFriction(viscousOnly());

    rig.setViscosityRatio(1.0);
    const double warm = rig.frictionForce(5.0, 0.0);

    rig.setViscosityRatio(14.0);
    const double cold = rig.frictionForce(5.0, 0.0);

    EXPECT_NEAR(warm, 100.0, 1e-6);
    EXPECT_NEAR(cold, warm * 14.0, 1e-6);
}

TEST(CylinderFrictionTests, TheDefaultRatioLeavesTheForceUnchanged) {
    FrictionRig rig;
    CombustionChamber::FrictionModelParams params;
    rig.setFriction(params);
    rig.setViscosityRatio(1.0);

    const double F_coul = params.frictionCoeff * 400.0;
    const double v_st = params.breakawayFrictionVelocity * 1.4142135623730951;
    const double v_coul = params.breakawayFrictionVelocity / 10.0;
    const double F_0 =
        1.4142135623730951 * 2.718281828459045 * (params.breakawayFriction - F_coul);
    const double F_1 = 5.0 / v_st;
    const double expected =
        F_0 * std::exp(-F_1 * F_1) * F_1
        + F_coul * std::tanh(5.0 / v_coul)
        + params.viscousFrictionCoefficient * 5.0;

    EXPECT_NEAR(rig.frictionForce(5.0, 400.0), expected, 1e-9);
}

TEST(CylinderFrictionTests, AZeroBreakawayVelocityDoesNotProduceNaN) {
    FrictionRig rig;
    CombustionChamber::FrictionModelParams params;
    params.breakawayFrictionVelocity = 0.0;
    rig.setFriction(params);
    rig.setViscosityRatio(1.0);

    for (double v = 0.0; v <= 20.0; v += 1.0) {
        const double F = rig.frictionForce(v, 400.0);
        EXPECT_FALSE(std::isnan(F)) << v;
        EXPECT_FALSE(std::isinf(F)) << v;
    }
}

TEST(CylinderFrictionTests, TheStribeckHumpSitsAboveTheCoulombFloor) {
    FrictionRig rig;
    CombustionChamber::FrictionModelParams params;
    params.frictionCoeff = 0.01;
    params.breakawayFriction = 200.0;
    params.breakawayFrictionVelocity = 0.1;
    params.viscousFrictionCoefficient = 0.0;
    rig.setFriction(params);
    rig.setViscosityRatio(1.0);

    const double coulomb = params.frictionCoeff * 400.0;
    const double hump = rig.frictionForce(0.1, 400.0);
    const double fast = rig.frictionForce(10.0, 400.0);

    EXPECT_GT(hump, coulomb * 2.0);
    EXPECT_NEAR(fast, coulomb, coulomb * 0.05);
}

TEST(CylinderFrictionTests, ThinningOilRaisesTheBoundaryFriction) {
    FrictionRig rig;
    CombustionChamber::FrictionModelParams params;
    params.frictionCoeff = 0.06;
    params.breakawayFriction = 50.0;
    params.breakawayFrictionVelocity = 0.1;
    params.viscousFrictionCoefficient = 0.0;
    params.boundaryExponent = 0.5;
    rig.setFriction(params);

    rig.setViscosityRatio(1.0);
    const double warm = rig.frictionForce(10.0, 400.0);

    rig.setViscosityRatio(0.25);
    const double scorching = rig.frictionForce(10.0, 400.0);

    EXPECT_NEAR(scorching, warm * 2.0, warm * 0.01);
}

TEST(CylinderFrictionTests, WithoutTheBoundaryExponentTheViscosityLeavesItAlone) {
    FrictionRig rig;
    CombustionChamber::FrictionModelParams params;
    params.viscousFrictionCoefficient = 0.0;
    params.boundaryExponent = 0.0;
    rig.setFriction(params);

    rig.setViscosityRatio(1.0);
    const double warm = rig.frictionForce(10.0, 400.0);

    rig.setViscosityRatio(0.25);
    EXPECT_NEAR(rig.frictionForce(10.0, 400.0), warm, 1e-12);
}

TEST(FrictionRegistryTests, EveryFrictionParameterIsReachable) {
    EngineFriction friction;
    friction.initialize(EngineFriction::Parameters());

    config::ParameterRegistry registry;
    friction.registerParameters(&registry);

    for (const char *path : {
        "friction.constant_fmep",
        "friction.peak_pressure_factor",
        "friction.speed_factor",
        "friction.speed_squared_factor",
        "friction.vogel_k",
        "friction.vogel_b",
        "friction.vogel_theta",
        "friction.reference_temperature",
        "friction.peak_pressure_decay",
        "friction.heat_to_oil" })
    {
        EXPECT_TRUE(registry.contains(path)) << path;
    }

    EXPECT_EQ(registry.getCount(), 10);
}

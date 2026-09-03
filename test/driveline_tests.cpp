#include <gtest/gtest.h>

#include "../include/ratio_clutch_constraint.h"
#include "../include/torque_converter_constraint.h"
#include "../include/transmission.h"
#include "../include/vehicle.h"
#include "../include/units.h"

#include <cmath>

namespace {
    class ConstantTorque : public atg_scs::ForceGenerator {
    public:
        ConstantTorque(atg_scs::RigidBody *body, double torque)
            : m_body(body), m_torque(torque) { /* void */ }

        virtual void apply(atg_scs::SystemState *state) override {
            state->t[m_body->index] += m_torque;
        }

        atg_scs::RigidBody *m_body;
        double m_torque;
    };

    struct TwoBodyRig {
        atg_scs::OptimizedNsvRigidBodySystem system;
        atg_scs::RigidBody input;
        atg_scs::RigidBody output;

        TwoBodyRig(double inputInertia, double outputInertia) {
            input.reset();
            output.reset();
            input.m = output.m = 1.0;
            input.I = inputInertia;
            output.I = outputInertia;

            system.initialize(new atg_scs::GaussSeidelSleSolver);
            system.addRigidBody(&input);
            system.addRigidBody(&output);
        }
    };
}

TEST(RatioClutchTests, ALockedClutchHoldsTheSpeedRatio) {
    TwoBodyRig rig(0.2, 2.0);

    RatioClutchConstraint clutch;
    clutch.setInput(&rig.input);
    clutch.setOutput(&rig.output);
    clutch.m_ratio = 3.0;
    clutch.m_capacity = 1000.0;
    clutch.m_pressure = 1.0;
    rig.system.addConstraint(&clutch);

    ConstantTorque drive(&rig.input, 50.0);
    rig.system.addForceGenerator(&drive);

    for (int i = 0; i < 2000; ++i) rig.system.process(1e-4, 1);

    EXPECT_GT(rig.input.v_theta, 1.0);
    EXPECT_NEAR(rig.input.v_theta / rig.output.v_theta, 3.0, 1e-3);
}

TEST(RatioClutchTests, TheRatioMultipliesTheOutputTorque) {
    TwoBodyRig rig(0.2, 2.0);

    RatioClutchConstraint clutch;
    clutch.setInput(&rig.input);
    clutch.setOutput(&rig.output);
    clutch.m_ratio = 3.0;
    clutch.m_capacity = 1000.0;
    clutch.m_pressure = 1.0;
    rig.system.addConstraint(&clutch);

    ConstantTorque drive(&rig.input, 50.0);
    rig.system.addForceGenerator(&drive);

    for (int i = 0; i < 2000; ++i) rig.system.process(1e-4, 1);

    EXPECT_NEAR(
        clutch.F_t[0][1] / clutch.F_t[0][0],
        -3.0,
        1e-6);
}

TEST(RatioClutchTests, TheEffectiveInertiaFollowsTheSquareOfTheRatio) {
    const double inputInertia = 0.2;
    const double outputInertia = 2.0;
    const double ratio = 3.0;
    const double torque = 50.0;
    const double duration = 0.2;

    TwoBodyRig rig(inputInertia, outputInertia);

    RatioClutchConstraint clutch;
    clutch.setInput(&rig.input);
    clutch.setOutput(&rig.output);
    clutch.m_ratio = ratio;
    clutch.m_capacity = 1e6;
    clutch.m_pressure = 1.0;
    rig.system.addConstraint(&clutch);

    ConstantTorque drive(&rig.input, torque);
    rig.system.addForceGenerator(&drive);

    const int steps = 2000;
    const double dt = duration / steps;
    for (int i = 0; i < steps; ++i) rig.system.process(dt, 1);

    const double effectiveInertia =
        inputInertia + outputInertia / (ratio * ratio);
    const double expected = torque / effectiveInertia * duration;

    EXPECT_NEAR(rig.input.v_theta, expected, expected * 1e-3);
}

TEST(RatioClutchTests, ASlippingClutchCapsTheTransmittedTorque) {
    TwoBodyRig rig(0.2, 2.0);

    RatioClutchConstraint clutch;
    clutch.setInput(&rig.input);
    clutch.setOutput(&rig.output);
    clutch.m_ratio = 1.0;
    clutch.m_capacity = 40.0;
    clutch.m_pressure = 0.5;
    rig.system.addConstraint(&clutch);

    ConstantTorque drive(&rig.input, 200.0);
    rig.system.addForceGenerator(&drive);

    for (int i = 0; i < 2000; ++i) rig.system.process(1e-4, 1);

    EXPECT_NEAR(std::abs(clutch.F_t[0][0]), 20.0, 1e-6);
    EXPECT_GT(clutch.getSlipSpeed(), 1.0);
    EXPECT_GT(clutch.getSlipPower(), 0.0);
}

TEST(RatioClutchTests, ZeroPressureOpensTheDriveline) {
    TwoBodyRig rig(0.2, 2.0);

    RatioClutchConstraint clutch;
    clutch.setInput(&rig.input);
    clutch.setOutput(&rig.output);
    clutch.m_ratio = 2.0;
    clutch.m_capacity = 1000.0;
    clutch.m_pressure = 0.0;
    rig.system.addConstraint(&clutch);

    ConstantTorque drive(&rig.input, 50.0);
    rig.system.addForceGenerator(&drive);

    for (int i = 0; i < 2000; ++i) rig.system.process(1e-4, 1);

    EXPECT_NEAR(rig.output.v_theta, 0.0, 1e-9);
    EXPECT_GT(rig.input.v_theta, 1.0);
}

TEST(TorqueConverterTests, TheStallTorqueRatioMatchesTheCurve) {
    TorqueConverterConstraint converter;

    EXPECT_NEAR(converter.torqueRatio(0.0), 2.0, 1e-12);
    EXPECT_NEAR(converter.torqueRatio(converter.m_couplingPoint), 1.0, 1e-12);
    EXPECT_NEAR(converter.torqueRatio(1.0), 1.0, 1e-12);

    EXPECT_LT(converter.torqueRatio(0.4), converter.torqueRatio(0.2));
}

TEST(TorqueConverterTests, TheCapacityVanishesAtTheCouplingSpeed) {
    TorqueConverterConstraint converter;

    EXPECT_NEAR(converter.capacityFactor(0.0), converter.m_capacityFactor, 1e-12);
    EXPECT_NEAR(converter.capacityFactor(1.0), 0.0, 1e-12);
    EXPECT_LT(converter.capacityFactor(0.8), converter.capacityFactor(0.2));
}

TEST(TorqueConverterTests, TheTurbineMultipliesTorqueAtStall) {
    TwoBodyRig rig(0.5, 1e9);

    TorqueConverterConstraint converter;
    converter.setPump(&rig.input);
    converter.setTurbine(&rig.output);
    rig.system.addConstraint(&converter);

    ConstantTorque drive(&rig.input, 200.0);
    rig.system.addForceGenerator(&drive);

    for (int i = 0; i < 4000; ++i) rig.system.process(1e-4, 1);

    ASSERT_NEAR(converter.getSpeedRatio(), 0.0, 1e-3);
    EXPECT_NEAR(
        converter.getTurbineTorque() / converter.getPumpTorque(),
        -converter.m_stallTorqueRatio,
        1e-6);
}

TEST(TorqueConverterTests, TheStallSpeedFollowsTheCapacityFactor) {
    TwoBodyRig rig(0.5, 1e9);

    TorqueConverterConstraint converter;
    converter.setPump(&rig.input);
    converter.setTurbine(&rig.output);
    rig.system.addConstraint(&converter);

    const double torque = 200.0;
    ConstantTorque drive(&rig.input, torque);
    rig.system.addForceGenerator(&drive);

    for (int i = 0; i < 20000; ++i) rig.system.process(1e-4, 1);

    const double expected = std::sqrt(torque / converter.m_capacityFactor);
    EXPECT_NEAR(rig.input.v_theta, expected, expected * 1e-2);
}

TEST(TorqueConverterTests, TheConverterCouplesAsTheTurbineCatchesUp) {
    TwoBodyRig rig(0.5, 2.0);

    TorqueConverterConstraint converter;
    converter.setPump(&rig.input);
    converter.setTurbine(&rig.output);
    rig.system.addConstraint(&converter);

    ConstantTorque drive(&rig.input, 100.0);
    rig.system.addForceGenerator(&drive);

    double previousRatio = 0.0;
    double previousTorqueRatio = converter.m_stallTorqueRatio;

    for (int block = 0; block < 20; ++block) {
        for (int i = 0; i < 2000; ++i) rig.system.process(1e-4, 1);

        const double ratio = converter.getSpeedRatio();
        const double tr = converter.torqueRatio(ratio);

        EXPECT_GE(ratio, previousRatio - 1e-9);
        EXPECT_LE(tr, previousTorqueRatio + 1e-9);

        previousRatio = ratio;
        previousTorqueRatio = tr;
    }

    EXPECT_GT(previousRatio, 0.75);
    EXPECT_LT(previousTorqueRatio, 1.1);
    EXPECT_LT(
        std::abs(converter.getTurbineTorque()),
        converter.m_stallTorqueRatio * std::abs(converter.getPumpTorque()));
}

namespace {
    Transmission::Parameters gearboxParameters(Transmission::Type type) {
        static const double ratios[] = { 3.60, 2.19, 1.41, 1.00, 0.83, 0.69 };

        Transmission::Parameters params;
        params.GearCount = 6;
        params.GearRatios = ratios;
        params.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        params.GearboxType = type;

        return params;
    }

    Vehicle::Parameters vehicleParameters() {
        Vehicle::Parameters params;
        params.mass = 1597.0;
        params.dragCoefficient = 0.25;
        params.crossSectionArea = 2.2;
        params.diffRatio = 3.42;
        params.tireRadius = units::distance(10.0, units::inch);
        params.rollingResistance = 2000.0;

        return params;
    }
}

TEST(GearboxModelTests, TheConstantDrivelineInertiaReproducesTheLegacyGearInertia) {
    Vehicle vehicle;
    vehicle.initialize(vehicleParameters());

    Transmission gearbox;
    gearbox.initialize(gearboxParameters(Transmission::Type::Manual));

    atg_scs::RigidBody driveline;
    driveline.reset();
    gearbox.bind(&driveline, &vehicle, nullptr);

    for (int gear = 0; gear < gearbox.getGearCount(); ++gear) {
        const double ratio = gearbox.getGearRatio(gear);
        const double f =
            vehicle.getTireRadius() / (vehicle.getDiffRatio() * ratio);
        const double legacyInertia = vehicle.getMass() * f * f;

        EXPECT_NEAR(
            gearbox.getDrivelineInertia() / (ratio * ratio),
            legacyInertia,
            legacyInertia * 1e-12) << "gear " << gear;
    }
}

TEST(GearboxModelTests, AGearChangeNoLongerRewritesTheDrivelineState) {
    Vehicle vehicle;
    vehicle.initialize(vehicleParameters());

    Transmission gearbox;
    gearbox.initialize(gearboxParameters(Transmission::Type::Manual));

    atg_scs::RigidBody driveline;
    driveline.reset();
    gearbox.bind(&driveline, &vehicle, nullptr);
    gearbox.update(1e-3);

    driveline.v_theta = 42.0;
    const double inertia = driveline.I;

    gearbox.changeGear(0);
    gearbox.update(1e-3);
    EXPECT_NEAR(driveline.v_theta, 42.0, 1e-12);
    EXPECT_NEAR(driveline.I, inertia, 1e-12);

    gearbox.changeGear(3);
    gearbox.update(1e-3);
    EXPECT_NEAR(driveline.v_theta, 42.0, 1e-12);
    EXPECT_NEAR(driveline.I, inertia, 1e-12);
}

TEST(GearboxModelTests, TheLegacyGearboxStillSwapsTheInertia) {
    Vehicle vehicle;
    vehicle.initialize(vehicleParameters());

    Transmission gearbox;
    gearbox.initialize(gearboxParameters(Transmission::Type::Legacy));

    atg_scs::RigidBody driveline;
    driveline.reset();
    driveline.I = 1.0;
    gearbox.bind(&driveline, &vehicle, nullptr);

    gearbox.changeGear(0);
    const double first = driveline.I;

    gearbox.changeGear(5);
    EXPECT_GT(driveline.I, first);
}

TEST(GearboxModelTests, TheCapabilitiesFollowTheGearboxType) {
    Transmission manual;
    manual.initialize(gearboxParameters(Transmission::Type::Manual));
    EXPECT_TRUE(manual.requiresTorqueInterrupt());
    EXPECT_FALSE(manual.supportsPreselect());
    EXPECT_FALSE(manual.hasLaunchDevice());

    Transmission dct;
    dct.initialize(gearboxParameters(Transmission::Type::DualClutch));
    EXPECT_FALSE(dct.requiresTorqueInterrupt());
    EXPECT_TRUE(dct.supportsPreselect());
    EXPECT_FALSE(dct.hasLaunchDevice());

    Transmission converter;
    converter.initialize(gearboxParameters(Transmission::Type::Converter));
    EXPECT_FALSE(converter.requiresTorqueInterrupt());
    EXPECT_FALSE(converter.supportsPreselect());
    EXPECT_TRUE(converter.hasLaunchDevice());
}

TEST(GearboxModelTests, TheSecondClutchOnlyCarriesTorqueOnADualClutchGearbox) {
    Vehicle vehicle;
    vehicle.initialize(vehicleParameters());

    Transmission manual;
    manual.initialize(gearboxParameters(Transmission::Type::Manual));

    atg_scs::RigidBody drivelineA;
    drivelineA.reset();
    manual.bind(&drivelineA, &vehicle, nullptr);
    manual.changeGear(1);
    manual.setPreselectedGear(2);
    manual.setClutchPressure(0, 1.0);
    manual.setClutchPressure(1, 1.0);
    manual.update(1e-3);

    EXPECT_NEAR(manual.getClutchCapacity(1), 0.0, 1e-12);

    Transmission dct;
    dct.initialize(gearboxParameters(Transmission::Type::DualClutch));

    atg_scs::RigidBody drivelineB;
    drivelineB.reset();
    dct.bind(&drivelineB, &vehicle, nullptr);
    dct.changeGear(1);
    dct.setPreselectedGear(2);
    dct.setClutchPressure(0, 0.4);
    dct.setClutchPressure(1, 0.6);
    dct.update(1e-3);

    EXPECT_NEAR(dct.getClutchRatio(0), dct.getGearRatio(1), 1e-12);
    EXPECT_NEAR(dct.getClutchRatio(1), dct.getGearRatio(2), 1e-12);
    EXPECT_GT(dct.getClutchCapacity(1), 0.0);
}

#include <gtest/gtest.h>

#include "../include/ratio_clutch_constraint.h"
#include "../include/torque_converter_constraint.h"
#include "../include/powertrain/powertrain_unit.h"
#include "../include/config/config_server.h"
#include "../include/transmission.h"
#include "../include/vehicle.h"
#include "../include/vehicle_drag_constraint.h"
#include "../include/powertrain/selector_gate.h"
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

TEST(TelemetryTests, TheSampleIsAvailableWithoutAConfigServer) {
    powertrain::PowertrainUnit unit;
    unit.initialize(
        powertrain::EngineControlUnit::Parameters(),
        powertrain::TransmissionControlUnit::Parameters());

    powertrain::PowertrainState state;
    state.coolantTemperature = units::celcius(90.0);
    state.engineRpm = 3000.0;
    state.engineSpeed = units::rpm(3000.0);
    state.engineRunning = true;
    state.gear = 1;

    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.5;

    powertrain::ActuatorCommands commands;
    unit.update(1e-3, state, inputs, &commands);

    config::TelemetrySample sample;
    unit.fillTelemetry(&sample);

    EXPECT_FALSE(sample.engineState.empty());
    EXPECT_FALSE(sample.shiftState.empty());
    EXPECT_GT(sample.torqueRequest, 0.0);
}

namespace {
    class DrivelineRig {
        public:
            DrivelineRig(double mass, double grade, double rolling) {
                Vehicle::Parameters params = vehicleParameters();
                params.mass = mass;
                params.rollingResistance = rolling;
                params.dragCoefficient = 0.0;
                m_vehicle.initialize(params);

                m_body.reset();
                m_body.m = 1.0;
                m_body.I = 1.0;

                m_system.initialize(new atg_scs::GaussSeidelSleSolver);
                m_system.addRigidBody(&m_body);

                m_vehicle.addToSystem(&m_system, &m_body);
                m_vehicle.setRoadGrade(grade);

                const double f = params.tireRadius / params.diffRatio;
                m_body.I = params.mass * f * f;

                m_drag.initialize(&m_body, &m_vehicle);
                m_system.addConstraint(&m_drag);
            }

            void run(int steps, double dt = 1e-3) {
                for (int i = 0; i < steps; ++i) m_system.process(dt, 1);
            }

            atg_scs::OptimizedNsvRigidBodySystem m_system;
            atg_scs::RigidBody m_body;
            Vehicle m_vehicle;
            VehicleDragConstraint m_drag;
    };
}

TEST(VehicleDragTests, DragOpposesMotionInBothDirections) {
    DrivelineRig forward(1500.0, 0.0, 400.0);
    forward.m_body.v_theta = -5.0;
    forward.run(2000);

    DrivelineRig backward(1500.0, 0.0, 400.0);
    backward.m_body.v_theta = 5.0;
    backward.run(2000);

    EXPECT_GT(forward.m_body.v_theta, -5.0) << "forward drag did not slow the car";
    EXPECT_LT(backward.m_body.v_theta, 5.0) << "reverse drag did not slow the car";

    EXPECT_NEAR(
        std::abs(forward.m_body.v_theta),
        std::abs(backward.m_body.v_theta),
        1e-6) << "drag is asymmetric between forward and reverse";
}

TEST(VehicleDragTests, TheSignedSpeedIsPositiveWhenDrivingForward) {
    DrivelineRig rig(1500.0, 0.0, 400.0);

    rig.m_body.v_theta = -5.0;
    EXPECT_GT(rig.m_vehicle.getSignedSpeed(), 0.0);

    rig.m_body.v_theta = 5.0;
    EXPECT_LT(rig.m_vehicle.getSignedSpeed(), 0.0);
}

TEST(VehicleDragTests, RollingResistanceHoldsTheCarOnAGentleGrade) {
    DrivelineRig rig(1500.0, 0.02, 2000.0);
    rig.run(4000);

    EXPECT_NEAR(rig.m_vehicle.getSpeed(), 0.0, 1e-3);
}

TEST(VehicleDragTests, ASteepGradeOvercomesRollingResistance) {
    DrivelineRig rig(1500.0, 0.20, 400.0);
    rig.run(4000);

    EXPECT_GT(rig.m_vehicle.getSpeed(), 0.1);
}

TEST(ParkLockTests, TheParkLockHoldsAgainstAGradeAndSlipsBeyondItsTorque) {
    Vehicle::Parameters vp = vehicleParameters();
    vp.rollingResistance = 0.0;
    vp.dragCoefficient = 0.0;

    static const double ratios[] = { 3.6, 2.1, 1.4, 1.0, 0.8, 0.7 };
    Transmission::Parameters tp;
    tp.GearCount = 6;
    tp.GearRatios = ratios;
    tp.MaxClutchTorque = units::torque(1000.0, units::Nm);
    tp.GearboxType = Transmission::Type::Manual;

    for (int strong = 0; strong < 2; ++strong) {
        Vehicle vehicle;
        vehicle.initialize(vp);

        Transmission::Parameters params = tp;
        params.ParkLockTorque = strong
            ? units::torque(20000.0, units::Nm)
            : units::torque(1.0, units::Nm);

        Transmission gearbox;
        gearbox.initialize(params);

        atg_scs::OptimizedNsvRigidBodySystem system;
        atg_scs::RigidBody body;
        body.reset();
        body.m = 1.0;
        body.I = 1.0;
        system.initialize(new atg_scs::GaussSeidelSleSolver);
        system.addRigidBody(&body);

        vehicle.addToSystem(&system, &body);
        vehicle.setRoadGrade(0.15);

        const double f = vp.tireRadius / vp.diffRatio;
        body.I = vp.mass * f * f;

        VehicleDragConstraint drag;
        drag.initialize(&body, &vehicle);
        system.addConstraint(&drag);

        gearbox.bind(&body, &vehicle, nullptr);
        gearbox.addParkLockForTest(&system);
        gearbox.setEngagement(powertrain::GateEngagement::Park);

        for (int i = 0; i < 3000; ++i) {
            gearbox.update(1e-3);
            system.process(1e-3, 1);
        }

        if (strong) {
            EXPECT_NEAR(vehicle.getSpeed(), 0.0, 1e-2) << "strong park lock let go";
        }
        else {
            EXPECT_GT(vehicle.getSpeed(), 0.1) << "weak park lock did not slip";
        }
    }
}

namespace {
    class CreepRig {
        public:
            CreepRig(double brake) {
                Vehicle::Parameters vp = vehicleParameters();
                vp.dragCoefficient = 0.0;
                vp.rollingResistance = 300.0;
                vp.maxBrakeForce = 12000.0;
                m_vehicle.initialize(vp);

                m_crank.reset();
                m_crank.m = 1.0;
                m_crank.I = 0.2;

                m_turbine.reset();
                m_turbine.m = 1.0;
                m_turbine.I = 0.08;

                m_driveline.reset();
                m_driveline.m = 1.0;
                m_driveline.I = 1.0;

                m_system.initialize(new atg_scs::GaussSeidelSleSolver);
                m_system.addRigidBody(&m_crank);
                m_system.addRigidBody(&m_turbine);
                m_system.addRigidBody(&m_driveline);

                m_vehicle.addToSystem(&m_system, &m_driveline);
                m_vehicle.setBrake(brake);

                const double f = vp.tireRadius / vp.diffRatio;
                m_driveline.I = vp.mass * f * f;

                m_converter.m_stallTorqueRatio = 2.0;
                m_converter.m_couplingPoint = 0.85;
                m_converter.m_capacityFactor = 4.04e-3;
                m_converter.setPump(&m_crank);
                m_converter.setTurbine(&m_turbine);
                m_system.addConstraint(&m_converter);

                m_clutch.setInput(&m_turbine);
                m_clutch.setOutput(&m_driveline);
                m_clutch.m_ratio = 3.60;
                m_clutch.m_capacity = units::torque(1000.0, units::Nm);
                m_clutch.m_pressure = 1.0;
                m_system.addConstraint(&m_clutch);

                m_drag.initialize(&m_driveline, &m_vehicle);
                m_system.addConstraint(&m_drag);
            }

            void run(int steps) {
                for (int i = 0; i < steps; ++i) {
                    m_crank.v_theta = -units::rpm(800.0);
                    m_system.process(1e-3, 1);
                }
            }

            atg_scs::OptimizedNsvRigidBodySystem m_system;
            atg_scs::RigidBody m_crank;
            atg_scs::RigidBody m_turbine;
            atg_scs::RigidBody m_driveline;
            Vehicle m_vehicle;
            TorqueConverterConstraint m_converter;
            RatioClutchConstraint m_clutch;
            VehicleDragConstraint m_drag;
    };
}

TEST(TorqueConverterTests, TheConverterCreepsForwardWithoutThrottle) {
    CreepRig rolling(0.0);
    rolling.run(3000);

    EXPECT_GT(rolling.m_vehicle.getSignedSpeed(), 0.5)
        << "the converter did not creep in a forward detent";

    CreepRig held(1.0);
    held.run(3000);

    EXPECT_NEAR(held.m_vehicle.getSpeed(), 0.0, 1e-2)
        << "the brake did not hold against the stall torque";
}

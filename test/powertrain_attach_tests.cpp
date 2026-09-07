#include <gtest/gtest.h>

#include "../include/powertrain_system.h"
#include "../include/simulator.h"
#include "../include/engine.h"
#include "../include/vehicle.h"
#include "../include/transmission.h"
#include "../include/powertrain/passthrough_controller.h"
#include "../include/units.h"

namespace {
    class FakeSimulator : public Simulator {
        public:
            virtual void writeToSynthesizer() override { /* void */ }
    };

    class TestablePowertrain : public PowertrainSystem {
        public:
            using PowertrainSystem::applyCommands;
            using PowertrainSystem::m_commands;
    };

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

    class Rig {
        public:
            Rig(Transmission::Type type) {
                vehicle.initialize(vehicleParameters());

                driveline.reset();
                driveline.m = 1.0;
                driveline.I = 1.0;
                vehicle.addToSystem(nullptr, &driveline);

                gearbox.initialize(gearboxParameters(type));
                gearbox.bind(&driveline, &vehicle, &engine);

                simulator.loadSimulation(&engine, &vehicle, &gearbox);

                controller.initialize(powertrain::PassthroughController::Parameters());

                system.initialize(PowertrainSystem::Parameters());
                system.setController(&controller);
                system.attach(&simulator);
            }

            ~Rig() {
                system.detach();
            }

            Engine engine;
            Vehicle vehicle;
            Transmission gearbox;
            atg_scs::RigidBody driveline;
            FakeSimulator simulator;
            powertrain::PassthroughController controller;
            TestablePowertrain system;
    };
}

TEST(PowertrainAttachTests, TheSystemReallyReachesTheSimulation) {
    Rig rig(Transmission::Type::Manual);

    ASSERT_TRUE(rig.system.isActive());
    ASSERT_NE(rig.engine.getThrottleController(), nullptr);

    rig.system.m_commands.throttlePlate = 0.4;
    rig.system.applyCommands();
    rig.engine.update(1e-3);

    const double open = rig.engine.getThrottle();

    rig.system.m_commands.throttlePlate = 0.9;
    rig.system.applyCommands();
    rig.engine.update(1e-3);

    EXPECT_LT(rig.engine.getThrottle(), open)
        << "the commanded plate never reached the engine";
}

TEST(PowertrainAttachTests, TheIgnitionCommandsReachTheModule) {
    Rig rig(Transmission::Type::Manual);

    IgnitionModule *ignition = rig.engine.getIgnitionModule();
    ASSERT_NE(ignition, nullptr);

    rig.system.m_commands.ignitionEnabled = false;
    rig.system.m_commands.ignitionCutFraction = 0.4;
    rig.system.m_commands.timingOffset = units::angle(3.0, units::deg);
    rig.system.applyCommands();

    EXPECT_FALSE(ignition->m_enabled);
    EXPECT_NEAR(ignition->getCutFraction(), 0.4, 1e-12);
    EXPECT_NEAR(ignition->getTimingOffset(), units::angle(3.0, units::deg), 1e-12);
}

TEST(PowertrainAttachTests, TheCommandedTimingAdvanceReachesTheSpark) {
    Rig rig(Transmission::Type::Manual);

    IgnitionModule *ignition = rig.engine.getIgnitionModule();

    rig.system.m_commands.timingAdvance = units::angle(22.0, units::deg);
    rig.system.m_commands.timingAdvanceValid = true;
    rig.system.m_commands.timingOffset = 0.0;
    rig.system.applyCommands();

    ASSERT_TRUE(ignition->hasTimingOverride());
    EXPECT_NEAR(
        ignition->getTimingAdvance(), units::angle(22.0, units::deg), 1e-12)
        << "the scripted timing map never reaches the spark";

    rig.system.m_commands.timingAdvanceValid = false;
    rig.system.applyCommands();

    EXPECT_FALSE(ignition->hasTimingOverride());
}

TEST(PowertrainAttachTests, ZeroLimitsAreADoNotTouchSentinel) {
    Rig rig(Transmission::Type::Manual);

    IgnitionModule *ignition = rig.engine.getIgnitionModule();

    ignition->setRevLimit(units::rpm(7000.0));
    ignition->setLimiterDuration(0.25);

    rig.system.m_commands.revLimit = 0.0;
    rig.system.m_commands.limiterDuration = 0.0;
    rig.system.applyCommands();

    EXPECT_NEAR(ignition->getRevLimit(), units::rpm(7000.0), 1e-9);
    EXPECT_NEAR(ignition->getLimiterDuration(), 0.25, 1e-12);

    rig.system.m_commands.revLimit = units::rpm(5000.0);
    rig.system.m_commands.limiterDuration = 0.5;
    rig.system.applyCommands();

    EXPECT_NEAR(ignition->getRevLimit(), units::rpm(5000.0), 1e-9);
    EXPECT_NEAR(ignition->getLimiterDuration(), 0.5, 1e-12);
}

TEST(PowertrainAttachTests, TheFuelFactorCombinesCutAndEnrichment) {
    Rig rig(Transmission::Type::Manual);

    rig.system.m_commands.fuelCutFraction = 0.25;
    rig.system.m_commands.fuelEnrichment = 1.2;
    rig.system.applyCommands();

    EXPECT_NEAR(rig.engine.getFuelFactor(), 0.75 * 1.2, 1e-12);

    rig.system.m_commands.fuelCutFraction = 1.0;
    rig.system.applyCommands();

    EXPECT_NEAR(rig.engine.getFuelFactor(), 0.0, 1e-12);

    rig.system.m_commands.fuelCutFraction = 0.0;
    rig.system.m_commands.fuelEnrichment = 9.0;
    rig.system.applyCommands();

    EXPECT_NEAR(rig.engine.getFuelFactor(), 4.0, 1e-12);
}

TEST(PowertrainAttachTests, TheClutchPressuresReachTheGearboxAndAreClamped) {
    Rig rig(Transmission::Type::DualClutch);

    rig.system.m_commands.clutchPressure[0] = 0.3;
    rig.system.m_commands.clutchPressure[1] = 1.8;
    rig.system.applyCommands();

    EXPECT_NEAR(rig.gearbox.getClutchPressure(0), 0.3, 1e-12);
    EXPECT_NEAR(rig.gearbox.getClutchPressure(1), 1.0, 1e-12);
}

TEST(PowertrainAttachTests, PreselectIsSilentlyDroppedWithoutSupport) {
    Rig manual(Transmission::Type::Manual);

    ASSERT_FALSE(manual.gearbox.supportsPreselect());

    manual.system.m_commands.preselectGear = 3;
    manual.system.applyCommands();

    EXPECT_NE(manual.gearbox.getPreselectedGear(), 3)
        << "a manual gearbox accepted a preselect it cannot honour";

    Rig dct(Transmission::Type::DualClutch);

    ASSERT_TRUE(dct.gearbox.supportsPreselect());

    dct.system.m_commands.preselectGear = 3;
    dct.system.applyCommands();

    EXPECT_EQ(dct.gearbox.getPreselectedGear(), 3);
}

TEST(PowertrainAttachTests, TheTargetGearReachesTheGearbox) {
    Rig rig(Transmission::Type::Manual);

    rig.system.m_commands.targetGear = 2;
    rig.system.applyCommands();

    EXPECT_EQ(rig.gearbox.getGear(), 2);
}

TEST(PowertrainAttachTests, TheStarterCommandReachesTheMotor) {
    Rig rig(Transmission::Type::Manual);

    rig.system.m_commands.starterEnabled = true;
    rig.system.applyCommands();

    EXPECT_TRUE(rig.simulator.m_starterMotor.m_enabled);

    rig.system.m_commands.starterEnabled = false;
    rig.system.applyCommands();

    EXPECT_FALSE(rig.simulator.m_starterMotor.m_enabled);
}

TEST(PowertrainAttachTests, TheStateMirrorsTheSimulationBack) {
    Rig rig(Transmission::Type::Manual);

    rig.gearbox.changeGear(3);
    rig.system.sampleState(1e-3);

    EXPECT_EQ(rig.system.getState().gear, 3);
    EXPECT_EQ(rig.system.getState().gearCount, rig.gearbox.getGearCount());
}

TEST(PowertrainAttachTests, TheParkLockActuatorReachesTheGearbox) {
    Rig rig(Transmission::Type::DualClutch);

    ASSERT_FALSE(rig.gearbox.isParkLockEngaged());

    rig.system.m_commands.engagement = powertrain::GateEngagement::Forward;
    rig.system.m_commands.parkLock = true;
    rig.system.applyCommands();

    EXPECT_TRUE(rig.gearbox.isParkLockEngaged())
        << "the park lock actuator channel goes nowhere";

    rig.system.m_commands.parkLock = false;
    rig.system.applyCommands();

    EXPECT_FALSE(rig.gearbox.isParkLockEngaged());
}

TEST(PowertrainAttachTests, TheGateStillEngagesTheParkLockOnItsOwn) {
    Rig rig(Transmission::Type::DualClutch);

    rig.system.m_commands.engagement = powertrain::GateEngagement::Park;
    rig.system.m_commands.parkLock = false;
    rig.system.applyCommands();

    EXPECT_TRUE(rig.gearbox.isParkLockEngaged());
}

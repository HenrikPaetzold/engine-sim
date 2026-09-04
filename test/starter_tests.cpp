#include <gtest/gtest.h>

#include "../include/starter_motor.h"
#include "../include/units.h"
#include "../include/crankshaft.h"
#include "../include/scs.h"
#include "../include/engine.h"

#include <cmath>

namespace {
    void buildTorqueMap(control::Map2d *map, double cold, double warm, double freeSpeed) {
        map->initialize(3, 2, 0.0);

        map->setXAxis(0, 0.0);
        map->setXAxis(1, freeSpeed * 0.5);
        map->setXAxis(2, freeSpeed);

        map->setYAxis(0, units::celcius(-20.0));
        map->setYAxis(1, units::celcius(80.0));

        map->setValue(0, 0, cold);
        map->setValue(1, 0, cold * 0.5);
        map->setValue(2, 0, 0.0);

        map->setValue(0, 1, warm);
        map->setValue(1, 1, warm * 0.5);
        map->setValue(2, 1, 0.0);
    }
}

TEST(StarterMotorTests, WithoutAMapTheConstantsStillApply) {
    StarterMotor starter;
    starter.m_maxTorque = units::torque(80.0, units::ft_lb);
    starter.m_rotationSpeed = -units::rpm(200.0);

    EXPECT_NEAR(starter.availableTorque(0.0), units::torque(80.0, units::ft_lb), 1e-9);
    EXPECT_NEAR(
        starter.availableTorque(units::rpm(500.0)),
        units::torque(80.0, units::ft_lb),
        1e-9) << "the constant must not depend on speed";
    EXPECT_NEAR(starter.targetSpeed(), -units::rpm(200.0), 1e-9);
}

TEST(StarterMotorTests, TheTorqueFallsWithSpeed) {
    StarterMotor starter;
    starter.m_temperature = units::celcius(80.0);
    buildTorqueMap(
        &starter.m_torqueMap,
        units::torque(110.0, units::Nm),
        units::torque(200.0, units::Nm),
        units::rpm(300.0));

    const double stall = starter.availableTorque(0.0);
    const double middle = starter.availableTorque(units::rpm(150.0));
    const double top = starter.availableTorque(units::rpm(300.0));

    EXPECT_GT(stall, middle);
    EXPECT_GT(middle, top);
    EXPECT_NEAR(top, 0.0, 1e-9);
}

TEST(StarterMotorTests, AColdStarterIsWeaker) {
    StarterMotor starter;
    buildTorqueMap(
        &starter.m_torqueMap,
        units::torque(110.0, units::Nm),
        units::torque(200.0, units::Nm),
        units::rpm(300.0));

    starter.m_temperature = units::celcius(80.0);
    const double warm = starter.availableTorque(units::rpm(100.0));

    starter.m_temperature = units::celcius(-20.0);
    const double cold = starter.availableTorque(units::rpm(100.0));

    EXPECT_LT(cold, warm) << "the temperature axis has no effect";
    EXPECT_GT(cold, 0.0);
}

TEST(StarterMotorTests, TheTargetSpeedKeepsTheForwardSign) {
    StarterMotor starter;
    starter.m_rotationSpeed = -units::rpm(200.0);

    starter.m_speedMap.initialize(2, 1, 0.0);
    starter.m_speedMap.setXAxis(0, units::celcius(-20.0));
    starter.m_speedMap.setXAxis(1, units::celcius(80.0));
    starter.m_speedMap.setYAxis(0, 0.0);
    starter.m_speedMap.setValue(0, 0, units::rpm(150.0));
    starter.m_speedMap.setValue(1, 0, units::rpm(260.0));

    starter.m_temperature = units::celcius(80.0);
    const double warm = starter.targetSpeed();

    starter.m_temperature = units::celcius(-20.0);
    const double cold = starter.targetSpeed();

    EXPECT_LT(warm, 0.0) << "forward cranking must stay negative";
    EXPECT_LT(cold, 0.0);
    EXPECT_NEAR(warm, -units::rpm(260.0), 1e-9);
    EXPECT_NEAR(cold, -units::rpm(150.0), 1e-9);
    EXPECT_GT(std::abs(warm), std::abs(cold)) << "a cold engine must crank slower";
}

namespace {
    class StarterRig {
        public:
            StarterRig(double temperature) {
                m_crank.m_body.reset();
                m_crank.m_body.m = 1.0;
                m_crank.m_body.I = 0.25;

                m_system.initialize(new atg_scs::GaussSeidelSleSolver);
                m_system.addRigidBody(&m_crank.m_body);

                m_starter.connectCrankshaft(&m_crank);
                m_starter.m_maxTorque = units::torque(200.0, units::Nm);
                m_starter.m_rotationSpeed = -units::rpm(300.0);
                m_starter.m_temperature = temperature;
                m_starter.m_enabled = true;

                buildTorqueMap(
                    &m_starter.m_torqueMap,
                    units::torque(60.0, units::Nm),
                    units::torque(200.0, units::Nm),
                    units::rpm(300.0));

                m_system.addConstraint(&m_starter);
            }

            void run(int steps, double dt = 1e-4) {
                for (int i = 0; i < steps; ++i) m_system.process(dt, 1);
            }

            atg_scs::OptimizedNsvRigidBodySystem m_system;
            Crankshaft m_crank;
            StarterMotor m_starter;
    };
}

TEST(StarterMotorTests, AColdEngineComesUpToSpeedMoreSlowly) {
    StarterRig warm(units::celcius(80.0));
    StarterRig cold(units::celcius(-20.0));

    warm.run(300);
    cold.run(300);

    const double warmSpeed = std::abs(warm.m_crank.m_body.v_theta);
    const double coldSpeed = std::abs(cold.m_crank.m_body.v_theta);

    EXPECT_GT(warmSpeed, 0.0) << "the starter did not turn the crank at all";
    EXPECT_LT(coldSpeed, warmSpeed)
        << "the cold starter spun the engine up just as fast";
}

TEST(StarterMotorTests, TheCrankSpinsForward) {
    StarterRig rig(units::celcius(80.0));
    rig.run(2000);

    EXPECT_LT(rig.m_crank.m_body.v_theta, 0.0)
        << "forward rotation is negative v_theta";
}

TEST(StarterMotorTests, TheEngineHandsItsMapsToTheConstraint) {
    Engine engine;

    control::Map2d &source = engine.getStarterTorqueMap();
    buildTorqueMap(
        &source,
        units::torque(60.0, units::Nm),
        units::torque(200.0, units::Nm),
        units::rpm(300.0));

    control::Map2d torque;
    control::Map2d speed;
    engine.copyStarterMaps(&torque, &speed);

    ASSERT_TRUE(torque.isInitialized());
    EXPECT_FALSE(speed.isInitialized()) << "an unset map must stay unset";

    EXPECT_EQ(torque.getXCount(), source.getXCount());
    EXPECT_EQ(torque.getYCount(), source.getYCount());

    for (int j = 0; j < source.getYCount(); ++j) {
        for (int i = 0; i < source.getXCount(); ++i) {
            EXPECT_NEAR(torque.getValue(i, j), source.getValue(i, j), 1e-12);
        }
    }
}

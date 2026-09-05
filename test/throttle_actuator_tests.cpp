#include <gtest/gtest.h>

#include "../include/external_throttle.h"
#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <cmath>

TEST(ThrottleActuatorTests, TheDefaultPassesTheCommandStraightThrough) {
    ExternalThrottle throttle;

    throttle.setPlatePosition(0.75);

    EXPECT_NEAR(throttle.getPlatePosition(), 0.75, 1e-12);
    EXPECT_NEAR(throttle.getCommandedPosition(), 0.75, 1e-12);
}

TEST(ThrottleActuatorTests, ARateLimitHoldsThePlateBack) {
    ExternalThrottle throttle;
    throttle.m_params.openRate = 10.0;
    throttle.m_params.closeRate = 10.0;
    throttle.reset();

    throttle.setPlatePosition(1.0);

    EXPECT_NEAR(throttle.getPlatePosition(), 0.0, 1e-12);

    for (int i = 0; i < 50; ++i) throttle.update(1e-3, nullptr);

    EXPECT_NEAR(throttle.getPlatePosition(), 0.5, 1e-9);

    for (int i = 0; i < 50; ++i) throttle.update(1e-3, nullptr);

    EXPECT_NEAR(throttle.getPlatePosition(), 1.0, 1e-9);
}

TEST(ThrottleActuatorTests, TheLagApproachesTheCommandWithoutOvershoot) {
    ExternalThrottle throttle;
    throttle.m_params.timeConstant = 0.05;
    throttle.reset();

    throttle.setPlatePosition(1.0);

    double previous = throttle.getPlatePosition();
    for (int i = 0; i < 1000; ++i) {
        throttle.update(1e-3, nullptr);
        const double now = throttle.getPlatePosition();
        ASSERT_GE(now, previous - 1e-12);
        ASSERT_LE(now, 1.0 + 1e-12);
        previous = now;
    }

    EXPECT_GT(throttle.getPlatePosition(), 0.99);
}

TEST(ThrottleActuatorTests, TheLagIsTimeCorrect) {
    const auto settle = [](double dt, int steps) {
        ExternalThrottle throttle;
        throttle.m_params.timeConstant = 0.05;
        throttle.reset();
        throttle.setPlatePosition(1.0);

        for (int i = 0; i < steps; ++i) throttle.update(dt, nullptr);

        return throttle.getPlatePosition();
    };

    const double fine = settle(1e-4, 1000);
    const double coarse = settle(1e-3, 100);

    EXPECT_NEAR(fine, coarse, 1e-2);
}

TEST(ThrottleActuatorTests, TheClosingRateCanDifferFromTheOpeningRate) {
    ExternalThrottle throttle;
    throttle.m_params.openRate = 4.0;
    throttle.m_params.closeRate = 20.0;
    throttle.reset();

    throttle.setPlatePosition(1.0);
    for (int i = 0; i < 100; ++i) throttle.update(1e-3, nullptr);
    const double opened = throttle.getPlatePosition();

    throttle.setPlatePosition(0.0);
    for (int i = 0; i < 100; ++i) throttle.update(1e-3, nullptr);
    const double closed = throttle.getPlatePosition();

    EXPECT_NEAR(opened, 0.4, 1e-9);
    EXPECT_NEAR(closed, 0.0, 1e-9);
}

TEST(ThrottleActuatorTests, TheParametersAreReachableThroughTheRegistry) {
    config::ParameterRegistry registry;
    ExternalThrottle throttle;
    throttle.registerParameters(&registry, "");

    ASSERT_TRUE(registry.contains("throttle.open_rate"));
    ASSERT_TRUE(registry.contains("throttle.close_rate"));
    ASSERT_TRUE(registry.contains("throttle.time_constant"));

    ASSERT_TRUE(registry.set("throttle.time_constant", 0.08));
    EXPECT_NEAR(throttle.m_params.timeConstant, 0.08, 1e-12);
}

#include <gtest/gtest.h>

#include "../include/control/pid_controller.h"
#include "../include/control/map_2d.h"
#include "../include/control/rate_limiter.h"
#include "../include/control/hysteresis.h"

#include <cmath>

TEST(PidControllerTests, ProportionalOnlyMatchesGainTimesError) {
    control::PidController::Parameters params;
    params.kp = 2.0;
    params.outputMin = -100.0;
    params.outputMax = 100.0;

    control::PidController pid;
    pid.initialize(params);

    const double output = pid.update(0.01, 5.0, 3.0);

    EXPECT_NEAR(output, 4.0, 1e-12);
    EXPECT_NEAR(pid.getError(), 2.0, 1e-12);
}

TEST(PidControllerTests, ProportionalClosedLoopReachesAnalyticSteadyState) {
    const double tau = 0.5;
    const double kp = 4.0;
    const double setpoint = 1.0;
    const double dt = 1e-4;

    control::PidController::Parameters params;
    params.kp = kp;
    params.outputMin = -1e6;
    params.outputMax = 1e6;

    control::PidController pid;
    pid.initialize(params);

    double y = 0.0;
    for (int i = 0; i < 200000; ++i) {
        const double u = pid.update(dt, setpoint, y);
        y += ((u - y) / tau) * dt;
    }

    const double expected = kp * setpoint / (1.0 + kp);

    EXPECT_NEAR(y, expected, 1e-4);
}

TEST(PidControllerTests, IntegralActionRemovesSteadyStateError) {
    const double tau = 0.5;
    const double dt = 1e-4;

    control::PidController::Parameters params;
    params.kp = 2.0;
    params.ki = 10.0;
    params.outputMin = -1e6;
    params.outputMax = 1e6;

    control::PidController pid;
    pid.initialize(params);

    double y = 0.0;
    for (int i = 0; i < 200000; ++i) {
        const double u = pid.update(dt, 1.0, y);
        y += ((u - y) / tau) * dt;
    }

    EXPECT_NEAR(y, 1.0, 1e-6);
}

TEST(PidControllerTests, SetpointStepProducesNoDerivativeKick) {
    control::PidController::Parameters params;
    params.kp = 0.0;
    params.kd = 1.0;
    params.outputMin = -1e6;
    params.outputMax = 1e6;

    control::PidController pid;
    pid.initialize(params);

    pid.update(0.01, 0.0, 0.0);
    const double output = pid.update(0.01, 1000.0, 0.0);

    EXPECT_NEAR(output, 0.0, 1e-12);
}

TEST(PidControllerTests, DerivativeRespondsToMeasurementChange) {
    control::PidController::Parameters params;
    params.kd = 1.0;
    params.outputMin = -1e6;
    params.outputMax = 1e6;

    control::PidController pid;
    pid.initialize(params);

    pid.update(0.01, 0.0, 0.0);
    const double output = pid.update(0.01, 0.0, 2.0);

    EXPECT_NEAR(output, -200.0, 1e-9);
}

namespace {
    int settlingSteps(double trackingGain) {
        const double tau = 0.5;
        const double dt = 1e-3;
        const double unreachable = 5.0;
        const double reachable = 0.5;
        const double tolerance = 0.02;

        control::PidController::Parameters params;
        params.kp = 1.0;
        params.ki = 50.0;
        params.outputMin = 0.0;
        params.outputMax = 1.0;
        params.trackingGain = trackingGain;

        control::PidController pid;
        pid.initialize(params);

        double y = 0.0;
        for (int i = 0; i < 5000; ++i) {
            const double u = pid.update(dt, unreachable, y);
            y += ((u - y) / tau) * dt;
        }

        for (int i = 0; i < 60000; ++i) {
            const double u = pid.update(dt, reachable, y);
            y += ((u - y) / tau) * dt;

            if (std::abs(y - reachable) < tolerance) return i;
        }

        return -1;
    }
}

TEST(PidControllerTests, BackCalculationPreventsIntegratorWindup) {
    const int withAntiWindup = settlingSteps(10.0);
    const int withoutAntiWindup = settlingSteps(0.0);

    ASSERT_GE(withAntiWindup, 0);
    ASSERT_GE(withoutAntiWindup, 0);
    EXPECT_LT(withAntiWindup, withoutAntiWindup);
}

TEST(PidControllerTests, SaturationIsReported) {
    control::PidController::Parameters params;
    params.kp = 1.0;
    params.outputMin = 0.0;
    params.outputMax = 1.0;

    control::PidController pid;
    pid.initialize(params);

    pid.update(1e-3, 10.0, 0.0);
    EXPECT_TRUE(pid.isSaturated());

    pid.update(1e-3, 0.5, 0.0);
    EXPECT_FALSE(pid.isSaturated());
}

TEST(PidControllerTests, IntegratorLimitIsRespected) {
    control::PidController::Parameters params;
    params.ki = 100.0;
    params.outputMin = -1e6;
    params.outputMax = 1e6;
    params.integratorLimit = 3.0;

    control::PidController pid;
    pid.initialize(params);

    for (int i = 0; i < 10000; ++i) pid.update(1e-3, 1.0, 0.0);

    EXPECT_NEAR(pid.getIntegrator(), 3.0, 1e-9);
}

TEST(PidControllerTests, ResetClearsState) {
    control::PidController::Parameters params;
    params.kp = 1.0;
    params.ki = 10.0;
    params.kd = 0.1;
    params.outputMin = -1e6;
    params.outputMax = 1e6;

    control::PidController pid;
    pid.initialize(params);

    for (int i = 0; i < 100; ++i) pid.update(1e-3, 1.0, 0.0);
    ASSERT_GT(pid.getIntegrator(), 0.0);

    pid.reset();

    EXPECT_EQ(pid.getIntegrator(), 0.0);
    EXPECT_EQ(pid.getOutput(), 0.0);

    const double firstOutput = pid.update(1e-3, 0.0, 5.0);
    EXPECT_NEAR(firstOutput, -5.0, 1e-9);
}

TEST(PidControllerTests, DerivativeFilterAttenuatesFastMeasurementNoise) {
    control::PidController::Parameters filtered;
    filtered.kd = 1.0;
    filtered.derivativeCutoff = 1.0;
    filtered.outputMin = -1e9;
    filtered.outputMax = 1e9;

    control::PidController::Parameters unfiltered = filtered;
    unfiltered.derivativeCutoff = 0.0;

    control::PidController a;
    control::PidController b;
    a.initialize(filtered);
    b.initialize(unfiltered);

    const double dt = 1e-3;
    double peakFiltered = 0.0;
    double peakUnfiltered = 0.0;

    for (int i = 0; i < 1000; ++i) {
        const double measurement = (i % 2 == 0) ? 0.0 : 1.0;
        peakFiltered = std::max(peakFiltered, std::abs(a.update(dt, 0.0, measurement)));
        peakUnfiltered = std::max(peakUnfiltered, std::abs(b.update(dt, 0.0, measurement)));
    }

    EXPECT_LT(peakFiltered, peakUnfiltered * 0.1);
}

TEST(Map2dTests, SamplesNodesExactly) {
    control::Map2d map;
    map.initialize(3, 2);

    map.setXAxis(0, 0.0);
    map.setXAxis(1, 10.0);
    map.setXAxis(2, 20.0);
    map.setYAxis(0, 0.0);
    map.setYAxis(1, 1.0);

    map.setValue(0, 0, 1.0);
    map.setValue(1, 0, 2.0);
    map.setValue(2, 0, 3.0);
    map.setValue(0, 1, 4.0);
    map.setValue(1, 1, 5.0);
    map.setValue(2, 1, 6.0);

    EXPECT_NEAR(map.sample(0.0, 0.0), 1.0, 1e-12);
    EXPECT_NEAR(map.sample(10.0, 0.0), 2.0, 1e-12);
    EXPECT_NEAR(map.sample(20.0, 1.0), 6.0, 1e-12);
}

TEST(Map2dTests, InterpolatesBilinearly) {
    control::Map2d map;
    map.initialize(2, 2);

    map.setXAxis(0, 0.0);
    map.setXAxis(1, 2.0);
    map.setYAxis(0, 0.0);
    map.setYAxis(1, 4.0);

    map.setValue(0, 0, 0.0);
    map.setValue(1, 0, 10.0);
    map.setValue(0, 1, 20.0);
    map.setValue(1, 1, 30.0);

    EXPECT_NEAR(map.sample(1.0, 0.0), 5.0, 1e-12);
    EXPECT_NEAR(map.sample(0.0, 2.0), 10.0, 1e-12);
    EXPECT_NEAR(map.sample(1.0, 2.0), 15.0, 1e-12);
}

TEST(Map2dTests, ClampsOutsideTheDomain) {
    control::Map2d map;
    map.initialize(2, 2);

    map.setXAxis(0, 0.0);
    map.setXAxis(1, 1.0);
    map.setYAxis(0, 0.0);
    map.setYAxis(1, 1.0);

    map.setValue(0, 0, 1.0);
    map.setValue(1, 0, 2.0);
    map.setValue(0, 1, 3.0);
    map.setValue(1, 1, 4.0);

    EXPECT_NEAR(map.sample(-100.0, -100.0), 1.0, 1e-12);
    EXPECT_NEAR(map.sample(100.0, 100.0), 4.0, 1e-12);
    EXPECT_NEAR(map.sample(100.0, -100.0), 2.0, 1e-12);
}

TEST(Map2dTests, AccumulateDistributesTheFullDelta) {
    control::Map2d map;
    map.initialize(2, 2, 0.0);

    map.setXAxis(0, 0.0);
    map.setXAxis(1, 1.0);
    map.setYAxis(0, 0.0);
    map.setYAxis(1, 1.0);

    map.accumulate(0.25, 0.75, 1.0, -10.0, 10.0);

    const double total =
        map.getValue(0, 0) + map.getValue(1, 0)
        + map.getValue(0, 1) + map.getValue(1, 1);

    EXPECT_NEAR(total, 1.0, 1e-12);
    EXPECT_NEAR(map.getValue(0, 1), 0.75 * 0.75, 1e-12);
    EXPECT_NEAR(map.getValue(1, 1), 0.25 * 0.75, 1e-12);
}

TEST(Map2dTests, AccumulateRespectsLimits) {
    control::Map2d map;
    map.initialize(2, 2, 0.0);

    map.setXAxis(0, 0.0);
    map.setXAxis(1, 1.0);
    map.setYAxis(0, 0.0);
    map.setYAxis(1, 1.0);

    for (int i = 0; i < 1000; ++i) map.accumulate(0.0, 0.0, 1.0, -0.5, 0.5);

    EXPECT_NEAR(map.getValue(0, 0), 0.5, 1e-12);
}

TEST(Map2dTests, RepeatedAccumulationConvergesToTheTarget) {
    control::Map2d map;
    map.initialize(2, 2, 0.0);

    map.setXAxis(0, 0.0);
    map.setXAxis(1, 1.0);
    map.setYAxis(0, 0.0);
    map.setYAxis(1, 1.0);

    const double target = 7.0;
    const double rate = 0.2;

    for (int i = 0; i < 500; ++i) {
        const double error = target - map.sample(0.0, 0.0);
        map.accumulate(0.0, 0.0, rate * error, -100.0, 100.0);
    }

    EXPECT_NEAR(map.sample(0.0, 0.0), target, 1e-6);
}

TEST(RateLimiterTests, LimitsRiseAndFallSeparately) {
    control::RateLimiter limiter;
    limiter.initialize(2.0, 5.0);
    limiter.reset(0.0);

    EXPECT_NEAR(limiter.update(0.5, 100.0), 1.0, 1e-12);
    EXPECT_NEAR(limiter.update(0.5, 100.0), 2.0, 1e-12);
    EXPECT_NEAR(limiter.update(0.1, -100.0), 1.5, 1e-12);
}

TEST(RateLimiterTests, DoesNotOvershootTheTarget) {
    control::RateLimiter limiter;
    limiter.initialize(100.0, 100.0);
    limiter.reset(0.0);

    EXPECT_NEAR(limiter.update(1.0, 0.5), 0.5, 1e-12);
}

TEST(HysteresisTests, SwitchesOnlyOutsideTheBand) {
    control::Hysteresis hysteresis;
    hysteresis.initialize(2.0, 8.0, false);

    EXPECT_FALSE(hysteresis.update(5.0));
    EXPECT_FALSE(hysteresis.update(8.0));
    EXPECT_TRUE(hysteresis.update(8.5));
    EXPECT_TRUE(hysteresis.update(3.0));
    EXPECT_TRUE(hysteresis.update(2.0));
    EXPECT_FALSE(hysteresis.update(1.5));
}

TEST(StateTimerTests, AccumulatesAndResets) {
    control::StateTimer timer;

    timer.advance(0.4);
    timer.advance(0.4);

    EXPECT_FALSE(timer.hasElapsed(1.0));

    timer.advance(0.4);

    EXPECT_TRUE(timer.hasElapsed(1.0));

    timer.reset();

    EXPECT_NEAR(timer.getElapsed(), 0.0, 1e-12);
}

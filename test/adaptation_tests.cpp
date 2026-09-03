#include <gtest/gtest.h>

#include "../include/adaptation/rls_estimator.h"
#include "../include/adaptation/adaptation_manager.h"
#include "../include/control/iterative_learning.h"
#include "../include/powertrain/engine_control_unit.h"
#include "../include/powertrain/transmission_control_unit.h"
#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <cmath>
#include <random>
#include <vector>

TEST(RlsEstimatorTests, RecoversAKnownGain) {
    adaptation::RlsEstimator::Parameters params;
    params.initialEstimate = 1.0;
    params.initialCovariance = 100.0;
    params.forgettingFactor = 1.0;
    params.estimateMax = 20.0;

    adaptation::RlsEstimator estimator;
    estimator.initialize(params);

    const double truth = 3.7;
    std::mt19937 generator(1234);
    std::uniform_real_distribution<double> input(0.2, 1.0);

    double excitation = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const double x = input(generator);
        excitation += x * x;
        estimator.update(x, truth * x);
    }

    const double initialError = truth - params.initialEstimate;
    const double predictedError =
        initialError / (1.0 + params.initialCovariance * excitation);

    EXPECT_NEAR(estimator.getEstimate() - truth, -predictedError, 1e-9);
    EXPECT_NEAR(estimator.getEstimate(), truth, 1e-4);
}

TEST(RlsEstimatorTests, MoreDataReducesTheRemainingError) {
    adaptation::RlsEstimator::Parameters params;
    params.initialEstimate = 1.0;
    params.initialCovariance = 100.0;
    params.forgettingFactor = 1.0;
    params.estimateMax = 20.0;

    const double truth = 3.7;
    double previousError = 1.0;

    for (int samples : { 500, 2000, 8000 }) {
        adaptation::RlsEstimator estimator;
        estimator.initialize(params);

        std::mt19937 generator(1234);
        std::uniform_real_distribution<double> input(0.2, 1.0);

        for (int i = 0; i < samples; ++i) {
            const double x = input(generator);
            estimator.update(x, truth * x);
        }

        const double error = std::abs(estimator.getEstimate() - truth);
        EXPECT_LT(error, previousError);
        previousError = error;
    }
}

TEST(RlsEstimatorTests, ConvergesDespiteNoise) {
    adaptation::RlsEstimator::Parameters params;
    params.initialEstimate = 1.0;
    params.initialCovariance = 100.0;
    params.forgettingFactor = 1.0;
    params.estimateMax = 20.0;

    adaptation::RlsEstimator estimator;
    estimator.initialize(params);

    const double truth = 2.5;
    std::mt19937 generator(99);
    std::uniform_real_distribution<double> input(0.2, 1.0);
    std::normal_distribution<double> noise(0.0, 0.02);

    for (int i = 0; i < 20000; ++i) {
        const double x = input(generator);
        estimator.update(x, truth * x + noise(generator));
    }

    EXPECT_NEAR(estimator.getEstimate(), truth, 0.02);
}

TEST(RlsEstimatorTests, ForgettingFactorTracksAChangingGain) {
    adaptation::RlsEstimator::Parameters params;
    params.initialEstimate = 1.0;
    params.initialCovariance = 100.0;
    params.forgettingFactor = 0.99;
    params.estimateMax = 20.0;

    adaptation::RlsEstimator estimator;
    estimator.initialize(params);

    std::mt19937 generator(7);
    std::uniform_real_distribution<double> input(0.2, 1.0);

    for (int i = 0; i < 2000; ++i) {
        const double x = input(generator);
        estimator.update(x, 2.0 * x);
    }
    EXPECT_NEAR(estimator.getEstimate(), 2.0, 1e-3);

    for (int i = 0; i < 2000; ++i) {
        const double x = input(generator);
        estimator.update(x, 5.0 * x);
    }
    EXPECT_NEAR(estimator.getEstimate(), 5.0, 1e-3);
}

TEST(RlsEstimatorTests, EstimateStaysWithinItsLimits) {
    adaptation::RlsEstimator::Parameters params;
    params.initialEstimate = 1.0;
    params.estimateMin = 0.5;
    params.estimateMax = 2.0;
    params.forgettingFactor = 1.0;

    adaptation::RlsEstimator estimator;
    estimator.initialize(params);

    for (int i = 0; i < 1000; ++i) estimator.update(1.0, 100.0);
    EXPECT_NEAR(estimator.getEstimate(), 2.0, 1e-12);

    for (int i = 0; i < 1000; ++i) estimator.update(1.0, -100.0);
    EXPECT_NEAR(estimator.getEstimate(), 0.5, 1e-12);
}

TEST(RlsEstimatorTests, IgnoresAVanishingRegressor) {
    adaptation::RlsEstimator estimator;
    estimator.initialize(adaptation::RlsEstimator::Parameters());

    const double before = estimator.getEstimate();
    estimator.update(0.0, 1000.0);

    EXPECT_NEAR(estimator.getEstimate(), before, 1e-12);
}

namespace {
    double runIterativeLearning(
        control::IterativeLearningControl &ilc,
        const std::vector<double> &reference,
        double plantGain)
    {
        const int samples = static_cast<int>(reference.size());
        double norm = 0.0;

        ilc.beginIteration();
        for (int i = 0; i < samples; ++i) {
            const double phase = static_cast<double>(i) / samples;
            const double command = 0.0 + ilc.correction(phase);
            const double output = plantGain * command;
            const double error = reference[i] - output;

            ilc.sample(phase, error);
            norm += error * error;
        }
        ilc.endIteration();

        return std::sqrt(norm / samples);
    }
}

TEST(IterativeLearningTests, ErrorFallsWithEachIteration) {
    control::IterativeLearningControl::Parameters params;
    params.binCount = 8;
    params.learningRate = 0.5;
    params.smoothing = 0.0;
    params.outputMin = -10.0;
    params.outputMax = 10.0;

    control::IterativeLearningControl ilc;
    ilc.initialize(params);

    std::vector<double> reference(64);
    for (size_t i = 0; i < reference.size(); ++i) {
        reference[i] = 0.5 + 0.4 * std::sin(2.0 * constants::pi * i / reference.size());
    }

    double previous = runIterativeLearning(ilc, reference, 1.0);

    for (int iteration = 0; iteration < 12; ++iteration) {
        const double current = runIterativeLearning(ilc, reference, 1.0);
        EXPECT_LE(current, previous + 1e-9) << "iteration=" << iteration;
        previous = current;
    }

    EXPECT_LT(previous, 0.15);
}

TEST(IterativeLearningTests, ProfileIsClamped) {
    control::IterativeLearningControl::Parameters params;
    params.binCount = 4;
    params.learningRate = 1.0;
    params.smoothing = 0.0;
    params.outputMin = -0.2;
    params.outputMax = 0.2;

    control::IterativeLearningControl ilc;
    ilc.initialize(params);

    for (int iteration = 0; iteration < 50; ++iteration) {
        ilc.beginIteration();
        for (int i = 0; i < 16; ++i) ilc.sample(i / 16.0, 5.0);
        ilc.endIteration();
    }

    for (int i = 0; i < params.binCount; ++i) {
        EXPECT_LE(ilc.getBin(i), 0.2 + 1e-12);
        EXPECT_GE(ilc.getBin(i), -0.2 - 1e-12);
    }
}

TEST(IterativeLearningTests, DiscardedIterationDoesNotLearn) {
    control::IterativeLearningControl::Parameters params;
    params.binCount = 4;
    params.learningRate = 1.0;

    control::IterativeLearningControl ilc;
    ilc.initialize(params);

    ilc.beginIteration();
    for (int i = 0; i < 16; ++i) ilc.sample(i / 16.0, 1.0);
    ilc.discardIteration();

    EXPECT_EQ(ilc.getIterationCount(), 0);
    for (int i = 0; i < params.binCount; ++i) {
        EXPECT_NEAR(ilc.getBin(i), 0.0, 1e-12);
    }
}

TEST(IterativeLearningTests, SamplesOutsideAnIterationAreIgnored) {
    control::IterativeLearningControl::Parameters params;
    params.binCount = 4;
    params.learningRate = 1.0;

    control::IterativeLearningControl ilc;
    ilc.initialize(params);

    ilc.sample(0.5, 100.0);
    ilc.endIteration();

    EXPECT_NEAR(ilc.getBin(2), 0.0, 1e-12);
}

namespace {
    adaptation::AdaptationManager::Parameters managerParameters() {
        adaptation::AdaptationManager::Parameters params;
        params.conditions.requireSteadySpeed = false;
        params.conditions.warmTemperature = units::celcius(70.0);

        return params;
    }

    powertrain::PowertrainState adaptationState() {
        powertrain::PowertrainState state;
        state.coolantTemperature = units::celcius(90.0);
        state.engineSpeed = units::rpm(2500.0);
        state.engineRunning = true;
        state.gear = 2;
        state.indicatedTorque = units::torque(60.0, units::Nm);

        return state;
    }
}

TEST(AdaptationManagerTests, ColdEngineBlocksAdaption) {
    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());

    powertrain::PowertrainState state = adaptationState();
    state.coolantTemperature = units::celcius(20.0);

    powertrain::PowertrainBus bus;
    manager.update(1e-3, state, bus);

    EXPECT_FALSE(manager.wasEnabledLastUpdate());
}

TEST(AdaptationManagerTests, ShiftBlocksAdaption) {
    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());

    powertrain::PowertrainState state = adaptationState();
    powertrain::PowertrainBus bus;
    bus.shiftInProgress = true;

    manager.update(1e-3, state, bus);

    EXPECT_FALSE(manager.wasEnabledLastUpdate());
}

TEST(AdaptationManagerTests, LimiterBlocksAdaption) {
    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());

    powertrain::PowertrainState state = adaptationState();
    powertrain::PowertrainBus bus;
    bus.engineState = powertrain::EngineState::Limiting;

    manager.update(1e-3, state, bus);

    EXPECT_FALSE(manager.wasEnabledLastUpdate());
}

TEST(AdaptationManagerTests, UnsteadySpeedBlocksAdaption) {
    adaptation::AdaptationManager::Parameters params = managerParameters();
    params.conditions.requireSteadySpeed = true;
    params.conditions.speedStabilityWindow = units::rpm(50.0);

    adaptation::AdaptationManager manager;
    manager.initialize(params);

    powertrain::PowertrainState state = adaptationState();
    powertrain::PowertrainBus bus;

    manager.update(1e-3, state, bus);
    for (int i = 0; i < 200; ++i) {
        state.engineSpeed += units::rpm(30.0);
        manager.update(1e-3, state, bus);
    }

    EXPECT_FALSE(manager.wasEnabledLastUpdate());
}

TEST(AdaptationManagerTests, WarmSteadyEngineEnablesAdaption) {
    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());

    powertrain::PowertrainState state = adaptationState();
    powertrain::PowertrainBus bus;

    manager.update(1e-3, state, bus);

    EXPECT_TRUE(manager.wasEnabledLastUpdate());
}

TEST(AdaptationManagerTests, ThrottleMapAbsorbsThePidCorrection) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.5;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    const double dt = 1e-3;
    const double plantGain = 0.6;

    double firstCorrection = 0.0;
    for (int i = 0; i < 60000; ++i) {
        ecu.update(dt, state, inputs, &commands);
        state.indicatedTorque =
            commands.throttlePlate * plantGain * ecu.maxTorqueAt(state.engineSpeed);
        manager.update(dt, state, bus);

        if (i == 2000) firstCorrection = std::abs(ecu.getTorqueController().getOutput());
    }

    const double finalCorrection = std::abs(ecu.getTorqueController().getOutput());

    EXPECT_GT(manager.getThrottleUpdateCount(), 0);
    EXPECT_LT(finalCorrection, firstCorrection);
}

TEST(AdaptationManagerTests, IdleTrimDrainsTheIntegrator) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.engineSpeed = units::rpm(700.0);

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    const double dt = 1e-3;
    for (int i = 0; i < 20000; ++i) {
        ecu.update(dt, state, inputs, &commands);
        manager.update(dt, state, bus);
    }

    const double learned =
        ecu.getIdleTrimMap().sample(state.coolantTemperature, 0.0);

    EXPECT_GT(learned, 0.0);
}

TEST(AdaptationManagerTests, LambdaTrimMovesTowardsTheTarget) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager::Parameters params = managerParameters();
    params.lambdaTarget = 0.05;

    adaptation::AdaptationManager manager;
    manager.initialize(params);
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.exhaustO2 = 0.10;

    powertrain::PowertrainBus bus;

    for (int i = 0; i < 5000; ++i) manager.update(1e-3, state, bus);

    EXPECT_LT(manager.getShortTermFuelTrim(), 0.0);
    EXPECT_LT(ecu.getFuelTrim(), 1.0);
}

TEST(AdaptationManagerTests, LambdaTrimIsBounded) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager::Parameters params = managerParameters();
    params.lambdaTrimLimit = 0.1;

    adaptation::AdaptationManager manager;
    manager.initialize(params);
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.exhaustO2 = 5.0;

    powertrain::PowertrainBus bus;
    for (int i = 0; i < 100000; ++i) manager.update(1e-3, state, bus);

    EXPECT_NEAR(manager.getShortTermFuelTrim(), -0.1, 1e-9);
}

TEST(AdaptationManagerTests, ShiftQualityImprovesOverRepeatedShifts) {
    powertrain::TransmissionControlUnit::Parameters tcuParams;
    tcuParams.gearCount = 6;
    tcuParams.minGearTime = 0.1;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(tcuParams);

    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());
    manager.attach(nullptr, &tcu);

    powertrain::PowertrainState state = adaptationState();
    state.vehicleSpeed = 60.0;
    state.gear = 0;

    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.3;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    const double dt = 1e-3;
    for (int i = 0; i < 40000; ++i) {
        tcu.update(dt, state, inputs, &commands);
        state.gear = commands.targetGear;
        state.clutchSlipSpeed[0] =
            units::rpm(900.0) * (1.0 - commands.clutchPressure[0]);

        bus.shiftInProgress = tcu.isShifting();
        manager.update(dt, state, bus);

        if (state.gear >= 5) state.gear = 0;
    }

    EXPECT_GT(manager.getShiftIterationCount(), 2);
}

TEST(AdaptationManagerTests, ParametersAreReachableThroughTheRegistry) {
    config::ParameterRegistry registry;

    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());
    manager.registerParameters(&registry, "");

    ASSERT_TRUE(registry.contains("adaptation.throttle_map.rate"));
    ASSERT_TRUE(registry.contains("adaptation.idle.enabled"));

    ASSERT_TRUE(registry.set("adaptation.idle.enabled", 0.0));

    double value = 1.0;
    ASSERT_TRUE(registry.get("adaptation.idle.enabled", &value));
    EXPECT_NEAR(value, 0.0, 1e-12);
}

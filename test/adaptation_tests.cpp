#include <gtest/gtest.h>

#include "../include/adaptation/rls_estimator.h"
#include "../include/adaptation/adaptation_manager.h"
#include "../include/control/iterative_learning.h"
#include "../include/powertrain/engine_control_unit.h"
#include "../include/powertrain/transmission_control_unit.h"
#include "../include/powertrain/powertrain_unit.h"
#include "../include/config/channel_recorder.h"
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

    EXPECT_GT(manager.getShortTermFuelTrim(), 0.0);
    EXPECT_GT(ecu.getFuelTrim(), 1.0);
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

    EXPECT_NEAR(manager.getShortTermFuelTrim(), 0.1, 1e-9);
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

TEST(LongTermTrimTests, TheDefaultLeavesEnrichmentUntouched) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.exhaustO2 = 0.30;

    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    inputs.accelerator = 0.4;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    for (int i = 0; i < 20000; ++i) {
        ecu.update(1e-3, state, inputs, &commands);
        manager.update(1e-3, state, bus);
    }

    EXPECT_NEAR(ecu.getLongTermFuelTrim(), 0.0, 1e-12);
    EXPECT_NEAR(commands.fuelEnrichment, ecu.getFuelTrim(), 1e-12);
    EXPECT_GT(manager.getShortTermFuelTrim(), 0.0);
}

TEST(LongTermTrimTests, TheShortTermTrimMigratesIntoTheZone) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager::Parameters params = managerParameters();
    params.lambdaLongTermRate = 0.5;

    adaptation::AdaptationManager manager;
    manager.initialize(params);
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.exhaustO2 = 0.30;

    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    inputs.accelerator = 0.4;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    for (int i = 0; i < 40000; ++i) {
        ecu.update(1e-3, state, inputs, &commands);
        manager.update(1e-3, state, bus);
    }

    const double learned = ecu.getLambdaTrimMap().sample(
        state.engineSpeed, ecu.lambdaTrimLoad(state));

    EXPECT_GT(learned, 0.0);
    EXPECT_GT(ecu.getLongTermFuelTrim(), 0.0);
}

namespace {
    double trimMapTotal(const control::Map2d &map) {
        double total = 0.0;
        for (int i = 0; i < map.getXCount(); ++i) {
            for (int j = 0; j < map.getYCount(); ++j) total += map.getValue(i, j);
        }

        return total;
    }
}

TEST(LongTermTrimTests, TheTransferPreservesTheAppliedTrim) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager::Parameters params = managerParameters();
    params.lambdaLongTermRate = 0.5;

    adaptation::AdaptationManager manager;
    manager.initialize(params);
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.exhaustO2 = 0.05;

    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    inputs.accelerator = 0.4;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    for (int i = 0; i < 3000; ++i) {
        ecu.update(1e-3, state, inputs, &commands);
        manager.update(1e-3, state, bus);
    }

    state.exhaustO2 = params.lambdaTarget;

    const double shortBefore = manager.getShortTermFuelTrim();
    const double longBefore = trimMapTotal(ecu.getLambdaTrimMap());

    for (int i = 0; i < 20000; ++i) {
        ecu.update(1e-3, state, inputs, &commands);
        manager.update(1e-3, state, bus);
    }

    const double shortAfter = manager.getShortTermFuelTrim();
    const double longAfter = trimMapTotal(ecu.getLambdaTrimMap());

    EXPECT_LT(shortAfter, shortBefore);
    EXPECT_GT(longAfter, longBefore);
    EXPECT_NEAR(shortAfter + longAfter, shortBefore + longBefore, 1e-9);
}

TEST(LongTermTrimTests, AnUnvisitedZoneStaysEmpty) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager::Parameters params = managerParameters();
    params.lambdaLongTermRate = 0.5;

    adaptation::AdaptationManager manager;
    manager.initialize(params);
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.engineSpeed = units::rpm(1000.0);
    state.exhaustO2 = 0.30;

    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    for (int i = 0; i < 40000; ++i) {
        ecu.update(1e-3, state, inputs, &commands);
        manager.update(1e-3, state, bus);
    }

    const double visited = ecu.getLambdaTrimMap().sample(
        units::rpm(1000.0), 0.0);
    const double untouched = ecu.getLambdaTrimMap().sample(
        units::rpm(6500.0), units::torque(200.0, units::Nm));

    EXPECT_GT(visited, 0.0);
    EXPECT_NEAR(untouched, 0.0, 1e-12);
}

TEST(LongTermTrimTests, TheZoneIsBoundedByTheTrimLimit) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager::Parameters params = managerParameters();
    params.lambdaLongTermRate = 2.0;
    params.lambdaTrimLimit = 0.05;

    adaptation::AdaptationManager manager;
    manager.initialize(params);
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.exhaustO2 = 5.0;

    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    for (int i = 0; i < 200000; ++i) {
        ecu.update(1e-3, state, inputs, &commands);
        manager.update(1e-3, state, bus);
    }

    EXPECT_GT(ecu.getLongTermFuelTrim(), 0.0);
    EXPECT_LE(ecu.getLongTermFuelTrim(), 0.05 + 1e-9);
}

TEST(LongTermTrimTests, TheLearnedZoneReachesTheFuelCommand) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    powertrain::PowertrainState state = adaptationState();
    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    inputs.accelerator = 0.4;

    powertrain::ActuatorCommands commands;
    ecu.update(1e-3, state, inputs, &commands);
    const double baseline = commands.fuelEnrichment;

    control::Map2d &trim = ecu.getLambdaTrimMap();
    for (int i = 0; i < trim.getXCount(); ++i) {
        for (int j = 0; j < trim.getYCount(); ++j) trim.setValue(i, j, 0.1);
    }

    ecu.update(1e-3, state, inputs, &commands);

    EXPECT_NEAR(commands.fuelEnrichment, baseline * 1.1, 1e-9);
}

TEST(LongTermTrimTests, TheLoadAxisIsSelectable) {
    powertrain::EngineControlUnit::Parameters params;
    params.lambdaTrimLoadIsManifold = true;

    powertrain::EngineControlUnit ecu;
    ecu.initialize(params);

    powertrain::PowertrainState state = adaptationState();
    state.manifoldPressure = 60000.0;

    EXPECT_NEAR(ecu.lambdaTrimLoad(state), 60000.0, 1e-9);

    powertrain::EngineControlUnit torqueEcu;
    torqueEcu.initialize(powertrain::EngineControlUnit::Parameters());

    EXPECT_NEAR(torqueEcu.lambdaTrimLoad(state), torqueEcu.getTorqueRequest(), 1e-9);
}

namespace {
    void widenTorqueController(powertrain::EngineControlUnit *ecu) {
        control::PidController::Parameters pid =
            ecu->getTorqueController().getParameters();
        pid.ki = 0.0;
        pid.kp = 0.002;
        pid.outputMin = -1e6;
        pid.outputMax = 1e6;
        pid.trackingGain = 0.0;

        ecu->getTorqueController().setParameters(pid);
    }

    int learnWithSource(bool fromIntegrator, double *integrator, double *output) {
        powertrain::EngineControlUnit ecu;
        ecu.initialize(powertrain::EngineControlUnit::Parameters());
        widenTorqueController(&ecu);

        adaptation::AdaptationManager::Parameters params = managerParameters();
        params.throttleLearnFromIntegrator = fromIntegrator;
        params.idleEnabled = false;
        params.lambdaEnabled = false;

        adaptation::AdaptationManager manager;
        manager.initialize(params);
        manager.attach(&ecu, nullptr);

        powertrain::PowertrainState state = adaptationState();
        powertrain::DriverInputs inputs;
        inputs.ignitionKey = true;
        inputs.accelerator = 0.8;
        powertrain::ActuatorCommands commands;
        powertrain::PowertrainBus bus;

        for (int i = 0; i < 5000; ++i) {
            ecu.update(1e-3, state, inputs, &commands);
            manager.update(1e-3, state, bus);
        }

        *integrator = ecu.getTorqueController().getIntegrator();
        *output = ecu.getTorqueController().getOutput();

        return manager.getThrottleUpdateCount();
    }
}

TEST(ThrottleLearnSourceTests, TheIntegratorSourceLeavesAnEmptyIntegratorAlone) {
    double integrator = 0.0;
    double output = 0.0;
    const int updates = learnWithSource(true, &integrator, &output);

    ASSERT_GT(std::abs(output), 0.01);

    EXPECT_EQ(updates, 0);
    EXPECT_NEAR(integrator, 0.0, 1e-12);
}

TEST(ThrottleLearnSourceTests, TheDefaultSourceDrainsAnIntegratorThatWasNeverFilled) {
    double integrator = 0.0;
    double output = 0.0;
    const int updates = learnWithSource(false, &integrator, &output);

    ASSERT_GT(std::abs(output), 0.01);

    EXPECT_GT(updates, 0);
    EXPECT_LT(integrator, -0.01);
}

TEST(ThrottleLearnSourceTests, ASaturatedPlateCanBlockTheAdaption) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager::Parameters params = managerParameters();
    params.conditions.requireUnsaturatedPlate = true;

    adaptation::AdaptationManager manager;
    manager.initialize(params);
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    inputs.accelerator = 1.0;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    for (int i = 0; i < 2000; ++i) {
        ecu.update(1e-3, state, inputs, &commands);
        manager.update(1e-3, state, bus);
    }

    EXPECT_NEAR(ecu.getCommandedPlate(), 1.0, 1e-9);
    EXPECT_FALSE(manager.wasEnabledLastUpdate());
}

TEST(LongTermTrimTests, TheNewParametersAreReachableThroughTheRegistry) {
    config::ParameterRegistry registry;

    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());
    manager.registerParameters(&registry, "");

    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());
    ecu.registerParameters(&registry, "");

    ASSERT_TRUE(registry.contains("adaptation.lambda.long_term_rate"));
    ASSERT_TRUE(registry.contains("adaptation.throttle_map.learn_from_integrator"));
    ASSERT_TRUE(registry.contains("adaptation.conditions.require_unsaturated_plate"));
    ASSERT_TRUE(registry.contains("ecu.lambda.trim_load_manifold"));
    ASSERT_TRUE(registry.contains("ecu.lambda.trim[0][0]"));

    ASSERT_TRUE(registry.set("ecu.lambda.trim[1][1]", 0.07));
    EXPECT_NEAR(ecu.getLambdaTrimMap().getValue(1, 1), 0.07, 1e-12);
}

TEST(LongTermTrimTests, BothTrimsAppearAsChannels) {
    powertrain::PowertrainUnit unit;
    unit.initialize(
        powertrain::EngineControlUnit::Parameters(),
        powertrain::TransmissionControlUnit::Parameters());

    powertrain::EngineControlUnit &ecu = unit.getEngineControlUnit();

    control::Map2d &trim = ecu.getLambdaTrimMap();
    for (int i = 0; i < trim.getXCount(); ++i) {
        for (int j = 0; j < trim.getYCount(); ++j) trim.setValue(i, j, 0.04);
    }

    powertrain::PowertrainState state = adaptationState();
    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    powertrain::ActuatorCommands commands;

    ecu.setFuelTrim(1.03);
    unit.update(1e-3, state, inputs, &commands);

    config::ChannelTable channels;
    unit.fillChannels(&channels);

    EXPECT_NEAR(
        channels.getValue(channels.find("ecu.lambda.long_term")), 0.04, 1e-9);
    EXPECT_NEAR(
        channels.getValue(channels.find("ecu.lambda.short_term")),
        ecu.getFuelTrim() - 1.0,
        1e-12);
}

namespace {
    struct LambdaLoop {
        powertrain::EngineControlUnit ecu;
        adaptation::AdaptationManager manager;
        double richness = 0.15;

        void build(double target) {
            ecu.initialize(powertrain::EngineControlUnit::Parameters());

            adaptation::AdaptationManager::Parameters params = managerParameters();
            params.lambdaTarget = target;
            manager.initialize(params);
            manager.attach(&ecu, nullptr);
        }

        double run(int steps) {
            powertrain::PowertrainState state = adaptationState();
            powertrain::PowertrainBus bus;

            for (int i = 0; i < steps; ++i) {
                state.exhaustO2 = richness / std::max(ecu.getFuelTrim(), 1e-3);
                manager.update(1e-3, state, bus);
            }

            return richness / std::max(ecu.getFuelTrim(), 1e-3);
        }
    };
}

TEST(LambdaLoopTests, MoreFuelLeavesLessOxygenInTheExhaust) {
    powertrain::EngineControlUnit::Parameters params;

    powertrain::EngineControlUnit lean;
    lean.initialize(params);
    powertrain::EngineControlUnit rich;
    rich.initialize(params);

    powertrain::PowertrainState state = adaptationState();
    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    inputs.accelerator = 0.4;

    powertrain::ActuatorCommands leanCommands;
    powertrain::ActuatorCommands richCommands;

    lean.setFuelTrim(0.9);
    rich.setFuelTrim(1.1);
    lean.update(1e-3, state, inputs, &leanCommands);
    rich.update(1e-3, state, inputs, &richCommands);

    EXPECT_LT(leanCommands.fuelEnrichment, richCommands.fuelEnrichment);
}

TEST(LambdaLoopTests, TheLoopConvergesOnTheTargetFromLean) {
    LambdaLoop loop;
    loop.build(0.05);
    loop.richness = 0.06;

    const double start = loop.run(0);
    const double settled = loop.run(60000);

    EXPECT_GT(start, 0.05);
    EXPECT_LT(std::abs(settled - 0.05), std::abs(start - 0.05));
    EXPECT_NEAR(settled, 0.05, 0.005);
}

TEST(LambdaLoopTests, TheLoopConvergesOnTheTargetFromRich) {
    LambdaLoop loop;
    loop.build(0.05);
    loop.richness = 0.045;

    const double start = loop.run(0);
    const double settled = loop.run(60000);

    EXPECT_LT(start, 0.05);
    EXPECT_LT(std::abs(settled - 0.05), std::abs(start - 0.05));
    EXPECT_NEAR(settled, 0.05, 0.005);
}

TEST(LambdaLoopTests, TheTrimDoesNotRunToTheRail) {
    LambdaLoop loop;
    loop.build(0.05);
    loop.richness = 0.06;

    loop.run(60000);

    EXPECT_LT(std::abs(loop.manager.getShortTermFuelTrim()), 0.29);
}

TEST(IdleTrimTests, TheOverrunDoesNotTeachTheIdleTrim) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.engineSpeed = units::rpm(3000.0);
    state.gear = 3;

    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;
    inputs.accelerator = 0.0;

    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    for (int i = 0; i < 20000; ++i) {
        ecu.update(1e-3, state, inputs, &commands);
        manager.update(1e-3, state, bus);
    }

    EXPECT_EQ(ecu.getEngineState(), powertrain::EngineState::Idle);
    EXPECT_NEAR(
        ecu.getIdleTrimMap().sample(state.coolantTemperature, 0.0), 0.0, 1e-9);
}

TEST(IdleTrimTests, TheRealIdleStillTeachesIt) {
    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());

    adaptation::AdaptationManager manager;
    manager.initialize(managerParameters());
    manager.attach(&ecu, nullptr);

    powertrain::PowertrainState state = adaptationState();
    state.engineSpeed = units::rpm(700.0);

    powertrain::DriverInputs inputs;
    inputs.ignitionKey = true;

    powertrain::ActuatorCommands commands;
    powertrain::PowertrainBus bus;

    for (int i = 0; i < 20000; ++i) {
        ecu.update(1e-3, state, inputs, &commands);
        manager.update(1e-3, state, bus);
    }

    EXPECT_GT(ecu.getIdleTrimMap().sample(state.coolantTemperature, 0.0), 0.0);
}

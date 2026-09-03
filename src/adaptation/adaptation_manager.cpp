#include "../../include/adaptation/adaptation_manager.h"

#include "../../include/powertrain/engine_control_unit.h"
#include "../../include/powertrain/transmission_control_unit.h"
#include "../../include/config/parameter_registry.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace {
    config::ParameterDescriptor describe(
        const std::string &path,
        double min,
        double max,
        double defaultValue,
        const char *unit)
    {
        config::ParameterDescriptor d;
        d.path = path;
        d.minValue = min;
        d.maxValue = max;
        d.defaultValue = defaultValue;
        d.unit = unit;

        return d;
    }
}

adaptation::AdaptationManager::AdaptationManager() {
    m_ecu = nullptr;
    m_tcu = nullptr;
    m_shortTermTrim = 0.0;
    m_speedAverage = 0.0;
    m_speedDeviation = 0.0;
    m_speedPrimed = false;
    m_enabled = false;
    m_shiftActive = false;
    m_shiftElapsed = 0.0;
    m_throttleUpdates = 0;
}

adaptation::AdaptationManager::~AdaptationManager() {
    /* void */
}

void adaptation::AdaptationManager::initialize(const Parameters &params) {
    m_params = params;

    m_torqueModel.initialize(m_params.torqueModel);

    reset();
}

void adaptation::AdaptationManager::reset() {
    m_torqueModel.reset();

    m_shortTermTrim = 0.0;
    m_speedAverage = 0.0;
    m_speedDeviation = 0.0;
    m_speedPrimed = false;
    m_enabled = false;
    m_shiftActive = false;
    m_shiftElapsed = 0.0;
    m_throttleUpdates = 0;
}

int adaptation::AdaptationManager::getShiftIterationCount() const {
    return (m_tcu != nullptr)
        ? m_tcu->getEngageProfile().getIterationCount()
        : 0;
}

double adaptation::AdaptationManager::getShiftErrorNorm() const {
    return (m_tcu != nullptr)
        ? m_tcu->getEngageProfile().getLastErrorNorm()
        : 0.0;
}

void adaptation::AdaptationManager::attach(
    powertrain::EngineControlUnit *ecu,
    powertrain::TransmissionControlUnit *tcu)
{
    m_ecu = ecu;
    m_tcu = tcu;
}

bool adaptation::AdaptationManager::conditionsMet(
    const powertrain::PowertrainState &state,
    const powertrain::PowertrainBus &bus) const
{
    const EnableConditions &c = m_params.conditions;

    if (state.engineSpeed < c.minimumSpeed) return false;
    if (c.requireWarm && state.coolantTemperature < c.warmTemperature) return false;
    if (c.requireNoShift && bus.shiftInProgress) return false;
    if (c.requireNoLimiting && bus.engineState == powertrain::EngineState::Limiting) return false;
    if (c.requireSteadySpeed && m_speedDeviation > c.speedStabilityWindow) return false;

    return true;
}

void adaptation::AdaptationManager::updateThrottleMap(
    double dt,
    const powertrain::PowertrainState &state)
{
    if (!m_params.throttleMapEnabled || m_ecu == nullptr) return;

    control::PidController &torqueController = m_ecu->getTorqueController();
    const double correction = torqueController.getOutput();

    if (std::abs(correction) < m_params.throttleDeadband) return;

    control::Map2d &map = m_ecu->getThrottleMap();
    if (!map.isInitialized()) return;

    const double delta = m_params.throttleLearningRate * correction * dt;

    map.accumulate(
        state.engineSpeed,
        m_ecu->getTorqueRequest(),
        delta,
        0.0,
        1.0);

    torqueController.setIntegrator(
        torqueController.getIntegrator() - delta);

    ++m_throttleUpdates;

    const double regressor = std::max(m_ecu->getCommandedPlate(), 0.0);
    if (regressor > 0.05) {
        m_torqueModel.update(regressor, state.indicatedTorque);
    }
}

void adaptation::AdaptationManager::updateIdleTrim(
    double dt,
    const powertrain::PowertrainState &state)
{
    if (!m_params.idleEnabled || m_ecu == nullptr) return;
    if (m_ecu->getEngineState() != powertrain::EngineState::Idle) return;

    control::PidController &idle = m_ecu->getIdleController();
    control::Map2d &trim = m_ecu->getIdleTrimMap();
    if (!trim.isInitialized()) return;

    const double transfer = m_params.idleDrainRate * idle.getIntegrator() * dt;
    if (transfer == 0.0) return;

    trim.accumulate(
        state.coolantTemperature,
        0.0,
        transfer,
        -m_params.idleTrimLimit,
        m_params.idleTrimLimit);

    idle.setIntegrator(idle.getIntegrator() - transfer);
}

void adaptation::AdaptationManager::updateLambdaTrim(
    double dt,
    const powertrain::PowertrainState &state)
{
    if (!m_params.lambdaEnabled || m_ecu == nullptr) return;

    const double error = m_params.lambdaTarget - state.exhaustO2;

    m_shortTermTrim = std::clamp(
        m_shortTermTrim + m_params.lambdaShortTermGain * error * dt,
        -m_params.lambdaTrimLimit,
        m_params.lambdaTrimLimit);

    m_ecu->setFuelTrim(1.0 + m_shortTermTrim);
}

void adaptation::AdaptationManager::updateShiftLearning(
    double dt,
    const powertrain::PowertrainState &state)
{
    if (!m_params.shiftEnabled || m_tcu == nullptr) return;

    const bool engaging =
        m_tcu->getShiftState() == powertrain::ShiftState::ClutchEngage;

    if (engaging && !m_shiftActive) {
        m_shiftActive = true;
        m_shiftElapsed = 0.0;
        m_tcu->getEngageProfile().beginIteration();
    }

    if (engaging) {
        m_shiftElapsed += dt;

        const double slip = std::abs(state.clutchSlipSpeed[0]);
        const double phase = m_tcu->getEngagePhase();
        const double target =
            m_tcu->getParameters().launchLockSlip * (1.0 - phase);

        m_tcu->getEngageProfile().sample(phase, (slip - target) * 1e-3);
    }
    else if (m_shiftActive) {
        m_shiftActive = false;
        m_tcu->getEngageProfile().endIteration();
    }
}

void adaptation::AdaptationManager::update(
    double dt,
    const powertrain::PowertrainState &state,
    const powertrain::PowertrainBus &bus)
{
    if (dt <= 0.0) return;

    if (!m_speedPrimed) {
        m_speedAverage = state.engineSpeed;
        m_speedDeviation = 0.0;
        m_speedPrimed = true;
    }
    else {
        const double alpha = std::clamp(dt / (dt + 0.25), 0.0, 1.0);
        const double difference = state.engineSpeed - m_speedAverage;
        m_speedAverage += alpha * difference;
        m_speedDeviation += alpha * (std::abs(difference) - m_speedDeviation);
    }

    updateShiftLearning(dt, state);

    m_enabled = conditionsMet(state, bus);
    if (!m_enabled) return;

    updateThrottleMap(dt, state);
    updateIdleTrim(dt, state);
    updateLambdaTrim(dt, state);
}

void adaptation::AdaptationManager::registerParameters(
    config::ParameterRegistry *registry,
    const char *prefix)
{
    if (registry == nullptr) return;

    const std::string base = std::string(prefix) + "adaptation.";

    registry->registerBoolean(
        describe(base + "throttle_map.enabled", 0.0, 1.0,
            m_params.throttleMapEnabled ? 1.0 : 0.0, ""),
        &m_params.throttleMapEnabled);
    registry->registerScalar(
        describe(base + "throttle_map.rate", 0.0, 5.0,
            m_params.throttleLearningRate, ""),
        &m_params.throttleLearningRate);
    registry->registerScalar(
        describe(base + "throttle_map.deadband", 0.0, 0.5,
            m_params.throttleDeadband, ""),
        &m_params.throttleDeadband);

    registry->registerBoolean(
        describe(base + "idle.enabled", 0.0, 1.0,
            m_params.idleEnabled ? 1.0 : 0.0, ""),
        &m_params.idleEnabled);
    registry->registerScalar(
        describe(base + "idle.drain_rate", 0.0, 5.0,
            m_params.idleDrainRate, ""),
        &m_params.idleDrainRate);
    registry->registerScalar(
        describe(base + "idle.limit", 0.0, 1.0,
            m_params.idleTrimLimit, ""),
        &m_params.idleTrimLimit);

    registry->registerBoolean(
        describe(base + "lambda.enabled", 0.0, 1.0,
            m_params.lambdaEnabled ? 1.0 : 0.0, ""),
        &m_params.lambdaEnabled);
    registry->registerScalar(
        describe(base + "lambda.short_term_gain", 0.0, 10.0,
            m_params.lambdaShortTermGain, ""),
        &m_params.lambdaShortTermGain);
    registry->registerScalar(
        describe(base + "lambda.limit", 0.0, 1.0,
            m_params.lambdaTrimLimit, ""),
        &m_params.lambdaTrimLimit);
    registry->registerScalar(
        describe(base + "lambda.target", 0.0, 1.0,
            m_params.lambdaTarget, ""),
        &m_params.lambdaTarget);

    registry->registerBoolean(
        describe(base + "shift.enabled", 0.0, 1.0,
            m_params.shiftEnabled ? 1.0 : 0.0, ""),
        &m_params.shiftEnabled);

    registry->registerScalar(
        describe(base + "conditions.warm_temperature",
            units::celcius(0.0), units::celcius(120.0),
            m_params.conditions.warmTemperature, "K"),
        &m_params.conditions.warmTemperature);
    registry->registerScalar(
        describe(base + "conditions.speed_window", 0.0, units::rpm(2000.0),
            m_params.conditions.speedStabilityWindow, "rad/s"),
        &m_params.conditions.speedStabilityWindow);
}

#include "../../include/powertrain/engine_control_unit.h"

#include "../../include/config/parameter_registry.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace {
    constexpr int ThrottleMapSpeedPoints = 12;
    constexpr int ThrottleMapTorquePoints = 9;
    constexpr int TorqueCurvePoints = 12;
    constexpr int PedalMapPoints = 6;
    constexpr int IdleTrimPoints = 6;
    constexpr int LambdaTrimSpeedZones = 6;
    constexpr int LambdaTrimLoadZones = 4;
    constexpr int TimingMapSpeedPoints = 10;
    constexpr int TimingMapLoadPoints = 6;
}

control::PidController::Parameters powertrain::EngineControlUnit::defaultIdleController() {
    control::PidController::Parameters params;
    params.kp = 0.004;
    params.ki = 0.02;
    params.kd = 0.0;
    params.outputMin = 0.0;
    params.outputMax = 1.0;
    params.trackingGain = 1.0;

    return params;
}

control::PidController::Parameters powertrain::EngineControlUnit::defaultTorqueController() {
    control::PidController::Parameters params;
    params.kp = 0.0008;
    params.ki = 0.004;
    params.kd = 0.0;
    params.outputMin = -0.35;
    params.outputMax = 0.35;
    params.trackingGain = 1.0;

    return params;
}

powertrain::EngineControlUnit::EngineControlUnit() {
    m_torqueRequest = 0.0;
    m_driverTorqueRequest = 0.0;
    m_idleTorqueRequest = 0.0;
    m_feedforwardPlate = 0.0;
    m_feedforwardScale = 1.0;
    m_commandedPlate = 0.0;
    m_fuelTrim = 1.0;
    m_longTermTrim = 0.0;
    m_engineState = EngineState::Off;
}

powertrain::EngineControlUnit::~EngineControlUnit() {
    /* void */
}

void powertrain::EngineControlUnit::buildDefaultMaps() {
    m_maxTorqueMap.initialize(TorqueCurvePoints, 1, m_params.referenceTorque);
    for (int i = 0; i < TorqueCurvePoints; ++i) {
        const double t = static_cast<double>(i) / (TorqueCurvePoints - 1);
        const double speed = units::rpm(500.0) + t * (m_params.revLimit - units::rpm(500.0));
        m_maxTorqueMap.setXAxis(i, speed);

        const double shape = 0.55 + 0.45 * std::sin(constants::pi * std::pow(t, 0.85));
        m_maxTorqueMap.setValue(i, 0, m_params.referenceTorque * shape);
    }

    m_idleTrim.initialize(IdleTrimPoints, 1, 0.0);
    for (int i = 0; i < IdleTrimPoints; ++i) {
        const double t = static_cast<double>(i) / (IdleTrimPoints - 1);
        m_idleTrim.setXAxis(
            i,
            m_params.coldTemperature + t * (m_params.warmTemperature - m_params.coldTemperature));
    }

    m_lambdaTrim.initialize(LambdaTrimSpeedZones, LambdaTrimLoadZones, 0.0);
    for (int i = 0; i < LambdaTrimSpeedZones; ++i) {
        const double t = static_cast<double>(i) / (LambdaTrimSpeedZones - 1);
        m_lambdaTrim.setXAxis(
            i,
            units::rpm(500.0) + t * (m_params.revLimit - units::rpm(500.0)));
    }

    for (int j = 0; j < LambdaTrimLoadZones; ++j) {
        const double t = static_cast<double>(j) / (LambdaTrimLoadZones - 1);
        m_lambdaTrim.setYAxis(j, t * m_params.referenceTorque);
    }

    m_timingMap.initialize(TimingMapSpeedPoints, TimingMapLoadPoints, 0.0);
    for (int i = 0; i < TimingMapSpeedPoints; ++i) {
        const double t = static_cast<double>(i) / (TimingMapSpeedPoints - 1);
        m_timingMap.setXAxis(
            i,
            units::rpm(500.0) + t * (m_params.revLimit - units::rpm(500.0)));
    }

    for (int j = 0; j < TimingMapLoadPoints; ++j) {
        const double t = static_cast<double>(j) / (TimingMapLoadPoints - 1);
        m_timingMap.setYAxis(j, t * m_params.referenceTorque);
    }

    m_pedalMap.initialize(PedalMapPoints, 1, 0.0);
    for (int i = 0; i < PedalMapPoints; ++i) {
        const double t = static_cast<double>(i) / (PedalMapPoints - 1);
        m_pedalMap.setXAxis(i, t);
        m_pedalMap.setValue(i, 0, t);
    }

    m_throttleMap.initialize(ThrottleMapSpeedPoints, ThrottleMapTorquePoints, 0.0);
    for (int i = 0; i < ThrottleMapSpeedPoints; ++i) {
        const double t = static_cast<double>(i) / (ThrottleMapSpeedPoints - 1);
        m_throttleMap.setXAxis(
            i,
            units::rpm(500.0) + t * (m_params.revLimit - units::rpm(500.0)));
    }

    for (int j = 0; j < ThrottleMapTorquePoints; ++j) {
        const double t = static_cast<double>(j) / (ThrottleMapTorquePoints - 1);
        m_throttleMap.setYAxis(j, t * m_params.referenceTorque);
    }

    for (int i = 0; i < ThrottleMapSpeedPoints; ++i) {
        const double speed = m_throttleMap.getXAxis(i);
        const double available = std::max(maxTorqueAt(speed), 1e-6);

        for (int j = 0; j < ThrottleMapTorquePoints; ++j) {
            const double torque = m_throttleMap.getYAxis(j);
            m_throttleMap.setValue(i, j, std::clamp(torque / available, 0.0, 1.0));
        }
    }
}

void powertrain::EngineControlUnit::initialize(const Parameters &params) {
    m_params = params;

    buildDefaultMaps();

    m_idleController.initialize(m_params.idleController);
    m_torqueController.initialize(m_params.torqueController);
    m_torqueLimiter.initialize(m_params.torqueRiseRate, m_params.torqueFallRate);
    m_overrunCut.initialize(
        m_params.overrunResumeSpeed,
        m_params.overrunCutSpeed,
        false);

    reset();
}

void powertrain::EngineControlUnit::reset() {
    PowertrainController::reset();

    m_idleController.reset();
    m_torqueController.reset();
    m_torqueLimiter.reset(0.0);
    m_overrunCut.setState(false);

    m_torqueRequest = 0.0;
    m_driverTorqueRequest = 0.0;
    m_idleTorqueRequest = 0.0;
    m_feedforwardPlate = 0.0;
    m_feedforwardScale = 1.0;
    m_commandedPlate = 0.0;
    m_fuelTrim = 1.0;
    m_longTermTrim = 0.0;
    m_engineState = EngineState::Off;
}

double powertrain::EngineControlUnit::lambdaTrimLoad(const PowertrainState &state) const {
    return m_params.lambdaTrimLoadIsManifold
        ? state.manifoldPressure
        : m_torqueRequest;
}

double powertrain::EngineControlUnit::maxTorqueAt(double engineSpeed) const {
    if (!m_maxTorqueMap.isInitialized()) return m_params.referenceTorque;
    return m_maxTorqueMap.sample(engineSpeed, 0.0);
}

double powertrain::EngineControlUnit::warmupFraction(double coolantTemperature) const {
    const double span = m_params.warmTemperature - m_params.coldTemperature;
    if (span <= 0.0) return 1.0;

    return std::clamp(
        (coolantTemperature - m_params.coldTemperature) / span,
        0.0,
        1.0);
}

double powertrain::EngineControlUnit::idleSpeedAt(double coolantTemperature) const {
    const double warm = warmupFraction(coolantTemperature);
    return m_params.idleSpeedCold + (m_params.idleSpeedWarm - m_params.idleSpeedCold) * warm;
}

void powertrain::EngineControlUnit::setTransmissionRequests(const PowertrainBus &bus) {
    m_bus.torqueReductionRequest = bus.torqueReductionRequest;
    m_bus.interventionType = bus.interventionType;
    m_bus.speedRequest = bus.speedRequest;
    m_bus.speedRequestActive = bus.speedRequestActive;
    m_bus.shiftInProgress = bus.shiftInProgress;
}

powertrain::EngineState powertrain::EngineControlUnit::resolveState(
    const PowertrainState &state,
    const DriverInputs &inputs,
    bool limiterActive) const
{
    if (!inputs.ignitionKey) return EngineState::Off;
    if (state.engineSpeed < m_params.crankingSpeed) {
        return inputs.starterRequest ? EngineState::Cranking : EngineState::Off;
    }

    if (limiterActive) return EngineState::Limiting;
    if (warmupFraction(state.coolantTemperature) < 1.0) return EngineState::ColdStart;
    if (inputs.accelerator <= 0.0) return EngineState::Idle;

    return EngineState::Running;
}

double powertrain::EngineControlUnit::effectiveRevLimit(double coolantTemperature) const {
    const double warm = warmupFraction(coolantTemperature);
    return m_params.revLimitCold + (m_params.revLimit - m_params.revLimitCold) * warm;
}

void powertrain::EngineControlUnit::update(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs,
    ActuatorCommands *commands)
{
    const double warm = warmupFraction(state.coolantTemperature);
    const double available = maxTorqueAt(state.engineSpeed);

    const double pedal = std::clamp(inputs.accelerator, 0.0, 1.0);
    const double pedalFraction = m_pedalMap.isInitialized()
        ? std::clamp(m_pedalMap.sample(pedal, 0.0), 0.0, 1.0)
        : pedal;

    m_driverTorqueRequest = pedalFraction * available;

    const double coldCap =
        m_params.coldStartTorqueCap + (1.0 - m_params.coldStartTorqueCap) * warm;
    m_driverTorqueRequest = std::min(m_driverTorqueRequest, available * coldCap);

    double idleTarget = idleSpeedAt(state.coolantTemperature);
    if (m_bus.speedRequestActive) {
        idleTarget = std::max(idleTarget, m_bus.speedRequest);
    }

    const double idleTrim = m_idleTrim.isInitialized()
        ? m_idleTrim.sample(state.coolantTemperature, 0.0)
        : 0.0;

    const double idleAuthority =
        m_idleController.update(dt, idleTarget, state.engineSpeed, idleTrim);
    m_idleTorqueRequest = idleAuthority * available;

    double coordinated = std::max(m_driverTorqueRequest, m_idleTorqueRequest);
    coordinated *= std::clamp(1.0 - m_bus.torqueReductionRequest, 0.0, 1.0);

    m_torqueLimiter.setRates(m_params.torqueRiseRate, m_params.torqueFallRate);
    m_torqueRequest = m_torqueLimiter.update(dt, coordinated);

    m_feedforwardPlate = m_throttleMap.isInitialized()
        ? std::clamp(
            m_throttleMap.sample(state.engineSpeed, m_torqueRequest)
                * m_feedforwardScale,
            0.0,
            1.0)
        : 0.0;

    const double correction =
        m_torqueController.update(dt, m_torqueRequest, state.indicatedTorque);

    double plate = std::clamp(m_feedforwardPlate + correction, 0.0, 1.0);

    const double applied = plate - m_feedforwardPlate;
    if (applied != correction) {
        m_torqueController.setIntegrator(
            m_torqueController.getIntegrator()
                + m_torqueController.getParameters().trackingGain
                    * (applied - correction) * dt);
    }

    const double revLimit = effectiveRevLimit(state.coolantTemperature);
    const double softLimitStart = revLimit - m_params.softLimitBand;
    double ignitionCut = 0.0;
    if (m_params.softLimitBand > 0.0 && state.engineSpeed > softLimitStart) {
        ignitionCut = std::clamp(
            (state.engineSpeed - softLimitStart) / m_params.softLimitBand,
            0.0,
            1.0);
    }

    if (m_bus.torqueReductionRequest > 0.0
        && m_bus.interventionType == TorqueIntervention::Spark)
    {
        ignitionCut = std::max(ignitionCut, m_bus.torqueReductionRequest);
    }

    double fuelCut = 0.0;
    if (state.engineSpeed > revLimit + m_params.hardLimitOffset) fuelCut = 1.0;

    const bool coasting = (pedal <= 0.0) && (state.gear != -1);
    m_overrunCut.setThresholds(
        m_params.overrunResumeSpeed, m_params.overrunCutSpeed);
    const bool overrun = m_overrunCut.update(state.engineSpeed) && coasting;
    if (overrun) fuelCut = 1.0;
    if (!coasting) m_overrunCut.setState(false);

    if (m_bus.torqueReductionRequest > 0.0
        && m_bus.interventionType == TorqueIntervention::Fuel)
    {
        fuelCut = std::max(fuelCut, m_bus.torqueReductionRequest);
    }

    const bool limiterActive = (ignitionCut > 0.0) || (fuelCut > 0.0 && !overrun);
    m_engineState = resolveState(state, inputs, limiterActive);

    m_longTermTrim = m_lambdaTrim.isInitialized()
        ? m_lambdaTrim.sample(state.engineSpeed, lambdaTrimLoad(state))
        : 0.0;

    const double enrichment =
        m_params.coldStartEnrichment + (1.0 - m_params.coldStartEnrichment) * warm;

    if (m_engineState == EngineState::Off) {
        plate = 0.0;
        fuelCut = 1.0;
        m_idleController.reset();
        m_torqueController.reset();
        m_torqueLimiter.reset(0.0);
    }
    else if (m_engineState == EngineState::Cranking) {
        plate = std::max(plate, 0.08);
        fuelCut = 0.0;
    }

    m_commandedPlate = plate;
    commands->throttlePlate = plate;
    commands->revLimit = revLimit + m_params.hardLimitOffset;
    commands->softLimitStart = softLimitStart;
    commands->limiterDuration = m_params.limiterDuration;
    commands->ignitionCutFraction = ignitionCut;
    commands->fuelCutFraction = std::clamp(fuelCut, 0.0, 1.0);
    commands->fuelEnrichment = enrichment * m_fuelTrim * (1.0 + m_longTermTrim);
    commands->timingOffset = -m_params.coldStartTimingRetard * (1.0 - warm);
    commands->timingAdvanceValid =
        m_params.timingMapEnabled && m_timingMap.isInitialized();
    commands->timingAdvance = commands->timingAdvanceValid
        ? m_timingMap.sample(state.engineSpeed, m_torqueRequest)
        : 0.0;
    commands->ignitionEnabled = inputs.ignitionKey;
    commands->starterEnabled =
        inputs.starterRequest && state.engineSpeed < m_params.crankingSpeed;

    m_bus.engineSpeed = state.engineSpeed;
    m_bus.indicatedTorque = state.indicatedTorque;
    m_bus.engineState = m_engineState;
}

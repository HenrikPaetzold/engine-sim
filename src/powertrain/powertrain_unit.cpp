#include "../../include/powertrain/powertrain_unit.h"

#include "../../include/config/config_server.h"
#include "../../include/config/channel_recorder.h"

powertrain::PowertrainUnit::PowertrainUnit() {
    /* void */
}

powertrain::PowertrainUnit::~PowertrainUnit() {
    /* void */
}

void powertrain::PowertrainUnit::initialize(
    const EngineControlUnit::Parameters &engineParams,
    const TransmissionControlUnit::Parameters &transmissionParams)
{
    m_ecu.initialize(engineParams);
    m_tcu.initialize(transmissionParams);

    reset();
}

void powertrain::PowertrainUnit::reset() {
    PowertrainController::reset();

    m_ecu.reset();
    m_tcu.reset();
}

void powertrain::PowertrainUnit::registerParameters(
    config::ParameterRegistry *registry,
    const char *prefix)
{
    m_ecu.registerParameters(registry, prefix);
    m_tcu.registerParameters(registry, prefix);
}

namespace {
    const char *engineStateName(powertrain::EngineState state) {
        switch (state) {
        case powertrain::EngineState::Off: return "Off";
        case powertrain::EngineState::Cranking: return "Cranking";
        case powertrain::EngineState::ColdStart: return "ColdStart";
        case powertrain::EngineState::Idle: return "Idle";
        case powertrain::EngineState::Running: return "Running";
        case powertrain::EngineState::Limiting: return "Limiting";
        default: return "Unknown";
        }
    }

    const char *shiftStateName(powertrain::ShiftState state) {
        switch (state) {
        case powertrain::ShiftState::Idle: return "Idle";
        case powertrain::ShiftState::TorqueReduction: return "TorqueReduction";
        case powertrain::ShiftState::ClutchRelease: return "ClutchRelease";
        case powertrain::ShiftState::GearChange: return "GearChange";
        case powertrain::ShiftState::SpeedMatch: return "SpeedMatch";
        case powertrain::ShiftState::ClutchOverlap: return "ClutchOverlap";
        case powertrain::ShiftState::ClutchEngage: return "ClutchEngage";
        default: return "Unknown";
        }
    }
}

void powertrain::PowertrainUnit::fillTelemetry(config::TelemetrySample *sample) const {
    if (sample == nullptr) return;

    sample->torqueRequest = m_ecu.getTorqueRequest();
    sample->idleIntegrator = m_ecu.getIdleController().getIntegrator();
    sample->torqueCorrection = m_ecu.getTorqueController().getOutput();
    sample->fuelTrim = m_ecu.getFuelTrim();
    sample->engineState = engineStateName(m_ecu.getEngineState());

    sample->shiftState = shiftStateName(m_tcu.getShiftState());
    sample->range = m_tcu.getPosition().name;
    sample->parkLock = m_tcu.getEngagement() == powertrain::GateEngagement::Park;
    sample->shiftIterations = m_tcu.getEngageProfile().getIterationCount();
    sample->shiftErrorNorm = m_tcu.getEngageProfile().getLastErrorNorm();
}

namespace {
    void addController(
        config::ChannelTable *table,
        const std::string &prefix,
        const control::PidController &pid)
    {
        table->set(prefix + ".error", pid.getError());
        table->set(prefix + ".p", pid.getProportionalTerm());
        table->set(prefix + ".i", pid.getIntegrator());
        table->set(prefix + ".d", pid.getDerivativeTerm());
        table->set(prefix + ".output", pid.getOutput());
        table->set(prefix + ".saturated", pid.isSaturated() ? 1.0 : 0.0);
    }
}

void powertrain::PowertrainUnit::fillChannels(config::ChannelTable *table) const {
    if (table == nullptr) return;

    addController(table, "pid.ecu.idle", m_ecu.getIdleController());
    addController(table, "pid.ecu.torque", m_ecu.getTorqueController());
    addController(table, "pid.tcu.slip", m_tcu.getSlipController());
    addController(table, "pid.tcu.lockup", m_tcu.getLockupController());

    table->set("ecu.torque_request", m_ecu.getTorqueRequest());
    table->set("ecu.fuel_trim", m_ecu.getFuelTrim());
    table->set("tcu.engage_phase", m_tcu.getEngagePhase());
    table->set("tcu.shift_error_norm", m_tcu.getEngageProfile().getLastErrorNorm());
    table->set("tcu.shifts_learned", m_tcu.getEngageProfile().getIterationCount());
    table->set("tcu.target_gear", m_tcu.getTargetGear());
    table->set("tcu.active_clutch", m_tcu.getActiveClutch());
    table->set("tcu.pedal_rate", m_tcu.getPedalRate());
    table->set("tcu.shifting", m_tcu.isShifting() ? 1.0 : 0.0);
}

void powertrain::PowertrainUnit::update(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs,
    ActuatorCommands *commands)
{
    m_tcu.update(dt, state, inputs, commands);
    m_ecu.setTransmissionRequests(m_tcu.getBus());
    m_ecu.update(dt, state, inputs, commands);

    m_bus = m_ecu.getBus();
    m_bus.shiftInProgress = m_tcu.getBus().shiftInProgress;
}

const std::string &powertrain::PowertrainUnit::getRequestedMode() const {
    return m_tcu.getRequestedMode();
}

const std::string &powertrain::PowertrainUnit::getPositionName() const {
    return m_tcu.getPosition().name;
}

void powertrain::PowertrainUnit::configureGearbox(
    const GearboxCapabilities &capabilities)
{
    m_tcu.configureGearbox(capabilities);
}

#include "../../include/powertrain/powertrain_unit.h"

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

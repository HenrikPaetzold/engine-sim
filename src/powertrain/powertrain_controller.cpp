#include "../../include/powertrain/powertrain_controller.h"

void powertrain::PowertrainController::registerParameters(
    config::ParameterRegistry *registry,
    const char *prefix)
{
    /* void */
}

void powertrain::PowertrainController::fillTelemetry(config::TelemetrySample *sample) const {
    /* void */
}

void powertrain::PowertrainController::reset() {
    m_bus = PowertrainBus();
}

void powertrain::PowertrainController::configureGearbox(
    const GearboxCapabilities &capabilities)
{
    /* void */
}

const std::string &powertrain::PowertrainController::getRequestedMode() const {
    static const std::string empty;
    return empty;
}

const std::string &powertrain::PowertrainController::getPositionName() const {
    static const std::string empty;
    return empty;
}

void powertrain::PowertrainController::fillChannels(config::ChannelTable *table) const {
    (void)table;
}

#include "../../include/powertrain/powertrain_controller.h"

void powertrain::PowertrainController::registerParameters(
    config::ParameterRegistry *registry,
    const char *prefix)
{
    /* void */
}

void powertrain::PowertrainController::reset() {
    m_bus = PowertrainBus();
}

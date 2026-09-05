#include "../include/powertrain_bootstrap.h"

#include "../include/powertrain_system.h"
#include "../include/config/config_server.h"

powertrain::BootstrapResult powertrain::installPowertrain(
    const BootstrapInputs &inputs,
    const BootstrapContext &context)
{
    BootstrapResult result;

    if (context.system == nullptr || context.registry == nullptr) return result;
    if (inputs.unit == nullptr && inputs.program == nullptr) return result;

    const bool overlayProgram =
        inputs.program != nullptr && inputs.unit != nullptr;

    result.controller = (inputs.program != nullptr && !overlayProgram)
        ? static_cast<PowertrainController *>(inputs.program)
        : static_cast<PowertrainController *>(inputs.unit);

    if (inputs.program != nullptr) inputs.program->setOverlay(overlayProgram);
    result.overlay = overlayProgram
        ? static_cast<PowertrainController *>(inputs.program)
        : nullptr;

    PowertrainSystem &system = *context.system;
    system.initialize(PowertrainSystem::Parameters());
    system.setController(result.controller);
    system.setOverlayController(result.overlay);

    if (context.modes != nullptr) {
        system.setDriveModes(context.modes, context.registry);
    }

    if (result.controller == inputs.unit
        && inputs.unit != nullptr
        && context.adaptation != nullptr)
    {
        context.adaptation->initialize(inputs.adaptation);
        context.adaptation->attach(
            &inputs.unit->getEngineControlUnit(),
            &inputs.unit->getTransmissionControlUnit());

        system.setAdaptationManager(context.adaptation);
        result.adaptationAttached = true;
    }

    if (context.simulator != nullptr) system.attach(context.simulator);

    system.registerParameters(context.registry);

    for (const auto &override : inputs.parameterOverrides) {
        context.registry->set(override.first, override.second);
    }

    if (!inputs.defaultMode.empty() && context.modes != nullptr) {
        result.defaultModeIndex = context.modes->find(inputs.defaultMode);
        context.modes->select(inputs.defaultMode, context.registry);
    }

    if (context.server != nullptr) {
        config::ConfigServer::Parameters serverParams;
        serverParams.uiPath = context.uiPath;

        context.server->initialize(serverParams, context.registry, context.modes);

        if (context.server->start()) {
            system.setConfigServer(context.server);
            result.serverStarted = true;
        }
    }

    return result;
}

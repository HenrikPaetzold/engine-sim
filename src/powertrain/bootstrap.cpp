#include "../../include/powertrain/bootstrap.h"

#include "../../include/powertrain_system.h"
#include "../../include/config/config_server.h"
#include "../../include/simulator.h"
#include "../../include/engine.h"

powertrain::ControllerSelection powertrain::selectControllers(
    PowertrainUnit *unit,
    ScriptedControlUnit *program)
{
    ControllerSelection selection;

    if (program != nullptr && unit != nullptr) {
        selection.mode = ControlMode::ScriptOverlay;
        selection.primary = unit;
        selection.overlay = program;
    }
    else if (program != nullptr) {
        selection.mode = ControlMode::ScriptOnly;
        selection.primary = program;
    }
    else {
        selection.mode = ControlMode::ControlUnits;
        selection.primary = unit;
    }

    return selection;
}

powertrain::BootstrapResult powertrain::installPowertrain(
    const BootstrapInputs &inputs,
    const BootstrapContext &context)
{
    BootstrapResult result;

    if (context.system == nullptr || context.registry == nullptr) return result;
    if (inputs.unit == nullptr && inputs.program == nullptr) return result;

    const ControllerSelection selection =
        selectControllers(inputs.unit, inputs.program);

    result.controller = selection.primary;
    result.overlay = selection.overlay;

    if (inputs.program != nullptr) {
        inputs.program->setOverlay(selection.mode == ControlMode::ScriptOverlay);
    }

    PowertrainSystem &system = *context.system;

    PowertrainSystem::Parameters systemParams;
    if (inputs.driverPedalTimeConstant >= 0.0) {
        systemParams.pedalTimeConstant = inputs.driverPedalTimeConstant;
    }
    if (inputs.driverClutchTimeConstant >= 0.0) {
        systemParams.clutchTimeConstant = inputs.driverClutchTimeConstant;
    }
    if (inputs.driverClutchPedalRate >= 0.0) {
        systemParams.clutchPedalRate = inputs.driverClutchPedalRate;
    }

    system.initialize(systemParams);
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

    if (context.simulator != nullptr) {
        system.attach(context.simulator);

        Engine *engine = context.simulator->getEngine();
        if (inputs.thermalAuthored && engine != nullptr) {
            engine->getThermalModel().initialize(inputs.thermal);
        }
    }

    system.registerParameters(context.registry);

    for (const AdaptiveOverride &override : inputs.adaptiveOverrides) {
        context.registry->setAdaptive(
            override.path,
            override.adaptive,
            override.adaptMin,
            override.adaptMax);
    }

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

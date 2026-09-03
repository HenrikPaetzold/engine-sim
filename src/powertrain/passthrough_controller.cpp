#include "../../include/powertrain/passthrough_controller.h"

#include "../../include/config/parameter_registry.h"

#include <algorithm>
#include <cmath>
#include <string>

powertrain::PassthroughController::PassthroughController() {
    m_previousShiftUp = false;
    m_previousShiftDown = false;
}

powertrain::PassthroughController::~PassthroughController() {
    /* void */
}

void powertrain::PassthroughController::initialize(const Parameters &params) {
    m_params = params;
    reset();
}

double powertrain::PassthroughController::plateFromPedal(double accelerator, double gamma) {
    const double pedal = std::clamp(accelerator, 0.0, 1.0);
    return std::pow(pedal, gamma);
}

void powertrain::PassthroughController::registerParameters(
    config::ParameterRegistry *registry,
    const char *prefix)
{
    if (registry == nullptr) return;

    const std::string base = std::string(prefix) + "passthrough.";

    config::ParameterDescriptor gamma;
    gamma.path = base + "throttle_gamma";
    gamma.minValue = 0.1;
    gamma.maxValue = 8.0;
    gamma.defaultValue = m_params.throttleGamma;
    gamma.unit = "";
    registry->registerScalar(gamma, &m_params.throttleGamma);
}

void powertrain::PassthroughController::reset() {
    m_previousShiftUp = false;
    m_previousShiftDown = false;
    m_bus = PowertrainBus();
}

void powertrain::PassthroughController::update(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs,
    ActuatorCommands *commands)
{
    commands->throttlePlate = plateFromPedal(inputs.accelerator, m_params.throttleGamma);

    commands->clutchPressure[0] = std::clamp(1.0 - inputs.clutchPedal, 0.0, 1.0);
    commands->clutchPressure[1] = 0.0;
    commands->lockupPressure = 0.0;

    commands->ignitionEnabled = inputs.ignitionKey;
    commands->starterEnabled = inputs.starterRequest;
    commands->ignitionCutFraction = 0.0;
    commands->fuelCutFraction = 0.0;
    commands->timingOffset = 0.0;

    int gear = state.gear;
    if (inputs.shiftUpRequest && !m_previousShiftUp) {
        if (gear + 1 < state.gearCount) ++gear;
    }
    else if (inputs.shiftDownRequest && !m_previousShiftDown) {
        if (gear - 1 >= -1) --gear;
    }

    m_previousShiftUp = inputs.shiftUpRequest;
    m_previousShiftDown = inputs.shiftDownRequest;

    commands->targetGear = gear;
    commands->preselectGear = -1;

    m_bus.engineSpeed = state.engineSpeed;
    m_bus.indicatedTorque = state.indicatedTorque;
    m_bus.engineState = state.engineRunning
        ? EngineState::Running
        : EngineState::Off;
}

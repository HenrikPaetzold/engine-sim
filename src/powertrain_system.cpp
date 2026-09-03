#include "../include/powertrain_system.h"

#include "../include/simulator.h"
#include "../include/units.h"

#include <algorithm>
#include <cmath>

PowertrainSystem::PowertrainSystem() {
    m_controller = nullptr;
    m_simulator = nullptr;
    m_previousThrottle = nullptr;
    m_accumulator = 0.0;
    m_time = 0.0;
}

PowertrainSystem::~PowertrainSystem() {
    detach();
}

void PowertrainSystem::initialize(const Parameters &params) {
    m_params = params;
    if (m_params.controlFrequency <= 0.0) m_params.controlFrequency = 1000.0;
}

void PowertrainSystem::setController(powertrain::PowertrainController *controller) {
    m_controller = controller;
    reset();
}

void PowertrainSystem::attach(Simulator *simulator) {
    detach();

    m_simulator = simulator;
    if (m_simulator == nullptr) return;

    Engine *engine = m_simulator->getEngine();
    if (engine == nullptr) {
        m_simulator = nullptr;
        return;
    }

    m_previousThrottle = engine->replaceThrottleController(&m_throttle);
    m_throttle.setPlatePosition(0.0);

    reset();
}

void PowertrainSystem::detach() {
    if (m_simulator != nullptr) {
        Engine *engine = m_simulator->getEngine();
        if (engine != nullptr && engine->getThrottleController() == &m_throttle) {
            engine->replaceThrottleController(m_previousThrottle);
        }
    }

    m_previousThrottle = nullptr;
    m_simulator = nullptr;
}

void PowertrainSystem::reset() {
    m_accumulator = 0.0;
    m_time = 0.0;
    m_state = powertrain::PowertrainState();
    m_commands = powertrain::ActuatorCommands();

    if (m_controller != nullptr) m_controller->reset();
}

void PowertrainSystem::sampleState(double dt) {
    if (m_simulator == nullptr) return;

    Engine *engine = m_simulator->getEngine();
    Transmission *transmission = m_simulator->getTransmission();
    Vehicle *vehicle = m_simulator->getVehicle();

    m_state.dt = dt;
    m_state.time = m_time;

    if (engine != nullptr) {
        m_state.engineSpeed = engine->getSpeed();
        m_state.engineRpm = engine->getRpm();
        m_state.throttlePlate = m_throttle.getPlatePosition();
        m_state.manifoldPressure = engine->getManifoldPressure();
        m_state.intakeAfr = engine->getIntakeAfr();
        m_state.exhaustO2 = engine->getExhaustO2();
        m_state.engineRunning = engine->getRpm() > 1.0;

        IgnitionModule *ignition = engine->getIgnitionModule();
        if (ignition != nullptr) m_state.timingAdvance = ignition->getTimingAdvance();
    }

    m_state.indicatedTorque = m_simulator->getFilteredDynoTorque();

    if (transmission != nullptr) {
        m_state.gear = transmission->getGear();
        m_state.gearCount = transmission->getGearCount();
        m_state.clutchPressure[0] = transmission->getClutchPressure();
        m_state.clutchSlipSpeed[0] = transmission->getClutchSlipSpeed();
    }

    if (vehicle != nullptr) {
        m_state.vehicleSpeed = vehicle->getSignedSpeed();
        m_state.wheelSpeed = vehicle->getRotationalSpeed();
    }
}

void PowertrainSystem::applyCommands() {
    if (m_simulator == nullptr) return;

    m_throttle.setPlatePosition(m_commands.throttlePlate);

    Engine *engine = m_simulator->getEngine();
    if (engine != nullptr) {
        IgnitionModule *ignition = engine->getIgnitionModule();
        if (ignition != nullptr) {
            ignition->m_enabled = m_commands.ignitionEnabled;
        }
    }

    Transmission *transmission = m_simulator->getTransmission();
    if (transmission != nullptr) {
        transmission->setClutchPressure(
            std::clamp(m_commands.clutchPressure[0], 0.0, 1.0));

        if (m_commands.targetGear != transmission->getGear()) {
            transmission->changeGear(m_commands.targetGear);
        }
    }

    m_simulator->m_starterMotor.m_enabled = m_commands.starterEnabled;
}

void PowertrainSystem::update(double dt) {
    if (!isActive()) return;

    m_time += dt;
    m_accumulator += dt;

    const double controlPeriod = 1.0 / m_params.controlFrequency;
    if (m_accumulator < controlPeriod) return;

    const double controlDt = m_accumulator;
    m_accumulator = 0.0;

    sampleState(controlDt);
    m_controller->update(controlDt, m_state, m_inputs, &m_commands);
    applyCommands();
}

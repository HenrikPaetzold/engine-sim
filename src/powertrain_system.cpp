#include "../include/powertrain_system.h"

#include "../include/simulator.h"
#include "../include/units.h"

#include <algorithm>
#include <cmath>

PowertrainSystem::PowertrainSystem() {
    m_controller = nullptr;
    m_adaptation = nullptr;
    m_server = nullptr;
    m_simulator = nullptr;
    m_previousThrottle = nullptr;
    m_accumulator = 0.0;
    m_telemetryAccumulator = 0.0;
    m_time = 0.0;
    m_roadGrade = 0.0;
    m_ambientTemperature = units::celcius(20.0);
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

void PowertrainSystem::setAdaptationManager(adaptation::AdaptationManager *manager) {
    m_adaptation = manager;
}

void PowertrainSystem::setConfigServer(config::ConfigServer *server) {
    m_server = server;
}

void PowertrainSystem::registerParameters(config::ParameterRegistry *registry) {
    if (registry == nullptr) return;

    config::ParameterDescriptor grade;
    grade.path = "environment.road_grade";
    grade.minValue = -0.30;
    grade.maxValue = 0.30;
    grade.defaultValue = 0.0;
    grade.unit = "rad";
    registry->registerScalar(grade, &m_roadGrade);

    config::ParameterDescriptor ambient;
    ambient.path = "environment.ambient_temperature";
    ambient.minValue = units::celcius(-40.0);
    ambient.maxValue = units::celcius(60.0);
    ambient.defaultValue = units::celcius(20.0);
    ambient.unit = "K";
    registry->registerScalar(ambient, &m_ambientTemperature);

    config::ParameterDescriptor frequency;
    frequency.path = "control.frequency";
    frequency.minValue = 20.0;
    frequency.maxValue = 10000.0;
    frequency.defaultValue = m_params.controlFrequency;
    frequency.unit = "Hz";
    registry->registerScalar(frequency, &m_params.controlFrequency);

    if (m_controller != nullptr) m_controller->registerParameters(registry, "");
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
    m_telemetryAccumulator = 0.0;
    m_time = 0.0;
    m_state = powertrain::PowertrainState();
    m_commands = powertrain::ActuatorCommands();

    if (m_controller != nullptr) m_controller->reset();
    if (m_adaptation != nullptr) m_adaptation->reset();
}

void PowertrainSystem::publishTelemetry() {
    if (m_server == nullptr) return;

    config::TelemetrySample sample;
    sample.time = m_time;
    sample.engineRpm = m_state.engineRpm;
    sample.throttlePlate = m_state.throttlePlate;
    sample.indicatedTorque = m_state.indicatedTorque;
    sample.coolantTemperature = m_state.coolantTemperature;
    sample.oilTemperature = m_state.oilTemperature;
    sample.vehicleSpeed = m_state.vehicleSpeed;
    sample.roadGrade = m_state.roadGrade;
    sample.gear = m_state.gear;
    sample.clutchPressure = m_state.clutchPressure[0];
    sample.ignitionCut = m_commands.ignitionCutFraction;
    sample.fuelCut = m_commands.fuelCutFraction;

    if (m_controller != nullptr) m_controller->fillTelemetry(&sample);
    if (m_adaptation != nullptr) {
        sample.adaptionEnabled = m_adaptation->wasEnabledLastUpdate();
        sample.shiftIterations = m_adaptation->getShiftIterationCount();
        sample.shiftErrorNorm = m_adaptation->getShiftErrorNorm();
    }

    m_server->publish(sample);
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
        m_state.coolantTemperature = engine->getCoolantTemperature();
        m_state.oilTemperature = engine->getOilTemperature();

        IgnitionModule *ignition = engine->getIgnitionModule();
        if (ignition != nullptr) m_state.timingAdvance = ignition->getTimingAdvance();
    }

    m_state.indicatedTorque = (engine != nullptr)
        ? engine->getIndicatedTorque()
        : 0.0;

    if (transmission != nullptr) {
        m_state.gear = transmission->getGear();
        m_state.gearCount = transmission->getGearCount();
        m_state.clutchPressure[0] = transmission->getClutchPressure();
        m_state.clutchSlipSpeed[0] = transmission->getClutchSlipSpeed();
    }

    if (vehicle != nullptr) {
        m_state.vehicleSpeed = vehicle->getSignedSpeed();
        m_state.wheelSpeed = vehicle->getRotationalSpeed();
        m_state.roadGrade = vehicle->getRoadGrade();
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
            ignition->setCutFraction(m_commands.ignitionCutFraction);
            ignition->setTimingOffset(m_commands.timingOffset);
        }

        const double fuelFactor =
            (1.0 - std::clamp(m_commands.fuelCutFraction, 0.0, 1.0))
            * std::max(m_commands.fuelEnrichment, 0.0);
        engine->setFuelFactor(std::clamp(fuelFactor, 0.0, 4.0));
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

    Vehicle *vehicle = m_simulator->getVehicle();
    if (vehicle != nullptr) vehicle->setRoadGrade(m_roadGrade);

    if (engine != nullptr) {
        engine->getThermalModel().getParameters().ambientTemperature =
            m_ambientTemperature;
    }
}

void PowertrainSystem::update(double dt) {
    if (!isActive()) return;

    m_time += dt;
    m_accumulator += dt;

    const double controlPeriod =
        1.0 / std::max(m_params.controlFrequency, 1.0);
    if (m_accumulator < controlPeriod) return;

    const double controlDt = m_accumulator;
    m_accumulator = 0.0;

    sampleState(controlDt);
    m_controller->update(controlDt, m_state, m_inputs, &m_commands);

    if (m_adaptation != nullptr) {
        m_adaptation->update(controlDt, m_state, m_controller->getBus());
    }

    applyCommands();

    m_telemetryAccumulator += controlDt;
    const double telemetryPeriod =
        1.0 / std::max(m_params.telemetryFrequency, 1.0);

    if (m_telemetryAccumulator >= telemetryPeriod) {
        m_telemetryAccumulator = 0.0;

        if (m_server != nullptr) m_server->applyPendingCommands();
        publishTelemetry();
    }
}

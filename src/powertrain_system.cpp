#include "../include/powertrain_system.h"

#include "../include/simulator.h"
#include "../include/units.h"

#include <algorithm>
#include <cmath>

PowertrainSystem::PowertrainSystem() {
    m_controller = nullptr;
    m_adaptation = nullptr;
    m_server = nullptr;
    m_modes = nullptr;
    m_registry = nullptr;
    m_simulator = nullptr;
    m_previousThrottle = nullptr;
    m_accumulator = 0.0;
    m_telemetryAccumulator = 0.0;
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
    syncGearbox();
    reset();
}

void PowertrainSystem::setAdaptationManager(adaptation::AdaptationManager *manager) {
    m_adaptation = manager;
}

void PowertrainSystem::setConfigServer(config::ConfigServer *server) {
    m_server = server;
}

void PowertrainSystem::setDriveModes(
    config::DriveModeSet *modes,
    config::ParameterRegistry *registry)
{
    m_modes = modes;
    m_registry = registry;
    m_activeMode.clear();
}

void PowertrainSystem::applyGateMode() {
    if (m_controller == nullptr || m_modes == nullptr || m_registry == nullptr) return;

    const std::string &requested = m_controller->getRequestedMode();
    if (requested.empty() || requested == m_activeMode) return;

    if (m_modes->select(requested, m_registry)) m_activeMode = requested;
}

void PowertrainSystem::registerParameters(config::ParameterRegistry *registry) {
    if (registry == nullptr) return;

    registry->registerScalar(
        config::describeScalar(
            "control.frequency", 20.0, 10000.0, m_params.controlFrequency, "Hz"),
        &m_params.controlFrequency);
    registry->registerScalar(
        config::describeScalar(
            "control.telemetry_frequency", 1.0, 200.0, m_params.telemetryFrequency, "Hz"),
        &m_params.telemetryFrequency);

    if (m_simulator != nullptr) {
        Vehicle *vehicle = m_simulator->getVehicle();
        if (vehicle != nullptr) vehicle->registerParameters(registry, "");

        Engine *engine = m_simulator->getEngine();
        if (engine != nullptr) engine->getThermalModel().registerParameters(registry, "");

        Transmission *transmission = m_simulator->getTransmission();
        if (transmission != nullptr) transmission->registerParameters(registry, "");
    }

    if (m_controller != nullptr) m_controller->registerParameters(registry, "");
}

bool PowertrainSystem::selectDriveMode(
    const std::string &name,
    config::DriveModeSet *modes,
    config::ParameterRegistry *registry)
{
    if (name.empty() || modes == nullptr || registry == nullptr) return false;

    return modes->select(name, registry);
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

    syncGearbox();

    reset();
}

void PowertrainSystem::syncGearbox() {
    if (m_controller == nullptr || m_simulator == nullptr) return;

    Transmission *transmission = m_simulator->getTransmission();
    if (transmission == nullptr) return;

    powertrain::GearboxCapabilities capabilities;
    capabilities.gearCount = transmission->getGearCount();
    capabilities.gearRatios = transmission->getGearRatios();
    capabilities.supportsPreselect = transmission->supportsPreselect();
    capabilities.requiresTorqueInterrupt = transmission->requiresTorqueInterrupt();
    capabilities.hasLaunchDevice = transmission->hasLaunchDevice();
    capabilities.supportsRange = transmission->supportsEngagement();

    Vehicle *vehicle = m_simulator->getVehicle();
    if (vehicle != nullptr) {
        capabilities.finalDrive = vehicle->getDiffRatio();
        capabilities.tireRadius = vehicle->getTireRadius();
    }

    m_controller->configureGearbox(capabilities);
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

void PowertrainSystem::fillTelemetry(config::TelemetrySample *sample) const {
    if (sample == nullptr) return;

    config::TelemetrySample &out = *sample;
    out.time = m_time;
    out.engineRpm = m_state.engineRpm;
    out.throttlePlate = m_state.throttlePlate;
    out.indicatedTorque = m_state.indicatedTorque;
    out.coolantTemperature = m_state.coolantTemperature;
    out.oilTemperature = m_state.oilTemperature;
    out.vehicleSpeed = m_state.vehicleSpeed;
    out.roadGrade = m_state.roadGrade;
    out.gear = m_state.gear;

    out.parkLock = m_state.parkLockEngaged;
    out.clutchPressure = m_state.clutchPressure[0];
    out.ignitionCut = m_commands.ignitionCutFraction;
    out.fuelCut = m_commands.fuelCutFraction;

    if (m_controller != nullptr) m_controller->fillTelemetry(&out);
    if (m_adaptation != nullptr) {
        out.adaptionEnabled = m_adaptation->wasEnabledLastUpdate();
        out.shiftIterations = m_adaptation->getShiftIterationCount();
        out.shiftErrorNorm = m_adaptation->getShiftErrorNorm();
    }
}

void PowertrainSystem::publishTelemetry() {
    if (m_server == nullptr) return;

    config::TelemetrySample sample;
    fillTelemetry(&sample);

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
        m_state.preselectedGear = transmission->getPreselectedGear();
        m_state.gearCount = transmission->getGearCount();

        for (int i = 0; i < powertrain::MaxClutches; ++i) {
            m_state.clutchPressure[i] = transmission->getClutchPressure(i);
            m_state.clutchSlipSpeed[i] = transmission->getClutchSlipSpeed(i);
        }

        m_state.turbineSpeed = transmission->getTurbineSpeed();
        m_state.converterSlip = transmission->getConverterSlip();
        m_state.lockupPressure = transmission->getLockupPressure();
        m_state.engagement = transmission->getEngagement();
        m_state.gatePosition = m_commands.gatePosition;
        m_state.parkLockEngaged = transmission->isParkLockEngaged();
    }

    if (vehicle != nullptr) {
        vehicle->setBrake(std::clamp(m_inputs.brake, 0.0, 1.0));
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

            if (m_commands.revLimit > 0.0) {
                ignition->setRevLimit(m_commands.revLimit);
            }
            if (m_commands.limiterDuration > 0.0) {
                ignition->setLimiterDuration(m_commands.limiterDuration);
            }
        }

        const double fuelFactor =
            (1.0 - std::clamp(m_commands.fuelCutFraction, 0.0, 1.0))
            * std::max(m_commands.fuelEnrichment, 0.0);
        engine->setFuelFactor(std::clamp(fuelFactor, 0.0, 4.0));
    }

    Transmission *transmission = m_simulator->getTransmission();
    if (transmission != nullptr) {
        transmission->setEngagement(m_commands.engagement);

        for (int i = 0; i < powertrain::MaxClutches; ++i) {
            transmission->setClutchPressure(
                i, std::clamp(m_commands.clutchPressure[i], 0.0, 1.0));
        }

        if (transmission->supportsPreselect()) {
            transmission->setPreselectedGear(m_commands.preselectGear);
        }

        if (transmission->hasLaunchDevice()) {
            transmission->setLockupPressure(
                std::clamp(m_commands.lockupPressure, 0.0, 1.0));
        }

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

    applyGateMode();
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

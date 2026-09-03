#include "../../include/powertrain/scripted_control_unit.h"

#include "../../include/config/parameter_registry.h"
#include "../../include/config/config_server.h"

#include <algorithm>
#include <cmath>

namespace {
    const char *const s_signalNames[] = {
        "dt",
        "time",
        "engine_speed",
        "engine_rpm",
        "throttle_plate",
        "manifold_pressure",
        "intake_afr",
        "exhaust_o2",
        "indicated_torque",
        "timing_advance",
        "coolant_temperature",
        "oil_temperature",
        "gear",
        "preselected_gear",
        "gear_count",
        "clutch_pressure",
        "clutch_pressure_2",
        "clutch_slip",
        "clutch_slip_2",
        "turbine_speed",
        "converter_slip",
        "lockup_pressure",
        "vehicle_speed",
        "wheel_speed",
        "road_grade",
        "engine_running",
        "accelerator",
        "brake",
        "clutch_pedal",
        "drive_mode",
        "selected_gear",
        "shift_up",
        "shift_down",
        "manual_mode",
        "gate_position",
        "engagement",
        "park_lock_engaged",
        "ignition_key",
        "starter_request" };

    static_assert(
        sizeof(s_signalNames) / sizeof(s_signalNames[0])
            == powertrain::signals::Count,
        "signal name table does not match the signal enum");

    const char *const s_actuatorNames[] = {
        "throttle_plate",
        "ignition_cut",
        "fuel_cut",
        "fuel_enrichment",
        "timing_offset",
        "rev_limit",
        "limiter_duration",
        "target_gear",
        "preselect_gear",
        "clutch_pressure",
        "clutch_pressure_2",
        "lockup_pressure",
        "starter_enabled",
        "ignition_enabled",
        "gate_position",
        "park_lock" };

    static_assert(
        sizeof(s_actuatorNames) / sizeof(s_actuatorNames[0])
            == powertrain::actuators::Count,
        "actuator name table does not match the actuator enum");
}

const char *powertrain::signals::name(int signal) {
    if (signal < 0 || signal >= Count) return "";
    return s_signalNames[signal];
}

const char *powertrain::actuators::name(int actuator) {
    if (actuator < 0 || actuator >= Count) return "";
    return s_actuatorNames[actuator];
}

powertrain::ScriptedControlUnit::ScriptedControlUnit() {
    initialize();
}

powertrain::ScriptedControlUnit::~ScriptedControlUnit() {
    /* void */
}

void powertrain::ScriptedControlUnit::initialize() {
    for (int i = 0; i < signals::Count; ++i) {
        m_program.getInputs().declare(signals::name(i));
    }

    for (int i = 0; i < actuators::Count; ++i) {
        m_program.getOutputs().declare(actuators::name(i));
    }
}

void powertrain::ScriptedControlUnit::registerParameters(
    config::ParameterRegistry *registry,
    const char *prefix)
{
    if (registry == nullptr) return;

    const std::string base = std::string(prefix) + "program.";

    for (int i = 0; i < m_program.getBlockCount(); ++i) {
        control::ControlBlock *block = m_program.getBlock(i);
        if (block == nullptr || block->m_name.empty()) continue;

        if (control::ConstantBlock *constant =
            dynamic_cast<control::ConstantBlock *>(block))
        {
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name, -1e9, 1e9, constant->m_value, ""),
                &constant->m_value);
        }
        else if (control::GainBlock *gain =
            dynamic_cast<control::GainBlock *>(block))
        {
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".gain", -1e9, 1e9, gain->m_gain, ""),
                &gain->m_gain);
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".offset", -1e9, 1e9, gain->m_offset, ""),
                &gain->m_offset);
        }
        else if (control::PidBlock *pid =
            dynamic_cast<control::PidBlock *>(block))
        {
            control::PidController::Parameters &params =
                pid->m_controller.getParametersMutable();

            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".kp", -1e6, 1e6, params.kp, ""),
                &params.kp);
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".ki", -1e6, 1e6, params.ki, ""),
                &params.ki);
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".kd", -1e6, 1e6, params.kd, ""),
                &params.kd);
        }
        else if (control::ClampBlock *clamp =
            dynamic_cast<control::ClampBlock *>(block))
        {
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".min", -1e9, 1e9, clamp->m_min, ""),
                &clamp->m_min);
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".max", -1e9, 1e9, clamp->m_max, ""),
                &clamp->m_max);
        }
        else if (control::RateLimitBlock *rate =
            dynamic_cast<control::RateLimitBlock *>(block))
        {
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".rise", 0.0, 1e9, rate->m_riseRate, "1/s"),
                &rate->m_riseRate);
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".fall", 0.0, 1e9, rate->m_fallRate, "1/s"),
                &rate->m_fallRate);
        }
        else if (control::LowPassBlock *lowPass =
            dynamic_cast<control::LowPassBlock *>(block))
        {
            registry->registerScalar(
                config::describeScalar(
                    base + block->m_name + ".tau", 0.0, 1e4, lowPass->m_timeConstant, "s"),
                &lowPass->m_timeConstant);
        }
    }
}

void powertrain::ScriptedControlUnit::fillTelemetry(config::TelemetrySample *sample) const {
    if (sample == nullptr) return;

    sample->engineState = "scripted";
    sample->shiftState = "scripted";
}

void powertrain::ScriptedControlUnit::reset() {
    m_program.reset();
    m_bus = PowertrainBus();
}

void powertrain::ScriptedControlUnit::sampleSignals(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs)
{
    control::SignalTable &table = m_program.getInputs();

    table.set(signals::Dt, dt);
    table.set(signals::Time, state.time);
    table.set(signals::EngineSpeed, state.engineSpeed);
    table.set(signals::EngineRpm, state.engineRpm);
    table.set(signals::ThrottlePlate, state.throttlePlate);
    table.set(signals::ManifoldPressure, state.manifoldPressure);
    table.set(signals::IntakeAfr, state.intakeAfr);
    table.set(signals::ExhaustO2, state.exhaustO2);
    table.set(signals::IndicatedTorque, state.indicatedTorque);
    table.set(signals::TimingAdvance, state.timingAdvance);
    table.set(signals::CoolantTemperature, state.coolantTemperature);
    table.set(signals::OilTemperature, state.oilTemperature);
    table.set(signals::Gear, state.gear);
    table.set(signals::PreselectedGear, state.preselectedGear);
    table.set(signals::GearCount, state.gearCount);
    table.set(signals::ClutchPressure, state.clutchPressure[0]);
    table.set(signals::ClutchPressure2, state.clutchPressure[1]);
    table.set(signals::ClutchSlip, state.clutchSlipSpeed[0]);
    table.set(signals::ClutchSlip2, state.clutchSlipSpeed[1]);
    table.set(signals::TurbineSpeed, state.turbineSpeed);
    table.set(signals::ConverterSlip, state.converterSlip);
    table.set(signals::LockupPressure, state.lockupPressure);
    table.set(signals::VehicleSpeed, state.vehicleSpeed);
    table.set(signals::WheelSpeed, state.wheelSpeed);
    table.set(signals::RoadGrade, state.roadGrade);
    table.set(signals::EngineRunning, state.engineRunning ? 1.0 : 0.0);

    table.set(signals::Accelerator, inputs.accelerator);
    table.set(signals::Brake, inputs.brake);
    table.set(signals::ClutchPedal, inputs.clutchPedal);
    table.set(signals::DriveMode, inputs.driveMode);
    table.set(signals::SelectedGear, inputs.selectedGear);
    table.set(signals::ShiftUp, inputs.shiftUpRequest ? 1.0 : 0.0);
    table.set(signals::ShiftDown, inputs.shiftDownRequest ? 1.0 : 0.0);
    table.set(signals::ManualMode, inputs.manualMode ? 1.0 : 0.0);
    table.set(signals::GatePosition, inputs.gatePosition);
    table.set(signals::Engagement, static_cast<int>(state.engagement));
    table.set(signals::ParkLockEngaged, state.parkLockEngaged ? 1.0 : 0.0);
    table.set(signals::IgnitionKey, inputs.ignitionKey ? 1.0 : 0.0);
    table.set(signals::StarterRequest, inputs.starterRequest ? 1.0 : 0.0);
}

void powertrain::ScriptedControlUnit::seedActuators(const PowertrainState &state) {
    control::SignalTable &table = m_program.getOutputs();
    const ActuatorCommands defaults;

    table.set(actuators::ThrottlePlate, defaults.throttlePlate);
    table.set(actuators::IgnitionCut, defaults.ignitionCutFraction);
    table.set(actuators::FuelCut, defaults.fuelCutFraction);
    table.set(actuators::FuelEnrichment, defaults.fuelEnrichment);
    table.set(actuators::TimingOffset, defaults.timingOffset);
    table.set(actuators::RevLimit, defaults.revLimit);
    table.set(actuators::LimiterDuration, defaults.limiterDuration);
    table.set(actuators::TargetGear, state.gear);
    table.set(actuators::PreselectGear, state.preselectedGear);
    table.set(actuators::ClutchPressure, state.clutchPressure[0]);
    table.set(actuators::ClutchPressure2, state.clutchPressure[1]);
    table.set(actuators::LockupPressure, state.lockupPressure);
    table.set(actuators::StarterEnabled, 0.0);
    table.set(actuators::IgnitionEnabled, 1.0);
    table.set(actuators::GatePosition, state.gatePosition);
    table.set(actuators::ParkLock, state.parkLockEngaged ? 1.0 : 0.0);
}

void powertrain::ScriptedControlUnit::applyActuators(ActuatorCommands *commands) const {
    const control::SignalTable &table = m_program.getOutputs();

    commands->throttlePlate =
        std::clamp(table.get(actuators::ThrottlePlate), 0.0, 1.0);
    commands->ignitionCutFraction =
        std::clamp(table.get(actuators::IgnitionCut), 0.0, 1.0);
    commands->fuelCutFraction =
        std::clamp(table.get(actuators::FuelCut), 0.0, 1.0);
    commands->fuelEnrichment =
        std::max(table.get(actuators::FuelEnrichment), 0.0);
    commands->timingOffset = table.get(actuators::TimingOffset);
    commands->revLimit = std::max(table.get(actuators::RevLimit), 0.0);
    commands->limiterDuration = std::max(table.get(actuators::LimiterDuration), 0.0);

    commands->targetGear =
        static_cast<int>(std::lround(table.get(actuators::TargetGear)));
    commands->preselectGear =
        static_cast<int>(std::lround(table.get(actuators::PreselectGear)));

    commands->clutchPressure[0] =
        std::clamp(table.get(actuators::ClutchPressure), 0.0, 1.0);
    commands->clutchPressure[1] =
        std::clamp(table.get(actuators::ClutchPressure2), 0.0, 1.0);
    commands->lockupPressure =
        std::clamp(table.get(actuators::LockupPressure), 0.0, 1.0);

    commands->starterEnabled = table.get(actuators::StarterEnabled) >= 0.5;
    commands->ignitionEnabled = table.get(actuators::IgnitionEnabled) >= 0.5;

    commands->gatePosition =
        static_cast<int>(std::lround(table.get(actuators::GatePosition)));
    commands->parkLock = table.get(actuators::ParkLock) >= 0.5;
}

void powertrain::ScriptedControlUnit::update(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs,
    ActuatorCommands *commands)
{
    if (commands == nullptr) return;

    sampleSignals(dt, state, inputs);
    seedActuators(state);

    m_program.update(dt);

    applyActuators(commands);

    m_bus.engineSpeed = state.engineSpeed;
    m_bus.indicatedTorque = state.indicatedTorque;
}

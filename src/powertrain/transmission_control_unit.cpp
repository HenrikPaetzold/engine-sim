#include "../../include/powertrain/transmission_control_unit.h"

#include "../../include/config/parameter_registry.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace {
    constexpr int PedalPoints = 5;

    config::ParameterDescriptor describe(
        const std::string &path,
        double min,
        double max,
        double defaultValue,
        const char *unit)
    {
        config::ParameterDescriptor d;
        d.path = path;
        d.minValue = min;
        d.maxValue = max;
        d.defaultValue = defaultValue;
        d.unit = unit;

        return d;
    }
}

control::PidController::Parameters
    powertrain::TransmissionControlUnit::defaultSlipController()
{
    control::PidController::Parameters params;
    params.kp = 0.004;
    params.ki = 0.9;
    params.kd = 0.0;
    params.outputMin = 0.0;
    params.outputMax = 1.0;
    params.trackingGain = 1.0;

    return params;
}

powertrain::TransmissionControlUnit::TransmissionControlUnit() {
    m_shiftState = ShiftState::Idle;
    m_currentGear = -1;
    m_targetGear = -1;
    m_previousGear = -1;
    m_clutchPressure = 0.0;
    m_secondaryPressure = 0.0;
    m_engagePhase = 0.0;
    m_completedShifts = 0;
    m_previousShiftUp = false;
    m_previousShiftDown = false;
}

powertrain::TransmissionControlUnit::~TransmissionControlUnit() {
    /* void */
}

void powertrain::TransmissionControlUnit::buildDefaultMaps() {
    const int gears = std::max(m_params.gearCount, 1);

    m_upshiftMap.initialize(PedalPoints, gears, 0.0);
    m_downshiftMap.initialize(PedalPoints, gears, 0.0);

    for (int i = 0; i < PedalPoints; ++i) {
        const double pedal = static_cast<double>(i) / (PedalPoints - 1);
        m_upshiftMap.setXAxis(i, pedal);
        m_downshiftMap.setXAxis(i, pedal);
    }

    for (int g = 0; g < gears; ++g) {
        m_upshiftMap.setYAxis(g, static_cast<double>(g));
        m_downshiftMap.setYAxis(g, static_cast<double>(g));
    }

    for (int g = 0; g < gears; ++g) {
        for (int i = 0; i < PedalPoints; ++i) {
            const double pedal = m_upshiftMap.getXAxis(i);

            const double upshiftSpeed =
                engineSpeedForGear(g, 1.0) > 0.0
                ? (units::rpm(2200.0) + pedal * units::rpm(3800.0)) / engineSpeedForGear(g, 1.0)
                : 0.0;

            m_upshiftMap.setValue(i, g, upshiftSpeed);

            const double downshiftSpeed =
                (g > 0 && engineSpeedForGear(g - 1, 1.0) > 0.0)
                ? (units::rpm(1300.0) + pedal * units::rpm(3600.0)) / engineSpeedForGear(g - 1, 1.0)
                : 0.0;

            m_downshiftMap.setValue(i, g, downshiftSpeed);
        }
    }
}

void powertrain::TransmissionControlUnit::initialize(const Parameters &params) {
    m_params = params;
    m_params.gearCount = std::clamp(m_params.gearCount, 1, MaxGears);

    buildDefaultMaps();
    m_slipController.initialize(m_params.slipController);
    m_engageProfile.initialize(m_params.engageProfile);

    reset();
}

void powertrain::TransmissionControlUnit::reset() {
    PowertrainController::reset();

    m_shiftState = ShiftState::Idle;
    m_shiftTimer.reset();
    m_gearTimer.reset();
    m_slipController.reset();
    m_engageProfile.reset();

    m_currentGear = -1;
    m_targetGear = -1;
    m_previousGear = -1;
    m_clutchPressure = 0.0;
    m_secondaryPressure = 0.0;
    m_engagePhase = 0.0;
    m_completedShifts = 0;
    m_previousShiftUp = false;
    m_previousShiftDown = false;
}

double powertrain::TransmissionControlUnit::engineSpeedForGear(
    int gear,
    double vehicleSpeed) const
{
    if (gear < 0 || gear >= m_params.gearCount) return 0.0;
    if (m_params.tireRadius <= 0.0) return 0.0;

    const double wheelSpeed = vehicleSpeed / m_params.tireRadius;

    return wheelSpeed * m_params.finalDrive * m_params.gearRatios[gear];
}

int powertrain::TransmissionControlUnit::scheduleGear(
    int currentGear,
    double pedal,
    double vehicleSpeed) const
{
    if (currentGear < 0) return currentGear;

    const double clampedPedal = std::clamp(pedal, 0.0, 1.0);
    const double speed = std::abs(vehicleSpeed);

    if (clampedPedal >= m_params.kickdownThreshold) {
        for (int g = 0; g < m_params.gearCount; ++g) {
            if (engineSpeedForGear(g, speed) <= units::rpm(6200.0)) {
                if (g < currentGear) return g;
                break;
            }
        }
    }

    if (currentGear + 1 < m_params.gearCount) {
        const double threshold =
            m_upshiftMap.sample(clampedPedal, static_cast<double>(currentGear));
        if (threshold > 0.0 && speed > threshold) return currentGear + 1;
    }

    if (currentGear > 0) {
        const double threshold =
            m_downshiftMap.sample(clampedPedal, static_cast<double>(currentGear));
        if (threshold > 0.0 && speed < threshold) return currentGear - 1;
    }

    return currentGear;
}

void powertrain::TransmissionControlUnit::beginShift(int gear) {
    m_previousGear = m_currentGear;
    m_targetGear = gear;
    m_shiftTimer.reset();

    if (m_params.supportsPreselect && m_currentGear >= 0) {
        m_shiftState = ShiftState::ClutchOverlap;
    }
    else if (m_params.requiresTorqueInterrupt) {
        m_shiftState = ShiftState::TorqueReduction;
    }
    else {
        m_shiftState = ShiftState::GearChange;
    }
}

double powertrain::TransmissionControlUnit::launchPressure(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs)
{
    if (m_params.hasLaunchDevice) return 1.0;

    const double slip = std::abs(state.clutchSlipSpeed[0]);
    if (slip < m_params.launchLockSlip
        && std::abs(state.vehicleSpeed) > m_params.launchSpeed)
    {
        m_slipController.setIntegrator(1.0);
        return 1.0;
    }

    const double target =
        m_params.launchSlipTarget * std::clamp(inputs.accelerator, 0.05, 1.0);

    return m_slipController.update(dt, target, slip);
}

void powertrain::TransmissionControlUnit::advanceShift(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs,
    ActuatorCommands *commands)
{
    m_shiftTimer.advance(dt);

    switch (m_shiftState) {
    case ShiftState::TorqueReduction: {
        const double t = (m_params.torqueReductionTime > 0.0)
            ? std::clamp(m_shiftTimer.getElapsed() / m_params.torqueReductionTime, 0.0, 1.0)
            : 1.0;

        m_bus.torqueReductionRequest = m_params.shiftTorqueReduction * t;
        m_bus.interventionType = TorqueIntervention::Spark;

        if (t >= 1.0) {
            m_shiftState = ShiftState::ClutchRelease;
            m_shiftTimer.reset();
        }
        break;
    }

    case ShiftState::ClutchRelease: {
        const double t = (m_params.clutchReleaseTime > 0.0)
            ? std::clamp(m_shiftTimer.getElapsed() / m_params.clutchReleaseTime, 0.0, 1.0)
            : 1.0;

        m_clutchPressure = m_clutchPressure * (1.0 - t);
        m_bus.torqueReductionRequest = m_params.shiftTorqueReduction;

        if (t >= 1.0) {
            m_clutchPressure = 0.0;
            m_shiftState = ShiftState::GearChange;
            m_shiftTimer.reset();
        }
        break;
    }

    case ShiftState::GearChange: {
        m_clutchPressure = 0.0;
        m_currentGear = m_targetGear;
        m_bus.torqueReductionRequest = m_params.shiftTorqueReduction;

        if (m_shiftTimer.hasElapsed(m_params.gearChangeTime)) {
            const bool downshift = m_targetGear > m_previousGear ? false : true;
            m_shiftState = (downshift && !m_params.hasLaunchDevice)
                ? ShiftState::SpeedMatch
                : ShiftState::ClutchEngage;
            m_shiftTimer.reset();
        }
        break;
    }

    case ShiftState::SpeedMatch: {
        const double target =
            engineSpeedForGear(m_currentGear, std::abs(state.vehicleSpeed));

        m_bus.speedRequest = target;
        m_bus.speedRequestActive = target > 0.0;
        m_bus.torqueReductionRequest = 0.0;
        m_clutchPressure = 0.0;

        const bool matched =
            std::abs(state.engineSpeed - target) < m_params.speedMatchTolerance;

        if (matched || m_shiftTimer.hasElapsed(m_params.speedMatchTime)) {
            m_bus.speedRequestActive = false;
            m_shiftState = ShiftState::ClutchEngage;
            m_shiftTimer.reset();
        }
        break;
    }

    case ShiftState::ClutchOverlap: {
        const double t = (m_params.clutchOverlapTime > 0.0)
            ? std::clamp(m_shiftTimer.getElapsed() / m_params.clutchOverlapTime, 0.0, 1.0)
            : 1.0;

        m_secondaryPressure = t;
        m_clutchPressure = 1.0 - t;
        m_bus.torqueReductionRequest = m_params.shiftTorqueReduction * (1.0 - std::abs(2.0 * t - 1.0));

        if (t >= 1.0) {
            m_currentGear = m_targetGear;
            m_clutchPressure = 1.0;
            m_secondaryPressure = 0.0;
            m_bus.torqueReductionRequest = 0.0;
            m_shiftState = ShiftState::Idle;
            m_gearTimer.reset();
            ++m_completedShifts;
        }
        break;
    }

    case ShiftState::ClutchEngage: {
        const double t = (m_params.clutchEngageTime > 0.0)
            ? std::clamp(m_shiftTimer.getElapsed() / m_params.clutchEngageTime, 0.0, 1.0)
            : 1.0;

        m_engagePhase = t;
        m_clutchPressure = std::clamp(t + m_engageProfile.correction(t), 0.0, 1.0);
        m_bus.torqueReductionRequest = m_params.shiftTorqueReduction * (1.0 - t);

        if (t >= 1.0) {
            m_clutchPressure = 1.0;
            m_bus.torqueReductionRequest = 0.0;
            m_shiftState = ShiftState::Idle;
            m_gearTimer.reset();
            ++m_completedShifts;
        }
        break;
    }

    default:
        break;
    }
}

void powertrain::TransmissionControlUnit::update(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs,
    ActuatorCommands *commands)
{
    m_bus.resetTransmissionRequests();
    m_gearTimer.advance(dt);

    if (m_shiftState == ShiftState::Idle) {
        m_currentGear = state.gear;

        int requested = m_currentGear;

        if (inputs.manualMode) {
            if (inputs.shiftUpRequest && !m_previousShiftUp) {
                if (requested + 1 < m_params.gearCount) ++requested;
            }
            else if (inputs.shiftDownRequest && !m_previousShiftDown) {
                if (requested - 1 >= -1) --requested;
            }
        }
        else {
            if (m_currentGear < 0 && inputs.accelerator > 0.0) requested = 0;
            else requested = scheduleGear(m_currentGear, inputs.accelerator, state.vehicleSpeed);

            if (requested > 0
                && engineSpeedForGear(requested, std::abs(state.vehicleSpeed))
                    < m_params.stallProtectSpeed)
            {
                requested = m_currentGear;
            }
        }

        if (requested != m_currentGear && m_gearTimer.hasElapsed(m_params.minGearTime)) {
            beginShift(requested);
        }
    }

    if (m_shiftState != ShiftState::Idle) {
        advanceShift(dt, state, inputs, commands);
        m_bus.shiftInProgress = true;
    }
    else {
        m_targetGear = m_currentGear;

        if (m_currentGear < 0) {
            m_clutchPressure = 0.0;
            m_slipController.reset();
        }
        else {
            m_clutchPressure = launchPressure(dt, state, inputs);
        }
    }

    m_previousShiftUp = inputs.shiftUpRequest;
    m_previousShiftDown = inputs.shiftDownRequest;

    const double driverLimit = m_params.driverClutchAuthority
        ? std::clamp(1.0 - inputs.clutchPedal, 0.0, 1.0)
        : 1.0;

    commands->targetGear = m_currentGear;
    commands->preselectGear = m_params.supportsPreselect ? m_targetGear : -1;
    commands->clutchPressure[0] =
        std::min(std::clamp(m_clutchPressure, 0.0, 1.0), driverLimit);
    commands->clutchPressure[1] =
        std::min(std::clamp(m_secondaryPressure, 0.0, 1.0), driverLimit);
    commands->lockupPressure = m_params.hasLaunchDevice
        ? ((m_shiftState == ShiftState::Idle && std::abs(state.vehicleSpeed) > m_params.launchSpeed) ? 1.0 : 0.0)
        : 0.0;
}

void powertrain::TransmissionControlUnit::registerParameters(
    config::ParameterRegistry *registry,
    const char *prefix)
{
    if (registry == nullptr) return;

    const std::string base = std::string(prefix) + "tcu.";

    registry->registerScalar(
        describe(base + "shift.torque_reduction", 0.0, 1.0,
            m_params.shiftTorqueReduction, ""),
        &m_params.shiftTorqueReduction);
    registry->registerScalar(
        describe(base + "shift.torque_reduction_time", 0.0, 1.0,
            m_params.torqueReductionTime, "s"),
        &m_params.torqueReductionTime);
    registry->registerScalar(
        describe(base + "shift.clutch_release_time", 0.0, 1.0,
            m_params.clutchReleaseTime, "s"),
        &m_params.clutchReleaseTime);
    registry->registerScalar(
        describe(base + "shift.gear_change_time", 0.0, 1.0,
            m_params.gearChangeTime, "s"),
        &m_params.gearChangeTime);
    registry->registerScalar(
        describe(base + "shift.speed_match_time", 0.0, 2.0,
            m_params.speedMatchTime, "s"),
        &m_params.speedMatchTime);
    registry->registerScalar(
        describe(base + "shift.clutch_engage_time", 0.0, 2.0,
            m_params.clutchEngageTime, "s"),
        &m_params.clutchEngageTime);
    registry->registerScalar(
        describe(base + "shift.clutch_overlap_time", 0.0, 2.0,
            m_params.clutchOverlapTime, "s"),
        &m_params.clutchOverlapTime);
    registry->registerScalar(
        describe(base + "shift.min_gear_time", 0.0, 5.0,
            m_params.minGearTime, "s"),
        &m_params.minGearTime);
    registry->registerScalar(
        describe(base + "shift.kickdown_threshold", 0.0, 1.0,
            m_params.kickdownThreshold, ""),
        &m_params.kickdownThreshold);

    registry->registerScalar(
        describe(base + "launch.slip_target", 0.0, units::rpm(4000.0),
            m_params.launchSlipTarget, "rad/s"),
        &m_params.launchSlipTarget);
    registry->registerScalar(
        describe(base + "launch.lock_slip", 0.0, units::rpm(1000.0),
            m_params.launchLockSlip, "rad/s"),
        &m_params.launchLockSlip);
    registry->registerScalar(
        describe(base + "launch.pid.kp", 0.0, 1.0,
            m_params.slipController.kp, ""),
        &m_slipController.getParametersMutable().kp);
    registry->registerScalar(
        describe(base + "launch.pid.ki", 0.0, 20.0,
            m_params.slipController.ki, ""),
        &m_slipController.getParametersMutable().ki);
    registry->registerScalar(
        describe(base + "launch.pid.kd", 0.0, 1.0,
            m_params.slipController.kd, ""),
        &m_slipController.getParametersMutable().kd);
    registry->registerScalar(
        describe(base + "launch.stall_protect_speed", units::rpm(200.0), units::rpm(3000.0),
            m_params.stallProtectSpeed, "rad/s"),
        &m_params.stallProtectSpeed);
    registry->registerScalar(
        describe(base + "gearbox.final_drive", 0.5, 12.0,
            m_params.finalDrive, ""),
        &m_params.finalDrive);
    registry->registerScalar(
        describe(base + "gearbox.tire_radius", 0.05, 1.5,
            m_params.tireRadius, "m"),
        &m_params.tireRadius);
    registry->registerBoolean(
        describe(base + "gearbox.driver_clutch_authority", 0.0, 1.0,
            m_params.driverClutchAuthority ? 1.0 : 0.0, ""),
        &m_params.driverClutchAuthority);

    config::ParameterDescriptor upshift =
        describe(base + "upshift_map", 0.0, 200.0, 0.0, "m/s");
    upshift.adaptive = true;
    upshift.adaptMin = 0.0;
    upshift.adaptMax = 200.0;
    registry->registerMap(upshift, &m_upshiftMap);

    registry->registerMap(
        describe(base + "downshift_map", 0.0, 200.0, 0.0, "m/s"),
        &m_downshiftMap);
}

void powertrain::TransmissionControlUnit::configureGearbox(
    const GearboxCapabilities &capabilities)
{
    m_params.supportsPreselect = capabilities.supportsPreselect;
    m_params.requiresTorqueInterrupt = capabilities.requiresTorqueInterrupt;
    m_params.hasLaunchDevice = capabilities.hasLaunchDevice;

    if (capabilities.gearRatios != nullptr && capabilities.gearCount > 0) {
        m_params.gearCount = std::min(capabilities.gearCount, MaxGears);
        for (int i = 0; i < m_params.gearCount; ++i) {
            m_params.gearRatios[i] = capabilities.gearRatios[i];
        }
    }

    if (capabilities.finalDrive > 0.0) m_params.finalDrive = capabilities.finalDrive;
    if (capabilities.tireRadius > 0.0) m_params.tireRadius = capabilities.tireRadius;
}

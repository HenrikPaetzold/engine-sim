#include "../../include/powertrain/transmission_control_unit.h"

#include "../../include/config/parameter_registry.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace {
    constexpr int PedalPoints = 5;
    constexpr int PhasePoints = 9;

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
    m_gateIndex = 0;
    m_positionRefused = false;
    m_upshiftAuthored = false;
    m_downshiftAuthored = false;
    m_lockupAuthored = false;
}

powertrain::TransmissionControlUnit::~TransmissionControlUnit() {
    /* void */
}

control::PidController::Parameters
    powertrain::TransmissionControlUnit::defaultLockupController()
{
    control::PidController::Parameters params;
    params.kp = 0.004;
    params.ki = 0.9;
    params.outputMin = 0.0;
    params.outputMax = 1.0;

    return params;
}

void powertrain::TransmissionControlUnit::snapshotGearAxis(
    const control::Map2d &map,
    int gears,
    std::vector<double> *values)
{
    values->clear();
    if (gears < 1 || !map.isInitialized()) return;

    const int points = map.getXCount();
    values->resize(static_cast<size_t>(points) * gears);

    for (int g = 0; g < gears; ++g) {
        for (int i = 0; i < points; ++i) {
            (*values)[static_cast<size_t>(g) * points + i] =
                map.sample(map.getXAxis(i), static_cast<double>(g));
        }
    }
}

void powertrain::TransmissionControlUnit::restoreGearAxis(
    control::Map2d *map,
    const std::vector<double> &values)
{
    if (map == nullptr || !map->isInitialized()) return;

    const int points = map->getXCount();
    const int gears = map->getYCount();
    if (values.size() != static_cast<size_t>(points) * gears) return;

    for (int g = 0; g < gears; ++g) {
        for (int i = 0; i < points; ++i) {
            map->setValue(i, g, values[static_cast<size_t>(g) * points + i]);
        }
    }
}

void powertrain::TransmissionControlUnit::buildDefaultMaps() {
    const int gears = std::max(m_params.gearCount, 1);

    m_upshiftMap.initialize(PedalPoints, gears, 0.0);
    m_downshiftMap.initialize(PedalPoints, gears, 0.0);
    m_lockupMap.initialize(PedalPoints, gears, 0.0);

    for (int i = 0; i < PedalPoints; ++i) {
        const double pedal = static_cast<double>(i) / (PedalPoints - 1);
        m_upshiftMap.setXAxis(i, pedal);
        m_downshiftMap.setXAxis(i, pedal);
        m_lockupMap.setXAxis(i, pedal);
    }

    for (int g = 0; g < gears; ++g) {
        m_upshiftMap.setYAxis(g, static_cast<double>(g));
        m_downshiftMap.setYAxis(g, static_cast<double>(g));
        m_lockupMap.setYAxis(g, static_cast<double>(g));
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

            const double lockupSpeed = (upshiftSpeed > 0.0)
                ? upshiftSpeed * (0.45 + 0.25 * pedal)
                : 0.0;

            m_lockupMap.setValue(i, g, lockupSpeed);
        }
    }

    buildDefaultShapes();
}

void powertrain::TransmissionControlUnit::buildDefaultShapes() {
    control::Map2d *shapes[2] = { &m_overlapShape, &m_engageShape };

    for (control::Map2d *shape : shapes) {
        if (shape->isInitialized()) continue;

        shape->initialize(PhasePoints, PedalPoints, 0.0);

        for (int i = 0; i < PhasePoints; ++i) {
            shape->setXAxis(i, static_cast<double>(i) / (PhasePoints - 1));
        }

        for (int j = 0; j < PedalPoints; ++j) {
            shape->setYAxis(j, static_cast<double>(j) / (PedalPoints - 1));
        }

        for (int j = 0; j < PedalPoints; ++j) {
            for (int i = 0; i < PhasePoints; ++i) {
                shape->setValue(i, j, shape->getXAxis(i));
            }
        }
    }
}

void powertrain::TransmissionControlUnit::initialize(const Parameters &params) {
    m_params = params;
    m_params.gearCount = std::clamp(m_params.gearCount, 1, MaxGears);

    buildDefaultMaps();
    m_slipController.initialize(m_params.slipController);
    m_lockupController.initialize(m_params.lockupController);
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
    m_lockupController.reset();
    m_lockupLimiter.initialize(m_params.lockupApplyRate, 0.0);
    m_lockupLimiter.reset(0.0);
    m_lockupPressure = 0.0;

    m_currentGear = -1;
    m_targetGear = -1;
    m_previousGear = -1;
    m_clutchPressure = 0.0;
    m_secondaryPressure = 0.0;
    m_engagePhase = 0.0;
    m_completedShifts = 0;
    m_previousShiftUp = false;
    m_previousShiftDown = false;

    if (m_gate.isEmpty()) m_gate.buildDefault();

    const int requested = m_gate.find(m_params.defaultPosition);
    if (requested >= 0) {
        m_gateIndex = requested;
    }
    else {
        m_gateIndex = 0;
        for (int i = 0; i < m_gate.getCount(); ++i) {
            if (m_gate.get(i).engagement == GateEngagement::Forward) {
                m_gateIndex = i;
                break;
            }
        }
    }

    m_positionRefused = false;
    m_requestedMode.clear();

    m_activeClutch = 0;
    m_clutchGear[0] = -1;
    m_clutchGear[1] = -1;
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

const powertrain::GatePosition &powertrain::TransmissionControlUnit::getPosition() const {
    return m_gate.get(m_gateIndex);
}

bool powertrain::TransmissionControlUnit::positionAllowed(
    int from,
    int to,
    const PowertrainState &state,
    const DriverInputs &inputs) const
{
    if (from == to) return true;
    if (to < 0 || to >= m_gate.getCount()) return false;
    if (std::abs(from - to) != 1) return false;

    const GatePosition &leaving = m_gate.get(from);
    const GatePosition &entering = m_gate.get(to);

    if (!m_params.supportsEngagement
        && (entering.engagement == GateEngagement::Park
            || entering.engagement == GateEngagement::Reverse))
    {
        return false;
    }

    const double speed = std::abs(state.vehicleSpeed);

    if (entering.maxEntrySpeed >= 0.0 && speed > entering.maxEntrySpeed) return false;
    if (leaving.maxExitSpeed >= 0.0 && speed > leaving.maxExitSpeed) return false;

    if (leaving.requiresBrake && inputs.brake < 0.1) return false;

    return true;
}

void powertrain::TransmissionControlUnit::resolvePosition(
    const PowertrainState &state,
    const DriverInputs &inputs)
{
    m_positionRefused = false;

    if (m_gate.isEmpty()) m_gate.buildDefault();

    const int requested = (inputs.gatePosition < 0)
        ? m_gateIndex
        : m_gate.clampIndex(inputs.gatePosition);

    if (requested == m_gateIndex) return;

    const int step = (requested > m_gateIndex) ? 1 : -1;
    const int next = m_gateIndex + step;

    if (positionAllowed(m_gateIndex, next, state, inputs)) {
        m_gateIndex = next;
        m_requestedMode = m_gate.get(m_gateIndex).mode;

        if (m_shiftState != ShiftState::Idle) {
            m_shiftState = ShiftState::Idle;
            m_shiftTimer.reset();
        }

        m_gearTimer.reset();
    }
    else {
        m_positionRefused = true;
    }
}

void powertrain::TransmissionControlUnit::markAuthoredMaps(
    bool upshift,
    bool downshift,
    bool lockup)
{
    m_upshiftAuthored = upshift;
    m_downshiftAuthored = downshift;
    m_lockupAuthored = lockup;
}

int powertrain::TransmissionControlUnit::clutchForGear(int gear) const {
    if (gear < 0) return 0;
    return gear % MaxClutches;
}

int powertrain::TransmissionControlUnit::getClutchGear(int clutch) const {
    if (clutch < 0 || clutch >= MaxClutches) return -1;
    return m_clutchGear[clutch];
}

int powertrain::TransmissionControlUnit::preselectedNeighbour(
    const PowertrainState &state,
    const DriverInputs &inputs) const
{
    if (m_currentGear < 0) return -1;

    const double pedal = std::clamp(inputs.accelerator, 0.0, 1.0);
    const double speed = std::abs(state.vehicleSpeed);
    const double gear = static_cast<double>(m_currentGear);

    const bool canUp = (m_currentGear + 1) < m_params.gearCount;
    const bool canDown = m_currentGear > 0;

    if (!canUp && !canDown) return -1;
    if (!canDown) return m_currentGear + 1;
    if (!canUp) return m_currentGear - 1;

    const double upThreshold = m_upshiftMap.sample(pedal, gear);
    const double downThreshold = m_downshiftMap.sample(pedal, gear);

    const double toUp = (upThreshold > 0.0)
        ? std::abs(upThreshold - speed)
        : 1e9;
    const double toDown = (downThreshold > 0.0)
        ? std::abs(speed - downThreshold)
        : 1e9;

    return (toUp <= toDown) ? (m_currentGear + 1) : (m_currentGear - 1);
}

void powertrain::TransmissionControlUnit::updateClutchAssignment(
    const PowertrainState &state,
    const DriverInputs &inputs)
{
    if (!m_params.supportsPreselect) {
        m_activeClutch = 0;
        m_clutchGear[0] = m_currentGear;
        m_clutchGear[1] = -1;
        return;
    }

    m_activeClutch = clutchForGear(m_currentGear);
    m_clutchGear[m_activeClutch] = m_currentGear;

    const int idle = (m_activeClutch + 1) % MaxClutches;
    if (m_shiftState == ShiftState::Idle) {
        m_clutchGear[idle] = preselectedNeighbour(state, inputs);
    }
}

void powertrain::TransmissionControlUnit::beginShift(int gear) {
    m_previousGear = m_currentGear;
    m_targetGear = gear;
    m_shiftTimer.reset();

    const bool sameShaft = m_params.supportsPreselect
        && m_currentGear >= 0
        && clutchForGear(gear) == clutchForGear(m_currentGear);

    if (m_params.supportsPreselect && m_currentGear >= 0 && !sameShaft) {
        m_clutchGear[clutchForGear(gear)] = gear;
        m_shiftState = ShiftState::ClutchOverlap;
    }
    else if (sameShaft || m_params.requiresTorqueInterrupt) {
        m_shiftState = ShiftState::TorqueReduction;
    }
    else {
        m_shiftState = ShiftState::GearChange;
    }
}

double powertrain::TransmissionControlUnit::lockupPressure(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs)
{
    if (!m_params.hasLaunchDevice) return 0.0;

    const bool shifting = m_shiftState != ShiftState::Idle;
    const bool kickdown = inputs.accelerator >= m_params.kickdownThreshold;

    m_lockupLimiter.setRates(m_params.lockupApplyRate, 0.0);

    if (shifting || kickdown || m_currentGear < 0) {
        m_lockupController.reset();
        m_lockupLimiter.reset(0.0);
        m_lockupPressure = 0.0;

        return 0.0;
    }

    const double pedal = std::clamp(inputs.accelerator, 0.0, 1.0);
    const double threshold =
        m_lockupMap.sample(pedal, static_cast<double>(m_currentGear));
    const double speed = std::abs(state.vehicleSpeed);

    if (threshold <= 0.0 || speed < threshold) {
        m_lockupController.reset();
        m_lockupLimiter.reset(0.0);
        m_lockupPressure = 0.0;

        return 0.0;
    }

    const double slip = std::abs(state.converterSlip);

    const double demand = (slip < m_params.lockupLockSlip)
        ? 1.0
        : m_lockupController.update(dt, -m_params.lockupSlipTarget, -slip);

    if (slip < m_params.lockupLockSlip) m_lockupController.setIntegrator(1.0);

    m_lockupPressure = m_lockupLimiter.update(dt, demand);

    return m_lockupPressure;
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

    return m_slipController.update(dt, -target, -slip);
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
        m_bus.torqueReductionRequest = m_params.shiftTorqueCut;

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
        m_bus.torqueReductionRequest = m_params.shiftTorqueCut;

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

        m_engagePhase = t;

        const double pedal = std::clamp(inputs.accelerator, 0.0, 1.0);
        const double shaped =
            std::clamp(m_overlapShape.sample(t, pedal), 0.0, 1.0);

        const double oncoming =
            std::clamp(shaped + m_engageProfile.correction(t), 0.0, 1.0);
        const double offgoing =
            std::clamp((1.0 - shaped)
                + m_params.overlapHold * (1.0 - std::abs(2.0 * t - 1.0)),
                0.0, 1.0);

        const int target = clutchForGear(m_targetGear);
        const int source = clutchForGear(m_currentGear);

        m_clutchPressure = (source == 0) ? offgoing : oncoming;
        m_secondaryPressure = (target == 1) ? oncoming : offgoing;

        m_bus.torqueReductionRequest =
            m_params.shiftTorqueReduction * (1.0 - std::abs(2.0 * t - 1.0));

        if (t >= 1.0) {
            m_currentGear = m_targetGear;
            m_activeClutch = target;
            m_clutchGear[target] = m_targetGear;

            m_clutchPressure = (target == 0) ? 1.0 : 0.0;
            m_secondaryPressure = (target == 1) ? 1.0 : 0.0;

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

        const double pedal = std::clamp(inputs.accelerator, 0.0, 1.0);
        const double shaped =
            std::clamp(m_engageShape.sample(t, pedal), 0.0, 1.0);

        m_clutchPressure = std::clamp(shaped + m_engageProfile.correction(t), 0.0, 1.0);
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

    resolvePosition(state, inputs);

    const GateEngagement engagement = getEngagement();
    const bool driving = (engagement == GateEngagement::Forward);
    const bool reversing = (engagement == GateEngagement::Reverse);

    if (!driving && !reversing) {
        m_currentGear = -1;
        m_targetGear = -1;
        m_shiftState = ShiftState::Idle;
        m_clutchPressure = 0.0;
        m_secondaryPressure = 0.0;
        m_slipController.reset();
    }

    if (driving && m_shiftState == ShiftState::Idle) {
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
    else if (reversing) {
        m_currentGear = -1;
        m_targetGear = -1;
        m_clutchPressure = launchPressure(dt, state, inputs);
    }
    else if (driving) {
        m_targetGear = m_currentGear;

        if (m_currentGear < 0) {
            m_clutchPressure = 0.0;
            m_secondaryPressure = 0.0;
            m_slipController.reset();
        }
        else {
            const double pressure = launchPressure(dt, state, inputs);

            if (m_params.supportsPreselect) {
                m_clutchPressure = (m_activeClutch == 0) ? pressure : 0.0;
                m_secondaryPressure = (m_activeClutch == 1) ? pressure : 0.0;
            }
            else {
                m_clutchPressure = pressure;
            }
        }
    }

    m_previousShiftUp = inputs.shiftUpRequest;
    m_previousShiftDown = inputs.shiftDownRequest;

    const double driverLimit = m_params.driverClutchAuthority
        ? std::clamp(1.0 - inputs.clutchPedal, 0.0, 1.0)
        : 1.0;

    updateClutchAssignment(state, inputs);

    commands->engagement = engagement;
    commands->gatePosition = m_gateIndex;
    commands->clutchGear[0] = m_clutchGear[0];
    commands->clutchGear[1] = m_clutchGear[1];
    commands->parkLock = (engagement == GateEngagement::Park);
    commands->targetGear = m_currentGear;
    commands->preselectGear = m_params.supportsPreselect ? m_targetGear : -1;
    commands->clutchPressure[0] =
        std::min(std::clamp(m_clutchPressure, 0.0, 1.0), driverLimit);
    commands->clutchPressure[1] =
        std::min(std::clamp(m_secondaryPressure, 0.0, 1.0), driverLimit);
    commands->lockupPressure = lockupPressure(dt, state, inputs);
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
        describe(base + "shift.torque_cut", 0.0, 1.0, m_params.shiftTorqueCut, ""),
        &m_params.shiftTorqueCut);
    registry->registerScalar(
        describe(base + "shift.overlap_hold", 0.0, 1.0, m_params.overlapHold, ""),
        &m_params.overlapHold);
    registry->registerScalar(
        describe(base + "shift.speed_match_tolerance", units::rpm(10.0), units::rpm(1000.0),
            m_params.speedMatchTolerance, "rad/s"),
        &m_params.speedMatchTolerance);

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
    registry->registerBoolean(
        describe(base + "gate.brake_interlock", 0.0, 1.0,
            m_params.brakeInterlock ? 1.0 : 0.0, ""),
        &m_params.brakeInterlock);
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

    registry->registerScalar(
        describe(base + "lockup.slip_target", 0.0, units::rpm(2000.0),
            m_params.lockupSlipTarget, "rad/s"),
        &m_params.lockupSlipTarget);
    registry->registerScalar(
        describe(base + "lockup.lock_slip", 0.0, units::rpm(500.0),
            m_params.lockupLockSlip, "rad/s"),
        &m_params.lockupLockSlip);
    registry->registerScalar(
        describe(base + "lockup.apply_rate", 0.05, 20.0, m_params.lockupApplyRate, "1/s"),
        &m_params.lockupApplyRate);
    registry->registerScalar(
        describe(base + "lockup.pid.kp", 0.0, 1.0, m_params.lockupController.kp, ""),
        &m_lockupController.getParametersMutable().kp);
    registry->registerScalar(
        describe(base + "lockup.pid.ki", 0.0, 20.0, m_params.lockupController.ki, ""),
        &m_lockupController.getParametersMutable().ki);
    registry->registerMap(
        describe(base + "lockup_map", 0.0, 200.0, 0.0, "m/s"),
        &m_lockupMap);
    registry->registerMap(
        describe(base + "overlap_shape", 0.0, 1.0, 0.0, ""),
        &m_overlapShape);
    registry->registerMap(
        describe(base + "engage_shape", 0.0, 1.0, 0.0, ""),
        &m_engageShape);

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
    m_params.supportsEngagement = capabilities.supportsRange;

    if (!m_params.supportsEngagement
        && (getEngagement() == GateEngagement::Park
            || getEngagement() == GateEngagement::Reverse))
    {
        const int neutral = m_gate.find("N");
        m_gateIndex = (neutral >= 0) ? neutral : m_gateIndex;
    }

    if (capabilities.gearRatios != nullptr && capabilities.gearCount > 0) {
        m_params.gearCount = std::min(capabilities.gearCount, MaxGears);
        for (int i = 0; i < m_params.gearCount; ++i) {
            m_params.gearRatios[i] = capabilities.gearRatios[i];
        }

        control::Map2d *maps[3] = { &m_upshiftMap, &m_downshiftMap, &m_lockupMap };
        const bool authored[3] =
            { m_upshiftAuthored, m_downshiftAuthored, m_lockupAuthored };

        std::vector<double> kept[3];
        for (int i = 0; i < 3; ++i) {
            if (authored[i]) snapshotGearAxis(*maps[i], m_params.gearCount, &kept[i]);
        }

        buildDefaultMaps();

        for (int i = 0; i < 3; ++i) {
            if (authored[i]) restoreGearAxis(maps[i], kept[i]);
        }
    }

    if (capabilities.finalDrive > 0.0) m_params.finalDrive = capabilities.finalDrive;
    if (capabilities.tireRadius > 0.0) m_params.tireRadius = capabilities.tireRadius;
}

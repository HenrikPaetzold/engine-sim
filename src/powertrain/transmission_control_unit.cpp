#include "../../include/powertrain/transmission_control_unit.h"

#include <algorithm>
#include <cmath>
#include <vector>

powertrain::TransmissionControlUnit::TransmissionControlUnit() {
    m_shiftState = ShiftState::Idle;
    m_currentGear = -1;
    m_targetGear = -1;
    m_previousGear = -1;
    m_clutchPressure = 0.0;
    m_releasePressure = 0.0;
    m_engageBins = 0;
    m_secondaryPressure = 0.0;
    m_engagePhase = 0.0;
    m_completedShifts = 0;
    m_previousShiftUp = false;
    m_previousShiftDown = false;
    m_gateIndex = 0;
    m_positionRefused = false;
    m_finalGear = -1;
    m_pedalFiltered = 0.0;
    m_pedalRate = 0.0;
    m_revLimit = 0.0;
    m_upshiftAuthored = false;
    m_downshiftAuthored = false;
    m_lockupAuthored = false;
    m_kickdownAuthored = false;
    m_requestedGears = 0;
    m_gateElapsed = 0.0;
    m_intermediateAuthored = false;
    m_finalDriveAuthored = false;
    m_tireRadiusAuthored = false;
}

powertrain::TransmissionControlUnit::~TransmissionControlUnit() {
    /* void */
}

void powertrain::TransmissionControlUnit::initialize(const Parameters &params) {
    m_params = params;
    m_params.gearCount = std::clamp(m_params.gearCount, 1, MaxGears);

    buildDefaultMaps();
    m_slipController.initialize(m_params.slipController);
    m_lockupController.initialize(m_params.lockupController);
    m_params.engageProfile.binCount = m_params.engageBins;
    m_params.engageProfile.outputMax = m_params.engageLimit;
    m_params.engageProfile.outputMin = -m_params.engageLimit;

    m_engageProfile.initialize(m_params.engageProfile);
    m_engageBins = m_params.engageBins;

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
    m_releasePressure = 0.0;
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
    m_shiftBlock = ShiftBlock::None;
    m_lastShiftDuration = 0.0;
    m_requestedMode.clear();

    m_activeClutch = 0;
    m_clutchGear[0] = -1;
    m_clutchGear[1] = -1;

    m_pedalFiltered = 0.0;
    m_pedalRate = 0.0;
    m_finalGear = -1;
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

double powertrain::TransmissionControlUnit::kickdownTarget(double pedal) const {
    const double clampedPedal = std::clamp(pedal, 0.0, 1.0);

    double target = m_kickdownMap.isInitialized()
        ? m_kickdownMap.sample(clampedPedal, 0.0)
        : m_params.kickdownTargetSpeed;

    if (target <= 0.0) target = m_params.kickdownTargetSpeed;

    if (m_revLimit > 0.0) {
        target = std::min(target, m_revLimit - m_params.kickdownRevMargin);
    }

    return std::max(target, 0.0);
}

int powertrain::TransmissionControlUnit::kickdownGear(
    double pedal,
    double vehicleSpeed) const
{
    const double target = kickdownTarget(pedal);
    const double speed = std::abs(vehicleSpeed);

    for (int g = 0; g < m_params.gearCount; ++g) {
        if (engineSpeedForGear(g, speed) <= target) return g;
    }

    return -1;
}

int powertrain::TransmissionControlUnit::scheduleGear(
    int currentGear,
    double pedal,
    double vehicleSpeed) const
{
    return scheduleGear(
        currentGear,
        pedal,
        vehicleSpeed,
        std::clamp(pedal, 0.0, 1.0) >= m_params.kickdownThreshold);
}

int powertrain::TransmissionControlUnit::scheduleGear(
    int currentGear,
    double pedal,
    double vehicleSpeed,
    bool kickdown) const
{
    if (currentGear < 0) return currentGear;

    const double clampedPedal = std::clamp(pedal, 0.0, 1.0);
    const double speed = std::abs(vehicleSpeed);

    if (kickdown) {
        const int target = kickdownGear(clampedPedal, speed);
        if (target >= 0 && target < currentGear) return target;
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

    if (m_params.brakeInterlock
        && leaving.requiresBrake
        && inputs.brake < 0.1)
    {
        return false;
    }

    return true;
}

void powertrain::TransmissionControlUnit::resolvePosition(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs)
{
    m_positionRefused = false;

    if (m_gate.isEmpty()) m_gate.buildDefault();

    const int requested = (inputs.gatePosition < 0)
        ? m_gateIndex
        : m_gate.clampIndex(inputs.gatePosition);

    if (requested == m_gateIndex) {
        m_gateElapsed = 0.0;
        return;
    }

    m_gateElapsed += dt;
    if (m_gateElapsed < m_params.gateStepTime) return;

    m_gateElapsed = 0.0;

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
        m_shiftBlock = ShiftBlock::GateRefused;
    }
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

int powertrain::TransmissionControlUnit::intermediateGear(
    int from,
    int to,
    double pedal) const
{
    if (from < 0 || to < 0) return -1;

    const int step = (to > from) ? 1 : -1;
    const int wanted = 1 - clutchForGear(from);

    int candidates[MaxGears];
    int count = 0;

    for (int g = from + step; g != to; g += step) {
        if (g < 0 || g >= m_params.gearCount) break;
        if (clutchForGear(g) != wanted) continue;

        candidates[count++] = g;
    }

    if (count == 0) return -1;
    if (count == 1) return candidates[0];

    const double jump = static_cast<double>(std::abs(to - from));
    const double bias = m_intermediateBias.isInitialized()
        ? std::clamp(m_intermediateBias.sample(std::clamp(pedal, 0.0, 1.0), jump), 0.0, 1.0)
        : 1.0;

    const int index = static_cast<int>(std::lround(bias * (count - 1)));

    return candidates[std::clamp(index, 0, count - 1)];
}

void powertrain::TransmissionControlUnit::beginShift(int gear) {
    m_previousGear = m_currentGear;
    m_targetGear = gear;
    m_shiftTimer.reset();

    const bool sameShaft = m_params.supportsPreselect
        && m_currentGear >= 0
        && clutchForGear(gear) == clutchForGear(m_currentGear);

    if (sameShaft && m_params.multiShiftViaIntermediate
        && std::abs(gear - m_currentGear)
            <= static_cast<int>(std::lround(m_params.multiShiftMaxGears)))
    {
        const int bridge = intermediateGear(m_currentGear, gear, m_pedalFiltered);

        if (bridge >= 0) {
            m_finalGear = gear;
            m_targetGear = bridge;
            m_clutchGear[clutchForGear(bridge)] = bridge;
            m_shiftState = ShiftState::ClutchOverlap;

            return;
        }
    }

    m_finalGear = -1;

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

    const double slip = std::abs(state.clutchSlipSpeed[
        std::clamp(m_activeClutch, 0, MaxClutches - 1)]);
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

double powertrain::TransmissionControlUnit::phaseFraction(double duration) const {
    if (duration <= 0.0) return 1.0;
    return std::clamp(m_shiftTimer.getElapsed() / duration, 0.0, 1.0);
}

void powertrain::TransmissionControlUnit::advanceTorqueReduction() {
    const double t = phaseFraction(m_params.torqueReductionTime);

    m_bus.torqueReductionRequest = m_params.shiftTorqueReduction * t;
    m_bus.interventionType = TorqueIntervention::Spark;

    if (t < 1.0) return;

    m_shiftState = ShiftState::ClutchRelease;
    m_releasePressure = m_clutchPressure;
    m_shiftTimer.reset();
}

void powertrain::TransmissionControlUnit::advanceClutchRelease() {
    const double t = phaseFraction(m_params.clutchReleaseTime);

    m_clutchPressure = m_releasePressure * (1.0 - t);
    m_bus.torqueReductionRequest = m_params.shiftTorqueCut;

    if (t < 1.0) return;

    m_clutchPressure = 0.0;
    m_shiftState = ShiftState::GearChange;
    m_shiftTimer.reset();
}

void powertrain::TransmissionControlUnit::advanceGearChange() {
    m_clutchPressure = 0.0;
    m_currentGear = m_targetGear;
    m_bus.torqueReductionRequest = m_params.shiftTorqueCut;

    if (!m_shiftTimer.hasElapsed(m_params.gearChangeTime)) return;

    const bool downshift = m_targetGear > m_previousGear ? false : true;
    m_shiftState = (downshift && !m_params.hasLaunchDevice)
        ? ShiftState::SpeedMatch
        : ShiftState::ClutchEngage;
    m_shiftTimer.reset();
}

void powertrain::TransmissionControlUnit::advanceSpeedMatch(
    const PowertrainState &state)
{
    const double target =
        engineSpeedForGear(m_currentGear, std::abs(state.vehicleSpeed));

    m_bus.speedRequest = target;
    m_bus.speedRequestActive = target > 0.0;
    m_bus.torqueReductionRequest = 0.0;
    m_clutchPressure = 0.0;

    const bool matched =
        std::abs(state.engineSpeed - target) < m_params.speedMatchTolerance;

    if (!matched && !m_shiftTimer.hasElapsed(m_params.speedMatchTime)) return;

    m_bus.speedRequestActive = false;
    m_shiftState = ShiftState::ClutchEngage;
    m_shiftTimer.reset();
}

void powertrain::TransmissionControlUnit::advanceClutchOverlap(
    const DriverInputs &inputs)
{
    const double t = phaseFraction(m_params.clutchOverlapTime);

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

    if (t < 1.0) return;

    m_currentGear = m_targetGear;
    m_activeClutch = target;
    m_clutchGear[target] = m_targetGear;

    m_clutchPressure = (target == 0) ? 1.0 : 0.0;
    m_secondaryPressure = (target == 1) ? 1.0 : 0.0;

    m_bus.torqueReductionRequest = 0.0;
    m_lastShiftDuration = m_shiftTimer.getElapsed();
    m_shiftState = ShiftState::Idle;
    m_gearTimer.reset();
    ++m_completedShifts;

    const int pending = m_finalGear;
    m_finalGear = -1;

    if (pending >= 0 && pending != m_currentGear) beginShift(pending);
}

void powertrain::TransmissionControlUnit::advanceClutchEngage(
    const DriverInputs &inputs)
{
    const double t = phaseFraction(m_params.clutchEngageTime);

    m_engagePhase = t;

    const double pedal = std::clamp(inputs.accelerator, 0.0, 1.0);
    const double shaped =
        std::clamp(m_engageShape.sample(t, pedal), 0.0, 1.0);

    m_clutchPressure = std::clamp(shaped + m_engageProfile.correction(t), 0.0, 1.0);
    m_bus.torqueReductionRequest = m_params.shiftTorqueReduction * (1.0 - t);

    if (t < 1.0) return;

    m_clutchPressure = 1.0;
    m_bus.torqueReductionRequest = 0.0;
    m_lastShiftDuration = m_shiftTimer.getElapsed();
    m_shiftState = ShiftState::Idle;
    m_gearTimer.reset();
    ++m_completedShifts;
}

void powertrain::TransmissionControlUnit::advanceShift(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs)
{
    m_shiftTimer.advance(dt);

    switch (m_shiftState) {
    case ShiftState::TorqueReduction: advanceTorqueReduction(); break;
    case ShiftState::ClutchRelease:   advanceClutchRelease();   break;
    case ShiftState::GearChange:      advanceGearChange();      break;
    case ShiftState::SpeedMatch:      advanceSpeedMatch(state); break;
    case ShiftState::ClutchOverlap:   advanceClutchOverlap(inputs); break;
    case ShiftState::ClutchEngage:    advanceClutchEngage(inputs);  break;
    default: break;
    }
}

void powertrain::TransmissionControlUnit::syncEngageProfile() {
    if (m_params.engageBins != m_engageBins) {
        m_params.engageProfile.binCount = std::max(m_params.engageBins, 1);
        m_engageProfile.initialize(m_params.engageProfile);
        m_engageBins = m_params.engageBins;
    }

    m_engageProfile.getParametersMutable().outputMax = m_params.engageLimit;
    m_engageProfile.getParametersMutable().outputMin = -m_params.engageLimit;
}

void powertrain::TransmissionControlUnit::updatePedalFilter(double dt, double pedal) {
    if (dt <= 0.0) return;

    const double tau = std::max(m_params.kickdownFilter, dt);
    const double alpha = dt / tau;
    const double previous = m_pedalFiltered;

    m_pedalFiltered += (pedal - m_pedalFiltered) * std::min(alpha, 1.0);
    m_pedalRate = (m_pedalFiltered - previous) / dt;
}

int powertrain::TransmissionControlUnit::requestedGear(
    const PowertrainState &state,
    const DriverInputs &inputs,
    double pedal,
    ShiftBlock *block) const
{
    int requested = m_currentGear;
    *block = ShiftBlock::None;

    if (inputs.manualMode) {
        if (inputs.shiftUpRequest && !m_previousShiftUp) {
            if (requested + 1 < m_params.gearCount) ++requested;
            else *block = ShiftBlock::TopGear;
        }
        else if (inputs.shiftDownRequest && !m_previousShiftDown) {
            if (requested - 1 >= -1) --requested;
            else *block = ShiftBlock::BottomGear;
        }

        return requested;
    }

    const bool stab =
        m_params.kickdownPedalRate > 0.0
        && m_pedalRate >= m_params.kickdownPedalRate
        && pedal >= m_params.kickdownPedalFloor;

    const bool kickdown = stab || pedal >= m_params.kickdownThreshold;

    if (m_currentGear < 0 && inputs.accelerator > 0.0) requested = 0;
    else if (m_currentGear < 0) {
        *block = ShiftBlock::Coasting;
    }
    else {
        requested = scheduleGear(
            m_currentGear, inputs.accelerator, state.vehicleSpeed, kickdown);

        if (requested == m_currentGear) {
            *block = kickdown
                ? ShiftBlock::NoKickdownGear
                : ShiftBlock::NoUpshiftThreshold;
        }
        else if (requested < m_currentGear && !kickdown) {
            *block = ShiftBlock::NoDownshiftThreshold;
        }
    }

    if (requested > 0
        && engineSpeedForGear(requested, std::abs(state.vehicleSpeed))
            < m_params.stallProtectSpeed)
    {
        requested = m_currentGear;
        *block = ShiftBlock::StallProtection;
    }

    return requested;
}

void powertrain::TransmissionControlUnit::applyClutchPressures(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs,
    bool driving,
    bool reversing)
{
    if (m_shiftState != ShiftState::Idle) {
        advanceShift(dt, state, inputs);
        m_bus.shiftInProgress = true;
        return;
    }

    if (reversing) {
        m_currentGear = -1;
        m_targetGear = -1;
        m_clutchPressure = launchPressure(dt, state, inputs);
        return;
    }

    if (!driving) return;

    m_targetGear = m_currentGear;

    if (m_currentGear < 0) {
        m_clutchPressure = 0.0;
        m_secondaryPressure = 0.0;
        m_slipController.reset();
        return;
    }

    const double pressure = launchPressure(dt, state, inputs);

    if (m_params.supportsPreselect) {
        m_clutchPressure = (m_activeClutch == 0) ? pressure : 0.0;
        m_secondaryPressure = (m_activeClutch == 1) ? pressure : 0.0;
    }
    else {
        m_clutchPressure = pressure;
    }
}

void powertrain::TransmissionControlUnit::fillCommands(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs,
    GateEngagement engagement,
    double driverLimit,
    ActuatorCommands *commands)
{
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

void powertrain::TransmissionControlUnit::update(
    double dt,
    const PowertrainState &state,
    const DriverInputs &inputs,
    ActuatorCommands *commands)
{
    m_bus.resetTransmissionRequests();
    m_gearTimer.advance(dt);

    syncEngageProfile();

    if (commands != nullptr && commands->revLimit > 0.0) {
        m_revLimit = commands->revLimit;
    }

    const double pedalNow = std::clamp(inputs.accelerator, 0.0, 1.0);
    updatePedalFilter(dt, pedalNow);

    resolvePosition(dt, state, inputs);

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

    if (!driving) {
        m_shiftBlock = reversing ? ShiftBlock::Reversing : ShiftBlock::NotInDrive;
    }
    else if (m_shiftState != ShiftState::Idle) {
        m_shiftBlock = ShiftBlock::ShiftInProgress;
    }
    else {
        m_currentGear = state.gear;

        ShiftBlock reason = ShiftBlock::None;
        const int requested = requestedGear(state, inputs, pedalNow, &reason);

        if (requested == m_currentGear) {
            m_shiftBlock = reason;
        }
        else if (!m_gearTimer.hasElapsed(m_params.minGearTime)) {
            m_shiftBlock = ShiftBlock::GearDwell;
        }
        else {
            m_shiftBlock = ShiftBlock::None;
            beginShift(requested);
        }
    }

    applyClutchPressures(dt, state, inputs, driving, reversing);

    m_previousShiftUp = inputs.shiftUpRequest;
    m_previousShiftDown = inputs.shiftDownRequest;

    const double driverLimit = m_params.driverClutchAuthority
        ? std::clamp(1.0 - inputs.clutchPedal, 0.0, 1.0)
        : 1.0;

    updateClutchAssignment(state, inputs);
    fillCommands(dt, state, inputs, engagement, driverLimit, commands);
}

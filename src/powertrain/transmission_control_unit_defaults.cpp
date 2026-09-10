#include "../../include/powertrain/transmission_control_unit.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr int PedalPoints = 5;
    constexpr int PhasePoints = 9;
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

void powertrain::TransmissionControlUnit::resizeGearAxis(
    control::Map2d *map,
    int gears)
{
    if (map == nullptr || !map->isInitialized() || gears < 1) return;

    const int points = map->getXCount();

    std::vector<double> axis(points);
    for (int i = 0; i < points; ++i) axis[i] = map->getXAxis(i);

    std::vector<double> values(static_cast<size_t>(points) * gears);
    for (int g = 0; g < gears; ++g) {
        for (int i = 0; i < points; ++i) {
            values[static_cast<size_t>(g) * points + i] =
                map->sample(axis[i], static_cast<double>(g));
        }
    }

    map->initialize(points, gears, 0.0);
    for (int i = 0; i < points; ++i) map->setXAxis(i, axis[i]);
    for (int g = 0; g < gears; ++g) map->setYAxis(g, static_cast<double>(g));

    for (int g = 0; g < gears; ++g) {
        for (int i = 0; i < points; ++i) {
            map->setValue(i, g, values[static_cast<size_t>(g) * points + i]);
        }
    }
}

void powertrain::TransmissionControlUnit::buildDefaultMaps() {
    const int gears = std::max(m_params.gearCount, 1);

    if (!m_upshiftAuthored) m_upshiftMap.initialize(PedalPoints, gears, 0.0);
    if (!m_downshiftAuthored) m_downshiftMap.initialize(PedalPoints, gears, 0.0);
    if (!m_lockupAuthored) m_lockupMap.initialize(PedalPoints, gears, 0.0);
    if (!m_kickdownAuthored) {
        m_kickdownMap.initialize(PedalPoints, 1, 0.0);

        for (int i = 0; i < PedalPoints; ++i) {
            const double pedal = static_cast<double>(i) / (PedalPoints - 1);
            m_kickdownMap.setXAxis(i, pedal);
            m_kickdownMap.setValue(i, 0, m_params.kickdownTargetSpeed);
        }

        m_kickdownMap.setYAxis(0, 0.0);
    }

    if (!m_intermediateAuthored) {
        m_intermediateBias.initialize(PedalPoints, MaxGears, 1.0);
        for (int i = 0; i < PedalPoints; ++i) {
            m_intermediateBias.setXAxis(i, static_cast<double>(i) / (PedalPoints - 1));
        }
        for (int j = 0; j < MaxGears; ++j) {
            m_intermediateBias.setYAxis(j, static_cast<double>(j));
        }
    }

    for (int i = 0; i < PedalPoints; ++i) {
        const double pedal = static_cast<double>(i) / (PedalPoints - 1);
        if (!m_upshiftAuthored) m_upshiftMap.setXAxis(i, pedal);
        if (!m_downshiftAuthored) m_downshiftMap.setXAxis(i, pedal);
        if (!m_lockupAuthored) m_lockupMap.setXAxis(i, pedal);
    }

    for (int g = 0; g < gears; ++g) {
        if (!m_upshiftAuthored) m_upshiftMap.setYAxis(g, static_cast<double>(g));
        if (!m_downshiftAuthored) m_downshiftMap.setYAxis(g, static_cast<double>(g));
        if (!m_lockupAuthored) m_lockupMap.setYAxis(g, static_cast<double>(g));
    }

    for (int g = 0; g < gears; ++g) {
        for (int i = 0; i < PedalPoints; ++i) {
            const double pedal = static_cast<double>(i) / (PedalPoints - 1);

            const double upshiftSpeed =
                engineSpeedForGear(g, 1.0) > 0.0
                ? (units::rpm(2200.0) + pedal * units::rpm(3800.0)) / engineSpeedForGear(g, 1.0)
                : 0.0;

            if (!m_upshiftAuthored) m_upshiftMap.setValue(i, g, upshiftSpeed);

            const double downshiftSpeed =
                (g > 0 && engineSpeedForGear(g - 1, 1.0) > 0.0)
                ? (units::rpm(1300.0) + pedal * units::rpm(3600.0)) / engineSpeedForGear(g - 1, 1.0)
                : 0.0;

            if (!m_downshiftAuthored) m_downshiftMap.setValue(i, g, downshiftSpeed);

            const double lockupSpeed = (upshiftSpeed > 0.0)
                ? upshiftSpeed * (0.45 + 0.25 * pedal)
                : 0.0;

            if (!m_lockupAuthored) m_lockupMap.setValue(i, g, lockupSpeed);
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

void powertrain::TransmissionControlUnit::markAuthoredMaps(
    bool upshift,
    bool downshift,
    bool lockup)
{
    m_upshiftAuthored = upshift;
    m_downshiftAuthored = downshift;
    m_lockupAuthored = lockup;
}

void powertrain::TransmissionControlUnit::markAuthoredKickdown(bool authored) {
    m_kickdownAuthored = authored;
}

void powertrain::TransmissionControlUnit::markAuthoredDriveline(
    bool finalDrive,
    bool tireRadius)
{
    m_finalDriveAuthored = finalDrive;
    m_tireRadiusAuthored = tireRadius;
}

void powertrain::TransmissionControlUnit::markAuthoredIntermediateBias(bool authored) {
    m_intermediateAuthored = authored;
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
        m_requestedGears = capabilities.gearCount;
        m_params.gearCount = std::min(capabilities.gearCount, MaxGears);
        for (int i = 0; i < m_params.gearCount; ++i) {
            m_params.gearRatios[i] = capabilities.gearRatios[i];
        }

        buildDefaultMaps();

        control::Map2d *maps[3] = { &m_upshiftMap, &m_downshiftMap, &m_lockupMap };
        const bool authored[3] =
            { m_upshiftAuthored, m_downshiftAuthored, m_lockupAuthored };

        for (int i = 0; i < 3; ++i) {
            if (authored[i]) resizeGearAxis(maps[i], m_params.gearCount);
        }

        if (m_intermediateAuthored) {
            resizeGearAxis(&m_intermediateBias, MaxGears);
        }
    }

    if (!m_finalDriveAuthored && capabilities.finalDrive > 0.0) {
        m_params.finalDrive = capabilities.finalDrive;
    }
    if (!m_tireRadiusAuthored && capabilities.tireRadius > 0.0) {
        m_params.tireRadius = capabilities.tireRadius;
    }
}

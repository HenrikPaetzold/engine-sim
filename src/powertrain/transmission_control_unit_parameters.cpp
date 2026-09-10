#include "../../include/powertrain/transmission_control_unit.h"

#include "../../include/config/parameter_registry.h"

#include <string>


void powertrain::TransmissionControlUnit::registerParameters(config::ParameterRegistry *registry)
{
    if (registry == nullptr) return;

    const std::string base = "tcu.";

    registry->registerScalar(
        config::describeScalar(base + "shift.torque_reduction", 0.0, 1.0,
            m_params.shiftTorqueReduction, ""),
        &m_params.shiftTorqueReduction);
    registry->registerScalar(
        config::describeScalar(base + "shift.torque_reduction_time", 0.0, 1.0,
            m_params.torqueReductionTime, "s"),
        &m_params.torqueReductionTime);
    registry->registerScalar(
        config::describeScalar(base + "shift.clutch_release_time", 0.0, 1.0,
            m_params.clutchReleaseTime, "s"),
        &m_params.clutchReleaseTime);
    registry->registerScalar(
        config::describeScalar(base + "shift.gear_change_time", 0.0, 1.0,
            m_params.gearChangeTime, "s"),
        &m_params.gearChangeTime);
    registry->registerScalar(
        config::describeScalar(base + "shift.speed_match_time", 0.0, 2.0,
            m_params.speedMatchTime, "s"),
        &m_params.speedMatchTime);
    registry->registerScalar(
        config::describeScalar(base + "shift.clutch_engage_time", 0.0, 2.0,
            m_params.clutchEngageTime, "s"),
        &m_params.clutchEngageTime);
    registry->registerScalar(
        config::describeScalar(base + "shift.clutch_overlap_time", 0.0, 2.0,
            m_params.clutchOverlapTime, "s"),
        &m_params.clutchOverlapTime);
    registry->registerScalar(
        config::describeScalar(base + "shift.min_gear_time", 0.0, 5.0,
            m_params.minGearTime, "s"),
        &m_params.minGearTime);
    registry->registerScalar(
        config::describeScalar(base + "shift.kickdown_threshold", 0.0, 1.0,
            m_params.kickdownThreshold, ""),
        &m_params.kickdownThreshold);
    registry->registerScalar(
        config::describeScalar(base + "kickdown.pedal_rate", 0.0, 200.0,
            m_params.kickdownPedalRate, "1/s"),
        &m_params.kickdownPedalRate);
    registry->registerScalar(
        config::describeScalar(base + "kickdown.pedal_floor", 0.0, 1.0,
            m_params.kickdownPedalFloor, ""),
        &m_params.kickdownPedalFloor);
    registry->registerScalar(
        config::describeScalar(base + "kickdown.filter", 0.005, 1.0,
            m_params.kickdownFilter, "s"),
        &m_params.kickdownFilter);
    registry->registerScalar(
        config::describeScalar(base + "kickdown.rev_margin", 0.0, units::rpm(3000.0),
            m_params.kickdownRevMargin, "rad/s"),
        &m_params.kickdownRevMargin);
    registry->registerMap(
        config::describeScalar(base + "kickdown_map", 0.0, units::rpm(20000.0), 0.0, "rad/s"),
        &m_kickdownMap);
    registry->registerScalar(
        config::describeScalar(base + "shift.torque_cut", 0.0, 1.0, m_params.shiftTorqueCut, ""),
        &m_params.shiftTorqueCut);
    registry->registerScalar(
        config::describeScalar(base + "shift.overlap_hold", 0.0, 1.0, m_params.overlapHold, ""),
        &m_params.overlapHold);
    registry->registerBoolean(
        config::describeScalar(base + "shift.multi_via_intermediate", 0.0, 1.0,
            m_params.multiShiftViaIntermediate ? 1.0 : 0.0, ""),
        &m_params.multiShiftViaIntermediate);
    registry->registerScalar(
        config::describeScalar(base + "shift.multi_max_gears", 0.0, static_cast<double>(MaxGears),
            m_params.multiShiftMaxGears, ""),
        &m_params.multiShiftMaxGears);
    registry->registerMap(
        config::describeScalar(base + "intermediate_bias", 0.0, 1.0, 1.0, ""),
        &m_intermediateBias);
    registry->registerScalar(
        config::describeScalar(base + "shift.speed_match_tolerance", units::rpm(10.0), units::rpm(1000.0),
            m_params.speedMatchTolerance, "rad/s"),
        &m_params.speedMatchTolerance);

    registry->registerScalar(
        config::describeScalar(base + "launch.speed", 0.0,
            units::velocity(100.0, units::km / units::hour),
            m_params.launchSpeed, "m/s"),
        &m_params.launchSpeed);
    registry->registerScalar(
        config::describeScalar(base + "launch.slip_target", 0.0, units::rpm(4000.0),
            m_params.launchSlipTarget, "rad/s"),
        &m_params.launchSlipTarget);
    registry->registerScalar(
        config::describeScalar(base + "launch.lock_slip", 0.0, units::rpm(1000.0),
            m_params.launchLockSlip, "rad/s"),
        &m_params.launchLockSlip);
    config::registerPid(registry, base + "launch.pid.", &m_slipController);
    registry->registerScalar(
        config::describeScalar(base + "launch.stall_protect_speed", units::rpm(200.0), units::rpm(3000.0),
            m_params.stallProtectSpeed, "rad/s"),
        &m_params.stallProtectSpeed);
    registry->registerScalar(
        config::describeScalar(base + "gate.step_time", 0.0, 2.0, m_params.gateStepTime, "s"),
        &m_params.gateStepTime);
    registry->registerScalar(
        config::describeScalar(base + "kickdown.target_speed", units::rpm(1000.0),
            units::rpm(12000.0), m_params.kickdownTargetSpeed, "rad/s"),
        &m_params.kickdownTargetSpeed);
    registry->registerScalar(
        config::describeScalar(base + "engage.learning_rate", 0.0, 2.0,
            m_engageProfile.getParametersMutable().learningRate, ""),
        &m_engageProfile.getParametersMutable().learningRate);
    registry->registerScalar(
        config::describeScalar(base + "engage.smoothing", 0.0, 1.0,
            m_engageProfile.getParametersMutable().smoothing, ""),
        &m_engageProfile.getParametersMutable().smoothing);
    registry->registerScalar(
        config::describeScalar(base + "engage.limit", 0.0, 1.0,
            m_params.engageLimit, ""),
        &m_params.engageLimit);
    registry->registerInteger(
        config::describeScalar(base + "engage.bins", 1.0, 64.0, m_params.engageBins, ""),
        &m_params.engageBins);
    registry->registerBoolean(
        config::describeScalar(base + "gate.brake_interlock", 0.0, 1.0,
            m_params.brakeInterlock ? 1.0 : 0.0, ""),
        &m_params.brakeInterlock);
    registry->registerScalar(
        config::describeScalar(base + "gearbox.final_drive", 0.5, 12.0,
            m_params.finalDrive, ""),
        &m_params.finalDrive);
    registry->registerScalar(
        config::describeScalar(base + "gearbox.tire_radius", 0.05, 1.5,
            m_params.tireRadius, "m"),
        &m_params.tireRadius);
    registry->registerBoolean(
        config::describeScalar(base + "gearbox.driver_clutch_authority", 0.0, 1.0,
            m_params.driverClutchAuthority ? 1.0 : 0.0, ""),
        &m_params.driverClutchAuthority);

    registry->registerScalar(
        config::describeScalar(base + "lockup.slip_target", 0.0, units::rpm(2000.0),
            m_params.lockupSlipTarget, "rad/s"),
        &m_params.lockupSlipTarget);
    registry->registerScalar(
        config::describeScalar(base + "lockup.lock_slip", 0.0, units::rpm(500.0),
            m_params.lockupLockSlip, "rad/s"),
        &m_params.lockupLockSlip);
    registry->registerScalar(
        config::describeScalar(base + "lockup.apply_rate", 0.05, 20.0, m_params.lockupApplyRate, "1/s"),
        &m_params.lockupApplyRate);
    config::registerPid(registry, base + "lockup.pid.", &m_lockupController);
    config::ParameterDescriptor lockup =
        config::describeScalar(base + "lockup_map", 0.0, 200.0, 0.0, "m/s");
    lockup.adaptive = true;
    lockup.adaptMin = 0.0;
    lockup.adaptMax = 200.0;
    registry->registerMap(lockup, &m_lockupMap);
    registry->registerMap(
        config::describeScalar(base + "overlap_shape", 0.0, 1.0, 0.0, ""),
        &m_overlapShape);
    registry->registerMap(
        config::describeScalar(base + "engage_shape", 0.0, 1.0, 0.0, ""),
        &m_engageShape);

    config::ParameterDescriptor upshift =
        config::describeScalar(base + "upshift_map", 0.0, 200.0, 0.0, "m/s");
    upshift.adaptive = true;
    upshift.adaptMin = 0.0;
    upshift.adaptMax = 200.0;
    registry->registerMap(upshift, &m_upshiftMap);

    config::ParameterDescriptor downshift =
        config::describeScalar(base + "downshift_map", 0.0, 200.0, 0.0, "m/s");
    downshift.adaptive = true;
    downshift.adaptMin = 0.0;
    downshift.adaptMax = 200.0;
    registry->registerMap(downshift, &m_downshiftMap);
}

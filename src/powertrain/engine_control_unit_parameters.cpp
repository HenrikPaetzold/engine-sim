#include "../../include/powertrain/engine_control_unit.h"

#include "../../include/config/parameter_registry.h"

#include <string>


void powertrain::EngineControlUnit::registerParameters(config::ParameterRegistry *registry)
{
    if (registry == nullptr) return;

    const std::string base = "ecu.";

    registry->registerScalar(
        config::describeScalar(base + "reference_torque", 0.0, units::torque(2000.0, units::Nm),
            m_params.referenceTorque, "Nm"),
        &m_params.referenceTorque);
    registry->registerScalar(
        config::describeScalar(base + "torque_rise_rate", 0.0, units::torque(20000.0, units::Nm),
            m_params.torqueRiseRate, "Nm/s"),
        &m_params.torqueRiseRate);
    registry->registerScalar(
        config::describeScalar(base + "torque_fall_rate", 0.0, units::torque(20000.0, units::Nm),
            m_params.torqueFallRate, "Nm/s"),
        &m_params.torqueFallRate);

    registry->registerScalar(
        config::describeScalar(base + "idle.speed_cold", units::rpm(400.0), units::rpm(3000.0),
            m_params.idleSpeedCold, "rad/s"),
        &m_params.idleSpeedCold);
    registry->registerScalar(
        config::describeScalar(base + "idle.speed_warm", units::rpm(400.0), units::rpm(3000.0),
            m_params.idleSpeedWarm, "rad/s"),
        &m_params.idleSpeedWarm);
    config::registerPid(registry, base + "idle.pid.", &m_idleController);

    config::registerPid(registry, base + "torque.pid.", &m_torqueController);

    registry->registerScalar(
        config::describeScalar(base + "limiter.rev_limit", units::rpm(1000.0), units::rpm(20000.0),
            m_params.revLimit, "rad/s"),
        &m_params.revLimit);
    registry->registerScalar(
        config::describeScalar(base + "limiter.rev_limit_cold", units::rpm(1000.0), units::rpm(20000.0),
            m_params.revLimitCold, "rad/s"),
        &m_params.revLimitCold);
    registry->registerScalar(
        config::describeScalar(base + "limiter.soft_band", 0.0, units::rpm(2000.0),
            m_params.softLimitBand, "rad/s"),
        &m_params.softLimitBand);
    registry->registerScalar(
        config::describeScalar(base + "limiter.hard_offset", 0.0, units::rpm(2000.0),
            m_params.hardLimitOffset, "rad/s"),
        &m_params.hardLimitOffset);
    registry->registerScalar(
        config::describeScalar(base + "limiter.duration", 0.0, 5.0,
            m_params.limiterDuration, "s"),
        &m_params.limiterDuration);
    registry->registerScalar(
        config::describeScalar(base + "cranking_speed", units::rpm(50.0), units::rpm(2000.0),
            m_params.crankingSpeed, "rad/s"),
        &m_params.crankingSpeed);


    registry->registerScalar(
        config::describeScalar(base + "coldstart.enrichment", 1.0, 3.0,
            m_params.coldStartEnrichment, ""),
        &m_params.coldStartEnrichment);
    registry->registerScalar(
        config::describeScalar(base + "coldstart.timing_retard", 0.0, units::angle(30.0, units::deg),
            m_params.coldStartTimingRetard, "rad"),
        &m_params.coldStartTimingRetard);
    registry->registerScalar(
        config::describeScalar(base + "coldstart.torque_cap", 0.1, 1.0,
            m_params.coldStartTorqueCap, ""),
        &m_params.coldStartTorqueCap);

    registry->registerScalar(
        config::describeScalar(base + "overrun.cut_speed", units::rpm(500.0), units::rpm(8000.0),
            m_params.overrunCutSpeed, "rad/s"),
        &m_params.overrunCutSpeed);
    registry->registerScalar(
        config::describeScalar(base + "overrun.resume_speed", units::rpm(400.0), units::rpm(8000.0),
            m_params.overrunResumeSpeed, "rad/s"),
        &m_params.overrunResumeSpeed);

    config::ParameterDescriptor throttleMap =
        config::describeScalar(base + "throttle_map", 0.0, 1.0, 0.0, "");
    throttleMap.adaptive = true;
    throttleMap.adaptMin = 0.0;
    throttleMap.adaptMax = 1.0;
    registry->registerMap(throttleMap, &m_throttleMap);

    registry->registerMap(
        config::describeScalar(base + "max_torque_map", 0.0, units::torque(2000.0, units::Nm), 0.0, "Nm"),
        &m_maxTorqueMap);
    registry->registerMap(
        config::describeScalar(base + "pedal_map", 0.0, 1.0, 0.0, ""),
        &m_pedalMap);

    config::ParameterDescriptor idleTrim =
        config::describeScalar(base + "idle.trim", -1.0, 1.0, 0.0, "");
    idleTrim.adaptive = true;
    idleTrim.adaptMin = -1.0;
    idleTrim.adaptMax = 1.0;
    registry->registerMap(idleTrim, &m_idleTrim);

    config::ParameterDescriptor lambdaTrim =
        config::describeScalar(base + "lambda.trim", -1.0, 1.0, 0.0, "");
    lambdaTrim.adaptive = true;
    lambdaTrim.adaptMin = -1.0;
    lambdaTrim.adaptMax = 1.0;
    registry->registerMap(lambdaTrim, &m_lambdaTrim);

    registry->registerBoolean(
        config::describeScalar(base + "lambda.trim_load_manifold", 0.0, 1.0,
            m_params.lambdaTrimLoadIsManifold ? 1.0 : 0.0, ""),
        &m_params.lambdaTrimLoadIsManifold);

    registry->registerMap(
        config::describeScalar(base + "timing_map",
            units::angle(-40.0, units::deg), units::angle(70.0, units::deg),
            0.0, "rad"),
        &m_timingMap);
    registry->registerScalar(
        config::describeScalar(base + "cold_temperature", units::celcius(-40.0),
            units::celcius(60.0), m_params.coldTemperature, "K"),
        &m_params.coldTemperature);
    registry->registerScalar(
        config::describeScalar(base + "warm_temperature", units::celcius(20.0),
            units::celcius(120.0), m_params.warmTemperature, "K"),
        &m_params.warmTemperature);
    registry->registerBoolean(
        config::describeScalar(base + "timing.map_enabled", 0.0, 1.0,
            m_params.timingMapEnabled ? 1.0 : 0.0, ""),
        &m_params.timingMapEnabled);
}

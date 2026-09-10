#include "../include/engine_friction.h"

#include "../include/config/parameter_registry.h"
#include "../include/constants.h"

#include <algorithm>
#include <cmath>

EngineFriction::EngineFriction() {
    reset();
}

EngineFriction::~EngineFriction() {
    /* void */
}

void EngineFriction::initialize(const Parameters &params) {
    m_params = params;
    reset();
}

void EngineFriction::reset() {
    m_peakPressure = 0.0;
}

double EngineFriction::viscosity(double oilTemperature) const {
    const double denominator = oilTemperature - m_params.vogelTheta;
    if (denominator <= 0.0) return 0.0;

    return m_params.vogelK * std::exp(m_params.vogelB / denominator);
}

double EngineFriction::viscosityRatio(double oilTemperature) const {
    if (m_params.vogelB == 0.0) return 1.0;

    const double reference = viscosity(m_params.referenceTemperature);
    if (reference <= 0.0) return 1.0;

    const double current = viscosity(oilTemperature);
    if (current <= 0.0) return 1.0;

    return current / reference;
}

void EngineFriction::updatePeakPressure(double dt, double cylinderPressure) {
    if (dt <= 0.0) return;

    if (m_params.peakPressureDecay > 0.0) {
        m_peakPressure *= std::exp(-dt / m_params.peakPressureDecay);
    }
    else {
        m_peakPressure = 0.0;
    }

    m_peakPressure = std::max(m_peakPressure, cylinderPressure);
}

double EngineFriction::meanPistonSpeed(double crankshaftSpeed, double stroke) const {
    return 2.0 * stroke * (std::abs(crankshaftSpeed) / (2.0 * constants::pi));
}

double EngineFriction::fmep(
    double crankshaftSpeed, double stroke, double viscosityRatio) const
{
    const double v_mp = meanPistonSpeed(crankshaftSpeed, stroke);

    return m_params.constantFmep
        + m_params.peakPressureFactor * m_peakPressure
        + m_params.speedFactor * v_mp * viscosityRatio
        + m_params.speedSquaredFactor * v_mp * v_mp;
}

double EngineFriction::crankTorque(
    double crankshaftSpeed,
    double stroke,
    double displacement,
    double viscosityRatio) const
{
    if (displacement <= 0.0) return 0.0;

    const double pressure = fmep(crankshaftSpeed, stroke, viscosityRatio);
    if (pressure <= 0.0) return 0.0;

    return pressure * displacement / (4.0 * constants::pi);
}

void EngineFriction::registerParameters(config::ParameterRegistry *registry) {
    if (registry == nullptr) return;

    const std::string base = "friction.";

    registry->registerScalar(
        config::describeScalar(base + "constant_fmep", 0.0,
            units::pressure(4.0, units::bar), m_params.constantFmep, "Pa"),
        &m_params.constantFmep);
    registry->registerScalar(
        config::describeScalar(base + "peak_pressure_factor", 0.0, 0.02,
            m_params.peakPressureFactor, ""),
        &m_params.peakPressureFactor);
    registry->registerScalar(
        config::describeScalar(base + "speed_factor", 0.0,
            units::pressure(0.5, units::bar), m_params.speedFactor, "Pa s/m"),
        &m_params.speedFactor);
    registry->registerScalar(
        config::describeScalar(base + "speed_squared_factor", 0.0,
            units::pressure(0.05, units::bar), m_params.speedSquaredFactor, "Pa s2/m2"),
        &m_params.speedSquaredFactor);

    registry->registerScalar(
        config::describeScalar(base + "vogel_k", 0.0, 10.0, m_params.vogelK, "mm2/s"),
        &m_params.vogelK);
    registry->registerScalar(
        config::describeScalar(base + "vogel_b", 0.0, 3000.0, m_params.vogelB, "K"),
        &m_params.vogelB);
    registry->registerScalar(
        config::describeScalar(base + "vogel_theta", 0.0, 250.0, m_params.vogelTheta, "K"),
        &m_params.vogelTheta);
    registry->registerScalar(
        config::describeScalar(base + "reference_temperature",
            units::celcius(20.0), units::celcius(150.0),
            m_params.referenceTemperature, "K"),
        &m_params.referenceTemperature);

    registry->registerScalar(
        config::describeScalar(base + "peak_pressure_decay", 0.0, 5.0,
            m_params.peakPressureDecay, "s"),
        &m_params.peakPressureDecay);
    registry->registerScalar(
        config::describeScalar(base + "heat_to_oil", 0.0, 1.0,
            m_params.frictionHeatToOil, ""),
        &m_params.frictionHeatToOil);
}

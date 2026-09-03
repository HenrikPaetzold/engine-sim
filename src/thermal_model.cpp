#include "../include/thermal_model.h"

#include "../include/config/parameter_registry.h"

#include <algorithm>
#include <cmath>

ThermalModel::ThermalModel() {
    m_blockTemperature = m_params.ambientTemperature;
    m_oilTemperature = m_params.ambientTemperature;
    m_pendingHeat = 0.0;
}

ThermalModel::~ThermalModel() {
    /* void */
}

void ThermalModel::initialize(const Parameters &params) {
    m_params = params;
    reset();
}

void ThermalModel::registerParameters(config::ParameterRegistry *registry, const char *prefix) {
    if (registry == nullptr) return;

    const std::string base = std::string(prefix) + "thermal.";

    registry->registerScalar(
        config::describeScalar(base + "block_mass", 1.0, 1e6, m_params.blockThermalMass, "J/K"),
        &m_params.blockThermalMass);
    registry->registerScalar(
        config::describeScalar(base + "oil_mass", 1.0, 1e6, m_params.oilThermalMass, "J/K"),
        &m_params.oilThermalMass);
    registry->registerScalar(
        config::describeScalar(base + "block_to_oil", 0.0, 1e4, m_params.blockToOilConductance, "W/K"),
        &m_params.blockToOilConductance);
    registry->registerScalar(
        config::describeScalar(base + "radiator", 0.0, 1e5, m_params.radiatorConductance, "W/K"),
        &m_params.radiatorConductance);
    registry->registerScalar(
        config::describeScalar(base + "oil_to_ambient", 0.0, 1e4, m_params.oilToAmbientConductance, "W/K"),
        &m_params.oilToAmbientConductance);
    registry->registerScalar(
        config::describeScalar(base + "speed_cooling", 0.0, 500.0, m_params.speedCoolingCoefficient, ""),
        &m_params.speedCoolingCoefficient);
    registry->registerScalar(
        config::describeScalar(base + "thermostat_open",
            units::celcius(40.0), units::celcius(130.0),
            m_params.thermostatOpenTemperature, "K"),
        &m_params.thermostatOpenTemperature);
    registry->registerScalar(
        config::describeScalar(base + "thermostat_full",
            units::celcius(40.0), units::celcius(140.0),
            m_params.thermostatFullTemperature, "K"),
        &m_params.thermostatFullTemperature);
    registry->registerScalar(
        config::describeScalar(base + "combustion_heat_fraction", 0.0, 2.0,
            m_params.combustionHeatFraction, ""),
        &m_params.combustionHeatFraction);
    registry->registerScalar(
        config::describeScalar(base + "ambient_temperature",
            units::celcius(-40.0), units::celcius(60.0),
            m_params.ambientTemperature, "K"),
        &m_params.ambientTemperature);
}

void ThermalModel::reset() {
    m_blockTemperature = m_params.ambientTemperature;
    m_oilTemperature = m_params.ambientTemperature;
    m_pendingHeat = 0.0;
}

void ThermalModel::addHeat(double energy) {
    m_pendingHeat += energy;
}

double ThermalModel::thermostatOpening() const {
    const double span =
        m_params.thermostatFullTemperature - m_params.thermostatOpenTemperature;
    if (span <= 0.0) {
        return (m_blockTemperature >= m_params.thermostatOpenTemperature) ? 1.0 : 0.0;
    }

    return std::clamp(
        (m_blockTemperature - m_params.thermostatOpenTemperature) / span,
        0.0,
        1.0);
}

void ThermalModel::update(double dt, double vehicleSpeed) {
    if (dt <= 0.0) return;

    const double blockToOil =
        m_params.blockToOilConductance * (m_blockTemperature - m_oilTemperature);

    const double airflow =
        1.0 + m_params.speedCoolingCoefficient * std::abs(vehicleSpeed);
    const double radiator =
        m_params.radiatorConductance
        * thermostatOpening()
        * (m_blockTemperature - m_params.ambientTemperature);

    const double oilToAmbient =
        m_params.oilToAmbientConductance
        * (m_oilTemperature - m_params.ambientTemperature);

    const double blockPower =
        m_pendingHeat / dt - blockToOil - radiator * airflow;
    const double oilPower = blockToOil - oilToAmbient;

    m_pendingHeat = 0.0;

    if (m_params.blockThermalMass > 0.0) {
        m_blockTemperature += (blockPower / m_params.blockThermalMass) * dt;
    }

    if (m_params.oilThermalMass > 0.0) {
        m_oilTemperature += (oilPower / m_params.oilThermalMass) * dt;
    }

    m_blockTemperature = std::max(m_blockTemperature, 0.0);
    m_oilTemperature = std::max(m_oilTemperature, 0.0);
}

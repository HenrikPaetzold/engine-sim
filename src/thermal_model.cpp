#include "../include/thermal_model.h"

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

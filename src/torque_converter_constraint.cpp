#include "../include/torque_converter_constraint.h"

#include "../include/function.h"

#include <algorithm>
#include <cmath>

TorqueConverterConstraint::TorqueConverterConstraint() : Constraint(1, 2) {
    m_ks = 10.0;
    m_kd = 1.0;

    m_stallTorqueRatio = 2.0;
    m_couplingPoint = 0.85;
    m_capacityFactor = 4.04e-3;

    m_capacityCurve = nullptr;
    m_torqueRatioCurve = nullptr;
}

TorqueConverterConstraint::~TorqueConverterConstraint() {
    /* void */
}

double TorqueConverterConstraint::getSpeedRatio() const {
    if (m_bodies[0] == nullptr || m_bodies[1] == nullptr) return 0.0;

    const double pump = m_bodies[0]->v_theta;
    const double turbine = m_bodies[1]->v_theta;

    if (pump * turbine <= 0.0) return 0.0;

    const double ratio = std::abs(turbine) / std::abs(pump);
    return std::clamp(ratio, 0.0, 1.0);
}

double TorqueConverterConstraint::capacityFactor(double speedRatio) const {
    if (m_capacityCurve != nullptr) {
        return std::max(m_capacityCurve->sampleTriangle(speedRatio), 0.0);
    }

    const double sr = std::clamp(speedRatio, 0.0, 1.0);
    return m_capacityFactor * (1.0 - sr * sr);
}

double TorqueConverterConstraint::torqueRatio(double speedRatio) const {
    if (m_torqueRatioCurve != nullptr) {
        return std::max(m_torqueRatioCurve->sampleTriangle(speedRatio), 1.0);
    }

    const double sr = std::clamp(speedRatio, 0.0, 1.0);
    if (m_couplingPoint <= 0.0 || sr >= m_couplingPoint) return 1.0;

    return m_stallTorqueRatio
        + (1.0 - m_stallTorqueRatio) * (sr / m_couplingPoint);
}

void TorqueConverterConstraint::calculate(Output *output, atg_scs::SystemState *state) {
    const double speedRatio = getSpeedRatio();
    const double tr = torqueRatio(speedRatio);

    output->C[0] = 0;

    output->J[0][0] = 0.0;
    output->J[0][1] = 0.0;
    output->J[0][2] = -1.0;

    output->J[0][3] = 0.0;
    output->J[0][4] = 0.0;
    output->J[0][5] = tr;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->J_dot[0][3] = 0;
    output->J_dot[0][4] = 0;
    output->J_dot[0][5] = 0;

    output->kd[0] = m_kd;
    output->ks[0] = m_ks;

    output->v_bias[0] = 0;

    double reference = 0.0;
    if (m_bodies[0] != nullptr && m_bodies[1] != nullptr) {
        reference = std::max(
            std::abs(m_bodies[0]->v_theta),
            std::abs(m_bodies[1]->v_theta));
    }

    const double capacity =
        capacityFactor(speedRatio) * reference * reference;

    output->limits[0][0] = -capacity;
    output->limits[0][1] = capacity;
}

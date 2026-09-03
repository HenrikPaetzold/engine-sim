#include "../include/ratio_clutch_constraint.h"

#include <algorithm>
#include <cmath>

RatioClutchConstraint::RatioClutchConstraint() : Constraint(1, 2) {
    m_ks = 10.0;
    m_kd = 1.0;

    m_ratio = 1.0;
    m_capacity = 0.0;
    m_pressure = 0.0;
}

RatioClutchConstraint::~RatioClutchConstraint() {
    /* void */
}

double RatioClutchConstraint::getSlipSpeed() const {
    if (m_bodies[0] == nullptr || m_bodies[1] == nullptr) return 0.0;
    return m_bodies[0]->v_theta - m_ratio * m_bodies[1]->v_theta;
}

double RatioClutchConstraint::getSlipPower() const {
    return std::abs(getTorque() * getSlipSpeed());
}

void RatioClutchConstraint::calculate(Output *output, atg_scs::SystemState *state) {
    output->C[0] = 0;

    output->J[0][0] = 0.0;
    output->J[0][1] = 0.0;
    output->J[0][2] = -1.0;

    output->J[0][3] = 0.0;
    output->J[0][4] = 0.0;
    output->J[0][5] = m_ratio;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->J_dot[0][3] = 0;
    output->J_dot[0][4] = 0;
    output->J_dot[0][5] = 0;

    output->kd[0] = m_kd;
    output->ks[0] = m_ks;

    output->v_bias[0] = 0;

    const double limit =
        std::max(m_capacity, 0.0) * std::clamp(m_pressure, 0.0, 1.0);

    output->limits[0][0] = -limit;
    output->limits[0][1] = limit;
}

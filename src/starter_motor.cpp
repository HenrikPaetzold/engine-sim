#include "../include/starter_motor.h"

#include "../include/units.h"

#include <algorithm>
#include <cmath>

StarterMotor::StarterMotor() : atg_scs::Constraint(1, 1) {
    m_ks = 10.0;
    m_kd = 1.0;
    m_maxTorque = units::torque(80.0, units::ft_lb);
    m_rotationSpeed = -units::rpm(200.0);
    m_temperature = units::celcius(20.0);
    m_enabled = false;
}

double StarterMotor::availableTorque(double speed) const {
    if (!m_torqueMap.isInitialized()) return m_maxTorque;

    return std::max(0.0, m_torqueMap.sample(std::abs(speed), m_temperature));
}

double StarterMotor::targetSpeed() const {
    if (!m_speedMap.isInitialized()) return m_rotationSpeed;

    const double magnitude = std::max(0.0, m_speedMap.sample(m_temperature, 0.0));

    return (m_rotationSpeed < 0.0) ? -magnitude : magnitude;
}

StarterMotor::~StarterMotor() {
    /* void */
}

void StarterMotor::connectCrankshaft(Crankshaft *crankshaft) {
    m_bodies[0] = &crankshaft->m_body;
}

void StarterMotor::calculate(Output *output, atg_scs::SystemState *state) {
    const double speed = (m_bodies[0] != nullptr) ? m_bodies[0]->v_theta : 0.0;
    const double torque = availableTorque(speed);
    const double target = targetSpeed();

    output->J[0][0] = 0;
    output->J[0][1] = 0;
    output->J[0][2] = 1;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->ks[0] = m_ks;
    output->kd[0] = m_kd;

    output->C[0] = 0;

    output->v_bias[0] = -target;

    if (target < 0) {
        output->limits[0][0] = m_enabled ? -torque : 0.0;
        output->limits[0][1] = 0.0;
    }
    else {
        output->limits[0][0] = 0.0;
        output->limits[0][1] = m_enabled ? torque : 0.0;
    }
}

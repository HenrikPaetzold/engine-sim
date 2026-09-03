#include "../../include/control/pid_controller.h"

#include "../../include/constants.h"

#include <algorithm>
#include <cmath>

control::PidController::PidController() {
    m_integrator = 0.0;
    m_filteredDerivative = 0.0;
    m_previousMeasurement = 0.0;
    m_primed = false;

    m_proportional = 0.0;
    m_derivative = 0.0;
    m_error = 0.0;
    m_output = 0.0;
    m_saturated = false;
}

control::PidController::~PidController() {
    /* void */
}

void control::PidController::initialize(const Parameters &params) {
    m_params = params;
    reset();
}

void control::PidController::reset() {
    m_integrator = 0.0;
    m_filteredDerivative = 0.0;
    m_previousMeasurement = 0.0;
    m_primed = false;

    m_proportional = 0.0;
    m_derivative = 0.0;
    m_error = 0.0;
    m_output = 0.0;
    m_saturated = false;
}

void control::PidController::setIntegrator(double value) {
    m_integrator = std::clamp(
        value,
        -m_params.integratorLimit,
        m_params.integratorLimit);
}

double control::PidController::update(
    double dt,
    double setpoint,
    double measurement,
    double feedforward)
{
    if (dt <= 0.0) return m_output;

    m_error = setpoint - measurement;
    m_proportional = m_params.kp * m_error;

    double rawDerivative = 0.0;
    if (m_primed) {
        rawDerivative = -(measurement - m_previousMeasurement) / dt;
    }

    m_previousMeasurement = measurement;
    m_primed = true;

    if (m_params.derivativeCutoff > 0.0) {
        const double rc = 1.0 / (2.0 * constants::pi * m_params.derivativeCutoff);
        const double alpha = dt / (dt + rc);
        m_filteredDerivative += alpha * (rawDerivative - m_filteredDerivative);
    }
    else {
        m_filteredDerivative = rawDerivative;
    }

    m_derivative = m_params.kd * m_filteredDerivative;

    const double unsaturated =
        m_proportional + m_integrator + m_derivative + feedforward;
    m_output = std::clamp(unsaturated, m_params.outputMin, m_params.outputMax);
    m_saturated = (m_output != unsaturated);

    m_integrator +=
        (m_params.ki * m_error
            + m_params.trackingGain * (m_output - unsaturated)) * dt;
    m_integrator = std::clamp(
        m_integrator,
        -m_params.integratorLimit,
        m_params.integratorLimit);

    return m_output;
}

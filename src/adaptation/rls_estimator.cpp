#include "../../include/adaptation/rls_estimator.h"

#include <algorithm>
#include <cmath>

adaptation::RlsEstimator::RlsEstimator() {
    m_estimate = m_params.initialEstimate;
    m_covariance = m_params.initialCovariance;
    m_residual = 0.0;
}

adaptation::RlsEstimator::~RlsEstimator() {
    /* void */
}

void adaptation::RlsEstimator::initialize(const Parameters &params) {
    m_params = params;
    reset();
}

void adaptation::RlsEstimator::reset() {
    m_estimate = std::clamp(
        m_params.initialEstimate,
        m_params.estimateMin,
        m_params.estimateMax);
    m_covariance = m_params.initialCovariance;
    m_residual = 0.0;
}

double adaptation::RlsEstimator::update(double regressor, double observation) {
    if (std::abs(regressor) < m_params.minimumRegressor) return m_estimate;

    const double lambda = std::clamp(m_params.forgettingFactor, 0.5, 1.0);

    const double denominator = lambda + regressor * m_covariance * regressor;
    if (denominator <= 0.0) return m_estimate;

    const double gain = (m_covariance * regressor) / denominator;

    m_residual = observation - m_estimate * regressor;
    m_estimate = std::clamp(
        m_estimate + gain * m_residual,
        m_params.estimateMin,
        m_params.estimateMax);

    m_covariance = (m_covariance - gain * regressor * m_covariance) / lambda;
    m_covariance = std::clamp(m_covariance, 0.0, m_params.covarianceLimit);

    return m_estimate;
}

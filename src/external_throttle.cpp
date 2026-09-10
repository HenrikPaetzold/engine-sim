#include "../include/external_throttle.h"

#include "../include/engine.h"
#include "../include/config/parameter_registry.h"

#include <algorithm>
#include <string>

ExternalThrottle::ExternalThrottle() {
    m_platePosition = 0.0;
    m_commanded = 0.0;
}

ExternalThrottle::~ExternalThrottle() {
    /* void */
}

void ExternalThrottle::setSpeedControl(double s) {
    Throttle::setSpeedControl(s);
}

void ExternalThrottle::setPlatePosition(double position) {
    m_commanded = std::clamp(position, 0.0, 1.0);

    if (m_params.openRate <= 0.0
        && m_params.closeRate <= 0.0
        && m_params.timeConstant <= 0.0)
    {
        m_platePosition = m_commanded;
        m_limiter.reset(m_platePosition);
    }
}

void ExternalThrottle::reset() {
    m_platePosition = 0.0;
    m_commanded = 0.0;
    m_limiter.reset(0.0);
}

void ExternalThrottle::registerParameters(config::ParameterRegistry *registry)
{
    if (registry == nullptr) return;

    const std::string base = "throttle.";

    const auto describe = [](const std::string &path, double max, double value) {
        return config::describeScalar(path, 0.0, max, value, "");
    };

    registry->registerScalar(
        describe(base + "open_rate", 50.0, m_params.openRate),
        &m_params.openRate);
    registry->registerScalar(
        describe(base + "close_rate", 50.0, m_params.closeRate),
        &m_params.closeRate);
    registry->registerScalar(
        describe(base + "time_constant", 1.0, m_params.timeConstant),
        &m_params.timeConstant);
}

void ExternalThrottle::update(double dt, Engine *engine) {
    Throttle::update(dt, engine);

    if (dt > 0.0) {
        m_limiter.setRates(m_params.openRate, m_params.closeRate);
        const double limited = m_limiter.update(dt, m_commanded);

        if (m_params.timeConstant > 0.0) {
            const double alpha = dt / (dt + m_params.timeConstant);
            m_platePosition += alpha * (limited - m_platePosition);
        }
        else {
            m_platePosition = limited;
        }

        m_platePosition = std::clamp(m_platePosition, 0.0, 1.0);
    }

    if (engine != nullptr) engine->setThrottle(1.0 - m_platePosition);
}

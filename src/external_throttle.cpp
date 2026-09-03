#include "../include/external_throttle.h"

#include "../include/engine.h"

#include <algorithm>

ExternalThrottle::ExternalThrottle() {
    m_platePosition = 0.0;
}

ExternalThrottle::~ExternalThrottle() {
    /* void */
}

void ExternalThrottle::setSpeedControl(double s) {
    Throttle::setSpeedControl(s);
}

void ExternalThrottle::setPlatePosition(double position) {
    m_platePosition = std::clamp(position, 0.0, 1.0);
}

void ExternalThrottle::update(double dt, Engine *engine) {
    Throttle::update(dt, engine);
    engine->setThrottle(1.0 - m_platePosition);
}

#include "../../include/powertrain/manoeuvre.h"

#include <algorithm>

powertrain::Manoeuvre::Manoeuvre() {
    /* void */
}

powertrain::Manoeuvre::~Manoeuvre() {
    /* void */
}

void powertrain::Manoeuvre::add(const Setpoint &setpoint) {
    if (static_cast<int>(m_setpoints.size()) >= MaxSetpoints) return;

    m_setpoints.push_back(setpoint);
}

void powertrain::Manoeuvre::sort() {
    std::stable_sort(
        m_setpoints.begin(),
        m_setpoints.end(),
        [](const Setpoint &a, const Setpoint &b) { return a.time < b.time; });
}

void powertrain::Manoeuvre::clear() {
    m_setpoints.clear();
}

double powertrain::Manoeuvre::getDuration() const {
    if (m_setpoints.empty()) return 0.0;

    return m_setpoints.back().time;
}

powertrain::DriverInputs powertrain::Manoeuvre::sample(double time) const {
    DriverInputs inputs;
    if (m_setpoints.empty()) return inputs;

    const Setpoint *previous = &m_setpoints.front();
    const Setpoint *next = &m_setpoints.front();

    for (const Setpoint &setpoint : m_setpoints) {
        if (setpoint.time <= time) previous = &setpoint;
        if (setpoint.time > time) { next = &setpoint; break; }
        next = &setpoint;
    }

    const double span = next->time - previous->time;
    const double t = (span > 0.0)
        ? std::clamp((time - previous->time) / span, 0.0, 1.0)
        : 0.0;

    inputs.accelerator = previous->accelerator
        + t * (next->accelerator - previous->accelerator);
    inputs.brake = previous->brake + t * (next->brake - previous->brake);
    inputs.clutchPedal = previous->clutchPedal
        + t * (next->clutchPedal - previous->clutchPedal);

    inputs.gatePosition = previous->gatePosition;
    inputs.selectedGear = previous->selectedGear;
    inputs.driveMode = previous->driveMode;
    inputs.manualMode = previous->manualMode;
    inputs.shiftUpRequest = previous->shiftUp;
    inputs.shiftDownRequest = previous->shiftDown;
    inputs.ignitionKey = previous->ignitionKey;
    inputs.starterRequest = previous->starterRequest;

    return inputs;
}

powertrain::ManoeuvrePlayer::ManoeuvrePlayer() {
    m_manoeuvre = nullptr;
    m_running = false;
    m_startTime = 0.0;
    m_elapsed = 0.0;
    m_lastIndex = -1;
}

powertrain::ManoeuvrePlayer::~ManoeuvrePlayer() {
    /* void */
}

void powertrain::ManoeuvrePlayer::setManoeuvre(const Manoeuvre *manoeuvre) {
    m_manoeuvre = manoeuvre;
    stop();
}

void powertrain::ManoeuvrePlayer::start(double time) {
    if (m_manoeuvre == nullptr || m_manoeuvre->isEmpty()) return;

    m_running = true;
    m_startTime = time;
    m_elapsed = 0.0;
    m_lastIndex = -1;
}

void powertrain::ManoeuvrePlayer::stop() {
    m_running = false;
    m_elapsed = 0.0;
    m_lastIndex = -1;
}

double powertrain::ManoeuvrePlayer::getProgress() const {
    if (m_manoeuvre == nullptr) return 0.0;

    const double duration = m_manoeuvre->getDuration();
    if (duration <= 0.0) return m_running ? 1.0 : 0.0;

    return std::clamp(m_elapsed / duration, 0.0, 1.0);
}

bool powertrain::ManoeuvrePlayer::update(double time, DriverInputs *inputs) {
    if (!m_running || m_manoeuvre == nullptr || inputs == nullptr) return false;

    m_elapsed = time - m_startTime;

    if (m_elapsed > m_manoeuvre->getDuration()) {
        stop();
        return false;
    }

    *inputs = m_manoeuvre->sample(m_elapsed);

    int index = -1;
    for (int i = 0; i < m_manoeuvre->getCount(); ++i) {
        if (m_manoeuvre->get(i).time <= m_elapsed) index = i;
    }

    if (index == m_lastIndex) {
        inputs->shiftUpRequest = false;
        inputs->shiftDownRequest = false;
    }

    m_lastIndex = index;

    return true;
}

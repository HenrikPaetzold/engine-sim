#include "../../include/powertrain/manoeuvre.h"

#include "../../include/config/mr_number.h"

#include <algorithm>
#include <cmath>
#include <sstream>

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

    inputs->shiftUpRequest = false;
    inputs->shiftDownRequest = false;

    for (int i = m_lastIndex + 1; i <= index; ++i) {
        if (m_manoeuvre->get(i).shiftUp) inputs->shiftUpRequest = true;
        if (m_manoeuvre->get(i).shiftDown) inputs->shiftDownRequest = true;
    }

    m_lastIndex = index;

    return true;
}

powertrain::ManoeuvreRecorder::ManoeuvreRecorder() {
    m_recording = false;
    m_startTime = 0.0;
    m_elapsed = 0.0;
    m_sinceSample = 0.0;
    m_interval = 0.02;
    m_tolerance = 0.01;
}

powertrain::ManoeuvreRecorder::~ManoeuvreRecorder() {
    /* void */
}

void powertrain::ManoeuvreRecorder::start(double time) {
    m_samples.clear();
    m_recording = true;
    m_startTime = time;
    m_elapsed = 0.0;
    m_sinceSample = m_interval;
}

void powertrain::ManoeuvreRecorder::stop() {
    m_recording = false;
}

bool powertrain::ManoeuvreRecorder::discreteChanged(
    const Setpoint &a, const Setpoint &b)
{
    return a.gatePosition != b.gatePosition
        || a.selectedGear != b.selectedGear
        || a.driveMode != b.driveMode
        || a.manualMode != b.manualMode
        || a.shiftUp != b.shiftUp
        || a.shiftDown != b.shiftDown
        || a.ignitionKey != b.ignitionKey
        || a.starterRequest != b.starterRequest;
}

void powertrain::ManoeuvreRecorder::update(double time, const DriverInputs &inputs) {
    if (!m_recording) return;
    if (static_cast<int>(m_samples.size()) >= MaxSamples) return;

    const double elapsed = time - m_startTime;
    m_sinceSample += elapsed - m_elapsed;
    m_elapsed = elapsed;

    Setpoint setpoint;
    setpoint.time = elapsed;
    setpoint.accelerator = inputs.accelerator;
    setpoint.brake = inputs.brake;
    setpoint.clutchPedal = inputs.clutchPedal;
    setpoint.gatePosition = inputs.gatePosition;
    setpoint.selectedGear = inputs.selectedGear;
    setpoint.driveMode = inputs.driveMode;
    setpoint.manualMode = inputs.manualMode;
    setpoint.shiftUp = inputs.shiftUpRequest;
    setpoint.shiftDown = inputs.shiftDownRequest;
    setpoint.ignitionKey = inputs.ignitionKey;
    setpoint.starterRequest = inputs.starterRequest;

    const bool forced =
        m_samples.empty() || discreteChanged(m_samples.back(), setpoint);

    if (!forced && m_sinceSample < m_interval) return;

    m_samples.push_back(setpoint);
    m_sinceSample = 0.0;
}

void powertrain::ManoeuvreRecorder::thin(Manoeuvre *manoeuvre) const {
    if (manoeuvre == nullptr) return;

    manoeuvre->clear();
    if (m_samples.empty()) return;

    const int count = static_cast<int>(m_samples.size());
    if (count == 1) {
        manoeuvre->add(m_samples.front());
        return;
    }

    std::vector<bool> forced(count, false);
    for (int i = 1; i < count; ++i) {
        if (!discreteChanged(m_samples[i - 1], m_samples[i])) continue;

        forced[i - 1] = true;
        forced[i] = true;
    }

    const auto within = [this](int a, int b, int k) {
        const double span = m_samples[b].time - m_samples[a].time;
        if (span <= 0.0) return true;

        const double t = (m_samples[k].time - m_samples[a].time) / span;

        const double pedal = m_samples[a].accelerator
            + t * (m_samples[b].accelerator - m_samples[a].accelerator);
        const double brake = m_samples[a].brake
            + t * (m_samples[b].brake - m_samples[a].brake);
        const double clutch = m_samples[a].clutchPedal
            + t * (m_samples[b].clutchPedal - m_samples[a].clutchPedal);

        return std::abs(pedal - m_samples[k].accelerator) <= m_tolerance
            && std::abs(brake - m_samples[k].brake) <= m_tolerance
            && std::abs(clutch - m_samples[k].clutchPedal) <= m_tolerance;
    };

    manoeuvre->add(m_samples.front());

    int anchor = 0;
    while (anchor < count - 1) {
        int best = anchor + 1;

        for (int end = anchor + 2; end < count; ++end) {
            bool ok = true;
            for (int k = anchor + 1; k < end && ok; ++k) {
                if (forced[k] || !within(anchor, end, k)) ok = false;
            }

            if (!ok) break;

            best = end;
            if (forced[end]) break;
        }

        manoeuvre->add(m_samples[best]);
        anchor = best;
    }

    manoeuvre->sort();
}

std::string powertrain::ManoeuvreRecorder::toScript(const std::string &name) const {
    Manoeuvre manoeuvre;
    thin(&manoeuvre);

    std::ostringstream out;
    out << "add_manoeuvre(\n    manoeuvre(name: \"" << name << "\")";

    for (int i = 0; i < manoeuvre.getCount(); ++i) {
        const Setpoint &setpoint = manoeuvre.get(i);

        out << "\n        .at(time: " << config::mrNumber(setpoint.time)
            << ", accelerator: " << config::mrNumber(setpoint.accelerator)
            << ", brake: " << config::mrNumber(setpoint.brake)
            << ", clutch: " << config::mrNumber(setpoint.clutchPedal)
            << ", gate: " << setpoint.gatePosition
            << ", gear: " << setpoint.selectedGear
            << ", drive_mode: " << setpoint.driveMode
            << ", manual: " << (setpoint.manualMode ? "true" : "false")
            << ", shift_up: " << (setpoint.shiftUp ? "true" : "false")
            << ", shift_down: " << (setpoint.shiftDown ? "true" : "false")
            << ", ignition: " << (setpoint.ignitionKey ? "true" : "false")
            << ", starter: " << (setpoint.starterRequest ? "true" : "false")
            << ")";
    }

    out << ")\n";

    return out.str();
}

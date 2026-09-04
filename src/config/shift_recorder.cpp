#include "../../include/config/shift_recorder.h"

#include <algorithm>

namespace {
    const config::ShiftRecorder::Recording s_empty;
}

config::ShiftRecorder::ShiftRecorder() {
    m_duration = 1.5;
    m_interval = m_duration / MaxSamples;
    m_elapsed = 0.0;
    m_sinceSample = 0.0;
    m_startGear = -1;
    m_recording = false;
    m_previousShifting = false;
}

config::ShiftRecorder::~ShiftRecorder() {
    /* void */
}

void config::ShiftRecorder::initialize(double duration) {
    m_duration = (duration > 0.0) ? duration : 1.5;
    m_interval = m_duration / MaxSamples;
    reset();
}

void config::ShiftRecorder::reset() {
    m_recordings.clear();
    m_current = Recording();
    m_elapsed = 0.0;
    m_sinceSample = 0.0;
    m_startGear = -1;
    m_recording = false;
    m_previousShifting = false;
}

void config::ShiftRecorder::begin(int gear, double time) {
    m_current = Recording();
    m_current.fromGear = gear;
    m_current.toGear = gear;
    m_current.startTime = time;
    m_current.samples.reserve(MaxSamples);

    m_elapsed = 0.0;
    m_sinceSample = m_interval;
    m_startGear = gear;
    m_recording = true;
}

void config::ShiftRecorder::finish() {
    if (!m_recording) return;

    m_recording = false;
    if (m_current.samples.empty()) return;

    if (static_cast<int>(m_recordings.size()) >= MaxRecordings) {
        m_recordings.erase(m_recordings.begin());
    }

    m_recordings.push_back(m_current);
    m_current = Recording();
}

void config::ShiftRecorder::update(
    double dt,
    bool shifting,
    int gear,
    const Sample &sample)
{
    if (shifting && !m_previousShifting && !m_recording) {
        begin(gear, sample.time);
    }

    m_previousShifting = shifting;

    if (!m_recording) return;

    m_elapsed += dt;
    m_sinceSample += dt;

    if (gear != m_startGear) m_current.toGear = gear;

    if (m_sinceSample >= m_interval
        && static_cast<int>(m_current.samples.size()) < MaxSamples)
    {
        m_sinceSample = 0.0;

        Sample entry = sample;
        entry.time = m_elapsed;
        m_current.samples.push_back(entry);
    }

    if (m_elapsed >= m_duration
        || static_cast<int>(m_current.samples.size()) >= MaxSamples)
    {
        finish();
    }
}

int config::ShiftRecorder::getCount() const {
    return static_cast<int>(m_recordings.size());
}

const config::ShiftRecorder::Recording &config::ShiftRecorder::get(int index) const {
    if (index < 0 || index >= getCount()) return s_empty;
    return m_recordings[index];
}

void config::ShiftRecorder::serializeJson(std::ostream &out) const {
    out << '[';

    for (size_t i = 0; i < m_recordings.size(); ++i) {
        const Recording &recording = m_recordings[i];
        if (i != 0) out << ',';

        out << "{\"fromGear\":" << recording.fromGear
            << ",\"toGear\":" << recording.toGear
            << ",\"startTime\":" << recording.startTime
            << ",\"samples\":[";

        for (size_t j = 0; j < recording.samples.size(); ++j) {
            const Sample &s = recording.samples[j];
            if (j != 0) out << ',';

            out << '[' << s.time
                << ',' << s.clutchPressure
                << ',' << s.engineSpeed
                << ',' << s.torqueRequest
                << ',' << s.torqueReduction
                << ',' << s.clutchSlip
                << ']';
        }

        out << "]}";
    }

    out << ']';
}

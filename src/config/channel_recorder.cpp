#include "../../include/config/channel_recorder.h"

#include <algorithm>
#include <cassert>

namespace {
    const std::string s_missing;

    void writeJsonString(std::ostream &out, const std::string &s) {
        out << '"';
        for (char c : s) {
            if (c == '"' || c == '\\') out << '\\' << c;
            else out << c;
        }
        out << '"';
    }
}

config::ChannelTable::ChannelTable() {
    /* void */
}

config::ChannelTable::~ChannelTable() {
    /* void */
}

int config::ChannelTable::define(const std::string &name) {
    const auto existing = m_index.find(name);
    if (existing != m_index.end()) return existing->second;

    const int index = static_cast<int>(m_names.size());
    m_names.push_back(name);
    m_values.push_back(0.0);
    m_index[name] = index;

    return index;
}

void config::ChannelTable::set(int index, double value) {
    if (index < 0 || index >= static_cast<int>(m_values.size())) return;
    m_values[index] = value;
}

void config::ChannelTable::set(const std::string &name, double value) {
    m_values[define(name)] = value;
}

int config::ChannelTable::find(const std::string &name) const {
    const auto existing = m_index.find(name);
    if (existing == m_index.end()) return -1;

    return existing->second;
}

int config::ChannelTable::getCount() const {
    return static_cast<int>(m_names.size());
}

const std::string &config::ChannelTable::getName(int index) const {
    if (index < 0 || index >= getCount()) return s_missing;
    return m_names[index];
}

double config::ChannelTable::getValue(int index) const {
    if (index < 0 || index >= getCount()) return 0.0;
    return m_values[index];
}

void config::ChannelTable::serializeNames(std::ostream &out) const {
    out << '[';
    for (size_t i = 0; i < m_names.size(); ++i) {
        if (i != 0) out << ',';
        writeJsonString(out, m_names[i]);
    }
    out << ']';
}

config::ChannelRecorder::ChannelRecorder() {
    m_mode = Mode::Rolling;
    m_window = 4.0;
    m_interval = m_window / MaxSamples;
    m_sinceSample = 0.0;
    m_armed = false;
    m_resolved = false;
    m_full = false;
}

config::ChannelRecorder::~ChannelRecorder() {
    /* void */
}

void config::ChannelRecorder::initialize(double window) {
    setWindow(window);
    reset();
}

void config::ChannelRecorder::setWindow(double window) {
    m_window = (window > 0.0) ? window : 4.0;
    m_interval = m_window / MaxSamples;
}

void config::ChannelRecorder::setMode(Mode mode) {
    m_mode = mode;
    reset();
}

void config::ChannelRecorder::reset() {
    for (Track &track : m_tracks) {
        track.samples.clear();
        track.index = -1;
    }

    m_time.clear();
    m_sinceSample = m_interval;
    m_resolved = false;
    m_full = false;
    m_armed = (m_mode == Mode::Rolling);
}

void config::ChannelRecorder::select(const std::vector<std::string> &channels) {
    m_tracks.clear();

    for (const std::string &name : channels) {
        if (static_cast<int>(m_tracks.size()) >= MaxChannels) break;
        if (name.empty()) continue;

        Track track;
        track.name = name;
        m_tracks.push_back(track);
    }

    reset();
}

void config::ChannelRecorder::arm() {
    m_time.clear();
    for (Track &track : m_tracks) track.samples.clear();

    m_sinceSample = m_interval;
    m_full = false;
    m_armed = true;
}

void config::ChannelRecorder::resolve(const ChannelTable &table) {
    for (Track &track : m_tracks) {
        track.index = table.find(track.name);
    }

    m_resolved = true;
}

void config::ChannelRecorder::push(double time, const ChannelTable &table) {
    m_time.push_back(time);
    for (Track &track : m_tracks) {
        track.samples.push_back(table.getValue(track.index));
    }

    if (static_cast<int>(m_time.size()) <= MaxSamples) return;

    if (m_mode == Mode::Rolling) {
        m_time.erase(m_time.begin());
        for (Track &track : m_tracks) track.samples.erase(track.samples.begin());
    }
    else {
        m_time.pop_back();
        for (Track &track : m_tracks) track.samples.pop_back();
        m_full = true;
        m_armed = false;
    }
}

void config::ChannelRecorder::update(
    double dt,
    double time,
    const ChannelTable &table)
{
    if (m_tracks.empty() || dt <= 0.0) return;
    if (!m_armed) return;

    if (!m_resolved) resolve(table);

    m_sinceSample += dt;
    if (m_sinceSample < m_interval) return;

    m_sinceSample = 0.0;
    push(time, table);
}

int config::ChannelRecorder::getChannelCount() const {
    return static_cast<int>(m_tracks.size());
}

int config::ChannelRecorder::getSampleCount() const {
    return static_cast<int>(m_time.size());
}

const std::string &config::ChannelRecorder::getChannelName(int channel) const {
    if (channel < 0 || channel >= getChannelCount()) return s_missing;
    return m_tracks[channel].name;
}

double config::ChannelRecorder::getSample(int channel, int sample) const {
    if (channel < 0 || channel >= getChannelCount()) return 0.0;
    if (sample < 0 || sample >= getSampleCount()) return 0.0;

    return m_tracks[channel].samples[sample];
}

void config::ChannelRecorder::serializeJson(std::ostream &out) const {
    out << "{\"window\":" << m_window
        << ",\"mode\":" << (m_mode == Mode::Rolling ? "\"rolling\"" : "\"triggered\"")
        << ",\"armed\":" << (m_armed ? "true" : "false")
        << ",\"full\":" << (m_full ? "true" : "false")
        << ",\"time\":[";

    for (size_t i = 0; i < m_time.size(); ++i) {
        if (i != 0) out << ',';
        out << m_time[i];
    }

    out << "],\"channels\":[";

    for (size_t c = 0; c < m_tracks.size(); ++c) {
        if (c != 0) out << ',';

        out << "{\"name\":";
        writeJsonString(out, m_tracks[c].name);
        out << ",\"found\":" << (m_tracks[c].index >= 0 ? "true" : "false")
            << ",\"samples\":[";

        for (size_t i = 0; i < m_tracks[c].samples.size(); ++i) {
            if (i != 0) out << ',';
            out << m_tracks[c].samples[i];
        }

        out << "]}";
    }

    out << "]}";
}

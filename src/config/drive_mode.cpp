#include "../../include/config/drive_mode.h"

#include "../../include/config/parameter_registry.h"

#include <cassert>

config::DriveMode::DriveMode() {
    /* void */
}

config::DriveMode::DriveMode(const std::string &name) {
    m_name = name;
}

config::DriveMode::~DriveMode() {
    /* void */
}

void config::DriveMode::set(const std::string &path, double value) {
    for (DriveModeOverride &existing : m_overrides) {
        if (existing.path == path) {
            existing.value = value;
            return;
        }
    }

    m_overrides.push_back({ path, value });
}

int config::DriveMode::getOverrideCount() const {
    return static_cast<int>(m_overrides.size());
}

const config::DriveModeOverride &config::DriveMode::getOverride(int index) const {
    assert(index >= 0 && index < getOverrideCount());
    return m_overrides[index];
}

config::DriveModeSet::DriveModeSet() {
    m_selected = -1;
    m_baselineCaptured = false;
}

config::DriveModeSet::~DriveModeSet() {
    /* void */
}

void config::DriveModeSet::clear() {
    m_modes.clear();
    m_baseline.clear();
    m_selected = -1;
    m_baselineCaptured = false;
}

void config::DriveModeSet::add(const DriveMode &mode) {
    m_modes.push_back(mode);
    m_baselineCaptured = false;
}

int config::DriveModeSet::getCount() const {
    return static_cast<int>(m_modes.size());
}

const config::DriveMode &config::DriveModeSet::get(int index) const {
    assert(index >= 0 && index < getCount());
    return m_modes[index];
}

int config::DriveModeSet::find(const std::string &name) const {
    for (int i = 0; i < getCount(); ++i) {
        if (m_modes[i].getName() == name) return i;
    }

    return -1;
}

void config::DriveModeSet::captureBaseline(const ParameterRegistry *registry) {
    if (registry == nullptr) return;

    m_baseline.clear();

    for (const DriveMode &mode : m_modes) {
        for (int i = 0; i < mode.getOverrideCount(); ++i) {
            const std::string &path = mode.getOverride(i).path;
            if (m_baseline.find(path) != m_baseline.end()) continue;

            double value = 0.0;
            if (registry->get(path, &value)) m_baseline[path] = value;
        }
    }

    m_baselineCaptured = true;
}

void config::DriveModeSet::restoreBaseline(ParameterRegistry *registry) const {
    if (registry == nullptr) return;

    for (const auto &entry : m_baseline) {
        registry->set(entry.first, entry.second);
    }
}

bool config::DriveModeSet::select(int index, ParameterRegistry *registry) {
    if (registry == nullptr) return false;
    if (index < 0 || index >= getCount()) return false;

    if (!m_baselineCaptured) captureBaseline(registry);

    restoreBaseline(registry);

    const DriveMode &mode = m_modes[index];
    for (int i = 0; i < mode.getOverrideCount(); ++i) {
        const DriveModeOverride &item = mode.getOverride(i);
        registry->set(item.path, item.value);
    }

    m_selected = index;

    return true;
}

bool config::DriveModeSet::select(const std::string &name, ParameterRegistry *registry) {
    const int index = find(name);
    if (index < 0) return false;

    return select(index, registry);
}

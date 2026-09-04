#include "../../include/config/drive_mode.h"

#include "../../include/config/parameter_registry.h"
#include "../../include/control/map_2d.h"

#include <cassert>

std::string config::mapCellPath(const std::string &path, int x, int y) {
    return path + "[" + std::to_string(x) + "][" + std::to_string(y) + "]";
}

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

void config::DriveMode::setMap(
    const std::string &path,
    std::shared_ptr<control::Map2d> map)
{
    for (DriveModeMapOverride &existing : m_mapOverrides) {
        if (existing.path == path) {
            existing.map = map;
            return;
        }
    }

    m_mapOverrides.push_back({ path, map });
}

void config::DriveMode::expand(
    const ParameterRegistry *registry,
    std::vector<DriveModeOverride> *out) const
{
    *out = m_overrides;
    if (registry == nullptr) return;

    for (const DriveModeMapOverride &item : m_mapOverrides) {
        if (item.map == nullptr || !item.map->isInitialized()) continue;

        const control::Map2d *target = registry->findMap(item.path);
        if (target == nullptr || !target->isInitialized()) continue;

        for (int y = 0; y < target->getYCount(); ++y) {
            for (int x = 0; x < target->getXCount(); ++x) {
                out->push_back({
                    mapCellPath(item.path, x, y),
                    item.map->sample(target->getXAxis(x), target->getYAxis(y)) });
            }
        }
    }
}

int config::DriveMode::getMapOverrideCount() const {
    return static_cast<int>(m_mapOverrides.size());
}

const config::DriveModeMapOverride &config::DriveMode::getMapOverride(int index) const {
    assert(index >= 0 && index < getMapOverrideCount());
    return m_mapOverrides[index];
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

    std::vector<DriveModeOverride> expanded;

    for (const DriveMode &mode : m_modes) {
        mode.expand(registry, &expanded);

        for (const DriveModeOverride &item : expanded) {
            if (m_baseline.find(item.path) != m_baseline.end()) continue;

            double value = 0.0;
            if (registry->get(item.path, &value)) m_baseline[item.path] = value;
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

    std::vector<DriveModeOverride> expanded;
    m_modes[index].expand(registry, &expanded);

    for (const DriveModeOverride &item : expanded) {
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

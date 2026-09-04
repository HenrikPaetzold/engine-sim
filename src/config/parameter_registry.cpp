#include "../../include/config/parameter_registry.h"

#include "../../include/control/map_2d.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <ostream>

namespace {
    const char *typeName(config::ParameterType type) {
        switch (type) {
        case config::ParameterType::Scalar: return "scalar";
        case config::ParameterType::Integer: return "integer";
        case config::ParameterType::Boolean: return "boolean";
        case config::ParameterType::Map: return "map";
        default: return "scalar";
        }
    }

    bool parseCellPath(const std::string &path, std::string *base, int *x, int *y) {
        if (path.empty() || path.back() != ']') return false;

        const size_t second = path.rfind('[');
        if (second == std::string::npos || second == 0) return false;

        const size_t firstEnd = path.rfind(']', second);
        if (firstEnd == std::string::npos || firstEnd + 1 != second) return false;

        const size_t first = path.rfind('[', firstEnd);
        if (first == std::string::npos || first == 0) return false;

        const std::string xs = path.substr(first + 1, firstEnd - first - 1);
        const std::string ys = path.substr(second + 1, path.size() - second - 2);
        if (xs.empty() || ys.empty()) return false;

        for (char c : xs) if (c < '0' || c > '9') return false;
        for (char c : ys) if (c < '0' || c > '9') return false;

        *base = path.substr(0, first);
        *x = std::stoi(xs);
        *y = std::stoi(ys);

        return true;
    }

    void writeJsonString(std::ostream &out, const std::string &s) {
        out << '"';
        for (char c : s) {
            if (c == '"' || c == '\\') out << '\\' << c;
            else if (c == '\n') out << "\\n";
            else out << c;
        }
        out << '"';
    }
}

config::ParameterDescriptor config::describeScalar(
    const std::string &path,
    double minValue,
    double maxValue,
    double defaultValue,
    const char *unit)
{
    ParameterDescriptor descriptor;
    descriptor.path = path;
    descriptor.minValue = minValue;
    descriptor.maxValue = maxValue;
    descriptor.defaultValue = defaultValue;
    descriptor.unit = unit;

    return descriptor;
}

config::ParameterRegistry::ParameterRegistry() {
    /* void */
}

config::ParameterRegistry::~ParameterRegistry() {
    /* void */
}

void config::ParameterRegistry::clear() {
    m_entries.clear();
    m_index.clear();
}

bool config::ParameterRegistry::add(const Entry &entry) {
    if (entry.descriptor.path.empty()) return false;
    if (m_index.find(entry.descriptor.path) != m_index.end()) return false;

    m_index[entry.descriptor.path] = static_cast<int>(m_entries.size());
    m_entries.push_back(entry);

    return true;
}

bool config::ParameterRegistry::registerScalar(
    const ParameterDescriptor &descriptor,
    double *target)
{
    if (target == nullptr) return false;

    Entry entry;
    entry.descriptor = descriptor;
    entry.descriptor.type = ParameterType::Scalar;
    entry.scalarTarget = target;

    if (!add(entry)) return false;

    *target = std::clamp(
        descriptor.defaultValue,
        descriptor.minValue,
        descriptor.maxValue);

    return true;
}

bool config::ParameterRegistry::registerInteger(
    const ParameterDescriptor &descriptor,
    int *target)
{
    if (target == nullptr) return false;

    Entry entry;
    entry.descriptor = descriptor;
    entry.descriptor.type = ParameterType::Integer;
    entry.integerTarget = target;

    if (!add(entry)) return false;

    *target = static_cast<int>(std::lround(std::clamp(
        descriptor.defaultValue,
        descriptor.minValue,
        descriptor.maxValue)));

    return true;
}

bool config::ParameterRegistry::registerBoolean(
    const ParameterDescriptor &descriptor,
    bool *target)
{
    if (target == nullptr) return false;

    Entry entry;
    entry.descriptor = descriptor;
    entry.descriptor.type = ParameterType::Boolean;
    entry.descriptor.minValue = 0.0;
    entry.descriptor.maxValue = 1.0;
    entry.booleanTarget = target;

    if (!add(entry)) return false;

    *target = (descriptor.defaultValue != 0.0);

    return true;
}

bool config::ParameterRegistry::registerMap(
    const ParameterDescriptor &descriptor,
    control::Map2d *target)
{
    if (target == nullptr) return false;

    Entry entry;
    entry.descriptor = descriptor;
    entry.descriptor.type = ParameterType::Map;
    entry.mapTarget = target;

    return add(entry);
}

const config::ParameterRegistry::Entry *config::ParameterRegistry::find(
    const std::string &path) const
{
    const auto i = m_index.find(path);
    if (i == m_index.end()) return nullptr;

    return &m_entries[i->second];
}

config::ParameterRegistry::Entry *config::ParameterRegistry::find(const std::string &path) {
    const auto i = m_index.find(path);
    if (i == m_index.end()) return nullptr;

    return &m_entries[i->second];
}

bool config::ParameterRegistry::contains(const std::string &path) const {
    if (find(path) != nullptr) return true;

    control::Map2d *map = nullptr;
    int x = 0;
    int y = 0;

    return findCell(path, &map, &x, &y);
}

control::Map2d *config::ParameterRegistry::findMap(const std::string &path) const {
    const Entry *entry = find(path);
    if (entry == nullptr) return nullptr;
    if (entry->descriptor.type != ParameterType::Map) return nullptr;

    return entry->mapTarget;
}

bool config::ParameterRegistry::findCell(
    const std::string &path,
    control::Map2d **map,
    int *x,
    int *y) const
{
    std::string base;
    if (!parseCellPath(path, &base, x, y)) return false;

    const Entry *entry = find(base);
    if (entry == nullptr) return false;
    if (entry->descriptor.type != ParameterType::Map) return false;
    if (entry->mapTarget == nullptr || !entry->mapTarget->isInitialized()) return false;
    if (*x < 0 || *x >= entry->mapTarget->getXCount()) return false;
    if (*y < 0 || *y >= entry->mapTarget->getYCount()) return false;

    *map = entry->mapTarget;

    return true;
}

void config::ParameterRegistry::writeValue(const Entry &entry, double value) {
    const double clamped = std::clamp(
        value,
        entry.descriptor.minValue,
        entry.descriptor.maxValue);

    if (entry.scalarTarget != nullptr) *entry.scalarTarget = clamped;
    else if (entry.integerTarget != nullptr) *entry.integerTarget = static_cast<int>(std::lround(clamped));
    else if (entry.booleanTarget != nullptr) *entry.booleanTarget = (clamped != 0.0);
}

double config::ParameterRegistry::readValue(const Entry &entry) {
    if (entry.scalarTarget != nullptr) return *entry.scalarTarget;
    else if (entry.integerTarget != nullptr) return static_cast<double>(*entry.integerTarget);
    else if (entry.booleanTarget != nullptr) return *entry.booleanTarget ? 1.0 : 0.0;
    else return 0.0;
}

bool config::ParameterRegistry::set(const std::string &path, double value) {
    Entry *entry = find(path);
    if (entry != nullptr) {
        if (entry->descriptor.type == ParameterType::Map) return false;

        writeValue(*entry, value);

        return true;
    }

    control::Map2d *map = nullptr;
    int x = 0;
    int y = 0;
    if (!findCell(path, &map, &x, &y)) return false;

    map->setValue(x, y, value);

    return true;
}

bool config::ParameterRegistry::get(const std::string &path, double *value) const {
    const Entry *entry = find(path);
    if (entry != nullptr) {
        if (entry->descriptor.type == ParameterType::Map) return false;

        *value = readValue(*entry);

        return true;
    }

    control::Map2d *map = nullptr;
    int x = 0;
    int y = 0;
    if (!findCell(path, &map, &x, &y)) return false;

    *value = map->getValue(x, y);

    return true;
}

bool config::ParameterRegistry::isAdaptive(const std::string &path) const {
    const Entry *entry = find(path);
    return entry != nullptr && entry->descriptor.adaptive;
}

bool config::ParameterRegistry::adapt(const std::string &path, double delta) {
    Entry *entry = find(path);
    if (entry == nullptr) return false;
    if (!entry->descriptor.adaptive) return false;
    if (entry->descriptor.type == ParameterType::Map) return false;

    const double updated = std::clamp(
        readValue(*entry) + delta,
        entry->descriptor.adaptMin,
        entry->descriptor.adaptMax);

    writeValue(*entry, updated);

    return true;
}

bool config::ParameterRegistry::setAdaptive(const std::string &path, bool adaptive) {
    Entry *entry = find(path);
    if (entry == nullptr) return false;

    entry->descriptor.adaptive = adaptive;

    return true;
}

void config::ParameterRegistry::resetToDefaults() {
    for (Entry &entry : m_entries) {
        if (entry.descriptor.type == ParameterType::Map) continue;
        writeValue(entry, entry.descriptor.defaultValue);
    }
}

int config::ParameterRegistry::getCount() const {
    return static_cast<int>(m_entries.size());
}

const config::ParameterDescriptor &config::ParameterRegistry::getDescriptor(int index) const {
    assert(index >= 0 && index < getCount());
    return m_entries[index].descriptor;
}

double config::ParameterRegistry::getValue(int index) const {
    assert(index >= 0 && index < getCount());
    return readValue(m_entries[index]);
}

control::Map2d *config::ParameterRegistry::getMap(int index) const {
    assert(index >= 0 && index < getCount());
    return m_entries[index].mapTarget;
}

void config::ParameterRegistry::serializeJson(std::ostream &out) const {
    out << "{\"parameters\":[";

    for (size_t i = 0; i < m_entries.size(); ++i) {
        const Entry &entry = m_entries[i];
        const ParameterDescriptor &d = entry.descriptor;

        if (i != 0) out << ',';

        out << "{\"path\":";
        writeJsonString(out, d.path);
        out << ",\"type\":";
        writeJsonString(out, typeName(d.type));
        out << ",\"unit\":";
        writeJsonString(out, d.unit);
        out << ",\"min\":" << d.minValue
            << ",\"max\":" << d.maxValue
            << ",\"default\":" << d.defaultValue
            << ",\"adaptive\":" << (d.adaptive ? "true" : "false");

        if (d.adaptive) {
            out << ",\"adaptMin\":" << d.adaptMin
                << ",\"adaptMax\":" << d.adaptMax;
        }

        if (d.type == ParameterType::Map && entry.mapTarget != nullptr) {
            const control::Map2d *map = entry.mapTarget;
            out << ",\"xAxis\":[";
            for (int x = 0; x < map->getXCount(); ++x) {
                if (x != 0) out << ',';
                out << map->getXAxis(x);
            }
            out << "],\"yAxis\":[";
            for (int y = 0; y < map->getYCount(); ++y) {
                if (y != 0) out << ',';
                out << map->getYAxis(y);
            }
            out << "],\"values\":[";
            for (int y = 0; y < map->getYCount(); ++y) {
                for (int x = 0; x < map->getXCount(); ++x) {
                    if (x != 0 || y != 0) out << ',';
                    out << map->getValue(x, y);
                }
            }
            out << ']';
        }
        else {
            out << ",\"value\":" << readValue(entry);
        }

        out << '}';
    }

    out << "]}";
}

void config::ParameterRegistry::exportScript(std::ostream &out, ExportScope scope) const {
    for (const Entry &entry : m_entries) {
        if (scope == ExportScope::Learned) {
            if (!entry.descriptor.adaptive) continue;
        }
        else if (entry.descriptor.type == ParameterType::Map) {
            if (!entry.descriptor.adaptive) continue;
        }
        else if (readValue(entry) == entry.descriptor.defaultValue) {
            continue;
        }

        if (entry.descriptor.type == ParameterType::Map) {
            const control::Map2d *map = entry.mapTarget;
            if (map == nullptr) continue;

            for (int y = 0; y < map->getYCount(); ++y) {
                for (int x = 0; x < map->getXCount(); ++x) {
                    out << "set_map_cell(\"" << entry.descriptor.path << "\", "
                        << x << ", " << y << ", "
                        << map->getValue(x, y) << ")\n";
                }
            }
        }
        else {
            out << "set_parameter(\"" << entry.descriptor.path << "\", "
                << readValue(entry) << ")\n";
        }
    }
}

#include "../../include/config/parameter_registry.h"

#include "../../include/config/json_writer.h"
#include "../../include/control/map_2d.h"

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

    double displayLow(const config::ParameterDescriptor &d) {
        return (d.displayMin < d.displayMax) ? d.displayMin : d.minValue;
    }

    double displayHigh(const config::ParameterDescriptor &d) {
        return (d.displayMin < d.displayMax) ? d.displayMax : d.maxValue;
    }
}

void config::ParameterRegistry::serializeJson(std::ostream &out) const {
    out << "{\"parameters\":[";

    for (size_t i = 0; i < m_entries.size(); ++i) {
        const Entry &entry = m_entries[i];
        const ParameterDescriptor &d = entry.descriptor;

        if (i != 0) out << ',';

        out << "{\"path\":";
        config::writeJsonString(out, d.path);
        out << ",\"type\":";
        config::writeJsonString(out, typeName(d.type));
        out << ",\"unit\":";
        config::writeJsonString(out, d.unit);
        out << ",\"min\":" << d.minValue
            << ",\"max\":" << d.maxValue
            << ",\"display_min\":" << displayLow(d)
            << ",\"display_max\":" << displayHigh(d)
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

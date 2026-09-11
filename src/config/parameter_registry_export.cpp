#include "../../include/config/parameter_registry.h"

#include "../../include/control/map_2d.h"
#include "../../include/config/mr_number.h"

#include <ostream>

void config::ParameterRegistry::exportScript(std::ostream &out, ExportScope scope) const {
    for (const Entry &entry : m_entries) {
        if (scope == ExportScope::Learned) {
            if (!entry.descriptor.adaptive) continue;
        }
        else if (entry.descriptor.type == ParameterType::Map) {
            /* void */
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
                        << mrNumber(map->getValue(x, y)) << ")\n";
                }
            }
        }
        else {
            out << "set_parameter(\"" << entry.descriptor.path << "\", "
                << mrNumber(readValue(entry)) << ")\n";
        }
    }
}

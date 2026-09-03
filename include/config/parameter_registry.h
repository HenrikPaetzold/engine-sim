#ifndef ATG_ENGINE_SIM_PARAMETER_REGISTRY_H
#define ATG_ENGINE_SIM_PARAMETER_REGISTRY_H

#include <string>
#include <vector>
#include <unordered_map>
#include <iosfwd>
#include <cfloat>

namespace control {
    class Map2d;
}

namespace config {

    enum class ParameterType {
        Scalar,
        Integer,
        Boolean,
        Map
    };

    struct ParameterDescriptor {
        std::string path;
        ParameterType type = ParameterType::Scalar;
        double minValue = 0.0;
        double maxValue = 1.0;
        double defaultValue = 0.0;
        std::string unit;
        bool adaptive = false;
        double adaptMin = 0.0;
        double adaptMax = 0.0;
    };

    class ParameterRegistry {
        public:
            ParameterRegistry();
            ~ParameterRegistry();

            void clear();

            bool registerScalar(const ParameterDescriptor &descriptor, double *target);
            bool registerInteger(const ParameterDescriptor &descriptor, int *target);
            bool registerBoolean(const ParameterDescriptor &descriptor, bool *target);
            bool registerMap(const ParameterDescriptor &descriptor, control::Map2d *target);

            bool contains(const std::string &path) const;
            bool set(const std::string &path, double value);
            bool get(const std::string &path, double *value) const;

            bool adapt(const std::string &path, double delta);
            bool isAdaptive(const std::string &path) const;

            void resetToDefaults();

            void serializeJson(std::ostream &out) const;
            void exportScript(std::ostream &out) const;

            int getCount() const;
            const ParameterDescriptor &getDescriptor(int index) const;
            double getValue(int index) const;
            control::Map2d *getMap(int index) const;

        protected:
            struct Entry {
                ParameterDescriptor descriptor;
                double *scalarTarget = nullptr;
                int *integerTarget = nullptr;
                bool *booleanTarget = nullptr;
                control::Map2d *mapTarget = nullptr;
            };

            bool add(const Entry &entry);
            const Entry *find(const std::string &path) const;
            Entry *find(const std::string &path);

            static void writeValue(const Entry &entry, double value);
            static double readValue(const Entry &entry);

            std::vector<Entry> m_entries;
            std::unordered_map<std::string, int> m_index;
    };

} /* namespace config */

#endif /* ATG_ENGINE_SIM_PARAMETER_REGISTRY_H */

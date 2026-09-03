#ifndef ATG_ENGINE_SIM_DRIVE_MODE_H
#define ATG_ENGINE_SIM_DRIVE_MODE_H

#include <string>
#include <vector>
#include <unordered_map>

namespace config {

    class ParameterRegistry;

    struct DriveModeOverride {
        std::string path;
        double value = 0.0;
    };

    class DriveMode {
        public:
            DriveMode();
            explicit DriveMode(const std::string &name);
            ~DriveMode();

            void setName(const std::string &name) { m_name = name; }
            const std::string &getName() const { return m_name; }

            void set(const std::string &path, double value);

            int getOverrideCount() const;
            const DriveModeOverride &getOverride(int index) const;

        protected:
            std::string m_name;
            std::vector<DriveModeOverride> m_overrides;
    };

    class DriveModeSet {
        public:
            DriveModeSet();
            ~DriveModeSet();

            void clear();
            void add(const DriveMode &mode);

            int getCount() const;
            const DriveMode &get(int index) const;
            int find(const std::string &name) const;

            bool select(int index, ParameterRegistry *registry);
            bool select(const std::string &name, ParameterRegistry *registry);

            inline int getSelected() const { return m_selected; }
            void captureBaseline(const ParameterRegistry *registry);
            void restoreBaseline(ParameterRegistry *registry) const;

        protected:
            std::vector<DriveMode> m_modes;
            std::unordered_map<std::string, double> m_baseline;

            int m_selected;
            bool m_baselineCaptured;
    };

} /* namespace config */

#endif /* ATG_ENGINE_SIM_DRIVE_MODE_H */

#ifndef ATG_ENGINE_SIM_DRIVE_MODE_H
#define ATG_ENGINE_SIM_DRIVE_MODE_H

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

namespace control {
    class Map2d;
}

namespace config {

    class ParameterRegistry;

    struct DriveModeOverride {
        std::string path;
        double value = 0.0;
    };

    struct DriveModeMapOverride {
        std::string path;
        std::shared_ptr<control::Map2d> map;
    };

    std::string mapCellPath(const std::string &path, int x, int y);

    class DriveMode {
        public:
            DriveMode();
            explicit DriveMode(const std::string &name);
            ~DriveMode();

            void setName(const std::string &name) { m_name = name; }
            const std::string &getName() const { return m_name; }

            void set(const std::string &path, double value);
            void setMap(const std::string &path, std::shared_ptr<control::Map2d> map);

            int getOverrideCount() const;
            const DriveModeOverride &getOverride(int index) const;

            int getMapOverrideCount() const;
            const DriveModeMapOverride &getMapOverride(int index) const;

            void expand(
                const ParameterRegistry *registry,
                std::vector<DriveModeOverride> *out) const;

        protected:
            std::string m_name;
            std::vector<DriveModeOverride> m_overrides;
            std::vector<DriveModeMapOverride> m_mapOverrides;
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

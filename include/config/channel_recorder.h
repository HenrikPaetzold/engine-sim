#ifndef ATG_ENGINE_SIM_CHANNEL_RECORDER_H
#define ATG_ENGINE_SIM_CHANNEL_RECORDER_H

#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace config {

    class ChannelTable {
        public:
            ChannelTable();
            ~ChannelTable();

            int define(const std::string &name);
            void set(int index, double value);
            void set(const std::string &name, double value);

            int find(const std::string &name) const;
            int getCount() const;
            const std::string &getName(int index) const;
            double getValue(int index) const;

            void serializeNames(std::ostream &out) const;

        protected:
            std::vector<std::string> m_names;
            std::vector<double> m_values;
            std::unordered_map<std::string, int> m_index;
    };

    class ChannelRecorder {
        public:
            static constexpr int MaxChannels = 8;
            static constexpr int MaxSamples = 1024;

            enum class Mode {
                Rolling,
                Triggered
            };

        public:
            ChannelRecorder();
            ~ChannelRecorder();

            void initialize(double window);
            void reset();

            void setWindow(double window);
            void setMode(Mode mode);
            void select(const std::vector<std::string> &channels);
            void arm();

            void update(double dt, double time, const ChannelTable &table);

            inline Mode getMode() const { return m_mode; }
            inline double getWindow() const { return m_window; }
            inline bool isArmed() const { return m_armed; }
            int getChannelCount() const;
            int getSampleCount() const;
            const std::string &getChannelName(int channel) const;
            double getSample(int channel, int sample) const;

            void serializeJson(std::ostream &out) const;

        protected:
            void resolve(const ChannelTable &table);
            void push(double time, const ChannelTable &table);

            struct Track {
                std::string name;
                int index = -1;
                std::vector<double> samples;
            };

            std::vector<Track> m_tracks;
            std::vector<double> m_time;

            Mode m_mode;
            double m_window;
            double m_interval;
            double m_sinceSample;
            bool m_armed;
            bool m_resolved;
            bool m_full;
    };

} /* namespace config */

#endif /* ATG_ENGINE_SIM_CHANNEL_RECORDER_H */

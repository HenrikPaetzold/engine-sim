#ifndef ATG_ENGINE_SIM_SHIFT_RECORDER_H
#define ATG_ENGINE_SIM_SHIFT_RECORDER_H

#include <ostream>
#include <vector>

namespace config {

    class ShiftRecorder {
        public:
            static constexpr int MaxSamples = 512;
            static constexpr int MaxRecordings = 8;

            struct Sample {
                double time = 0.0;
                double clutchPressure = 0.0;
                double engineSpeed = 0.0;
                double torqueRequest = 0.0;
                double torqueReduction = 0.0;
                double clutchSlip = 0.0;
            };

            struct Recording {
                int fromGear = -1;
                int toGear = -1;
                double startTime = 0.0;
                std::vector<Sample> samples;
            };

        public:
            ShiftRecorder();
            ~ShiftRecorder();

            void initialize(double duration);
            void reset();

            void update(double dt, bool shifting, int gear, const Sample &sample);

            int getCount() const;
            const Recording &get(int index) const;
            void serializeJson(std::ostream &out) const;

        protected:
            void begin(int gear, double time);
            void finish();

            std::vector<Recording> m_recordings;
            Recording m_current;

            double m_duration;
            double m_interval;
            double m_elapsed;
            double m_sinceSample;
            int m_startGear;
            bool m_recording;
            bool m_previousShifting;
    };

} /* namespace config */

#endif /* ATG_ENGINE_SIM_SHIFT_RECORDER_H */

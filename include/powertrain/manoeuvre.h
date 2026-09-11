#ifndef ATG_ENGINE_SIM_MANOEUVRE_H
#define ATG_ENGINE_SIM_MANOEUVRE_H

#include "driver_inputs.h"

#include <string>
#include <vector>

namespace powertrain {

    struct Setpoint {
        double time = 0.0;

        double accelerator = 0.0;
        double brake = 0.0;
        double clutchPedal = 0.0;

        int gatePosition = -1;
        int selectedGear = -1;
        int driveMode = 0;

        bool manualMode = false;
        bool shiftUp = false;
        bool shiftDown = false;
        bool ignitionKey = true;
        bool starterRequest = false;
    };

    class Manoeuvre {
        public:
            static constexpr int MaxSetpoints = 512;

        public:
            Manoeuvre();
            ~Manoeuvre();

            void setName(const std::string &name) { m_name = name; }
            const std::string &getName() const { return m_name; }

            bool add(const Setpoint &setpoint);
            void sort();
            void clear();

            int getDroppedCount() const { return m_dropped; }
            bool isTruncated() const { return m_dropped > 0; }

            int getCount() const { return static_cast<int>(m_setpoints.size()); }
            const Setpoint &get(int index) const { return m_setpoints[index]; }
            double getDuration() const;
            bool isEmpty() const { return m_setpoints.empty(); }

            DriverInputs sample(double time) const;

        protected:
            std::string m_name;
            std::vector<Setpoint> m_setpoints;
            int m_dropped = 0;
    };

    class ManoeuvrePlayer {
        public:
            ManoeuvrePlayer();
            ~ManoeuvrePlayer();

            void setManoeuvre(const Manoeuvre *manoeuvre);
            const Manoeuvre *getManoeuvre() const { return m_manoeuvre; }

            void start(double time);
            void stop();

            bool isRunning() const { return m_running; }
            double getElapsed() const { return m_elapsed; }
            double getProgress() const;

            bool update(double time, DriverInputs *inputs);

        protected:
            const Manoeuvre *m_manoeuvre;

            bool m_running;
            double m_startTime;
            double m_elapsed;
            int m_lastIndex;
    };

    class ManoeuvreRecorder {
        public:
            static constexpr int MaxSamples = 8192;

        public:
            ManoeuvreRecorder();
            ~ManoeuvreRecorder();

            void setInterval(double interval) { m_interval = interval; }
            void setTolerance(double tolerance) { m_tolerance = tolerance; }

            void start(double time);
            void stop();
            bool isRecording() const { return m_recording; }

            int getCount() const { return static_cast<int>(m_samples.size()); }
            double getElapsed() const { return m_elapsed; }
            int getDroppedCount() const { return m_dropped; }
            bool isFull() const { return m_dropped > 0; }

            void update(double time, const DriverInputs &inputs);

            void thin(Manoeuvre *manoeuvre) const;
            std::string toScript(const std::string &name) const;

        protected:
            static bool discreteChanged(const Setpoint &a, const Setpoint &b);

            std::vector<Setpoint> m_samples;

            bool m_recording;
            double m_startTime;
            double m_elapsed;
            double m_sinceSample;
            double m_interval;
            double m_tolerance;
            int m_dropped;
    };

}

#endif /* ATG_ENGINE_SIM_MANOEUVRE_H */

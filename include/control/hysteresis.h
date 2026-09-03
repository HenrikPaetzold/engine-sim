#ifndef ATG_ENGINE_SIM_HYSTERESIS_H
#define ATG_ENGINE_SIM_HYSTERESIS_H

namespace control {

    class Hysteresis {
        public:
            Hysteresis() : m_lowThreshold(0.0), m_highThreshold(0.0), m_state(false) { /* void */ }
            ~Hysteresis() { /* void */ }

            inline void initialize(double lowThreshold, double highThreshold, bool state = false) {
                m_lowThreshold = lowThreshold;
                m_highThreshold = highThreshold;
                m_state = state;
            }

            inline bool update(double value) {
                if (m_state) {
                    if (value < m_lowThreshold) m_state = false;
                }
                else {
                    if (value > m_highThreshold) m_state = true;
                }

                return m_state;
            }

            inline bool getState() const { return m_state; }
            inline void setState(bool state) { m_state = state; }

        protected:
            double m_lowThreshold;
            double m_highThreshold;
            bool m_state;
    };

    class StateTimer {
        public:
            StateTimer() : m_elapsed(0.0) { /* void */ }
            ~StateTimer() { /* void */ }

            inline void reset() { m_elapsed = 0.0; }
            inline void advance(double dt) { m_elapsed += dt; }
            inline double getElapsed() const { return m_elapsed; }
            inline bool hasElapsed(double duration) const { return m_elapsed >= duration; }

        protected:
            double m_elapsed;
    };

} /* namespace control */

#endif /* ATG_ENGINE_SIM_HYSTERESIS_H */

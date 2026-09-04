#ifndef ATG_ENGINE_SIM_RATE_LIMITER_H
#define ATG_ENGINE_SIM_RATE_LIMITER_H

#include <algorithm>

namespace control {

    class RateLimiter {
        public:
            RateLimiter() : m_riseRate(0.0), m_fallRate(0.0), m_value(0.0), m_primed(false) { /* void */ }
            ~RateLimiter() { /* void */ }

            inline void initialize(double riseRate, double fallRate) {
                m_riseRate = riseRate;
                m_fallRate = fallRate;
                m_primed = false;
            }

            inline void setRates(double riseRate, double fallRate) {
                m_riseRate = riseRate;
                m_fallRate = fallRate;
            }

            inline void reset(double value) {
                m_value = value;
                m_primed = true;
            }

            inline double update(double dt, double target) {
                if (!m_primed) {
                    m_value = target;
                    m_primed = true;
                    return m_value;
                }

                const double delta = target - m_value;
                if (delta > 0.0 && m_riseRate > 0.0) {
                    m_value += std::min(delta, m_riseRate * dt);
                }
                else if (delta < 0.0 && m_fallRate > 0.0) {
                    m_value -= std::min(-delta, m_fallRate * dt);
                }
                else {
                    m_value = target;
                }

                return m_value;
            }

            inline double getValue() const { return m_value; }

        protected:
            double m_riseRate;
            double m_fallRate;
            double m_value;
            bool m_primed;
    };

} /* namespace control */

#endif /* ATG_ENGINE_SIM_RATE_LIMITER_H */

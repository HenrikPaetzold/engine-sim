#ifndef ATG_ENGINE_SIM_EXTERNAL_THROTTLE_H
#define ATG_ENGINE_SIM_EXTERNAL_THROTTLE_H

#include "throttle.h"

#include "control/rate_limiter.h"

namespace config {
    class ParameterRegistry;
}

class ExternalThrottle : public Throttle {
    public:
        struct Parameters {
            double openRate = 0.0;
            double closeRate = 0.0;
            double timeConstant = 0.0;
        };

    public:
        ExternalThrottle();
        virtual ~ExternalThrottle();

        virtual void setSpeedControl(double s);
        virtual void update(double dt, Engine *engine);

        void registerParameters(config::ParameterRegistry *registry);
        void reset();

        void setPlatePosition(double position);
        inline double getPlatePosition() const { return m_platePosition; }
        inline double getCommandedPosition() const { return m_commanded; }

        Parameters m_params;

    protected:
        double m_platePosition;
        double m_commanded;
        control::RateLimiter m_limiter;
};

#endif /* ATG_ENGINE_SIM_EXTERNAL_THROTTLE_H */

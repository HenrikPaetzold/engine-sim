#ifndef ATG_ENGINE_SIM_EXTERNAL_THROTTLE_H
#define ATG_ENGINE_SIM_EXTERNAL_THROTTLE_H

#include "throttle.h"

class ExternalThrottle : public Throttle {
    public:
        ExternalThrottle();
        virtual ~ExternalThrottle();

        virtual void setSpeedControl(double s);
        virtual void update(double dt, Engine *engine);

        void setPlatePosition(double position);
        inline double getPlatePosition() const { return m_platePosition; }

    protected:
        double m_platePosition;
};

#endif /* ATG_ENGINE_SIM_EXTERNAL_THROTTLE_H */

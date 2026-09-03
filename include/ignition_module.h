#ifndef ATG_ENGINE_SIM_IGNITION_MODULE_H
#define ATG_ENGINE_SIM_IGNITION_MODULE_H

#include "part.h"

#include "crankshaft.h"
#include "function.h"
#include "units.h"

class IgnitionModule : public Part {
    public:
        struct Parameters {
            int cylinderCount;
            Crankshaft *crankshaft;
            Function *timingCurve;
            double revLimit = units::rpm(6000.0);
            double limiterDuration = 0.5 * units::sec;
        };

        struct SparkPlug {
            double angle = 0;
            bool ignitionEvent = false;
            bool enabled = false;
        };

    public:
        IgnitionModule();
        virtual ~IgnitionModule();

        virtual void destroy();

        void initialize(const Parameters &params);
        void setFiringOrder(int cylinderIndex, double angle);
        void reset();
        void update(double dt);

        bool getIgnitionEvent(int index) const;
        void resetIgnitionEvents();

        double getTimingAdvance();

        void setTimingOffset(double offset);
        inline double getTimingOffset() const { return m_timingOffset; }

        void setCutFraction(double fraction);
        inline double getCutFraction() const { return m_cutFraction; }

        inline void setRevLimit(double revLimit) { m_revLimit = revLimit; }
        inline double getRevLimit() const { return m_revLimit; }
        inline void setLimiterDuration(double duration) { m_limiterDuration = duration; }
        inline double getLimiterDuration() const { return m_limiterDuration; }

        inline bool isLimiterActive() const { return m_revLimitTimer > 0; }

        bool m_enabled;

        bool consumeCutDecision();

    protected:
        SparkPlug *getPlug(int i);

        Function *m_timingCurve;
        SparkPlug *m_plugs;
        Crankshaft *m_crankshaft;
        int m_cylinderCount;

        double m_lastCrankshaftAngle;
        double m_revLimit;
        double m_revLimitTimer;
        double m_limiterDuration;

        double m_timingOffset;
        double m_cutFraction;
        double m_cutAccumulator;
};

#endif /* ATG_ENGINE_SIM_IGNITION_MODULE_H */

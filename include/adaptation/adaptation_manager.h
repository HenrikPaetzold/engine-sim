#ifndef ATG_ENGINE_SIM_ADAPTATION_MANAGER_H
#define ATG_ENGINE_SIM_ADAPTATION_MANAGER_H

#include "rls_estimator.h"

#include "../control/iterative_learning.h"

#include "../control/map_2d.h"
#include "../powertrain/powertrain_state.h"
#include "../powertrain/powertrain_bus.h"
#include "../units.h"

namespace config {
    class ParameterRegistry;
}

namespace powertrain {
    class EngineControlUnit;
    class TransmissionControlUnit;
}

namespace adaptation {

    struct EnableConditions {
        bool requireWarm = true;
        double warmTemperature = units::celcius(70.0);
        bool requireSteadySpeed = true;
        double speedStabilityWindow = units::rpm(120.0);
        bool requireNoShift = true;
        bool requireNoLimiting = true;
        double minimumSpeed = units::rpm(500.0);
    };

    class AdaptationManager {
        public:
            struct Parameters {
                bool throttleMapEnabled = true;
                bool idleEnabled = true;
                bool lambdaEnabled = true;
                bool shiftEnabled = true;

                double throttleLearningRate = 0.5;
                double throttleDeadband = 0.01;
                double throttleCorrectionLimit = 0.35;

                double idleDrainRate = 0.6;
                double idleTrimLimit = 0.35;

                double lambdaShortTermGain = 0.8;
                double lambdaLongTermRate = 0.05;
                double lambdaTrimLimit = 0.30;
                double lambdaTarget = 0.02;

                EnableConditions conditions;
                RlsEstimator::Parameters torqueModel;
            };

        public:
            AdaptationManager();
            ~AdaptationManager();

            void initialize(const Parameters &params);
            void reset();

            void attach(
                powertrain::EngineControlUnit *ecu,
                powertrain::TransmissionControlUnit *tcu);

            void registerParameters(config::ParameterRegistry *registry, const char *prefix);

            void update(
                double dt,
                const powertrain::PowertrainState &state,
                const powertrain::PowertrainBus &bus);

            bool conditionsMet(
                const powertrain::PowertrainState &state,
                const powertrain::PowertrainBus &bus) const;

            int getShiftIterationCount() const;
            double getShiftErrorNorm() const;
            inline const RlsEstimator &getTorqueModel() const { return m_torqueModel; }
            inline double getShortTermFuelTrim() const { return m_shortTermTrim; }
            inline int getThrottleUpdateCount() const { return m_throttleUpdates; }
            inline bool wasEnabledLastUpdate() const { return m_enabled; }

        protected:
            void updateThrottleMap(double dt, const powertrain::PowertrainState &state);
            void updateIdleTrim(double dt, const powertrain::PowertrainState &state);
            void updateLambdaTrim(double dt, const powertrain::PowertrainState &state);
            void updateShiftLearning(double dt, const powertrain::PowertrainState &state);

            Parameters m_params;

            powertrain::EngineControlUnit *m_ecu;
            powertrain::TransmissionControlUnit *m_tcu;

            RlsEstimator m_torqueModel;

            double m_shortTermTrim;
            double m_speedAverage;
            double m_speedDeviation;
            bool m_speedPrimed;

            bool m_enabled;
            bool m_shiftActive;
            double m_shiftElapsed;
            int m_throttleUpdates;
    };

} /* namespace adaptation */

#endif /* ATG_ENGINE_SIM_ADAPTATION_MANAGER_H */

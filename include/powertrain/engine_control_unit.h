#ifndef ATG_ENGINE_SIM_ENGINE_CONTROL_UNIT_H
#define ATG_ENGINE_SIM_ENGINE_CONTROL_UNIT_H

#include "powertrain_controller.h"

#include "../control/pid_controller.h"
#include "../control/map_2d.h"
#include "../control/rate_limiter.h"
#include "../control/hysteresis.h"
#include "../units.h"

namespace powertrain {

    class EngineControlUnit : public PowertrainController {
        public:
            struct Parameters {
                double referenceTorque = units::torque(200.0, units::Nm);
                double torqueRiseRate = units::torque(2000.0, units::Nm);
                double torqueFallRate = units::torque(4000.0, units::Nm);

                double idleSpeedCold = units::rpm(1300.0);
                double idleSpeedWarm = units::rpm(800.0);
                double coldTemperature = units::celcius(-10.0);
                double warmTemperature = units::celcius(80.0);

                double coldStartEnrichment = 1.6;
                double coldStartTimingRetard = units::angle(8.0, units::deg);
                double coldStartTorqueCap = 0.6;

                double revLimit = units::rpm(7000.0);
                double revLimitCold = units::rpm(7000.0);
                double softLimitBand = units::rpm(300.0);
                double hardLimitOffset = units::rpm(150.0);
                double limiterDuration = 0.5 * units::sec;

                double overrunCutSpeed = units::rpm(2000.0);
                double overrunResumeSpeed = units::rpm(1400.0);

                double crankingSpeed = units::rpm(400.0);
                double stallSpeed = units::rpm(200.0);

                control::PidController::Parameters idleController = defaultIdleController();
                control::PidController::Parameters torqueController = defaultTorqueController();
            };

            static control::PidController::Parameters defaultIdleController();
            static control::PidController::Parameters defaultTorqueController();

        public:
            EngineControlUnit();
            virtual ~EngineControlUnit();

            void initialize(const Parameters &params);

            virtual void registerParameters(config::ParameterRegistry *registry, const char *prefix);
            virtual void reset();

            virtual void update(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs,
                ActuatorCommands *commands);

            void setTransmissionRequests(const PowertrainBus &bus);

            inline control::Map2d &getThrottleMap() { return m_throttleMap; }
            inline control::Map2d &getIdleTrimMap() { return m_idleTrim; }
            inline control::PidController &getIdleController() { return m_idleController; }
            inline control::PidController &getTorqueController() { return m_torqueController; }
            inline const control::PidController &getIdleController() const { return m_idleController; }
            inline const control::PidController &getTorqueController() const { return m_torqueController; }

            inline void setFuelTrim(double trim) { m_fuelTrim = trim; }
            inline double getFuelTrim() const { return m_fuelTrim; }
            inline double getFeedforwardPlate() const { return m_feedforwardPlate; }
            inline double getCommandedPlate() const { return m_commandedPlate; }
            inline control::Map2d &getMaxTorqueMap() { return m_maxTorqueMap; }
            inline control::Map2d &getPedalMap() { return m_pedalMap; }

            double maxTorqueAt(double engineSpeed) const;
            double idleSpeedAt(double coolantTemperature) const;
            double warmupFraction(double coolantTemperature) const;

            inline double getTorqueRequest() const { return m_torqueRequest; }
            inline double getDriverTorqueRequest() const { return m_driverTorqueRequest; }
            inline double getIdleTorqueRequest() const { return m_idleTorqueRequest; }
            inline EngineState getEngineState() const { return m_engineState; }
            double effectiveRevLimit(double coolantTemperature) const;
            inline const Parameters &getParameters() const { return m_params; }

        protected:
            void buildDefaultMaps();
            EngineState resolveState(
                const PowertrainState &state,
                const DriverInputs &inputs,
                bool limiterActive) const;

            Parameters m_params;

            control::Map2d m_throttleMap;
            control::Map2d m_idleTrim;
            control::Map2d m_maxTorqueMap;
            control::Map2d m_pedalMap;

            control::PidController m_idleController;
            control::PidController m_torqueController;
            control::RateLimiter m_torqueLimiter;
            control::Hysteresis m_overrunCut;

            double m_torqueRequest;
            double m_driverTorqueRequest;
            double m_idleTorqueRequest;
            double m_feedforwardPlate;
            double m_commandedPlate;
            double m_fuelTrim;

            EngineState m_engineState;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_ENGINE_CONTROL_UNIT_H */

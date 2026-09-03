#ifndef ATG_ENGINE_SIM_SCRIPTED_CONTROL_UNIT_H
#define ATG_ENGINE_SIM_SCRIPTED_CONTROL_UNIT_H

#include "powertrain_controller.h"

#include "../control/control_program.h"

namespace powertrain {

    namespace signals {
        enum Signal {
            Dt,
            Time,
            EngineSpeed,
            EngineRpm,
            ThrottlePlate,
            ManifoldPressure,
            IntakeAfr,
            ExhaustO2,
            IndicatedTorque,
            TimingAdvance,
            CoolantTemperature,
            OilTemperature,
            Gear,
            PreselectedGear,
            GearCount,
            ClutchPressure,
            ClutchPressure2,
            ClutchSlip,
            ClutchSlip2,
            TurbineSpeed,
            ConverterSlip,
            LockupPressure,
            VehicleSpeed,
            WheelSpeed,
            RoadGrade,
            EngineRunning,
            Accelerator,
            Brake,
            ClutchPedal,
            DriveMode,
            SelectedGear,
            ShiftUp,
            ShiftDown,
            ManualMode,
            GatePosition,
            Engagement,
            ParkLockEngaged,
            IgnitionKey,
            StarterRequest,
            Count
        };

        const char *name(int signal);
    }

    namespace actuators {
        enum Actuator {
            ThrottlePlate,
            IgnitionCut,
            FuelCut,
            FuelEnrichment,
            TimingOffset,
            RevLimit,
            LimiterDuration,
            TargetGear,
            PreselectGear,
            ClutchPressure,
            ClutchPressure2,
            LockupPressure,
            StarterEnabled,
            IgnitionEnabled,
            GatePosition,
            ParkLock,
            Count
        };

        const char *name(int actuator);
    }

    class ScriptedControlUnit : public PowertrainController {
        public:
            ScriptedControlUnit();
            virtual ~ScriptedControlUnit();

            void initialize();

            inline control::ControlProgram &getProgram() { return m_program; }
            inline const control::ControlProgram &getProgram() const { return m_program; }

            virtual void registerParameters(config::ParameterRegistry *registry, const char *prefix) override;
            virtual void fillTelemetry(config::TelemetrySample *sample) const override;
            virtual void reset() override;

            virtual void update(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs,
                ActuatorCommands *commands) override;

        protected:
            void sampleSignals(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs);
            void seedActuators(const PowertrainState &state);
            void applyActuators(ActuatorCommands *commands) const;

            control::ControlProgram m_program;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_SCRIPTED_CONTROL_UNIT_H */

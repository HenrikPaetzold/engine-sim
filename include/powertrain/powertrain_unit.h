#ifndef ATG_ENGINE_SIM_POWERTRAIN_UNIT_H
#define ATG_ENGINE_SIM_POWERTRAIN_UNIT_H

#include "engine_control_unit.h"
#include "transmission_control_unit.h"

namespace powertrain {

    class PowertrainUnit : public PowertrainController {
        public:
            PowertrainUnit();
            virtual ~PowertrainUnit();

            void initialize(
                const EngineControlUnit::Parameters &engineParams,
                const TransmissionControlUnit::Parameters &transmissionParams);

            virtual void registerParameters(config::ParameterRegistry *registry, const char *prefix);
            virtual void reset();

            virtual void update(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs,
                ActuatorCommands *commands);

            inline EngineControlUnit &getEngineControlUnit() { return m_ecu; }
            inline TransmissionControlUnit &getTransmissionControlUnit() { return m_tcu; }

        protected:
            EngineControlUnit m_ecu;
            TransmissionControlUnit m_tcu;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_POWERTRAIN_UNIT_H */

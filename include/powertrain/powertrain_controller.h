#ifndef ATG_ENGINE_SIM_POWERTRAIN_CONTROLLER_H
#define ATG_ENGINE_SIM_POWERTRAIN_CONTROLLER_H

#include "powertrain_state.h"
#include "driver_inputs.h"
#include "actuator_commands.h"
#include "powertrain_bus.h"
#include "gearbox_capabilities.h"

#include <string>

namespace config {
    class ParameterRegistry;
    struct TelemetrySample;
}

namespace powertrain {

    class PowertrainController {
        public:
            PowertrainController() { /* void */ }
            virtual ~PowertrainController() { /* void */ }

            virtual void registerParameters(config::ParameterRegistry *registry, const char *prefix);
            virtual void fillTelemetry(config::TelemetrySample *sample) const;
            virtual void reset();
            virtual void configureGearbox(const GearboxCapabilities &capabilities);
            virtual const std::string &getRequestedMode() const;
            virtual const std::string &getPositionName() const;

            virtual void update(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs,
                ActuatorCommands *commands) = 0;

            inline const PowertrainBus &getBus() const { return m_bus; }

        protected:
            PowertrainBus m_bus;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_POWERTRAIN_CONTROLLER_H */

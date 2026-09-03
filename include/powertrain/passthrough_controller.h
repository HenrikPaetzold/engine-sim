#ifndef ATG_ENGINE_SIM_PASSTHROUGH_CONTROLLER_H
#define ATG_ENGINE_SIM_PASSTHROUGH_CONTROLLER_H

#include "powertrain_controller.h"

namespace powertrain {

    class PassthroughController : public PowertrainController {
        public:
            struct Parameters {
                double throttleGamma = 1.0;
            };

        public:
            PassthroughController();
            virtual ~PassthroughController();

            void initialize(const Parameters &params);

            virtual void registerParameters(config::ParameterRegistry *registry, const char *prefix);
            virtual void reset();

            virtual void update(
                double dt,
                const PowertrainState &state,
                const DriverInputs &inputs,
                ActuatorCommands *commands);

            static double plateFromPedal(double accelerator, double gamma);

        protected:
            Parameters m_params;
            bool m_previousShiftUp;
            bool m_previousShiftDown;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_PASSTHROUGH_CONTROLLER_H */
